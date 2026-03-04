"""
STM32 USB CDC 通信桥接器 — Jetson Nano 侧驱动。

硬件架构:
    Jetson Nano
        ↕  USB CDC (/dev/ttyACM0, 115200 baud)
    STM32F407 (FreeRTOS, 纯透传)
        ↕  USART + DMA
    12× STS3215 串行总线舵机
        ↕  UART
    WT61C IMU

通信协议 (与 weixue-wheeled-dog 固件一致):
    包头:  0xA5 0x5A
    常见格式 A: HEAD(2) | TYPE(1) | LEN(1) | PAYLOAD(N) | CHECKSUM(1)
               CHECKSUM = sum(前面所有字节) & 0xFF
    常见格式 B: HEAD(2) | LEN(1)  | CMD(1) | PAYLOAD(N) | CHECKSUM(1)
               CHECKSUM = XOR(LEN..PAYLOAD)

说明:
    固件分支存在差异，桥接器默认按格式 A 发包，并在接收端自动兼容 A/B。

上位机 → 下位机 命令:
    0x01  CMD_HEARTBEAT      心跳包
    0x10  CMD_SERVO_CTRL     舵机目标位置控制
    0x20  CMD_TORQUE         扭矩使能/禁用
    0x30  CMD_REQUEST_STATE  请求全身状态

下位机 → 上位机 反馈:
    0x11  FB_SERVO_INFO      舵机状态 (pos, vel, load)
    0x12  FB_IMU             IMU 数据 (euler + gyro + accel)
    0x31  FB_FULL_STATE      全身状态包 (servo×12 + IMU)
"""

from __future__ import annotations

import struct
import threading
import time

import numpy as np

try:
    import serial
except ImportError:
    serial = None

# ── 协议常量 ─────────────────────────────────────────────────────────────────
HEAD_1 = 0xA5
HEAD_2 = 0x5A

CMD_HEARTBEAT     = 0x01
CMD_SERVO_CTRL    = 0x10
CMD_TORQUE        = 0x11
CMD_REQUEST_STATE = 0x30

FB_SERVO_INFO  = 0x20
FB_IMU         = 0x30
FB_FULL_STATE  = 0x40


class STM32Bridge:
    """
    STM32 USB CDC 通信桥接器。

    特性:
    - 后台接收线程：持续解析固件反馈，无需轮询
    - 线程安全：get_state() 加锁读取最新状态
    - 宽松兼容：允许仅串口或仅 IMU 包存在（固件差异容忍）

    用法:
        bridge = STM32Bridge(port="/dev/ttyACM0")
        bridge.connect()
        bridge.set_torque(True)
        bridge.send_servo_targets(targets_0_4095)
        state = bridge.get_state()
        bridge.disconnect()
    """

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        timeout: float = 0.01,
        tx_packet_format: str = "type_len_sum",
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.tx_packet_format = tx_packet_format
        self.ser: serial.Serial | None = None

        # ── 最新状态缓存 ────────────────────────────────────────────────────
        # 舵机：DFS 顺序 [FL_h, FL_t, FL_c, FR_h, FR_t, FR_c, RL_, RR_]
        self.servo_positions = np.zeros(12, dtype=np.float32)    # rad
        self.servo_velocities = np.zeros(12, dtype=np.float32)   # rad/s
        self.servo_loads = np.zeros(12, dtype=np.float32)        # 0-1 归一化

        # IMU (WT61C 欧拉角 + 陀螺仪 + 加速度)
        self.imu_roll  = 0.0   # deg
        self.imu_pitch = 0.0   # deg
        self.imu_yaw   = 0.0   # deg
        self.imu_gyro  = np.zeros(3, dtype=np.float32)   # deg/s
        self.imu_accel = np.zeros(3, dtype=np.float32)   # m/s²

        self.last_state_time = 0.0   # perf_counter 时间戳

        self._lock = threading.Lock()
        self._running = False
        self._recv_thread: threading.Thread | None = None

    # ── 连接管理 ─────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """打开串口并启动后台接收线程。"""
        if serial is None:
            print("[STM32] 缺少 pyserial，请先安装: pip install pyserial==3.5")
            return False
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(0.1)  # 等待 STM32 枚举稳定

            self._running = True
            self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True, name="stm32-rx")
            self._recv_thread.start()

            print(f"[STM32] 已连接: {self.port} @ {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"[STM32] 连接失败: {e}")
            print(f"  提示: ls /dev/ttyACM*  或  ls /dev/wavego_stm32")
            return False

    def disconnect(self):
        """停止后台线程，关闭串口。"""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[STM32] 已断开")

    # ── 发送接口 ─────────────────────────────────────────────────────────────

    def send_servo_targets(self, target_raw: np.ndarray, speed: int = 0, time_ms: int = 0):
        """
        发送 12 个舵机目标位置。

        Args:
            target_raw: 目标刻度 (0-4095), shape=(12,), DFS 顺序
            speed:      速度上限 (0=最快，RL 模式必须为 0)
            time_ms:    插值时间 ms (0=立即执行，RL 模式必须为 0)
        """
        if not self._is_open():
            return
        target_raw = np.clip(target_raw, 0, 4095).astype(np.int16)
        # payload: 12×int16 position + uint16 speed + uint16 time
        payload = struct.pack(f"<12hHH", *target_raw, speed, time_ms)
        self._send_packet(CMD_SERVO_CTRL, payload)

    def set_torque(self, enable: bool):
        """使能或禁用全部 12 个舵机扭矩。"""
        if not self._is_open():
            return
        payload = struct.pack("<B", 1 if enable else 0)
        self._send_packet(CMD_TORQUE, payload)
        print(f"[STM32] 扭矩: {'ON ✓' if enable else 'OFF'}")

    def request_state(self):
        """主动请求一次全身状态包（CMD 0x30）。"""
        if not self._is_open():
            return
        self._send_packet(CMD_REQUEST_STATE, b"")

    def send_heartbeat(self):
        """发送心跳包。"""
        if not self._is_open():
            return
        self._send_packet(CMD_HEARTBEAT, b"")

    # ── 状态读取 ─────────────────────────────────────────────────────────────

    def get_state(self) -> dict:
        """
        线程安全地读取最新全身状态。

        Returns:
            dict:
                servo_pos  (12,) float32  rad, DFS 顺序
                servo_vel  (12,) float32  rad/s, DFS 顺序
                imu_roll   float  deg
                imu_pitch  float  deg
                imu_yaw    float  deg
                imu_gyro   (3,) float32  deg/s
                imu_accel  (3,) float32  m/s²
                timestamp  float  perf_counter 时间戳
        """
        with self._lock:
            return {
                "servo_pos":  self.servo_positions.copy(),
                "servo_vel":  self.servo_velocities.copy(),
                "imu_roll":   self.imu_roll,
                "imu_pitch":  self.imu_pitch,
                "imu_yaw":    self.imu_yaw,
                "imu_gyro":   self.imu_gyro.copy(),
                "imu_accel":  self.imu_accel.copy(),
                "timestamp":  self.last_state_time,
            }

    def is_state_fresh(self, max_age_s: float = 0.1) -> bool:
        """检查上次收到状态包是否在 max_age_s 秒内。"""
        return (time.perf_counter() - self.last_state_time) < max_age_s

    # ── 内部：发包 ───────────────────────────────────────────────────────────

    @staticmethod
    def _checksum_xor(data: bytes) -> int:
        """XOR 校验。"""
        cs = 0
        for b in data:
            cs ^= b
        return cs & 0xFF

    @staticmethod
    def _checksum_sum(data: bytes) -> int:
        """SUM 校验：sum(data) & 0xFF。"""
        return sum(data) & 0xFF

    def _build_packet(self, cmd: int, payload: bytes) -> bytes:
        """构建完整数据包，默认使用 TYPE+LEN+SUM。"""
        if self.tx_packet_format == "len_cmd_xor":
            # 兼容旧格式: HEAD LEN CMD PAYLOAD CS(XOR from LEN)
            length = 1 + len(payload) + 1
            header = bytes([HEAD_1, HEAD_2, length, cmd])
            body = header[2:] + payload
            cs = self._checksum_xor(body)
            return header + payload + bytes([cs])

        # 推荐格式: HEAD TYPE LEN PAYLOAD CS(SUM all bytes before CS)
        header = bytes([HEAD_1, HEAD_2, cmd, len(payload)])
        cs = self._checksum_sum(header + payload)
        return header + payload + bytes([cs])

    def _send_packet(self, cmd: int, payload: bytes):
        """底层发包（带异常捕获）。"""
        try:
            pkt = self._build_packet(cmd, payload)
            self.ser.write(pkt)
        except serial.SerialException as e:
            print(f"[STM32] 发送异常: {e}")

    def _is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    # ── 内部：后台接收 ───────────────────────────────────────────────────────

    def _receive_loop(self):
        """后台线程：持续读取并解析 STM32 回传数据包。"""
        buf = bytearray()
        while self._running:
            try:
                # 批量读取可用字节
                waiting = self.ser.in_waiting
                if waiting > 0:
                    buf.extend(self.ser.read(waiting))
                else:
                    time.sleep(0.001)
                    continue

                # 解析 buf 中的所有完整包
                while True:
                    # 查找包头
                    idx = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == HEAD_1 and buf[i + 1] == HEAD_2:
                            idx = i
                            break
                    if idx < 0:
                        # 无包头，清空噪声字节
                        if len(buf) > 256:
                            buf.clear()
                        break
                    if idx > 0:
                        buf = buf[idx:]   # 跳过包头前的乱码

                    # 至少要有 HEAD(2)+TYPE/CMD(1)+LEN(1)+CS(1)
                    if len(buf) < 5:
                        break

                    parsed = False

                    # 格式 A: HEAD TYPE LEN PAYLOAD CS(sum)
                    type_a = buf[2]
                    len_a = buf[3]
                    total_a = 2 + 1 + 1 + len_a + 1
                    if len_a <= 240 and len(buf) >= total_a:
                        packet_a = bytes(buf[:total_a])
                        cs_a = self._checksum_sum(packet_a[:-1])
                        if packet_a[-1] == cs_a:
                            data_a = packet_a[4:-1]
                            self._parse(type_a, data_a)
                            buf = buf[total_a:]
                            parsed = True

                    if parsed:
                        continue

                    # 格式 B: HEAD LEN CMD PAYLOAD CS(xor)
                    len_b = buf[2]
                    total_b = 2 + 1 + len_b
                    if len_b >= 2 and len_b <= 250 and len(buf) >= total_b:
                        packet_b = bytes(buf[:total_b])
                        body_b = packet_b[2:-1]
                        cs_b = self._checksum_xor(body_b)
                        if packet_b[-1] == cs_b:
                            cmd_b = packet_b[3]
                            data_b = packet_b[4:-1]
                            self._parse(cmd_b, data_b)
                            buf = buf[total_b:]
                            parsed = True

                    if parsed:
                        continue

                    # 当前包不完整或格式不匹配，丢弃一个字节继续同步
                    buf = buf[1:]

            except serial.SerialException:
                time.sleep(0.01)
            except Exception as e:
                print(f"[STM32] 接收异常: {type(e).__name__}: {e}")
                time.sleep(0.01)

    def _parse(self, cmd: int, data: bytes):
        """分发解析具体反馈包。"""
        with self._lock:
            if cmd == FB_FULL_STATE:
                self._parse_full_state(data)
            elif cmd == FB_SERVO_INFO:
                self._parse_servo_info(data)
            elif cmd == FB_IMU:
                self._parse_imu(data)

    def _parse_full_state(self, data: bytes):
        """
        全身状态包 (0x40，按当前 RobotIO 分支)：

        [0:36]  9×float32: [accel(3), gyro(3), euler(3)]
        [36]    uint8: servo_count
        [37:]   servo_count×11 bytes: <B h h h h B B>
                id, pos_raw, spd_raw, load_raw, current_raw, volt_raw, temp_raw

        兼容回退：若 payload 长度为 132 字节，也支持旧版 33×float32 格式。
        """
        # 兼容旧版 33f
        FULL_STATE_FMT = "<33f"
        size = struct.calcsize(FULL_STATE_FMT)
        if len(data) >= size:
            vals = struct.unpack_from(FULL_STATE_FMT, data)
            self.servo_positions[:]  = vals[0:12]
            self.servo_velocities[:] = vals[12:24]
            self.imu_roll   = vals[24]
            self.imu_pitch  = vals[25]
            self.imu_yaw    = vals[26]
            self.imu_gyro[:]  = vals[27:30]
            self.imu_accel[:] = vals[30:33]
            self.last_state_time = time.perf_counter()
            return

        # 当前分支格式: 9f + count + per-servo entries
        if len(data) < 37:
            self._parse_servo_info(data)
            return

        imu_vals = struct.unpack_from("<9f", data, 0)
        self.imu_accel[:] = imu_vals[0:3]
        self.imu_gyro[:] = imu_vals[3:6]
        self.imu_roll, self.imu_pitch, self.imu_yaw = imu_vals[6:9]

        count = int(data[36])
        servo_entry_size = 11
        tail = data[37:]
        max_count = min(count, 12, len(tail) // servo_entry_size)
        for i in range(max_count):
            off = i * servo_entry_size
            sid, pos_raw, spd_raw, load_raw, _curr_raw, _volt_raw, _temp_raw = struct.unpack_from(
                "<BhhhhBB", tail, off
            )
            idx = int(sid) - 1
            if 0 <= idx < 12:
                self.servo_positions[idx] = (pos_raw - 2048) * (2 * np.pi / 4096)
                self.servo_velocities[idx] = spd_raw * (2 * np.pi / 4096)
                self.servo_loads[idx] = load_raw / 1000.0

        self.last_state_time = time.perf_counter()

    def _parse_servo_info(self, data: bytes):
        """
        舵机状态包 (0x11)。

        格式 (12 × 6 bytes = 72 bytes):
            每个舵机: int16 pos_raw(0-4095) + int16 vel_raw + int16 load_raw
        """
        if len(data) < 72:
            return
        for i in range(12):
            off = i * 6
            pos_raw, vel_raw, load_raw = struct.unpack_from("<3h", data, off)
            self.servo_positions[i] = (pos_raw - 2048) * (2 * np.pi / 4096)
            self.servo_velocities[i] = vel_raw * (2 * np.pi / 4096)   # 转换系数需按固件确认
            self.servo_loads[i] = load_raw / 1000.0
        self.last_state_time = time.perf_counter()

    def _parse_imu(self, data: bytes):
        """
        IMU 数据包 (0x12)。

        格式 (9 × float32 = 36 bytes):
            3 × float32  euler_deg  [roll, pitch, yaw]
            3 × float32  gyro_degs  [x, y, z]
            3 × float32  accel_ms2  [x, y, z]
        """
        FMT = "<9f"
        if len(data) < struct.calcsize(FMT):
            return
        vals = struct.unpack_from(FMT, data)
        self.imu_roll, self.imu_pitch, self.imu_yaw = vals[0:3]
        self.imu_gyro[:]  = vals[3:6]
        self.imu_accel[:] = vals[6:9]
        self.last_state_time = time.perf_counter()


# ── 快速自测 ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="STM32Bridge 快速自测")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument(
        "--tx-format",
        default="type_len_sum",
        choices=["type_len_sum", "len_cmd_xor"],
        help="发包格式，默认 type_len_sum",
    )
    args = parser.parse_args()

    bridge = STM32Bridge(port=args.port, tx_packet_format=args.tx_format)
    if not bridge.connect():
        raise SystemExit(1)

    bridge.set_torque(True)
    time.sleep(0.3)

    print(f"持续读取 {args.duration}s 状态...")
    t_end = time.perf_counter() + args.duration
    while time.perf_counter() < t_end:
        bridge.request_state()
        time.sleep(0.02)
        s = bridge.get_state()
        print(
            f"  Roll={s['imu_roll']:+6.1f}° Pitch={s['imu_pitch']:+6.1f}°  "
            f"Pos[0]={s['servo_pos'][0]:+.3f} rad  "
            f"Latency={(time.perf_counter()-s['timestamp'])*1e3:.0f}ms"
            if s["timestamp"] > 0 else "  (等待首包...)"
        )

    bridge.set_torque(False)
    bridge.disconnect()
