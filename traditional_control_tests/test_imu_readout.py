"""
IMU 实时读数工具 — WT61C / JY901S 原始数据全屏显示。

用法:
    python test_imu_readout.py                # 自动连接
    python test_imu_readout.py /dev/ttyACM0   # 指定端口

操作:
    q / Esc   退出
    z         归零 Yaw 偏移
    r         重新扫描并连接
"""

import sys
import os
import time
import struct
import threading
import termios
import tty
import select

# ── 协议常量 (与 robot_io.py 一致) ──────────────────────────────────────────
HEAD_1 = 0xA5
HEAD_2 = 0x5A
FB_TYPE_SENSOR_IMU = 0x30
FB_TYPE_RL_STATE = 0x40


def _auto_find_port():
    """扫描串口，优先选含 USB / ACM 的设备。"""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    candidates = [p.device for p in ports if "USB" in p.description or "ACM" in p.device or "COM" in p.device]
    return candidates[-1] if candidates else ports[-1].device


class IMUReader:
    def __init__(self, port=None, baud=115200):
        self.ser = None
        self.port = port
        self.baud = baud
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        # 原始数据 (WT61C 帧)
        self.accel = [0.0, 0.0, 0.0]   # g
        self.gyro = [0.0, 0.0, 0.0]    # deg/s
        self.roll = 0.0                  # deg (WT61C 原始 Roll)
        self.pitch = 0.0                 # deg (WT61C 原始 Pitch)
        self.yaw = 0.0                   # deg
        self.yaw_offset = 0.0

        # 状态
        self.last_rx_time = 0.0
        self.rx_count = 0
        self.rx_rate = 0.0
        self._rx_count_window = 0
        self._rate_time = time.perf_counter()
        self.fresh = False
        self.connected = False

    def connect(self, port=None):
        import serial
        if port:
            self.port = port
        if not self.port:
            self.port = _auto_find_port()
        if not self.port:
            print("[ERROR] 未找到串口设备")
            return False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.running = True
            self.thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.thread.start()
            self.connected = True
            return True
        except Exception as e:
            print(f"[ERROR] 连接失败: {e}")
            self.connected = False
            return False

    def reconnect(self):
        self.close()
        time.sleep(0.5)
        self.yaw_offset = 0.0
        self.port = None
        return self.connect()

    def close(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.connected = False

    def zero_yaw(self):
        with self.lock:
            self.yaw_offset = self.yaw

    def _rx_loop(self):
        buf = bytearray()
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    buf.extend(self.ser.read(self.ser.in_waiting))

                while len(buf) >= 5:
                    if buf[0] != HEAD_1 or buf[1] != HEAD_2:
                        del buf[0]
                        continue

                    pkt_len = buf[3]
                    total = 4 + pkt_len + 1
                    if len(buf) < total:
                        break

                    pkt = buf[:total]
                    if (sum(pkt[:-1]) & 0xFF) == pkt[-1]:
                        pkt_type = pkt[2]
                        payload = pkt[4:4 + pkt_len]
                        if pkt_type in (FB_TYPE_SENSOR_IMU, FB_TYPE_RL_STATE):
                            self._parse_imu(payload)
                        del buf[:total]
                    else:
                        del buf[:2]

                time.sleep(0.001)
            except Exception:
                time.sleep(0.01)

    def _parse_imu(self, payload):
        if len(payload) < 36:
            return
        try:
            data = struct.unpack('<9f', payload[:36])
            alpha = 0.3  # 低通滤波系数
            with self.lock:
                for i in range(3):
                    self.accel[i] = self.accel[i] * (1 - alpha) + data[i] * alpha
                    self.gyro[i] = self.gyro[i] * (1 - alpha) + data[3 + i] * alpha
                self.roll = self.roll * (1 - alpha) + data[6] * alpha
                self.pitch = self.pitch * (1 - alpha) + data[7] * alpha
                self.yaw = self.yaw * (1 - alpha) + data[8] * alpha

                self.last_rx_time = time.perf_counter()
                self.rx_count += 1
                self._rx_count_window += 1
                self.fresh = True
        except Exception:
            pass

    def get_snapshot(self):
        """获取当前快照 (线程安全)。"""
        now = time.perf_counter()
        with self.lock:
            snap = {
                'accel': list(self.accel),
                'gyro': list(self.gyro),
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw - self.yaw_offset,
                'yaw_raw': self.yaw,
                'fresh': self.fresh and (now - self.last_rx_time < 0.5),
                'last_rx': now - self.last_rx_time,
                'total_rx': self.rx_count,
            }
        # 计算接收频率 (每秒更新一次)
        if now - self._rate_time >= 1.0:
            self.rx_rate = self._rx_count_window / (now - self._rate_time)
            self._rx_count_window = 0
            self._rate_time = now
        snap['rx_rate'] = self.rx_rate
        return snap


def _kbhit(timeout=0.05):
    """非阻塞按键检测。"""
    return select.select([sys.stdin], [], [], timeout)[0]


def _getch():
    ch = sys.stdin.read(1)
    return ch


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None

    reader = IMUReader(port=port)

    print("正在连接串口...")
    if not reader.connect():
        if not reader.reconnect():
            print("无法连接，退出。")
            sys.exit(1)

    port_name = reader.port or "?"
    print(f"已连接 {port_name}。按 q 退出，z 归零 Yaw，r 重连。")
    time.sleep(0.8)

    # 保存原始终端设置
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)

        while True:
            # 按键处理
            if _kbhit(0.02):
                ch = _getch()
                if ch in ('q', '\x03', '\x1b'):  # q / Ctrl+C / Esc
                    break
                elif ch == 'z':
                    reader.zero_yaw()
                elif ch == 'r':
                    reader.reconnect()
                    port_name = reader.port or "?"

            # 获取数据
            s = reader.get_snapshot()

            # ── 渲染 ──────────────────────────────────────────────────
            # ANSI: 清屏 + 光标归位
            sys.stdout.write("\033[2J\033[H")

            # 标题
            bar = "=" * 56
            sys.stdout.write(f"  {bar}\n")
            sys.stdout.write(f"  WT61C IMU 实时读数  |  {port_name}\n")
            sys.stdout.write(f"  {bar}\n\n")

            # 连接状态
            if s['fresh']:
                status = "\033[32m● 数据正常\033[0m"
            elif reader.last_rx_time > 0:
                status = f"\033[33m● 数据超时 ({s['last_rx']:.1f}s)\033[0m"
            else:
                status = "\033[31m● 无数据\033[0m"

            sys.stdout.write(f"  状态: {status}    接收频率: {s['rx_rate']:.0f} Hz    "
                             f"累计帧: {s['total_rx']}\n\n")

            # 欧拉角 (WT61C 原始输出)
            sys.stdout.write("  ┌─────────────────────────────────────────────┐\n")
            sys.stdout.write("  │  WT61C 原始欧拉角 (deg)                     │\n")
            sys.stdout.write("  ├─────────────────────────────────────────────┤\n")
            sys.stdout.write(f"  │  Roll  (angle[0]): {s['roll']:+8.2f}                  │\n")
            sys.stdout.write(f"  │  Pitch (angle[1]): {s['pitch']:+8.2f}                  │\n")
            sys.stdout.write(f"  │  Yaw   (angle[2]): {s['yaw_raw']:+8.2f}  (归零后: {s['yaw']:+8.2f}) │\n")
            sys.stdout.write("  └─────────────────────────────────────────────┘\n\n")

            # 重映射后 (与 obs_builder.py 一致)
            robot_roll_deg = -s['pitch']
            robot_pitch_deg = s['roll']
            sys.stdout.write("  ┌─────────────────────────────────────────────┐\n")
            sys.stdout.write("  │  机器人坐标系 (Y-forward 右手系)              │\n")
            sys.stdout.write("  │  IMU Roll  → Robot -Roll   (左右倾)         │\n")
            sys.stdout.write("  │  IMU Pitch → Robot Pitch  (前后倾)         │\n")
            sys.stdout.write("  ├─────────────────────────────────────────────┤\n")
            sys.stdout.write(f"  │  Robot Roll:  {robot_roll_deg:+8.2f}                       │\n")
            sys.stdout.write(f"  │  Robot Pitch: {robot_pitch_deg:+8.2f}                       │\n")
            sys.stdout.write(f"  │  Robot Yaw:   {s['yaw']:+8.2f}                       │\n")
            sys.stdout.write("  └─────────────────────────────────────────────┘\n\n")

            # 加速度
            sys.stdout.write("  ┌─────────────────────────────────────────────┐\n")
            sys.stdout.write("  │  加速度 (g)                                │\n")
            sys.stdout.write("  ├─────────────────────────────────────────────┤\n")
            sys.stdout.write(f"  │  Accel X: {s['accel'][0]:+8.3f}   Accel Y: {s['accel'][1]:+8.3f}   Accel Z: {s['accel'][2]:+8.3f} │\n")
            sys.stdout.write("  └─────────────────────────────────────────────┘\n\n")

            # 陀螺仪
            sys.stdout.write("  ┌─────────────────────────────────────────────┐\n")
            sys.stdout.write("  │  角速度 (deg/s)                            │\n")
            sys.stdout.write("  ├─────────────────────────────────────────────┤\n")
            sys.stdout.write(f"  │  Gyro X: {s['gyro'][0]:+8.2f}    Gyro Y: {s['gyro'][1]:+8.2f}    Gyro Z: {s['gyro'][2]:+8.2f}  │\n")
            sys.stdout.write("  └─────────────────────────────────────────────┘\n\n")

            # 操作提示
            sys.stdout.write("  ───────────────────────────────────────────\n")
            sys.stdout.write("  [q/Esc] 退出   [z] 归零 Yaw   [r] 重连串口\n")

            sys.stdout.flush()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        reader.close()
        print("\n已退出。")


if __name__ == "__main__":
    main()
