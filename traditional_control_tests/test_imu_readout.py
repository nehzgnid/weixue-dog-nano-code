"""
IMU 实时读数工具 — WT61C / JY901S 原始数据全屏显示。

用法:
    python test_imu_readout.py                # 自动连接
    python test_imu_readout.py /dev/ttyACM0   # 指定端口
    python test_imu_readout.py --plain        # 兼容模式（不清屏）

操作:
    q / Esc   退出
    z         归零 Yaw 偏移
    x         清零线速度/位移与偏置估计
    r         重新扫描并连接
"""

import sys
import time
import struct
import threading
import math
import shutil
import re
import unicodedata
import termios
import tty
import select

# ── 协议常量 (与 robot_io.py 一致) ──────────────────────────────────────────
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_REQUEST_STATE = 0x30  # 与 IMU 反馈共用数值：下发时表示“请求状态”
FB_TYPE_SENSOR_IMU = 0x30
FB_TYPE_RL_STATE = 0x40

DEG2RAD = math.pi / 180.0
G_M_S2 = 9.81

# WT61C 原始坐标系 -> 训练观测坐标系 的默认映射
# 当前设置: Y 前向 + 右手系测试候选
#   remap=[0,1,2], sign=[+1,+1,+1]
#   euler: obs_roll=raw_roll, obs_pitch=raw_pitch
IMU_AXIS_REMAP = (0, 1, 2)
IMU_AXIS_SIGN = (1.0, 1.0, 1.0)


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
        self.yaw_zeroed = False

        # 与 ObsBuilder 一致的“转化后观测量”
        self.obs_accel_g = [0.0, 0.0, 0.0]       # g
        self.obs_gyro_deg_s = [0.0, 0.0, 0.0]    # deg/s
        self.obs_roll = 0.0                        # deg
        self.obs_pitch = 0.0                       # deg
        self.obs_projected_gravity = [0.0, 0.0, -1.0]  # g 单位向量

        # 线加速度/线速度/线位移估计（OBS 坐标系）
        self.lin_acc_raw_g = [0.0, 0.0, 0.0]      # 去重力前修正量(g)
        self.lin_acc_corr_g = [0.0, 0.0, 0.0]     # 去偏置后线加速度(g)
        self.lin_acc_filt_g = [0.0, 0.0, 0.0]     # 滤波后线加速度(g)
        self.lin_vel_m_s = [0.0, 0.0, 0.0]        # m/s
        self.lin_pos_m = [0.0, 0.0, 0.0]          # m

        # 漂移抑制状态
        self.lin_acc_bias_g = [0.0, 0.0, 0.0]     # 静止段自校准偏置(g)
        self.stationary = False
        self._stationary_count = 0
        self._last_kin_time = None

        # 状态
        self.last_rx_time = 0.0
        self.rx_count = 0
        self.rx_rate = 0.0
        self._rx_count_window = 0
        self._rate_time = time.perf_counter()
        self.fresh = False
        self.connected = False
        self.last_request_time = 0.0
        self.raw_rx_bytes = 0
        self.raw_rx_chunks = 0
        self._imu_initialized = False

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
        self.yaw_zeroed = False
        self._imu_initialized = False
        self.reset_motion_estimation()
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
            self.yaw_zeroed = True

    def reset_motion_estimation(self):
        with self.lock:
            self.lin_acc_raw_g = [0.0, 0.0, 0.0]
            self.lin_acc_corr_g = [0.0, 0.0, 0.0]
            self.lin_acc_filt_g = [0.0, 0.0, 0.0]
            self.lin_vel_m_s = [0.0, 0.0, 0.0]
            self.lin_pos_m = [0.0, 0.0, 0.0]
            self.lin_acc_bias_g = [0.0, 0.0, 0.0]
            self.stationary = False
            self._stationary_count = 0
            self._last_kin_time = None

    @staticmethod
    def _projected_gravity(obs_roll_deg, obs_pitch_deg):
        """返回重力在观测坐标系中的方向向量（单位 g）。"""
        rr = math.radians(obs_roll_deg)
        rp = math.radians(obs_pitch_deg)
        cr, sr = math.cos(rr), math.sin(rr)
        cp, sp = math.cos(rp), math.sin(rp)
        gx = -sp
        gy = cp * sr
        gz = -cp * cr
        return [gx, gy, gz]

    @staticmethod
    def _remap_to_obs(raw_xyz):
        return [
            raw_xyz[IMU_AXIS_REMAP[0]] * IMU_AXIS_SIGN[0],
            raw_xyz[IMU_AXIS_REMAP[1]] * IMU_AXIS_SIGN[1],
            raw_xyz[IMU_AXIS_REMAP[2]] * IMU_AXIS_SIGN[2],
        ]

    def _send_packet(self, pkt_type, payload=b""):
        if not self.ser:
            return
        packet = bytearray([HEAD_1, HEAD_2, pkt_type, len(payload)])
        packet.extend(payload)
        packet.append(sum(packet) & 0xFF)
        self.ser.write(packet)

    def request_state(self):
        """主动请求一次状态包，适配需要轮询的 STM32 固件。"""
        self._send_packet(CMD_REQUEST_STATE, b"")
        self.last_request_time = time.perf_counter()

    def _rx_loop(self):
        buf = bytearray()
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting)
                    if chunk:
                        with self.lock:
                            self.raw_rx_bytes += len(chunk)
                            self.raw_rx_chunks += 1
                        buf.extend(chunk)

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
                if not self._imu_initialized:
                    # 首帧直接初始化，避免从 0 启动滤波导致虚假加速度积分。
                    self.accel = [data[0], data[1], data[2]]
                    self.gyro = [data[3], data[4], data[5]]
                    self.roll = data[6]
                    self.pitch = data[7]
                    self.yaw = data[8]
                    self._imu_initialized = True
                else:
                    for i in range(3):
                        self.accel[i] = self.accel[i] * (1 - alpha) + data[i] * alpha
                        self.gyro[i] = self.gyro[i] * (1 - alpha) + data[3 + i] * alpha
                    self.roll = self.roll * (1 - alpha) + data[6] * alpha
                    self.pitch = self.pitch * (1 - alpha) + data[7] * alpha
                    self.yaw = self.yaw * (1 - alpha) + data[8] * alpha

                now = time.perf_counter()
                self.last_rx_time = now
                self.rx_count += 1
                self._rx_count_window += 1
                self.fresh = True

                # 当前测试候选的观测转化链路
                # 1) 欧拉角: obs_roll=imu_roll, obs_pitch=imu_pitch
                # 2) gyro/acc: 先按 imu_axis_remap 重排, 再乘 imu_axis_sign
                self.obs_roll = self.roll
                self.obs_pitch = self.pitch
                self.obs_accel_g = self._remap_to_obs(self.accel)
                self.obs_gyro_deg_s = self._remap_to_obs(self.gyro)
                self.obs_projected_gravity = self._projected_gravity(self.obs_roll, self.obs_pitch)

                # 线加速度估计：a_lin_g = accel_obs_g + projected_gravity
                # 说明：静止时 accel_obs_g≈-projected_gravity，故相加后应接近 0。
                for i in range(3):
                    self.lin_acc_raw_g[i] = self.obs_accel_g[i] + self.obs_projected_gravity[i]

                # 静止检测：低角速度 + 加速度模长接近 1g
                gyro_norm = math.sqrt(
                    self.obs_gyro_deg_s[0] * self.obs_gyro_deg_s[0]
                    + self.obs_gyro_deg_s[1] * self.obs_gyro_deg_s[1]
                    + self.obs_gyro_deg_s[2] * self.obs_gyro_deg_s[2]
                )
                acc_norm = math.sqrt(
                    self.obs_accel_g[0] * self.obs_accel_g[0]
                    + self.obs_accel_g[1] * self.obs_accel_g[1]
                    + self.obs_accel_g[2] * self.obs_accel_g[2]
                )
                self.stationary = (gyro_norm < 2.0) and (abs(acc_norm - 1.0) < 0.06)
                if self.stationary:
                    self._stationary_count += 1
                else:
                    self._stationary_count = 0

                # 偏置自校准：仅在静止时更新，加速收敛与抗漂移
                if self.stationary:
                    beta = 0.08 if self._stationary_count < 25 else 0.02
                    for i in range(3):
                        self.lin_acc_bias_g[i] = (1 - beta) * self.lin_acc_bias_g[i] + beta * self.lin_acc_raw_g[i]

                # 去偏置 + 一阶低通
                alpha = 0.30
                deadband_g = 0.02
                lin_acc_m_s2 = [0.0, 0.0, 0.0]
                for i in range(3):
                    self.lin_acc_corr_g[i] = self.lin_acc_raw_g[i] - self.lin_acc_bias_g[i]
                    self.lin_acc_filt_g[i] = (1 - alpha) * self.lin_acc_filt_g[i] + alpha * self.lin_acc_corr_g[i]
                    if abs(self.lin_acc_filt_g[i]) < deadband_g:
                        self.lin_acc_filt_g[i] = 0.0
                    lin_acc_m_s2[i] = self.lin_acc_filt_g[i] * G_M_S2

                # 积分 + 漂移抑制（ZUPT + 速度泄漏 + 位置微泄漏）
                if self._last_kin_time is not None:
                    dt = max(1e-3, min(0.08, now - self._last_kin_time))
                    vel_decay = math.exp(-0.25 * dt)
                    pos_decay = math.exp(-0.06 * dt)
                    for i in range(3):
                        self.lin_vel_m_s[i] = self.lin_vel_m_s[i] * vel_decay + lin_acc_m_s2[i] * dt
                        if self.stationary:
                            self.lin_vel_m_s[i] *= 0.2
                        if abs(self.lin_vel_m_s[i]) < 0.01:
                            self.lin_vel_m_s[i] = 0.0
                        self.lin_pos_m[i] = self.lin_pos_m[i] * pos_decay + self.lin_vel_m_s[i] * dt
                self._last_kin_time = now
        except Exception:
            pass

    def get_snapshot(self):
        """获取当前快照 (线程安全)。"""
        now = time.perf_counter()
        with self.lock:
            snap = {
                'accel_raw': list(self.accel),
                'gyro_raw': list(self.gyro),
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw - self.yaw_offset,
                'yaw_raw': self.yaw,
                'yaw_offset': self.yaw_offset,
                'yaw_zeroed': self.yaw_zeroed,
                'obs_roll': self.obs_roll,
                'obs_pitch': self.obs_pitch,
                'obs_yaw': self.yaw - self.yaw_offset,
                'obs_accel_g': list(self.obs_accel_g),
                'obs_gyro_deg_s': list(self.obs_gyro_deg_s),
                'obs_gyro_rad_s': [v * DEG2RAD for v in self.obs_gyro_deg_s],
                'obs_projected_gravity': list(self.obs_projected_gravity),
                'lin_acc_raw_g': list(self.lin_acc_raw_g),
                'lin_acc_corr_g': list(self.lin_acc_corr_g),
                'lin_acc_filt_g': list(self.lin_acc_filt_g),
                'lin_vel_m_s': list(self.lin_vel_m_s),
                'lin_pos_m': list(self.lin_pos_m),
                'lin_acc_bias_g': list(self.lin_acc_bias_g),
                'stationary': self.stationary,
                'fresh': self.fresh and (now - self.last_rx_time < 0.5),
                'last_rx': now - self.last_rx_time,
                'total_rx': self.rx_count,
                'raw_rx_bytes': self.raw_rx_bytes,
                'raw_rx_chunks': self.raw_rx_chunks,
            }
            # 兼容旧显示字段名
            snap['accel'] = snap['accel_raw']
            snap['gyro'] = snap['gyro_raw']
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


def _parse_args(argv):
    """解析参数：支持可选端口和 --plain 兼容模式。"""
    port = None
    plain_mode = False
    for arg in argv[1:]:
        if arg in ("-h", "--help"):
            print("用法:")
            print("  python test_imu_readout.py [PORT] [--plain]")
            print("示例:")
            print("  python test_imu_readout.py")
            print("  python test_imu_readout.py /dev/ttyACM0")
            print("  python test_imu_readout.py --plain")
            sys.exit(0)
        if arg == "--plain":
            plain_mode = True
            continue
        if arg.startswith("-"):
            print(f"[WARN] 未识别参数: {arg}")
            continue
        if port is None:
            port = arg
    return port, plain_mode


def _run_plain_mode(reader):
    """纯文本兼容模式：不使用清屏与 raw 输入，适合兼容性较差终端。"""
    print("已进入 --plain 兼容模式（Ctrl+C 退出）。")
    print("字段说明:")
    print("  RAW: 传感器直接输出 (acc[g], gyro[deg/s], rpy[deg])")
    print("  OBS: 按策略输入顺序展示")
    print("       obs[0:3]=base_lin_vel_est(m/s), obs[3:6]=base_ang_vel(rad/s), obs[6:9]=projected_gravity(g)")
    print("  对照: 每条 OBS 旁同时显示 RAW 与映射来源")
    print("  当前测试映射: remap=[0,1,2], sign=[+1,+1,+1], euler=(roll,pitch)=(raw_roll,raw_pitch)")
    last_print = 0.0
    try:
        while True:
            now = time.perf_counter()
            if now - reader.last_request_time >= 0.05:
                reader.request_state()

            s = reader.get_snapshot()
            if now - last_print >= 0.2:
                status = "OK" if s['fresh'] else "TIMEOUT"
                print(
                    f"[{time.strftime('%H:%M:%S')}] {status} "
                    f"OBS[0:3]v=({s['lin_vel_m_s'][0]:+5.2f},{s['lin_vel_m_s'][1]:+5.2f},{s['lin_vel_m_s'][2]:+5.2f}) "
                    f"Pos=({s['lin_pos_m'][0]:+5.2f},{s['lin_pos_m'][1]:+5.2f},{s['lin_pos_m'][2]:+5.2f}) "
                    f"OBS[3:6]w=({s['obs_gyro_rad_s'][0]:+5.2f},{s['obs_gyro_rad_s'][1]:+5.2f},{s['obs_gyro_rad_s'][2]:+5.2f}) "
                    f"OBS[6:9]g=({s['obs_projected_gravity'][0]:+5.2f},{s['obs_projected_gravity'][1]:+5.2f},{s['obs_projected_gravity'][2]:+5.2f}) "
                    f"| RAW acc=({s['accel_raw'][0]:+5.3f},{s['accel_raw'][1]:+5.3f},{s['accel_raw'][2]:+5.3f}) "
                    f"gyro=({s['gyro_raw'][0]:+5.2f},{s['gyro_raw'][1]:+5.2f},{s['gyro_raw'][2]:+5.2f}) "
                    f"rpy=({s['roll']:+5.2f},{s['pitch']:+5.2f},{s['yaw_raw']:+5.2f}) "
                    f"STAT={'Y' if s['stationary'] else 'N'} "
                    f"Rate={s['rx_rate']:.0f}Hz"
                )
                last_print = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


def main():
    port, plain_mode = _parse_args(sys.argv)

    reader = IMUReader(port=port)

    print("正在连接串口...")
    if not reader.connect():
        if not reader.reconnect():
            print("无法连接，退出。")
            sys.exit(1)

    port_name = reader.port or "?"
    print(f"已连接 {port_name}。按 q 退出，z 归零 Yaw，x 清零运动估计，r 重连。")
    time.sleep(0.8)

    if plain_mode:
        _run_plain_mode(reader)
        reader.close()
        print("\n已退出。")
        return

    # 保存原始终端设置
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)

        ansi_re = re.compile(r"\x1b\[[0-9;]*m")

        def visible_width(text):
            clean = ansi_re.sub("", text)
            width = 0
            for ch in clean:
                width += 2 if unicodedata.east_asian_width(ch) in ("F", "W") else 1
            return width

        def write_line(text=""):
            term_width = shutil.get_terminal_size(fallback=(100, 30)).columns
            clean_width = visible_width(text)
            if clean_width > term_width:
                # 保留 ANSI 转义码，只按可见宽度截断内容。
                result = []
                width = 0
                i = 0
                while i < len(text) and width < term_width - 1:
                    if text[i] == "\x1b":
                        m = ansi_re.match(text, i)
                        if m:
                            result.append(m.group(0))
                            i = m.end()
                            continue
                    ch = text[i]
                    ch_width = 2 if unicodedata.east_asian_width(ch) in ("F", "W") else 1
                    if width + ch_width > term_width - 1:
                        break
                    result.append(ch)
                    width += ch_width
                    i += 1
                text = "".join(result) + "…"
            # raw 模式下 '\n' 不会自动回到行首，必须显式输出 '\r\n'。
            sys.stdout.write("\r")
            sys.stdout.write(text)
            sys.stdout.write("\033[K\r\n")

        while True:
            # 按键处理
            if _kbhit(0.02):
                ch = _getch()
                if ch in ('q', '\x03', '\x1b'):  # q / Ctrl+C / Esc
                    break
                elif ch == 'z':
                    reader.zero_yaw()
                elif ch == 'x':
                    reader.reset_motion_estimation()
                elif ch == 'r':
                    reader.reconnect()
                    port_name = reader.port or "?"

            # 主动请求状态，避免只依赖 STM32 的主动推送
            now = time.perf_counter()
            if now - reader.last_request_time >= 0.05:
                reader.request_state()

            # 获取数据
            s = reader.get_snapshot()

            # ── 渲染 ──────────────────────────────────────────────────
            # ANSI: 清屏 + 光标归位
            sys.stdout.write("\033[2J\033[H")

            # 标题
            bar = "=" * min(40, max(10, shutil.get_terminal_size(fallback=(100, 30)).columns - 2))
            write_line(f"  {bar}")
            write_line(f"  WT61C IMU 实时读数 | {port_name}")
            write_line(f"  {bar}")
            write_line()

            # 连接状态
            if s['fresh']:
                status = "\033[32m● 数据正常\033[0m"
            elif reader.last_rx_time > 0:
                status = f"\033[33m● 数据超时 ({s['last_rx']:.1f}s)\033[0m"
            else:
                status = "\033[31m● 无数据\033[0m"

            write_line(f"  状态: {status} | 帧: {s['total_rx']} | 原始字节: {s['raw_rx_bytes']} | 频率: {s['rx_rate']:.0f} Hz")
            write_line()

            write_line("  展示方式: 按 obs 输入顺序 + RAW/映射对照")
            write_line("  当前测试映射: remap=[0,1,2], sign=[+1,+1,+1], euler=(raw_roll, raw_pitch)")
            write_line("  LIN算法: 静止检测 + 偏置自校准 + 低通 + ZUPT + 泄漏抑漂")
            write_line()

            yaw_note = "(已按 z 键零偏)" if s['yaw_zeroed'] else "(未零偏)"
            write_line(f"  RAW RPY [deg] -> OBS RPY [deg] : ({s['roll']:+6.2f}, {s['pitch']:+6.2f}, {s['yaw_raw']:+6.2f}) -> ({s['obs_roll']:+6.2f}, {s['obs_pitch']:+6.2f}, {s['obs_yaw']:+6.2f}) {yaw_note}")
            write_line()

            # obs[0:3] base_lin_vel
            write_line(f"  OBS[0:3] base_lin_vel_est [m/s] : X={s['lin_vel_m_s'][0]:+7.3f}  Y={s['lin_vel_m_s'][1]:+7.3f}  Z={s['lin_vel_m_s'][2]:+7.3f}")
            write_line(f"     对照: RAW acc[g]=({s['accel_raw'][0]:+6.3f},{s['accel_raw'][1]:+6.3f},{s['accel_raw'][2]:+6.3f})  ->  OBS acc[g]=({s['obs_accel_g'][0]:+6.3f},{s['obs_accel_g'][1]:+6.3f},{s['obs_accel_g'][2]:+6.3f})")
            write_line(f"     对照: LinAccFilt[m/s^2]=({s['lin_acc_filt_g'][0]*G_M_S2:+6.3f},{s['lin_acc_filt_g'][1]*G_M_S2:+6.3f},{s['lin_acc_filt_g'][2]*G_M_S2:+6.3f})  Pos[m]=({s['lin_pos_m'][0]:+6.3f},{s['lin_pos_m'][1]:+6.3f},{s['lin_pos_m'][2]:+6.3f})")
            write_line()

            # obs[3:6] base_ang_vel
            write_line(f"  OBS[3:6] base_ang_vel [rad/s]   : X={s['obs_gyro_rad_s'][0]:+7.3f}  Y={s['obs_gyro_rad_s'][1]:+7.3f}  Z={s['obs_gyro_rad_s'][2]:+7.3f}")
            write_line(f"     对照: RAW gyro[deg/s]=({s['gyro_raw'][0]:+6.2f},{s['gyro_raw'][1]:+6.2f},{s['gyro_raw'][2]:+6.2f})")
            write_line()

            # obs[6:9] projected_gravity
            write_line(f"  OBS[6:9] projected_gravity [g]  : X={s['obs_projected_gravity'][0]:+7.3f}  Y={s['obs_projected_gravity'][1]:+7.3f}  Z={s['obs_projected_gravity'][2]:+7.3f}")
            write_line(f"     对照: Bias[g]=({s['lin_acc_bias_g'][0]:+6.3f},{s['lin_acc_bias_g'][1]:+6.3f},{s['lin_acc_bias_g'][2]:+6.3f})  静止判定={'是' if s['stationary'] else '否'}")
            write_line()

            # 操作提示
            write_line("  [q/Esc] 退出   [z] Yaw零偏(仅显示)   [x] 清零运动估计   [r] 重连串口")

            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        reader.close()
        print("\n已退出。")


if __name__ == "__main__":
    main()
