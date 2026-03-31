import time
import math
import struct
import sys
import os
from pathlib import Path
import serial
import serial.tools.list_ports

# 复用现有的库
def _ensure_repo_root_on_path():
    for parent in Path(__file__).resolve().parents:
        if (parent / "MIGRATION_PLAN_SCHEME_B_EXECUTION.md").exists():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            break


_ensure_repo_root_on_path()

from common.config.robot_config import cfg
from common.motion.kinematics import LegKinematics, OFFSET_HIP, OFFSET_KNEE
from common.io.servo_safety import ServoSafetyGuard, build_servo_control_payload

# === 配置 ===
PORT = os.getenv("WAVEGO_PORT") or ("COM13" if os.name == "nt" else "/dev/ttyACM0")
BAUD_RATE = 115200
LEG_NAME = "FL"
LEG_ID_OFFSET = 0 # FL=0, FR=3, RL=6, RR=9
SERVO_IDS = [1, 2, 3] # FL 的舵机 ID

# 运动参数
STEP_HEIGHT = 40.0 # mm
FREQ = 0.5 # Hz
BODY_HEIGHT = -180.0 # mm

# 协议常量
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_SERVO_CTRL = 0x10
CMD_TYPE_TORQUE     = 0x11

class SingleLegTester:
    def __init__(self):
        self.ser = None
        self.running = True
        self.kinematics = LegKinematics(LEG_NAME)
        self.start_time = 0
        self.guard = ServoSafetyGuard(
            enforce_nonzero_timing=True,
            default_time_ms=30,
            default_speed=1200,
            max_step_delta=260,
        )
        
        self.connect()
        
    def connect(self):
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("连接失败: 未找到任何串口设备")
            self.running = False
            return

        devices = {p.device for p in ports}
        target_port = PORT if PORT in devices else None

        if target_port is None:
            def score(port_info):
                device = (port_info.device or "").lower()
                desc = (port_info.description or "").lower()
                val = 0
                if "wavego" in desc or "stm32" in desc:
                    val += 100
                if "ttyacm" in device:
                    val += 80
                if "ttyusb" in device:
                    val += 60
                if "usb" in desc:
                    val += 30
                if "com" in device:
                    val += 20
                return val

            target_port = sorted(ports, key=score, reverse=True)[0].device

        try:
            self.ser = serial.Serial(target_port, BAUD_RATE, timeout=0.01)
            print(f"已连接到 {target_port}")
        except Exception as e:
            print(f"连接失败: {e}")
            self.ser = None
            self.running = False

    def send_raw(self, pkt_type, payload):
        packet = bytearray([HEAD_1, HEAD_2, pkt_type, len(payload)])
        packet.extend(payload)
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        if self.ser: self.ser.write(packet)

    def send_torque(self, enable):
        print(f"发送扭矩指令: {'ON' if enable else 'OFF'}")
        self.send_raw(CMD_TYPE_TORQUE, bytearray([1 if enable else 0]))

    def send_pos(self, servo_data):
        # servo_data supports both legacy (id,pos,spd,acc) and new (id,pos,time,speed,acc)
        safe_cmds = self.guard.sanitize_batch(servo_data)
        payload = build_servo_control_payload(safe_cmds)
        self.send_raw(CMD_TYPE_SERVO_CTRL, payload)

    def run(self):
        if not self.running: return

        # 1. 初始状态：放松
        self.send_torque(False)
        print("\n=== 单腿踏步测试 (FL) ===")
        print("当前状态: RELAX (放松)")
        print("请手动调整机器狗姿态，确保左前腿悬空或可自由活动。")
        input("按 Enter 键开始上锁并运动 (Ctrl+C 停止)...")
        
        # 2. 上锁
        self.send_torque(True)
        time.sleep(0.5)
        
        # 3. 归中 (先走到初始位置)
        print("正在归位到初始高度...")
        self.move_to_z(0) # z_offset = 0
        time.sleep(1.0)
        
        print("开始踏步循环...")
        self.start_time = time.perf_counter()
        
        try:
            while self.running:
                t = time.perf_counter() - self.start_time
                
                # 计算相位 0~1
                period = 1.0 / FREQ
                phase = (t % period) / period
                
                # 简单的正弦波抬腿
                # 0.0~0.5: 抬起; 0.5~1.0: 放下(保持0)
                z_offset = 0.0
                if phase < 0.5:
                    # 归一化到 0~PI
                    local_p = phase / 0.5
                    z_offset = STEP_HEIGHT * math.sin(local_p * math.pi)
                else:
                    z_offset = 0.0
                
                # 执行
                self.move_to_z(z_offset)
                
                # 50Hz
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\n用户停止")
        finally:
            print("进入放松模式...")
            self.send_torque(False)
            if self.ser: self.ser.close()

    def move_to_z(self, z_offset):
        # 目标 Z 坐标 (相对于 Hip)
        # 注意: 这里的坐标系是 Leg Frame。
        # 默认站立时，脚在 Hip 正下方 (0, side_sign*L1, BODY_HEIGHT)
        # 但 kinematics_v5 的 solve_ik 接受的是 (x, y, z)
        # x=0, y=0 (相对于腿的根部?), z=BODY_HEIGHT
        
        # 检查 kinematics_v5 定义:
        # x: 前后, y: 左右(相对于腿根), z: 上下
        # y 需要考虑 L1 (Hip 长度)
        # 传入的 y 是 "足端相对于 Hip 旋转中心" 的距离?
        # 让我们看 kinematics_v5.py 的 solve_ik 注释或逻辑
        # "Target (x,y,z) in hip frame"
        # y should include L1?
        # 在 simulation_ik_v5 中: 
        # foot_world = [dx, dy+side_sign*L1, h]
        # hip_world = [dx, dy, 0]
        # foot_rel = foot_world - hip_world = [0, side_sign*L1, h]
        # 所以 y 应该是 side_sign * L1
        
        target_z = BODY_HEIGHT + z_offset
        target_y = self.kinematics.side_sign * cfg.L1 # 0 偏航时，脚在肩膀外侧 L1 处
        target_x = 0
        
        res = self.kinematics.solve_ik(target_x, target_y, target_z)
        
        if res:
            q1, q2, q3 = res
            # 转换为 PWM
            # 注意: kinematics_v5 里的 dir_abd 等已经在 rad_to_pwm 里处理了吗？
            # 没有，它只返回弧度。
            # 我们需要手动转 PWM，参考 robot_config.py 或 simulation_ik_v5.py
            
            # 复用 simulation 中的逻辑
            kin = self.kinematics
            pwm_abd = 2048 + (q1 - 0) * kin.rad_to_step * kin.dir_abd
            pwm_hip = 2048 + (q2 - OFFSET_HIP) * kin.rad_to_step * kin.dir_hip
            pwm_knee = 2048 + (q3 - OFFSET_KNEE) * kin.rad_to_step * kin.dir_knee
            
            # 组装指令
            # ID: FL=[1,2,3]
            data = [
                (1, pwm_abd, 0, 0), # 速度0表示最快(或由下位机插值)，建议给个速度?
                # V3 协议速度 0=Max? 不，stm32 代码里 0=Max。
                #为了柔顺，我们可以给个速度，比如 1000
                (2, pwm_hip, 0, 0),
                (3, pwm_knee, 0, 0)
            ]
            self.send_pos(data)

if __name__ == "__main__":
    app = SingleLegTester()
    app.run()
