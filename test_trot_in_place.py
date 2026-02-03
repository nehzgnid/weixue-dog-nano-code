import time
import math
import struct
import threading
import serial
import serial.tools.list_ports
import numpy as np

from robot_config import cfg
from kinematics_v5 import LegKinematics, OFFSET_HIP, OFFSET_KNEE, L1

# === 配置 ===
BAUD_RATE = 115200
BODY_HEIGHT = -180.0 # 标准站立高度
STEP_HEIGHT_TARGET = 40.0 # 目标抬腿高度 (mm)
FREQ = 2.0 # 步频 (Hz)
SOFT_START_DURATION = 3.0 # 缓启动时间 (秒)

# 重心微调 (COG Shift)
# 正值表示将"脚"相对于身体向后/右移 (即身体向前/左移) 
# 修正: 上述注释可能不准确。坐标系 X前 Y左。
# Target X > 0 => 脚在前 => 身体重心相对靠后 (抵抗前倾)。
# Target Y < 0 => 脚在右 => 身体重心相对靠左 (抵抗右倾)。
# 依然前移 -> 需要更多 X (30.0)。顺时针转 -> 可能是 Y 偏移导致左右腿不对称，减小 Y (-5.0)。
COG_X_SHIFT = 40.0 
COG_Y_SHIFT = 5.0

# 协议常量
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_SERVO_CTRL = 0x10
CMD_TYPE_TORQUE     = 0x11

class TrotTester:
    def __init__(self):
        self.ser = None
        self.running = True
        self.legs = {
            "FL": LegKinematics("FL"),
            "FR": LegKinematics("FR"),
            "RL": LegKinematics("RL"),
            "RR": LegKinematics("RR")
        }
        # 舵机 ID 映射
        self.leg_ids = {
            "FL": [1, 2, 3],
            "FR": [4, 5, 6],
            "RL": [7, 8, 9],
            "RR": [10, 11, 12]
        }
        
        # Trot 相位偏移
        self.phase_offsets = {
            "FL": 0.0, "RR": 0.0,
            "FR": 0.5, "RL": 0.5
        }
        
        self.connect()
        
    def connect(self):
        ports = list(serial.tools.list_ports.comports())
        target_port = "COM13" # 请根据实际串口修改
        
        
        if target_port:
            try:
                self.ser = serial.Serial(target_port, BAUD_RATE, timeout=0.01)
                print(f"已连接到 {target_port}")
            except Exception as e:
                print(f"连接失败: {e}")
                self.running = False
        else:
            print("未找到串口")
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

    def send_all_pos(self, servo_data_list):
        # servo_data_list: [(id, pos, spd, acc), ...]
        count = len(servo_data_list)
        payload = bytearray([count])
        for item in servo_data_list:
            sid, pos, spd, acc = item
            payload.extend(struct.pack('<B h h B', sid, int(pos), int(spd), int(acc)))
        self.send_raw(CMD_TYPE_SERVO_CTRL, payload)

    def run(self):
        if not self.running: return

        # 1. 放松，等待用户确认
        self.send_torque(False)
        print("\n=== 四足原地踏步测试 (Trot) ===")
        print("警告: 请务必将机器狗架空！")
        input("按 Enter 键开始 (Ctrl+C 停止)...")
        
        # 2. 上锁并归位
        self.send_torque(True)
        print("正在归位到标准站立姿态...")
        # 发送一次全归位指令
        init_cmds = []
        for name in self.legs:
            pwm_abd, pwm_hip, pwm_knee = self.calc_pwm(name, 0)
            ids = self.leg_ids[name]
            init_cmds.append((ids[0], pwm_abd, 1000, 50))
            init_cmds.append((ids[1], pwm_hip, 1000, 50))
            init_cmds.append((ids[2], pwm_knee, 1000, 50))
        self.send_all_pos(init_cmds)
        time.sleep(2.0) # 等待归位
        
        print("开始踏步...")
        start_time = time.perf_counter()
        
        try:
            while self.running:
                now = time.perf_counter()
                elapsed = now - start_time
                
                # 缓启动逻辑: 步高从 0 慢慢增加到 40
                current_step_height = min(STEP_HEIGHT_TARGET, STEP_HEIGHT_TARGET * (elapsed / SOFT_START_DURATION))
                
                # 计算全局相位
                period = 1.0 / FREQ
                global_phase = (now % period) / period
                
                all_servos = []
                
                for name in self.legs:
                    # 计算该腿相位
                    leg_phase = (global_phase + self.phase_offsets[name]) % 1.0
                    
                    z_offset = 0.0
                    # Swing Phase (0~0.5) 抬腿
                    if leg_phase < 0.5:
                        local_p = leg_phase / 0.5
                        z_offset = current_step_height * math.sin(local_p * math.pi)
                    
                    # 计算 PWM
                    p1, p2, p3 = self.calc_pwm(name, z_offset)
                    ids = self.leg_ids[name]
                    
                    # 速度设置为 0 (最快)，因为我们是 50Hz 连续控制
                    # 加速度设为 0 (最大)
                    all_servos.append((ids[0], p1, 0, 0))
                    all_servos.append((ids[1], p2, 0, 0))
                    all_servos.append((ids[2], p3, 0, 0))
                    
                self.send_all_pos(all_servos)
                
                # 50Hz Loop
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\n停止")
        finally:
            print("放松...")
            self.send_torque(False)
            if self.ser: self.ser.close()

    def calc_pwm(self, leg_name, z_offset):
        kin = self.legs[leg_name]
        
        target_x = 0 + COG_X_SHIFT
        # y 坐标需要包含 L1 偏移 (脚在肩外侧)
        target_y = kin.side_sign * cfg.L1 + COG_Y_SHIFT
        target_z = BODY_HEIGHT + z_offset
        
        res = kin.solve_ik(target_x, target_y, target_z)
        
        if res:
            q1, q2, q3 = res
            pwm_abd = 2048 + (q1 - 0) * kin.rad_to_step * kin.dir_abd
            pwm_hip = 2048 + (q2 - OFFSET_HIP) * kin.rad_to_step * kin.dir_hip
            pwm_knee = 2048 + (q3 - OFFSET_KNEE) * kin.rad_to_step * kin.dir_knee
            return int(pwm_abd), int(pwm_hip), int(pwm_knee)
        else:
            # IK 失败时保持默认位置 (2048 还是上次位置? 这里简单返回 2048 可能会跳变，最好是用上一次的值)
            # 但只要 BODY_HEIGHT 合理，IK 不会失败
            return 2048, 2048, 2048

if __name__ == "__main__":
    app = TrotTester()
    app.run()
