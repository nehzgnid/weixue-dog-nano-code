import time
import math
import struct
import serial
import serial.tools.list_ports
import threading
import numpy as np

from robot_config import cfg
from kinematics_v5 import LegKinematics, OFFSET_HIP, OFFSET_KNEE

# === 配置 ===
BAUD_RATE = 115200
BODY_HEIGHT = -180.0
STEP_HEIGHT_TARGET = 30.0 
FREQ = 1.5 
SOFT_START_DURATION = 3.0

# 重心微调
COG_X_SHIFT = 0.0 
COG_Y_SHIFT = 0.0

# 开环摆动参数 (作为前馈)
BODY_SWAY_X = 10.0 
BODY_SWAY_Y = 20.0 

# === 闭环 PID 参数 ===
# Roll 控制 (左右平衡)
KP_ROLL = 3.0   # 每倾斜 1 度，腿伸缩 3mm
KD_ROLL = 0.1   # 阻尼
# Pitch 控制 (前后平衡)
KP_PITCH = 3.0  
KD_PITCH = 0.1

# 协议常量
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_SERVO_CTRL = 0x10
CMD_TYPE_TORQUE     = 0x11
FB_TYPE_SENSOR_IMU  = 0x30
FB_TYPE_RL_STATE    = 0x40 

class PID:
    def __init__(self, kp, ki, kd, limit=30):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.prev_error = 0
        self.integral = 0
        
    def update(self, error, dt):
        # 延迟尖峰保护：如果通信卡顿，dt 可能很大 (e.g. 0.1s)
        # 强制认为 dt 最大为 0.05s (20Hz)，防止积分暴涨
        dt = min(dt, 0.05) 
        if dt <= 0: return 0
        
        self.integral += error * dt
        # 积分限幅
        self.integral = max(-self.limit, min(self.limit, self.integral))
        
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return max(-self.limit, min(self.limit, output))

class WalkTesterClosedLoop:
    def __init__(self):
        self.ser = None
        self.running = True
        self.legs = {name: LegKinematics(name) for name in ["FL", "FR", "RL", "RR"]}
        self.leg_ids = {
            "FL": [1, 2, 3], "FR": [4, 5, 6],
            "RL": [7, 8, 9], "RR": [10, 11, 12]
        }
        
        self.phase_offsets = {"FL": 0.0, "RR": 0.25, "FR": 0.50, "RL": 0.75}
        
        # IMU Data (Shared)
        self.imu_lock = threading.Lock()
        self.current_rpy = (0.0, 0.0, 0.0) # Roll, Pitch, Yaw
        self.imu_updated = False
        
        self.pid_roll = PID(KP_ROLL, 0, KD_ROLL)
        self.pid_pitch = PID(KP_PITCH, 0, KD_PITCH)
        
        self.connect()

    def connect(self):
        print("正在扫描串口...")
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("[错误] 未找到任何串口设备！"); self.running = False; return

        print("\n=== 可用串口列表 ===")
        for i, p in enumerate(ports):
            print(f"{i+1}. {p.device} ({p.description})")
        
        target_port = None
        if len(ports) == 1:
            if input(f"\n连接 {ports[0].device}? [y]: ").lower() != 'n': target_port = ports[0].device
        else:
            idx = input("选择序号: ")
            if idx.isdigit() and 0 < int(idx) <= len(ports): target_port = ports[int(idx)-1].device

        if target_port:
            try:
                self.ser = serial.Serial(target_port, BAUD_RATE, timeout=0.01)
                print(f"成功连接 {target_port}")
                # 开启接收线程
                self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
                self.rx_thread.start()
            except Exception as e:
                print(f"连接失败: {e}"); self.running = False
        else: self.running = False

    def rx_loop(self):
        """后台接收 IMU 数据"""
        buffer = bytearray()
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    buffer.extend(self.ser.read(self.ser.in_waiting))
                    
                    while len(buffer) >= 5:
                        if buffer[0] != HEAD_1 or buffer[1] != HEAD_2:
                            del buffer[0]; continue
                            
                        pkt_len = buffer[3]
                        if len(buffer) < 4 + pkt_len + 1: break
                        
                        pkt = buffer[:4+pkt_len+1]
                        if (sum(pkt[:-1]) & 0xFF) == pkt[-1]:
                            pkt_type = pkt[2]
                            payload = pkt[4:4+pkt_len]
                            
                            if pkt_type == FB_TYPE_SENSOR_IMU or pkt_type == FB_TYPE_RL_STATE:
                                # 解析 IMU (前36字节是 9个float: acc, gyro, rpy)
                                if len(payload) >= 36:
                                    data = struct.unpack('<9f', payload[:36])
                                    r, p, y = data[6], data[7], data[8]
                                    with self.imu_lock:
                                        self.current_rpy = (r, p, y)
                                        self.imu_updated = True
                            
                            del buffer[:len(pkt)]
                        else:
                            del buffer[:2]
                else:
                    time.sleep(0.001)
            except: time.sleep(0.01)

    def send_raw(self, pkt_type, payload):
        packet = bytearray([HEAD_1, HEAD_2, pkt_type, len(payload)])
        packet.extend(payload)
        packet.append(sum(packet) & 0xFF)
        if self.ser: self.ser.write(packet)

    def send_torque(self, enable):
        self.send_raw(CMD_TYPE_TORQUE, bytearray([1 if enable else 0]))

    def send_all_pos(self, servo_data_list):
        count = len(servo_data_list)
        payload = bytearray([count])
        for item in servo_data_list:
            sid, pos, spd, acc = item
            payload.extend(struct.pack('<B h h B', sid, int(pos), int(spd), int(acc)))
        self.send_raw(CMD_TYPE_SERVO_CTRL, payload)

    def run(self):
        if not self.running: return

        self.send_torque(False)
        print("\n=== IMU 闭环 Walk 测试 ===")
        print(f"Kp_Roll: {KP_ROLL}, Kp_Pitch: {KP_PITCH}")
        input("按 Enter 开始 (Ctrl+C 停止)...")
        
        self.send_torque(True)
        # 归位
        init_cmds = []
        for name in self.legs:
            p1, p2, p3 = self.calc_pwm(name, 0, 0, 0, 0)
            ids = self.leg_ids[name]
            init_cmds.extend([(ids[0], p1, 1000, 50), (ids[1], p2, 1000, 50), (ids[2], p3, 1000, 50)])
        self.send_all_pos(init_cmds)
        time.sleep(2.0)
        
        print("开始闭环行走...")
        start_time = time.perf_counter()
        last_loop_time = start_time
        
        try:
            while self.running:
                now = time.perf_counter()
                dt = now - last_loop_time
                last_loop_time = now
                
                elapsed = now - start_time
                step_h = min(STEP_HEIGHT_TARGET, STEP_HEIGHT_TARGET * (elapsed / SOFT_START_DURATION))
                
                # === 1. 获取 IMU 数据 ===
                with self.imu_lock:
                    curr_roll, curr_pitch, _ = self.current_rpy
                
                # === 2. 计算 PID 输出 (平衡修正) ===
                # 目标是 0 度
                # 如果 Roll > 0 (向右歪)，我们需要让右腿伸长 (+Z)，左腿缩短 (-Z)
                # 或者：让右腿多撑一点力。通常伸长腿可以把身体顶回去。
                # 注意：LegKinematics 中 Z 越小越向下（伸长）。BODY_HEIGHT 是负数。
                # 如果要伸长，Z 应该更小 (更负)。
                # 所以 PID 输出正值代表 "修正量"，需要根据符号加减。
                
                adj_roll = self.pid_roll.update(0 - curr_roll, dt)   # Roll Error
                adj_pitch = self.pid_pitch.update(0 - curr_pitch, dt) # Pitch Error
                
                # 打印调试 (每 0.5s)
                if int(now * 2) > int((now-dt)*2):
                    print(f"R:{curr_roll:.1f} P:{curr_pitch:.1f} | AdjR:{adj_roll:.1f} AdjP:{adj_pitch:.1f}")

                # === 3. 步态生成 ===
                period = 1.0 / FREQ
                global_phase = (now % period) / period
                
                # 计算开环 Sway (前馈)
                swing_leg = None
                swing_duty = 0.25
                for name, offset in self.phase_offsets.items():
                    p = (global_phase + offset) % 1.0
                    if p < swing_duty: swing_leg = name; break
                
                target_sway_x, target_sway_y = 0, 0
                if swing_leg == "FL":   target_sway_x, target_sway_y = -BODY_SWAY_X, -BODY_SWAY_Y
                elif swing_leg == "RR": target_sway_x, target_sway_y = BODY_SWAY_X, BODY_SWAY_Y
                elif swing_leg == "FR": target_sway_x, target_sway_y = -BODY_SWAY_X, BODY_SWAY_Y
                elif swing_leg == "RL": target_sway_x, target_sway_y = BODY_SWAY_X, -BODY_SWAY_Y
                
                if not hasattr(self, 'cur_sway_x'): self.cur_sway_x, self.cur_sway_y = 0, 0
                alpha = 0.1
                self.cur_sway_x += (target_sway_x - self.cur_sway_x) * alpha
                self.cur_sway_y += (target_sway_y - self.cur_sway_y) * alpha
                
                all_servos = []
                for name in self.legs:
                    leg_phase = (global_phase + self.phase_offsets[name]) % 1.0
                    z_offset = 0.0
                    if leg_phase < swing_duty:
                        local_p = leg_phase / swing_duty
                        z_offset = step_h * math.sin(local_p * math.pi)
                    
                    # === 4. 融合平衡控制 ===
                    # adj_roll > 0 意味着需要向左修正 (Roll < 0, 向左歪了? 不, error=0-curr. 如果 curr=-5(左歪), err=5, adj=15)
                    # 如果向左歪 (Roll < 0)，adj > 0。我们需要左腿伸长 (Z变小)，右腿缩短 (Z变大)。
                    # 左腿 (FL, RL): Z_final = Z_nom - adj_roll
                    # 右腿 (FR, RR): Z_final = Z_nom + adj_roll
                    
                    # Pitch: 如果向前歪 (Pitch > 0? 假设抬头为正? 需确认 IMU 方向)
                    # 假设 Pitch > 0 是车头抬起。则 Error < 0, adj < 0。
                    # 车头抬起，需要前腿缩短 (Z变大)，后腿伸长 (Z变小)。
                    # 前腿: Z_final = Z_nom - adj_pitch (adj是负的，所以Z变大，缩短。Correct)
                    # 后腿: Z_final = Z_nom + adj_pitch (adj是负的，所以Z变小，伸长。Correct)
                    
                    balance_z = 0
                    if "L" in name: balance_z -= adj_roll
                    else:           balance_z += adj_roll
                    
                    if "F" in name: balance_z -= adj_pitch
                    else:           balance_z += adj_pitch
                    
                    # 最终 PWM 计算
                    p1, p2, p3 = self.calc_pwm(name, z_offset, self.cur_sway_x, self.cur_sway_y, balance_z)
                    ids = self.leg_ids[name]
                    all_servos.extend([(ids[0], p1, 0, 0), (ids[1], p2, 0, 0), (ids[2], p3, 0, 0)])
                    
                self.send_all_pos(all_servos)
                time.sleep(0.015) 
                
        except KeyboardInterrupt:
            print("停止")
        finally:
            self.send_torque(False)
            if self.ser: self.ser.close()

    def calc_pwm(self, leg_name, z_offset, sway_x, sway_y, balance_z):
        kin = self.legs[leg_name]
        
        target_x = 0 + COG_X_SHIFT - sway_x
        target_y = kin.side_sign * cfg.L1 + COG_Y_SHIFT - sway_y
        
        # 基础高度 + 抬腿动作 + 平衡修正
        # 注意: 抬腿时 z_offset > 0。
        # balance_z 是修正量。
        # BODY_HEIGHT 是负数 (比如 -180)。
        # 如果要伸长腿，target_z 应该更负。
        target_z = BODY_HEIGHT + z_offset + balance_z
        
        # 限幅保护 (防止 PID 发散)
        target_z = max(-240, min(-120, target_z))
        
        res = kin.solve_ik(target_x, target_y, target_z)
        if res:
            q1, q2, q3 = res
            p1 = 2048 + (q1 - 0) * kin.rad_to_step * kin.dir_abd
            p2 = 2048 + (q2 - OFFSET_HIP) * kin.rad_to_step * kin.dir_hip
            p3 = 2048 + (q3 - OFFSET_KNEE) * kin.rad_to_step * kin.dir_knee
            return int(p1), int(p2), int(p3)
        return 2048, 2048, 2048

if __name__ == "__main__":
    app = WalkTesterClosedLoop()
    app.run()
