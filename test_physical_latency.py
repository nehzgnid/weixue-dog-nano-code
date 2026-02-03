import time
import struct
import serial
import serial.tools.list_ports
import threading
import statistics
import sys
import numpy as np

# === 配置 ===
BAUD_RATE = 115200
TARGET_ID = 4     # 选择一条前腿 (FL_KNEE or FR_KNEE)
KICK_POS  = 2600  # 猛踢的位置 (向下伸腿)
REST_POS  = 2048  # 收腿位置

# 协议常量
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_SERVO_CTRL = 0x10
CMD_TYPE_TORQUE     = 0x11
FB_TYPE_RL_STATE    = 0x40 

class PhysicalLatencyTester:
    def __init__(self):
        self.ser = None
        self.running = True
        self.lock = threading.Lock()
        self.latest_acc_z = 0.0
        self.base_acc_z = 1.0 # 重力 G
        
        self.connect()
        
    def connect(self):
        print("正在扫描串口 (Jetson Nano)...")
        ports = list(serial.tools.list_ports.comports())
        candidates = [p for p in ports if "ACM" in p.device or "USB" in p.device]
        if not candidates: candidates = ports

        if not candidates:
            print("[错误] 未找到串口"); return

        target_port = candidates[0].device
        print(f"连接到: {target_port}")
        
        try:
            self.ser = serial.Serial(target_port, BAUD_RATE, timeout=0) 
            # 开启接收线程
            self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
            self.rx_thread.start()
        except Exception as e:
            print(f"连接失败: {e}"); self.running = False

    def rx_loop(self):
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
                            if pkt[2] == FB_TYPE_RL_STATE and len(pkt) >= 41: # 4+37
                                # 解析 IMU (前36字节 float*9)
                                data = struct.unpack('<9f', pkt[4:40])
                                # data[0,1,2] 是 Acc X,Y,Z
                                with self.lock:
                                    self.latest_acc_z = data[2] # Z轴加速度
                            del buffer[:len(pkt)]
                        else:
                            del buffer[:2]
                else:
                    time.sleep(0.0005)
            except: pass

    def send_pos(self, sid, pos, spd=2000, acc=100):
        # 极速发送指令
        payload = bytearray([1])
        payload.extend(struct.pack('<B h h B', sid, int(pos), int(spd), int(acc)))
        pkt = bytearray([HEAD_1, HEAD_2, CMD_TYPE_SERVO_CTRL, len(payload)])
        pkt.extend(payload)
        pkt.append(sum(pkt) & 0xFF)
        if self.ser: self.ser.write(pkt)

    def send_torque(self, enable):
        pkt = bytearray([HEAD_1, HEAD_2, CMD_TYPE_TORQUE, 1, 1 if enable else 0])
        pkt.append(sum(pkt) & 0xFF)
        if self.ser: self.ser.write(pkt)

    def run(self):
        if not self.running: return
        print("\n=== 物理响应延迟测试 (Impact Test) ===")
        print("原理: 发送指令 -> IMU检测到震动 的时间差")
        print(f"测试舵机 ID: {TARGET_ID}")
        print("请确保机器人放在桌面上，腿部悬空或轻触地面")
        input("按 Enter 开始...")
        
        self.send_torque(True)
        time.sleep(1)
        
        # 1. 归位
        self.send_pos(TARGET_ID, REST_POS)
        time.sleep(1)
        
        # 2. 采样静态 AccZ
        print("校准 IMU 基准值...")
        acc_samples = []
        for _ in range(50):
            with self.lock: acc_samples.append(self.latest_acc_z)
            time.sleep(0.01)
        self.base_acc_z = statistics.mean(acc_samples)
        print(f"基准 AccZ: {self.base_acc_z:.3f} g")
        
        threshold = 0.15 # 触发阈值 (0.15g 的震动)
        
        results = []
        for i in range(10):
            print(f"\nRound {i+1}:")
            # 准备
            self.send_pos(TARGET_ID, REST_POS)
            time.sleep(0.8)
            
            # 踢!
            t0 = time.perf_counter()
            self.send_pos(TARGET_ID, KICK_POS, 0, 0) # 0=Max Speed
            
            # 监测震动
            triggered = False
            t_impact = 0
            while time.perf_counter() - t0 < 0.5: # 500ms 超时
                with self.lock: curr_acc = self.latest_acc_z
                if abs(curr_acc - self.base_acc_z) > threshold:
                    t_impact = time.perf_counter()
                    triggered = True
                    break
            
            if triggered:
                latency = (t_impact - t0) * 1000.0
                print(f"  Impact Detected! Latency: {latency:.2f} ms")
                results.append(latency)
            else:
                print("  No Impact detected (Check threshold or servo power)")
        
        if results:
            print(f"\n=== 最终结果 ===")
            print(f"平均物理响应延迟: {statistics.mean(results):.2f} ms")
            print(f"其中包含约 7ms 通信延迟")
            print(f"实际电机机械延迟: {statistics.mean(results) - 7:.2f} ms")
        
        self.send_torque(False)

if __name__ == "__main__":
    app = PhysicalLatencyTester()
    app.run()
