import time
import struct
import serial
import serial.tools.list_ports
import threading
import statistics
import sys
import os

# === 配置 ===
BAUD_RATE = 115200
TARGET_ID = 1
START_POS = 2048
END_POS   = 2300
TEST_ROUNDS = 20

# 协议常量
HEAD_1 = 0xA5
HEAD_2 = 0x5A
CMD_TYPE_SERVO_CTRL = 0x10
CMD_TYPE_TORQUE     = 0x11
FB_TYPE_SERVO_INFO  = 0x20
FB_TYPE_RL_STATE    = 0x40

class LatencyTesterLinux:
    def __init__(self):
        self.ser = None
        self.running = True
        self.latest_pos = -1
        self.packet_count = 0
        self.lock = threading.Lock()
        
        self.connect()
        
    def connect(self):
        print("正在扫描串口 (Linux/Jetson)...")
        ports = list(serial.tools.list_ports.comports())
        
        # 过滤出可能的设备 (ACM 或 USB)
        candidates = [p for p in ports if "ACM" in p.device or "USB" in p.device]
        
        if not candidates:
            print("[警告] 未检测到典型串口设备 (ttyACM/ttyUSB)，显示所有设备：")
            candidates = ports

        if not candidates:
            print("[错误] 未找到任何串口设备！")
            self.running = False
            return

        print("\n=== 可用串口列表 ===")
        for i, p in enumerate(candidates):
            print(f"{i+1}. {p.device} - {p.description} [{p.hwid}]")
        
        target_port = None
        if len(candidates) == 1:
            # 自动选择唯一的候选者
            print(f"\n自动选择: {candidates[0].device}")
            target_port = candidates[0].device
        else:
            while True:
                idx = input("\n请输入序号选择 (1-{}): ".format(len(candidates))).strip()
                if idx.isdigit() and 1 <= int(idx) <= len(candidates):
                    target_port = candidates[int(idx)-1].device
                    break

        if target_port:
            try:
                # Linux 下 timeout=0 表示非阻塞读，这对于高频读取很重要
                self.ser = serial.Serial(target_port, BAUD_RATE, timeout=0) 
                print(f"成功连接到 {target_port}")
                
                # 开启接收线程
                self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
                self.rx_thread.start()
            except Exception as e: 
                print(f"连接失败: {e}"); self.running = False
        else: self.running = False

    def rx_loop(self):
        """Linux 优化版读取循环"""
        buffer = bytearray()
        while self.running and self.ser:
            try:
                # 非阻塞读取，有多少读多少
                waiting = self.ser.in_waiting
                if waiting > 0:
                    buffer.extend(self.ser.read(waiting))
                    
                    while len(buffer) >= 5:
                        if buffer[0] != HEAD_1 or buffer[1] != HEAD_2:
                            del buffer[0]; continue
                        
                        pkt_len = buffer[3]
                        total_len = 4 + pkt_len + 1
                        if len(buffer) < total_len: break
                        
                        pkt = buffer[:total_len]
                        if (sum(pkt[:-1]) & 0xFF) == pkt[-1]:
                            pkt_type = pkt[2]
                            payload = pkt[4:4+pkt_len]
                            
                            with self.lock:
                                self.packet_count += 1
                            
                            if pkt_type == FB_TYPE_RL_STATE and len(payload) >= 37:
                                count = payload[36]
                                idx = 37
                                for _ in range(count):
                                    if idx + 7 > len(payload): break
                                    sid, pos, spd, load = struct.unpack('<B h h h', payload[idx:idx+7])
                                    idx += 7
                                    if sid == TARGET_ID:
                                        with self.lock:
                                            self.latest_pos = pos
                            
                            del buffer[:total_len]
                        else:
                            del buffer[:2]
                else:
                    # 极短睡眠，避免 100% CPU 占用
                    # Linux sleep精度较高
                    time.sleep(0.0005) 
            except OSError:
                print("串口设备已断开！")
                self.running = False
                break
            except Exception:
                pass

    def send_raw(self, pkt_type, payload):
        packet = bytearray([HEAD_1, HEAD_2, pkt_type, len(payload)])
        packet.extend(payload)
        packet.append(sum(packet) & 0xFF)
        if self.ser: self.ser.write(packet)

    def send_pos(self, sid, pos, spd=0, acc=0):
        payload = bytearray([1])
        payload.extend(struct.pack('<B h h B', sid, int(pos), int(spd), int(acc)))
        self.send_raw(CMD_TYPE_SERVO_CTRL, payload)

    def send_torque(self, enable):
        self.send_raw(CMD_TYPE_TORQUE, bytearray([1 if enable else 0]))

    def run(self):
        if not self.running: return
        
        print(f"\n=== Jetson Nano 延迟测试工具 ===")
        print("1. 通信往返测试 (RTT) - 测试 Linux CDC 驱动性能")
        print("2. 物理启动测试 (Inertia) - 测试控制响应")
        
        choice = input("请选择 (1/2): ").strip()
        
        if choice == '1': self.run_ping_test()
        else: self.run_physical_test()
            
        self.running = False
        self.send_torque(False)
        if self.ser: self.ser.close()

    def run_ping_test(self):
        print(f"\n=== RTT 测试 (Linux) ===")
        
        results = []
        for i in range(TEST_ROUNDS):
            with self.lock: start_cnt = self.packet_count
            
            t0 = time.perf_counter()
            self.send_pos(TARGET_ID, self.latest_pos, 0, 0)
            
            while True:
                with self.lock: curr_cnt = self.packet_count
                if curr_cnt > start_cnt:
                    t1 = time.perf_counter()
                    break
                if time.perf_counter() - t0 > 0.5: 
                    t1 = t0 + 0.5 # Timeout
                    break
            
            latency = (t1 - t0) * 1000.0
            print(f"Ping {i+1}: {latency:.2f} ms")
            if latency < 500: results.append(latency)
            
            # Linux 下可以跑得更快，但为了对比，保持一定间隔
            time.sleep(0.05)
            
        if results:
            print(f"Avg RTT: {statistics.mean(results):.2f} ms")
            print("注意: Linux (cdc_acm) 通常不需要特殊驱动设置，延迟应在 1-5ms。")

    def run_physical_test(self):
        print(f"\n=== 物理启动测试 ===")
        input("确认电机安全，按 Enter 开始...")
        self.send_torque(True)
        time.sleep(1.0)
        
        results = []
        MOVE_THRESHOLD = 5
        
        for i in range(10):
            print(f"\nRound {i+1}:")
            self.send_pos(TARGET_ID, START_POS, 1000, 50)
            time.sleep(1.0)
            
            with self.lock: start_p = self.latest_pos
            
            t0 = time.perf_counter()
            self.send_pos(TARGET_ID, END_POS, 0, 0)
            
            t1 = t0
            while True:
                with self.lock: curr_p = self.latest_pos
                if abs(curr_p - start_p) > MOVE_THRESHOLD:
                    t1 = time.perf_counter()
                    break
                if time.perf_counter() - t0 > 1.0: break
            
            lat = (t1 - t0) * 1000.0
            print(f"  Response: {lat:.2f} ms")
            results.append(lat)
            
        print(f"\nAvg Physical Latency: {statistics.mean(results):.2f} ms")

if __name__ == "__main__":
    app = LatencyTesterLinux()
    app.run()
