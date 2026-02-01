import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import math
import struct
from datetime import datetime
from robot_config import cfg # 引入配置

# === 用户配置 ===
DISPLAY_SERVO_COUNT = 12 

# === 协议常量 (Protocol V3) ===
BAUD_RATE = 115200
HEAD_1 = 0xA5
HEAD_2 = 0x5A

CMD_TYPE_PING       = 0x01
CMD_TYPE_SERVO_CTRL = 0x10
FB_TYPE_SERVO_INFO  = 0x20
FB_TYPE_SENSOR_IMU  = 0x30
FB_TYPE_RL_STATE    = 0x40 

class SerialManager(threading.Thread):
    def __init__(self, port, log_func, latency_cb, imu_cb):
        super().__init__()
        self.port = port
        self.log = log_func
        self.latency_cb = latency_cb
        self.imu_cb = imu_cb
        self.ser = None
        self.tx_queue = []
        self.running = True
        self.ping_start_time = 0
        self.has_received_data = False # 新增：数据接收标志
        
        try:
            self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.01, write_timeout=0.1)
            self.log(f"[Success] {self.port} Connected (RL-Mode Ready)")
        except Exception as e:
            self.log(f"[Error] {e}")
            self.running = False

    def send_cmd(self, servo_data):
        if not self.ser or not self.running: return
        count = len(servo_data)
        payload = bytearray([count])
        for item in servo_data:
            sid, pos, spd, acc = item
            payload.extend(struct.pack('<B h h B', sid, int(pos), int(spd), int(acc)))
        self.send_raw(CMD_TYPE_SERVO_CTRL, payload)

    def send_ping(self):
        if not self.ser or not self.running: return
        self.ping_start_time = time.perf_counter()
        self.send_raw(CMD_TYPE_PING, b'\xAA\xBB')

    def send_raw(self, pkt_type, payload):
        packet = bytearray([HEAD_1, HEAD_2, pkt_type, len(payload)])
        packet.extend(payload)
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.tx_queue.append(packet)

    def run(self):
        rx_buffer = bytearray()
        while self.running:
            # 1. 优先处理发送 (非阻塞)
            while self.tx_queue:
                try: self.ser.write(self.tx_queue.pop(0))
                except: pass
            
            # 2. 极速读取 (无 sleep)
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    rx_buffer.extend(self.ser.read(waiting))
                    
                    # 3. 解析循环
                    while len(rx_buffer) >= 5:
                        if rx_buffer[0] != HEAD_1 or rx_buffer[1] != HEAD_2:
                            rx_buffer.pop(0); continue
                        
                        pkt_type = rx_buffer[2]
                        # 增加长度保护
                        pkt_len = rx_buffer[3]
                        total_len = 4 + pkt_len + 1
                        
                        if len(rx_buffer) < total_len: break # 数据不够，等下一次
                        
                        pkt = rx_buffer[:total_len]
                        if (sum(pkt[:-1]) & 0xFF) == pkt[-1]:
                            payload = pkt[4:4+pkt_len]
                            
                            if pkt_type == FB_TYPE_SERVO_INFO: self.parse_servo(payload)
                            elif pkt_type == FB_TYPE_SENSOR_IMU: self.parse_imu(payload)
                            elif pkt_type == FB_TYPE_RL_STATE: self.parse_rl_state(payload)
                            elif pkt_type == CMD_TYPE_PING: self.latency_cb((time.perf_counter()-self.ping_start_time)*1000)
                            
                            rx_buffer = rx_buffer[total_len:]
                        else:
                            rx_buffer.pop(0) # 校验失败
                else:
                    time.sleep(0.001) # 只有闲的时候才休息，避免空转烧CPU
            except Exception as e:
                # self.log(f"RX Error: {e}") # 避免刷屏
                time.sleep(0.01)

    def parse_servo(self, payload):
        if len(payload) < 1: return
        count = payload[0]
        idx = 1
        for _ in range(count):
            if idx + 7 > len(payload): break
            sid, pos, spd, load = struct.unpack('<B h h h', payload[idx:idx+7])
            idx += 7
            servo_states[sid] = {'pos': pos, 'speed': spd, 'load': load}
        self.has_received_data = True

    def parse_imu(self, payload):
        if len(payload) < 36: return
        try:
            data = struct.unpack('<9f', payload[:36])
            self.imu_cb(data[6], data[7], data[8])
        except: pass

    def parse_rl_state(self, payload):
        if len(payload) < 37: return
        try:
            imu_data = struct.unpack('<9f', payload[:36])
            self.imu_cb(imu_data[6], imu_data[7], imu_data[8])
            
            count = payload[36]
            idx = 37
            for _ in range(count):
                if idx + 7 > len(payload): break
                sid, pos, spd, load = struct.unpack('<B h h h', payload[idx:idx+7])
                idx += 7
                servo_states[sid] = {'pos': pos, 'speed': spd, 'load': load}
            self.has_received_data = True
        except: pass

# Global States
lock = threading.Lock()
servo_states = {i: {'pos': 2048, 'speed': 0, 'load': 0} for i in range(1, 32)}
CONTROL_MODE = "MANUAL"

class MainApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STM32 RL-Ready 调试终端 (Auto-Sync)")
        self.geometry("1400x900")
        self.configure(bg="#2b2b2b")
        
        self.serial_mgr = None
        self.sliders = {}
        self.fb_labels = {}
        self.pos_vars = {} 
        self.sync_pending = False # 待同步标志
        
        self.setup_styles()
        self.setup_top_bar()
        self.setup_main_area()
        self.setup_log_area()
        
        # === 关键修改：绑定关闭事件 ===
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.after(50, self.update_ui_loop)
        self.after(20, self.control_loop)

    def on_closing(self):
        """安全退出程序"""
        self.log("正在关闭程序...")
        # 1. 停止串口线程
        if self.serial_mgr:
            self.serial_mgr.running = False
            if self.serial_mgr.ser:
                try:
                    self.serial_mgr.ser.close()
                except: pass
        
        # 2. 销毁窗口
        self.destroy()
        # 3. 强制退出 Python 进程 (防止残留)
        import os
        os._exit(0)

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TLabel", foreground="#ffffff", background="#2b2b2b", font=("微软雅黑", 10))
        style.configure("TFrame", background="#2b2b2b")

    def log(self, msg):
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_box.insert(tk.END, f"[{ts}] {msg}\n")
        self.log_box.see(tk.END)

    def setup_top_bar(self):
        bar = tk.Frame(self, bg="#3c3f41", pady=10, padx=10)
        bar.pack(fill="x")
        
        tk.Label(bar, text="通信端口:", fg="#ccc", bg="#3c3f41").pack(side="left")
        self.port_combo = ttk.Combobox(bar, values=[p.device for p in serial.tools.list_ports.comports()], width=15)
        self.port_combo.pack(side="left", padx=5)
        if self.port_combo['values']: self.port_combo.current(0)
        
        self.btn_conn = tk.Button(bar, text="连接", bg="#4CAF50", fg="white", width=12, relief="flat", command=self.toggle_conn)
        self.btn_conn.pack(side="left", padx=5)
        
        self.lbl_imu = tk.Label(bar, text="IMU: 等待...", font=("Consolas", 14, "bold"), bg="#3c3f41", fg="#00E5FF")
        self.lbl_imu.pack(side="left", padx=50)
        
        self.lbl_latency = tk.Label(bar, text="延迟: -- ms", font=("Consolas", 10), bg="#3c3f41", fg="#aaa")
        self.lbl_latency.pack(side="right", padx=10)
        tk.Button(bar, text="Ping", command=lambda: self.serial_mgr.send_ping() if self.serial_mgr else None).pack(side="right")

    def setup_main_area(self):
        container = tk.Frame(self, bg="#2b2b2b")
        container.pack(fill="both", expand=True, padx=10, pady=10)
        
        canvas = tk.Canvas(container, bg="#2b2b2b", highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        self.scrollable_frame = tk.Frame(canvas, bg="#2b2b2b")
        
        self.scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        for i in range(1, DISPLAY_SERVO_COUNT + 1):
            self.create_servo_row(self.scrollable_frame, i)

    def create_servo_row(self, parent, sid):
        row = tk.Frame(parent, bg="#323232", pady=5, padx=10, relief="groove", bd=1)
        row.pack(fill="x", pady=2)
        
        color = "#ffeb3b" if sid <= 6 else "#00e676" 
        tk.Label(row, text=f"ID {sid:02d}", font=("Consolas", 12, "bold"), fg=color, bg="#323232", width=6).pack(side="left")
        
        var = tk.IntVar(value=2048)
        self.pos_vars[sid] = var
        
        slider = tk.Scale(row, from_=0, to=4095, orient="horizontal", variable=var,
                         length=600, bg="#323232", fg="#eee", highlightthickness=0,
                         troughcolor="#1e1e1e", activebackground="#4CAF50",
                         font=("Consolas", 8))
        slider.pack(side="left", padx=20)
        self.sliders[sid] = slider
        
        entry = tk.Entry(row, width=6, font=("Consolas", 12), bg="#1e1e1e", fg="#4CAF50", insertbackground="white")
        entry.insert(0, "2048")
        entry.pack(side="left", padx=10)
        entry.bind("<Return>", lambda e, s=sid, en=entry: self.on_entry_set(s, en))
        
        fb_frame = tk.Frame(row, bg="#323232")
        fb_frame.pack(side="right", padx=10)
        
        tk.Label(fb_frame, text="Real:", fg="#888", bg="#323232").pack(side="left")
        l_pos = tk.Label(fb_frame, text="----", font=("Consolas", 12, "bold"), fg="#ffffff", bg="#1e1e1e", width=6)
        l_pos.pack(side="left", padx=5)
        
        l_load = tk.Label(fb_frame, text="----", font=("Consolas", 12), fg="#ffffff", bg="#1e1e1e", width=6)
        l_load.pack(side="left", padx=5)
        
        self.fb_labels[sid] = [l_pos, None, l_load]

    def on_entry_set(self, sid, entry):
        try:
            val = int(entry.get())
            val = max(0, min(4095, val))
            self.pos_vars[sid].set(val)
        except: pass

    def setup_log_area(self):
        f = tk.Frame(self, bg="#2b2b2b", padx=10, pady=5)
        f.pack(fill="x", side="bottom")
        
        m_frame = tk.Frame(f, bg="#2b2b2b")
        m_frame.pack(side="left", fill="y")
        tk.Button(m_frame, text="手动控制 (MANUAL)", width=20, bg="#444", fg="white", command=lambda: self.set_mode("MANUAL")).pack(pady=2)
        tk.Button(m_frame, text="正弦波测试 (SINE)", width=20, bg="#444", fg="white", command=lambda: self.set_mode("SINE")).pack(pady=2)
        tk.Button(m_frame, text="全局复位 (RESET)", width=20, bg="#800", fg="white", command=lambda: self.set_mode("RESET")).pack(pady=2)
        # 新增粘贴按钮
        tk.Button(m_frame, text="粘贴并执行 (PASTE)", width=20, bg="#2196F3", fg="white", command=self.paste_and_move).pack(pady=2)
        
        self.lbl_mode = tk.Label(m_frame, text="MODE: MANUAL", font=("Arial", 10, "bold"), fg="#ffeb3b", bg="#2b2b2b")
        self.lbl_mode.pack(pady=5)

        self.log_box = scrolledtext.ScrolledText(f, height=6, bg="#1e1e1e", fg="#00ff00", font=("Consolas", 9), relief="flat")
        self.log_box.pack(side="right", fill="both", expand=True, padx=(10, 0))

    def toggle_conn(self):
        if self.serial_mgr and self.serial_mgr.running:
            self.serial_mgr.running = False; self.btn_conn.config(text="连接", bg="#4CAF50")
            self.set_mode("MANUAL") # 断开时重置
        else:
            p = self.port_combo.get()
            if p:
                self.serial_mgr = SerialManager(p, self.log, lambda r: self.lbl_latency.config(text=f"延迟: {r:.1f} ms"), 
                                               lambda r,p,y: self.lbl_imu.config(text=f"IMU: R:{r:>6.2f} P:{p:>6.2f} Y:{y:>6.2f}"))
                self.serial_mgr.daemon = True # 设置为守护线程
                self.serial_mgr.start(); self.btn_conn.config(text="断开", bg="#f44336")
                
                # === 关键修改: 进入同步模式 ===
                self.set_mode("SYNCING")
                self.sync_pending = True
                self.log("正在同步舵机位置...")

    def set_mode(self, m):
        global CONTROL_MODE; CONTROL_MODE = m
        self.lbl_mode.config(text=f"MODE: {m}")
        if m == "RESET":
            for sid in self.pos_vars: self.pos_vars[sid].set(2048)

    def paste_and_move(self):
        """利用硬件特性的真·平滑移动"""
        import json
        try:
            content = self.clipboard_get()
            targets = json.loads(content)
            
            # 直接把所有变量设为目标值
            # control_loop 会在下一次循环捕获到这些值，并发送给舵机
            for str_id, pos in targets.items():
                i = int(str_id)
                if 1 <= i <= DISPLAY_SERVO_COUNT:
                    self.pos_vars[i].set(pos) 
            
            self.log(f"指令已下达，硬件自动平滑移动")
            
        except Exception as e:
            self.log(f"错误: {e}")

    def smooth_move_task(self, targets):
        """利用舵机内置规划进行丝滑运动"""
        self.set_mode("MANUAL") 
        
        # 1. 准备指令数据
        # 协议: (sid, pos, speed, acc)
        # 我们不再切分小步，而是发一条带速度/加速度限制的指令
        
        cmd_data = []
        
        for sid_str, target in targets.items():
            sid = int(sid_str)
            if sid not in self.pos_vars: continue
            
            current = self.pos_vars[sid].get()
            diff = abs(target - current)
            
            if diff > 0:
                # 自适应速度策略 (从配置读取上限)
                # adaptive_speed = int(diff * 1.5) 
                adaptive_speed = 500
                
                # 加速度: 从配置读取
                acc = 20
                
                cmd_data.append((sid, target, adaptive_speed, acc))
                
                # 直接更新 UI 滑块到终点，不再慢慢爬
                self.pos_vars[sid].set(target)
        
        # 2. 发送指令
        if cmd_data:
            self.log(f"发送 {len(cmd_data)} 个平滑指令...")
            self.serial_mgr.send_cmd(cmd_data)
            
        self.log("指令已下达")

    def update_ui_loop(self):
        # 1. 检查是否需要执行同步
        if self.sync_pending and self.serial_mgr and self.serial_mgr.has_received_data:
            self.log("收到反馈，开始同步滑块...")
            for i in range(1, DISPLAY_SERVO_COUNT + 1):
                if i in servo_states:
                    real_pos = servo_states[i]['pos']
                    if real_pos > 0: # 过滤无效值
                        self.pos_vars[i].set(real_pos)
            self.sync_pending = False
            self.set_mode("MANUAL") # 同步完成，允许控制
            self.log("同步完成，控制权已移交 (MANUAL)")

        # 2. 常规 UI 更新
        for i in range(1, DISPLAY_SERVO_COUNT + 1):
            if i in servo_states:
                st = servo_states[i]
                lbls = self.fb_labels[i]
                lbls[0].config(text=str(st['pos']))
                # lbls[1].config(text=str(st['speed'])) # 空间不够省略速度显示
                lbls[2].config(text=f"L:{st['load']}")
                load_val = abs(st['load'])
                lbls[2].config(fg="#ff4444" if load_val > 500 else "#ffffff")
        self.after(50, self.update_ui_loop)

    def control_loop(self):
        if self.serial_mgr and self.serial_mgr.running:
            # === 关键修改: 如果正在同步，禁止发送指令 ===
            if CONTROL_MODE == "SYNCING": 
                self.after(20, self.control_loop)
                return

            data = []
            if CONTROL_MODE == "MANUAL":
                for i in range(1, DISPLAY_SERVO_COUNT + 1):
                    data.append((i, self.pos_vars[i].get(),500, 20))
            elif CONTROL_MODE == "SINE":
                t = time.time() * 2
                for i in range(1, DISPLAY_SERVO_COUNT + 1):
                    pos = 2048 + 800 * math.sin(t + i*0.4)
                    self.pos_vars[i].set(int(pos))
                    data.append((i, int(pos), 0, 0))
            self.serial_mgr.send_cmd(data)
        self.after(20, self.control_loop)

if __name__ == "__main__":
    app = MainApp(); app.mainloop()