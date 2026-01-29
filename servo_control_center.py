import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import math
import struct
from datetime import datetime

# === 用户配置 ===
# 如果您的实际舵机少于6个，可以在这里修改数量
# 但注意：下位机目前的固件会始终发送6个舵机的数据包(39字节)
# 所以这里即使改小，也只是界面显示变少，底层解析依然需要处理39字节
DISPLAY_SERVO_COUNT = 2

# === 协议常量 ===
BAUD_RATE = 115200
FB_SIZE = 39
FB_HEAD_1 = 0xA5
FB_HEAD_2 = 0x5B
CMD_HEAD_1 = 0xA5
CMD_HEAD_2 = 0x5A

# === 全局共享 ===
lock = threading.Lock()
servo_states = {i: {'pos': 0, 'speed': 0, 'load': 0} for i in range(1, 7)}
running = False
CONTROL_MODE = "MANUAL"

class SerialManager(threading.Thread):
    def __init__(self, port, log_func):
        super().__init__()
        self.port = port
        self.log = log_func
        self.ser = None
        self.tx_queue = []
        try:
            self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.01, write_timeout=0.1)
            self.log(f"[Success] 已连接串口 {self.port}")
            global running
            running = True
        except Exception as e:
            self.log(f"[Error] 串口连接失败: {e}")

    def send_cmd(self, servo_data):
        if not self.ser or not running: return
        
        # 构建包
        packet = bytearray([CMD_HEAD_1, CMD_HEAD_2])
        # 始终填充6个舵机的数据以匹配下位机协议，没有的补0
        for i in range(1, 7):
            # 在传入的数据中查找 ID i
            target = next((item for item in servo_data if item[0] == i), None)
            
            if target:
                sid, pos, spd, acc = target
            else:
                sid, pos, spd, acc = (i, 0, 0, 0) # 默认填充
            
            pos = max(0, min(4095, int(pos)))
            spd = max(0, min(3000, int(spd)))
            acc = max(0, min(254, int(acc)))
            
            packet.append(sid)
            packet.append(pos & 0xFF)
            packet.append((pos >> 8) & 0xFF)
            packet.append(spd & 0xFF)
            packet.append((spd >> 8) & 0xFF)
            packet.append(acc & 0xFF)
            
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        
        with lock:
            self.tx_queue.append(packet)

    def run(self):
        global servo_states, running
        if not self.ser: return
        
        rx_buffer = bytearray()
        last_rx_time = time.time()
        
        while running:
            # 1. TX
            with lock:
                while self.tx_queue:
                    try:
                        pkt = self.tx_queue.pop(0)
                        self.ser.write(pkt)
                    except: pass
            
            # 2. RX
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    rx_buffer.extend(data)
                    last_rx_time = time.time()
                
                while len(rx_buffer) >= FB_SIZE:
                    if rx_buffer[0] == FB_HEAD_1 and rx_buffer[1] == FB_HEAD_2:
                        pkt = rx_buffer[:FB_SIZE]
                        if (sum(pkt[:-1]) & 0xFF) == pkt[-1]:
                            self.parse_feedback(pkt)
                            rx_buffer = rx_buffer[FB_SIZE:]
                        else:
                            rx_buffer = rx_buffer[1:]
                    else:
                        rx_buffer = rx_buffer[1:]
            except Exception as e:
                self.log(f"[RX Error] {e}")
                
            time.sleep(0.002)
        
        self.ser.close()
        self.log("[Info] 串口已关闭")

    def parse_feedback(self, pkt):
        idx = 2
        for i in range(1, 7):
            pos = int.from_bytes(pkt[idx:idx+2], 'little', signed=True)
            spd = int.from_bytes(pkt[idx+2:idx+4], 'little', signed=True)
            load = int.from_bytes(pkt[idx+4:idx+6], 'little', signed=True)
            idx += 6
            servo_states[i] = {'pos': pos, 'speed': spd, 'load': load}

class MainApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STM32 舵机控制中心 (增强版)")
        self.geometry("1000x700")
        
        self.serial_mgr = None
        self.sliders = {}
        self.fb_labels = {} 
        
        self.setup_top_bar()
        self.setup_main_area()
        self.setup_log_area()
        
        # 定时器
        self.update_ui_loop()
        self.control_loop()

    def log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_box.insert(tk.END, f"[{ts}] {msg}\n")
        self.log_box.see(tk.END)

    def setup_top_bar(self):
        bar = tk.Frame(self, pady=5, padx=5, bg="#ddd")
        bar.pack(fill="x")
        
        tk.Label(bar, text="串口:").pack(side="left")
        
        self.port_combo = ttk.Combobox(bar, values=self.get_ports(), width=15)
        self.port_combo.pack(side="left", padx=5)
        if self.port_combo['values']: self.port_combo.current(0)
        
        self.btn_connect = tk.Button(bar, text="连接", command=self.toggle_connect, bg="green", fg="white", width=10)
        self.btn_connect.pack(side="left", padx=5)
        
        tk.Button(bar, text="刷新串口", command=self.refresh_ports).pack(side="left")

    def get_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def refresh_ports(self):
        self.port_combo['values'] = self.get_ports()
        if self.port_combo['values']: self.port_combo.current(0)

    def toggle_connect(self):
        global running
        if self.serial_mgr and running:
            # 断开
            running = False
            self.btn_connect.config(text="连接", bg="green")
            self.log("正在断开连接...")
        else:
            # 连接
            port = self.port_combo.get()
            if not port: return
            self.serial_mgr = SerialManager(port, self.log)
            self.serial_mgr.start()
            self.btn_connect.config(text="断开", bg="red")

    def setup_main_area(self):
        paned = tk.PanedWindow(self, orient="horizontal")
        paned.pack(fill="both", expand=True, padx=5, pady=5)
        
        # === 左侧控制 ===
        left = tk.LabelFrame(paned, text="发送指令 (Control)", padx=5, pady=5)
        paned.add(left)
        
        btn_frame = tk.Frame(left)
        btn_frame.pack(fill="x", pady=5)
        tk.Button(btn_frame, text="手动模式", command=lambda: self.set_mode("MANUAL"), width=10).pack(side="left", padx=2)
        tk.Button(btn_frame, text="正弦波", command=lambda: self.set_mode("SINE"), width=10).pack(side="left", padx=2)
        tk.Button(btn_frame, text="归中", command=lambda: self.set_mode("RESET"), width=10).pack(side="left", padx=2)
        
        self.lbl_mode = tk.Label(left, text="当前: MANUAL", fg="blue", font=("Arial", 10, "bold"))
        self.lbl_mode.pack(pady=5)
        
        # 滚动区域
        canvas = tk.Canvas(left)
        scrollbar = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        scroll_frame = tk.Frame(canvas)
        
        scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        for i in range(1, DISPLAY_SERVO_COUNT + 1):
            f = tk.Frame(scroll_frame, pady=2)
            f.pack(fill="x")
            tk.Label(f, text=f"ID {i}").pack(side="left")
            s = tk.Scale(f, from_=0, to=4095, orient="horizontal", length=200)
            s.set(2048)
            s.pack(side="left", padx=5)
            self.sliders[i] = s

        # === 右侧反馈 ===
        right = tk.LabelFrame(paned, text="接收反馈 (Feedback)", padx=5, pady=5)
        paned.add(right)
        
        # 表头
        h_frame = tk.Frame(right)
        h_frame.pack(fill="x")
        for idx, t in enumerate(["ID", "Pos", "Spd", "Load"]):
            tk.Label(h_frame, text=t, width=6, font=("bold")).grid(row=0, column=idx)
            
        for i in range(1, DISPLAY_SERVO_COUNT + 1):
            f = tk.Frame(right)
            f.pack(fill="x", pady=2)
            tk.Label(f, text=f"{i}", width=6).grid(row=0, column=0)
            l_pos = tk.Label(f, text="--", width=6, bg="white", relief="sunken")
            l_pos.grid(row=0, column=1)
            l_spd = tk.Label(f, text="--", width=6, bg="white", relief="sunken")
            l_spd.grid(row=0, column=2)
            l_load = tk.Label(f, text="--", width=6, bg="white", relief="sunken")
            l_load.grid(row=0, column=3)
            
            self.fb_labels[i] = [l_pos, l_spd, l_load]

    def setup_log_area(self):
        f = tk.LabelFrame(self, text="系统日志")
        f.pack(side="bottom", fill="x", padx=5, pady=5)
        self.log_box = scrolledtext.ScrolledText(f, height=6, state='normal', bg="black", fg="#00ff00")
        self.log_box.pack(fill="both")

    def set_mode(self, mode):
        global CONTROL_MODE
        CONTROL_MODE = mode
        self.lbl_mode.config(text=f"当前: {mode}")
        if mode == "RESET":
            for s in self.sliders.values(): s.set(2048)

    def update_ui_loop(self):
        for i in range(1, DISPLAY_SERVO_COUNT + 1):
            if i in servo_states:
                s = servo_states[i]
                l = self.fb_labels[i]
                l[0].config(text=str(s['pos']))
                l[1].config(text=str(s['speed']))
                l[2].config(text=str(s['load']))
                
                # 负载变色
                l[2].config(bg="red" if abs(s['load']) > 500 else "white")
        
        self.after(50, self.update_ui_loop)

    def control_loop(self):
        if self.serial_mgr and running:
            data = []
            if CONTROL_MODE == "MANUAL":
                for i in range(1, DISPLAY_SERVO_COUNT + 1):
                    data.append((i, self.sliders[i].get(), 0, 0))
            elif CONTROL_MODE == "SINE":
                t = time.time() * 2
                for i in range(1, DISPLAY_SERVO_COUNT + 1):
                    pos = 2048 + 800 * math.sin(t + i*0.5)
                    data.append((i, int(pos), 0, 0))
                    self.sliders[i].set(int(pos))
            elif CONTROL_MODE == "RESET":
                for i in range(1, DISPLAY_SERVO_COUNT + 1):
                    data.append((i, 2048, 500, 50))
            
            self.serial_mgr.send_cmd(data)
            
        self.after(20, self.control_loop)

if __name__ == "__main__":
    app = MainApp()
    def on_exit():
        global running
        running = False
        app.destroy()
    app.protocol("WM_DELETE_WINDOW", on_exit)
    app.mainloop()