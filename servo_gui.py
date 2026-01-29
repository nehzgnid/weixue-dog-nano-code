import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import time

# === 配置 ===
BAUD_RATE = 115200

# === 协议定义 ===
FEEDBACK_SIZE = 39
FB_HEAD_1 = 0xA5
FB_HEAD_2 = 0x5B

# 全局数据 (存储最新的舵机状态)
servo_states = {i: {'pos': 0, 'speed': 0, 'load': 0} for i in range(1, 7)}
running = True

class SerialReader(threading.Thread):
    """专门负责 100Hz 数据解析的后台线程"""
    def __init__(self):
        super().__init__()
        self.port = self.find_port()
        self.ser = None
        if self.port:
            try:
                self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.1)
                print(f"[成功] 已连接到: {self.port}")
            except Exception as e:
                print(f"[错误] 串口打开失败: {e}")

    def find_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if "STMicroelectronics" in p.description or "USB Serial" in p.description:
                return p.device
        return ports[-1].device if ports else None

    def run(self):
        global servo_states, running
        if not self.ser: return
        buffer = bytearray()
        
        while running:
            try:
                if self.ser.in_waiting > 0:
                    buffer.extend(self.ser.read(self.ser.in_waiting))
                
                while len(buffer) >= FEEDBACK_SIZE:
                    if buffer[0] == FB_HEAD_1 and buffer[1] == FB_HEAD_2:
                        packet = buffer[:FEEDBACK_SIZE]
                        if (sum(packet[:-1]) & 0xFF) == packet[-1]:
                            self.parse_packet(packet)
                            buffer = buffer[FEEDBACK_SIZE:]
                        else:
                            buffer = buffer[1:]
                    else:
                        buffer = buffer[1:]
                time.sleep(0.005)
            except: break

    def parse_packet(self, packet):
        idx = 2
        for i in range(1, 7):
            # 解析位置、速度、负载 (16位有符号)
            pos = int.from_bytes(packet[idx:idx+2], 'little', signed=True)
            speed = int.from_bytes(packet[idx+2:idx+4], 'little', signed=True)
            load = int.from_bytes(packet[idx+4:idx+6], 'little', signed=True)
            idx += 6
            servo_states[i] = {'pos': pos, 'speed': speed, 'load': load}

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("轮足机器人舵机状态监控")
        self.geometry("700x450")
        self.widgets = {}
        self.setup_ui()
        self.update_loop()

    def setup_ui(self):
        # 标题行
        titles = ["ID", "实时位置 (0-4095)", "速度", "负载/电流"]
        for i, t in enumerate(titles):
            tk.Label(self, text=t, font=('微软雅黑', 10, 'bold')).grid(row=0, column=i, padx=20, pady=10)

        for i in range(1, 7):
            # ID
            tk.Label(self, text=f"舵机 {i}", font=('微软雅黑', 10)).grid(row=i, column=0)
            
            # 位置条
            frame = tk.Frame(self)
            frame.grid(row=i, column=1, padx=10)
            pb = ttk.Progressbar(frame, length=200, maximum=4095)
            pb.pack(side="left")
            l_pos = tk.Label(frame, text="0", width=6)
            l_pos.pack(side="left")
            
            # 速度
            l_spd = tk.Label(self, text="0", width=10)
            l_spd.grid(row=i, column=2)
            
            # 负载
            l_load = tk.Label(self, text="0", width=10)
            l_load.grid(row=i, column=3)
            
            self.widgets[i] = {'pb': pb, 'pos': l_pos, 'spd': l_spd, 'load': l_load}

    def update_loop(self):
        for i in range(1, 7):
            s = servo_states[i]
            self.widgets[i]['pb']['value'] = s['pos']
            self.widgets[i]['pos'].config(text=str(s['pos']))
            self.widgets[i]['spd'].config(text=str(s['speed']))
            self.widgets[i]['load'].config(text=str(s['load']))
            # 负载过大变红
            self.widgets[i]['load'].config(fg="red" if abs(s['load']) > 400 else "black")
        
        self.after(50, self.update_loop) # 20Hz 刷新界面

if __name__ == "__main__":
    reader = SerialReader()
    reader.start()
    app = App()
    def on_exit():
        global running; running = False
        app.destroy()
    app.protocol("WM_DELETE_WINDOW", on_exit)
    app.mainloop()