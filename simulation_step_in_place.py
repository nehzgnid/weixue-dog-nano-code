import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import math

# 复用现有的运动学库
from kinematics_v5 import LegKinematics, L1, L2, L3, OFFSET_HIP, OFFSET_KNEE
from robot_config import cfg

LENGTH = cfg.LENGTH
WIDTH = cfg.WIDTH
DEFAULT_HEIGHT = -180

class GaitController:
    def __init__(self):
        self.freq = 1.0 # Hz
        self.step_height = 40.0 # mm (抬腿高度)
        self.time = 0.0
        
        # Trot 步态相位偏移
        # FL(0)  FR(1)
        # RL(2)  RR(3)
        # Trot: FL & RR 同相 (0.0), FR & RL 同相 (0.5)
        self.offsets = {
            "FL": 0.0, "RR": 0.0,
            "FR": 0.5, "RL": 0.5
        }
        
        # 状态机：0=Stance(支撑), 1=Swing(摆动)
        self.leg_states = {k: 0 for k in self.offsets}
        self.gait_enabled = False

    def update(self, dt):
        if not self.gait_enabled:
            return
        self.time += dt

    def get_z_offset(self, leg_name):
        if not self.gait_enabled:
            return 0.0
            
        period = 1.0 / self.freq
        # 归一化时间 (0.0 ~ 1.0)
        global_phase = (self.time % period) / period
        
        # 计算该腿的相位
        leg_phase = (global_phase + self.offsets[leg_name]) % 1.0
        
        # 假设 0.0 ~ 0.5 为摆动相 (Swing)，0.5 ~ 1.0 为支撑相 (Stance)
        # 实际上为了平稳，摆动相可以占比小一点，比如 0.4
        swing_ratio = 0.5
        
        z_offset = 0.0
        
        if leg_phase < swing_ratio:
            # === Swing Phase (抬腿) ===
            self.leg_states[leg_name] = 1 # Swing
            
            # 局部相位 0.0 -> 1.0
            local_p = leg_phase / swing_ratio
            
            # 使用正弦波模拟抬腿轨迹 (半个周期)
            # sin(0)=0, sin(pi/2)=1, sin(pi)=0
            z_offset = self.step_height * math.sin(local_p * math.pi)
            
        else:
            # === Stance Phase (支撑) ===
            self.leg_states[leg_name] = 0 # Stance
            z_offset = 0.0
            
        return z_offset

class StepSimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("机器狗原地踏步仿真 (Gait Simulation)")
        self.geometry("1200x800")
        
        self.legs = {name: LegKinematics(name) for name in ["FL", "FR", "RL", "RR"]}
        self.gait = GaitController()
        
        self.sim_running = False
        self.last_time = time.perf_counter()
        
        # UI Layout
        left = tk.Frame(self, width=250, bg="#eee")
        left.pack(side="left", fill="y")
        right = tk.Frame(self, bg="white")
        right.pack(side="right", fill="both", expand=True)
        
        # Plot
        self.fig = plt.Figure(figsize=(5,4), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Controls
        tk.Label(left, text="Gait Parameters", font=("Arial", 12, "bold")).pack(pady=10)
        
        self.vars = {}
        self.add_slider(left, "Frequency (Hz)", 0.2, 3.0, 1.0, 0.1)
        self.add_slider(left, "Step Height (mm)", 0, 100, 40, 1.0)
        self.add_slider(left, "Body Height (mm)", -250, -100, -180, 1.0)
        
        tk.Label(left, text="Manual Controls", font=("Arial", 10, "bold")).pack(pady=5)
        self.add_slider(left, "Pitch (deg)", -20, 20, 0, 1.0)
        self.add_slider(left, "Roll (deg)", -20, 20, 0, 1.0)
        
        btn_frame = tk.Frame(left)
        btn_frame.pack(pady=20)
        self.btn_run = tk.Button(btn_frame, text="Start Walking", bg="#4CAF50", fg="white", font=("Arial", 12), command=self.toggle_walk)
        self.btn_run.pack(fill="x", padx=20)
        
        self.lbl_status = tk.Label(left, text="Status: IDLE", fg="#666")
        self.lbl_status.pack(pady=5)
        
        # Initial Plot
        self.update_plot()
        
        # Start Animation Loop
        self.animation_loop()

    def add_slider(self, p, name, min_v, max_v, val, res):
        f = tk.Frame(p, bg="#eee"); f.pack(fill="x", padx=10, pady=5)
        tk.Label(f, text=name, bg="#eee", width=15, anchor="w").pack(side="left")
        v = tk.DoubleVar(value=val)
        self.vars[name] = v
        tk.Scale(f, from_=min_v, to=max_v, orient="horizontal", variable=v, resolution=res, bg="#eee").pack(side="right", expand=True, fill="x")

    def toggle_walk(self):
        self.sim_running = not self.sim_running
        self.gait.gait_enabled = self.sim_running
        if self.sim_running:
            self.btn_run.config(text="Stop Walking", bg="#f44336")
            self.lbl_status.config(text="Status: WALKING (Trot)", fg="green")
        else:
            self.btn_run.config(text="Start Walking", bg="#4CAF50")
            self.lbl_status.config(text="Status: IDLE", fg="#666")

    def animation_loop(self):
        # 计算 dt
        current_time = time.perf_counter()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 更新参数
        self.gait.freq = self.vars["Frequency (Hz)"].get()
        self.gait.step_height = self.vars["Step Height (mm)"].get()
        
        # 更新步态
        self.gait.update(dt)
        
        # 更新绘图 (限制刷新率以免卡顿，比如每 3 帧刷一次？这里简化每帧刷)
        # 实际 Tkinter loop 受到 after 限制，这里设为 30ms (33fps)
        self.update_plot()
        self.after(30, self.animation_loop)

    def update_plot(self):
        self.ax.clear()
        
        # 设置坐标轴范围 (固定视角)
        self.ax.set_xlim(-200, 200); self.ax.set_ylim(-150, 150); self.ax.set_zlim(-300, 50)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        self.ax.set_box_aspect([1, 1, 0.8])
        
        # 基础参数
        base_h = self.vars["Body Height (mm)"].get()
        pitch = np.radians(self.vars["Pitch (deg)"].get())
        roll = np.radians(self.vars["Roll (deg)"].get())
        
        # 旋转矩阵 (简化版, 假设 Yaw=0)
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        # R = Ry(pitch) * Rx(roll)
        R = np.array([
            [cp, 0, sp],
            [sr*sp, cr, -sr*cp],
            [-cr*sp, sr, cr*cp]
        ]).T # 注意这里转置逻辑要根据定义来，这里先试一下
        
        # 足端与髋关节定义
        dx, dy = LENGTH/2, WIDTH/2
        origins = {"FL": [dx, dy, 0], "FR": [dx, -dy, 0], "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]}
        
        # 绘制
        # 1. 机身 (连接 Hip 原点)
        body_corners = []
        for name in ["FL", "FR", "RR", "RL"]:
            p = np.array(origins[name])
            # 机身旋转应用在 Hip 点上? 通常机身旋转是 Hip 相对于 Center 的旋转
            # 这里简单起见：Center 设为 (0,0,0) 不动，Hip 点跟随旋转
            p_rot = R @ p
            body_corners.append(p_rot)
        
        body_corners.append(body_corners[0]) # 闭合
        b_pts = np.array(body_corners)
        self.ax.plot(b_pts[:,0], b_pts[:,1], b_pts[:,2], 'k-', lw=2)
        
        # 2. 腿部
        for name, origin in origins.items():
            kin = self.legs[name]
            
            # === 核心逻辑: 计算目标足端位置 ===
            # Hip 坐标系下的目标 (x, y, z)
            # z 由 base_h (负值) + z_offset (正值) 组成
            z_lift = self.gait.get_z_offset(name)
            target_z = base_h + z_lift
            
            # 考虑机身姿态修正:
            # IK 的输入是 "足端相对于 Hip" 的坐标。
            # Hip 世界坐标 = R @ origin
            # Foot 世界坐标 = [origin.x, origin.y + side_sign*L1, target_z] (假设脚在肩膀正下方)
            # Foot Rel Hip = R.T @ (Foot_World - Hip_World)
            
            # 简化版：假设脚一直在 (origin.x, origin.y + offset, target_z) 垂直上下
            # 这样对于测试抬腿足够了
            foot_world = np.array([origin[0], origin[1] + kin.side_sign*L1, target_z])
            hip_world = R @ np.array(origin)
            
            foot_rel = R.T @ (foot_world - hip_world)
            
            # IK 解算
            res = kin.solve_ik(foot_rel[0], foot_rel[1], foot_rel[2])
            
            # 绘图颜色：Swing(摆动)时变红，Stance(支撑)时变蓝
            color = 'r' if self.gait.leg_states[name] == 1 else 'b'
            
            if res:
                q1, q2, q3 = res
                # FK 绘图
                p0, p1, p2, p3 = kin.forward_kinematics_strict(q1, q2, q3)
                
                # 转回世界坐标绘图
                pts = []
                for pt in [p0, p1, p2, p3]:
                    pts.append(hip_world + R @ pt)
                pts = np.array(pts)
                
                self.ax.plot(pts[:,0], pts[:,1], pts[:,2], f'{color}-o', lw=2)
                
                # 画脚下的点 (影子)
                self.ax.plot([pts[-1,0]], [pts[-1,1]], [-200], 'k.', alpha=0.2)
            else:
                # IK 失败
                self.ax.text(origin[0], origin[1], 0, "IK FAIL", color='red')

        self.canvas.draw()

if __name__ == "__main__":
    app = StepSimApp()
    app.mainloop()