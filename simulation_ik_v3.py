import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from kinematics_v3 import LegKinematics, L1, L2, L3

# 机身参数
LENGTH = 263.7
WIDTH = 49.0

class SimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("机器狗 IK 仿真 V4 (自平衡 + 无畸变)")
        self.geometry("1200x800")
        
        self.legs = {
            "FL": LegKinematics("FL"),
            "FR": LegKinematics("FR"),
            "RL": LegKinematics("RL"),
            "RR": LegKinematics("RR")
        }
        
        # UI Setup
        left = tk.Frame(self, width=300, bg="#eee")
        left.pack(side="left", fill="y")
        right = tk.Frame(self, bg="white")
        right.pack(side="right", fill="both", expand=True)
        
        # Sliders
        self.vars = {}
        self.add_slider(left, "Height", -100, -250, -180)
        self.add_slider(left, "Pitch", -30, 30, 0)
        self.add_slider(left, "Roll", -30, 30, 0)
        self.add_slider(left, "Yaw", -30, 30, 0)
        
        # === 功能按钮 ===
        btn_frame = tk.Frame(left, bg="#eee")
        btn_frame.pack(pady=10)
        
        tk.Button(btn_frame, text="Reset Pose", command=self.reset, width=12).pack(side="left", padx=5)
        # 归中: 强制所有舵机回 2048 (不走 IK)
        tk.Button(btn_frame, text="Zero All (2048)", command=self.zero_all, width=12, bg="#ffcccb").pack(side="left", padx=5)
        
        # PWM Display
        self.lbls = {}
        grid = tk.Frame(left); grid.pack(pady=20)
        for i, leg in enumerate(self.legs):
            tk.Label(grid, text=leg).grid(row=i, column=0)
            self.lbls[leg] = []
            for j in range(3):
                l = tk.Label(grid, text="0000", width=6, bg="white", relief="sunken")
                l.grid(row=i, column=j+1, padx=2)
                self.lbls[leg].append(l)

        # Plot
        self.fig = plt.Figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        self.update_plot()

    def add_slider(self, p, name, min_v, max_v, val):
        f = tk.Frame(p); f.pack(fill="x", padx=5)
        tk.Label(f, text=name, width=6).pack(side="left")
        v = tk.DoubleVar(value=val)
        self.vars[name] = v
        tk.Scale(f, from_=min_v, to=max_v, orient="horizontal", variable=v, command=lambda x: self.update_plot()).pack(fill="x")

    def reset(self):
        self.vars["Height"].set(-180)
        self.vars["Roll"].set(0)
        self.vars["Pitch"].set(0)
        self.vars["Yaw"].set(0)
        self.update_plot()

    def zero_all(self):
        """强制显示归中状态"""
        for leg in self.legs:
            for i in range(3):
                self.lbls[leg][i].config(text="2048", fg="blue")
        # 此时 IK 绘图可能会乱，因为 2048 对应特定的物理角度，我们可以反推 FK 来画图
        # 但为了简单，这里只更新数字，不更新图，或者画一个默认图
        self.ax.clear()
        self.ax.text2D(0.5, 0.5, "ALL SERVOS CENTERED (2048)", transform=self.ax.transAxes, ha="center")
        self.canvas.draw()

    def update_plot(self):
        self.ax.clear()
        
        # === 关键修正: 强制等比例 XYZ，防止 Z 轴被压缩 ===
        self.ax.set_box_aspect([1, 1, 1]) 
        self.ax.set_xlim(-200, 200); self.ax.set_ylim(-150, 150); self.ax.set_zlim(-300, 100)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        
        h = self.vars["Height"].get()
        r, p, y = [np.radians(self.vars[k].get()) for k in ["Roll", "Pitch", "Yaw"]]
        
        # RPY Matrix
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        R = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                      [-sp, cp*sr, cp*cr]])
        
        # Origins
        dx, dy = LENGTH/2, WIDTH/2
        origins = {
            "FL": [dx, dy, 0], "FR": [dx, -dy, 0],
            "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]
        }
        
        # === 重心自平衡逻辑 ===
        # 我们假设足端不动，机身绕重心旋转。
        # 但在 IK 中，我们输入的是足端相对于机身的位置。
        # 如果机身只是旋转，足端在世界坐标系不动，那么重心投影自然保持不变。
        # 所以这里的关键是: 计算 Foot_World 时，要确保它们相对于重心的 X,Y 对称。
        
        # 默认足端位置 (世界坐标): 此时重心 (0,0) 在四脚中心
        # FL: [dx, dy+L1, h], FR: [dx, -dy-L1, h] ...
        
        # 绘制机身
        body_pts = []
        for leg in ["FL", "FR", "RR", "RL", "FL"]:
            # 机身点 = R * 原点
            body_pts.append(R @ np.array(origins[leg]))
        body_pts = np.array(body_pts)
        self.ax.plot(body_pts[:,0], body_pts[:,1], body_pts[:,2], 'k-', lw=3)
        
        # 绘制重心投影 (红点)
        self.ax.plot([0], [0], [0], 'ro', markersize=8, label='CoM')
        self.ax.plot([0,0], [0,0], [0, h], 'r--', lw=1) # 垂线
        
        # Calc Legs
        for name, origin in origins.items():
            kin = self.legs[name]
            
            # 足端目标 (World Frame)
            # 始终保持在四角，不动，这样重心就永远在正中间
            foot_world = np.array([origin[0], origin[1] + kin.side * L1, h])
            
            # Transform to Hip Frame for IK
            # Vector = R.T @ (Foot_World - Body_Pos) - Hip_Origin
            # Body_Pos is (0,0,0) because we rotate around CoM
            foot_rel = R.T @ foot_world - np.array(origin)
            
            # Solve IK
            res = kin.solve_ik(foot_rel[0], foot_rel[1], foot_rel[2])
            
            if res:
                q1, q2, q3 = res
                
                # PWM
                pwms = kin.calc_pwm(q1, q2, q3)
                for k in range(3):
                    self.lbls[name][k].config(text=str(pwms[k]), fg="black")
                
                # FK for Plotting
                p0, p1, p2, p3 = kin.forward_kinematics(q1, q2, q3)
                
                # Transform FK points to World Frame
                pts = []
                for pt in [p0, p1, p2, p3]:
                    pts.append(R @ (np.array(origin) + pt))
                pts = np.array(pts)
                
                col = 'b' if "L" in name else 'r'
                self.ax.plot(pts[:,0], pts[:,1], pts[:,2], f'{col}-o', lw=2)
            else:
                for k in range(3): self.lbls[name][k].config(text="ERR", fg="red")
        
        self.canvas.draw()

if __name__ == "__main__":
    app = SimApp()
    app.mainloop()