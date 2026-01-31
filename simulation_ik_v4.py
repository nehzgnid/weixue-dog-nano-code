import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from kinematics_v5 import LegKinematics, L1, L2, L3, OFFSET_HIP, OFFSET_KNEE

LENGTH = 263.7
WIDTH = 49.0

class SimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("机器狗 IK 仿真 V4 (严格刚体版)")
        self.geometry("1200x800")
        
        self.legs = {name: LegKinematics(name) for name in ["FL", "FR", "RL", "RR"]}
        self.forced_zero = False
        
        # UI
        left = tk.Frame(self, width=300, bg="#eee")
        left.pack(side="left", fill="y")
        right = tk.Frame(self, bg="white")
        right.pack(side="right", fill="both", expand=True)
        
        # Controls
        self.vars = {}
        self.add_slider(left, "Height", -100, -250, -180)
        self.add_slider(left, "Pitch", -30, 30, 0)
        self.add_slider(left, "Roll", -30, 30, 0)
        self.add_slider(left, "Yaw", -30, 30, 0)
        self.add_slider(left, "Shift X", -50, 50, 0)
        self.add_slider(left, "Shift Y", -50, 50, 0)
        
        btn_f = tk.Frame(left); btn_f.pack(pady=10)
        tk.Button(btn_f, text="Reset Pose", command=self.reset).pack(side="left", padx=5)
        tk.Button(btn_f, text="Zero All (2048)", command=self.toggle_zero, bg="#ffcccb").pack(side="left", padx=5)
        
        # PWM
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
        self.forced_zero = False
        self.vars["Height"].set(-180)
        for k in ["Pitch","Roll","Yaw","Shift X","Shift Y"]: self.vars[k].set(0)
        self.update_plot()

    def toggle_zero(self):
        self.forced_zero = not self.forced_zero
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_box_aspect([1,1,1]) # 绝对等比例
        self.ax.set_xlim(-200, 200); self.ax.set_ylim(-150, 150); self.ax.set_zlim(-300, 100)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        
        # 机身变换
        h = self.vars["Height"].get()
        bx, by = self.vars["Shift X"].get(), self.vars["Shift Y"].get()
        r, p, y = [np.radians(self.vars[k].get()) for k in ["Roll", "Pitch", "Yaw"]]
        
        cr, sr = np.cos(r), np.sin(r); cp, sp = np.cos(p), np.sin(p); cy, sy = np.cos(y), np.sin(y)
        R = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
        
        dx, dy = LENGTH/2, WIDTH/2
        origins = {"FL": [dx, dy, 0], "FR": [dx, -dy, 0], "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]}
        
        # 绘制机身
        body_pts = [R @ np.array(origins[l]) for l in origins]
        body_pts.append(body_pts[0]) # 闭合
        body_pts = np.array(body_pts)
        # 加上平移偏移 (Body Shift)
        # 注意: 这里的 Shift 是移动 CoM
        body_pts += np.array([bx, by, 0])
        self.ax.plot(body_pts[:,0], body_pts[:,1], body_pts[:,2], 'k-', lw=2)
        
        # 计算每条腿
        for name, origin in origins.items():
            kin = self.legs[name]
            
            if self.forced_zero:
                # 强制归中模式: PWM = 2048
                # 对应的物理角度:
                # Abd = 0
                # Hip = OFFSET_HIP
                # Knee = OFFSET_KNEE
                # 这里的正负号取决于安装方向，我们先假设正向
                q1, q2, q3 = 0, OFFSET_HIP, OFFSET_KNEE
                
                # 更新显示
                for i in range(3): self.lbls[name][i].config(text="2048", fg="blue")
                
            else:
                # IK 模式
                # 足端目标: 世界坐标系下固定不动
                foot_world = np.array([origin[0], origin[1] + kin.side_sign*L1, h])
                
                # 转换到 Hip 局部坐标系
                # P_rel = R.T @ (Foot_World - (Body_Pos + R@Hip_Origin))
                hip_world = R @ np.array(origin) + np.array([bx, by, 0])
                foot_rel = R.T @ (foot_world - hip_world)
                
                res = kin.solve_ik(foot_rel[0], foot_rel[1], foot_rel[2])
                if res:
                    q1, q2, q3 = res
                    pwm = kin.rad_to_pwm(q1, q2, q3)
                    for i in range(3): self.lbls[name][i].config(text=str(pwm[i]), fg="black")
                else:
                    q1, q2, q3 = 0, 0, 0
                    for i in range(3): self.lbls[name][i].config(text="ERR", fg="red")

            # FK 绘图 (共用)
            p0, p1, p2, p3 = kin.forward_kinematics_strict(q1, q2, q3)
            
            # 转世界坐标
            pts = []
            hip_world_pos = R @ np.array(origin) + np.array([bx, by, 0])
            
            for pt in [p0, p1, p2, p3]:
                # pt 是局部向量，需要随身体旋转 R
                pts.append(hip_world_pos + R @ pt)
            pts = np.array(pts)
            
            col = 'b' if "L" in name else 'r'
            self.ax.plot(pts[:,0], pts[:,1], pts[:,2], f'{col}-o', lw=2)

        self.canvas.draw()

if __name__ == "__main__":
    app = SimApp()
    app.mainloop()
