import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# 引用最新的配置系统
from kinematics_v5 import LegKinematics, L1, L2, L3, OFFSET_HIP, OFFSET_KNEE
from robot_config import cfg

LENGTH = cfg.LENGTH
WIDTH = cfg.WIDTH

class SimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("机器狗 IK 仿真 V5 (物理极性对齐版)")
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
        self.add_slider(left, "Height", -30, -250, -180)
        self.add_slider(left, "Pitch", -30, 30, 0)
        self.add_slider(left, "Roll", -30, 30, 0)
        self.add_slider(left, "Yaw", -30, 30, 0)
        self.add_slider(left, "Shift X", -50, 50, 0)
        self.add_slider(left, "Shift Y", -50, 50, 0)
        
        btn_f = tk.Frame(left); btn_f.pack(pady=10)
        tk.Button(btn_f, text="Reset Pose", command=self.reset).pack(side="left", padx=5)
        tk.Button(btn_f, text="Zero All (2048)", command=self.toggle_zero, bg="#ffcccb").pack(side="left", padx=5)
        self.btn_copy = tk.Button(btn_f, text="Copy PWM", command=self.copy_pwm, bg="#cce5ff")
        self.btn_copy.pack(side="left", padx=5)
        
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

    def copy_pwm(self):
        """收集当前姿态的 PWM 并复制到剪贴板"""
        import json
        
        # 重新计算一遍或者从 label 获取？
        # 为了精确，建议从 update_plot 的计算结果获取，但 update_plot 没有返回值
        # 我们这里简单从 label 获取，虽然 dirty 但有效
        
        data = {}
        mapping = {
            "FL": [1, 2, 3], "FR": [4, 5, 6],
            "RL": [7, 8, 9], "RR": [10, 11, 12]
        }
        
        try:
            for leg, ids in mapping.items():
                # abd, hip, knee
                data[ids[0]] = int(self.lbls[leg][0].cget("text").replace("!", ""))
                data[ids[1]] = int(self.lbls[leg][1].cget("text").replace("!", ""))
                data[ids[2]] = int(self.lbls[leg][2].cget("text").replace("!", ""))
            
            json_str = json.dumps(data)
            self.clipboard_clear()
            self.clipboard_append(json_str)
            self.update() # 必须调用一次 update 才能生效
            
            # 反馈
            orig_text = self.btn_copy.cget("text")
            self.btn_copy.config(text="Copied!", bg="#ccffcc")
            self.after(1000, lambda: self.btn_copy.config(text=orig_text, bg="#cce5ff"))
            
        except Exception as e:
            print(f"Copy failed: {e}")

    def update_plot(self):
        # 预计算所有腿的 IK 解和 PWM，检查是否越界
        # 如果有任何一个越界，则不更新绘图，以此模拟"卡死保护"
        
        # 1. 准备数据
        h = self.vars["Height"].get()
        bx, by = self.vars["Shift X"].get(), self.vars["Shift Y"].get()
        r, p, y_ang = [np.radians(self.vars[k].get()) for k in ["Roll", "Pitch", "Yaw"]]
        
        cr, sr = np.cos(r), np.sin(r); cp, sp = np.cos(p), np.sin(p); cy, sy = np.cos(y_ang), np.sin(y_ang)
        R = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
        
        dx, dy = LENGTH/2, WIDTH/2
        origins = {"FL": [dx, dy, 0], "FR": [dx, -dy, 0], "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]}
        
        # 临时存储计算结果
        results = {}
        global_valid = True
        
        for name, origin in origins.items():
            kin = self.legs[name]
            
            if self.forced_zero:
                # 归中模式总是合法的
                results[name] = {"type": "zero"}
                continue
                
            foot_world = np.array([origin[0], origin[1] + kin.side_sign*L1, h])
            hip_world = R @ np.array(origin) + np.array([bx, by, 0])
            foot_rel = R.T @ (foot_world - hip_world)
            
            res = kin.solve_ik(foot_rel[0], foot_rel[1], foot_rel[2])
            
            if res:
                q1, q2, q3 = res
                # 调用无截断的原始计算 (kinematics_v5里我们已经加了clip，这里需要临时移除或直接判断)
                # 实际上 kin.rad_to_pwm 已经 clip 了。我们需要知道是否 *被* clip 了。
                # 简单起见，我们重新手动算一遍 raw 值来检查
                
                raw_pwm_abd = 2048 + (q1 - 0) * kin.rad_to_step * kin.dir_abd
                raw_pwm_hip = 2048 + (q2 - OFFSET_HIP) * kin.rad_to_step * kin.dir_hip
                raw_pwm_knee = 2048 + (q3 - OFFSET_KNEE) * kin.rad_to_step * kin.dir_knee
                print(f"offsets: hip {OFFSET_HIP * 180/np.pi}, knee {OFFSET_KNEE * 180/np.pi}")
                pwms = [int(raw_pwm_abd), int(raw_pwm_hip), int(raw_pwm_knee)]
                limits = [(1800, 2300), (1200, 2900), (850, 3250)]
                
                is_leg_valid = True
                for val, (low, high) in zip(pwms, limits):
                    if val < low or val > high:
                        is_leg_valid = False
                        global_valid = False
                
                results[name] = {
                    "type": "ik",
                    "q": (q1, q2, q3),
                    "pwm": pwms,
                    "valid": is_leg_valid
                }
            else:
                results[name] = {"type": "fail"}
                global_valid = False

        # 2. 只有当 global_valid 为 True (或者 forced_zero) 时，才更新画面
        # 否则只更新报错信息
        
        self.ax.clear()
        self.ax.set_box_aspect([1,1,1]) 
        self.ax.set_xlim(-200, 200); self.ax.set_ylim(-150, 150); self.ax.set_zlim(-350, 50)
        self.ax.set_xlabel('X (Front)'); self.ax.set_ylabel('Y (Left)'); self.ax.set_zlabel('Z (Up)')
        
        # 即使卡死，也画出当前的姿态(上一帧)？不，清空了就没了。
        # 这里的逻辑是：如果卡死，就不画新的，保持旧的？matplotlib做不到不画新的保持旧的(除非不clear)。
        # 妥协方案：如果卡死，画出"红色"的非法构型，提醒用户。
        
        # 绘制机身
        body_pts = [R @ np.array(origins[l]) + np.array([bx, by, 0]) for l in ["FL", "FR", "RR", "RL", "FL"]]
        body_pts = np.array(body_pts)
        style = 'k-' if global_valid else 'r--' # 非法时变红虚线
        self.ax.plot(body_pts[:,0], body_pts[:,1], body_pts[:,2], style, lw=2)
        
        self.ax.plot([bx], [by], [0], 'ro', markersize=5)
        self.ax.plot([bx, bx], [by, by], [0, h], 'r--', alpha=0.3)

        for name, origin in origins.items():
            res = results[name]
            
            if res["type"] == "zero":
                # 归中
                for i in range(3): self.lbls[name][i].config(text="2048", fg="blue")
                # 画归中图...
                
            elif res["type"] == "fail":
                for i in range(3): self.lbls[name][i].config(text="ERR", fg="red")
                
            elif res["type"] == "ik":
                pwms = res["pwm"]
                valid = res["valid"]
                
                # 更新标签
                for i in range(3):
                    val = pwms[i]
                    # 从配置读取限幅
                    limits = [cfg.LIMITS["abd"], cfg.LIMITS["hip"], cfg.LIMITS["knee"]]
                    low, high = limits[i]
                    
                    if val < low or val > high:
                        self.lbls[name][i].config(text=f"{val}!", fg="red", bg="#ffff00") # 越界高亮
                    else:
                        if global_valid:
                            self.lbls[name][i].config(text=str(val), fg="black", bg="white")
                        else:
                            # 即使这个关节没越界，但因为别的关节越界了，所以全身锁死，显示灰色
                            self.lbls[name][i].config(text=str(val), fg="gray", bg="white")

                # FK 绘图
                q1, q2, q3 = res["q"]
                p0, p1, p2, p3 = self.legs[name].forward_kinematics_strict(q1, q2, q3)
                
                pts = []
                hip_world_pos = R @ np.array(origin) + np.array([bx, by, 0])
                for pt in [p0, p1, p2, p3]:
                    pts.append(hip_world_pos + R @ pt)
                pts = np.array(pts)
                
                col = 'b' if "L" in name else 'r'
                if not valid: col = 'm' # 单腿越界变洋红色
                self.ax.plot(pts[:,0], pts[:,1], pts[:,2], f'{col}-o', lw=2)

        self.canvas.draw()

if __name__ == "__main__":
    app = SimApp()
    app.mainloop()