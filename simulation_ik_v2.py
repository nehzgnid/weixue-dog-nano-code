import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D

# === 机器人几何参数 (mm) ===
LENGTH = 263.7  # 前后髋关节距离
WIDTH = 49.0    # 左右髋关节距离
L1 = 63.0       # 侧摆臂长 (Abduction Link)
L2 = 100.0      # 大腿长 (Thigh Link)
L3 = 117.0      # 小腿长 (Shank Link)

# === 零点偏置 (度) ===
# 定义：当 PWM=2048 时，关节相对于理论“直线/垂直”状态的物理夹角
# 注意：这里需要根据您的实际安装方向仔细确认正负号
OFFSET_HIP = 8.6   
OFFSET_KNEE = 15.3 

# === 舵机限幅 ===
LIMITS = {
    "abd": (1800, 2300),
    "hip": (1200, 2900),
    "knee": (1450, 2650)
}

class RobotIK:
    def __init__(self):
        self.rad_to_step = 4096 / (2 * np.pi)

    def inverse_kinematics(self, x, y, z, is_left):
        """
        输入: 足端相对于髋关节(侧摆轴心)的坐标 (x, y, z)
        输出: 关节角度 (abd, hip, knee) 弧度
        """
        # 1. 侧摆角 (Theta 1)
        # 目标: 将足端投影到侧摆旋转平面 (YZ平面)
        # 我们希望腿向外张开。
        # 对于左腿 (y>0)，我们希望 y 保持正。
        # 对于右腿 (y<0)，我们希望 y 保持负。
        
        # d 是侧摆轴心到足端的直线距离在 YZ 平面的投影
        d = np.sqrt(y**2 + z**2)
        if d < L1: return None # 构型不可达(腿太短构不成三角形)
        
        # alpha 是目标向量的角度 (从 -Z 轴开始算的极角?)
        # 这里的数学推导基于: z 是负的 (向下)，y 是外侧
        
        # 侧摆产生的有效长度 (从侧摆轴到脚底的垂直距离)
        # l_eff = sqrt(d^2 - L1^2)
        l_eff = np.sqrt(d**2 - L1**2)
        
        # theta_d 是 (y, z) 向量的角度
        theta_d = np.arctan2(y, -z) 
        
        # theta_off 是 L1 造成的角度偏移
        theta_off = np.arctan2(L1, l_eff)
        
        # 计算侧摆角
        if is_left:
            # 左腿：y 是正的。
            # 如果腿垂直向下，y=L1, theta_d = atan2(L1, -z) = theta_off.
            # 此时 abd 应该为 0.
            theta_abd = theta_d - theta_off
        else:
            # 右腿：y 是负的。
            # 如果腿垂直向下，y=-L1, theta_d = atan2(-L1, -z) = -theta_off.
            # 此时 abd 应该为 0.
            theta_abd = theta_d + theta_off

        # 2. 腿部平面 IK (2-Link)
        # 将问题转换到大腿旋转平面 (X-Z' 平面)
        # z_prime 是在这个旋转平面内的深度 (即 l_eff)
        # 注意：这里 z_prime 取负值，表示在髋关节下方
        z_prime = -l_eff
        
        # 目标点到髋关节轴心(大腿轴)的距离
        r_sq = x**2 + z_prime**2
        r = np.sqrt(r_sq)
        
        if r > (L2 + L3) or r < abs(L2 - L3): return None # 超出航程

        # 3. 大腿 (Theta 2) 和 小腿 (Theta 3)
        # phi 是目标向量的角度 (相对于 -Z')
        phi = np.arctan2(x, -z_prime)
        
        # 余弦定理计算内角
        # c_hip 是大腿与目标向量的夹角
        c_hip = (L2**2 + r_sq - L3**2) / (2 * L2 * r)
        angle_hip_internal = np.arccos(np.clip(c_hip, -1, 1))
        
        theta_hip = phi + angle_hip_internal
        
        # c_knee 是膝关节内角 (大腿和小腿夹角)
        c_knee = (L2**2 + L3**2 - r_sq) / (2 * L2 * L3)
        angle_knee_internal = np.arccos(np.clip(c_knee, -1, 1))
        
        # 我们的 knee 舵机角度通常定义为 0=折叠 or 0=伸直?
        # 假设 0 度是伸直 (和大腿共线) -> 那么我们需要转 PI - internal
        theta_knee = np.pi - angle_knee_internal

        return theta_abd, theta_hip, theta_knee

    def forward_kinematics(self, abd, hip, knee, is_left):
        """
        正运动学: 用于画图验证
        返回: [P0, P1, P2, P3] 关节点坐标
        """
        side = 1 if is_left else -1
        
        # P0: 髋关节原点 (0,0,0)
        p0 = np.array([0, 0, 0])
        
        # P1: 大腿根部 (Abduction Link 末端)
        # 绕 X 轴旋转。0度时，link 平行于 Y 轴 (水平)
        # 左腿: [0, L1, 0]. 右腿: [0, -L1, 0]
        # 旋转矩阵 Rx(theta)
        c1, s1 = np.cos(abd), np.sin(abd)
        
        # 初始向量 (未旋转前)
        v1_init = np.array([0, side * L1, 0])
        
        # 旋转后的 P1
        # RotX = [[1,0,0],[0,c,-s],[0,s,c]]
        p1 = np.array([
            0,
            v1_init[1] * c1,
            v1_init[1] * s1
        ])
        
        # 在腿部局部平面内计算 P2(膝) 和 P3(脚)
        # 局部平面: X-Z' (垂直于 Abduction Link)
        # 定义: hip=0 垂直向下 (-Z'), knee=0 伸直
        
        # 局部向量 (大腿)
        v2_loc = np.array([
            L2 * np.sin(hip),
            0, # Y 分量在局部平面为 0
            -L2 * np.cos(hip)
        ])
        
        # 局部向量 (小腿) - 膝关节角度叠加
        v3_loc = np.array([
            L3 * np.sin(hip + knee),
            0,
            -L3 * np.cos(hip + knee)
        ])
        
        # 将局部向量旋转回世界坐标 (绕 X 轴转 abd)
        # 注意: 局部平面的 Z' 轴 其实就是 旋转后的 Z 轴方向? 
        # 不完全是。局部平面的 Y 轴是指向 link 方向的。
        
        def rot_x(v, angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([v[0], v[1]*c - v[2]*s, v[1]*s + v[2]*c])

        # P2 = P1 + RotX(v2_loc)
        p2 = p1 + rot_x(v2_loc, abd)
        
        # P3 = P1 + RotX(v3_loc) (注意 v3_loc 是相对于 P1 的矢量)
        p3 = p1 + rot_x(v3_loc, abd)
        
        return p0, p1, p2, p3

    def calc_pwm(self, abd, hip, knee):
        """计算 PWM 值"""
        # 这里需要根据您的实际舵机方向微调符号 (+/-)
        # 假设:
        # abd: 2048 = 水平
        pwm_abd = 2048 + abd * self.rad_to_step
        
        # hip: 2048 = 向前偏 8.6 度
        # IK 算出的是 hip (0=垂直向下)
        # 如果 hip=0 (垂直), 舵机应该往“后”转 8.6 度才能归中?
        # 也就是: 目标 0 度 -> 需要 PWM 对应 -8.6 度
        pwm_hip = 2048 + (hip - np.radians(OFFSET_HIP)) * self.rad_to_step
        
        # knee: 2048 = 向前偏 15.3 度
        # IK 算出的是 knee (0=伸直)
        # 小腿通常是反向安装 (数值增大 = 弯曲)
        pwm_knee = 2048 - (knee - np.radians(OFFSET_KNEE)) * self.rad_to_step
        
        return int(pwm_abd), int(pwm_hip), int(pwm_knee)

class SimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("机器狗运动学仿真 V2 (参数修正版)")
        self.geometry("1200x800")
        self.ik = RobotIK()
        
        # === UI ===
        left = tk.Frame(self, width=300, bg="#eee")
        left.pack(side="left", fill="y")
        right = tk.Frame(self, bg="white")
        right.pack(side="right", fill="both", expand=True)
        
        # 滑块
        self.vars = {}
        self.add_slider(left, "Height", -100, -250, -180)
        self.add_slider(left, "Body X", -50, 50, 0)
        self.add_slider(left, "Body Y", -50, 50, 0)
        self.add_slider(left, "Roll", -20, 20, 0)
        self.add_slider(left, "Pitch", -20, 20, 0)
        self.add_slider(left, "Yaw", -20, 20, 0)
        
        tk.Button(left, text="Reset", command=self.reset).pack(pady=10)
        
        # 数据监视
        self.lbls = {}
        grid = tk.Frame(left)
        grid.pack(pady=10)
        for i, leg in enumerate(["FL", "FR", "RL", "RR"]):
            tk.Label(grid, text=leg).grid(row=i, column=0)
            self.lbls[leg] = []
            for j in range(3):
                l = tk.Label(grid, text="0000", width=5, bg="white", relief="sunken")
                l.grid(row=i, column=j+1)
                self.lbls[leg].append(l)

        # 绘图
        self.fig = plt.Figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        self.update_plot()

    def add_slider(self, parent, name, min_v, max_v, val):
        f = tk.Frame(parent)
        f.pack(fill="x", padx=5)
        tk.Label(f, text=name, width=8).pack(side="left")
        v = tk.DoubleVar(value=val)
        self.vars[name] = v
        tk.Scale(f, from_=min_v, to=max_v, orient="horizontal", variable=v, command=lambda x: self.update_plot()).pack(fill="x")

    def reset(self):
        self.vars["Height"].set(-180)
        self.vars["Body X"].set(0)
        self.vars["Body Y"].set(0)
        self.vars["Roll"].set(0)
        self.vars["Pitch"].set(0)
        self.vars["Yaw"].set(0)
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(-200, 200); self.ax.set_ylim(-150, 150); self.ax.set_zlim(-300, 50)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        
        h = self.vars["Height"].get()
        bx, by = self.vars["Body X"].get(), self.vars["Body Y"].get()
        r, p, y_ang = [np.radians(self.vars[k].get()) for k in ["Roll", "Pitch", "Yaw"]]
        
        # 旋转矩阵 (Body -> World)
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y_ang), np.sin(y_ang)
        Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        R = Rz @ Ry @ Rx
        
        # 髋关节原点 (Body Frame)
        dx, dy = LENGTH/2, WIDTH/2
        origins = {
            "FL": [dx, dy, 0], "FR": [dx, -dy, 0],
            "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]
        }
        
        # 绘制机身
        body_pts = []
        for leg in ["FL", "FR", "RR", "RL", "FL"]:
            pt = np.array(origins[leg])
            # 机身旋转 + 平移
            pt_world = R @ pt + np.array([bx, by, 0])
            body_pts.append(pt_world)
        body_pts = np.array(body_pts)
        self.ax.plot(body_pts[:,0], body_pts[:,1], body_pts[:,2], 'k-', lw=2)
        
        # 计算每条腿
        for leg, origin in origins.items():
            is_left = (origin[1] > 0)
            
            # 足端目标 (World Frame)
            # 默认脚在髋关节正侧方下方: [x, y +/- L1, h]
            foot_x = origin[0]
            foot_y = origin[1] + (L1 if is_left else -L1)
            foot_z = h # 地面高度
            foot_world = np.array([foot_x, foot_y, foot_z])
            
            # 转换回 Body Frame (相对于髋关节)
            # P_rel = R.T @ (P_world - Body_Pos) - Hip_Pos_Body
            hip_pos_world = R @ np.array(origin) + np.array([bx, by, 0])
            foot_rel = R.T @ (foot_world - hip_pos_world) # 这一步简化了，直接算相对髋的向量
            
            # 更精确的: Foot_Rel_Hip = R.T @ (Foot_World - (Body_Pos + R@Hip_Body))
            foot_rel = R.T @ (foot_world - np.array([bx, by, 0])) - np.array(origin)
            
            # IK 解算
            res = self.ik.inverse_kinematics(foot_rel[0], foot_rel[1], foot_rel[2], is_left)
            
            if res:
                abd, hip, knee = res
                
                # PWM
                pwms = self.ik.calc_pwm(abd, hip, knee)
                for k in range(3):
                    self.lbls[leg][k].config(text=str(pwms[k]), fg="black")
                
                # FK 绘图 (在 Body Frame 绘制，然后转 World)
                pts = self.ik.forward_kinematics(abd, hip, knee, is_left)
                
                world_pts = []
                for pt in pts:
                    # Pt is relative to Hip. 
                    # World = Body_Pos + R @ (Hip_Body + Pt)
                    wp = np.array([bx, by, 0]) + R @ (np.array(origin) + pt)
                    world_pts.append(wp)
                world_pts = np.array(world_pts)
                
                col = 'b' if is_left else 'r'
                self.ax.plot(world_pts[:,0], world_pts[:,1], world_pts[:,2], f'{col}-o', lw=2)
            else:
                for k in range(3): self.lbls[leg][k].config(text="ERR", fg="red")

        self.canvas.draw()

if __name__ == "__main__":
    app = SimApp()
    app.mainloop()
