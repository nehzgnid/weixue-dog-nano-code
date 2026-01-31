import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D

# === 机器人物理参数 (单位: mm) ===
BODY_LENGTH = 263.7
BODY_WIDTH = 49.0
L1 = 63.0   # 侧向偏置 (Abduction)
L2 = 100.0  # 大腿 (Upper)
L3 = 117.0  # 小腿 (Lower)

# === 舵机零点偏置 (用户提供) ===
# 定义：当舵机输出 2048 时，关节相对于理论垂直/水平线的物理偏移
OFFSET_HIP_DEG = 8.6
OFFSET_KNEE_DEG = 15.3
OFFSET_HIP_RAD = np.radians(OFFSET_HIP_DEG)
OFFSET_KNEE_RAD = np.radians(OFFSET_KNEE_DEG)

# === 舵机限幅 (0-4095) ===
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
        计算单腿 IK
        x, y, z: 足端相对于髋关节(侧摆轴心)的坐标
        """
        side_sign = 1 if is_left else -1
        
        # 1. 侧摆角 (Theta 1)
        # 投影到 YZ 平面
        d = np.sqrt(y**2 + z**2)
        if d < L1: return None # 构型不可达
        
        theta_d = np.arctan2(y, -z)
        theta_off = np.arccos(L1 / d)
        
        if is_left:
            theta_abd = theta_d - theta_off
        else:
            theta_abd = theta_d + theta_off # 右侧镜像

        # 2. 投影到大腿平面
        # 此时我们将问题转化为 X-Z' 平面内的 2-Link IK
        # z_prime 是腿部平面的垂直深度
        z_prime = -np.sqrt(d**2 - L1**2)
        
        # 目标点到髋轴心的距离
        r = np.sqrt(x**2 + z_prime**2)
        if r > (L2 + L3) or r < abs(L2 - L3): return None

        # 3. 大腿 (Theta 2) 和 小腿 (Theta 3)
        # 使用余弦定理
        phi = np.arctan2(x, -z_prime)
        
        acos_hip = (L2**2 + r**2 - L3**2) / (2 * L2 * r)
        theta_hip = phi + np.arccos(np.clip(acos_hip, -1, 1))
        
        acos_knee = (L2**2 + L3**2 - r**2) / (2 * L2 * L3)
        theta_knee = np.pi - np.arccos(np.clip(acos_knee, -1, 1))

        return theta_abd, theta_hip, theta_knee

    def forward_kinematics(self, abd, hip, knee, is_left):
        """
        正运动学 (仅用于画图验证)
        返回: [p0, p1, p2, p3] 四个关节点的坐标
        """
        side = 1 if is_left else -1
        
        # P0: 髋关节原点 (0,0,0)
        p0 = np.array([0, 0, 0])
        
        # P1: 大腿根部 (经过侧摆)
        # 绕 X 轴旋转 theta_abd
        p1 = np.array([0, side * L1 * np.cos(abd), side * L1 * np.sin(abd)])
        
        # 在腿部局部平面内计算 P2(膝) 和 P3(脚)
        # 局部平面的 Z 轴是倾斜的
        # 这里简化计算，先算局部再旋转
        
        # 局部坐标 (x, z_loc)
        x2 = L2 * np.sin(hip)
        z2 = -L2 * np.cos(hip)
        
        x3 = x2 + L3 * np.sin(hip + knee) # knee 是相对角度
        z3 = z2 - L3 * np.cos(hip + knee)
        
        # 将局部坐标 (x, 0, z) 绕 X 轴旋转 abd
        def rotate_x(vec, angle):
            c, s = np.cos(angle), np.sin(angle)
            rm = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
            return rm @ vec

        # P1 是已经偏置过的，我们需要把 (x, 0, z) 加上 L1 的侧向偏移后再旋转？
        # 不，应该是在 (0, L1, 0) 的基础上叠加 (x, 0, z) 然后旋转
        # 这种几何关系比较绕，用 IK 的逆推更直观
        
        # 重新推导 FK 绘图点:
        # 坐标系: X前, Y左, Z上
        
        # P1 (侧摆臂末端)
        p1 = np.array([0, side * L1 * np.cos(abd), side * L1 * np.sin(abd)])
        
        # 腿部平面旋转矩阵 (绕 X 轴转 abd)
        c1, s1 = np.cos(abd), np.sin(abd)
        # 右腿需要特殊处理符号
        
        # 大腿向量 (在腿部平面内)
        # hip 角 0度是垂直向下 (-Z)
        v2_loc = np.array([L2 * np.sin(hip), 0, -L2 * np.cos(hip)])
        
        # 小腿向量
        # knee 角 0度是和小腿折叠? 不，IK里算的是内角，这里 theta_knee 是相对大腿的偏角
        # 上面 IK 算出的是关节角， forward kinematics 需要绝对角?
        # 这里的 theta_knee 是大腿和小腿的夹角
        
        # 修正: IK 返回的 knee 是大腿和小腿之间的夹角 (折叠为0，伸直为PI)
        # 还是通常定义的关节角? 让我们复用 IK 的逻辑
        # IK: theta_knee = pi - acos(...) -> 伸直是 0, 折叠是 pi?
        # 不，通常 r^2 = l2^2 + l3^2 - 2 l2 l3 cos(beta)
        # theta_knee = beta. 
        
        # 让我们用简单的向量叠加
        # 腿部平面内的向量
        vec_thigh = np.array([L2*np.sin(hip), -L2*np.cos(hip)]) # X-Z'
        vec_shank = np.array([L3*np.sin(hip+knee), -L3*np.cos(hip+knee)]) # knee 是相对角
        
        # 转换回 3D
        # P2 = P1 + RotX(abd) * (vec_thigh_3d)
        p2 = p1 + np.array([vec_thigh[0], 
                           -vec_thigh[1] * s1,
                            vec_thigh[1] * c1])
                            
        p3 = p2 + np.array([vec_shank[0],
                           -vec_shank[1] * s1,
                            vec_shank[1] * c1])
                            
        return p0, p1, p2, p3

    def calc_servo_value(self, rad, joint_type):
        """计算包含偏置的 PWM 值"""
        if joint_type == "abd":
            val = 2048 + rad * self.rad_to_step
            limit = LIMITS["abd"]
        elif joint_type == "hip":
            # 2048 对应 OFFSET_HIP_RAD
            # 如果 hip 计算出 0 度(垂直向下)，实际上舵机应该在 2048 - offset
            # 或者: 2048 是 "偏置后的姿态"。
            # 假设: 2048时，腿是垂直的吗？不，用户说 "2048...大腿朝前偏移8.6度"
            # 这意味着: 角度 = 8.6度时，PWM = 2048
            # PWM = 2048 + (目标角 - 8.6度) * 系数
            val = 2048 + (rad - OFFSET_HIP_RAD) * self.rad_to_step
            limit = LIMITS["hip"]
        elif joint_type == "knee":
            # 2048时，小腿相对于大腿偏移 15.3 度
            # 我们的 IK 计算出的 knee 是相对大腿的弯曲角
            # PWM = 2048 - (目标角 - 15.3度) * 系数  (小腿通常反向)
            val = 2048 - (rad - OFFSET_KNEE_RAD) * self.rad_to_step
            limit = LIMITS["knee"]
            
        return int(val), limit

class SimulationApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("机器狗运动学仿真 (Kinematics Simulator)")
        self.geometry("1200x800")
        self.ik = RobotIK()
        
        # 布局
        left_panel = tk.Frame(self, width=300, bg="#f0f0f0")
        left_panel.pack(side="left", fill="y")
        
        right_panel = tk.Frame(self, bg="white")
        right_panel.pack(side="right", fill="both", expand=True)
        
        # === 控件区 ===
        tk.Label(left_panel, text="姿态控制 (Body Pose)", font=("Arial", 12, "bold"), bg="#f0f0f0").pack(pady=10)
        
        self.vars = {}
        self.create_slider(left_panel, "Height (Z)", -100, -250, -180)
        self.create_slider(left_panel, "Shift X", -100, 100, 0)
        self.create_slider(left_panel, "Shift Y", -50, 50, 0)
        self.create_slider(left_panel, "Pitch (deg)", -20, 20, 0)
        self.create_slider(left_panel, "Roll (deg)", -20, 20, 0)
        self.create_slider(left_panel, "Yaw (deg)", -20, 20, 0)
        
        tk.Button(left_panel, text="重置 (Reset)", command=self.reset_sliders, bg="#ffcccb").pack(pady=10)
        
        # 舵机数据显示
        self.servo_labels = {}
        info_frame = tk.Frame(left_panel, bg="#ddd", padx=5, pady=5)
        info_frame.pack(fill="x", padx=5)
        
        legs = ["FL", "FR", "RL", "RR"]
        tk.Label(info_frame, text="Servo PWM (0-4095)", bg="#ddd", font=("Consolas", 10)).grid(row=0, columnspan=4)
        for i, leg in enumerate(legs):
            tk.Label(info_frame, text=leg, bg="#ddd", font="bold").grid(row=i+1, column=0)
            self.servo_labels[leg] = []
            for j in range(3):
                l = tk.Label(info_frame, text="----", width=5, bg="white", font=("Consolas", 9))
                l.grid(row=i+1, column=j+1, padx=2, pady=2)
                self.servo_labels[leg].append(l)

        # === 绘图区 ===
        self.fig = plt.Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # 初始绘制
        self.update_plot()

    def create_slider(self, parent, label, min_v, max_v, default):
        frame = tk.Frame(parent, bg="#f0f0f0")
        frame.pack(fill="x", padx=10, pady=5)
        tk.Label(frame, text=label, width=10, anchor="w", bg="#f0f0f0").pack(side="left")
        var = tk.DoubleVar(value=default)
        self.vars[label] = var
        s = tk.Scale(frame, from_=min_v, to=max_v, orient="horizontal", variable=var, 
                     command=lambda v: self.update_plot(), showvalue=True, resolution=1)
        s.pack(side="left", fill="x", expand=True)

    def reset_sliders(self):
        defaults = {"Height (Z)": -180, "Shift X": 0, "Shift Y": 0, "Pitch (deg)": 0, "Roll (deg)": 0, "Yaw (deg)": 0}
        for k, v in defaults.items():
            self.vars[k].set(v)
        self.update_plot()

    def transform(self, coord, dx, dy, dz, r, p, y):
        # 欧拉角旋转矩阵
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        
        R = Rz @ Ry @ Rx
        return (R @ coord) + np.array([dx, dy, dz])

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(-200, 200)
        self.ax.set_ylim(-150, 150)
        self.ax.set_zlim(-300, 100)
        self.ax.set_xlabel('X (Front)')
        self.ax.set_ylabel('Y (Left)')
        self.ax.set_zlabel('Z (Up)')
        
        # 获取输入
        h = self.vars["Height (Z)"].get()
        dx = self.vars["Shift X"].get()
        dy = self.vars["Shift Y"].get()
        roll = np.radians(self.vars["Roll (deg)"].get())
        pitch = np.radians(self.vars["Pitch (deg)"].get())
        yaw = np.radians(self.vars["Yaw (deg)"].get())
        
        # 定义四条腿的髋关节原点 (相对于机身中心)
        lx, ly = BODY_LENGTH/2, BODY_WIDTH/2
        origins = {
            "FL": np.array([lx, ly, 0]),
            "FR": np.array([lx, -ly, 0]),
            "RL": np.array([-lx, ly, 0]),
            "RR": np.array([-lx, -ly, 0])
        }
        
        # 绘制机身 (连接四个髋关节)
        body_pts = []
        for leg in ["FL", "FR", "RR", "RL", "FL"]:
            # 机身也会随着 RPY 旋转
            p = self.transform(origins[leg], dx, dy, 0, roll, pitch, yaw) # 机身中心高度设为0，脚底为 -h
            body_pts.append(p)
        body_pts = np.array(body_pts)
        self.ax.plot(body_pts[:,0], body_pts[:,1], body_pts[:,2], 'k-', linewidth=2, label='Body')

        # 计算并绘制每条腿
        for leg_name, origin_body in origins.items():
            is_left = "L" in leg_name
            
            # 1. 计算髋关节在世界坐标系中的位置 (应用机身姿态)
            hip_world = self.transform(origin_body, dx, dy, 0, roll, pitch, yaw)
            
            # 2. 计算足端目标点 (Foot Target)
            # 我们希望足端始终在地面 (z = h)，且相对于机身中心的初始投影位置不变
            # 默认站立姿态: 足端 x,y 与 侧摆关节后的髋关节对齐? 
            # 通常足端在侧摆延伸线的垂直下方
            foot_default_body = origin_body + np.array([0, L1 if is_left else -L1, h])
            
            # 足端不随机身旋转 (保持在地面接触点不变)
            # 但随 X/Y Shift 平移 (模拟机身移动时，相对脚的位置变化)
            # 这里 IK 的输入是: 足端相对于髋关节坐标系的坐标
            
            # 简单起见：假设脚不动，身子动 -> 
            # 相对坐标 = R_inv * (脚_world - 髋_world)
            
            foot_world = np.array([origin_body[0], origin_body[1] + (L1 if is_left else -L1), h]) 
            # 注意: 这里 h 是负数，所以脚在下方
            
            # 将 World 坐标转回 Body 局部坐标 (用于 IK)
            # P_body = R.T @ (P_world - Shift)
            
            # 欧拉角旋转矩阵的转置 (逆旋转)
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            R = Rz @ Ry @ Rx
            R_inv = R.T
            
            foot_rel_body = R_inv @ (foot_world - np.array([dx, dy, 0])) - origin_body
            
            # 3. 求解 IK
            res = self.ik.inverse_kinematics(foot_rel_body[0], foot_rel_body[1], foot_rel_body[2], is_left)
            
            if res:
                abd, hip, knee = res
                
                # 4. 显示 PWM 值
                types = ["abd", "hip", "knee"]
                vals = [abd, hip, knee]
                for i in range(3):
                    pwm, (low, high) = self.ik.calc_servo_value(vals[i], types[i])
                    lbl = self.servo_labels[leg_name][i]
                    lbl.config(text=str(pwm))
                    # 越界变红
                    if pwm < low or pwm > high:
                        lbl.config(fg="red", bg="#ffeeee")
                    else:
                        lbl.config(fg="black", bg="white")

                # 5. 绘制腿部线框 (FK)
                p0, p1, p2, p3 = self.ik.forward_kinematics(abd, hip, knee, is_left)
                # p0~p3 是相对于髋关节的局部坐标
                # 需转换到世界坐标画图
                # P_world = R * P_local + Hip_World
                pts = []
                for p in [p0, p1, p2, p3]:
                    pts.append( (R @ p) + hip_world )
                pts = np.array(pts)
                
                color = 'b-' if is_left else 'r-'
                self.ax.plot(pts[:,0], pts[:,1], pts[:,2], color, marker='o', markersize=4)
            else:
                # IK 无解
                for i in range(3):
                    self.servo_labels[leg_name][i].config(text="ERR", fg="red")

        self.canvas.draw()

if __name__ == "__main__":
    app = SimulationApp()
    app.mainloop()
