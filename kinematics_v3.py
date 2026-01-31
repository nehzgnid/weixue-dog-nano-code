import numpy as np

# === 机器人几何参数 (mm) ===
L1 = 63.0   # 侧摆 (Abduction)
L2 = 100.0  # 大腿 (Thigh)
L3 = 117.0  # 小腿 (Shank)

# === 零点偏置 (角度制) ===
# 定义: 舵机输出 2048 时，关节相对于 [侧摆水平, 大腿垂直向下, 小腿垂直向下] 的偏差
# 根据您的描述:
# Hip: 2048时，大腿向前偏 8.6 度
# Knee: 2048时，小腿相对于大腿向前偏 15.3 度
OFFSET_HIP = np.radians(8.6)
OFFSET_KNEE = np.radians(15.3)

class LegKinematics:
    def __init__(self, leg_name):
        self.name = leg_name
        # 左右腿镜像系数: 左腿 +1, 右腿 -1
        self.side = 1 if "L" in leg_name else -1 
        self.rad_to_step = 4096 / (2 * np.pi)

    def solve_ik(self, x, y, z):
        """
        标准四足机器人逆运动学
        输入: 足端在髋关节坐标系下的坐标 (x, y, z)
        输出: 关节角 (q1_abd, q2_hip, q3_knee) 弧度
        """
        # 1. 侧摆角 (Abduction/Roll) - q1
        # 在 Y-Z 平面，足端到髋轴的距离 R_yz = sqrt(y^2 + z^2)
        # 侧摆臂长 L1, 有效腿长 L_eff = sqrt(R_yz^2 - L1^2)
        # q1 = atan2(y, -z) +/- atan2(L1, L_eff)
        # 注意: Z 是负的 (向下)
        
        R_yz = np.sqrt(y**2 + z**2)
        if R_yz < L1: return None # 构型不可达
        
        L_eff = np.sqrt(R_yz**2 - L1**2)
        
        # 计算 q1
        # 左腿(y>0): 希望向左张开
        # 右腿(y<0): 希望向右张开
        if self.side == 1: # 左
            q1 = np.arctan2(y, -z) - np.arctan2(L1, L_eff)
        else: # 右
            q1 = np.arctan2(y, -z) + np.arctan2(L1, L_eff)

        # 2. 投影到大腿平面 (Pitch)
        # 旋转 q1 后，腿部平面的坐标 (x_p, z_p)
        # x_p = x
        # z_p = -L_eff (因为 L_eff 是垂直向下的投影长度)
        x_p = x
        z_p = -L_eff
        
        R_p = np.sqrt(x_p**2 + z_p**2)
        if R_p > (L2 + L3): return None
        
        # 3. 大腿角 (Hip Pitch) - q2
        # 利用余弦定理计算三角形内角
        # alpha: 目标向量的极角
        alpha = np.arctan2(-x_p, -z_p) # 注意: 0度是垂直向下 (-Z)
        
        # beta: 大腿与目标向量的夹角 (内角)
        cos_beta = (L2**2 + R_p**2 - L3**2) / (2 * L2 * R_p)
        beta = np.arccos(np.clip(cos_beta, -1, 1))
        
        # 关键: 膝关节构型选择
        # 我们希望膝关节向后弯曲 (狗后腿式) -> q2 = alpha + beta
        # 如果是向前弯曲 (狗前腿式) -> q2 = alpha - beta
        # 这是一个设计选择。通常四足机器人选择 "Elbow-Elbow" (前腿向后弯，后腿向前弯) 或 "Knee-Knee" (全向后弯)。
        # 为了简单，我们先统一设为 "膝关节向后弯曲" (即小腿往后收)
        # 这意味着大腿要往前抬一点，小腿往后折。
        
        q2 = alpha + beta # 这种解法通常对应膝关节向后弯 (假设 x 正向为前)
        # 让我们修正一下定义: q2=0 是垂直向下。q2>0 是向前抬。
        
        # 4. 膝关节角 (Knee Pitch) - q3
        # gamma: 膝关节内角
        cos_gamma = (L2**2 + L3**2 - R_p**2) / (2 * L2 * L3)
        gamma = np.arccos(np.clip(cos_gamma, -1, 1))
        
        # q3 定义为小腿相对于大腿的偏转角
        # 0度是伸直。
        # 膝关节向后弯曲 -> q3 = gamma - pi  (负值)
        q3 = gamma - np.pi

        return q1, q2, q3

    def forward_kinematics(self, q1, q2, q3):
        """
        正运动学: 从角度推算坐标 (用于绘图验证长度是否变化)
        """
        # 坐标变换矩阵
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
        
        # P0: 髋原点
        p0 = np.array([0, 0, 0])
        
        # P1: 侧摆臂末端 (大腿根)
        # 绕 X 轴转 q1
        # 未旋转前: [0, side*L1, 0]
        # 旋转后:
        p1 = np.array([0, self.side * L1 * c1, self.side * L1 * s1])
        
        # 在腿部局部平面 (X-Z')
        # 大腿向量: [L2*s2, 0, -L2*c2] (q2=0 垂直向下)
        # 小腿向量: [L3*s23, 0, -L3*c23]
        
        v_thigh_loc = np.array([L2*np.sin(q2), 0, -L2*np.cos(q2)])
        v_shank_loc = np.array([L3*np.sin(q2+q3), 0, -L3*np.cos(q2+q3)])
        
        # 将局部向量绕 X 轴旋转 q1 变换到世界系
        # RotX = [[1, 0, 0], [0, c1, -s1], [0, s1, c1]]
        # 这里的旋转是作用在 (y, z) 上的。
        # 局部平面的 Y 是 0，Z 是负的。
        # 注意: 这里的 Y 轴定义需要小心。对于右腿，L1 是负的，向外是 -Y。
        
        def rot_x(v, angle):
            # 标准 Rx 旋转
            y_new = v[1]*np.cos(angle) - v[2]*np.sin(angle)
            z_new = v[1]*np.sin(angle) + v[2]*np.cos(angle)
            return np.array([v[0], y_new, z_new])

        p2 = p1 + rot_x(v_thigh_loc, q1)
        p3 = p1 + rot_x(v_shank_loc, q1) # 注意 shank 是相对于 hip 的绝对角度向量
        
        # 这种 FK 写法保证了线段长度恒定为 L1, L2, L3
        return p0, p1, p2, p3

    def calc_pwm(self, q1, q2, q3):
        # 侧摆: 2048=0度
        pwm1 = 2048 + q1 * self.rad_to_step
        
        # 大腿: 2048=向前8.6度 (OFFSET_HIP)
        # 我们算出的 q2=0 是垂直向下。
        # 所以 q2 = OFFSET_HIP 时，PWM 应该是 2048。
        # PWM = 2048 + (q2 - OFFSET_HIP) * ratio
        pwm2 = 2048 + (q2 - OFFSET_HIP) * self.rad_to_step
        
        # 小腿: 2048=向前15.3度 (相对于大腿)
        # 我们算出的 q3 是负值 (向后弯)。
        # 如果 q3 = OFFSET_KNEE (这是个正值，向前弯), PWM = 2048
        # PWM = 2048 - (q3 - OFFSET_KNEE) * ratio ? 
        # 修正: 小腿通常反向驱动。
        # 假设 2048 时，小腿确实向前翘了 15.3 度。
        # 现在我们要它向后弯 (q3 < 0)。
        # 那么舵机应该转很多。
        pwm3 = 2048 - (q3 - OFFSET_KNEE) * self.rad_to_step
        
        return int(pwm1), int(pwm2), int(pwm3)
