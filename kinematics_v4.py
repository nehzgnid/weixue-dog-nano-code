import numpy as np

# === 机器人几何参数 (mm) ===
L1 = 63.0   # 侧摆 (Abduction)
L2 = 100.0  # 大腿 (Thigh)
L3 = 117.0  # 小腿 (Shank)

# === 零点偏置 (物理角度) ===
# 2048 时:
# Hip: 向前 8.6 度 (Rot Y+)
# Knee: 相对于大腿再向前 15.3 度 (Rot Y+)
OFFSET_HIP = np.radians(8.6)
OFFSET_KNEE = np.radians(15.3)

class LegKinematics:
    def __init__(self, leg_name):
        self.name = leg_name
        # 左腿 L1 为正，右腿 L1 为负
        self.side_sign = 1 if "L" in leg_name else -1
        self.rad_to_step = 4096 / (2 * np.pi)

    def rot_x(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def rot_y(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def forward_kinematics_strict(self, q_abd, q_hip, q_knee):
        """
        严格刚体正运动学 (FK)
        q_abd, q_hip, q_knee: 都是相对于上一级关节的物理旋转角 (弧度)
        """
        # P0: 髋关节原点
        p0 = np.array([0, 0, 0])

        # === 连杆 1: 侧摆臂 ===
        # 初始向量: 沿 Y 轴 (左腿+L1, 右腿-L1)
        v1 = np.array([0, self.side_sign * L1, 0])
        # 旋转: 绕 X 轴转 q_abd
        R1 = self.rot_x(q_abd)
        p1 = R1 @ v1  # P1 是大腿根部坐标

        # === 连杆 2: 大腿 ===
        # 初始向量: 垂直向下 (-Z)
        v2 = np.array([0, 0, -L2])
        # 旋转: 
        # 1. 先随侧摆整体转 R1
        # 2. 自身绕 Y 轴转 q_hip (注意: 是在 R1 坐标系下的 Y 轴)
        # 组合旋转矩阵 R_thigh = R1 * Ry(q_hip)
        R2_local = self.rot_y(q_hip)
        R2_global = R1 @ R2_local
        
        p2 = p1 + R2_global @ v2 # P2 是膝关节坐标

        # === 连杆 3: 小腿 ===
        # 初始向量: 垂直向下 (-Z)
        v3 = np.array([0, 0, -L3])
        # 旋转:
        # 1. 基础是 R2_global
        # 2. 自身绕 Y 轴再转 q_knee
        # R_shank = R2_global * Ry(q_knee)
        R3_local = self.rot_y(q_knee)
        R3_global = R2_global @ R3_local
        
        p3 = p2 + R3_global @ v3 # P3 是足端坐标

        return p0, p1, p2, p3

    def pwm_to_rad(self, pwm_abd, pwm_hip, pwm_knee):
        """将 PWM 转换为物理旋转角"""
        # Abd: 2048 = 0度
        q_abd = (pwm_abd - 2048) / self.rad_to_step
        
        # Hip: 2048 = 向前 OFFSET_HIP
        # 假设舵机正转 = 向前 (Y轴旋转 +)
        q_hip = (pwm_hip - 2048) / self.rad_to_step + OFFSET_HIP
        
        # Knee: 2048 = 向前 OFFSET_KNEE
        # 假设舵机正转 = 向前 (与大腿同向)
        q_knee = (pwm_knee - 2048) / self.rad_to_step + OFFSET_KNEE
        
        return q_abd, q_hip, q_knee

    def rad_to_pwm(self, q_abd, q_hip, q_knee):
        """IK 算出角度后转 PWM"""
        pwm_abd = 2048 + q_abd * self.rad_to_step
        pwm_hip = 2048 + (q_hip - OFFSET_HIP) * self.rad_to_step
        pwm_knee = 2048 + (q_knee - OFFSET_KNEE) * self.rad_to_step
        return int(pwm_abd), int(pwm_hip), int(pwm_knee)

    def solve_ik(self, x, y, z):
        """
        逆运动学
        x,y,z: 足端相对于 P0 的坐标
        """
        # 1. 侧摆 (q_abd)
        dist_yz = np.sqrt(y**2 + z**2)
        if dist_yz < L1: return None
        
        l_eff = np.sqrt(dist_yz**2 - L1**2)
        
        # 这种算法假设腿是向外撇的
        if self.side_sign == 1: # Left
            q_abd = np.arctan2(y, -z) - np.arctan2(L1, l_eff)
        else: # Right
            q_abd = np.arctan2(y, -z) + np.arctan2(L1, l_eff)

        # 2. 投影到大腿平面
        # 旋转回侧摆平面内
        # R1_inv = RotX(-q_abd)
        # P_local = R1_inv * P_global
        # 我们只需要 x 和 z 分量
        # 简化: z_eff 是垂直向下的分量 (负值)
        z_eff = -l_eff
        x_eff = x
        
        # 3. 双连杆 IK (X-Z平面)
        r_sq = x_eff**2 + z_eff**2
        r = np.sqrt(r_sq)
        if r > (L2 + L3): return None
        
        # 目标向量角度 (相对于 -Z 轴)
        phi = np.arctan2(x_eff, -z_eff)
        
        # 内角 (余弦定理)
        cos_hip = (L2**2 + r_sq - L3**2) / (2 * L2 * r)
        ang_hip_internal = np.arccos(np.clip(cos_hip, -1, 1))
        
        cos_knee = (L2**2 + L3**2 - r_sq) / (2 * L2 * L3)
        ang_knee_internal = np.arccos(np.clip(cos_knee, -1, 1))
        
        # === 构型选择 ===
        # 方案 A: 膝盖向后弯 (Dog Leg, >)
        # q_hip = phi + ang_hip_internal
        # q_knee = - (pi - ang_knee_internal)  <-- 负角度表示向后折
        
        # 方案 B: 膝盖向前弯 (Human Leg, <)
        # q_hip = phi - ang_hip_internal
        # q_knee = + (pi - ang_knee_internal)  <-- 正角度表示向前折
        
        # 根据您的描述 "2048时小腿向前偏"，您的机械结构可能允许向前弯。
        # 但为了机器狗行走的稳定性，通常推荐向后弯 (方案A)。
        # 如果您的舵机安装导致 2048 是向前弯的，那我们需要给出一个负很大的角度才能让它向后弯。
        
        # 这里默认采用 **后弯构型 (Dog)**，因为这是四足行走的标准。
        q_hip = phi + ang_hip_internal
        q_knee = -(np.pi - ang_knee_internal) 
        
        return q_abd, q_hip, q_knee
