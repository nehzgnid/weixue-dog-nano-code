import numpy as np
from robot_config import cfg

# === 机器人几何参数 (从配置读取) ===
L1 = cfg.L1
L2 = cfg.L2
L3 = cfg.L3

# === 零点偏置 (从配置读取) ===
OFFSET_HIP = cfg.OFFSET_HIP
OFFSET_KNEE = cfg.OFFSET_KNEE

class LegKinematics:
    def __init__(self, leg_name):
        self.name = leg_name
        self.side_sign = 1 if "L" in leg_name else -1 
        self.rad_to_step = 4096 / (2 * np.pi)
        
        # === 舵机 ID 映射 ===
        self.servo_ids = self._get_ids(leg_name)
        
        # === 从配置加载 12 个独立方向 ===
        self.dir_abd = cfg.get_dir(self.servo_ids[0])
        self.dir_hip = cfg.get_dir(self.servo_ids[1])
        self.dir_knee = cfg.get_dir(self.servo_ids[2])

    def _get_ids(self, name):
        if name == "FL": return (1, 2, 3)
        if name == "FR": return (4, 5, 6)
        if name == "RL": return (7, 8, 9)
        if name == "RR": return (10, 11, 12)
        return (0,0,0)

    def solve_ik(self, x, y, z):
        # ... (IK 核心算法保持不变) ...
        dist_yz = np.sqrt(y**2 + z**2)
        if dist_yz < L1: return None
        l_eff = np.sqrt(dist_yz**2 - L1**2)
        
        if self.side_sign == 1:
            q_abd = np.arctan2(y, -z) - np.arctan2(L1, l_eff)
        else:
            q_abd = np.arctan2(y, -z) + np.arctan2(L1, l_eff)

        z_eff = -l_eff
        x_eff = x
        r_sq = x_eff**2 + z_eff**2
        r = np.sqrt(r_sq)
        if r > (L2 + L3): return None
        
        phi = np.arctan2(x_eff, -z_eff)
        cos_hip = (L2**2 + r_sq - L3**2) / (2 * L2 * r)
        ang_hip_internal = np.arccos(np.clip(cos_hip, -1, 1))
        cos_knee = (L2**2 + L3**2 - r_sq) / (2 * L2 * L3)
        ang_knee_internal = np.arccos(np.clip(cos_knee, -1, 1))
        
        q_hip = phi + ang_hip_internal
        q_knee = -(np.pi - ang_knee_internal) 
        
        return q_abd, q_hip, q_knee

    def rad_to_pwm(self, q_abd, q_hip, q_knee):
        """
        转换公式：PWM = 2048 + (目标物理弧度 - 归中时物理弧度) * 步数比例 * 方向系数
        """
        pwm_abd = 2048 + (q_abd - 0) * self.rad_to_step * self.dir_abd
        pwm_hip = 2048 + (q_hip - OFFSET_HIP) * self.rad_to_step * self.dir_hip
        pwm_knee = 2048 + (q_knee - OFFSET_KNEE) * self.rad_to_step * self.dir_knee
        
        # 限幅 (从配置读取)
        limits = cfg.LIMITS
        pwm_abd = np.clip(pwm_abd, *limits["abd"])
        pwm_hip = np.clip(pwm_hip, *limits["hip"])
        pwm_knee = np.clip(pwm_knee, *limits["knee"])
        
        return int(pwm_abd), int(pwm_hip), int(pwm_knee)

    # ... (FK 保持不变) ...
    def rot_x(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    def rot_y(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    def forward_kinematics_strict(self, q_abd, q_hip, q_knee):
        p0 = np.array([0, 0, 0])
        v1 = np.array([0, self.side_sign * L1, 0])
        R1 = self.rot_x(q_abd)
        p1 = R1 @ v1 
        v2 = np.array([0, 0, -L2])
        R2_local = self.rot_y(q_hip)
        R2_global = R1 @ R2_local
        p2 = p1 + R2_global @ v2 
        v3 = np.array([0, 0, -L3])
        R3_local = self.rot_y(q_knee)
        R3_global = R2_global @ R3_local
        p3 = p2 + R3_global @ v3 
        return p0, p1, p2, p3
