import numpy as np

class LegKinematics:
    def __init__(self, l1, l2, l3, is_left=True):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.side_sign = 1 if is_left else -1

    def solve_ik(self, x, y, z):
        # 基础 IK 解析解 (弧度)
        d = np.sqrt(y**2 + z**2)
        if d < self.l1: return None
        
        theta_d = np.arctan2(y, -z)
        theta_off = np.arccos(self.l1 / d)
        
        if self.side_sign == 1:
            theta_abd = theta_d - theta_off
        else:
            theta_abd = theta_d + theta_off

        z_rel = -d * np.cos(theta_d - theta_abd)
        r_sq = x**2 + z_rel**2
        r = np.sqrt(r_sq)
        
        if r > (self.l2 + self.l3) or r < abs(self.l2 - self.l3):
            return None

        # 余弦定理计算
        cos_beta = (self.l2**2 + self.l3**2 - r_sq) / (2 * self.l2 * self.l3)
        theta_knee = np.pi - np.arccos(np.clip(cos_beta, -1.0, 1.0))

        alpha = np.arctan2(x, -z_rel)
        cos_gamma = (self.l2**2 + r_sq - self.l3**2) / (2 * self.l2 * r)
        theta_hip = alpha + np.arccos(np.clip(cos_gamma, -1.0, 1.0))

        return theta_abd, theta_hip, theta_knee

class RobotKinematics:
    def __init__(self):
        # 尺寸
        self.l1 = 63.0
        self.l2 = 100.0
        self.l3 = 117.0
        
        # 偏置量 (用户提供)
        self.hip_offset_rad = np.radians(8.6)
        self.knee_offset_rad = np.radians(15.3)
        
        # 限幅 (基于 anger-limit.txt)
        self.limits = {
            "abd": (1800, 2300),
            "hip": (1200, 2900),
            "knee": (1450, 2650)
        }

        self.legs = {
            "FL": LegKinematics(self.l1, self.l2, self.l3, is_left=True),
            "FR": LegKinematics(self.l1, self.l2, self.l3, is_left=False),
            "RL": LegKinematics(self.l1, self.l2, self.l3, is_left=True),
            "RR": LegKinematics(self.l1, self.l2, self.l3, is_left=False)
        }

    def rad_to_step(self, rad, joint_type):
        """
        根据舵机安装和偏置转换弧度到步数
        """
        RAD_TO_STEPS = 4096 / (2 * np.pi)
        
        if joint_type == "abd":
            step = 2048 + (rad * RAD_TO_STEPS)
            low, high = self.limits["abd"]
        elif joint_type == "hip":
            # 2048 对应 hip_offset_rad
            step = 2048 + ((rad - self.hip_offset_rad) * RAD_TO_STEPS)
            low, high = self.limits["hip"]
        elif joint_type == "knee":
            # 2048 对应 knee_offset_rad
            # 通常小腿运动方向与大腿相反
            step = 2048 - ((rad - self.knee_offset_rad) * RAD_TO_STEPS)
            low, high = self.limits["knee"]
        
        return int(np.clip(step, low, high))

    def get_all_legs_steps(self, x, y, z):
        """
        计算四条腿在相同高度 z 下的舵机指令
        """
        commands = []
        mapping = {
            "FL": (1, 2, 3), "FR": (4, 5, 6),
            "RL": (7, 8, 9), "RR": (10, 11, 12)
        }
        
        for leg_id, ids in mapping.items():
            # 这里 y 对于右侧腿需要取反，因为坐标系定义 y+ 为左
            y_val = y if "L" in leg_id else -y
            
            res = self.legs[leg_id].solve_ik(x, y_val, z)
            if res:
                abd, hip, knee = res
                commands.append((ids[0], self.rad_to_step(abd, "abd"), 0, 0))
                commands.append((ids[1], self.rad_to_step(hip, "hip"), 0, 0))
                commands.append((ids[2], self.rad_to_step(knee, "knee"), 0, 0))
        
        return commands