"""
观测向量构建器 + 动作转换器。

将 STM32Bridge.get_state() 的原始传感器数据按 Isaac Lab 训练时完全相同的格式
组装为 48 维观测向量，并将策略输出的 12 维动作转换为舵机目标刻度。

关节顺序约定
────────────
• Isaac Lab (策略 I/O 使用的顺序，按关节类型分组):
    [0]  FL_hip   [1]  FR_hip   [2]  RL_hip   [3]  RR_hip
    [4]  FL_thigh [5]  FR_thigh [6]  RL_thigh [7]  RR_thigh
    [8]  FL_calf  [9]  FR_calf  [10] RL_calf  [11] RR_calf

• DFS / 舵机 ID 顺序 (STM32 反馈/下发时的物理顺序，按腿 DFS):
    [0]  FL_hip   [1]  FL_thigh [2]  FL_calf
    [3]  FR_hip   [4]  FR_thigh [5]  FR_calf
    [6]  RL_hip   [7]  RL_thigh [8]  RL_calf
    [9]  RR_hip   [10] RR_thigh [11] RR_calf

• servo_to_isaac 映射:
    DFS index i → Isaac index servo_to_isaac[i]
    [0→0, 1→4, 2→8, 3→1, 4→5, 5→9, 6→2, 7→6, 8→10, 9→3, 10→7, 11→11]
"""

from __future__ import annotations

import numpy as np


class ObsBuilder:
    """
    48 维观测构建器 + 12 维动作转换器。

    主要职责:
    1. build_observation(): 原始传感器数据 → 48 维 obs（未归一化）
    2. action_to_servo_targets(): 策略 raw action → 舵机刻度 (0-4095)
    3. 维护 last_action 缓存 (obs[36:48])
    """

    # ── Isaac Lab 训练参数 (与 env.yaml / IO descriptors 严格一致) ───────────

    # Isaac 顺序的默认站姿 (rad)
    DEFAULT_JOINT_POS = np.array([
        0.1,  -0.1,  -0.1,   0.1,      # hip:   FL FR RL RR
       -0.65,  0.65, -0.65,  0.65,     # thigh: FL FR RL RR
        0.6,  -0.6,   0.6,  -0.6,      # calf:  FL FR RL RR
    ], dtype=np.float32)

    ACTION_SCALE = 0.25   # JointPositionActionCfg.scale

    # DFS index → Isaac index
    SERVO_TO_ISAAC = np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int32)
    # Isaac index → DFS index
    ISAAC_TO_SERVO = np.zeros(12, dtype=np.int32)

    ISAAC_JOINT_NAMES = [
        "FL_hip",  "FR_hip",  "RL_hip",  "RR_hip",
        "FL_thigh","FR_thigh","RL_thigh","RR_thigh",
        "FL_calf", "FR_calf", "RL_calf", "RR_calf",
    ]

    def __init__(self, config: dict | None = None):
        """
        Args:
            config: 部署配置字典 (来自 wavego_deploy_config.yaml 的 observation 节)

                servo_to_isaac (list[int]):
                    DFS→Isaac 的索引映射。**首次请通过 test_hardware.py --test joint_map 标定确认**。

                joint_direction (list[float]):
                    每个关节 Isaac 顺序的方向标志（+1 或 -1）。
                    +1 = 仿真正方向与实机一致，-1 = 需要取反。
                    **必须通过逐关节测试确认！**

                joint_zero_offsets (list[float]):
                    每个关节的零点偏移 (Isaac 顺序, rad)。
                    offset = 实机零位弧度 - 仿真零位弧度。

                imu_axis_remap (list[int]):
                    WT61C → Isaac body frame 的轴重映射索引，例如 [0, 1, 2]。
                    Isaac Lab: X=前 Y=左 Z=上；WT61C 请查手册确认后修改。

                imu_axis_sign (list[float]):
                    重映射后各轴的符号，例如 [1, 1, 1]。

                base_lin_vel_mode (str):
                    'zero'      填零（推荐首次部署）
                    'integrate' 加速度积分 + 指数衰减
        """
        cfg = config or {}

        servo_to_isaac = cfg.get("servo_to_isaac", list(self.SERVO_TO_ISAAC))
        self.servo_to_isaac = np.array(servo_to_isaac, dtype=np.int32)

        # 反向映射
        for dfs_i, isaac_i in enumerate(self.servo_to_isaac):
            self.ISAAC_TO_SERVO[isaac_i] = dfs_i

        self.joint_direction = np.array(cfg.get("joint_direction", [1.0] * 12), dtype=np.float32)
        self.joint_zero_offsets = np.array(cfg.get("joint_zero_offsets", [0.0] * 12), dtype=np.float32)

        self.imu_axis_remap = list(cfg.get("imu_axis_remap", [0, 1, 2]))
        self.imu_axis_sign  = np.array(cfg.get("imu_axis_sign", [1.0, 1.0, 1.0]), dtype=np.float32)

        self.base_lin_vel_mode = cfg.get("base_lin_vel_mode", "zero")
        self._vel_estimate = np.zeros(3, dtype=np.float32)
        self._vel_decay    = 0.95

        # 缓存
        self.last_action    : np.ndarray = np.zeros(12, dtype=np.float32)
        self.prev_joint_pos : np.ndarray | None = None

    # ── 核心接口 ─────────────────────────────────────────────────────────────

    def build_observation(
        self,
        state: dict,
        commands: np.ndarray,
        dt: float = 0.02,
    ) -> np.ndarray:
        """
        将 STM32Bridge.get_state() 的原始状态构建为 48 维观测向量（未归一化）。

        Args:
            state:    STM32Bridge.get_state() 的返回值
            commands: [vx_cmd(m/s), vy_cmd(m/s), wz_cmd(rad/s)]
            dt:       控制步长 (秒), 默认 0.02 (50 Hz)

        Returns:
            np.ndarray shape=(48,), float32, 原始量纲，未归一化
        """
        # ── [0:3]  base_lin_vel (m/s, body frame) ──────────────────────────
        base_lin_vel = self._estimate_lin_vel(state, dt)

        # ── [3:6]  base_ang_vel (rad/s, body frame) ────────────────────────
        # WT61C 陀螺仪直接输出 body frame 角速度 (deg/s) → 转 rad/s
        gyro_raw = np.array([state["imu_gyro"][i] for i in self.imu_axis_remap], dtype=np.float32)
        gyro_raw *= self.imu_axis_sign
        base_ang_vel = gyro_raw * (np.pi / 180.0)

        # ── [6:9]  projected_gravity (body frame) ──────────────────────────
        roll_rad  = state["imu_roll"]  * (np.pi / 180.0)
        pitch_rad = state["imu_pitch"] * (np.pi / 180.0)
        projected_gravity = self._projected_gravity(roll_rad, pitch_rad)

        # ── [9:12] velocity_commands ───────────────────────────────────────
        cmd = np.array(commands, dtype=np.float32)

        # ── [12:24] joint_pos_rel (Isaac 顺序, rad) ─────────────────────────
        joint_pos_isaac = self._servo_to_isaac_rad(state["servo_pos"])
        joint_pos_rel   = joint_pos_isaac - self.DEFAULT_JOINT_POS

        # ── [24:36] joint_vel (Isaac 顺序, rad/s) ───────────────────────────
        joint_vel = self._compute_joint_vel(state, joint_pos_isaac, dt)
        self.prev_joint_pos = joint_pos_isaac.copy()

        # ── [36:48] last_action ─────────────────────────────────────────────
        last_action = self.last_action.copy()

        obs = np.concatenate([
            base_lin_vel,      # [0:3]
            base_ang_vel,      # [3:6]
            projected_gravity, # [6:9]
            cmd,               # [9:12]
            joint_pos_rel,     # [12:24]
            joint_vel,         # [24:36]
            last_action,       # [36:48]
        ]).astype(np.float32)

        assert obs.shape == (48,), f"[ObsBuilder] 维度错误: {obs.shape}"
        return obs

    def action_to_servo_targets(self, raw_action: np.ndarray) -> np.ndarray:
        """
        将策略原始输出转换为舵机目标刻度。

        公式（与 Isaac Lab JointPositionActionCfg 完全一致）:
            target_rad (Isaac) = default_joint_pos + raw_action * ACTION_SCALE
            target_rad (DFS)   = 方向校正 + 零点偏移 + 重排
            target_raw (0-4095)= target_rad_dfs / (2π) * 4096 + 2048

        Args:
            raw_action: 策略输出 (12,) Isaac 顺序

        Returns:
            np.ndarray (12,) int32, DFS 舵机顺序, 值域 0-4095
        """
        # Isaac 顺序的目标角度
        target_isaac = self.DEFAULT_JOINT_POS + raw_action * self.ACTION_SCALE

        # 零点偏移还原（实机零位 ≠ 仿真零位时补偿）
        target_isaac = target_isaac + self.joint_zero_offsets
        # 方向还原（取反轴的方向修正）
        target_isaac = target_isaac * self.joint_direction

        # Isaac 顺序 → DFS 舵机顺序
        target_dfs = target_isaac[self.ISAAC_TO_SERVO]

        # rad → STS3215 刻度 (4096 步/圈, 中位 2048 = 0 rad)
        target_raw = np.round(target_dfs / (2 * np.pi) * 4096 + 2048).astype(np.int32)
        target_raw = np.clip(target_raw, 0, 4095)

        # 更新 last_action 缓存
        self.last_action = raw_action.astype(np.float32).copy()
        return target_raw

    def reset(self):
        """重置帧间缓存（新 episode 或急停恢复后调用）。"""
        self.last_action    = np.zeros(12, dtype=np.float32)
        self.prev_joint_pos = None
        self._vel_estimate  = np.zeros(3, dtype=np.float32)

    # ── 内部辅助 ─────────────────────────────────────────────────────────────

    def _servo_to_isaac_rad(self, servo_pos_dfs: np.ndarray) -> np.ndarray:
        """DFS 顺序弧度 → Isaac 顺序弧度（含方向修正和零点修正）。"""
        isaac_rad = servo_pos_dfs[self.servo_to_isaac]   # 重排
        isaac_rad = isaac_rad * self.joint_direction       # 方向
        isaac_rad = isaac_rad - self.joint_zero_offsets    # 零点
        return isaac_rad.astype(np.float32)

    def _compute_joint_vel(
        self, state: dict, joint_pos_isaac: np.ndarray, dt: float
    ) -> np.ndarray:
        """关节速度：优先用舵机反馈，次选有限差分。"""
        vel_raw = state.get("servo_vel")
        if vel_raw is not None and np.any(vel_raw != 0):
            # 从舵机反馈（需确认固件单位，此处假设已为 rad/s）
            vel = np.array(vel_raw, dtype=np.float32)[self.servo_to_isaac] * self.joint_direction
        elif self.prev_joint_pos is not None and dt > 1e-5:
            vel = (joint_pos_isaac - self.prev_joint_pos) / dt
        else:
            vel = np.zeros(12, dtype=np.float32)
        return vel.astype(np.float32)

    def _estimate_lin_vel(self, state: dict, dt: float) -> np.ndarray:
        """base_lin_vel 估计（WT61C 无法直接提供线速度）。"""
        if self.base_lin_vel_mode == "zero":
            return np.zeros(3, dtype=np.float32)

        if self.base_lin_vel_mode == "integrate":
            # 加速度积分 + 指数衰减
            accel = np.array(
                [state["imu_accel"][i] for i in self.imu_axis_remap], dtype=np.float32
            ) * self.imu_axis_sign
            self._vel_estimate = self._vel_estimate * self._vel_decay + accel * dt
            return self._vel_estimate.copy()

        return np.zeros(3, dtype=np.float32)

    @staticmethod
    def _projected_gravity(roll_rad: float, pitch_rad: float) -> np.ndarray:
        """
        重力在机体坐标系的投影（单位向量）。

        等价于 Isaac Lab 中: quat_rotate_inverse(root_quat, [0, 0, -1])
        静止水平时输出 ≈ [0, 0, -1]。

        Isaac Lab 坐标系: X=前, Y=左, Z=上
          - 前倾 (pitch > 0) → gx < 0
          - 右倾 (roll > 0)  → gy < 0
        """
        cr, sr = np.cos(roll_rad),  np.sin(roll_rad)
        cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
        gx = -sp
        gy =  cp * sr
        gz = -cp * cr
        return np.array([gx, gy, gz], dtype=np.float32)
