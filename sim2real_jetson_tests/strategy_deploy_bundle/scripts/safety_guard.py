"""
安全保护模块 — 独立的急停与限幅逻辑。

安全层级:
    1. 关节限位 (硬限幅)
    2. 动作变化率限制 (防抖动)
    3. 姿态保护 (翻倒检测)
    4. 心跳超时 (通信断联检测)
"""

from __future__ import annotations

import time

import numpy as np


class SafetyGuard:
    """多层安全保护。"""

    def __init__(
        self,
        joint_limits_low: np.ndarray,
        joint_limits_high: np.ndarray,
        max_action_delta: float = 0.5,
        max_pitch_deg: float = 45.0,
        max_roll_deg: float = 45.0,
        heartbeat_timeout: float = 0.1,
    ):
        self.joint_limits_low = joint_limits_low.astype(np.float32)
        self.joint_limits_high = joint_limits_high.astype(np.float32)
        self.max_action_delta = max_action_delta
        self.max_pitch_deg = max_pitch_deg
        self.max_roll_deg = max_roll_deg
        self.heartbeat_timeout = heartbeat_timeout

        self._prev_target = None
        self._emergency = False
        self._emergency_reason = ""
        self._last_heartbeat = time.perf_counter()

    def update_heartbeat(self):
        """在每个控制步调用，重置心跳计时器。"""
        self._last_heartbeat = time.perf_counter()

    def check_heartbeat(self) -> bool:
        """检查心跳是否超时。"""
        dt = time.perf_counter() - self._last_heartbeat
        if dt > self.heartbeat_timeout:
            self._emergency = True
            self._emergency_reason = f"Heartbeat timeout: {dt:.3f}s > {self.heartbeat_timeout}s"
            return False
        return True

    def check_and_clip(
        self,
        target_pos: np.ndarray,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
    ) -> tuple[np.ndarray, bool]:
        """
        对目标位置执行多层安全检查。

        Returns:
            (safe_target, is_safe)
            is_safe=False 时应立即停止。
        """
        # 如果已触发急停，拒绝所有新指令
        if self._emergency:
            return (
                self._prev_target
                if self._prev_target is not None
                else np.zeros_like(target_pos)
            ), False

        # Layer 1: 关节硬限幅
        target_pos = np.clip(target_pos, self.joint_limits_low, self.joint_limits_high)

        # Layer 2: 姿态保护
        if abs(roll_deg) > self.max_roll_deg:
            self._emergency = True
            self._emergency_reason = f"Roll exceeded: {roll_deg:.1f}° > ±{self.max_roll_deg}°"
            return self._prev_target if self._prev_target is not None else target_pos, False

        if abs(pitch_deg) > self.max_pitch_deg:
            self._emergency = True
            self._emergency_reason = f"Pitch exceeded: {pitch_deg:.1f}° > ±{self.max_pitch_deg}°"
            return self._prev_target if self._prev_target is not None else target_pos, False

        # Layer 3: 动作变化率限制
        if self._prev_target is not None:
            delta = target_pos - self._prev_target
            delta_clipped = np.clip(delta, -self.max_action_delta, self.max_action_delta)
            target_pos = self._prev_target + delta_clipped

        self._prev_target = target_pos.copy()
        return target_pos, True

    @property
    def is_emergency(self) -> bool:
        return self._emergency

    @property
    def emergency_reason(self) -> str:
        return self._emergency_reason

    def reset(self):
        """重置急停状态（谨慎使用）。"""
        self._emergency = False
        self._emergency_reason = ""
        self._prev_target = None
        print("[SafetyGuard] Reset.")
