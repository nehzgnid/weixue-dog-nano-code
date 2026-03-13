#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_io import RobotIO  # noqa: E402


class RobotIOBridge:
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.io: RobotIO | None = None
        self.last_state_time = 0.0

    def connect(self) -> bool:
        self.io = RobotIO(port=self.port, baud_rate=self.baudrate)
        ok = self.io.ser is not None
        if ok:
            self.last_state_time = time.perf_counter()
        return ok

    def disconnect(self):
        if self.io is not None:
            self.io.close()

    def set_torque(self, enable: bool):
        if self.io is not None:
            self.io.send_torque(enable)

    def send_servo_targets(self, target_raw: np.ndarray, speed: int = 3400, time_ms: int = 120):
        if self.io is None:
            return
        target_raw = np.clip(target_raw, 0, 4095).astype(np.int32)
        cmd = []
        for sid, pos in enumerate(target_raw.tolist(), start=1):
            cmd.append((sid, int(pos), int(time_ms), int(speed), 0))
        self.io.send_servos(cmd)

    def get_state(self) -> dict:
        now = time.perf_counter()
        if self.io is None:
            return {
                "servo_pos": np.zeros(12, dtype=np.float32),
                "servo_vel": np.zeros(12, dtype=np.float32),
                "imu_roll": 0.0,
                "imu_pitch": 0.0,
                "imu_yaw": 0.0,
                "imu_gyro": np.zeros(3, dtype=np.float32),
                "imu_accel": np.zeros(3, dtype=np.float32),
                "servo_count": 0,
                "servo_ids": tuple(),
                "timestamp": 0.0,
            }

        imu = self.io.get_imu()
        with self.io.lock:
            states = dict(self.io.servo_states)
            servo_count = int(getattr(self.io, "servo_count", len(states)))
            servo_ids = tuple(getattr(self.io, "servo_ids", sorted(states.keys())))

        servo_pos = np.zeros(12, dtype=np.float32)
        servo_vel = np.zeros(12, dtype=np.float32)
        for sid in range(1, 13):
            st = states.get(sid)
            if st is None:
                continue
            pos_raw = float(st.get("pos", 2048.0))
            spd_raw = float(st.get("spd", 0.0))
            servo_pos[sid - 1] = (pos_raw - 2048.0) * (2.0 * np.pi / 4096.0)
            servo_vel[sid - 1] = spd_raw * (2.0 * np.pi / 4096.0)

        imu_roll = float(imu.get("roll", 0.0))
        imu_pitch = float(imu.get("pitch", 0.0))
        imu_yaw = float(imu.get("yaw", 0.0))
        imu_gyro = np.array(imu.get("gyro", [0.0, 0.0, 0.0]), dtype=np.float32)
        imu_accel = np.array(imu.get("accel", [0.0, 0.0, 0.0]), dtype=np.float32)

        rx_ts = float(getattr(self.io, "last_rl_state_time", 0.0))
        if rx_ts > 0.0:
            self.last_state_time = rx_ts
        elif self.last_state_time <= 0.0:
            self.last_state_time = now

        return {
            "servo_pos": servo_pos,
            "servo_vel": servo_vel,
            "imu_roll": imu_roll,
            "imu_pitch": imu_pitch,
            "imu_yaw": imu_yaw,
            "imu_gyro": imu_gyro,
            "imu_accel": imu_accel,
            "servo_count": servo_count,
            "servo_ids": servo_ids,
            "timestamp": self.last_state_time,
        }
