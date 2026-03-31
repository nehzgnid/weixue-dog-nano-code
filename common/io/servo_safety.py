from __future__ import annotations

import re
import struct
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Sequence, Tuple

from ..config.robot_config import cfg

MAX_SERVO_PER_PACKET = 18
_LIMIT_PATTERN = re.compile(r"^\s*(\d+)\s*[:\uff1a]\s*(-?\d+)\s*-\s*(-?\d+)\s*$")


def _find_repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "MIGRATION_PLAN_SCHEME_B_EXECUTION.md").exists():
            return parent
    return start


def _fallback_limits(max_servo_id: int = MAX_SERVO_PER_PACKET) -> Dict[int, Tuple[int, int]]:
    fallback: Dict[int, Tuple[int, int]] = {}
    limits = cfg.LIMITS
    for sid in range(1, max_servo_id + 1):
        joint = ("abd", "hip", "knee")[(sid - 1) % 3]
        low, high = limits.get(joint, (0, 4095))
        fallback[sid] = (int(low), int(high))
    return fallback


def anger_limit_path() -> Path:
    root = _find_repo_root(Path(__file__).resolve().parent)
    return root / "anger-limit.txt"


def load_hard_limits(
    limit_file: Path | None = None,
    max_servo_id: int = MAX_SERVO_PER_PACKET,
) -> Dict[int, Tuple[int, int]]:
    path = limit_file or anger_limit_path()
    parsed: Dict[int, Tuple[int, int]] = {}

    if path.exists():
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            match = _LIMIT_PATTERN.match(raw_line)
            if not match:
                continue
            sid = int(match.group(1))
            low = int(match.group(2))
            high = int(match.group(3))
            if low > high:
                low, high = high, low
            parsed[sid] = (low, high)

    fallback = _fallback_limits(max_servo_id=max_servo_id)
    for sid in range(1, max_servo_id + 1):
        parsed.setdefault(sid, fallback[sid])
    return parsed


def split_servo_batches(
    commands: Sequence[Tuple[int, int, int, int, int]],
    max_count: int = MAX_SERVO_PER_PACKET,
) -> List[List[Tuple[int, int, int, int, int]]]:
    return [list(commands[i : i + max_count]) for i in range(0, len(commands), max_count)]


def build_servo_control_payload(commands: Sequence[Tuple[int, int, int, int, int]]) -> bytearray:
    if len(commands) > 255:
        raise ValueError("too many servo commands in one packet")

    payload = bytearray([len(commands)])
    for sid, pos, time_ms, speed, acc in commands:
        payload.extend(
            struct.pack(
                "<B B h H H",
                max(0, min(255, int(sid))),
                max(0, min(254, int(acc))),
                max(-32768, min(32767, int(pos))),
                max(0, min(65535, int(time_ms))),
                max(0, min(65535, int(speed))),
            )
        )
    return payload


class ServoSafetyGuard:
    def __init__(
        self,
        *,
        limit_file: Path | None = None,
        enforce_nonzero_timing: bool = True,
        default_time_ms: int = 30,
        default_speed: int = 1200,
        max_speed: int = 3400,
        max_time_ms: int = 2000,
        max_step_delta: int = 260,
    ):
        self.hard_limits = load_hard_limits(limit_file=limit_file)
        self.enforce_nonzero_timing = bool(enforce_nonzero_timing)
        self.default_time_ms = max(1, int(default_time_ms))
        self.default_speed = max(1, int(default_speed))
        self.max_speed = max(1, int(max_speed))
        self.max_time_ms = max(0, int(max_time_ms))
        self.max_step_delta = max(0, int(max_step_delta))
        self._last_command_pos: Dict[int, int] = {}

    def reset_history(self):
        self._last_command_pos.clear()

    def sanitize_batch(
        self,
        raw_commands: Iterable[Sequence[float | int]],
        *,
        offset_getter: Callable[[int], int] | None = None,
        feedback_getter: Callable[[int], int | None] | None = None,
    ) -> List[Tuple[int, int, int, int, int]]:
        safe: List[Tuple[int, int, int, int, int]] = []

        for raw in raw_commands:
            if len(raw) == 5:
                sid_raw, pos_raw, time_raw, speed_raw, acc_raw = raw
                legacy_format = False
            elif len(raw) == 4:
                sid_raw, pos_raw, speed_raw, acc_raw = raw
                time_raw = 0
                legacy_format = True
            else:
                raise ValueError(f"unsupported servo command format: {raw}")

            sid = max(0, min(255, int(sid_raw)))
            pos = int(round(float(pos_raw)))
            if offset_getter is not None:
                pos += int(offset_getter(sid))

            time_ms = int(time_raw)
            speed = int(speed_raw)
            acc = max(0, min(254, int(acc_raw)))

            if self.enforce_nonzero_timing and (legacy_format or time_ms <= 0):
                time_ms = self.default_time_ms
            if self.enforce_nonzero_timing and speed <= 0:
                speed = self.default_speed

            max_time = self.max_time_ms if self.max_time_ms > 0 else 65535
            time_ms = max(0, min(max_time, time_ms))
            speed = max(0, min(self.max_speed, speed))

            if time_ms > 0 and speed == 0:
                speed = self.default_speed

            low_high = self.hard_limits.get(sid)
            if low_high is not None:
                low, high = low_high
                pos = max(low, min(high, pos))

            reference = self._last_command_pos.get(sid)
            if reference is None and feedback_getter is not None:
                feedback_pos = feedback_getter(sid)
                if feedback_pos is not None:
                    reference = int(feedback_pos)

            if reference is not None and self.max_step_delta > 0:
                allowed_delta = self.max_step_delta
                if time_ms > self.default_time_ms:
                    scale = time_ms / float(self.default_time_ms)
                    allowed_delta = max(allowed_delta, int(self.max_step_delta * scale))

                delta = pos - reference
                if delta > allowed_delta:
                    pos = reference + allowed_delta
                elif delta < -allowed_delta:
                    pos = reference - allowed_delta

                if low_high is not None:
                    low, high = low_high
                    pos = max(low, min(high, pos))

            self._last_command_pos[sid] = pos
            safe.append((sid, pos, time_ms, speed, acc))

        return safe
