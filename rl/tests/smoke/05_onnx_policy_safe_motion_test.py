#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import select
import signal
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from common.io.robotio_bridge import RobotIOBridge  # noqa: E402
from rl.core.obs_builder import ObsBuilder  # noqa: E402
from rl.core.safety_guard import SafetyGuard  # noqa: E402


NUM_OBS = 48
NUM_ACTIONS = 12


def _load_ort():
    try:
        return importlib.import_module("onnxruntime")
    except Exception:
        return None


class ObsNormalizer:
    def __init__(self, stats_path: str, clip: float = 5.0, eps: float = 1e-8):
        data = np.load(stats_path)
        self.mean = data["mean"].astype(np.float32)
        self.std = data["std"].astype(np.float32)
        self.clip = clip
        self.eps = eps
        assert self.mean.shape == (NUM_OBS,)
        assert self.std.shape == (NUM_OBS,)

    def normalize(self, obs: np.ndarray) -> np.ndarray:
        normalized = (obs - self.mean) / (self.std + self.eps)
        return np.clip(normalized, -self.clip, self.clip)


class ONNXPolicy:
    def __init__(self, onnx_path: str, use_gpu: bool = False):
        ort = _load_ort()
        if ort is None:
            self.session = None
            self.input_name = None
            self.output_name = None
            print("[WARN] onnxruntime 不可用，使用零动作")
            return

        providers = ["CPUExecutionProvider"]
        if use_gpu:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

        self.session = ort.InferenceSession(onnx_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        inp_shape = self.session.get_inputs()[0].shape
        out_shape = self.session.get_outputs()[0].shape
        print(f"Policy loaded: input={inp_shape}, output={out_shape}")

    def predict(self, obs_normalized: np.ndarray) -> np.ndarray:
        if self.session is None:
            return np.zeros(NUM_ACTIONS, dtype=np.float32)
        obs_batch = obs_normalized.astype(np.float32).reshape(1, -1)
        result = self.session.run([self.output_name], {self.input_name: obs_batch})
        return result[0].squeeze().astype(np.float32)


class KeyEStop:
    def __init__(self, key: str = "q"):
        self.key = key.lower()
        self._fd = None
        self._old_term = None
        self.enabled = False

    def __enter__(self):
        if sys.stdin.isatty():
            self._fd = sys.stdin.fileno()
            self._old_term = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self.enabled = True
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.enabled and self._fd is not None and self._old_term is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)

    def poll_estop(self) -> bool:
        if not self.enabled:
            return False
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return False
        ch = sys.stdin.read(1).lower()
        return ch == self.key


def _load_config(config_path: str) -> tuple[dict, Path]:
    p = Path(config_path)
    if not p.is_absolute():
        candidates = [
            (REPO_ROOT / p).resolve(),
            (Path.cwd() / p).resolve(),
        ]
        p = next((c for c in candidates if c.exists()), candidates[0])
    with open(p, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cfg, p


def _resolve_artifact_path(path_str: str, cfg_path: Path) -> Path:
    raw = Path(path_str)
    if raw.is_absolute():
        return raw
    candidates = [
        (cfg_path.parent / raw).resolve(),
        (cfg_path.parent.parent / raw).resolve(),
        (REPO_ROOT / raw).resolve(),
    ]
    for c in candidates:
        if c.exists():
            return c
    return candidates[0]


def main() -> int:
    parser = argparse.ArgumentParser(description="ONNX 策略安全动作测试（带按键急停）")
    parser.add_argument("--config", default="rl/config/wavego_deploy_config.yaml")
    parser.add_argument("--port", default=None)
    parser.add_argument("--cmd-x", type=float, default=0.2)
    parser.add_argument("--cmd-y", type=float, default=0.0)
    parser.add_argument("--cmd-wz", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--virtual-imu", action="store_true", help="忽略真实 IMU，强制使用零姿态/零角速度")
    parser.add_argument("--estop-key", default="q", help="急停按键，默认 q")
    parser.add_argument("--no-gpu", action="store_true")
    parser.add_argument("--max-cmd", type=float, default=0.1, help="安全命令上限，默认 0.1")
    parser.add_argument("--action-gain", type=float, default=1.0, help="动作增益，默认 1.0")
    parser.add_argument(
        "--zero-cmd-action-gain",
        type=float,
        default=0.35,
        help="当速度命令接近0时使用的保守动作增益，默认 0.35",
    )
    parser.add_argument(
        "--zero-cmd-threshold",
        type=float,
        default=1e-3,
        help="判定零速度命令的阈值，默认 1e-3",
    )
    parser.add_argument("--servo-speed", type=int, default=3400, help="下发速度参数，默认 3400")
    parser.add_argument("--servo-time-ms", type=int, default=120, help="下发插值时间 ms，默认 120")
    parser.add_argument("--print-every", type=int, default=25, help="日志打印步数间隔，默认 25")
    parser.add_argument("--print-joint-vel", action="store_true", help="打印12维关节速度与位置")
    args = parser.parse_args()

    cmd = np.array([-args.cmd_y, args.cmd_x, args.cmd_wz], dtype=np.float32)  # Fix coordinate mapping: network 0=Right, 1=Forward
    cmd_norm = float(np.linalg.norm(cmd))
    is_zero_cmd = cmd_norm <= float(args.zero_cmd_threshold)
    if np.any(np.abs(cmd) > args.max_cmd):
        print(f"[FAIL] 命令超安全上限 max_cmd={args.max_cmd}: {cmd.tolist()}")
        return 1

    cfg, cfg_path = _load_config(args.config)

    timing = cfg["timing"]
    control_hz = int(timing["control_freq_hz"])
    control_dt = float(timing["control_dt_s"])

    policy_cfg = cfg["policy"]
    action_scale = float(policy_cfg["action_scale"])

    limits_low = np.array(cfg["joints"]["limits_low_policy"], dtype=np.float32)
    limits_high = np.array(cfg["joints"]["limits_high_policy"], dtype=np.float32)

    hw = cfg["hardware"]
    port = args.port or hw["serial_port"]

    print("=== 05 ONNX 策略安全动作测试 ===")
    print(f"config={cfg_path}")
    print(f"cmd={cmd.tolist()} | duration={args.duration}s | dry_run={args.dry_run}")
    print(f"virtual_imu={args.virtual_imu} | estop_key={args.estop_key}")
    if is_zero_cmd:
        print(
            f"[INFO] 零速度命令模式: 使用更保守增益 min(action_gain, zero_cmd_action_gain)="
            f"min({args.action_gain:.2f}, {args.zero_cmd_action_gain:.2f})"
        )
    print("[SAFE] 架空状态下测试；按 q 立即急停（并下发零位 + 扭矩关闭）")

    normalizer = ObsNormalizer(
        str(_resolve_artifact_path(cfg["model"]["normalizer_path"], cfg_path)),
        clip=float(policy_cfg["normalizer_clip"]),
        eps=float(policy_cfg["normalizer_eps"]),
    )
    policy = ONNXPolicy(
        str(_resolve_artifact_path(cfg["model"]["onnx_path"], cfg_path)),
        use_gpu=not args.no_gpu,
    )
    obs_builder = ObsBuilder(cfg.get("observation", {}))

    safety_cfg = cfg["safety"]
    safety = SafetyGuard(
        joint_limits_low=limits_low,
        joint_limits_high=limits_high,
        max_action_delta=float(safety_cfg.get("max_action_delta", 0.5)),
        max_pitch_deg=float(safety_cfg.get("max_pitch_deg", 45.0)),
        max_roll_deg=float(safety_cfg.get("max_roll_deg", 45.0)),
        heartbeat_timeout=0.2,
    )

    bridge = RobotIOBridge(
        port=port,
        baudrate=int(hw["serial_baud"]),
    )

    if not args.dry_run:
        if not bridge.connect():
            return 1
        bridge.set_torque(True)
        time.sleep(0.2)

    running = True
    estop_triggered = False
    estop_reason = ""

    def _sigint_handler(sig, frame):
        nonlocal running, estop_triggered, estop_reason
        running = False
        estop_triggered = True
        estop_reason = "Ctrl+C"

    signal.signal(signal.SIGINT, _sigint_handler)

    t_start = time.perf_counter()
    next_tick = t_start
    total_steps = max(1, int(args.duration * control_hz))

    try:
        with KeyEStop(args.estop_key) as key_estop:
            if not key_estop.enabled:
                print("[WARN] 当前终端不可捕获按键，仅支持 Ctrl+C 急停")

            for step in range(total_steps):
                if not running:
                    break

                if key_estop.poll_estop():
                    estop_triggered = True
                    estop_reason = f"key '{args.estop_key}'"
                    break

                state = bridge.get_state() if not args.dry_run else {
                    "servo_pos": np.zeros(12, dtype=np.float32),
                    "servo_vel": np.zeros(12, dtype=np.float32),
                    "imu_roll": 0.0,
                    "imu_pitch": 0.0,
                    "imu_yaw": 0.0,
                    "imu_gyro": np.zeros(3, dtype=np.float32),
                    "imu_accel": np.zeros(3, dtype=np.float32),
                    "timestamp": time.perf_counter(),
                }

                if args.virtual_imu:
                    state["imu_roll"] = 0.0
                    state["imu_pitch"] = 0.0
                    state["imu_yaw"] = 0.0
                    state["imu_gyro"] = np.zeros(3, dtype=np.float32)
                    state["imu_accel"] = np.zeros(3, dtype=np.float32)

                if not args.dry_run:
                    age = time.perf_counter() - state["timestamp"]
                    if age > 0.2:
                        estop_triggered = True
                        estop_reason = f"state timeout {age:.3f}s"
                        break

                obs_raw = obs_builder.build_observation(state, cmd, control_dt)
                obs_norm = normalizer.normalize(obs_raw)
                action_raw = policy.predict(obs_norm)
                gain_used = float(args.action_gain)
                if is_zero_cmd:
                    gain_used = min(gain_used, float(args.zero_cmd_action_gain))
                action_cmd = np.clip(action_raw * gain_used, -1.0, 1.0)

                target_pos_isaac = obs_builder.DEFAULT_JOINT_POS + action_cmd * action_scale
                target_pos_isaac = np.clip(target_pos_isaac, limits_low, limits_high)

                gx, gy, gz = obs_raw[6:9]
                roll_deg = float(np.degrees(np.arctan2(gy, -gz)))
                pitch_deg = float(np.degrees(np.arctan2(-gx, np.sqrt(gy**2 + gz**2))))

                safe_target, is_safe = safety.check_and_clip(
                    target_pos=target_pos_isaac,
                    roll_deg=roll_deg,
                    pitch_deg=pitch_deg,
                )
                if not is_safe:
                    estop_triggered = True
                    estop_reason = safety.emergency_reason or "safety_guard"
                    break

                corrected_action = (safe_target - obs_builder.DEFAULT_JOINT_POS) / action_scale
                servo_targets = obs_builder.action_to_servo_targets(corrected_action)

                if not args.dry_run:
                    bridge.send_servo_targets(
                        servo_targets,
                        speed=int(args.servo_speed),
                        time_ms=int(args.servo_time_ms),
                    )

                if step % max(1, int(args.print_every)) == 0:
                    a_mean = float(np.mean(np.abs(action_cmd)))
                    a_raw_mean = float(np.mean(np.abs(action_raw)))
                    print(
                        f"step={step:4d} |a|={a_mean:.3f} |a_raw|={a_raw_mean:.3f} gain={gain_used:.2f} "
                        f"roll={roll_deg:+5.1f} pitch={pitch_deg:+5.1f} "
                        f"tgt0={int(servo_targets[0])} {'[DRY]' if args.dry_run else ''}"
                    )
                    if args.print_joint_vel:
                        vel_arr = np.array(state["servo_vel"], dtype=np.float32)
                        pos_arr = np.array(state["servo_pos"], dtype=np.float32)
                        obs_joint_vel = np.array(obs_raw[24:36], dtype=np.float32)
                        cur_raw = np.round(pos_arr / (2 * np.pi) * 4096 + 2048).astype(np.int32)
                        track_err = servo_targets.astype(np.int32) - cur_raw
                        vel_text = np.array2string(vel_arr, precision=3, suppress_small=False, max_line_width=200)
                        pos_text = np.array2string(pos_arr, precision=3, suppress_small=False, max_line_width=200)
                        obs_vel_text = np.array2string(obs_joint_vel, precision=3, suppress_small=False, max_line_width=200)
                        print(f"    servo_vel(rad/s)={vel_text}  vel_norm={float(np.linalg.norm(vel_arr)):.3f}")
                        print(f"    obs_joint_vel_used(rad/s)={obs_vel_text}  obs_vel_norm={float(np.linalg.norm(obs_joint_vel)):.3f}")
                        print(
                            f"    servo_pos(rad)={pos_text}  max|target-cur|={int(np.max(np.abs(track_err)))}"
                        )

                next_tick += control_dt
                sleep_s = next_tick - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.perf_counter()

    finally:
        if not args.dry_run:
            zero_targets = obs_builder.action_to_servo_targets(np.zeros(12, dtype=np.float32))
            bridge.send_servo_targets(
                zero_targets,
                speed=int(args.servo_speed),
                time_ms=int(args.servo_time_ms),
            )
            time.sleep(0.2)
            bridge.set_torque(False)
            bridge.disconnect()

    elapsed = time.perf_counter() - t_start
    if estop_triggered:
        print(f"[ESTOP] 已触发: {estop_reason}")
        return 2

    print(f"[PASS] 完成 {elapsed:.2f}s 测试，未触发急停")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
