"""
WAVEGO keyboard teleop for policy deployment.

Controls:
    Hold W/S   - forward / backward
    Hold A/D   - left / right
    Hold J/L   - turn left / right
    Q/E        - increase / decrease walk speed
    U/O        - increase / decrease turn speed
    1/2/3      - walk speed presets
    Space      - stop all motion
    Ctrl+C     - exit
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path
from threading import Lock

import numpy as np
import yaml

try:
    import onnxruntime as ort
except ImportError:
    ort = None
    print("[WARN] onnxruntime not installed, will use dummy policy")

try:
    import keyboard as keyboard_lib
except ImportError:
    keyboard_lib = None

SCRIPT_DIR = Path(__file__).resolve().parent
TESTS_DIR = SCRIPT_DIR.parent.parent
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(TESTS_DIR))

try:
    from obs_builder import ObsBuilder
    from robotio_bridge import RobotIOBridge
    from safety_guard import SafetyGuard as SharedSafetyGuard
except ImportError as e:
    print(f"[ERROR] 导入失败: {e}")
    print("  请确认 obs_builder.py / robotio_bridge.py / safety_guard.py 可导入")
    sys.exit(1)

NUM_OBS = 48
NUM_ACTIONS = 12

MAX_WALK_SPEED = 0.8
MAX_ROT_SPEED = 0.5
BASE_SPEED_INC = 0.1
ROT_SPEED_INC = 0.05
STATUS_REFRESH_DT = 0.1


class KeyboardController:
    """Track key press/release state so motion only happens while keys are held."""

    MOTION_KEYS = ("w", "a", "s", "d", "j", "l")
    ACTION_KEYS = MOTION_KEYS + ("q", "e", "u", "o", "1", "2", "3", "space")

    def __init__(self):
        if keyboard_lib is None:
            raise RuntimeError("missing python package `keyboard`")

        self.lock = Lock()
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_wz = 0.0
        self.base_speed = 0.3
        self.rot_speed = 0.2
        self.message = "按住 WASD / JL 运动，松开即停止"
        self.active_keys: set[str] = set()
        self._latched_keys: set[str] = set()
        self._hooks = []

        try:
            for key in self.ACTION_KEYS:
                self._hooks.append(
                    keyboard_lib.on_press_key(
                        key,
                        lambda _event, bound_key=key: self._on_press(bound_key),
                    )
                )
                self._hooks.append(
                    keyboard_lib.on_release_key(
                        key,
                        lambda _event, bound_key=key: self._on_release(bound_key),
                    )
                )
        except Exception as exc:
            self._restore()
            raise RuntimeError(f"keyboard hook init failed: {exc}") from exc

    def __del__(self):
        self._restore()

    def _restore(self):
        if keyboard_lib is None:
            return
        for hook in self._hooks:
            try:
                keyboard_lib.unhook(hook)
            except Exception:
                pass
        self._hooks.clear()

    def _set_message_locked(self, message: str):
        self.message = message

    def set_message(self, message: str):
        with self.lock:
            self._set_message_locked(message)

    def _recompute_motion_locked(self):
        forward = "w" in self.active_keys
        backward = "s" in self.active_keys
        left = "a" in self.active_keys
        right = "d" in self.active_keys
        turn_left = "j" in self.active_keys
        turn_right = "l" in self.active_keys

        self.cmd_x = self.base_speed if forward and not backward else -self.base_speed if backward and not forward else 0.0
        self.cmd_y = self.base_speed if left and not right else -self.base_speed if right and not left else 0.0
        self.cmd_wz = self.rot_speed if turn_left and not turn_right else -self.rot_speed if turn_right and not turn_left else 0.0

    def _on_press(self, key: str):
        with self.lock:
            if key in self.MOTION_KEYS:
                self.active_keys.add(key)
                self._recompute_motion_locked()
                return

            if key in self._latched_keys:
                return
            self._latched_keys.add(key)

            if key == "q":
                self.base_speed = min(self.base_speed + BASE_SPEED_INC, MAX_WALK_SPEED)
                self._recompute_motion_locked()
                self._set_message_locked(f"基础速度调至 {self.base_speed:.1f} m/s")
            elif key == "e":
                self.base_speed = max(self.base_speed - BASE_SPEED_INC, 0.0)
                self._recompute_motion_locked()
                self._set_message_locked(f"基础速度调至 {self.base_speed:.1f} m/s")
            elif key == "u":
                self.rot_speed = min(self.rot_speed + ROT_SPEED_INC, MAX_ROT_SPEED)
                self._recompute_motion_locked()
                self._set_message_locked(f"旋转速度调至 {self.rot_speed:.2f} rad/s")
            elif key == "o":
                self.rot_speed = max(self.rot_speed - ROT_SPEED_INC, 0.0)
                self._recompute_motion_locked()
                self._set_message_locked(f"旋转速度调至 {self.rot_speed:.2f} rad/s")
            elif key == "space":
                self.active_keys.clear()
                self._recompute_motion_locked()
                self._set_message_locked("已停止所有运动")
            elif key == "1":
                self.base_speed = 0.1
                self._recompute_motion_locked()
                self._set_message_locked("切换到慢速模式 0.1 m/s")
            elif key == "2":
                self.base_speed = 0.3
                self._recompute_motion_locked()
                self._set_message_locked("切换到中速模式 0.3 m/s")
            elif key == "3":
                self.base_speed = 0.5
                self._recompute_motion_locked()
                self._set_message_locked("切换到快速模式 0.5 m/s")

    def _on_release(self, key: str):
        with self.lock:
            if key in self.MOTION_KEYS:
                self.active_keys.discard(key)
                self._recompute_motion_locked()
            else:
                self._latched_keys.discard(key)

    def get_command(self) -> np.ndarray:
        with self.lock:
            cmd = np.array([self.cmd_x, self.cmd_y, self.cmd_wz], dtype=np.float32)

        cmd[0] = np.clip(cmd[0], -MAX_WALK_SPEED, MAX_WALK_SPEED)
        cmd[1] = np.clip(cmd[1], -MAX_WALK_SPEED, MAX_WALK_SPEED)
        cmd[2] = np.clip(cmd[2], -MAX_ROT_SPEED, MAX_ROT_SPEED)

        return np.array([-cmd[1], cmd[0], cmd[2]], dtype=np.float32)

    def get_status(self) -> dict:
        with self.lock:
            return {
                "cmd_x": self.cmd_x,
                "cmd_y": self.cmd_y,
                "cmd_wz": self.cmd_wz,
                "base_speed": self.base_speed,
                "rot_speed": self.rot_speed,
                "held_keys": " ".join(key.upper() for key in self.MOTION_KEYS if key in self.active_keys) or "-",
                "message": self.message,
            }


class ONNXPolicy:
    def __init__(self, model_path: str, use_gpu: bool = True):
        if ort is None:
            self.session = None
            return

        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if use_gpu else ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(model_path, providers=providers)
        self.inp_name = self.session.get_inputs()[0].name

    def predict(self, obs: np.ndarray) -> np.ndarray:
        if self.session is None:
            return np.zeros(NUM_ACTIONS, dtype=np.float32)
        return self.session.run(None, {self.inp_name: obs.reshape(1, -1)})[0].squeeze()


class ObsNormalizer:
    def __init__(self, stats_path: str, clip: float = 10.0, eps: float = 1e-9):
        data = np.load(stats_path)
        self.mean = data["mean"]
        self.std = data["std"]
        self.clip = clip
        self.eps = eps

    def normalize(self, obs: np.ndarray) -> np.ndarray:
        return np.clip((obs - self.mean) / (self.std + self.eps), -self.clip, self.clip)


class TerminalStatusPanel:
    """Keep a fixed guide on screen and refresh only the status area."""

    def __init__(self):
        self.enabled = sys.stdout.isatty()
        self.status_row = 1

    def start(self, header_lines: list[str]):
        if self.enabled:
            sys.stdout.write("\033[2J\033[H\033[?25l")
        sys.stdout.write("\n".join(header_lines) + "\n")
        sys.stdout.flush()
        self.status_row = len(header_lines) + 1

    def update(self, status_lines: list[str]):
        if self.enabled:
            sys.stdout.write(f"\033[{self.status_row};1H\033[J")
        sys.stdout.write("\n".join(status_lines) + "\n")
        sys.stdout.flush()

    def close(self, final_lines: list[str] | None = None):
        lines = final_lines or []
        if self.enabled:
            sys.stdout.write(f"\033[{self.status_row};1H\033[J")
            if lines:
                sys.stdout.write("\n".join(lines) + "\n")
            sys.stdout.write("\033[?25h")
            sys.stdout.flush()
            return
        if lines:
            print("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description="WAVEGO 键盘遥控模式")
    parser.add_argument("--config", type=str, default="config/wavego_deploy_config.yaml")
    parser.add_argument("--dry-run", action="store_true", help="只推理不发指令")
    parser.add_argument("--no-gpu", action="store_true", help="强制 CPU 推理")
    parser.add_argument("--port", type=str, default=None, help="覆盖串口设备")
    parser.add_argument("--servo-speed", type=int, default=3400, help="舵机速度参数")
    parser.add_argument("--servo-time-ms", type=int, default=120, help="舵机插值时间 ms")
    parser.add_argument("--init-wait", type=float, default=1.0, help="初始站姿等待时间(秒)")
    parser.add_argument("--action-lpf-alpha", type=float, default=None, help="动作低通 alpha(0~1)")
    parser.add_argument("--print-every", type=int, default=20, help="保留参数，状态面板改为固定刷新")
    parser.add_argument("--print-joint-vel", action="store_true", help="在状态面板中显示关节速度数组")
    args = parser.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = Path(__file__).parent.parent / cfg_path
    if not cfg_path.exists():
        print(f"[ERROR] 配置文件不存在: {cfg_path}")
        sys.exit(1)

    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    sim2real_root = cfg_path.parent.parent
    control_dt = cfg["timing"]["control_dt_s"]
    control_hz = cfg["timing"]["control_freq_hz"]
    action_scale = cfg["policy"]["action_scale"]
    limits_low = np.array(cfg["joints"]["limits_low_policy"], dtype=np.float32)
    limits_high = np.array(cfg["joints"]["limits_high_policy"], dtype=np.float32)
    cfg_alpha = float(cfg.get("filters", {}).get("action_lpf_alpha", 1.0))
    action_alpha = float(args.action_lpf_alpha if args.action_lpf_alpha is not None else cfg_alpha)
    action_alpha = max(0.0, min(1.0, action_alpha))

    normalizer = ObsNormalizer(
        str(sim2real_root / cfg["model"]["normalizer_path"]),
        clip=cfg["policy"]["normalizer_clip"],
        eps=cfg["policy"]["normalizer_eps"],
    )

    policy = ONNXPolicy(
        str(sim2real_root / cfg["model"]["onnx_path"]),
        use_gpu=not args.no_gpu,
    )

    bridge = None
    port_text = "dry-run"
    if not args.dry_run:
        hw_cfg = cfg["hardware"]
        port_text = args.port or hw_cfg["serial_port"]
        bridge = RobotIOBridge(
            port=port_text,
            baudrate=hw_cfg["serial_baud"],
        )
        if not bridge.connect():
            print(f"[ERROR] 无法连接 STM32，退出。识别描述符: {port_text}")
            sys.exit(1)

    obs_builder = ObsBuilder(cfg.get("observation", {}))
    safety_cfg = cfg["safety"]
    safety = SharedSafetyGuard(
        joint_limits_low=limits_low,
        joint_limits_high=limits_high,
        max_action_delta=float(safety_cfg.get("max_action_delta", 0.5)),
        max_pitch_deg=float(safety_cfg.get("max_pitch_deg", 45.0)),
        max_roll_deg=float(safety_cfg.get("max_roll_deg", 45.0)),
        heartbeat_timeout=0.2,
    )

    try:
        keyboard_controller = KeyboardController()
    except RuntimeError as exc:
        print(f"[ERROR] 键盘监听初始化失败: {exc}")
        print("  请先 `pip install keyboard`，并在 Linux 上以 root 或具备 input 设备访问权限的用户运行。")
        sys.exit(1)

    time.sleep(0.2)

    action_prev = np.zeros(NUM_ACTIONS, dtype=np.float32)
    zero_action = np.zeros(NUM_ACTIONS, dtype=np.float32)
    target_prev = None
    step = 0
    t_start = time.perf_counter()
    running = True
    shutdown_reason = "正常退出"

    def clip_action_to_joint_limits(raw_action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        target_pos_isaac = obs_builder.DEFAULT_JOINT_POS + raw_action * action_scale
        target_pos_isaac = np.clip(target_pos_isaac, limits_low, limits_high)
        corrected = (target_pos_isaac - obs_builder.DEFAULT_JOINT_POS) / action_scale
        return target_pos_isaac, corrected

    def signal_handler(_sig, _frame):
        nonlocal running, shutdown_reason
        shutdown_reason = "收到退出信号，正在停止"
        keyboard_controller.set_message(shutdown_reason)
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if bridge:
        bridge.set_torque(True)
        time.sleep(0.3)
        _, zero_corrected = clip_action_to_joint_limits(zero_action)
        zero_targets = obs_builder.action_to_servo_targets(zero_corrected)
        bridge.send_servo_targets(zero_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)
        if args.init_wait > 0:
            time.sleep(args.init_wait)
        keyboard_controller.set_message("站姿初始化完成，可以开始遥控")

    obs_builder.reset()

    panel = TerminalStatusPanel()
    header_lines = [
        "WAVEGO Keyboard Teleop",
        "=" * 64,
        "按住 W/S 前后，按住 A/D 左右，按住 J/L 旋转，松开即停止",
        "Q/E 调整行走速度，U/O 调整旋转速度，1/2/3 速度档位，Space 急停，Ctrl+C 退出",
        f"模式: {'DRY-RUN' if args.dry_run else 'HARDWARE'} | 端口: {port_text} | 控制频率: {control_hz} Hz | dt: {control_dt*1000:.0f} ms",
        f"动作低通 alpha: {action_alpha:.2f} | 舵机 speed={args.servo_speed} time_ms={args.servo_time_ms}",
        f"速度上限: walk={MAX_WALK_SPEED:.2f} m/s | rot={MAX_ROT_SPEED:.2f} rad/s",
        "-" * 64,
    ]
    panel.start(header_lines)

    last_render_time = 0.0
    last_loop_ms = 0.0
    last_roll_deg = 0.0
    last_pitch_deg = 0.0
    last_action_mag = 0.0
    last_tgt_jump = 0
    last_vel_text = "-"

    def render_status(now: float, status: dict, command: np.ndarray):
        elapsed = now - t_start
        hz_avg = step / elapsed if elapsed > 0 else 0.0
        lines = [
            f"状态    : {'EMERGENCY' if safety.is_emergency else 'RUNNING'} | 步数={step} | 运行={elapsed:6.1f}s | 平均频率={hz_avg:5.1f} Hz | 回路={last_loop_ms:5.1f} ms",
            f"按键    : {status['held_keys']:<12} | 提示: {status['message']}",
            f"命令    : vx={status['cmd_x']:+.2f} m/s | vy={status['cmd_y']:+.2f} m/s | wz={status['cmd_wz']:+.2f} rad/s | net=({command[0]:+.2f}, {command[1]:+.2f}, {command[2]:+.2f})",
            f"速度档  : walk={status['base_speed']:.2f} m/s | rot={status['rot_speed']:.2f} rad/s | |a|={last_action_mag:.3f} | max|Δtarget|={last_tgt_jump}",
            f"姿态    : roll={last_roll_deg:+5.1f} deg | pitch={last_pitch_deg:+5.1f} deg",
        ]
        if args.print_joint_vel:
            lines.append(f"关节速度: {last_vel_text}")
        panel.update(lines)

    initial_now = time.perf_counter()
    render_status(initial_now, keyboard_controller.get_status(), keyboard_controller.get_command())
    last_render_time = initial_now

    try:
        while running:
            t_loop_start = time.perf_counter()

            command = keyboard_controller.get_command()
            status = keyboard_controller.get_status()

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

            obs_raw = obs_builder.build_observation(state, command, control_dt)
            obs_norm = normalizer.normalize(obs_raw)

            action_raw = policy.predict(obs_norm)
            action_cmd = action_alpha * action_raw + (1.0 - action_alpha) * action_prev
            action_prev = action_cmd.copy()

            target_pos_isaac, corrected = clip_action_to_joint_limits(action_cmd)

            gx, gy, gz = obs_raw[6:9]
            roll_deg = float(np.degrees(np.arctan2(gy, -gz)))
            pitch_deg = float(np.degrees(np.arctan2(-gx, np.sqrt(gy**2 + gz**2))))
            target_pos_isaac, is_safe = safety.check_and_clip(target_pos_isaac, roll_deg, pitch_deg)
            if not is_safe:
                shutdown_reason = f"异常急停: {safety.emergency_reason}"
                keyboard_controller.set_message(shutdown_reason)
                running = False
                continue

            corrected = (target_pos_isaac - obs_builder.DEFAULT_JOINT_POS) / action_scale
            servo_targets = obs_builder.action_to_servo_targets(corrected)
            if not args.dry_run:
                bridge.send_servo_targets(servo_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)

            last_tgt_jump = 0 if target_prev is None else int(np.max(np.abs(servo_targets - target_prev)))
            target_prev = servo_targets.copy()
            step += 1

            t_now = time.perf_counter()
            last_loop_ms = (t_now - t_loop_start) * 1000.0
            last_roll_deg = roll_deg
            last_pitch_deg = pitch_deg
            last_action_mag = float(np.abs(action_cmd).mean())
            if args.print_joint_vel:
                vel_arr = np.array(state["servo_vel"], dtype=np.float32)
                last_vel_text = np.array2string(vel_arr, precision=2, max_line_width=160)

            if (t_now - last_render_time) >= STATUS_REFRESH_DT:
                render_status(t_now, status, command)
                last_render_time = t_now

            sleep_time = control_dt - (time.perf_counter() - t_loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        shutdown_reason = "键盘中断，正在退出"
        keyboard_controller.set_message(shutdown_reason)

    finally:
        keyboard_controller._restore()
        if bridge:
            _, zero_corrected = clip_action_to_joint_limits(zero_action)
            zero_targets = obs_builder.action_to_servo_targets(zero_corrected)
            bridge.send_servo_targets(zero_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)
            time.sleep(0.3)
            bridge.set_torque(False)
            bridge.disconnect()

        final_status = keyboard_controller.get_status()
        final_command = keyboard_controller.get_command()
        render_status(time.perf_counter(), final_status, final_command)
        panel.close([f"退出: {shutdown_reason}"])


if __name__ == "__main__":
    main()
