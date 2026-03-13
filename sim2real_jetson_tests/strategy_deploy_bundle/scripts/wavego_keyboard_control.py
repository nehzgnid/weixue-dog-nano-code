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
import glob
import os
import select
import signal
import struct
import sys
import termios
import time
import tty
from pathlib import Path
from threading import Event, Lock, Thread

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
    print(f"[ERROR] import failed: {e}")
    print("  Please ensure obs_builder.py / robotio_bridge.py / safety_guard.py are importable")
    sys.exit(1)

NUM_OBS = 48
NUM_ACTIONS = 12

MAX_WALK_SPEED = 1.5
MAX_ROT_SPEED = 1.5
BASE_SPEED_INC = 0.1
ROT_SPEED_INC = 0.05
STATUS_REFRESH_DT = 0.1
TTY_REPEAT_TIMEOUT_S = 0.60


class KeyboardController:
    """Track key press/release state so motion only happens while keys are held."""

    MOTION_KEYS = ("w", "a", "s", "d", "j", "l")
    ACTION_KEYS = MOTION_KEYS + ("q", "e", "u", "o", "1", "2", "3", "space")

    EV_KEY = 0x01
    INPUT_EVENT = struct.Struct("llHHI")
    KEY_CODE_TO_NAME = {
        17: "w",
        30: "a",
        31: "s",
        32: "d",
        36: "j",
        38: "l",
        16: "q",
        18: "e",
        22: "u",
        24: "o",
        2: "1",
        3: "2",
        4: "3",
        57: "space",
    }

    def __init__(self):
        self.lock = Lock()
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_wz = 0.0
        self.base_speed = 0.3
        self.rot_speed = 0.2
        self.message = "Hold WASD / JL to move. Release to stop."
        self.active_keys: set[str] = set()
        self._latched_keys: set[str] = set()
        self._motion_seen_at: dict[str, float] = {}

        self._hooks = []
        self._fds: list[int] = []
        self._stop_event = Event()
        self._reader_thread: Thread | None = None
        self._stdin_fd: int | None = None
        self._stdin_termios = None
        self.backend = "uninitialized"

        backend_errors: list[str] = []
        for backend_name in self._backend_order():
            try:
                if backend_name == "tty-stdin":
                    self._init_tty_backend()
                elif backend_name == "linux-input":
                    self._init_linux_input_backend()
                elif backend_name == "python-keyboard":
                    self._init_keyboard_backend()
                else:
                    continue
                self.backend = backend_name
                self.message = f"Input backend: {backend_name}"
                return
            except Exception as exc:
                backend_errors.append(f"{backend_name} failed: {exc}")
                self._restore()

        if not backend_errors:
            backend_errors.append("no keyboard backend available")
        raise RuntimeError("; ".join(backend_errors))

    def __del__(self):
        self._restore()

    @staticmethod
    def _running_in_container() -> bool:
        return os.path.exists("/.dockerenv") or bool(os.environ.get("container"))

    def _backend_order(self) -> list[str]:
        forced = os.environ.get("WAVEGO_KEYBOARD_BACKEND", "auto").strip().lower()
        valid = {"auto", "tty", "linux-input", "python-keyboard"}
        if forced not in valid:
            forced = "auto"

        if forced == "tty":
            return ["tty-stdin"]
        if forced == "linux-input":
            return ["linux-input"]
        if forced == "python-keyboard":
            return ["python-keyboard"]

        prefers_tty = sys.stdin.isatty() and (
            self._running_in_container() or os.environ.get("TERM_PROGRAM", "").lower() == "vscode"
        )
        if prefers_tty:
            return ["tty-stdin", "linux-input", "python-keyboard"]
        return ["linux-input", "tty-stdin", "python-keyboard"]

    def _init_tty_backend(self):
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY")

        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)
        new_attr = termios.tcgetattr(fd)
        new_attr[3] &= ~(termios.ICANON | termios.ECHO)
        new_attr[6][termios.VMIN] = 0
        new_attr[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSADRAIN, new_attr)

        self._stdin_fd = fd
        self._stdin_termios = old_attr
        self._reader_thread = Thread(
            target=self._tty_input_loop,
            name="wavego-tty-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _init_keyboard_backend(self):
        if keyboard_lib is None:
            raise RuntimeError("python package `keyboard` not installed")

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

    def _init_linux_input_backend(self):
        if not sys.platform.startswith("linux"):
            raise RuntimeError("linux-input backend only works on Linux")

        event_paths = sorted(glob.glob("/dev/input/event*"))
        if not event_paths:
            raise RuntimeError("no /dev/input/event* devices found")

        for path in event_paths:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                continue
            self._fds.append(fd)

        if not self._fds:
            raise RuntimeError("unable to open any /dev/input/event* device")

        self._reader_thread = Thread(
            target=self._linux_input_loop,
            name="wavego-linux-input-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _tty_input_loop(self):
        assert self._stdin_fd is not None
        while not self._stop_event.is_set():
            try:
                ready, _, _ = select.select([self._stdin_fd], [], [], 0.05)
            except (OSError, ValueError):
                break

            now = time.monotonic()
            if ready:
                try:
                    data = os.read(self._stdin_fd, 64)
                except BlockingIOError:
                    data = b""
                except OSError:
                    break

                for value in data:
                    if value == 3:
                        signal.raise_signal(signal.SIGINT)
                        continue
                    if value in (10, 13):
                        continue
                    ch = chr(value).lower()
                    mapped = "space" if ch == " " else ch
                    if mapped in self.ACTION_KEYS:
                        self._on_press(mapped)

            self._expire_tty_motion_keys(now)

    def _linux_input_loop(self):
        pending = {fd: b"" for fd in self._fds}
        event_size = self.INPUT_EVENT.size

        while not self._stop_event.is_set():
            try:
                ready, _, _ = select.select(self._fds, [], [], 0.1)
            except (OSError, ValueError):
                break

            for fd in ready:
                try:
                    chunk = os.read(fd, event_size * 32)
                except BlockingIOError:
                    continue
                except OSError:
                    continue

                if not chunk:
                    continue

                data = pending[fd] + chunk
                while len(data) >= event_size:
                    event_raw = data[:event_size]
                    data = data[event_size:]
                    _sec, _usec, event_type, code, value = self.INPUT_EVENT.unpack(event_raw)
                    if event_type != self.EV_KEY:
                        continue
                    key_name = self.KEY_CODE_TO_NAME.get(code)
                    if key_name is None:
                        continue
                    if value == 0:
                        self._on_release(key_name)
                    elif value in (1, 2):
                        self._on_press(key_name)
                pending[fd] = data

    def _restore(self):
        self._stop_event.set()

        for hook in self._hooks:
            try:
                keyboard_lib.unhook(hook)
            except Exception:
                pass
        self._hooks.clear()

        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.2)
        self._reader_thread = None

        for fd in self._fds:
            try:
                os.close(fd)
            except OSError:
                pass
        self._fds.clear()

        if self._stdin_fd is not None and self._stdin_termios is not None:
            try:
                termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_termios)
            except Exception:
                pass
        self._stdin_fd = None
        self._stdin_termios = None

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

    def _expire_tty_motion_keys(self, now: float):
        if self.backend != "tty-stdin":
            return

        with self.lock:
            expired = [
                key for key in list(self.active_keys)
                if key in self.MOTION_KEYS and (now - self._motion_seen_at.get(key, 0.0)) > TTY_REPEAT_TIMEOUT_S
            ]
            if not expired:
                return
            for key in expired:
                self.active_keys.discard(key)
                self._motion_seen_at.pop(key, None)
            self._recompute_motion_locked()

    def _on_press(self, key: str):
        now = time.monotonic()
        with self.lock:
            if key in self.MOTION_KEYS:
                self.active_keys.add(key)
                self._motion_seen_at[key] = now
                self._recompute_motion_locked()
                return

            if self.backend != "tty-stdin" and key in self._latched_keys:
                return
            self._latched_keys.add(key)

            if key == "q":
                self.base_speed = min(self.base_speed + BASE_SPEED_INC, MAX_WALK_SPEED)
                self._recompute_motion_locked()
                self._set_message_locked(f"Walk speed set to {self.base_speed:.1f} m/s")
            elif key == "e":
                self.base_speed = max(self.base_speed - BASE_SPEED_INC, 0.0)
                self._recompute_motion_locked()
                self._set_message_locked(f"Walk speed set to {self.base_speed:.1f} m/s")
            elif key == "u":
                self.rot_speed = min(self.rot_speed + ROT_SPEED_INC, MAX_ROT_SPEED)
                self._recompute_motion_locked()
                self._set_message_locked(f"Turn speed set to {self.rot_speed:.2f} rad/s")
            elif key == "o":
                self.rot_speed = max(self.rot_speed - ROT_SPEED_INC, 0.0)
                self._recompute_motion_locked()
                self._set_message_locked(f"Turn speed set to {self.rot_speed:.2f} rad/s")
            elif key == "space":
                self.active_keys.clear()
                self._motion_seen_at.clear()
                self._recompute_motion_locked()
                self._set_message_locked("All motion stopped")
            elif key == "1":
                self.base_speed = 0.1
                self._recompute_motion_locked()
                self._set_message_locked("Walk speed preset: 0.1 m/s")
            elif key == "2":
                self.base_speed = 0.3
                self._recompute_motion_locked()
                self._set_message_locked("Walk speed preset: 0.3 m/s")
            elif key == "3":
                self.base_speed = 0.5
                self._recompute_motion_locked()
                self._set_message_locked("Walk speed preset: 0.5 m/s")

    def _on_release(self, key: str):
        with self.lock:
            if key in self.MOTION_KEYS:
                self.active_keys.discard(key)
                self._motion_seen_at.pop(key, None)
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
                "backend": self.backend,
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
    parser = argparse.ArgumentParser(description="WAVEGO keyboard teleop mode")
    parser.add_argument("--config", type=str, default="config/wavego_deploy_config.yaml")
    parser.add_argument("--dry-run", action="store_true", help="run policy only, do not send commands")
    parser.add_argument("--no-gpu", action="store_true", help="force CPU inference")
    parser.add_argument("--port", type=str, default=None, help="override serial device")
    parser.add_argument("--servo-speed", type=int, default=3400, help="servo speed parameter")
    parser.add_argument("--servo-time-ms", type=int, default=30, help="servo interpolation time in ms")
    parser.add_argument("--init-wait", type=float, default=1.0, help="initial stand wait time in seconds")
    parser.add_argument("--action-lpf-alpha", type=float, default=None, help="action low-pass alpha (0~1)")
    parser.add_argument("--print-every", type=int, default=20, help="kept for compatibility; status is refreshed in place")
    parser.add_argument("--print-joint-vel", action="store_true", help="show joint velocity array in the status panel")
    args = parser.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = Path(__file__).parent.parent / cfg_path
    if not cfg_path.exists():
        print(f"[ERROR] config file not found: {cfg_path}")
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
            print(f"[ERROR] unable to connect to STM32: {port_text}")
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
        print(f"[ERROR] keyboard listener init failed: {exc}")
        print("  In Docker/VSCode terminals this should use tty-stdin. If it still fails, check TTY availability and /dev/input permissions.")
        sys.exit(1)

    time.sleep(0.2)

    action_prev = np.zeros(NUM_ACTIONS, dtype=np.float32)
    zero_action = np.zeros(NUM_ACTIONS, dtype=np.float32)
    target_prev = None
    step = 0
    t_start = time.perf_counter()
    running = True
    shutdown_reason = "normal exit"
    expected_servo_ids = tuple(range(1, 13))

    def clip_action_to_joint_limits(raw_action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        target_pos_isaac = obs_builder.DEFAULT_JOINT_POS + raw_action * action_scale
        target_pos_isaac = np.clip(target_pos_isaac, limits_low, limits_high)
        corrected = (target_pos_isaac - obs_builder.DEFAULT_JOINT_POS) / action_scale
        return target_pos_isaac, corrected

    def signal_handler(_sig, _frame):
        nonlocal running, shutdown_reason
        shutdown_reason = "received exit signal, stopping"
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
        keyboard_controller.set_message("stand pose initialized; teleop ready")

    obs_builder.reset()

    panel = TerminalStatusPanel()
    header_lines = [
        "WAVEGO Keyboard Teleop",
        "=" * 72,
        "Hold W/S to move forward/back, hold A/D to move left/right, hold J/L to turn, release to stop",
        "Q/E adjust walk speed, U/O adjust turn speed, 1/2/3 speed presets, Space stop, Ctrl+C exit",
        f"Mode: {'DRY-RUN' if args.dry_run else 'HARDWARE'} | Port: {port_text} | Control: {control_hz} Hz | dt: {control_dt*1000:.0f} ms",
        f"LPF alpha: {action_alpha:.2f} | Servo speed={args.servo_speed} time_ms={args.servo_time_ms}",
        f"Limits: walk={MAX_WALK_SPEED:.2f} m/s | rot={MAX_ROT_SPEED:.2f} rad/s | Input backend={keyboard_controller.backend}",
        "-" * 72,
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
            f"State   : {'EMERGENCY' if safety.is_emergency else 'RUNNING'} | steps={step} | elapsed={elapsed:6.1f}s | avg_hz={hz_avg:5.1f} | loop={last_loop_ms:5.1f} ms",
            f"Keys    : {status['held_keys']:<12} | backend={status['backend']:<10} | note: {status['message']}",
            f"Command : vx={status['cmd_x']:+.2f} m/s | vy={status['cmd_y']:+.2f} m/s | wz={status['cmd_wz']:+.2f} rad/s | net=({command[0]:+.2f}, {command[1]:+.2f}, {command[2]:+.2f})",
            f"Speeds  : walk={status['base_speed']:.2f} m/s | rot={status['rot_speed']:.2f} rad/s | |a|={last_action_mag:.3f} | max|dtarget|={last_tgt_jump}",
            f"Attitude: roll={last_roll_deg:+5.1f} deg | pitch={last_pitch_deg:+5.1f} deg",
        ]
        if args.print_joint_vel:
            lines.append(f"JointVel: {last_vel_text}")
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

            if not args.dry_run:
                state_age = time.perf_counter() - state["timestamp"]
                servo_count = int(state.get("servo_count", 0))
                servo_ids = tuple(state.get("servo_ids", ()))
                if state_age > 0.2:
                    shutdown_reason = f"state timeout: {state_age:.3f}s"
                    keyboard_controller.set_message(shutdown_reason)
                    running = False
                    continue
                if servo_count != 12 or servo_ids != expected_servo_ids:
                    shutdown_reason = f"incomplete servo state: count={servo_count}, ids={servo_ids}"
                    keyboard_controller.set_message(shutdown_reason)
                    running = False
                    continue

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
                shutdown_reason = f"emergency stop: {safety.emergency_reason}"
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
        shutdown_reason = "keyboard interrupt, exiting"
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
        panel.close([f"Exit: {shutdown_reason}"])


if __name__ == "__main__":
    main()
