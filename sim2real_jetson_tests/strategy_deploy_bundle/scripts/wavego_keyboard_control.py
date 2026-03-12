"""
WAVEGO 键盘遥控器 - 通过 WASD+QEJL+UO 控制机器狗

按键说明:
    W/S   - 前进/后退 (cmd_x)
    A/D   - 左/右平移 (cmd_y)
    Q/E   - 加速/减速 (调整基础速度倍率)
    J/L   - 左转/右转 (cmd_wz)
    U/O   - 增加/减小旋转速度倍率
    Space - 停止所有运动
    Ctrl+C - 退出程序

限制:
    行走速度 <= 0.8 m/s
    旋转速度 <= 0.5 rad/s

依赖:
    pip install numpy pyserial pyyaml onnxruntime keyboard
"""

from __future__ import annotations

import argparse
import os
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
    import select
    import tty
    import termios
    import sys
except ImportError:
    print("[ERROR] 此脚本需要在 Linux 上运行")
    sys.exit(1)

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


class KeyboardController:
    """键盘遥控器状态管理 - 使用 select + termios 实现非阻塞读取"""
    
    def __init__(self):
        self.lock = Lock()
        
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_wz = 0.0
        
        self.base_speed = 0.3
        self.rot_speed = 0.2
        
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    
    def __del__(self):
        self._restore()
    
    def _restore(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
        except:
            pass
    
    def _process_key(self, ch):
        with self.lock:
            if ch == 'w':
                self.cmd_x = self.base_speed
            elif ch == 's':
                self.cmd_x = -self.base_speed
            elif ch == 'a':
                self.cmd_y = self.base_speed
            elif ch == 'd':
                self.cmd_y = -self.base_speed
            elif ch == 'q':
                self.base_speed = min(self.base_speed + BASE_SPEED_INC, MAX_WALK_SPEED)
                self._update_xy()
                print(f"[Speed] 基础速度: {self.base_speed:.1f}")
            elif ch == 'e':
                self.base_speed = max(self.base_speed - BASE_SPEED_INC, 0.0)
                self._update_xy()
                print(f"[Speed] 基础速度: {self.base_speed:.1f}")
            elif ch == 'j':
                self.cmd_wz = self.rot_speed
            elif ch == 'l':
                self.cmd_wz = -self.rot_speed
            elif ch == 'u':
                self.rot_speed = min(self.rot_speed + ROT_SPEED_INC, MAX_ROT_SPEED)
                self._update_wz()
                print(f"[Rot] 旋转速度: {self.rot_speed:.2f}")
            elif ch == 'o':
                self.rot_speed = max(self.rot_speed - ROT_SPEED_INC, 0.0)
                self._update_wz()
                print(f"[Rot] 旋转速度: {self.rot_speed:.2f}")
            elif ch == ' ':
                self.cmd_x = 0.0
                self.cmd_y = 0.0
                self.cmd_wz = 0.0
                print("[Control] 停止!")
            elif ch == '1':
                self.base_speed = 0.1
                self._update_xy()
                print(f"[Speed] 慢速模式: {self.base_speed:.1f}")
            elif ch == '2':
                self.base_speed = 0.3
                self._update_xy()
                print(f"[Speed] 中速模式: {self.base_speed:.1f}")
            elif ch == '3':
                self.base_speed = 0.5
                self._update_xy()
                print(f"[Speed] 快速模式: {self.base_speed:.1f}")
            elif ch == '\x03':
                raise KeyboardInterrupt
    
    def _update_xy(self):
        if self.cmd_x > 0:
            self.cmd_x = self.base_speed
        elif self.cmd_x < 0:
            self.cmd_x = -self.base_speed
        
        if self.cmd_y > 0:
            self.cmd_y = self.base_speed
        elif self.cmd_y < 0:
            self.cmd_y = -self.base_speed
    
    def _update_wz(self):
        if self.cmd_wz > 0:
            self.cmd_wz = self.rot_speed
        elif self.cmd_wz < 0:
            self.cmd_wz = -self.rot_speed
    
    def check_key(self):
        """检查是否有键盘输入"""
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        if dr:
            ch = sys.stdin.read(1)
            self._process_key(ch)
    
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
                'cmd_x': self.cmd_x,
                'cmd_y': self.cmd_y,
                'cmd_wz': self.cmd_wz,
                'base_speed': self.base_speed,
                'rot_speed': self.rot_speed,
            }


class ONNXPolicy:
    def __init__(self, model_path: str, use_gpu: bool = True):
        if ort is None:
            self.session = None
            return
        
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if use_gpu else ['CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)
        self.inp_name = self.session.get_inputs()[0].name
    
    def predict(self, obs: np.ndarray) -> np.ndarray:
        if self.session is None:
            return np.zeros(NUM_ACTIONS, dtype=np.float32)
        return self.session.run(None, {self.inp_name: obs.reshape(1, -1)})[0].squeeze()


class ObsNormalizer:
    def __init__(self, stats_path: str, clip: float = 10.0, eps: float = 1e-9):
        data = np.load(stats_path)
        self.mean = data['mean']
        self.std = data['std']
        self.clip = clip
        self.eps = eps
    
    def normalize(self, obs: np.ndarray) -> np.ndarray:
        return np.clip((obs - self.mean) / (self.std + self.eps), -self.clip, self.clip)


def main():
    parser = argparse.ArgumentParser(description="WAVEGO 键盘遥控模式")
    parser.add_argument("--config", type=str, default="config/wavego_deploy_config.yaml")
    parser.add_argument("--dry-run", action="store_true", help="只推理不发指令")
    parser.add_argument("--no-gpu", action="store_true", help="强制 CPU 推理")
    parser.add_argument("--port", type=str, default=None, help="覆盖串口设备")
    parser.add_argument("--servo-speed", type=int, default=3400, help="舵机速度参数")
    parser.add_argument("--servo-time-ms", type=int, default=120, help="舵机插值时间ms")
    parser.add_argument("--init-wait", type=float, default=1.0, help="初始站姿等待时间(秒)")
    parser.add_argument("--action-lpf-alpha", type=float, default=None, help="动作低通alpha(0~1)")
    parser.add_argument("--print-every", type=int, default=20, help="日志打印步数间隔")
    parser.add_argument("--print-joint-vel", action="store_true", help="打印关节速度")
    args = parser.parse_args()
    
    print("\n" + "=" * 60)
    print("  WAVEGO 键盘遥控模式")
    print("=" * 60)
    print("按键说明:")
    print("  W/S   - 前进/后退")
    print("  A/D   - 左/右平移")
    print("  Q/E   - 加速/减速 (当前: 0.3)")
    print("  J/L   - 左转/右转")
    print("  U/O   - 增加/减小旋转速度 (当前: 0.2)")
    print("  Space - 停止所有运动")
    print("  Ctrl+C - 退出")
    print("=" * 60 + "\n")
    
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
    if not args.dry_run:
        hw_cfg = cfg["hardware"]
        port = args.port or hw_cfg["serial_port"]
        bridge = RobotIOBridge(
            port=port,
            baudrate=hw_cfg["serial_baud"],
        )
        if not bridge.connect():
            print(f"[ERROR] 无法连接 STM32，退出。识别描述符: {port}")
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
    
    print(f"Control: {control_hz} Hz, dt={control_dt*1000:.0f}ms")
    print(f"Action LPF alpha={action_alpha:.2f}")
    print(f"Servo: speed={args.servo_speed}, time_ms={args.servo_time_ms}")
    print(f"Speed limits: walk={MAX_WALK_SPEED}, rot={MAX_ROT_SPEED}")
    
    keyboard_controller = KeyboardController()
    time.sleep(0.5)
    
    action_prev = np.zeros(NUM_ACTIONS, dtype=np.float32)
    zero_action = np.zeros(NUM_ACTIONS, dtype=np.float32)
    target_prev = None
    step = 0
    t_start = time.perf_counter()
    running = True

    def clip_action_to_joint_limits(raw_action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        target_pos_isaac = obs_builder.DEFAULT_JOINT_POS + raw_action * action_scale
        target_pos_isaac = np.clip(target_pos_isaac, limits_low, limits_high)
        corrected = (target_pos_isaac - obs_builder.DEFAULT_JOINT_POS) / action_scale
        return target_pos_isaac, corrected
    
    def signal_handler(sig, frame):
        nonlocal running
        print("\n[INFO] 收到退出信号，正在停止...")
        running = False
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if bridge:
        bridge.set_torque(True)
        print(f"[INFO] RobotIOBridge 已连接, port={bridge.port}")
        time.sleep(0.3)
        _, zero_corrected = clip_action_to_joint_limits(zero_action)
        zero_targets = obs_builder.action_to_servo_targets(zero_corrected)
        bridge.send_servo_targets(zero_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)
        if args.init_wait > 0:
            time.sleep(args.init_wait)
    
    print("\n[INFO] 开始键盘控制，按键即可控制机器狗！\n")
    
    obs_builder.reset()

    try:
        while running:
            t_loop_start = time.perf_counter()
            
            keyboard_controller.check_key()
            command = keyboard_controller.get_command()
            status = keyboard_controller.get_status()
            
            state = bridge.get_state() if not args.dry_run else {
                "servo_pos": np.zeros(12, dtype=np.float32),
                "servo_vel": np.zeros(12, dtype=np.float32),
                "imu_roll": 0.0, "imu_pitch": 0.0, "imu_yaw": 0.0,
                "imu_gyro": np.zeros(3), "imu_accel": np.zeros(3),
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
                print(f"[EMERGENCY] 异常急停! reason={safety.emergency_reason}")
                break
            
            corrected = (target_pos_isaac - obs_builder.DEFAULT_JOINT_POS) / action_scale
            if not args.dry_run:
                servo_targets = obs_builder.action_to_servo_targets(corrected)
                bridge.send_servo_targets(servo_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)
            else:
                servo_targets = obs_builder.action_to_servo_targets(corrected)
            
            target_prev = servo_targets.copy()
            step += 1
            t_now = time.perf_counter()
            loop_ms = (t_now - t_loop_start) * 1000
            
            if step % args.print_every == 0:
                elapsed = t_now - t_start
                hz_avg = step / elapsed if elapsed > 0 else 0
                act_mag = float(np.abs(action_cmd).mean())
                print(
                    f"step={step:5d} t={elapsed:6.1f}s hz={hz_avg:5.1f} |a|={act_mag:.3f} "
                    f"vx={status['cmd_x']:+.2f} vy={status['cmd_y']:+.2f} wz={status['cmd_wz']:+.2f} "
                    f"spd={status['base_speed']:.1f} rot={status['rot_speed']:.2f} "
                    f"roll={roll_deg:+5.1f} pitch={pitch_deg:+5.1f}"
                )
                
                if args.print_joint_vel and not args.dry_run:
                    vel_arr = np.array(state["servo_vel"], dtype=np.float32)
                    print(f"  servo_vel={np.array2string(vel_arr, precision=2, max_line_width=120)}")
            
            sleep_time = control_dt - (time.perf_counter() - t_loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n[INFO] 键盘中断，退出")
    
    finally:
        keyboard_controller._restore()
        if bridge:
            _, zero_corrected = clip_action_to_joint_limits(zero_action)
            zero_targets = obs_builder.action_to_servo_targets(zero_corrected)
            bridge.send_servo_targets(zero_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)
            time.sleep(0.3)
            bridge.set_torque(False)
            bridge.disconnect()
        print("[INFO] 程序结束")


if __name__ == "__main__":
    main()
