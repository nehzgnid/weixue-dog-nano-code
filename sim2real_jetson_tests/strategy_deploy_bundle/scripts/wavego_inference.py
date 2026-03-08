"""
WAVEGO Sim2Real 主推理循环 — Jetson Nano 上运行。

硬件架构:
    Jetson Nano → (USB CDC) → STM32F407 → (USART+DMA) → 12× STS3215 舵机
    Jetson Nano ← (USB CDC) ← STM32F407 ← (UART) ← WT61C IMU

用法:
    # 零指令站立测试（架空机器狗！）
    python scripts/wavego_inference.py \
        --config config/wavego_deploy_config.yaml \
        --cmd-x 0 --cmd-y 0 --cmd-wz 0 --duration 10

    # 慢速前进
    python scripts/wavego_inference.py \
        --config config/wavego_deploy_config.yaml \
        --cmd-x 0.3 --duration 30

依赖:
    pip install numpy pyserial pyyaml onnxruntime
"""

from __future__ import annotations

import argparse
import gc
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np
import yaml

try:
    import onnxruntime as ort
except ImportError:
    ort = None
    print("[WARN] onnxruntime not installed, will use dummy policy")

# 导入核心模块
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

# =============================================================================
# 常量
# =============================================================================
NUM_OBS = 48
NUM_ACTIONS = 12
GRAVITY_VEC_W = np.array([0.0, 0.0, -1.0], dtype=np.float32)



# =============================================================================
# 观测辅助函数（保留公共接口，内部改用 ObsBuilder 实现）
# =============================================================================

def quat_apply_inverse_wxyz(q_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """deprecated: 请改用 ObsBuilder._projected_gravity()"""
    q = q_wxyz / np.linalg.norm(q_wxyz)
    w, x, y, z = q
    q_vec = np.array([x, y, z], dtype=np.float32)
    t = 2.0 * np.cross(q_vec, vec)
    return vec + w * t + np.cross(q_vec, t)



# ==============================================================================
# Normalizer
# ==============================================================================

class ObsNormalizer:
    def __init__(self, stats_path: str, clip: float = 5.0, eps: float = 1e-8):
        data = np.load(stats_path)
        self.mean = data["mean"].astype(np.float32)
        self.std = data["std"].astype(np.float32)
        self.clip = clip
        self.eps = eps
        assert self.mean.shape == (NUM_OBS,), f"Mean shape: {self.mean.shape}"
        assert self.std.shape == (NUM_OBS,), f"Std shape: {self.std.shape}"

    def normalize(self, obs: np.ndarray) -> np.ndarray:
        normalized = (obs - self.mean) / (self.std + self.eps)
        return np.clip(normalized, -self.clip, self.clip)


# ==============================================================================
# ONNX Policy
# ==============================================================================

class ONNXPolicy:
    def __init__(self, onnx_path: str, use_gpu: bool = True):
        if ort is None:
            self.session = None
            self.input_name = None
            self.output_name = None
            print("[WARN] onnxruntime 不可用，启用 dummy policy（输出全零动作）")
            return
        providers = (
            ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if use_gpu
            else ["CPUExecutionProvider"]
        )
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


# ==============================================================================
# 传感器接口（占位实现，需根据实际硬件修改）
# ==============================================================================

class IMUReader:
    """
    [DEPRECATED] 旧版 MPU6050/BNO055 I2C 驱动。
    实际硬件: WT61C IMU 经 STM32 回传，请改用 STM32Bridge.get_state() 读取。
    """
    def __init__(self, *args, **kwargs):
        print("[WARN] IMUReader 已弃用，实际应改用 STM32Bridge")

    def read(self):
        return (
            np.zeros(3, dtype=np.float32),
            np.array([0, 0, -1], dtype=np.float32),
            np.zeros(3, dtype=np.float32),
        )


class ServoDriver:
    """
    [DEPRECATED] 旧版舵机驱动。
    实际硬件: 请改用 STM32Bridge.send_servo_targets()。
    """
    def __init__(self, *args, **kwargs):
        print("[WARN] ServoDriver 已弃用，实际应改用 STM32Bridge")

    def read_positions(self):
        return np.zeros(12, dtype=np.float32)

    def send_targets(self, *args):
        pass

    def close(self):
        pass






# =============================================================================
# 主循环——改用 RobotIOBridge + ObsBuilder
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="WAVEGO Sim2Real 出 (透传STM32)推理")
    parser.add_argument("--config", type=str, default="config/wavego_deploy_config.yaml")
    parser.add_argument("--cmd-x",  type=float, default=0, help="前进速度 m/s")
    parser.add_argument("--cmd-y",  type=float, default=0.2, help="侧移速度 m/s")
    parser.add_argument("--cmd-wz", type=float, default=0.0, help="转向 rad/s")
    parser.add_argument("--duration", type=float, default=5.0, help="运行时长 (秒)")
    parser.add_argument("--dry-run", action="store_true", help="只推理不发指令")
    parser.add_argument("--no-gpu",  action="store_true", help="强制 CPU 推理")
    parser.add_argument("--log-csv", type=str, default=None)
    parser.add_argument("--port",   type=str, default=None, help="覆盖串口设备")
    parser.add_argument("--servo-speed", type=int, default=3400, help="舵机速度参数")
    parser.add_argument("--servo-time-ms", type=int, default=120, help="舵机插值时间ms")
    parser.add_argument("--init-wait", type=float, default=1.0, help="初始站姿等待时间(秒)")
    parser.add_argument("--action-lpf-alpha", type=float, default=None, help="动作低通alpha(0~1)，默认读取配置")
    parser.add_argument("--print-every", type=int, default=None, help="日志打印步数间隔，默认读取配置")
    parser.add_argument("--verbose-debug", action="store_true", help="打印更多连续性诊断信息")
    parser.add_argument("--print-joint-vel", dest="print_joint_vel", action="store_true", help="打印12维关节速度数组(rad/s)")
    parser.add_argument("--no-print-joint-vel", dest="print_joint_vel", action="store_false", help="关闭12维关节速度打印")
    parser.set_defaults(print_joint_vel=None)
    args = parser.parse_args()

    # --- 加载配置 ---
    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = Path(__file__).parent.parent / cfg_path
    if not cfg_path.exists():
        print(f"[ERROR] 配置文件不存在: {cfg_path}")
        sys.exit(1)
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    sim2real_root = cfg_path.parent.parent
    control_dt    = cfg["timing"]["control_dt_s"]
    control_hz    = cfg["timing"]["control_freq_hz"]
    action_scale  = cfg["policy"]["action_scale"]
    limits_low    = np.array(cfg["joints"]["limits_low_policy"],  dtype=np.float32)
    limits_high   = np.array(cfg["joints"]["limits_high_policy"], dtype=np.float32)
    print_every   = int(args.print_every if args.print_every is not None else cfg["debug"]["print_every"])
    cfg_print_joint_vel = bool(cfg.get("debug", {}).get("print_joint_vel", True))
    print_joint_vel = cfg_print_joint_vel if args.print_joint_vel is None else args.print_joint_vel
    command       = np.array([args.cmd_x, args.cmd_y, args.cmd_wz], dtype=np.float32)
    cfg_alpha     = float(cfg.get("filters", {}).get("action_lpf_alpha", 1.0))
    action_alpha  = float(args.action_lpf_alpha if args.action_lpf_alpha is not None else cfg_alpha)
    action_alpha  = max(0.0, min(1.0, action_alpha))

    print(f"Command: vx={command[0]:.2f} vy={command[1]:.2f} wz={command[2]:.2f}")
    print(f"Control: {control_hz} Hz, dt={control_dt*1000:.0f}ms, 时长={args.duration}s")
    print(f"Servo cmd: speed={args.servo_speed}, time_ms={args.servo_time_ms}, init_wait={args.init_wait}s")
    print(f"Action LPF alpha={action_alpha:.2f}")
    print(f"Joint vel print: {'ON' if print_joint_vel else 'OFF'}")
    if np.all(np.abs(command) < 1e-6):
        print("[INFO] 当前速度命令为 0：策略将偏向平衡/姿态保持，不会表现持续步态。")

    # --- Normalizer ---
    normalizer = ObsNormalizer(
        str(sim2real_root / cfg["model"]["normalizer_path"]),
        clip=cfg["policy"]["normalizer_clip"],
        eps=cfg["policy"]["normalizer_eps"],
    )

    # --- ONNX Policy ---
    policy = ONNXPolicy(
        str(sim2real_root / cfg["model"]["onnx_path"]),
        use_gpu=not args.no_gpu,
    )

    # --- RobotIOBridge ---
    hw_cfg = cfg["hardware"]
    port = args.port or hw_cfg["serial_port"]
    bridge = RobotIOBridge(
        port=port,
        baudrate=hw_cfg["serial_baud"],
    )
    if not args.dry_run:
        if not bridge.connect():
            print(f"[ERROR] 无法连接 STM32，退出。识别描述符: {port}")
            sys.exit(1)

    # --- ObsBuilder ---
    obs_builder = ObsBuilder(cfg.get("observation", {}))

    # --- SafetyGuard (复用 scripts/safety_guard.py) ---
    safety_cfg = cfg["safety"]
    safety = SharedSafetyGuard(
        joint_limits_low=limits_low,
        joint_limits_high=limits_high,
        max_action_delta=float(safety_cfg.get("max_action_delta", 0.5)),
        max_pitch_deg=float(safety_cfg.get("max_pitch_deg", 45.0)),
        max_roll_deg=float(safety_cfg.get("max_roll_deg", 45.0)),
        heartbeat_timeout=0.2,
    )

    # --- 加载完成——打印确认信息 ---
    print(f"\n[硬件] STM32 端口: {port}")
    print(f"[ObsBuilder] servo_to_isaac: {list(obs_builder.servo_to_isaac)}")
    print(f"[ObsBuilder] joint_direction: {list(obs_builder.joint_direction)}")
    print(f"[ObsBuilder] base_lin_vel_mode: {obs_builder.base_lin_vel_mode}")
    print("⚠️  屋架空机器狗再启动!首次运行可能产生剧烈运动!")

    # --- 初始化舵机 ---
    if not args.dry_run:
        bridge.set_torque(True)
        time.sleep(0.3)
        # 先归默认站姿
        zero_targets = obs_builder.action_to_servo_targets(np.zeros(12, dtype=np.float32))
        bridge.send_servo_targets(zero_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)
        if args.init_wait > 0:
            print(f"初始站姿已发送，等待{args.init_wait:.1f}秒...")
            time.sleep(args.init_wait)

    # --- CSV logger ---
    csv_file = None
    if args.log_csv:
        csv_file = open(args.log_csv, "w")
        csv_file.write(
            "step,time_s,loop_ms,roll_deg,pitch_deg,"
            + ",".join(f"a{i}" for i in range(12)) + "\n"
        )

    # --- 信号处理 ---
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        print("\n[STOP] Ctrl+C, stopping...")
        running = False
    signal.signal(signal.SIGINT, signal_handler)

    gc.disable()
    print(f"\n=== 控制循环开始 ({control_hz} Hz) ===")

    obs_builder.reset()
    t_start    = time.perf_counter()
    next_time  = t_start
    total_steps = int(args.duration * control_hz)
    step        = 0
    action_prev = np.zeros(12, dtype=np.float32)
    target_prev = None

    try:
        while running and step < total_steps:
            t_loop_start = time.perf_counter()

            # --- 1. 读取 STM32 最新状态（默认依赖持续回传流） ---
            state = bridge.get_state() if not args.dry_run else {
                "servo_pos": np.zeros(12, dtype=np.float32),
                "servo_vel": np.zeros(12, dtype=np.float32),
                "imu_roll": 0.0, "imu_pitch": 0.0, "imu_yaw": 0.0,
                "imu_gyro": np.zeros(3), "imu_accel": np.zeros(3),
                "timestamp": time.perf_counter(),
            }

            if not args.dry_run and (time.perf_counter() - state["timestamp"] > 0.2):
                if step % print_every == 0:
                    print("[WARN] STM32 状态超过 200ms 未更新，检查固件回传频率/协议格式")

            # --- 2. 构建 48 维观测 ---
            obs_raw  = obs_builder.build_observation(state, command, control_dt)
            obs_norm = normalizer.normalize(obs_raw)

            # --- 3. 策略推理 ---
            action_raw = policy.predict(obs_norm)  # shape (12,)
            action_cmd = action_alpha * action_raw + (1.0 - action_alpha) * action_prev
            action_prev = action_cmd.copy()

            # --- 4. 动作转换为关节目标角度 (Isaac 顺序, rad) ---
            target_pos_isaac = obs_builder.DEFAULT_JOINT_POS + action_cmd * action_scale
            target_pos_isaac = np.clip(target_pos_isaac, limits_low, limits_high)

            # --- 5. 安全检查 ---
            gx, gy, gz = obs_raw[6:9]  # projected_gravity
            roll_deg  = float(np.degrees(np.arctan2(gy,  -gz)))
            pitch_deg = float(np.degrees(np.arctan2(-gx, np.sqrt(gy**2 + gz**2))))
            target_pos_isaac, is_safe = safety.check_and_clip(target_pos_isaac, roll_deg, pitch_deg)

            if not is_safe:
                reason = safety.emergency_reason or "safety_guard"
                print(f"[EMERGENCY] 异常急停! reason={reason}")
                break

            # --- 6. 转换为舵机刻度并发送 ---
            if not args.dry_run:
                corrected = (target_pos_isaac - obs_builder.DEFAULT_JOINT_POS) / action_scale
                servo_targets = obs_builder.action_to_servo_targets(corrected)
                bridge.send_servo_targets(servo_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)
            else:
                obs_builder.last_action = action_cmd.copy()
                servo_targets = obs_builder.action_to_servo_targets(action_cmd)

            tgt_jump = 0 if target_prev is None else int(np.max(np.abs(servo_targets - target_prev)))
            target_prev = servo_targets.copy()

            step += 1
            t_now = time.perf_counter()
            loop_ms = (t_now - t_loop_start) * 1000

            # --- 7. 日志 ---
            if step % print_every == 0:
                elapsed = t_now - t_start
                hz_avg  = step / elapsed if elapsed > 0 else 0
                act_mag = float(np.abs(action_cmd).mean())
                state_age_ms = (time.perf_counter() - state["timestamp"]) * 1000.0 if not args.dry_run else 0.0
                gyro_norm = float(np.linalg.norm(state["imu_gyro"]))
                print(
                    f"step={step:5d}  t={elapsed:6.1f}s  hz={hz_avg:5.1f}  "
                    f"loop={loop_ms:5.1f}ms  |a|={act_mag:.3f}  "
                    f"roll={roll_deg:+5.1f}°  pitch={pitch_deg:+5.1f}°"
                    f"  {'[DRY]' if args.dry_run else ''}"
                )
                if args.verbose_debug:
                    print(
                        f"    tgt0={int(servo_targets[0])}  tgt1={int(servo_targets[1])}  "
                        f"max|Δtarget|={tgt_jump}  gyro_norm={gyro_norm:.3f}  state_age={state_age_ms:.1f}ms"
                    )
                if print_joint_vel:
                    vel_arr = np.array(state["servo_vel"], dtype=np.float32)
                    pos_arr = np.array(state["servo_pos"], dtype=np.float32)
                    cur_raw = np.round(pos_arr / (2 * np.pi) * 4096 + 2048).astype(np.int32)
                    track_err = servo_targets.astype(np.int32) - cur_raw
                    vel_text = np.array2string(vel_arr, precision=3, suppress_small=False, max_line_width=200)
                    pos_text = np.array2string(pos_arr, precision=3, suppress_small=False, max_line_width=200)
                    print(f"    servo_vel(rad/s)={vel_text}  vel_norm={float(np.linalg.norm(vel_arr)):.3f}")
                    print(
                        f"    servo_pos(rad)={pos_text}  max|target-cur|={int(np.max(np.abs(track_err)))}"
                    )
                if not args.dry_run and gyro_norm < 1e-3 and step >= print_every:
                    print("[WARN] imu_gyro 近似为 0，策略动态观测不足，可能表现为仅姿态保持。")

            if csv_file:
                csv_file.write(
                    f"{step},{t_now-t_start:.4f},{loop_ms:.2f},{roll_deg:.2f},{pitch_deg:.2f},"
                    + ",".join(f"{a:.5f}" for a in action_cmd) + "\n"
                )

            # --- 8. 精确控频 ---
            next_time += control_dt
            slack = next_time - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                next_time = time.perf_counter()  # 丢帧恢复

            if step % 500 == 0:
                gc.collect()

    finally:
        gc.enable()
        if not args.dry_run:
            zero_targets = obs_builder.action_to_servo_targets(np.zeros(12, dtype=np.float32))
            bridge.send_servo_targets(zero_targets, speed=args.servo_speed, time_ms=args.servo_time_ms)
            time.sleep(0.3)
            bridge.set_torque(False)
            bridge.disconnect()
        if csv_file:
            csv_file.close()
            print(f"CSV 已保存: {args.log_csv}")

        elapsed = time.perf_counter() - t_start
        print(f"\n=== 完成: {step} 步 / {elapsed:.1f}s =  avg {step/max(elapsed,0.001):.1f} Hz ===")


if __name__ == "__main__":
    main()
