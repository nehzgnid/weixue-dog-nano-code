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
SCRIPT_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPT_DIR))

try:
    from stm32_bridge import STM32Bridge
    from obs_builder import ObsBuilder
except ImportError as e:
    print(f"[ERROR] 导入失败: {e}")
    print("  请确认 stm32_bridge.py 和 obs_builder.py 在同一目录")
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
# 安全保护 (SafetyGuard 转发 scripts/safety_guard.py)
# =============================================================================

class SafetyGuard:
    def __init__(self, cfg: dict):
        self.max_action_delta = cfg.get("max_action_delta", 0.3)
        self.max_pitch_deg = cfg.get("max_pitch_deg", 45.0)
        self.max_roll_deg = cfg.get("max_roll_deg", 45.0)
        self.enabled = cfg.get("emergency_stop_enabled", True)
        self._prev_target = None
        self._triggered = False

    def check(
        self,
        target_pos: np.ndarray,
        roll_deg: float,
        pitch_deg: float,
    ) -> tuple:
        if not self.enabled:
            return target_pos, False
        if abs(roll_deg) > self.max_roll_deg or abs(pitch_deg) > self.max_pitch_deg:
            print(f"[SAFETY] E-stop! roll={roll_deg:.1f}°, pitch={pitch_deg:.1f}°")
            self._triggered = True
            return (self._prev_target if self._prev_target is not None else target_pos), True
        if self._prev_target is not None:
            delta = np.clip(
                target_pos - self._prev_target,
                -self.max_action_delta, self.max_action_delta
            )
            target_pos = self._prev_target + delta
        self._prev_target = target_pos.copy()
        return target_pos, False

    @property
    def is_triggered(self) -> bool:
        return self._triggered


# =============================================================================
# 主循环——改用 STM32Bridge + ObsBuilder
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="WAVEGO Sim2Real 出 (透传STM32)推理")
    parser.add_argument("--config", type=str, default="config/wavego_deploy_config.yaml")
    parser.add_argument("--cmd-x",  type=float, default=0.0, help="前进速度 m/s")
    parser.add_argument("--cmd-y",  type=float, default=0.0, help="侧移速度 m/s")
    parser.add_argument("--cmd-wz", type=float, default=0.0, help="转向 rad/s")
    parser.add_argument("--duration", type=float, default=10.0, help="运行时长 (秒)")
    parser.add_argument("--dry-run", action="store_true", help="只推理不发指令")
    parser.add_argument("--no-gpu",  action="store_true", help="强制 CPU 推理")
    parser.add_argument("--log-csv", type=str, default=None)
    parser.add_argument("--port",   type=str, default=None, help="覆盖串口设备")
    args = parser.parse_args()

    # --- 加载配置 ---
    cfg_path = Path(__file__).parent.parent / args.config
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    sim2real_root = cfg_path.parent.parent
    control_dt    = cfg["timing"]["control_dt_s"]
    control_hz    = cfg["timing"]["control_freq_hz"]
    action_scale  = cfg["policy"]["action_scale"]
    limits_low    = np.array(cfg["joints"]["limits_low_policy"],  dtype=np.float32)
    limits_high   = np.array(cfg["joints"]["limits_high_policy"], dtype=np.float32)
    print_every   = cfg["debug"]["print_every"]
    command       = np.array([args.cmd_x, args.cmd_y, args.cmd_wz], dtype=np.float32)

    print(f"Command: vx={command[0]:.2f} vy={command[1]:.2f} wz={command[2]:.2f}")
    print(f"Control: {control_hz} Hz, dt={control_dt*1000:.0f}ms, 时长={args.duration}s")

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

    # --- STM32Bridge ---
    hw_cfg = cfg["hardware"]
    port = args.port or hw_cfg["serial_port"]
    bridge = STM32Bridge(
        port=port,
        baudrate=hw_cfg["serial_baud"],
        timeout=hw_cfg.get("serial_timeout", 0.01),
    )
    if not args.dry_run:
        if not bridge.connect():
            print(f"[ERROR] 无法连接 STM32，退出。识别描述符: {port}")
            sys.exit(1)

    # --- ObsBuilder ---
    obs_builder = ObsBuilder(cfg.get("observation", {}))

    # --- SafetyGuard ---
    safety = SafetyGuard(cfg["safety"])

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
        bridge.send_servo_targets(zero_targets)
        print("初始站姿已发送，等待1秒...")
        time.sleep(1.0)

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

            # --- 4. 动作转换为关节目标角度 (Isaac 顺序, rad) ---
            target_pos_isaac = obs_builder.DEFAULT_JOINT_POS + action_raw * action_scale
            target_pos_isaac = np.clip(target_pos_isaac, limits_low, limits_high)

            # --- 5. 安全检查 ---
            gx, gy, gz = obs_raw[6:9]  # projected_gravity
            roll_deg  = float(np.degrees(np.arctan2(gy,  -gz)))
            pitch_deg = float(np.degrees(np.arctan2(-gx, np.sqrt(gy**2 + gz**2))))
            target_pos_isaac, emergency = safety.check(target_pos_isaac, roll_deg, pitch_deg)

            if emergency:
                print("[EMERGENCY] 异常急停!")
                break

            # --- 6. 转换为舵机刻度并发送 ---
            if not args.dry_run:
                corrected = (target_pos_isaac - obs_builder.DEFAULT_JOINT_POS) / action_scale
                servo_targets = obs_builder.action_to_servo_targets(corrected)
                bridge.send_servo_targets(servo_targets, speed=0, time_ms=0)
            else:
                obs_builder.last_action = action_raw.copy()

            step += 1
            t_now = time.perf_counter()
            loop_ms = (t_now - t_loop_start) * 1000

            # --- 7. 日志 ---
            if step % print_every == 0:
                elapsed = t_now - t_start
                hz_avg  = step / elapsed if elapsed > 0 else 0
                act_mag = float(np.abs(action_raw).mean())
                print(
                    f"step={step:5d}  t={elapsed:6.1f}s  hz={hz_avg:5.1f}  "
                    f"loop={loop_ms:5.1f}ms  |a|={act_mag:.3f}  "
                    f"roll={roll_deg:+5.1f}°  pitch={pitch_deg:+5.1f}°"
                    f"  {'[DRY]' if args.dry_run else ''}"
                )

            if csv_file:
                csv_file.write(
                    f"{step},{t_now-t_start:.4f},{loop_ms:.2f},{roll_deg:.2f},{pitch_deg:.2f},"
                    + ",".join(f"{a:.5f}" for a in action_raw) + "\n"
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
            bridge.send_servo_targets(zero_targets)
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
