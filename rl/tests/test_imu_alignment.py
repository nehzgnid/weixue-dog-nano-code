#!/usr/bin/env python3
import time
import numpy as np
import yaml
import sys
from pathlib import Path


def _ensure_repo_root_on_path():
    for parent in Path(__file__).resolve().parents:
        if (parent / "MIGRATION_PLAN_SCHEME_B_EXECUTION.md").exists():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            return parent
    fallback = Path(__file__).resolve().parents[2]
    if str(fallback) not in sys.path:
        sys.path.insert(0, str(fallback))
    return fallback


REPO_ROOT = _ensure_repo_root_on_path()

from common.io.stm32_bridge import STM32Bridge
from rl.core.obs_builder import ObsBuilder

# 1. 加载你的配置
CONFIG_PATH = REPO_ROOT / "rl" / "config" / "wavego_deploy_config.yaml"
if not CONFIG_PATH.exists():
    CONFIG_PATH = REPO_ROOT / "sim2real_jetson_tests" / "strategy_deploy_bundle" / "config" / "wavego_deploy_config.yaml"

if not CONFIG_PATH.exists():
    raise FileNotFoundError(f"未找到部署配置文件: {CONFIG_PATH}")

with open(CONFIG_PATH) as f:
    cfg = yaml.safe_load(f)["observation"]

axis_remap = cfg.get("imu_axis_remap", [1, 0, 2])
axis_sign = np.array(cfg.get("imu_axis_sign", [1.0, -1.0, 1.0]), dtype=np.float32)

def main():
    bridge = STM32Bridge(port="/dev/ttyACM0")
    if not bridge.connect(): return
    
    obs_builder = ObsBuilder(cfg)
    
    print("\n" + "="*65)
    print("IMU 高性能速度解算与状态测试 (Isaac Lab 视角)")
    print(f"当前映射: remap={axis_remap}, sign={axis_sign}")
    print("算法: 重力补偿 (Gravity Compensation) + 航向积分滤波 (Leaky Integrator)")
    print("="*65 + "\n")
    
    try:
        # 记录时间差
        last_time = time.perf_counter()
        
        while True:
            bridge.request_state()
            time.sleep(0.05)
            
            now = time.perf_counter()
            dt = now - last_time
            last_time = now
            
            state = bridge.get_state()
            
            # 获取原始 Euler 
            roll, pitch, yaw = state["imu_roll"], state["imu_pitch"], state["imu_yaw"]
            
            # 机体坐标系下的欧拉角
            robot_roll = -pitch
            robot_pitch = roll
            
            # --- 使用 ObsBuilder 内部的高性能解算 ---
            obs_raw = obs_builder.build_observation(state, np.zeros(3), dt)
            
            isaac_lin_vel = obs_raw[0:3]
            isaac_gyro = obs_raw[3:6]
            projected_gravity = obs_raw[6:9]
            
            # 提取加速度供显示 (包含重力)
            accel_g = np.array([state["imu_accel"][i] for i in axis_remap], dtype=np.float32) * axis_sign
            
            # 计算去重力后的净加速度 (m/s^2)
            a_lin_m_s2 = (accel_g + projected_gravity) * 9.81
            
            # --- 验证输出 ---
            print(f"[{time.strftime('%H:%M:%S.%f')[:-3]}] 原始位姿与高性能解算数据:")
            print(f"  IMU Euler(原始)   : Roll={roll:+6.2f}°, Pitch={pitch:+6.2f}°, Yaw={yaw:+6.2f}°")
            print(f"  Robot Euler(Isaac): Roll={robot_roll:+6.2f}°, Pitch={robot_pitch:+6.2f}°")
            print(f"  Gyro  (角速度)    : X={isaac_gyro[0]:+5.2f}, Y={isaac_gyro[1]:+5.2f}, Z={isaac_gyro[2]:+5.2f} rad/s")
            print(f"  Accel (含重力)    : X={accel_g[0]:+5.2f}, Y={accel_g[1]:+5.2f}, Z={accel_g[2]:+5.2f} g")
            print(f"  Accel (去重力净值): X={a_lin_m_s2[0]:+5.2f}, Y={a_lin_m_s2[1]:+5.2f}, Z={a_lin_m_s2[2]:+5.2f} m/s²")
            print(f"  LinVel(预估线速度): X={isaac_lin_vel[0]:+5.2f}, Y={isaac_lin_vel[1]:+5.2f}, Z={isaac_lin_vel[2]:+5.2f} m/s")
            print("-" * 65)
    except KeyboardInterrupt:
        bridge.disconnect()

if __name__ == "__main__":
    main()