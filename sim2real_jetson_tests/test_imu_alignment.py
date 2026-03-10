#!/usr/bin/env python3
import time
import numpy as np
import yaml
import sys
from pathlib import Path

# Add the bundle directory to sys.path so we can import stm32_bridge
BUNDLE_DIR = Path(__file__).parent / "strategy_deploy_bundle"
sys.path.insert(0, str(BUNDLE_DIR))

from scripts.stm32_bridge import STM32Bridge
from scripts.obs_builder import ObsBuilder

# 1. 加载你的配置
CONFIG_PATH = BUNDLE_DIR / "config/wavego_deploy_config.yaml"
with open(CONFIG_PATH) as f:
    cfg = yaml.safe_load(f)["observation"]

axis_remap = cfg.get("imu_axis_remap", [1, 0, 2])
axis_sign = np.array(cfg.get("imu_axis_sign", [1.0, -1.0, 1.0]), dtype=np.float32)

def main():
    bridge = STM32Bridge(port="/dev/ttyACM0")
    if not bridge.connect(): return
    
    obs_builder = ObsBuilder(cfg)
    
    print("\n" + "="*50)
    print("IMU 坐标系对齐及速度测试 (Isaac Lab 视角)")
    print(f"当前映射: remap={axis_remap}, sign={axis_sign}")
    print("注意: 目前 Python 端解析 IMU 数据仅提取了 9 个 float (姿态, 陀螺仪, 加速度)。")
    print("若你的 WT61C 或 STM32 真的封装了线速度数据，下面的 'Raw Payload Size' 会大于 36 字节。")
    print("="*50 + "\n")
    
    try:
        # 为了更准确地计算积分，记录时间差
        last_time = time.perf_counter()
        
        # Patch STM32Bridge to print raw IMU packet sizes once
        original_parse_full = bridge._parse_full_state
        original_parse_imu = bridge._parse_imu
        printed_size = False
        
        def patched_parse_full(data):
            nonlocal printed_size
            if not printed_size:
                print(f"[DEBUG] 收到 FULL_STATE 包，数据载荷长度: {len(data)} 字节")
                print(f"        (预期 9个float+1个数量+舵机数据 = 36 + 1 + 12*11 = 169 字节)")
                printed_size = True
            return original_parse_full(data)
            
        def patched_parse_imu(data):
            nonlocal printed_size
            if not printed_size:
                print(f"[DEBUG] 收到 IMU_ONLY 包，数据载荷长度: {len(data)} 字节")
                print(f"        (预期 9个float = 36 字节。如果有额外的 velocity 数据，长度会大于 36)")
                printed_size = True
            return original_parse_imu(data)
            
        bridge._parse_full_state = patched_parse_full
        bridge._parse_imu = patched_parse_imu
        
        while True:
            # 获取底层包长度（通过 patch 一个内部方法来观测）
            bridge.request_state()
            time.sleep(0.05)
            
            now = time.perf_counter()
            dt = now - last_time
            last_time = now
            
            state = bridge.get_state()
            
            # --- 转换到 Isaac 坐标系 ---
            # 使用 ObsBuilder 提供的积分或置零逻辑获取线速度
            # 注意: 这里需要传入虚拟的命令 (0,0,0) 来构造 observation
            obs_raw = obs_builder.build_observation(state, np.zeros(3), dt)
            
            # obs_raw 的前 3 项是 base_lin_vel
            isaac_lin_vel = obs_raw[0:3]
            
            # 角速度 (rad/s) 和 加速度 (m/s^2) [仅用于展示]
            gyro_raw = np.array([state["imu_gyro"][i] for i in axis_remap], dtype=np.float32)
            isaac_gyro = gyro_raw * axis_sign * (np.pi / 180.0)
            
            accel_raw = np.array([state["imu_accel"][i] for i in axis_remap], dtype=np.float32)
            isaac_accel = accel_raw * axis_sign
            
            # --- 验证输出 ---
            print(f"[{time.strftime('%H:%M:%S')}] Isaac 坐标系下数据:")
            print(f"  Gyro  (角速度) X/Y/Z : [{isaac_gyro[0]:+5.2f}, {isaac_gyro[1]:+5.2f}, {isaac_gyro[2]:+5.2f}] rad/s")
            print(f"  Accel (加速度) X/Y/Z : [{isaac_accel[0]:+5.2f}, {isaac_accel[1]:+5.2f}, {isaac_accel[2]:+5.2f}] m/s²")
            print(f"  LinVel(预估速度)X/Y/Z: [{isaac_lin_vel[0]:+5.2f}, {isaac_lin_vel[1]:+5.2f}, {isaac_lin_vel[2]:+5.2f}] m/s")
            print("-" * 50)
    except KeyboardInterrupt:
        bridge.disconnect()

if __name__ == "__main__":
    main()