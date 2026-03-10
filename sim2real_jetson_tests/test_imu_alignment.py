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

# 1. 加载你的配置
CONFIG_PATH = BUNDLE_DIR / "config/wavego_deploy_config.yaml"
with open(CONFIG_PATH) as f:
    cfg = yaml.safe_load(f)["observation"]

axis_remap = cfg.get("imu_axis_remap", [1, 0, 2])
axis_sign = np.array(cfg.get("imu_axis_sign", [1.0, -1.0, 1.0]), dtype=np.float32)

def main():
    bridge = STM32Bridge(port="/dev/ttyACM0")
    if not bridge.connect(): return
    
    print("\n" + "="*50)
    print("IMU 坐标系对齐测试 (Isaac Lab 视角)")
    print(f"当前映射: remap={axis_remap}, sign={axis_sign}")
    print("预期表现:")
    print("  1. 狗头朝下(低头): Pitch为正，Gyro_Y(左轴)为正，重力投影 gx 变负")
    print("  2. 狗向右倾斜(右滚): Roll为正，Gyro_X(前轴)为正，重力投影 gy 变负")
    print("  3. 狗左转(偏航): Yaw改变，Gyro_Z(上轴)为正")
    print("="*50 + "\n")
    
    try:
        while True:
            bridge.request_state()
            time.sleep(0.1)
            state = bridge.get_state()
            
            # --- 转换到 Isaac 坐标系 ---
            # 角速度 (rad/s)
            gyro_raw = np.array([state["imu_gyro"][i] for i in axis_remap], dtype=np.float32)
            isaac_gyro = gyro_raw * axis_sign * (np.pi / 180.0)
            
            # 加速度 (m/s^2)
            accel_raw = np.array([state["imu_accel"][i] for i in axis_remap], dtype=np.float32)
            isaac_accel = accel_raw * axis_sign
            
            # --- 验证输出 ---
            print(f"[{time.strftime('%H:%M:%S')}] Isaac 坐标系下数据:")
            print(f"  Gyro (角速度) X/Y/Z : [{isaac_gyro[0]:+5.2f}, {isaac_gyro[1]:+5.2f}, {isaac_gyro[2]:+5.2f}] rad/s")
            print(f"  Accel (加速度) X/Y/Z: [{isaac_accel[0]:+5.2f}, {isaac_accel[1]:+5.2f}, {isaac_accel[2]:+5.2f}] m/s²")
            print("-" * 50)
    except KeyboardInterrupt:
        bridge.disconnect()

if __name__ == "__main__":
    main()