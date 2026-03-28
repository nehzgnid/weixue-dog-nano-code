# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Weixue Dog Nano — 12 舵机四足机器人的**上位机控制系统**，Python 实现。运行在 Jetson Nano 或 PC 上，作为机器人的"大脑"，通过 USB CDC 串口与下位机 STM32 通信。

**配套下位机仓库**: `/home/user/weixue-wheeled-dog`（STM32F407 FreeRTOS 固件，Keil MDK 工程）。

## 上下位机关系

```
本仓库 (上位机)                    weixue-wheeled-dog (下位机)
Jetson Nano / PC  ───USB CDC───>  STM32F407 (FreeRTOS)  ───UART+DMA───>  12× STS3215 舵机
                                  STM32F407              <──UART──       WT61C IMU
```

- 上位机**从不直接控制硬件**，所有实时舵机驱动和 IMU 采集由 STM32 完成
- 通信协议：`0xA5 0x5A` 包头 + 类型/长度 + 载荷 + 校验和（详见 `GEMINI.md`）
- 关键命令：`0x10` 舵机控制、`0x11` 扭矩使能、`0x30` 请求状态
- 关键反馈：`0x20` 舵机状态、`0x30` IMU 数据、`0x40` 全身状态

## Architecture

分层架构，自底向上：

| 层 | 模块 | 职责 |
|---|---|---|
| 硬件抽象 | `robot_io.py` | 串口通信、协议编解码、IMU/舵机状态维护 |
| 配置 | `config.json` + `robot_config.py` | 机器人几何参数、舵机限位、方向映射 |
| 运动学 | `kinematics_v5.py` | IK（足端位置→关节角度）、FK、PWM 转换 |
| 步态 | `gait_generator.py` | Bezier 曲线步态轨迹生成（摆动/支撑相） |
| 平衡 | `balance_controller.py` | PID 姿态稳定（低通滤波微分项） |
| 应用 | `control_center_unified.py` | tkinter GUI 主控中心（手动调试/动作序列） |

**舵机 ID 映射**: FL=(1,2,3) FR=(4,5,6) RL=(7,8,9) RR=(10,11,12)，每条腿 3 关节（外展/髋/膝）

**关节顺序注意**: Isaac Lab 策略按类型分组（all hip→all thigh→all calf），舵机按腿 DFS 分组，部署时需索引映射（见 `wavego_deploy_config.yaml` 中的 `policy_to_servo`）。

## RL Sim2Real 部署

`sim2real_jetson_tests/strategy_deploy_bundle/` 包含从 IsaacLab 训练策略部署到实机的完整管线：

- **模型**: ONNX 策略网络（48D obs → 12D action），50Hz 控制频率
- **核心文件**: `scripts/wavego_inference.py`（推理主循环）、`scripts/stm32_bridge.py`（STM32 通信桥接）
- **配置**: `config/wavego_deploy_config.yaml`（关节映射、观测构建、安全参数）
- **依赖**: `numpy`, `pyserial`, `pyyaml`, `onnxruntime`
- **部署流程**: `01_env_check.py` → `02_stm32_state_probe.py` → `03_single_servo_safe_test.py` → `04_onnx_latency_smoke_test.py`

## Running

```bash
# GUI 控制中心（需连接 STM32）
python control_center_unified.py

# 3D 仿真（无需硬件）
python simulation_unified.py

# Trot 步态测试（需连接硬件）
python main_trot.py

# RL 推理部署（Jetson Nano）
cd sim2real_jetson_tests/strategy_deploy_bundle
python scripts/wavego_inference.py --config config/wavego_deploy_config.yaml --cmd-x 0.3 --duration 30

# 串口自动发现：优先匹配含 "USB" 的设备名
```

## Key Conventions

- 所有模块通过 `robot_config.py` 的 `cfg` 单例获取配置（`from robot_config import cfg`）
- `LegKinematics.solve_ik(x, y, z)` 返回 `(q_abd, q_hip, q_knee)` 弧度值或 `None`（不可达）
- 串口波特率统一 115200，STM32 USB CDC 实际不依赖波特率设置
- 每条腿有独立的方向系数（`config.json` → `directions`），处理舵机安装方向差异
