# WAVEGO Sim2Real 文档总览（准确性优先版）

本目录用于 **sim2real 前的资料准备**，已按当前工作空间代码再次核对。

## 事实来源（本次核对优先级）

1. `scripts/sim2sim_test.py`（你提供的可运行 MuJoCo 链路）
2. `logs/rsl_rl/wavego_flat/2026-02-12_03-17-29/params/env.yaml`
3. `tmp/io_descriptors_wavego/isaac_velocity_flat_wavego_v0_IO_descriptors.yaml`
4. `weixue-dog-nano-code/robot_io.py`（当前上位机串口协议实现）
5. `weixue-wheeled-dog/Core/Inc/usb_cmd_handler.h`（协议类型定义）

> 注：本机当前环境缺少 `mujoco` 包，无法在本次会话中直接复跑 `sim2sim_test.py`。因此所有“运行时结论”均标注为“需运行确认”。

## 当前可确认的关键事实

- 观测维度：48
- 动作维度：12
- 控制频率：50 Hz（`sim_dt=0.005`, `decimation=4`）
- `action_scale=0.25`
- policy 关节顺序（按类型分组）：`hip[FL,FR,RL,RR] + thigh[...] + calf[...]`
- MuJoCo/DFS 物理顺序（按腿分组）：`FL(h,t,c), FR(h,t,c), RL(h,t,c), RR(h,t,c)`

以上均来自 `sim2sim_test.py + env.yaml + io_descriptor` 的静态一致信息。

## 串口协议状态（重要）

当前代码库存在两种协议写法：

- 分支 A：`HEAD | TYPE | LEN | PAYLOAD | SUM`
- 分支 B：`HEAD | LEN | CMD | PAYLOAD | XOR`

`sim2real_test/scripts/stm32_bridge.py` 已改为：

- 默认发送：分支 A（与 `robot_io.py` 一致）
- 接收：兼容 A/B 两种格式

**最终真值仍以你当前 STM32 固件分支抓包为准。**

## 文件结构

- `01_JETSON_NANO_ENV_SETUP.md`：Jetson 环境准备
- `02_DEPLOYMENT_PIPELINE.md`：部署流程与验收（仅保留可证实流程）
- `03_OBSERVATION_AND_INFERENCE.md`：48 维观测与推理链路（按 `sim2sim_test.py` 对齐）
- `04_MODEL_EXPORT_AND_OPTIMIZATION.md`：ONNX/TensorRT 路线
- `05_TROUBLESHOOTING.md`：常见故障（已移除已知过时 I2C 直连结论）
- `config/wavego_deploy_config.yaml`：部署参数（已修复 `safety` 结构）
- `scripts/stm32_bridge.py`：串口桥接（A/B 协议兼容）
- `scripts/obs_builder.py`：观测构建
- `scripts/wavego_inference.py`：推理主循环（已去除重复残留代码）

## 已明确标记为“废弃参考”

- `scripts/imu_reader.py`
- `scripts/servo_driver.py`

这两个文件保留仅供历史参考，不应进入当前部署主链路。
