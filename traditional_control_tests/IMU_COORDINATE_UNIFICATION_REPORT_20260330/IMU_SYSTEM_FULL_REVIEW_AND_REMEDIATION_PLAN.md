# IMU 双链路复核与坐标统一整改报告

- 日期: 2026-03-30
- 输入依据: `traditional_control_tests/IMU_OBS_AUDIT_REPORT_20260329_061452.md`
- 复核范围:
  - 传统控制链路（RobotIO + Trot/GUI）
  - 强化学习部署链路（RobotIOBridge + ObsBuilder + ONNX 推理）

## 1. 执行摘要

本次复核结论：**不能仅按审计报告修改 `imu_axis_remap/imu_axis_sign` 就结束**。

原因如下：

1. RL 链路中欧拉角映射并未配置化，代码中仍硬编码为 `obs_roll=-raw_pitch, obs_pitch=+raw_roll`，与审计最佳候选 `obs_pitch=-raw_roll` 不一致。
2. RL 链路当前 `base_lin_vel_mode=zero`，导致神经网络输入 `obs[0:3]` 恒为零，无法改善速度延迟与低速丢失问题。
3. 传统控制链路直接使用 `RobotIO.get_imu()` 的 raw roll/pitch/yaw，没有与 RL 链路共享同一套坐标变换定义；若只改 RL 配置，会形成双标准。
4. 工程内存在两套桥接实现（`robot_io.py` 路径与 `stm32_bridge.py` 路径），IMU 字段顺序与单位注释不完全一致，存在长期维护风险。

## 2. 诊断报告与当前代码的一致性检查

### 2.1 诊断报告给出的最佳候选

- `imu_axis_remap: [0, 1, 2]`
- `imu_axis_sign: [-1.0, -1.0, 1.0]`
- `euler_map: obs_roll=-raw_pitch, obs_pitch=-raw_roll`

### 2.2 当前 RL 实际实现

- 当前配置文件仍是:
  - `imu_axis_remap: [1, 0, 2]`
  - `imu_axis_sign: [1.0, -1.0, 1.0]`
  - `base_lin_vel_mode: "zero"`
- 代码中欧拉映射硬编码:
  - `robot_roll_rad = -imu_pitch`
  - `robot_pitch_rad = +imu_roll`

结论：与诊断最佳候选不一致，且欧拉映射无法仅通过配置修正。

### 2.3 当前传统控制实际实现

- `main_trot.py` 与 `control_center_unified.py` 直接用 `RobotIO.get_imu()` 的 roll/pitch/yaw 做 PID 平衡。
- 传统链路没有读取 RL 的 `imu_axis_remap/imu_axis_sign` 或任何统一坐标配置。

结论：传统链路与 RL 链路坐标定义没有统一入口。

## 3. 关键问题清单（按优先级）

### P0（必须先做）

1. **欧拉映射配置化缺失**
   - 现状：`obs_builder.py` 内硬编码欧拉映射。
   - 风险：审计结果无法完整落地，后续每次换安装都要改代码。

2. **base_lin_vel 输入恒为零**
   - 现状：`base_lin_vel_mode=zero`。
   - 风险：策略观测缺项，速度反馈无法反映真实运动。

3. **双链路坐标定义割裂**
   - 现状：传统控制与 RL 各用各的映射/假设。
   - 风险：同一 IMU 在不同模块解释不同，调试成本高、容易回归。

### P1（建议紧随其后）

4. **线速度估计参数未工程化配置**
   - 现状：死区、泄漏等参数硬编码。
   - 风险：低速被压零、延迟偏大时无法快速调参。

5. **协议字段顺序/单位认知不统一**
   - 现状：不同桥接实现注释与解析路径存在差异。
   - 风险：切换桥接器或固件分支时出现隐性错误。

## 4. 是否“按诊断报告修改”就足够？

结论：**不够**。

至少需要同时完成以下三类改动：

1. 观测坐标改动：`remap/sign + euler_map` 一起落地。
2. 速度链路改动：启用并调优 `base_lin_vel` 估计，不再恒零。
3. 架构改动：传统控制与 RL 控制共享同一套 IMU 变换定义。

## 5. 涉及 IMU 数据的文件清单

> 按“核心运行链路”与“测试/诊断链路”分组。

### 5.1 传统控制核心链路

- `robot_io.py`
  - 串口包解析、IMU 缓存、`get_imu()` 输出。
- `main_trot.py`
  - 使用 `get_imu()` 的 roll/pitch 做平衡控制。
- `control_center_unified.py`
  - `get_corrected_imu()`、`zero_imu()`、步态平衡中 roll/pitch/yaw 闭环。
- `balance_controller.py`
  - roll/pitch/yaw PID 控制器（消费 IMU 角度）。

### 5.2 RL 部署核心链路

- `sim2real_jetson_tests/robotio_bridge.py`
  - 将 `RobotIO.get_imu()` 包装为 RL 观测状态字段。
- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/obs_builder.py`
  - IMU 映射、`projected_gravity`、`base_ang_vel`、`base_lin_vel` 构建。
- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py`
  - 调用 `build_observation()`，喂给 ONNX 推理。
- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_keyboard_control.py`
  - 同样调用 `build_observation()`。
- `sim2real_jetson_tests/strategy_deploy_bundle/config/wavego_deploy_config.yaml`
  - RL 观测映射与速度模式配置源。

### 5.3 测试/诊断链路

- `traditional_control_tests/test_imu_readout.py`
- `traditional_control_tests/auto_imu_observation_audit.py`
- `sim2real_jetson_tests/test_imu_alignment.py`
- `sim2real_jetson_tests/05_onnx_policy_safe_motion_test.py`
- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/stm32_bridge.py`（备用桥接实现）
- `servo_control_center.py`（独立调试终端）

## 6. 工程化整改方案（推荐）

## 6.1 统一“单一真相源”（Single Source of Truth）

建议新增统一配置文件，例如：

- `config/imu_frame_unified.yaml`（仓库根目录或 `sim2real_jetson_tests/strategy_deploy_bundle/config/`）

建议字段：

- `raw_frame`: 传感器原始坐标系定义（文档字段）
- `target_frame`: 全系统统一目标坐标系定义
- `vector_map`:
  - `axis_remap: [0,1,2]`
  - `axis_sign: [-1,-1,1]`
- `euler_map`:
  - `obs_roll: {src: raw_pitch, sign: -1}`
  - `obs_pitch: {src: raw_roll, sign: -1}`
- `units`:
  - `gyro_raw_unit: deg_s`
  - `accel_raw_unit: g`
- `lin_vel_estimator`:
  - `mode: integrate`
  - `deadband_m_s2`
  - `vel_decay`
  - `lpf_alpha`
  - `stationary_gyro_thresh_dps`
  - `stationary_acc_norm_tol_g`

## 6.2 新增共享变换模块

建议新增模块：

- `common/imu_transform.py`

职责：

1. 读取统一配置。
2. 提供 `map_vector(raw_xyz)`、`map_euler(raw_roll, raw_pitch)`。
3. 提供 `projected_gravity(obs_roll, obs_pitch)`。
4. 输出统一数据结构（带单位注释和时间戳）。

调用原则：

- 传统控制、RL 观测构建、测试工具统一调用该模块。
- `robot_io.py` 只做“原始包解析”，不做坐标语义变换。

## 6.3 RL 链路具体改造

1. 修改 `wavego_deploy_config.yaml`
   - 更新 `imu_axis_remap` / `imu_axis_sign` 为审计最优候选。
   - 将 `base_lin_vel_mode` 从 `zero` 改为 `integrate`。
   - 新增线速度估计参数配置项，移除硬编码。

2. 修改 `obs_builder.py`
   - 欧拉映射改为读取配置（不要硬编码符号）。
   - `base_lin_vel` 参数全部改为配置项。
   - 启动时打印当前映射摘要与参数摘要。

3. 修改 `wavego_inference.py` / `wavego_keyboard_control.py`
   - 启动日志打印“已加载的 IMU 配置版本 + 摘要”。
   - 每 N 步可选打印 `obs[0:3]` 统计，便于验证速度链路。

## 6.4 传统控制链路具体改造

1. 修改 `main_trot.py`、`control_center_unified.py`
   - 从共享变换模块读取统一后的 roll/pitch/yaw（或显式声明使用 raw）。
   - 若采用统一坐标后符号变化，按一次性步骤重新标定 PID 正负方向。

2. 保留 `zero_imu`，但限定为“零偏”而非“坐标变换”。

3. 在 UI 中区分显示：
   - raw IMU
   - unified IMU
   避免调试时混淆。

## 6.5 测试/诊断链路同步改造

1. `test_imu_readout.py`
   - 去掉硬编码映射常量，改为读取统一配置。
2. `auto_imu_observation_audit.py`
   - 当前配置对照项增加 euler_map 的配置读取（目前 euler_map 仍部分硬编码）。
3. `test_imu_alignment.py`
   - 显示逻辑改为与共享变换模块一致，避免双实现。

## 7. 建议实施顺序（最小风险）

1. 第一步：仅引入“统一配置 + 共享变换模块”，保持行为不变（配置填当前值）。
2. 第二步：落地审计最佳候选映射（含 euler_map）。
3. 第三步：启用 `base_lin_vel_mode=integrate`，从保守参数开始调。
4. 第四步：联合回归（传统控制 + RL 控制 + 诊断工具）。

## 8. 验收标准（建议）

1. 坐标一致性:
   - 审计脚本 PASS，且最佳候选与当前配置一致。
2. 速度可用性:
   - `obs[0:3]` 在低速拖动时非全零，且静止时接近零。
3. 双链路一致:
   - 传统控制显示与 RL 观测在同一动作下符号一致。
4. 可维护性:
   - 不再出现多处硬编码映射。

## 9. 直接结论

- 你的诊断报告方向是正确的，但**只改配置里的 remap/sign 不足以闭环**。
- 必须把 `euler_map` 与 `base_lin_vel` 一并纳入改造，并通过“统一配置 + 共享变换模块”把传统控制与 RL 控制收敛到同一坐标定义。
- 以上方案可在不大改现有控制逻辑的前提下，工程化地解决“坐标不一致、速度延迟大、低速被压零”这三类问题。

## 10. 代码证据定位（便于逐项落地）

1. RL 当前 `base_lin_vel_mode` 为 `zero`
   - `sim2real_jetson_tests/strategy_deploy_bundle/config/wavego_deploy_config.yaml:131`
   - `sim2real_jetson_tests/strategy_deploy_bundle/scripts/obs_builder.py:257`

2. RL 当前欧拉映射硬编码（未配置化）
   - `sim2real_jetson_tests/strategy_deploy_bundle/scripts/obs_builder.py:139`
   - `sim2real_jetson_tests/strategy_deploy_bundle/scripts/obs_builder.py:140`

3. RL 观测确实由 `ObsBuilder.build_observation()` 进入推理
   - `sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py:346`
   - `sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_keyboard_control.py:685`

4. 传统链路直接消费 raw IMU（未共享 RL 映射）
   - `main_trot.py:78`
   - `main_trot.py:84`
   - `main_trot.py:85`
   - `control_center_unified.py:37`
   - `control_center_unified.py:896`

5. 当前调试脚本中仍存在独立硬编码映射（与配置源分离）
   - `traditional_control_tests/test_imu_readout.py:42`
   - `traditional_control_tests/test_imu_readout.py:43`
   - `traditional_control_tests/test_imu_readout.py:267`
   - `traditional_control_tests/test_imu_readout.py:268`

6. 审计脚本对“当前配置候选”的欧拉映射仍硬编码
   - `traditional_control_tests/auto_imu_observation_audit.py:531`

7. IMU 解析链路存在“双实现”风险点
   - `robot_io.py:182`（按 `<9f` 解析并更新 accel/gyro/euler）
   - `sim2real_jetson_tests/strategy_deploy_bundle/scripts/stm32_bridge.py:448`（注释定义为 euler/gyro/accel）
   - `sim2real_jetson_tests/strategy_deploy_bundle/scripts/stm32_bridge.py:403`（在 full_state 分支先取 accel，再 gyro，再 euler）
