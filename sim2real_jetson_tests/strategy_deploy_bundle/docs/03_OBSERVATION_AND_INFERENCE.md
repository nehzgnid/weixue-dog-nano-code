# 03 — 观测与推理链路（按当前代码核对）

本文件以 `scripts/sim2sim_test.py`、`obs_builder.py`、`env.yaml`、`io_descriptor` 为准。

## 1. 观测维度与顺序

观测总维度：48。

顺序：

1. `base_lin_vel` (3)
2. `base_ang_vel` (3)
3. `projected_gravity` (3)
4. `velocity_commands` (3)
5. `joint_pos_rel` (12)
6. `joint_vel_rel` (12)
7. `last_action` (12)

该顺序与 `sim2sim_test.py` 中 `build_policy_observation()` 一致。

## 2. 动作维度与缩放

- 动作维度：12
- 缩放：`action_scale = 0.25`
- 目标关节角：

`target = default_joint_pos + action * 0.25`

与 `io_descriptor.actions[0].scale` 一致。

## 3. 控制周期

- `sim_dt = 0.005`
- `decimation = 4`
- `control_dt = 0.02`（50 Hz）

## 4. 关节顺序与映射

### 4.1 policy 顺序（按类型分组）

`FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, FR_thigh, RL_thigh, RR_thigh, FL_calf, FR_calf, RL_calf, RR_calf`

### 4.2 DFS 物理顺序（按腿分组）

`FL(h,t,c), FR(h,t,c), RL(h,t,c), RR(h,t,c)`

### 4.3 映射（当前默认）

- `servo_to_isaac = [0,4,8,1,5,9,2,6,10,3,7,11]`
- 反向映射由 `obs_builder.py` 自动生成

> 注意：映射是否最终正确必须通过实机单关节激励验证。

## 5. 各观测项的实机来源（当前实现）

- `base_lin_vel`：默认置零（`base_lin_vel_mode=zero`）
- `base_ang_vel`：由 IMU 角速度（deg/s）转 rad/s
- `projected_gravity`：由 roll/pitch 计算
- `velocity_commands`：上位机命令输入
- `joint_pos_rel`：舵机位置（rad）减默认位
- `joint_vel_rel`：优先舵机回传速度，缺失时差分估计
- `last_action`：上一控制步动作缓存

## 6. normalizer（必须）

训练策略依赖观测标准化：

`obs_norm = clip((obs - mean)/(std + 1e-8), -5, 5)`

`mean/std` 来自 `config/normalizer_stats.npz`。

## 7. 推理主循环（概览）

每 20ms：

1. 读取 STM32 状态
2. 构建 48 维 `obs_raw`
3. 标准化得到 `obs_norm`
4. ONNX 推理得到 12 维动作
5. 转目标角并限幅
6. 安全检查
7. 转舵机刻度并发送
8. 更新 `last_action`

## 8. 当前明确不写死的内容（留白）

以下项目因固件分支差异或现场安装差异，暂不写死数值：

- 串口包字段顺序/校验算法（由 `stm32_bridge.py` 兼容）
- `joint_direction` 最终符号
- `joint_zero_offsets` 最终标定值
- IMU 轴重映射最终配置

这些项必须以“当前固件 + 当前机体安装”实测为准后回填。
