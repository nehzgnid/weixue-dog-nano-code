# 02 — 部署流程与验收标准（准确性优先版）

本文档只保留可由当前代码直接支撑的步骤；不确定内容明确留白。

## 本轮实测快照（2026-03-04，env_isaaclab）

已执行并确认：

- `mujoco==3.5.0` 可在 `env_isaaclab` 导入
- `scripts/sim2sim_test.py` 可运行并输出：
  - `control_hz=50.00`
  - `kp=400.000, kv=10.000, forcerange=[-1.960, 1.960]`
  - `obs_dim=48`
  - `action_scale=0.25`
- `scripts/reinforcement_learning/rsl_rl/play.py --help` 可正常启动参数解析
- `onnxruntime` 已安装到 `env_isaaclab`
- `scripts/wavego_inference.py --dry-run --no-gpu` 已实测：
  - `Policy loaded: input=[1,48], output=[1,12]`
  - 2 秒内 `100` 步，平均 `50.0 Hz`
  - `|a|≈0.385`（非零，说明真实 ONNX 策略输出生效）
- `scripts/test_latency.py --no-gpu --full` 已实测：
  - ONNX 推理均值 `0.026 ms`（P99 `0.044 ms`）
  - 全流程（normalize+inference+action_scale）均值 `0.041 ms`

说明：

- 上述时延来自当前开发机 CPU 基线，不等同于 Jetson Nano 真机时延；Jetson 侧需按相同命令复测并记录。

## Step 0：事实基线对齐（先做）

对齐文件：

- `scripts/sim2sim_test.py`
- `logs/.../params/env.yaml`
- `tmp/io_descriptors_wavego/..._IO_descriptors.yaml`

验收：

- `obs_dim == 48`
- `action_dim == 12`
- `action_scale == 0.25`
- `sim_dt == 0.005`, `decimation == 4`, `control_dt == 0.02`
- 关节顺序一致（policy 顺序 vs DFS 物理顺序映射）

## Step 1：模型与统计量准备（开发机）

```bash
cd /home/user/IsaacLab
python sim2real_test/scripts/export_onnx_with_normalizer.py \
  --checkpoint logs/rsl_rl/wavego_flat/2026-02-12_03-17-29/model_2999.pt \
  --output-dir sim2real_test/config/
```

验收：

- `config/wavego_policy.onnx` 存在
- `config/normalizer_stats.npz` 存在且 `mean/std` 形状为 `(48,)`

## Step 2：Jetson 侧最小闭环前检查

### 2.1 设备可见性

```bash
ls /dev/ttyACM*
```

验收：

- 至少存在一个 `ttyACM*`

### 2.2 桥接器基础读写

```bash
python sim2real_test/scripts/stm32_bridge.py --port /dev/ttyACM0 --duration 3
```

验收：

- 能收到状态包（`timestamp` 更新）
- 无持续校验失败日志

> 不确定项：若无数据，需确认当前固件协议分支（TYPE+LEN+SUM 或 LEN+CMD+XOR）。

## Step 3：推理链路干跑（不下发舵机）

```bash
python sim2real_test/scripts/wavego_inference.py \
  --config config/wavego_deploy_config.yaml \
  --cmd-x 0 --cmd-y 0.2 --cmd-wz 0 \
  --duration 5 \
  --dry-run
```

验收：

- 循环频率接近 50 Hz
- 无 NaN/异常中断
- `obs_raw` 维度 48，`action` 维度 12
- 若 ONNX 已启用，`|a|` 应稳定非零（非全 0）

## Step 4：低风险真机联机（先架空）

```bash
python sim2real_test/scripts/wavego_inference.py \
  --config config/wavego_deploy_config.yaml \
  --cmd-x 0 --cmd-y 0 --cmd-wz 0 \
  --duration 5
```

验收：

- 可进入控制循环并平稳退出
- 安全保护不误触发

## Step 5：映射/方向标定（必须）

当前 `joint_direction` 与 `joint_zero_offsets` 默认值仅为占位。

必须做：

- 单关节激励确认每个关节“正方向”
- 标定后回填 `config/wavego_deploy_config.yaml`

验收：

- 12 个关节方向均与策略定义一致
- 零位姿态与训练默认姿态一致（允许小偏差）

## Step 6：落地测试（最后）

先低速，再逐步增加命令幅度。

建议顺序：

1. 站立（`cmd=[0,0,0]`）
2. 小侧移（`cmd_y=0.1~0.2`）
3. 小前进（`cmd_x=0.1~0.2`）

验收（建议记录）：

- 是否稳定不倒
- 是否出现持续高频抖动
- 是否出现明显方向反转（映射错误）

## 明确留白（需实测后补）

- 当前固件最终协议格式（两种分支中的哪一种）
- 串口回包频率上限与稳定性阈值
- 落地最大稳定速度
- 实机端到端时延统计（P95/P99）
