# Jetson Nano Sim2Real 测试指南（分步骤）

本目录用于在 Jetson Nano 端做 sim2real 前置验证，目标是按风险由低到高逐步确认：

1. 环境可用
2. 串口链路可用
3. 单舵机安全微动可用
4. ONNX 推理链路可用

---

## 目录结构

- `01_env_check.py`：环境与依赖检查
- `02_stm32_state_probe.py`：仅监听 STM32 状态数据
- `03_single_servo_safe_test.py`：单舵机微动（默认 dry-run）
- `04_onnx_latency_smoke_test.py`：ONNX 推理与延迟冒烟测试

---

## 一、环境配置（Jetson Nano）

建议在 `env_isaaclab` 环境中执行：

```bash
conda activate env_isaaclab
pip install numpy pyserial pyyaml onnxruntime
```

> 若你只测串口链路，`onnxruntime` 可先不装；做第 4 步时必须安装。

---

## 二、执行流程（按顺序）

### Step 1：环境检查

```bash
cd /home/user/weixue-dog-nano-code/sim2real_jetson_tests
python 01_env_check.py --port /dev/ttyACM0
```

验收标准：

- 全部关键项为 `[PASS]`
- 无 `[FAIL]`

---

### Step 2：STM32 状态链路探测

```bash
python 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 5
```

验收标准：

- 输出中能看到 IMU 或舵机状态更新
- 结束时出现 `[PASS] 状态链路有数据更新`

---

### Step 3：单舵机安全微动测试（先 dry-run）

先 dry-run：

```bash
python 03_single_servo_safe_test.py --id 1 --delta 50
```

确认机械安全后执行真实动作：

```bash
python 03_single_servo_safe_test.py --id 1 --delta 50 --execute
```

验收标准：

- 舵机按预期做小幅动作并回位
- 无异常抖动/撞限

> 建议：每次仅测一个舵机，`delta` 从小值开始（20~50）。

---

### Step 4：ONNX 推理冒烟测试

```bash
python 04_onnx_latency_smoke_test.py \
  --model /home/user/IsaacLab/sim2real_test/config/wavego_policy.onnx \
  --stats /home/user/IsaacLab/sim2real_test/config/normalizer_stats.npz \
  --iters 500
```

验收标准：

- 输出形状为 `(1, 12)`
- 出现 `[PASS] 推理时延满足 50Hz 控制预算`

---

## 三、常见问题

1. `No serial ports found` / `/dev/ttyACM0` 不存在
   - 检查 USB 线是否为数据线
   - 检查 STM32 固件是否启用 CDC
   - 检查 `dialout` 权限

2. 第 2 步无状态更新
   - 固件协议分支可能不一致
   - 确认下位机在持续发送状态包

3. `onnxruntime` 导入失败
   - 重新执行 `pip install onnxruntime`

---

## 四、安全建议

- 落地测试前，先完成本目录全部步骤。
- 第 3 步先 dry-run，再执行真实动作。
- 测试期间准备急停/断电手段。
