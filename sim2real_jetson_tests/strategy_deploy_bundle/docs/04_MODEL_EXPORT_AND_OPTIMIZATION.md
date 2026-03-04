# 04 — TensorRT / ONNX 模型导出与优化

> 解答：是否需要 TensorRT 或 ONNX？结论是**推荐使用 ONNX，TensorRT 可选但非必须**。

---

## 1. 模型规模分析

| 指标 | 值 |
|---|---|
| 输入维度 | 48 |
| 输出维度 | 12 |
| 隐层结构 | [512, 256, 128] |
| 激活函数 | ELU |
| 总参数量 | ~25K + 128K + 32K + 1.5K ≈ **187K** |
| 权重文件大小 | ~750 KB (float32) |

**结论**：这是一个**极小的网络**，推理计算量非常低。

---

## 2. 三种部署方案对比（延迟为经验值，需实测）

| 方案 | 推理延迟 (Jetson Nano) | 部署复杂度 | 推荐场景 |
|---|---|---|---|
| **A: ONNX Runtime (CPU)** | 约 1~5 ms（需实测） | ⭐ 最简单 | **首选，推荐** |
| **B: ONNX Runtime (GPU)** | 约 0.5~3 ms（需实测） | ⭐⭐ | GPU 配置成功时 |
| **C: TensorRT** | 约 0.3~2 ms（需实测） | ⭐⭐⭐ 最复杂 | 极端实时场景 |
| D: PyTorch (CPU) | 通常慢于 ONNX（需实测） | ⭐⭐ | 不推荐 |

### 2.1 本轮实测基线（2026-03-04，env_isaaclab，CPU）

命令：

```bash
python sim2real_test/scripts/test_latency.py \
    --model sim2real_test/config/wavego_policy.onnx \
    --stats sim2real_test/config/normalizer_stats.npz \
    --iterations 500 --no-gpu --full
```

结果：

- ONNX 推理：mean `0.026 ms`，P99 `0.044 ms`
- normalize：mean `4.3 µs`
- 观测拼接：mean `1.8 µs`
- 全流程：mean `0.041 ms`，P99 `0.069 ms`

> 说明：这是当前开发机 CPU 数据，仅作链路正确性与数量级参考；Jetson Nano 上必须复测并以 Jetson 结果为准。

### 为什么推荐 ONNX 而非 TensorRT

1. **推理耗时通常不是瓶颈**：50 Hz 控制要求 < 20 ms/步。此网络规模较小，实测常见可满足预算；最终仍以你的 Jetson 实测为准。
2. **TensorRT 配置成本高**：需处理 Opset 兼容性、FP16 精度损失、引擎序列化等额外工作。
3. **调试友好**：ONNX 可在任何平台加载验证，TensorRT 引擎绑定 GPU 型号。

### 什么情况下需要 TensorRT

- 控制频率需要 > 200 Hz
- 策略网络显著更大（如含视觉编码器）
- 有多策略并行推理需求

---

## 3. 方案 A（推荐）：ONNX 导出

### 3.1 导出脚本

使用 `sim2real_test/scripts/export_onnx_with_normalizer.py`：

```bash
cd /home/user/IsaacLab
python sim2real_test/scripts/export_onnx_with_normalizer.py \
  --checkpoint logs/rsl_rl/wavego_flat/2026-02-12_03-17-29/model_2999.pt \
  --output-dir sim2real_test/config/
```

输出：
- `wavego_policy.onnx` — 纯 actor MLP（输入 48 → 输出 12），**不含 normalizer**
- `normalizer_stats.npz` — normalizer 的 mean 和 std

### 3.2 关于 normalizer 是否嵌入 ONNX

**选项 A：normalizer 在 ONNX 外部（推荐）**

```
obs_raw → [Python: normalize] → obs_norm → [ONNX: MLP] → action
```

优点：
- 可独立验证每个环节
- 方便调试和修改 clip 参数
- normalizer 统计量可单独更新

**选项 B：normalizer 嵌入 ONNX**

```
obs_raw → [ONNX: normalize + MLP] → action
```

优点：
- 单文件部署
- 减少 Python 计算

对于当前项目，**推荐选项 A**，降低调试复杂度。

### 3.3 ONNX Opset 版本注意

Isaac Lab 默认导出 Opset 18。如果 Jetson 上的 ONNX Runtime 版本较旧：

| ONNX Runtime | 最高支持 Opset |
|---|---|
| 1.10.0 | 15 |
| 1.14.0 | 18 |
| 1.17.0 | 20 |

**如果报错 `Invalid Opset`**，在导出时降级：

```python
torch.onnx.export(model, dummy_input, output_path, opset_version=11)
```

### 3.4 在 Jetson 上验证 ONNX

```python
import numpy as np
import onnxruntime as ort
import time

sess = ort.InferenceSession("wavego_policy.onnx")
dummy = np.zeros((1, 48), dtype=np.float32)

# 预热
for _ in range(10):
    sess.run(None, {sess.get_inputs()[0].name: dummy})

# 测速
times = []
for _ in range(1000):
    t0 = time.perf_counter()
    sess.run(None, {sess.get_inputs()[0].name: dummy})
    times.append(time.perf_counter() - t0)

print(f"Mean: {np.mean(times)*1000:.2f} ms")
print(f"Max: {np.max(times)*1000:.2f} ms")
print(f"P99: {np.percentile(times, 99)*1000:.2f} ms")
```

---

## 4. 方案 C（可选）：TensorRT 优化

### 4.1 从 ONNX 转换

```bash
# JetPack 预装了 trtexec
/usr/src/tensorrt/bin/trtexec \
  --onnx=wavego_policy.onnx \
  --saveEngine=wavego_policy.trt \
  --fp16 \
  --workspace=256
```

### 4.2 FP16 精度验证

TensorRT FP16 可能引入 ~1e-3 级别精度偏差，对小网络影响极小，但建议验证：

```python
# 对比 ONNX FP32 与 TensorRT FP16 输出
import numpy as np

# 用 100 个随机输入对比
for _ in range(100):
    obs = np.random.randn(1, 48).astype(np.float32)
    out_onnx = onnx_session.run(None, {"input": obs})[0]
    out_trt = trt_infer(obs)
    diff = np.abs(out_onnx - out_trt).max()
    assert diff < 0.05, f"TRT precision drift too large: {diff}"
```

### 4.3 TensorRT Python 推理

```python
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np

class TRTInference:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # 分配 GPU 内存
        self.d_input = cuda.mem_alloc(1 * 48 * 4)   # float32
        self.d_output = cuda.mem_alloc(1 * 12 * 4)
        self.h_output = np.zeros((1, 12), dtype=np.float32)
        self.stream = cuda.Stream()

    def predict(self, obs):
        cuda.memcpy_htod_async(self.d_input, obs.astype(np.float32).ravel(), self.stream)
        self.context.execute_async_v2(
            bindings=[int(self.d_input), int(self.d_output)],
            stream_handle=self.stream.handle
        )
        cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
        self.stream.synchronize()
        return self.h_output.copy()
```

---

## 5. 推荐部署路径

```
1. 先用 ONNX Runtime CPU 跑通全链路（1-2天）
       ↓
2. 确认控制闭环正常后，尝试 ONNX Runtime GPU（0.5天）
       ↓
3. 如果需要更快，转 TensorRT FP16（1天）
       ↓
4. 仅在有明确需求时做 INT8 量化（需校准数据集）
```

---

## 6. 注意事项

### 6.1 ONNX 模型不含 normalizer 和 action_scale

导出的 ONNX 是**纯 actor 网络**。以下操作必须在 Python 中完成：

```python
# 推理前
obs_norm = normalizer.normalize(obs_raw)       # (48,) → (48,)

# 推理
action_raw = onnx_session.run(..., obs_norm)    # (48,) → (12,)

# 推理后
target_pos = default_pos + action_raw * 0.25    # action_scale
```

### 6.2 Batch Size

实机推理始终 `batch_size = 1`。ONNX 导出时建议固定维度：

```python
torch.onnx.export(
    model, dummy_input, output_path,
    input_names=["obs"],
    output_names=["action"],
    dynamic_axes=None,  # 固定 batch_size=1
)
```

### 6.3 数值类型

- 推理全程使用 **float32**
- Jetson Nano GPU 不支持高效 float64
- NumPy 默认 float64，注意在送入 ONNX 前转换为 float32
