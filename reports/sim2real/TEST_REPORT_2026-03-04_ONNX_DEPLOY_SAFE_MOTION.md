# ONNX 策略部署测试报告（安全动作联调）

Date: 2026-03-04
Environment: Jetson Nano + Docker + `/dev/ttyACM0` pass-through

## 新增测试脚本

- `05_onnx_policy_safe_motion_test.py`
- 功能：
  - `q` 键急停（触发后下发零位并关闭扭矩）
  - `Ctrl+C` 急停
  - `--virtual-imu`（使用虚拟 IMU 零姿态）
  - 命令安全上限 `--max-cmd`（默认 0.1）
  - 保持 50Hz 控制循环与原部署链路一致（ObsBuilder + ONNXPolicy + STM32Bridge）

## 结果

### 1) dry-run（仅推理，不发指令）

```bash
python3 05_onnx_policy_safe_motion_test.py --duration 3 --dry-run --no-gpu --virtual-imu
```

- PASS：`Policy loaded: input=[1, 48], output=[1, 12]`
- PASS：3 秒稳定运行，无急停

### 2) 零命令短时实发（最小风险）

```bash
python3 05_onnx_policy_safe_motion_test.py --duration 2 --cmd-x 0 --cmd-y 0 --cmd-wz 0
```

- PASS：串口连接、扭矩开关、发送与退出流程正常
- PASS：未触发急停

### 3) 微小命令渐进实发（架空）

```bash
python3 05_onnx_policy_safe_motion_test.py --duration 2 --cmd-y 0.05
python3 05_onnx_policy_safe_motion_test.py --duration 2 --cmd-x 0.05
python3 05_onnx_policy_safe_motion_test.py --duration 2 --cmd-wz 0.05
```

- PASS：三组均稳定完成
- PASS：未触发急停
- 观测到目标刻度随命令变化（例如 `tgt0` 在不同命令下有差异）

## 结论

ONNX 策略在当前部署链路下可以稳定输出动作，并已完成“零命令 + 微小命令”架空实发验证。

## 下一步建议

1. 继续架空，将单轴命令提升到 `0.1`，每次 2~3 秒。
2. 若稳定，再进入落地测试（先零命令）。
3. 全程保留急停键 `q` 和电源断电保护。
