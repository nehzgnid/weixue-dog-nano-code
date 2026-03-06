# ONNX 无可见动作问题排查报告

Date: 2026-03-04

## 现象

- ONNX 推理正常，日志持续输出
- 但现场观察“几乎无关节动作”

## 根因判断

高概率根因是控制包格式不匹配：

- `03_single_servo_safe_test.py`（可动）通过 `RobotIO` 发送格式：
  - `count + N*(sid, acc, pos, time, speed)`
- `strategy_deploy_bundle/scripts/stm32_bridge.py`（原始版本）发送格式：
  - `12h + speed + time` 紧凑包

若下位机固件按 RobotIO 格式解析，则会出现：
- 上位机看起来在运行控制循环，
- 但舵机命令被忽略或解析错误，导致不动。

## 已完成修复

1. 在 `stm32_bridge.py` 增加舵机发送格式兼容，默认改为 `robotio_id_list`。
2. 保留旧格式 `compact12` 作为可选模式。
3. 在 `05_onnx_policy_safe_motion_test.py` 增加可见性参数：
   - `--action-gain`（默认 1.5）
   - `--servo-speed`（默认 3400）
   - `--servo-time-ms`（默认 120）

## 修复后验证

执行：

```bash
python3 05_onnx_policy_safe_motion_test.py \
  --duration 2 --cmd-x 0.05 \
  --action-gain 1.8 --servo-time-ms 150 --servo-speed 3400
```

结果：
- 串口与扭矩控制正常
- 目标刻度在运行中出现明显变化（例如 `tgt0=2086 -> 2001 -> 2096`）
- 测试流程 PASS

## 建议后续验证（现场可见）

1. 继续架空，先跑 5s：
   - `cmd_y=0.05`
   - `cmd_x=0.05`
   - `cmd_wz=0.05`
2. 若可见动作仍偏小，提高到：
   - `--action-gain 2.2`
   - `--servo-time-ms 180`
3. 全程保留急停键 `q` 与电源断电保护。
