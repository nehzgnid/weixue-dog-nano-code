# 05 — 常见问题与解决思路（准确性优先版）

## 1. `wavego_inference.py` 启动即退出

排查：

- 检查 `config/wavego_deploy_config.yaml` 是否含顶层 `safety` 段
- 检查 ONNX 与 normalizer 文件路径是否存在

命令：

```bash
python sim2real_test/scripts/wavego_inference.py \
  --config config/wavego_deploy_config.yaml \
  --duration 1 --dry-run
```

## 2. 串口设备不存在

现象：`/dev/ttyACM0` 不存在。

排查：

```bash
ls /dev/ttyACM*
dmesg | tail -n 50
lsusb
```

处理：

- 检查 USB 线是否为数据线
- 确认 STM32 固件已启用 USB CDC
- 用户加入 `dialout` 组并重新登录

## 3. 串口打开成功但收不到状态

现象：`timestamp` 长时间不更新。

根因高概率：协议分支不一致。

处理：

- 用桥接器自测：

```bash
python sim2real_test/scripts/stm32_bridge.py --port /dev/ttyACM0 --duration 3
```

- 若仍无数据，切换发包格式重试：

```bash
python sim2real_test/scripts/stm32_bridge.py \
  --port /dev/ttyACM0 --duration 3 --tx-format len_cmd_xor
```

## 4. 机器人动作方向明显错误

高概率原因：

- `joint_direction` 未标定
- `servo_to_isaac` 映射与接线不一致

处理：

- 先 `--dry-run` 确认推理链路
- 再做单关节激励逐项校对
- 回填 `joint_direction` 与 `joint_zero_offsets`

## 5. 推理输出正常但落地不稳

排查优先级：

1. `normalizer` 是否正确加载
2. 观测维度与顺序是否保持 48 维定义
3. IMU 轴重映射是否正确
4. 命令是否过大（先用小命令）

## 6. 无法复跑 MuJoCo sim2sim 验证

现象：`ModuleNotFoundError: mujoco`。

说明：

- 当前环境未安装 `mujoco`，会影响“运行态一手验证”。
- 可先用静态核对（`sim2sim_test.py + env + io_descriptor`）完成文档准备。
- 后续补装 `mujoco` 后再补充运行结果字段。
