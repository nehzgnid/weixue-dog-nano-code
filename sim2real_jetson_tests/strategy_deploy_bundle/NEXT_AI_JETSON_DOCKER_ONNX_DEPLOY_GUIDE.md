# NEXT AI 部署指南（Jetson Nano + Docker + ONNX）

> 适用前提：
> - Jetson Nano, Ubuntu 20.04
> - 使用 Docker 运行
> - 前 4 项基础测试已通过
> - 当前目录：`sim2real_jetson_tests/strategy_deploy_bundle`

本指南目标：让下一位 AI 在 **不烧舵机** 前提下，循序渐进完成 ONNX 策略部署验证。

---

## 0. 安全红线（必须先读）

1. 第一次联机必须 **架空机器人**（脚不接地）。
2. 首次仅允许 `cmd_x=cmd_y=cmd_wz=0`，禁止动态行走命令。
3. 每次动作测试先 `--dry-run`，再短时实发（1~3 秒）。
4. 任何异常（高频抖动、连续撞限、舵机异响、过热）立即断电。
5. 未确认关节方向/零位前，禁止长时间闭环。

---

## 1. Docker 启动建议（Jetson）

```bash
docker run --rm -it \
  --network host \
  --device /dev/ttyACM0 \
  -v /home/user/weixue-dog-nano-code:/workspace \
  -w /workspace \
  <your_image>:<tag> \
  bash
```

容器内依赖建议：

```bash
pip install numpy pyserial pyyaml onnxruntime
```

> 如果镜像里已带这些依赖，可跳过安装。

---

## 2. 第一步：仅推理链路（不触硬件）

在容器内运行：

```bash
cd /workspace/sim2real_jetson_tests/strategy_deploy_bundle
python scripts/wavego_inference.py \
  --config config/wavego_deploy_config.yaml \
  --cmd-x 0 --cmd-y 0 --cmd-wz 0 \
  --duration 3 \
  --dry-run --no-gpu
```

验收标准：
- `Policy loaded: input=[1, 48], output=[1, 12]`
- 循环频率接近 50Hz
- 无异常退出

---

## 3. 第二步：ONNX 延迟冒烟

```bash
python scripts/test_latency.py \
  --model config/wavego_policy.onnx \
  --stats config/normalizer_stats.npz \
  --iterations 500 --no-gpu --full
```

验收标准：
- 推理均值远低于 5ms
- 输出无 NaN

---

## 4. 第三步：串口状态探测（只读）

在主机或容器内（确保 `/dev/ttyACM0` 已映射）运行：

```bash
cd /workspace/sim2real_jetson_tests
python 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 5
```

验收标准：
- IMU 或舵机状态有更新
- 无持续通信报错

---

## 5. 第四步：最小风险实发（架空 + 零命令 + 短时）

```bash
cd /workspace/sim2real_jetson_tests/strategy_deploy_bundle
python scripts/wavego_inference.py \
  --config config/wavego_deploy_config.yaml \
  --cmd-x 0 --cmd-y 0 --cmd-wz 0 \
  --duration 2
```

验收标准：
- 能正常启动和退出
- 无剧烈抖动、无撞限声

若异常，立即回退到 `--dry-run`，检查：
- `config/wavego_deploy_config.yaml` 的 `joint_direction`
- `joint_zero_offsets`
- `servo_to_isaac` 映射

---

## 6. 第五步：微小命令渐进（仍架空）

依次只测一个维度，每次 2~3 秒：

1. `cmd_y=0.05`
2. `cmd_x=0.05`
3. `cmd_wz=0.05`

每步间隔检查舵机温度/负载，确认稳定后再加到 `0.1`。

---

## 7. 允许落地前必须满足

- 干跑稳定
- 串口状态稳定
- 零命令短时实发稳定
- 微小命令架空稳定
- 无异常升温/异响/撞限

全部满足后，才允许进入落地测试。

---

## 8. 给下一位 AI 的执行约束

下一位 AI 必须遵守：

1. 先执行第 2~3 步，不得直接实发。
2. 实发必须先零命令，再微小命令。
3. 未经确认禁止使用 `|cmd| > 0.1`。
4. 任何异常必须自动建议回退到 dry-run。

---

## 9. 本目录关键文件清单

- `config/wavego_policy.onnx`
- `config/normalizer_stats.npz`
- `config/wavego_deploy_config.yaml`
- `scripts/wavego_inference.py`
- `scripts/stm32_bridge.py`
- `scripts/obs_builder.py`
- `scripts/safety_guard.py`
- `scripts/test_latency.py`
- `docs/*.md`（策略部署背景文档）
