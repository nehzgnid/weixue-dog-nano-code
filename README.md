# 微雪 Dog Nano 上位机控制与 Sim2Real 仓库

这是一个面向 12 自由度四足机器人的 Python 上位机仓库，负责运动学解算、步态生成、姿态平衡、串口通信、GUI 调试，以及 Isaac Lab 策略在 Jetson Nano 上的 sim2real 部署与安全测试。

如果把整套系统拆开来看：

- 本仓库负责上位机逻辑，也就是“算什么动作、怎么发命令、怎么做调试和部署”
- 下位机 STM32 负责实时舵机驱动、状态回传和 IMU 采集
- 两者通过 USB CDC 串口通信

## 项目概览

这个仓库目前包含两条主线：

1. 传统控制链路  
包括逆运动学、Bezier 步态、PID 平衡、GUI 控制台、单腿测试、原地踏步与基础 Trot 控制。

2. 强化学习 sim2real 链路  
包括 Jetson Nano 环境检查、STM32 状态探测、单舵机安全微动、ONNX 延迟测试，以及策略部署、观测构建、安全保护与键盘遥控。

适用场景：

- 在 PC 上做仿真和动作调参
- 在 Jetson Nano 或 Linux 主机上连接真机做控制
- 将 Isaac Lab 训练得到的策略导出为 ONNX 后部署到实机

## 系统结构

整体链路如下：

```text
Jetson Nano / PC
    │
    ├── 本仓库：上位机控制、仿真、推理部署
    │
    └── USB CDC 串口
            │
            ▼
        STM32F407
            ├── UART + DMA -> 12 个串口舵机
            └── UART <- WT61C IMU
```

在软件层面，可以理解为下面几层：

- 配置层：`config.json`、`robot_config.py`
- 通信层：`robot_io.py`
- 算法层：`kinematics_v5.py`、`gait_generator.py`、`balance_controller.py`
- 应用层：`control_center_unified.py`、`simulation_unified.py`、`main_trot.py`
- sim2real 层：`sim2real_jetson_tests/strategy_deploy_bundle/`

## 目录说明

根目录中最重要的文件和入口如下：

- `control_center_unified.py`  
统一 GUI 控制台，包含手动控制、动作序列、单腿实验和实时步态控制，是当前最完整的真机入口。

- `simulation_unified.py`  
综合仿真入口，适合先在本地验证 IK、姿态和动作效果，不需要连接硬件。

- `simulation_trot_interactive.py`  
交互式 Trot 仿真工具，可实时调 `Frequency`、`Step Height`、`Step Length`、模拟 IMU 倾斜，并在线调整平衡控制参数。

- `main_trot.py`  
无 GUI 的最小化 Trot 控制脚本，适合快速验证串口链路和步态控制主循环。

- `robot_io.py`  
当前上位机与 STM32 的通信核心，负责协议封包、发送舵机命令、接收 IMU 与舵机状态。

- `kinematics_v5.py`  
单腿逆运动学 / 正运动学求解，以及弧度到 PWM 的转换。

- `gait_generator.py`  
Bezier 曲线步态生成器，负责摆动相与支撑相轨迹。

- `balance_controller.py`  
姿态平衡控制，内部包含 PID 与简单微分滤波。

- `sim2real_jetson_tests/`  
Jetson Nano 侧 sim2real 验证脚本、测试报告与部署文档。

- `sim2real_jetson_tests/strategy_deploy_bundle/`  
ONNX 推理主链路，包括观测构建、STM32 桥接、安全保护、部署配置和排障文档。

## 快速开始

建议优先在 Linux 或 Jetson Nano 上运行，Windows 也可以用于部分 GUI 与串口调试。

最小依赖：

```bash
pip install numpy pyserial matplotlib pyyaml onnxruntime
```

如果你只跑传统控制和仿真，`onnxruntime` 不是必须；如果你要跑 sim2real 部署，则需要安装。

### 1. 先跑仿真

推荐先从综合仿真开始：

```bash
python simulation_unified.py
```

或者使用更适合调 Trot 参数的交互仿真：

```bash
python simulation_trot_interactive.py
```

### 2. 连接真机并打开 GUI 控制台

```bash
python control_center_unified.py
```

这个界面支持：

- 手动拖动 12 个舵机目标位
- 同步读取硬件当前位置
- 站立、趴下、坐下等预设动作
- 拜年动作序列
- 单腿实验模式
- Trot / Walk 参数在线调节

### 3. 使用最小化 Trot 控制入口

```bash
python main_trot.py
```

这个脚本没有 GUI，更适合快速验证：

- 串口是否正常连接
- IMU 是否有数据
- 步态主循环是否稳定运行

## 配置文件

### `config.json`

这是传统控制链路最关键的配置文件，定义了：

- 机体几何参数：`length`、`width`、`l1`、`l2`、`l3`
- 零位偏置：`hip`、`knee`
- 舵机限位：`abd`、`hip`、`knee`
- 12 个舵机的方向系数
- 默认速度与加速度参数

### `robot_config.py`

通过 `cfg` 单例统一读取配置，是全仓库共享的配置入口。

### `sim2real_jetson_tests/strategy_deploy_bundle/config/wavego_deploy_config.yaml`

这是 ONNX 实机部署的总配置文件，定义了：

- 模型与 normalizer 文件路径
- 观测维度、动作维度、动作缩放
- 控制频率与控制步长
- policy 顺序和舵机顺序映射
- IMU 轴重映射
- 安全阈值、关节方向、零点偏移

## 核心模块说明

### 1. 运动学：`kinematics_v5.py`

核心类为 `LegKinematics`，负责：

- `solve_ik(x, y, z)`：由足端目标位置求关节角
- `rad_to_pwm(...)`：关节弧度转换为舵机目标刻度
- `forward_kinematics_strict(...)`：由关节角反推关键点坐标，用于仿真显示

四条腿的舵机 ID 约定为：

- `FL = (1, 2, 3)`
- `FR = (4, 5, 6)`
- `RL = (7, 8, 9)`
- `RR = (10, 11, 12)`

### 2. 步态：`gait_generator.py`

负责生成：

- 摆动相轨迹 `get_swing_pos(...)`
- 支撑相轨迹 `get_stance_pos(...)`

当前采用 Bezier 曲线构造平滑的抬腿和落腿过程。

### 3. 平衡控制：`balance_controller.py`

使用 PID 对 `roll` 和 `pitch` 做补偿，主要用于：

- Trot 过程中机体姿态修正
- 仿真中模拟 IMU 倾斜后的姿态恢复

### 4. 通信：`robot_io.py`

这是传统控制链路中最重要的底层模块，负责：

- 自动发现串口并连接
- 下发扭矩开关命令
- 下发舵机目标位置命令
- 后台接收线程解析 IMU 与全身状态
- 维护 `servo_states`、`imu_data`、最新回包时间等共享状态

协议使用固定包头：

```text
0xA5 0x5A
```

在当前实现中，常见命令包括：

- `0x10`：舵机控制
- `0x11`：扭矩使能/失能
- `0x30`：IMU 反馈
- `0x40`：全身状态反馈

## 常用入口脚本

除了主入口外，仓库里还有一些实验性或专项测试脚本：

- `servo_control_center.py`  
较早的 GUI 串口调试终端，适合底层通信调试。

- `simulation_ik_v5.py`  
偏向单腿 IK / FK 可视化。

- `simulation_step_in_place.py`  
原地踏步仿真。

- `test_single_leg_step.py`  
单腿抬腿测试。

- `test_trot_in_place.py`  
原地 Trot 测试。

- `test_walk_in_place.py`  
带 IMU 闭环的 Walk 测试。

- `test_latency_linux.py`、`test_physical_latency.py`、`test_led_latency.py`  
与链路时延、物理响应或辅助实验相关的脚本。

这些脚本更偏实验用途，日常推荐优先使用：

- `control_center_unified.py`
- `simulation_unified.py`
- `simulation_trot_interactive.py`
- `sim2real_jetson_tests/` 下的标准验证脚本

## Sim2Real 部署与测试

`sim2real_jetson_tests/` 是当前强化学习部署链路的主目录。

### 目录用途

- `01_env_check.py`：检查 Python、依赖和串口设备
- `02_stm32_state_probe.py`：只监听 STM32 回包，确认状态链路正常
- `03_single_servo_safe_test.py`：单舵机小幅安全动作测试
- `04_onnx_latency_smoke_test.py`：检查 ONNX 推理是否正常、延迟是否满足 50Hz 预算
- `05_onnx_policy_safe_motion_test.py`：带安全约束和急停的 ONNX 动作联调
- `robotio_bridge.py`：将传统 `RobotIO` 封装为 sim2real 链路可直接复用的桥接层

### 推荐执行顺序

```bash
cd sim2real_jetson_tests
python 01_env_check.py --port /dev/ttyACM0
python 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 5
python 03_single_servo_safe_test.py --port /dev/ttyACM0 --id 1 --delta 50
python 04_onnx_latency_smoke_test.py --iters 500
```

如果前四步都正常，再进入策略联调：

```bash
python 05_onnx_policy_safe_motion_test.py --duration 3 --dry-run --no-gpu --virtual-imu
```

## ONNX 部署主链路

部署主链路位于：

- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py`
- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_keyboard_control.py`
- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/obs_builder.py`
- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/safety_guard.py`
- `sim2real_jetson_tests/strategy_deploy_bundle/scripts/stm32_bridge.py`

这条链路主要做几件事：

1. 从 STM32 读取关节状态和 IMU 数据
2. 构建 48 维观测向量
3. 对观测做 normalizer 标准化
4. 调用 ONNX Runtime 推理得到 12 维动作
5. 将动作映射为关节目标和舵机刻度
6. 经过安全层检查后发送到 STM32

### 观测和动作

当前部署约定：

- 观测维度：`48`
- 动作维度：`12`
- 控制频率：`50 Hz`
- `action_scale = 0.25`

由于 Isaac Lab 策略顺序和实际舵机 DFS 顺序不同，部署时必须做索引映射。相关配置在 `wavego_deploy_config.yaml` 中已经写出：

- `policy_to_servo`
- `servo_to_policy`
- `servo_to_isaac`

### 键盘控制

如果要在部署链路中做键盘遥控，可使用：

```bash
cd sim2real_jetson_tests/strategy_deploy_bundle
python scripts/wavego_keyboard_control.py --config config/wavego_deploy_config.yaml
```

默认支持按住式控制：

- `W/S`：前进 / 后退
- `A/D`：左移 / 右移
- `J/L`：左转 / 右转
- `Space`：停止

## 文档索引

如果你想进一步了解代码和部署细节，建议按下面顺序阅读：

- `API.md`  
传统控制模块的 API 说明。

- `CODE_DOCUMENTATION.md`  
项目架构、模块分层、使用方式与常见问题。

- `sim2real_jetson_tests/JETSON_SIM2REAL_TEST_GUIDE.md`  
Jetson Nano 上按风险递增的测试指南。

- `sim2real_jetson_tests/strategy_deploy_bundle/docs/README.md`  
部署文档总览。

- `sim2real_jetson_tests/strategy_deploy_bundle/docs/02_DEPLOYMENT_PIPELINE.md`  
部署步骤与验收标准。

- `sim2real_jetson_tests/strategy_deploy_bundle/docs/03_OBSERVATION_AND_INFERENCE.md`  
48 维观测与推理链路说明。

- `sim2real_jetson_tests/strategy_deploy_bundle/docs/05_TROUBLESHOOTING.md`  
常见问题排查。

## 安全建议

真机测试时请务必遵循以下原则：

1. 先仿真，再真机
2. 先架空，再落地
3. 先单舵机，再全身动作
4. 先 dry-run，再实发
5. 全程准备急停和断电手段

`05_onnx_policy_safe_motion_test.py` 支持按键急停，默认急停键为 `q`。  
即使如此，也不要把软件急停当成唯一保护手段，机械和电源级保护仍然是必须的。

## 当前状态

从仓库内容来看，这个项目已经不是“单一控制脚本”阶段，而是一个比较完整的四足机器人上位机工作区，既能做传统控制，也能承载 Isaac Lab 策略部署。根 README 现在的定位就是这个总入口，子目录里的测试报告、部署说明和排障文档继续承担细节文档角色。
