# 传统控制改造计划

## Context

当前仓库 `/home/user/weixue-dog-nano-code` 的运动控制主路径偏向 RL / ONNX 实机推理，而传统控制链路虽然已经具备基础模块（IK、串口协议、Bezier 步态、IMU PID、仿真器），但整体仍处于"实验脚本分散、符号约定不统一、稳定性不足"的状态，因此在真实地面上几乎不可用。

目标不是引入复杂控制理论，而是在**现有 12 舵机四足平台**上，落地一个**简单、可解释、可调参、能稳定慢速行走的传统控制器**，并尽量复用已有模块与成熟开源项目思路。

推荐路线：**放弃先做动态 trot，优先实现一个保守的 quasi-static walk/crawl 控制器**。这比 trot 更适合当前硬件与 50Hz 反馈条件，也最符合"少数学、先稳定"的要求。

## Recommended Approach

### 1. 控制策略

采用以下传统控制链路：

1. **相位调度器**
   - 使用 4 条腿依次摆动的 walk/crawl 相位，而不是对角 trot
   - 直接复用 `test_walk_in_place.py` 中已经出现的顺序：
     - FL = 0.00
     - RR = 0.25
     - FR = 0.50
     - RL = 0.75
   - 保证任意时刻只有 1 条腿进入 swing，相比 trot 更稳定

2. **简单足端轨迹**
   - 继续复用 `gait_generator.py` 的思路：
     - swing：前摆 + 抬脚 + 落脚
     - stance：身体相对足端缓慢后扫
   - 数学上只保留低门槛方案：Bezier / 正弦 / 分段线性轨迹，不引入动力学优化

3. **支撑侧重心偏移（body sway）**
   - 在抬腿前，先把身体轻微偏向剩余三足形成的支撑三角形
   - 这一步是传统四足稳定慢走的关键，比调大 PID 更有效
   - 采用启发式固定偏移，不做复杂 COM / ZMP 计算

4. **小幅 IMU 闭环稳定**
   - 继续复用 `balance_controller.py` 的 PID 思路，但只做：
     - roll：左右腿高度补偿
     - pitch：前后腿高度补偿
   - 加入死区、限幅、低通滤波，避免 IMU 噪声把机器人"抖倒"

5. **安全输出整形**
   - 保持 50Hz 控制频率
   - 对足端目标和 PWM 增量做限幅
   - 连续 IK 失败 / IMU 超时 / 串口状态异常时自动进入 stop / torque off

### 2. 参考的开源项目

只借鉴思路，不盲目搬硬件相关实现：

- **Petoi OpenCat**
  - 借鉴点：面向舵机四足的保守姿态控制、动作过渡、工程上偏实用
  - 适合提供"怎么把一台小型舵机狗先走稳"的经验模板

- **StanfordQuadruped**
  - 借鉴点：软件分层更清晰，适合作为 gait scheduler / foot target / IK / actuator pipeline 的结构参考
  - 适合指导本仓库把分散在多个脚本里的传统控制逻辑整理成一条主线

结论：
- **机械/调参哲学借鉴 OpenCat**
- **软件结构借鉴 StanfordQuadruped**
- **底层通信、IK、舵机映射继续沿用本仓库现有实现**

## Files To Modify

### `/home/user/weixue-dog-nano-code/gait_generator.py`
将其升级为统一的传统步态模块：
- 增加 walk/crawl 相位调度 API
- 明确 `global_phase -> leg_phase -> swing/stance -> foot target` 流程
- 增加 body sway / support shift 辅助函数
- 支持可配置参数：频率、步长、步高、摆动占空比、转向不对称量

### `/home/user/weixue-dog-nano-code/balance_controller.py`
保留为轻量姿态补偿模块：
- 统一输出语义（推荐直接输出每条腿的 z 补偿）
- 增加死区、限幅、滤波、速率限制
- 在 walk 模式下弱化 swing 腿补偿，优先修正支撑腿

### `/home/user/weixue-dog-nano-code/main_trot.py`
把它改造成"传统控制主入口"或作为实现参考迁移到新入口：
- 替换现有 trot 相位逻辑为 slow walk/crawl
- 建立标准控制循环：
  1. 读 IMU
  2. 计算 gait phase
  3. 计算 body sway
  4. 生成四足目标足端位置
  5. 叠加平衡补偿
  6. IK 求解
  7. 输出限幅
  8. 批量发 12 舵机命令
- 增加状态机：RELAX / STAND / WALK_SOFT_START / WALK_ACTIVE / FAULT

### `/home/user/weixue-dog-nano-code/robot_io.py`
保持为硬件抽象层，但补强安全性：
- 增加 IMU 新鲜度 / 串口状态检测
- 增加统一安全发送接口（PWM 限幅、默认 speed/acc）
- 增加统一紧急停止路径（torque off）

### `/home/user/weixue-dog-nano-code/simulation_unified.py`
作为离线调参主阵地：
- 接入与实机一致的 walk scheduler
- 可视化 swing leg、support polygon、body sway、IK 失败、关节限位
- 增加 roll/pitch 扰动滑块，用来验证补偿方向是否正确

### `/home/user/weixue-dog-nano-code/test_walk_in_place.py`
从"实验脚本"收敛为硬件验证脚本：
- 支持分阶段测试：
  - stand only
  - sway only
  - single-leg swing
  - suspended slow walk
- 作为地面测试前的安全验证入口

## Existing Code To Reuse

- `robot_io.py`
  - 已具备串口协议、12 舵机批量发送、IMU 状态维护
- `kinematics_v5.py`
  - 已具备 3 自由度腿 IK、弧度到 PWM 转换、每舵机方向配置
- `gait_generator.py`
  - 已具备 swing/stance 的雏形
- `balance_controller.py`
  - 已具备 PID 框架
- `test_walk_in_place.py`
  - 已经验证过 walk 型相位顺序与 body sway 方向思路
- `/home/user/weixue-dog-nano-code/sim2real_jetson_tests/strategy_deploy_bundle/scripts/stm32_bridge.py`
  - 可作为更完整的协议与安全发送参考

## Implementation Phases

### Phase 0 — 统一约定，先修"符号和接口"
先不追求跑起来，先统一这些约定：
- 坐标系：X 前、Y 左、Z 上/下的符号
- IMU roll/pitch 正方向
- balance 输出的正负含义
- 4 条腿 ID 与相位命名
- 步态模块与平衡模块的输入/输出接口

目标：避免现在这种"代码里注释自己都在怀疑方向符号"的情况。

### Phase 1 — 离线仿真实现 slow walk/crawl
在 `simulation_unified.py` 中先接入完整 slow walk：
- 默认很小的步长、步高、频率
- 接入 body sway
- 接入 bounded PID 补偿
- 检查完整 gait cycle 中是否存在 IK 失败和关节打限

目标：不接硬件时就能确认 gait 拍子、支撑切换、补偿方向都正确。

### Phase 2 — 架空测试（机器人离地）
用 `test_walk_in_place.py` 做安全验证：
1. 只站立
2. 只做 body sway
3. 单腿摆动
4. 四腿完整 walk cycle（低频、小步幅）

目标：验证：
- 舵机方向是否正确
- 抬腿顺序是否正确
- roll/pitch 补偿是否把正确的腿伸长/缩短
- 输出是否平滑，是否有跳变

### Phase 3 — 半支撑地面测试
地面测试前使用吊架或手扶减重：
- tiny stride
- low frequency
- 先不开启大幅平衡修正
- 每次只调一个参数

调参顺序建议：
1. 站高 / 站宽
2. body sway 幅度
3. 步高
4. 步长
5. roll gain
6. pitch gain
7. 摆动占空比

### Phase 4 — 无支撑低速行走
在能稳定迈步后再做：
- 连续慢速直行
- 起步 / 停止平滑过渡
- 小幅 yaw 转向（左右步长轻微不对称）
- 与 RL 基线做对比，但不追求速度，只追求稳定和可解释性

## Verification

### Simulation
- 完整 gait cycle 无 IK 失败
- 无明显 joint limit 饱和
- roll/pitch 扰动时补偿方向正确
- body sway 先于抬腿发生
- 足端轨迹在 swing/stance 切换点连续

### Suspended Bench Test
- 机器人离地时可连续稳定执行多个 gait cycle
- 单腿摆动时不会拖动其它腿出现异常大幅补偿
- PWM 输出无尖峰
- IMU 超时 / 串口异常时能停机

### Ground Test
- 可在极小步长下稳定迈步
- 不会一启动就侧翻/前扑
- 能连续执行起步、数步、停止
- 小幅外界扰动下不会立即失稳

## Risks And Mitigations

### 风险 1：方向符号错
现有代码已经暴露出 step length / stance 方向疑问。
- 处理：先做仿真和单腿测试，逐项验证正负方向，不直接上地面。

### 风险 2：IMU PID 把系统搞得更抖
- 处理：增加 deadband、限幅、低通；先让系统在"几乎不开环补偿"下能走，再逐步加 PID。

### 风险 3：舵机速度 / 间隙 / 延迟导致轨迹跟不上
- 处理：优先低频、小步、平滑轨迹；用 crawl 而不是 trot。

### 风险 4：逻辑分散在多个脚本里，越改越乱
- 处理：把核心逻辑收敛到 gait_generator / balance_controller / 主控制入口，测试脚本只做薄包装。

## Commands For Verification

```bash
# 离线仿真
python simulation_unified.py

# 当前传统 gait 主入口（后续会改造）
python main_trot.py

# 架空 walk 验证脚本
python test_walk_in_place.py

# 架空单腿验证
python test_single_leg_step.py
```

## Success Criteria

本次改造完成后，传统控制至少应达到：
- 不依赖 RL，即可完成低速稳定慢走
- 逻辑可解释：相位、重心偏移、抬腿、补偿都能说明白
- 可在仿真、架空、地面三阶段逐步验证
- 后续如果要再加 trot、转向、遥控输入，都能在这条主线上演进
