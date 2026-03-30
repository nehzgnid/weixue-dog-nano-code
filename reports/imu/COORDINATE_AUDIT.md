# 坐标系审计报告

> 本文档审计 weixue-dog-nano-code 项目中所有模块的坐标系定义，记录冲突点，供后续传统控制开发参考。

## 1. 用户声明

> **右手坐标系，Y 轴朝向机器狗前方**

即：X = Right, Y = Forward, Z = Up

## 2. 各模块实际约定

### 2.1 IK 运动学 — `kinematics_v5.py`

| 轴 | 实际含义 | 证据 |
|----|---------|------|
| X | **Forward**（前进方向） | `x_eff = x`，用于 hip 角度计算 `phi = atan2(x, -z)` |
| Y | **Lateral**（侧向，±L1） | FL 腿 `y = +L1`（左），FR 腿 `y = -L1`（右） |
| Z | **Up**（向上为正，脚为负） | BODY_HEIGHT = -180 |

隐含约定：**X = Forward, Y = Left, Z = Up**

### 2.2 仿真 — `simulation_unified.py`

```python
origins = {"FL": [dx, dy, 0], "FR": [dx, -dy, 0], "RL": [-dx, dy, 0], "RR": [-dx, -dy, 0]}
# dx = LENGTH/2 (前后), dy = WIDTH/2 (左右)
```

FL 在 (+x, +y) = 前左 → **X = Forward, Y = Left, Z = Up**。与 IK 一致。

### 2.3 步态 — `gait_generator.py`

`get_swing_pos` 返回 `(x, z)`，x 控制前后摆动，z 控制抬腿高度。
代码注释：`cp_x` 控制前后，`cp_z` 控制抬脚。

但注释中也暴露了方向未决：
```
# NOTE: User reported "Negative step length makes it go forward".
# Robot Front is -X in kinematics?
```

### 2.4 传统控制主循环 — `main_trot.py`

```python
# Coordinate System:
# X: Forward
# Y: Left
# Z: Up
```

注释明确声明 **X = Forward, Y = Left**。

### 2.5 RL 部署 — `obs_builder.py`

```python
# 根据配置文件 (Y朝前, X朝右)，需要将欧拉角重映射到机体坐标系:
# IMU Roll  (绕X/右侧旋转) -> 对应机体 Pitch (抬/低头)
# IMU Pitch (绕Y/前向旋转) -> 对应机体 -Roll  (左右倾侧)
robot_roll_rad  = -state["imu_pitch"] * (np.pi / 180.0)
robot_pitch_rad = state["imu_roll"] * (np.pi / 180.0)
```

RL 代码使用 **X = Right, Y = Forward**，与用户声明一致。
并做了 IMU→机器人坐标的轴重映射（Roll↔Pitch 互换 + 取反）。

## 3. 冲突清单

### 冲突 1：坐标轴定义不一致

| 维度 | 用户声明 + RL | IK + 传统控制脚本 |
|------|-------------|------------------|
| X | Right | **Forward** |
| Y | Forward | **Left** |
| Z | Up | Up |

IK/传统脚本使用的不是用户声明的坐标系。两者 X 和 Y 互换了。

### 冲突 2：IMU 轴重映射缺失

RL 的 `obs_builder.py` 已知 WT61C 安装方向为 Y-forward，做了正确的轴重映射：
```python
robot_roll  = -imu_pitch
robot_pitch = imu_roll
```

但 `main_trot.py` 和 `test_walk_in_place.py` 直接把 `imu['roll']` 当机器人 roll，`imu['pitch']` 当机器人 pitch。
**结果是 roll 和 pitch 修正方向完全颠倒。**

### 冲突 3：平衡补偿符号冲突

对于 `adj_roll > 0` 时 FL（前左腿）的行为：

| 文件 | FL 的 target_z 变化 | 腿的行为 |
|------|-------------------|---------|
| `simulation_unified.py` TrotSimFrame | `z_bal = adj_pitch - adj_roll` → 减小 → `-z_bal` 增大 → target_z 增大 | **腿缩短（收回）** |
| `test_walk_in_place.py` | `balance_z = -adj_roll` → 减小 → target_z 减小 | **腿伸长（伸出）** |

**相同 adj_roll 正值，两个脚本让 FL 做相反动作。**

### 冲突 4：step_length 方向未决

`gait_generator.py` 中有多处注释表明前进方向与 step_length 正负号的关系未确认：
- 用户反馈"负的 step length 让机器人前进"
- 代码注释猜测"Robot Front is -X in kinematics?"

## 4. WT61C IMU 数据链路

```
WT61C 硬件 → [UART 3] → STM32 imu_handler.c → g_imu_data.angle[0,1,2]
                                              → g_imu_data.acc[0,1,2]
                                              → g_imu_data.gyro[0,1,2]
           → [memcpy 36B] → USB CDC packet (0x40 RL_STATE)
           → [pyserial] → robot_io.py._parse_imu() → struct.unpack('<9f')
           → data[0:3]=acc, data[3:6]=gyro, data[6:8]=roll, data[7]=pitch, data[8]=yaw
           → imu_data['roll'] = data[6]  ← WT61C 原始 Roll
           → imu_data['pitch'] = data[7] ← WT61C 原始 Pitch
```

**注意**：WT61C 原始 Roll/Pitch 的物理含义取决于 IMU 安装方向。如果 Y-forward 安装：
- WT61C Roll = 绕 IMU X 轴旋转 = 绕机器人右左轴旋转 = **机器人 Pitch**
- WT61C Pitch = 绕 IMU Y 轴旋转 = 绕机器人前后轴旋转 = **机器人负 Roll**

## 5. 后续行动项

- [ ] 用 `test_imu_readout.py` 实测确认 WT61C Roll/Pitch 的物理含义
- [ ] 确认用户声明 (Y-forward) 后，决定传统控制代码统一到哪个坐标系
- [ ] 修复传统控制脚本中的 IMU 轴重映射缺失
- [ ] 统一平衡补偿符号
- [ ] 确认并修正 step_length 方向
