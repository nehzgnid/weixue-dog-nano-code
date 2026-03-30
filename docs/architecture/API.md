# Weixue Dog Nano - API 接口文档

## 1. 模块概览

本项目包含三个核心模块，分别负责运动学计算、步态生成、平衡控制及实时交互。

### 1.1 核心模块列表

| 模块文件名 | 功能描述 | 依赖 |
| :--- | :--- | :--- |
| `kinematics_v5.py` | 逆运动学 (IK) 与 正运动学 (FK) 计算 | `numpy`, `robot_config` |
| `gait_generator.py` | 步态轨迹生成器 (贝塞尔曲线) | `numpy` |
| `balance_controller.py` | 姿态平衡 PID 控制器 | `numpy` |
| `robot_io.py` | 串口通信与舵机指令封装 | `serial` |
| `control_center_unified.py` | **统一控制终端** (GUI + 动作执行) | `tkinter`, `threading` |

---

## 2. kinematics_v5 模块 API

### 类: `LegKinematics`

负责单腿的逆解算。

#### 构造函数

```python
LegKinematics(name: str)
```
*   **参数**:
    *   `name`: 腿部名称 (`"FL"`, `"FR"`, `"RL"`, `"RR"`)

#### 方法

##### solve_ik(target_x, target_y, target_z)

逆运动学求解：根据目标坐标计算三个关节的角度。

```python
def solve_ik(target_x: float, target_y: float, target_z: float) -> tuple | None
```
*   **参数**:
    *   `target_x`: 足端相对于髋关节 X 轴坐标 (mm)
    *   `target_y`: 足端相对于髋关节 Y 轴坐标 (mm)
    *   `target_z`: 足端相对于髋关节 Z 轴坐标 (mm)
*   **返回值**: 
    *   成功: `(q1_rad, q2_rad, q3_rad)` - 三个关节的弧度值
    *   失败: `None` (超出可达空间)

##### rad_to_pwm(q1, q2, q3)

将弧度转换为 PWM 脉宽值。

```python
def rad_to_pwm(q1: float, q2: float, q3: float) -> list[int]
```
*   **返回值**: `[pwm_abd, pwm_hip, pwm_knee]`

##### forward_kinematics_strict(q1, q2, q3)

正向运动学：根据关节角度计算足端坐标。

```python
def forward_kinematics_strict(q1: float, q2: float, q3: float) -> tuple
```
*   **返回值**: `(p0, p1, p2, p3)` - 髋关节、膝关节、足端的 3D 坐标点

---

## 3. gait_generator 模块 API

### 类: `BezierGait`

使用贝塞尔曲线生成步态轨迹。

#### 方法

##### get_swing_pos(t, step_length, step_height)

计算摆动相 (Swing Phase) 足端位置。

```python
def get_swing_pos(t: float, step_length: float, step_height: float) -> tuple[float, float]
```
*   **参数**:
    *   `t`: 归一化时间 (0.0 ~ 1.0)
    *   `step_length`: 步长 (mm)
    *   `step_height`: 抬腿高度 (mm)
*   **返回值**: `(x, z)` - 足端的 X 和 Z 轴偏移

##### get_stance_pos(t, step_length)

计算支撑相 (Stance Phase) 足端位置。

```python
def get_stance_pos(t: float, step_length: float) -> tuple[float, float]
```

---

## 4. balance_controller 模块 API

### 类: `BalanceController`

PID 平衡控制器。

#### 构造函数

```python
BalanceController()
```

#### 方法

##### solve(current_roll, current_pitch, target_roll, target_pitch, dt)

计算姿态修正量。

```python
def solve(current_roll: float, current_pitch: float, target_roll: float, target_pitch: float, dt: float) -> tuple
```
*   **返回值**: `(adjust_roll, adjust_pitch)` - 修正后的 Roll 和 Pitch 角度

---

## 5. robot_io 模块 API

### 类: `RobotIO`

负责与下位机串口通信。

#### 属性

*   `ser`: 串口对象 (`serial.Serial` 或 `None`)
*   `servo_states`: 字典，存储所有舵机的实时状态 `{id: {'pos': ..., 'spd': ..., ...}}`

#### 方法

##### connect(port: str)

连接串口。

```python
def connect(self, port: str) -> None
```

##### send_torque(enable: bool)

使能/失能 所有舵机扭矩。

```python
def send_torque(self, enable: bool) -> None
```

##### send_servos(cmds: list)

发送舵机目标位置指令。

```python
def send_servos(self, cmds: list[tuple]) -> None
```
*   **参数**: `cmds` - 命令列表，格式: `[(id, position, time_ms, speed, acc), ...]`
    *   `id`: 舵机 ID
    *   `position`: 目标 PWM 值
    *   `time_ms`: 目标时间 (ms)
    *   `speed`: 速度限制 (0-1000)
    *   `acc`: 加速度限制

---

## 6. control_center_unified 模块 API

### 类: `ManualControlFrame`

手动控制与动作序列执行器。

#### 动作关键帧数据 (属性)

项目内置了以下 PWM 关键帧数据：

```python
POSE_STAND = {"1": 2048, "2": 1527, ...}  # 站立
POSE_SIT   = {"1": 1995, "2": 1170, ...}  # 坐下
POSE_BOW_1 = {"1": 2534, "2": 1594, ...} # 作揖姿态 1
POSE_BOW_2 = {"1": 2535, "2": 1227, ...} # 作揖姿态 2
```

#### 方法

##### perform_synchronized_move(targets: dict, duration_sec: float, acc: int = 0)

执行同步平滑运动。

```python
def perform_synchronized_move(self, targets: dict, duration_sec: float, acc: int = 0) -> None
```
*   **参数**:
    *   `targets`: 目标 PWM 字典 `{"1": 2048, ...}`
    *   `duration_sec`: 运动时长 (秒)
    *   `acc`: 加速度限制

**特性**:
*   支持 **Native Mode** (使用硬件时间控制) 和 **Software Interpolation** (软件插值) 两种模式。
*   **最短路径插值**: 自动计算跨越 0/4095 边界的最短路径。

##### run_sit_only()

异步线程函数：执行坐下动作。

##### run_bow_only()

异步线程函数：执行作揖动作（3次循环）。

##### run_new_year_sequence()

异步线程函数：执行完整的拜年动作序列。

##### copy_pwm()

复制当前所有舵机位置到剪贴板 (JSON 格式)。

---

## 7. 数据结构与常量

### 舵机 ID 映射

| ID | 部位 |
| :--- | :--- |
| 1, 2, 3 | 左前腿 (FL): 髋关节(Hip), 大腿(Knee), 小腿(Ankle) |
| 4, 5, 6 | 右前腿 (FR): 同上 |
| 7, 8, 9 | 左后腿 (RL): 同上 |
| 10, 11, 12 | 右后腿 (RR): 同上 |

### 通信协议

使用 TTL 串口 (默认 `115200` 波特率)，遵循 Dynamixel 协议格式。
