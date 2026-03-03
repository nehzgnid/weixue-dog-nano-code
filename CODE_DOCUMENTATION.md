# Weixue Dog Nano - 代码说明文档

## 1. 项目概述

**Weixue Dog Nano** 是一款基于 Python 开发的四足机器人控制框架。该项目实现了从运动学解算、步态规划到实时控制的全套软件栈，支持包括**站立、坐下、行走、拜年作揖**等多种动作模式。

### 硬件平台
*   **主控**: Python (PC / Raspberry Pi)
*   **通信**: 串口 (TTL, 115200 Baud)
*   **执行器**: 串口舵机 (如 Dynamixel 兼容型号)
*   **结构**: 12 自由度 (4 legs x 3 joints)

---

## 2. 架构设计

### 2.1 模块分层

```
┌─────────────────────────────────────────────┐
│     User Interface (GUI / Simulation)       │
│  simulation_unified.py / control_center...  │
├─────────────────────────────────────────────┤
│         Control Logic & FSM                 │
│  gait_generator.py / balance_controller.py │
├─────────────────────────────────────────────┤
│         Kinematics Engine                   │
│           kinematics_v5.py                  │
├─────────────────────────────────────────────┤
│         Hardware Abstraction                │
│            robot_io.py                     │
└─────────────────────────────────────────────┘
```

### 2.2 核心状态机 (FSM)

*   **RELAX**: 扭矩关闭，机器狗处于无力状态，可手动摆放姿态。
*   **MANUAL**: 手动模式，用户通过滑块或预设动作控制。
*   **TROT**: 动态步态行走模式。
*   **TEST_SINGLE_LEG**: 单腿测试模式。

---

## 3. 核心模块详解

### 3.1 kinematics_v5.py (运动学)

这是最底层的数学引擎，负责将**空间坐标**转换为**舵机角度**。

#### 逆运动学 (Inverse Kinematics)
核心函数 `solve_ik(target_x, target_y, target_z)` 使用解析法求解三关节机械臂的逆解。

**数学原理简述**:
1.  **髋关节 (Hip)**: 控制水平面上的旋转。
2.  **膝关节 (Knee)**: 控制小腿相对于大腿的角度。
3.  **踝关节 (Ankle)**: 保持足端与地面平行。

> **注意**: `target_z` 为负值表示向下（朝向地面）。

#### 正运动学 (Forward Kinematics)
函数 `forward_kinematics_strict(q1, q2, q3)` 用于仿真界面绘图，根据当前关节角度反推足端在 3D 空间中的位置。

### 3.2 gait_generator.py (步态生成)

使用 **贝塞尔曲线 (Bezier Curve)** 生成平滑的步态轨迹。

*   **Swing Phase (摆动相)**: 足端离地移动。使用 `get_swing_pos` 计算。
    *   轨迹：抛物线或正弦波，确保离地间隙。
*   **Stance Phase (支撑相)**: 足端贴地移动。使用 `get_stance_pos` 计算。
    *   轨迹：直线或平滑曲线，推动身体前进。

### 3.3 balance_controller.py (平衡控制)

这是一个典型的 **PID 控制器**。

*   **输入**: IMU 实时读取的 Roll (横滚) 和 Pitch (俯仰) 角度。
*   **输出**: 腿部 Z 轴的补偿量 (Compensation)。
*   **逻辑**:
    *   如果身体前倾 (Pitch > 0)，则前腿伸长，后腿缩短，将身体“推”回原位。

### 3.4 robot_io.py (通信层)

负责将指令封装为串口协议。

#### 关键机制
1.  **发送**: `send_servos(cmds)` - 批量发送舵机指令。
2.  **接收**: 在后台线程中不断读取串口缓冲区，更新 `self.servo_states` 字典，实现舵机状态的实时反馈。

### 3.5 control_center_unified.py (控制终端)

这是面向用户的**主程序**，集成了仿真与真机控制。

#### 动作序列实现原理

以 **"拜年动作"** 为例：

1.  **关键帧数据**: 预先采集好 Sit (坐下), Bow 1, Bow 2 的 PWM 字典。
2.  **平滑插值 (Interpolation)**:
    *   使用 `_internal_move_to(target_pose, duration)` 函数。
    *   **算法**: `pos = start + (target - start) * t_eased`。
    *   **t_eased**: 使用 `3t^2 - 2t^3` (三次多项式) 或 `sin` 函数，确保速度曲线平滑。
3.  **最短路径算法 (Shortest Path)**:
    *   处理 0/4095 边界跨越问题。
    *   公式:
        ```python
        diff = target - start
        if diff > 2048: diff -= 4096  # 跨越 0 点顺时针
        elif diff < -2048: diff += 4096 # 跨越 0 点逆时针
        ```
4.  **速度限制**: 强制 `Speed Limit = 1000`，防止舵机响应过快损坏机械结构。
5.  **后腿锁定**:
    *   在作揖循环中，强制覆盖前腿的目标 PWM，而**忽略**后腿的目标 PWM，直接使用 Sit 姿态的 PWM，从而实现“后腿不动”。

---

## 4. 使用指南

### 4.1 环境搭建

```bash
# 安装依赖
pip install numpy pyserial
```

### 4.2 真机控制

1.  连接串口线。
2.  运行:
    ```bash
    python control_center_unified.py
    ```
3.  选择端口，点击 **Connect**。
4.  进入 **Manual Control** 标签页，点击 **Enable Manual**。
5.  使用滑块调节姿态，或选择 **Action** 下拉框中的动作。

### 4.3 仿真调试

1.  运行:
    ```bash
    python simulation_unified.py
    ```
2.  切换到 **Static IK Control** 标签页。
3.  调整滑块查看 3D 模拟效果，或选择 **Action** -> **Happy New Year** 观看动画演示。

---

## 5. 常见问题排查

### Q1: 机器人在坐下时向后翻倒？
**A**: 重心 (CoM) 不够靠后。可以减小前腿的高度 (`Height` 滑块向负方向调大)，或增大 `Pitch` 角度。

### Q2: 舵机发出嗡嗡声但不转动？
**A**: 检查电源是否足够 (12V 3A 以上)。检查扭矩是否使能 (`send_torque(True)`).

### Q3: 仿真中动作正常，真机动作错乱？
**A**: 检查机械结构组装是否正确，特别是关节方向 (`dir_hip`, `dir_knee`) 是否与代码一致。

---

## 6. 开发路线图

- [x] 基础运动学 (IK/FK)
- [x] 站立/坐下/趴下姿态
- [x] 步态生成器 (Trot)
- [x] 平衡控制器 (PID)
- [x] 拜年作揖动作
- [ ] 视觉追踪辅助
- [ ] 地形自适应步态
