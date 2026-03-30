# IMU 观测坐标自动审计报告

- 时间: 2026-03-29 05:29:10
- 串口: /dev/ttyACM0
- Y 前向约束: 开启

## RAW 陀螺轴能量 (RMS)

- 俯仰段: X=17.1743, Y=6.5274, Z=1.3204
- 侧倾段: X=5.4145, Y=38.4331, Z=3.4649
- 平转段: X=0.5689, Y=0.6216, Z=24.4034

## 最佳候选

- imu_axis_remap: [0, 1, 2]
- imu_axis_sign: [-1.0, -1.0, 1.0]
- euler_map: obs_roll=-raw_pitch, obs_pitch=-raw_roll
- vector_map: OBS_X = -RAW_X | OBS_Y = -RAW_Y | OBS_Z = +RAW_Z
- score(total/axis/corr/gravity_err): 58.6131/28.0560/1.8906/0.04556

## 当前仓库配置候选

- imu_axis_remap: [1, 0, 2]
- imu_axis_sign: [1.0, -1.0, 1.0]
- euler_map: obs_roll=-raw_pitch, obs_pitch=+raw_roll
- vector_map: OBS_X = +RAW_Y | OBS_Y = -RAW_X | OBS_Z = +RAW_Z
- score(total/axis/corr/gravity_err): 7.8302/3.6712/0.3576/0.00404
- rank_in_search: 当前配置不满足本次约束，未参与排名

## 结论

- FAIL: 当前仓库配置与最佳候选不一致
- 分数差: +50.7830