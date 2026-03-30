# IMU 观测坐标自动审计报告

- 时间: 2026-03-29 06:14:52
- 串口: /dev/ttyACM0
- Y 前向约束: 开启
- 手性约束: 仅右手系
- 当前配置来源: /workspace/weixue-dog-nano-code/sim2real_jetson_tests/strategy_deploy_bundle/config/wavego_deploy_config.yaml

## RAW 陀螺轴能量 (RMS)

- 俯仰段: X=30.0429, Y=5.4206, Z=5.8349
- 侧倾段: X=3.2140, Y=30.6245, Z=2.0996
- 平转段: X=0.1428, Y=0.4897, Z=21.9559

## 最佳候选

- imu_axis_remap: [0, 1, 2]
- imu_axis_sign: [-1.0, -1.0, 1.0]
- euler_map: obs_roll=-raw_pitch, obs_pitch=-raw_roll
- handedness: 右手系
- vector_map: OBS_X = -RAW_X | OBS_Y = -RAW_Y | OBS_Z = +RAW_Z
- score(total/axis/corr/gravity_err): 60.9539/29.2249/1.8953/0.04668

## 当前仓库配置候选

- imu_axis_remap: [1, 0, 2]
- imu_axis_sign: [1.0, -1.0, 1.0]
- euler_map: obs_roll=-raw_pitch, obs_pitch=+raw_roll
- handedness: 右手系
- vector_map: OBS_X = +RAW_Y | OBS_Y = -RAW_X | OBS_Z = +RAW_Z
- score(total/axis/corr/gravity_err): 0.2221/0.1411/-0.0338/0.00398
- rank_in_search: 当前配置不满足本次约束，未参与排名

## 结论

- FAIL: 当前仓库配置与最佳候选不一致
- 分数差: +60.7318