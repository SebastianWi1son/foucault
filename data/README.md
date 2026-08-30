# data/ — 测试数据集

- `NAV2_data.bin`：NAV2 数据集（源：reference/IMU_Attitude_Estimator 参考库，2026-08-30 复制入库，因 reference/ 整体不入 git）。
- 格式：文本，每行 12 列空格分隔：acc3(g) gyro3(rad/s) mag3(忽略) 真值euler3(rad, roll/pitch/yaw)；50Hz。
- 坐标系：世界系 z-down（NED 风格），喂入 core 前 acc 取反（见 host/replay/replay_nav2.cpp 头注释）。
