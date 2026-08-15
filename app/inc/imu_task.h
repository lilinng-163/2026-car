#pragma once

void imu_task_create(void);

// IMU 姿态全局(弧度)，由 imu_task 周期性刷新，供巡线外环/球平衡读取
extern volatile float imu_roll;    // 横滚角 (rad)
extern volatile float imu_pitch;   // 俯仰角 (rad)
extern volatile float imu_yaw;     // 偏航角 (rad, 用于 yaw 保持)
extern volatile float imu_gz;      // 陀螺仪 Z 轴角速度(原始 int16 值, 用于阻尼/圈数统计)
