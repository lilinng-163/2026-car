#pragma once

void imu_task_create(void);

extern volatile float imu_yaw;     // IMU 欧拉角 yaw (rad)
extern volatile float imu_gz;      // IMU 陀螺仪 Z 轴角速度 (raw)
