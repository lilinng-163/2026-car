#pragma once

void imu_task_create(void);

extern volatile float imu_roll;
extern volatile float imu_pitch;
extern volatile float imu_yaw;
extern volatile float imu_gz;
