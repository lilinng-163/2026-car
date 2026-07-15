#pragma once

#include <cstdint>
#include "pid.h"

extern pid left_motor_pid;
extern pid right_motor_pid;
extern pid track_pid;

extern volatile float left_base_rpm;
extern volatile float right_base_rpm;
extern volatile float left_actual_rpm;
extern volatile float right_actual_rpm;
extern volatile float left_setpoint_rpm;
extern volatile float right_setpoint_rpm;
extern volatile float left_out_val;
extern volatile float right_out_val;
extern volatile float vin_actual;

void vector_pid_task_create(void);

// 设定目标转速(RPM)
void set_left_target_rpm(float rpm);
void set_right_target_rpm(float rpm);
void set_both_target_rpm(float rpm);

// 读取实际转速(RPM，由 setpoint - 误差 反推)
float get_left_actual_rpm(void);
float get_right_actual_rpm(void);
