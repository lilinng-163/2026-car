#pragma once

#include <cstdint>
#include "pid.h"

enum class running_event : uint8_t
{
    straight,
    turning
};

extern volatile running_event r_e;
extern volatile uint16_t lap_count;
extern volatile uint8_t  corner_count;
extern volatile float    target_laps;

extern pid left_motor_pid;
extern pid right_motor_pid;
extern pid yaw_pid;
extern volatile float steer_kp;
extern volatile float steer_kd;
extern volatile float gz_k;
extern volatile float turn_k;
extern volatile float target_yaw;
extern volatile float yaw_gain;

extern volatile float left_base_rpm;
extern volatile float right_base_rpm;
extern volatile float left_actual_rpm;
extern volatile float right_actual_rpm;
extern volatile float left_setpoint_rpm;
extern volatile float right_setpoint_rpm;
extern volatile float left_out_val;
extern volatile float right_out_val;
extern volatile float vin_actual;
extern volatile int32_t left_enc_total;
extern volatile int32_t right_enc_total;

void vector_pid_task_create(void);

// 设定目标转速(RPM)
void set_left_target_rpm(float rpm);
void set_right_target_rpm(float rpm);
void set_both_target_rpm(float rpm);

// 读取实际转速(RPM，由 setpoint - 误差 反推)
float get_left_actual_rpm(void);
float get_right_actual_rpm(void);
