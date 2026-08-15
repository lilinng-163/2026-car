#pragma once

#include <cstdint>
#include "pid.h"

// ===== 巡线双电机速度环任务 (vecpid) 共享接口 =====
// 被 key_task / uart_cmd_task / oled_task 读写，跨任务共享的全局均以 volatile 修饰。
// 控制核心见 vector_pid_task.cpp。

enum class running_event : uint8_t
{
    straight,   // 巡线误差小，直行状态
    turning     // 巡线误差大，弯道状态
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

// 运行时共享状态说明:
//   left/right_base_rpm      基础目标转速(外环叠加转向修正前的期望值)
//   left/right_actual_rpm    编码器实测转速(低通滤波后)
//   left/right_setpoint_rpm  外环修正后的内环设定值
//   left/right_out_val       内环 PID 输出(PWM 占空比满量程=period)
//   left/right_enc_total     编码器累计计数
//   vin_actual               电池电压实测(V, 用于前馈补偿)
