#include <cmath>
#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "tim.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "vector_pid_task.h"

// 双电机速度环：编码器测速 -> PID -> PWM 输出（未验证）

static volatile float left_setpoint_rpm  = 0.0f;
static volatile float right_setpoint_rpm = 0.0f;

static constexpr const char *NAME  = "vecpid";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 4;
static constexpr float TS = 0.01f;   // 采样周期 10ms

direction left_dir(GPIOB, GPIO_PIN_0, GPIO_PIN_1);
direction right_dir(GPIOC, GPIO_PIN_8, GPIO_PIN_9);

motor left_motor(&htim2, TIM_CHANNEL_2, left_dir);
motor right_motor(&htim2, TIM_CHANNEL_3, right_dir);

motor_encoder left_enc(&htim3, 13);
motor_encoder right_enc(&htim4, 13);

static float period = static_cast<float>(left_motor.get_period());

pid left_motor_pid(1.0f, 0.3f, 0.01f, TS, -period, period);
pid right_motor_pid(1.0f, 0.3f, 0.01f, TS, -period, period);

// 把 PID 输出施加到电机：符号决定方向，绝对值作占空比
static void motor_apply_output(motor &mot, float out)
{
    if (out >= 0.0f)
    {
        mot.dir.set_dir(direction::MOTOR_DIRECTION::forward);
        mot.set_duty(static_cast<uint32_t>(out));
    }
    else
    {
        mot.dir.set_dir(direction::MOTOR_DIRECTION::reversal);
        mot.set_duty(static_cast<uint32_t>(-out));
    }
}

// 速度环任务：每 10ms 测速 -> PID 计算 -> 输出
static void vector_pid_task_entry(void *pv)
{
    (void)pv;
    TickType_t last_wake = xTaskGetTickCount();
    static constexpr TickType_t ticks = pdMS_TO_TICKS(10);

    left_enc.start();
    right_enc.start();
    left_motor.start();
    right_motor.start();

    while (1)
    {
        vTaskDelayUntil(&last_wake, ticks);

        float l_rpm = left_enc.get_rpm();
        float r_rpm = right_enc.get_rpm();

        float left_out  = left_motor_pid.calculate(left_setpoint_rpm, l_rpm);
        float right_out = right_motor_pid.calculate(right_setpoint_rpm, r_rpm);

        motor_apply_output(left_motor, left_out);
        motor_apply_output(right_motor, right_out);
    }
}

void vector_pid_task_create(void)
{
    xTaskCreate(vector_pid_task_entry, NAME, STACK, NULL, PRIO, NULL);
}

void set_left_target_rpm(float rpm)
{
    left_setpoint_rpm = rpm;
}

void set_right_target_rpm(float rpm)
{
    right_setpoint_rpm = rpm;
}

void set_both_target_rpm(float rpm)
{
    left_setpoint_rpm  = rpm;
    right_setpoint_rpm = rpm;
}

// 实际转速 = 目标 - 误差(误差取自 PID 内部 state[0])
float get_left_actual_rpm(void)
{
    return left_setpoint_rpm - left_motor_pid.get_instance().state[0];
}

float get_right_actual_rpm(void)
{
    return right_setpoint_rpm - right_motor_pid.get_instance().state[0];
}
