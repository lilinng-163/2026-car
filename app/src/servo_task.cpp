#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "tim.h"
#include "pid.h"
#include "servo.h"
#include "servo_task.h"

static constexpr const char *NAME  = "servo";
static constexpr configSTACK_DEPTH_TYPE STACK = 256;
static constexpr UBaseType_t PRIO = 3;

volatile float ball_pos       = 0.0f;
volatile float ball_setpoint  = 0.0f;
volatile float servo_angle    = 135.0f;

static constexpr float TS = 0.01f;
static constexpr float BASE_ANGLE = 135.0f;

static pid ball_pid(0.8f, 0.0f, 0.15f, TS, -30.0f, 30.0f);

static void servo_task(void *pv)
{
    (void)pv;

    servo s(&htim9, TIM_CHANNEL_1, 270.0f);
    s.start();
    s.set_angle(BASE_ANGLE);

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        float err = ball_setpoint - ball_pos;
        float out = ball_pid.calculate(0.0f, err);

        servo_angle = BASE_ANGLE + out;
        s.set_angle(servo_angle);
    }
}

void servo_task_create(void)
{
    xTaskCreate(servo_task, NAME, STACK, NULL, PRIO, NULL);
}
