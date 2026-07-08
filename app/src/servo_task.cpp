#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "tim.h"
#include "servo.h"
#include "servo_task.h"

static constexpr const char *NAME  = "servo";
static constexpr configSTACK_DEPTH_TYPE STACK = 128;
static constexpr UBaseType_t PRIO = 3;

static void servo_task(void *pv)
{
    (void)pv;

    servo s(&htim2, TIM_CHANNEL_1);

    float angle = 0.0f;
    int8_t dir = 1;

    while (1)
    {
        s.set_angle(angle);

        angle += dir * 2.0f;
        if (angle >= 180.0f)
        {
            angle = 180.0f;
            dir = -1;
        }
        else if (angle <= 0.0f)
        {
            angle = 0.0f;
            dir = 1;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void servo_task_create(void)
{
    xTaskCreate(servo_task, NAME, STACK, NULL, PRIO, NULL);
}