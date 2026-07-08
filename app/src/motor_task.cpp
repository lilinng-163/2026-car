#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "motor.h"
#include "pid.h"
#include "motor_task.h"

pid left_motor_pid(1.0, 0.3, 0.01, 0, 10);
pid right_motor_pid(1.0, 0.3, 0.01, 0, 10);

int motor_task(void *pv)
{
    (void)pv;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    while (1) 
    {
        vTaskDelayUntil(&last_wake, period);

        //速度环

        //结果pwm给电机

        //差速暂无实现
    }
}
