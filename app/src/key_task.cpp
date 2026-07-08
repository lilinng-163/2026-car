#include <cstdio>
#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "key.h"
#include "motor_task.h"
#include "lv_obj.h"

static constexpr const char *NAME  = "key";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 3;

static void key_task(void *pv) 
{
    (void)pv;
    key k0(GPIOF, GPIO_PIN_9);
    key k1(GPIOF, GPIO_PIN_8);
    key k2(GPIOF, GPIO_PIN_7);
    key k3(GPIOF, GPIO_PIN_6);
    printf("key_task start\n");

    while (1) 
    {
        // key1：上一页（page--），临界区只改标志位
        if (k1.key_tick() == key_event::click)
        {
            taskENTER_CRITICAL();
            ui_page_prev();
            taskEXIT_CRITICAL();
            printf("key1 click -> prev page\r\n");
        }

        // key3：下一页（page++），临界区只改标志位
        if (k3.key_tick() == key_event::click)
        {
            taskENTER_CRITICAL();
            ui_page_next();
            taskEXIT_CRITICAL();
            printf("key3 click -> next page\r\n");
        }

        // key0：左电机 Kp += 0.2（共享 PID 对象，show_pv 只读刷新）
        if(k0.key_tick() == key_event::click)
        {
            arm_pid_instance_f32 l = left_motor_pid.get_instance();
            left_motor_pid.set_gains(l.Kp + 0.2f, l.Ki, l.Kd);
            printf("k0 click -> left_motor_pid_kp + 0.2\r\n");
        }
        // key2：左电机 Kp -= 0.2
        if(k2.key_tick() == key_event::click)
        {
            arm_pid_instance_f32 l = left_motor_pid.get_instance();
            left_motor_pid.set_gains(l.Kp - 0.2f, l.Ki, l.Kd);
            printf("k0 click -> left_motor_pid_kp - 0.2\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void key_task_create() 
{
    xTaskCreate(key_task, NAME, STACK, NULL, PRIO, NULL);
}