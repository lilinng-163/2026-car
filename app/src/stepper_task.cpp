/**
 * @file    stepper_task.cpp
 * @brief   步进电机测试任务 (stepper)
 *
 *          周期驱动步进电机正/反转固定步数往复运动，用于驱动通路测试。
 *          引脚: STEP=TIM8_CH1(PC6), DIR=PG6, EN=PG8。
 */

#include <cstdio>
#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "tim.h"
#include "stepper.h"
#include "stepper_task.h"

static constexpr const char *NAME  = "stepper";
static constexpr configSTACK_DEPTH_TYPE STACK = 512;
static constexpr UBaseType_t PRIO = 3;

static constexpr uint32_t STEP_HZ   = 1000;   // 步进频率(Hz)
static constexpr uint32_t LEG_STEPS = 200;    // 每段步数

volatile int32_t stepper_steps = 0;

static void stepper_task(void *pv)
{
    (void)pv;

    stepper st(&htim8, TIM_CHANNEL_1, GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_8);
    st.set_speed(STEP_HZ);

    while (1)
    {
        // 正转 LEG_STEPS 步
        st.set_dir(true);
        st.start();
        while (st.get_step_count() < LEG_STEPS)
            vTaskDelay(pdMS_TO_TICKS(1));
        st.stop();
        stepper_steps += (int32_t)LEG_STEPS;
        printf("stepper +%lu -> %ld\r\n", (unsigned long)LEG_STEPS, stepper_steps);

        vTaskDelay(pdMS_TO_TICKS(500));

        // 反转 LEG_STEPS 步
        st.set_dir(false);
        st.start();
        while (st.get_step_count() < LEG_STEPS)
            vTaskDelay(pdMS_TO_TICKS(1));
        st.stop();
        stepper_steps -= (int32_t)LEG_STEPS;
        printf("stepper -%lu -> %ld\r\n", (unsigned long)LEG_STEPS, stepper_steps);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void stepper_task_create(void)
{
    xTaskCreate(stepper_task, NAME, STACK, NULL, PRIO, NULL);
}
