#pragma once
#include <cstdint>

// 步进电机测试任务，创建入口 (STEP=TIM8_CH1/PC6, DIR=PG6, EN=PG8)
void stepper_task_create(void);

extern volatile int32_t stepper_steps;   // 当前累计步数(带方向, 供调试)
