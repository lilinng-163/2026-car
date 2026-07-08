#include "servo.h"

servo::servo(TIM_HandleTypeDef *_htim, uint32_t _channel)
    : htim(_htim), channel(_channel)
{
}

void servo::start(void)
{
    HAL_TIM_PWM_Start(htim, channel);
}

void servo::stop(void)
{
    HAL_TIM_PWM_Stop(htim, channel);
}

void servo::set_angle(float angle)
{
    // 标准舵机：0.5ms~2.5ms 脉宽对应 0~180°，周期 20ms (50Hz)
    constexpr float min_pulse = 500.0f;
    constexpr float max_pulse = 2500.0f;
    constexpr float period_us = 20000.0f;

    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > 180.0f)
        angle = 180.0f;

    float pulse_us = min_pulse + (max_pulse - min_pulse) * angle / 180.0f;
    uint32_t compare = (uint32_t)(pulse_us / period_us * (htim->Init.Period + 1));

    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}
