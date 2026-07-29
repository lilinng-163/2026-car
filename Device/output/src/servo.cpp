#include "servo.h"

servo::servo(TIM_HandleTypeDef *_htim, uint32_t _channel, float max_angle)
    : htim(_htim), channel(_channel), max_deg(max_angle)
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
    constexpr float min_pulse = 500.0f;
    constexpr float max_pulse = 2500.0f;

    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > max_deg)
        angle = max_deg;

    float pulse_us = min_pulse + (max_pulse - min_pulse) * angle / max_deg;
    uint32_t compare = (uint32_t)(pulse_us);   // PSC=167 -> 1 tick = 1us

    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}
