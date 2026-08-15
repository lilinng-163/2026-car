/**
 * @file    servo.cpp
 * @brief   舵机 PWM 驱动实现
 *
 *          角度 -> 脉宽线性映射: 0°=500us ~ max_angle°=2500us。
 *          依赖定时器 PSC=167(F4@168MHz -> 1 tick=1us)，PWM 周期由 CubeMX 配置。
 */

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

// 设置舵机角度(°)：限幅到 [0, max_deg]，换算脉宽并写入比较寄存器
void servo::set_angle(float angle)
{
    constexpr float min_pulse = 500.0f;   // 0° 对应脉宽(us)
    constexpr float max_pulse = 2500.0f;  // max_deg 对应脉宽(us)

    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > max_deg)
        angle = max_deg;

    float pulse_us = min_pulse + (max_pulse - min_pulse) * angle / max_deg;
    uint32_t compare = (uint32_t)(pulse_us);   // PSC=167 -> 1 tick = 1us

    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}
