#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

// 舵机驱动: 角度 -> 脉宽(500~2500us)线性映射
class servo
{
public:
    servo(TIM_HandleTypeDef *_htim, uint32_t _channel, float max_angle = 180.0f);
    void start(void);
    void stop(void);
    void set_angle(float angle);   // 角度(°) 限幅 [0, max_angle]
private:
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float max_deg;
};
