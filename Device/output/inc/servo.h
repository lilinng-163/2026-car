#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

class servo
{
public:
    servo(TIM_HandleTypeDef *_htim, uint32_t _channel, float max_angle = 180.0f);
    void start(void);
    void stop(void);
    void set_angle(float angle);
private:
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float max_deg;
};
