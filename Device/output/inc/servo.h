#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

class servo
{
public:
    servo(TIM_HandleTypeDef *_htim, uint32_t _channel);
    void start(void);
    void stop(void);
    void set_angle(float angle);
private:
    TIM_HandleTypeDef *htim;
    uint32_t channel;
};