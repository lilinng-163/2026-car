#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

class motor
{
public:
    motor(TIM_HandleTypeDef *_htim, uint32_t _channel);
    void start(void);
    void stop(void);
    void set_duty(uint32_t _duty);
    uint32_t get_freq(void);
    uint32_t get_period(void);
private:
    TIM_HandleTypeDef *htim;
    uint32_t channel;
};
