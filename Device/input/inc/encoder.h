#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

class motor_encoder
{
public:
    motor_encoder(TIM_HandleTypeDef *_htim, int32_t _lines);

    void start(void);
    void stop(void);
    int32_t get_count(void);
    void reset(void);
    int32_t lines(void) const { return enc_lines; }

private:
    TIM_HandleTypeDef *htim;
    int32_t enc_lines;
};
