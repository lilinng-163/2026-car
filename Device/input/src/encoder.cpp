#include "encoder.h"

motor_encoder::motor_encoder(TIM_HandleTypeDef *_htim, int32_t _lines)
    : htim(_htim), enc_lines(_lines)
{
}

void motor_encoder::start(void)
{
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void motor_encoder::stop(void)
{
    HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
}

int32_t motor_encoder::get_count(void)
{
    return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim));
}

void motor_encoder::reset(void)
{
    __HAL_TIM_SET_COUNTER(htim, 0);
}
