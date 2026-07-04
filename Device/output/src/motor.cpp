#include "motor.h"

motor::motor(TIM_HandleTypeDef *_htim, uint32_t _channel)
    : htim(_htim), channel(_channel)
{
}

void motor::start(void)
{
    HAL_TIM_PWM_Start(htim, channel);
}

void motor::stop(void)
{
    HAL_TIM_PWM_Stop(htim, channel);
}

void motor::set_duty(uint32_t _duty)
{
    __HAL_TIM_SET_COMPARE(htim, channel, _duty);
}

uint32_t motor::get_freq(void)
{
    uint32_t tim_clk = 0;

    if (htim->Instance == TIM1 || htim->Instance == TIM8 ||
        htim->Instance == TIM9 || htim->Instance == TIM10 ||
        htim->Instance == TIM11)
    {
        tim_clk = HAL_RCC_GetPCLK2Freq();
        if (tim_clk < HAL_RCC_GetHCLKFreq())
            tim_clk *= 2;
    }
    else
    {
        tim_clk = HAL_RCC_GetPCLK1Freq();
        if (tim_clk < HAL_RCC_GetHCLKFreq())
            tim_clk *= 2;
    }

    return tim_clk / (htim->Init.Prescaler + 1) / (htim->Init.Period + 1);
}

uint32_t motor::get_period(void)
{
    return htim->Init.Period;
}
