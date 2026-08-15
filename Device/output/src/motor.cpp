/**
 * @file    motor.cpp
 * @brief   直流电机 PWM + H 桥方向控制驱动实现
 *
 *          direction: 两路 GPIO 控制 H 桥 IN1/IN2 方向
 *          motor:     定时器 PWM 输出控制占空比，另提供频率/周期查询
 *                     (周期 period 用于换算 PID 输出到占空比的满量程)。
 */

#include <cstdio>
#include <cstdint>
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "motor.h"

direction::direction(GPIO_TypeDef *_gpiox, uint16_t _pin1, uint16_t _pin2)
: gpio_x(_gpiox), pin1(_pin1), pin2(_pin2)
{

}

// 设置 H 桥方向: forward=IN1高IN2低, reversal=IN1低IN2高
int direction::set_dir(direction::MOTOR_DIRECTION dir)
{
    if(dir == MOTOR_DIRECTION::forward)
    {
        HAL_GPIO_WritePin(gpio_x, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(gpio_x, pin2, GPIO_PIN_RESET);
    }
    else if(dir == MOTOR_DIRECTION::reversal)
    {
        HAL_GPIO_WritePin(gpio_x, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(gpio_x, pin2, GPIO_PIN_SET);      
    }
    return 0;
}

motor::motor(TIM_HandleTypeDef *_htim, uint32_t _channel, direction _dir)
    : htim(_htim), channel(_channel), dir(_dir)
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

// 查询 PWM 实际频率(Hz): 由定时器时钟、预分频、周期反推
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

// 当前定时器自动重载值(period)，作为 PWM 占空比满量程(100%)
uint32_t motor::get_period(void)
{
    return htim->Init.Period;
}