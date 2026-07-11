#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

class direction
{
public:
    direction(GPIO_TypeDef *_gpiox, uint16_t _pin1, uint16_t _pin2);
    enum class MOTOR_DIRECTION : uint8_t
    {
        forward,
        reversal
    };
    int set_dir(direction::MOTOR_DIRECTION dir);
private:
    GPIO_TypeDef *gpio_x;
    uint16_t pin1;
    uint16_t pin2;
};

class motor
{
public:
    motor(TIM_HandleTypeDef *_htim, uint32_t _channel, direction _dir);
    void start(void);
    void stop(void);
    void set_duty(uint32_t _duty);
    uint32_t get_freq(void);
    uint32_t get_period(void);
    direction dir;
private:
    TIM_HandleTypeDef *htim;
    uint32_t channel;
}; 
