#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

// H 桥方向控制(两路 GPIO)
class direction
{
public:
    direction(GPIO_TypeDef *_gpiox, uint16_t _pin1, uint16_t _pin2);
    enum class MOTOR_DIRECTION : uint8_t
    {
        forward,   // IN1=高, IN2=低
        reversal   // IN1=低, IN2=高
    };
    int set_dir(direction::MOTOR_DIRECTION dir);
private:
    GPIO_TypeDef *gpio_x;
    uint16_t pin1;
    uint16_t pin2;
};

// PWM 电机驱动
class motor
{
public:
    motor(TIM_HandleTypeDef *_htim, uint32_t _channel, direction _dir);
    void start(void);
    void stop(void);
    void set_duty(uint32_t _duty);   // 占空比比较值(0~period)
    uint32_t get_freq(void);         // PWM 频率(Hz)
    uint32_t get_period(void);       // 自动重载值(占空比满量程)
    direction dir;
private:
    TIM_HandleTypeDef *htim;
    uint32_t channel;
}; 
