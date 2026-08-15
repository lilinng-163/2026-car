#pragma once

#include <cstdint>
#include "stm32f407xx.h"

// LED 驱动: 低电平点亮
class led
{
public:
    led(GPIO_TypeDef *_gpio_port, uint16_t _gpio_num);
    int on(void);            // 点亮
    int off(void);           // 熄灭
    bool get_status(void);   // 当前是否点亮
    int toggle(void);        // 翻转
private:
    bool is_on;
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_num;
};