#pragma once

#include <cstdint>
#include "stm32f407xx.h"

class led
{
public:
    led(GPIO_TypeDef *_gpio_port, uint16_t _gpio_num);
    int on(void);
    int off(void);
    bool get_status(void);
    int toggle(void);
private:
    bool is_on;
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_num;
};