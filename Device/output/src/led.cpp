#include <string_view>
#include <cstdio>
#include "stm32f4xx_hal.h"
#include "led.h"

led::led(GPIO_TypeDef *_gpio_port, uint16_t _gpio_num)
: gpio_port(_gpio_port), gpio_num(_gpio_num)
{
    HAL_GPIO_WritePin(gpio_port, gpio_num, GPIO_PIN_SET);
    is_on = false;
}
int led::on(void)
{
    HAL_GPIO_WritePin(gpio_port, gpio_num, GPIO_PIN_RESET);
    is_on = true;
    return 0;
}
int led::off(void)
{
    HAL_GPIO_WritePin(gpio_port, gpio_num, GPIO_PIN_SET);
    is_on = false;
    return 0;
}
int led::toggle(void)
{
    if(is_on)
    {
        HAL_GPIO_WritePin(gpio_port, gpio_num, GPIO_PIN_SET);
        is_on = false;
    }
    else
    {
        HAL_GPIO_WritePin(gpio_port, gpio_num, GPIO_PIN_RESET);
        is_on = true;
    }
    return 0;
}
bool led::get_status(void)
{
    return is_on;
}