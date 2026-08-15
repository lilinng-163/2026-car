/**
 * @file    key.cpp
 * @brief   按键驱动实现
 *
 *          状态机(每 10ms 由任务调用 key_tick 一次):
 *            idle -> debounce(按下) -> pressed -> click/release
 *            pressed 按住超过 long_pressed_ms -> long_pressed(长按, 持续返回)
 *          按键低电平有效(外部上拉)。
 */

#include <cstdio>
#include <cstdint>
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "key.h"

key::key(GPIO_TypeDef *_gpio_port, uint16_t _gpio_num)
: gpio_port(_gpio_port), gpio_num(_gpio_num)
{

}

// 按键状态机步进，返回本次发生的事件(click/long_press 等)
key_event key::key_tick(void)
{
    uint32_t now_tick = HAL_GetTick();
    switch(s)
    {
        case (key_status::idle):
        {
            if(HAL_GPIO_ReadPin(gpio_port, gpio_num) == GPIO_PIN_RESET)
            {
                s = key_status::debounce;
                m_enter_tick = now_tick;
            }
            return key_event::none;
            break;
        }
        case(key_status::debounce):
        {
            if(HAL_GPIO_ReadPin(gpio_port, gpio_num) == GPIO_PIN_RESET)
            {
                if((now_tick - m_enter_tick) > debounce_ms)
                {
                    s = key_status::pressed;
                }
            }
            else
            {
                s = key_status::idle;
            }
            return key_event::none;
            break;
        }
        case(key_status::pressed):
        {
            if(HAL_GPIO_ReadPin(gpio_port, gpio_num) == GPIO_PIN_SET)
            {
                s = key_status::release;
                return key_event::click;
            }
            else
            {
                if((now_tick - m_enter_tick) > long_pressed_ms)
                {
                    s = key_status::long_pressed;   
                    return key_event::long_press;
                }
            }
            break;
        }
        case(key_status::long_pressed):
        {
            if(HAL_GPIO_ReadPin(gpio_port, gpio_num) == GPIO_PIN_SET)
            {
                s = key_status::release;
                // return key_event::long_press;
            }
            else
            {
                return key_event::long_press;
            }
            break;
        }
        case(key_status::release):
        {
            if(HAL_GPIO_ReadPin(gpio_port, gpio_num) == GPIO_PIN_SET)
            {
                s = key_status::idle;
                return key_event::none;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return key_event::none;
}