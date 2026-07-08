#pragma once

#include <string_view>
#include <cstdint>
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

enum class key_status : uint8_t
{
    idle,
    debounce,
    pressed,
    long_pressed,
    release
};

enum class key_event : uint8_t
{
    none,
    click,
    long_press,
    repeat,
    doubleclick
};

class key
{
public:
    key(GPIO_TypeDef *_gpio_port, uint16_t _gpio_num);
    key_event key_tick(void);
private:
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_num;
    key_status s = key_status::idle;
    uint32_t m_enter_tick = 0;
    static constexpr uint32_t debounce_ms = 30;
    static constexpr uint32_t long_pressed_ms = 800;
    static constexpr uint32_t repeat_ms = 100;
    static constexpr uint32_t double_click_window_ms = 400;
};