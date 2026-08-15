#pragma once

#include <string_view>
#include <cstdint>
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

// 按键消抖/长按状态机状态
enum class key_status : uint8_t
{
    idle,          // 空闲(松开)
    debounce,      // 消抖中
    pressed,       // 已确认按下
    long_pressed,  // 长按触发后保持
    release        // 松开确认
};

// 按键事件
enum class key_event : uint8_t
{
    none,
    click,         // 短按单击
    long_press,    // 长按
    repeat,        // (预留)重复
    doubleclick    // (预留)双击
};

// 按键驱动: 低电平有效，需周期性调用 key_tick()
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
    static constexpr uint32_t debounce_ms = 30;        // 消抖时长(ms)
    static constexpr uint32_t long_pressed_ms = 800;   // 判定长按的时长(ms)
    static constexpr uint32_t repeat_ms = 100;         // (预留)重复间隔
    static constexpr uint32_t double_click_window_ms = 400;  // (预留)双击窗口
};