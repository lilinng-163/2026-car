#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

// 编码器抽象基类
class encoder
{
public:
    virtual ~encoder() = default;
    virtual int32_t get_count() = 0;   // 累计计数(带符号)
    virtual void reset() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
};

// 电机编码器: 基于定时器编码器模式(TI12), 硬件 4 倍频
class motor_encoder : public encoder
{
public:
    // _lines: 单相线数(计数×4 为整圈计数), _invert: 反向安装时取反
    motor_encoder(TIM_HandleTypeDef *_htim, int32_t _lines, bool _invert = false);

    void start(void) override;
    void stop(void) override;
    int32_t get_count(void) override;
    void reset(void) override;
    int32_t lines(void) const { return enc_lines; }

    float get_speed(void);   // 增量测速 (计数/ms)
    float get_rpm(void);     // 转速 (转/分)
    float get_cm_s(void);    // 线速度 (cm/s, 需先 set_wheel_circumference_mm)

    void set_wheel_circumference_mm(float _circumference_mm);

private:
    TIM_HandleTypeDef *htim;
    int32_t enc_lines;
    bool m_invert = false;
    float m_circumference_mm = 0.0f;
    uint16_t m_last_raw = 0;
    int32_t m_total_count = 0;
    int32_t m_last_count = 0;
    uint32_t m_last_tick = 0;
};

// 旋转编码器: GPIO 中断方式, 由外部 EXTI 回调调用 on_interrupt()
class rotary_encoder : public encoder
{
public:
    rotary_encoder(GPIO_TypeDef *_port_a, uint16_t _pin_a,
                   GPIO_TypeDef *_port_b, uint16_t _pin_b);

    void start(void) override;
    void stop(void) override;
    int32_t get_count(void) override;
    void reset(void) override;

    void on_interrupt(void);   // 在 A/B 相 EXTI 回调中调用

private:
    GPIO_TypeDef *port_a, *port_b;
    uint16_t pin_a, pin_b;
    volatile int32_t m_count;
    volatile uint8_t m_last_a;
    bool m_enabled;
};
