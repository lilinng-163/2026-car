#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

class encoder
{
public:
    virtual ~encoder() = default;
    virtual int32_t get_count() = 0;
    virtual void reset() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
};

class motor_encoder : public encoder
{
public:
    motor_encoder(TIM_HandleTypeDef *_htim, int32_t _lines, bool _invert = false);

    void start(void) override;
    void stop(void) override;
    int32_t get_count(void) override;
    void reset(void) override;
    int32_t lines(void) const { return enc_lines; }

    float get_speed(void);
    float get_rpm(void);
    float get_cm_s(void);

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

class rotary_encoder : public encoder
{
public:
    rotary_encoder(GPIO_TypeDef *_port_a, uint16_t _pin_a,
                   GPIO_TypeDef *_port_b, uint16_t _pin_b);

    void start(void) override;
    void stop(void) override;
    int32_t get_count(void) override;
    void reset(void) override;

    void on_interrupt(void);

private:
    GPIO_TypeDef *port_a, *port_b;
    uint16_t pin_a, pin_b;
    volatile int32_t m_count;
    volatile uint8_t m_last_a;
    bool m_enabled;
};
