/**
 * @file    encoder.cpp
 * @brief   编码器驱动实现
 *
 *          motor_encoder : 电机编码器, 利用定时器编码器模式(TI12)硬件 4 倍频计数,
 *                          无 CPU 开销; get_count() 返回带符号累计值(注意 16 位
 *                          计数器回绕处理), get_rpm()/get_cm_s() 换算转速。
 *          rotary_encoder: GPIO 中断旋转编码器, 由外部 EXTI 回调调用 on_interrupt()。
 */

#include "encoder.h"

motor_encoder::motor_encoder(TIM_HandleTypeDef *_htim, int32_t _lines, bool _invert)
    : htim(_htim), enc_lines(_lines), m_invert(_invert)
{
}

void motor_encoder::start(void)
{
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void motor_encoder::stop(void)
{
    HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
}

// 读取编码器累计计数:
// 16 位计数器按符号扩展求增量(m_invert 可反相)，累加到 32 位 m_total_count，
// 从而避免计数上限限制并正确处理回绕。
int32_t motor_encoder::get_count(void)
{
    uint16_t raw = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(htim));
    int16_t delta = static_cast<int16_t>(raw - m_last_raw);
    m_last_raw = raw;
    m_total_count += m_invert ? -delta : delta;
    return m_total_count;
}

void motor_encoder::reset(void)
{
    __HAL_TIM_SET_COUNTER(htim, 0);
    m_last_raw = 0;
    m_total_count = 0;
    m_last_count = 0;
    m_last_tick = HAL_GetTick();
}

// 增量测速: 两次调用间计数差 / 时间差(计数/ms)
float motor_encoder::get_speed(void)
{
    uint32_t now = HAL_GetTick();
    int32_t cnt = get_count();
    uint32_t dt = now - m_last_tick;
    if (dt == 0) return 0.0f;

    float speed = static_cast<float>(cnt - m_last_count) / dt;
    m_last_count = cnt;
    m_last_tick = now;
    return speed;
}

float motor_encoder::get_rpm(void)  // 转/分
{
    if (enc_lines == 0) return 0.0f;
    // 1 转 = 线数×4(倍频) 个计数; get_speed() 单位计数/ms -> rpm
    return get_speed() * 60000.0f / static_cast<float>(enc_lines * 4);
}

void motor_encoder::set_wheel_circumference_mm(float _circumference_mm)
{
    m_circumference_mm = _circumference_mm;
}

// 线速度(cm/s): 计数/ms * 25(减速比?) * 轮周长 / 线数 —— 需配合轮周长设置使用
float motor_encoder::get_cm_s(void)
{
    if (enc_lines == 0 || m_circumference_mm == 0.0f)
    {
        return 0.0f;
    }
    float raw = get_speed();
    return raw * 25.0f * m_circumference_mm / static_cast<float>(enc_lines);
}

rotary_encoder::rotary_encoder(GPIO_TypeDef *_port_a, uint16_t _pin_a,
                               GPIO_TypeDef *_port_b, uint16_t _pin_b)
    : port_a(_port_a), pin_a(_pin_a),
      port_b(_port_b), pin_b(_pin_b),
      m_count(0), m_last_a(0), m_enabled(false)
{
}

void rotary_encoder::start(void)
{
    m_count = 0;
    m_last_a = static_cast<uint8_t>(HAL_GPIO_ReadPin(port_a, pin_a));
    m_enabled = true;
}

void rotary_encoder::stop(void)
{
    m_enabled = false;
}

int32_t rotary_encoder::get_count(void)
{
    return m_count;
}

void rotary_encoder::reset(void)
{
    m_count = 0;
}

// A 相跳变沿触发: A 与 B 电平相同为正向(+1)，相反为反向(-1)
void rotary_encoder::on_interrupt(void)
{
    if (!m_enabled)
    {
        return;
    }

    uint8_t a = static_cast<uint8_t>(HAL_GPIO_ReadPin(port_a, pin_a));
    uint8_t b = static_cast<uint8_t>(HAL_GPIO_ReadPin(port_b, pin_b));

    if (a == m_last_a)
    {
        return;
    }
    m_last_a = a;

    if (a == b)
    {
        m_count++;
    }
    else
    {
        m_count--;
    }   
}
