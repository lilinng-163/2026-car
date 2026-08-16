#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

// 步进电机驱动: TIM PWM 输出 STEP 脉冲 + GPIO 控制 DIR/EN
// 每个 PWM 周期对应 1 个步进脉冲, PWM 频率 = 步进速度(Hz)。
// 通过定时器更新中断累加已发步数, 供位置控制使用。
class stepper
{
public:
    // _en_gpio/_en_pin 可传 nullptr/0 表示无 EN 引脚
    stepper(TIM_HandleTypeDef *_htim, uint32_t _channel,
            GPIO_TypeDef *_dir_gpio, uint16_t _dir_pin,
            GPIO_TypeDef *_en_gpio = nullptr, uint16_t _en_pin = 0);
    void start(void);              // 清零步数 + 启动 PWM(并置 EN 使能)
    void stop(void);               // 停止 PWM(并置 EN 失能)
    void enable(void);             // EN 拉低使能(驱动板低有效)
    void disable(void);            // EN 拉高失能
    void set_dir(bool forward);    // true=正转, false=反转
    void set_speed(uint32_t hz);   // 步进频率(Hz), 0 表示停止
    uint32_t get_speed(void) const;
    void reset_step_count(void);   // 清零累计步数
    uint32_t get_step_count(void) const;  // 累计步数(更新中断累加)

    static void on_tim_update(void);      // 由 HAL_TIM_PeriodElapsedCallback 调用
private:
    uint32_t timer_clk(void);             // 定时器实际时钟(Hz)
    void apply_config(uint32_t psc, uint32_t period);

    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *dir_gpio;
    uint16_t dir_pin;
    GPIO_TypeDef *en_gpio;
    uint16_t en_pin;
    uint32_t speed_hz;
    volatile uint32_t step_count;
    bool running;

    static stepper *active;       // 当前激活实例, 供 ISR 计数
};
