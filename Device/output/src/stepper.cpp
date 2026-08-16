/**
 * @file    stepper.cpp
 * @brief   步进电机驱动实现 (STEP/DIR/EN)
 *
 *          STEP 引脚由定时器 PWM 输出, 每个 PWM 周期产生 1 个步进脉冲，
 *          脉冲频率 = 步进速度。定时器更新中断每周期触发一次, 用于累计步数。
 *
 *          引脚: STEP = 定时器 PWM 通道, DIR/EN = 普通 GPIO。
 */

#include <cstdint>
#include "stepper.h"
#include "gpio.h"

stepper *stepper::active = nullptr;

stepper::stepper(TIM_HandleTypeDef *_htim, uint32_t _channel,
                 GPIO_TypeDef *_dir_gpio, uint16_t _dir_pin,
                 GPIO_TypeDef *_en_gpio, uint16_t _en_pin)
    : htim(_htim), channel(_channel),
      dir_gpio(_dir_gpio), dir_pin(_dir_pin),
      en_gpio(_en_gpio), en_pin(_en_pin),
      speed_hz(0), step_count(0), running(false)
{
    active = this;

    // DIR/EN 引脚初始化为推挽输出
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    if (dir_gpio != nullptr)
    {
        gpio.Pin = dir_pin;
        HAL_GPIO_Init(dir_gpio, &gpio);
        HAL_GPIO_WritePin(dir_gpio, dir_pin, GPIO_PIN_RESET);
    }
    if (en_gpio != nullptr)
    {
        gpio.Pin = en_pin;
        HAL_GPIO_Init(en_gpio, &gpio);
        HAL_GPIO_WritePin(en_gpio, en_pin, GPIO_PIN_SET);  // EN 低有效(常见驱动器)
    }
}

// 定时器实际时钟: TIM8 挂在 APB2, 预分频非 1 时定时器时钟 x2
uint32_t stepper::timer_clk(void)
{
    uint32_t clk = HAL_RCC_GetPCLK2Freq();
    if (clk < HAL_RCC_GetHCLKFreq())
        clk *= 2;
    return clk;
}

// 配置定时器分频与周期, 50% 占空比输出 STEP 脉冲
void stepper::apply_config(uint32_t psc, uint32_t period)
{
    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, period);
    __HAL_TIM_SET_COMPARE(htim, channel, period / 2);
    htim->Instance->EGR = TIM_EGR_UG;  // 生成更新事件, 立即装载 PSC/ARR
}

// 启动: 清零步数, 启动 PWM 并开启更新中断计数
void stepper::start(void)
{
    enable();
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_PWM_Start(htim, channel);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
    running = true;
    step_count = 0;
}

// 停止: 停 PWM 并关闭更新中断
void stepper::stop(void)
{
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    HAL_TIM_PWM_Stop(htim, channel);
    running = false;
    disable();
}

void stepper::enable(void)
{
    if (en_gpio != nullptr)
        HAL_GPIO_WritePin(en_gpio, en_pin, GPIO_PIN_RESET);  // EN 低有效
}

void stepper::disable(void)
{
    if (en_gpio != nullptr)
        HAL_GPIO_WritePin(en_gpio, en_pin, GPIO_PIN_SET);
}

// 设置方向
void stepper::set_dir(bool forward)
{
    if (dir_gpio != nullptr)
        HAL_GPIO_WritePin(dir_gpio, dir_pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 设置步进频率(Hz): 依据频率自适应选取分频使周期落在 16 位范围内
void stepper::set_speed(uint32_t hz)
{
    if (hz == 0)
    {
        stop();
        speed_hz = 0;
        return;
    }

    const uint32_t clk = timer_clk();
    uint32_t psc = 0;
    uint64_t cnt = (uint64_t)clk / hz;              // 每个脉冲的计数周期
    while (cnt > 65536ULL && psc < 65535U)
    {
        ++psc;
        cnt = (uint64_t)clk / ((uint64_t)(psc + 1) * hz);
    }
    if (cnt > 65536ULL)
        cnt = 65536ULL;

    const bool was_running = running;
    if (was_running)
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);  // 配置期间避免误计数

    apply_config(psc, (uint32_t)(cnt - 1));
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);    // 清除配置产生的更新标志

    if (was_running)
        __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);

    speed_hz = hz;
}

uint32_t stepper::get_speed(void) const
{
    return speed_hz;
}

void stepper::reset_step_count(void)
{
    step_count = 0;
}

uint32_t stepper::get_step_count(void) const
{
    return step_count;
}

// 定时器更新中断回调: 每发出一个 STEP 脉冲计数一次
void stepper::on_tim_update(void)
{
    if (active != nullptr && active->running)
        active->step_count = active->step_count + 1U;
}
