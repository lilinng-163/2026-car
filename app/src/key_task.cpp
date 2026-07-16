#include <cstdio>
#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "key.h"
#include "vector_pid_task.h"
#include "tune_ui.h"
#include "key_task.h"
#include "debug_print.h"

static constexpr const char *NAME  = "key";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 3;

// 步进与上下限
static constexpr float SP_STEP = 100.0f;
static constexpr float SP_MIN  = 0.0f;
static constexpr float SP_MAX  = 8000.0f;

static constexpr float KP_STEP = 0.05f;
static constexpr float KI_STEP = 0.01f;
static constexpr float KD_STEP = 0.01f;
static constexpr float GAIN_MIN = 0.0f;
static constexpr float GAIN_MAX = 9.99f;   // 显示格式 x.xx，最大两位整数前限制

// 循迹 PID：Kp 量级大(误差±3.5 -> 转速修正)，单独步进/限幅
static constexpr float TKP_STEP = 50.0f;
static constexpr float TKP_MAX  = 5000.0f;
static constexpr float TKI_STEP = 0.05f;
static constexpr float TKD_STEP = 0.05f;
static constexpr float TGAIN_MAX = 99.99f;

volatile uint8_t tune_page = 0;
volatile uint8_t tune_select = 0;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 页0：调 SP
static void adjust_sp(float dir)
{
    if (tune_select == TUNE_SP_L)
    {
        set_left_target_rpm(clampf(left_base_rpm + dir * SP_STEP, SP_MIN, SP_MAX));
        KEY_DBG("SP L -> %d\r\n", (int)left_base_rpm);
    }
    else
    {
        set_right_target_rpm(clampf(right_base_rpm + dir * SP_STEP, SP_MIN, SP_MAX));
        KEY_DBG("SP R -> %d\r\n", (int)right_base_rpm);
    }
}

// 页1：调 PID 增益，which: 0=Kp 1=Ki 2=Kd
static void adjust_gain(pid &p, uint8_t which, float dir)
{
    static constexpr float steps[3] = {KP_STEP, KI_STEP, KD_STEP};

    arm_pid_instance_f32 g = p.get_instance();
    float v[3] = {g.Kp, g.Ki, g.Kd};
    v[which] = clampf(v[which] + dir * steps[which], GAIN_MIN, GAIN_MAX);
    p.set_gains(v[0], v[1], v[2]);
    KEY_DBG("gain[%d] -> %d.%02d\r\n", which,
            (int)v[which], (int)(v[which] * 100.0f) % 100);
}

// 页2：调循迹 PID，which: 0=Kp 1=Ki 2=Kd
static void adjust_track_gain(uint8_t which, float dir)
{
    static constexpr float steps[3] = {TKP_STEP, TKI_STEP, TKD_STEP};
    static constexpr float maxs[3]  = {TKP_MAX, TGAIN_MAX, TGAIN_MAX};

    arm_pid_instance_f32 g = track_pid.get_instance();
    float v[3] = {g.Kp, g.Ki, g.Kd};
    v[which] = clampf(v[which] + dir * steps[which], GAIN_MIN, maxs[which]);
    track_pid.set_gains(v[0], v[1], v[2]);
    KEY_DBG("trk gain[%d] -> %d.%02d\r\n", which,
            (int)v[which], (int)(v[which] * 100.0f) % 100);
}

static void adjust_selected(float dir)
{
    if (tune_page == 0)
    {
        adjust_sp(dir);
    }
    else if (tune_page == 1)
    {
        uint8_t sel = tune_select;
        pid &p = (sel >= TUNE_R_KP) ? right_motor_pid : left_motor_pid;
        adjust_gain(p, sel % 3, dir);
    }
    else
    {
        adjust_track_gain(tune_select, dir);
    }
}

static uint8_t params_of_page(uint8_t page)
{
    switch (page)
    {
    case 0:  return TUNE_P0_PARAMS;
    case 1:  return TUNE_P1_PARAMS;
    default: return TUNE_P2_PARAMS;
    }
}

static void key_task(void *pv) 
{
    (void)pv;
    key k0(GPIOF, GPIO_PIN_9);
    key k1(GPIOF, GPIO_PIN_8);
    key k2(GPIOF, GPIO_PIN_7);
    key k3(GPIOF, GPIO_PIN_6);
    KEY_DBG("key_task start\r\n");

    while (1) 
    {
        // k0：选中参数 +step
        if (k0.key_tick() == key_event::click)
        {
            adjust_selected(+1.0f);
        }

        // k2：选中参数 -step
        if (k2.key_tick() == key_event::click)
        {
            adjust_selected(-1.0f);
        }

        // k1：切换选中的参数
        if (k1.key_tick() == key_event::click)
        {
            tune_select = (tune_select + 1) % params_of_page(tune_page);
            KEY_DBG("select -> %d\r\n", tune_select);
        }

        // k3：切换页面（重置选择）
        if (k3.key_tick() == key_event::click)
        {
            tune_page = (tune_page + 1) % TUNE_NUM_PAGES;
            tune_select = 0;
            KEY_DBG("page -> %d\r\n", tune_page);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void key_task_create() 
{
    xTaskCreate(key_task, NAME, STACK, NULL, PRIO, NULL);
}
