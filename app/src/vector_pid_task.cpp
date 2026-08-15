/**
 * @file    vector_pid_task.cpp
 * @brief   巡线双电机速度环任务 (vecpid)
 *
 *          每 10ms 一个控制周期:
 *            编码器测速 -> 巡线外环(转向修正, 含 PD + 陀螺仪阻尼 + yaw 保持)
 *            -> 速度内环 PID -> PWM 输出
 *          另外负责: 起步助推、圈数/转角统计、ques2 停车检测、电池电压前馈。
 *
 * @note    左/右电机、编码器、方向控制引脚见本文件下方对象实例化处，
 *          修改引脚时请同时确认 main.h 与 CubeMX 的引脚配置一致。
 */

#include <cmath>
#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "tim.h"
#include "adc.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "tracking_task.h"
#include "imu_task.h"
#include "vector_pid_task.h"
#include "key_task.h"
#include "debug_print.h"

volatile running_event r_e = running_event::straight;
volatile uint16_t lap_count   = 0;
volatile uint8_t  corner_count = 0;
volatile float    target_laps  = 1.0f;

// 双电机速度环：编码器测速 -> PID -> PWM 输出

volatile float left_base_rpm  = 3500.0f;
volatile float right_base_rpm = 3500.0f;
volatile float left_actual_rpm    = 0.0f;
volatile float right_actual_rpm   = 0.0f;
volatile float left_setpoint_rpm  = 0.0f;
volatile float right_setpoint_rpm = 0.0f;
volatile float left_out_val       = 0.0f;
volatile float right_out_val      = 0.0f;
volatile int32_t left_enc_total   = 0;
volatile int32_t right_enc_total  = 0;

static constexpr const char *NAME  = "vecpid";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 4;
static constexpr float TS = 0.01f;   // 采样周期 10ms

// 电压前馈相关 PA4
static volatile uint32_t adc_raw = 0;   // DMA 循环写入的电压通道原始值
static constexpr float ADC_VREF   = 3.3f;    // ADC 参考电压
static constexpr float ADC_FULL   = 4095.0f; // 12 位满量程
static constexpr float DIV_RATIO  = 11.0f;   // 分压比 (10K+1K)/1K
static constexpr float V_NOMINAL  = 12.0f;   // PID 整定时的标称电压
volatile float  vin_actual = 0.0f;    // 实测输入电压(供 UI/调试)

// 由 ADC 原始值还原真实输入电压(V)
// 分压采样: PA4 -> (10K+1K)/1K 分压 -> ADC1_IN4, DMA 循环搬运到 adc_raw
static float read_vin(void)
{
    return static_cast<float>(adc_raw) / ADC_FULL * ADC_VREF * DIV_RATIO;
}

direction left_dir(GPIOB, GPIO_PIN_0, GPIO_PIN_1);  // in1:PB1 in2:PB0
direction right_dir(GPIOC, GPIO_PIN_8, GPIO_PIN_9); // in1:PC9 in2:PC8

motor left_motor(&htim2, TIM_CHANNEL_1, left_dir);  // PA0
motor right_motor(&htim2, TIM_CHANNEL_2, right_dir);    // PA1

motor_encoder left_enc(&htim3, 13);     // a: PA6   b: PA7
motor_encoder right_enc(&htim4, 13);    // a: PD13   b: PD12

volatile float steer_kp = 0.25f;    // P: err=1时差速占基础转速的比例
volatile float steer_kd = 0.10f;   // D: 阻尼 err 变化率，防振荡
volatile float gz_k     = 0.015f;    // 陀螺仪Z轴角速度阻尼系数
volatile float turn_k   = 0.6f;     // 弯道速度比例 0~1，1=不减速

// 内环
// 注意：限幅在任务里 set_limits() 设置。
// 不能在此用 left_motor.get_period()，静态初始化早于 MX_TIM2_Init，此时 Period 还是 0
// 起始增益按 period=4200(g≈2.45 RPM/count)估算：Ki≈1/(g·20)≈0.02, Kp≈3Ki≈0.05, Kd=0
pid left_motor_pid (0.38f, 0.06f, 0.07f, TS, -100.0f, 100.0f);
pid right_motor_pid(0.38f, 0.06f, 0.07f, TS, -100.0f, 100.0f);
pid yaw_pid(0.68f, 0.0f, 0.0f, TS, -1.0f, 1.0f);

volatile float target_yaw = 0.0f;
volatile float yaw_gain   = 0.0f;    // yaw 先关，巡线+gz 稳了再加

// ===== 电机输出辅助函数 =====

// 把 PID 输出施加到电机：符号决定方向，绝对值作占空比
// 注意: out 范围由 set_limits 限定在 [-period, +period]，占空比以 period 为满量程
static void motor_apply_output(motor &mot, float out)
{
    if (out >= 0.0f)
    {
        mot.dir.set_dir(direction::MOTOR_DIRECTION::forward);
        mot.set_duty(static_cast<uint32_t>(out));
    }
    else
    {
        mot.dir.set_dir(direction::MOTOR_DIRECTION::reversal);
        mot.set_duty(static_cast<uint32_t>(-out));
    }
}

// 紧急/停车刹车：两路 H 桥同接高电平（IN1=IN2=SET 短路制动），同时 PWM 置 0
static void motor_brake(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    left_motor.set_duty(0);
    right_motor.set_duty(0);
}

static uint8_t ques2_pattern[8];

// 启动时刻状态初始化: 记录运行标志、设置起步助推剩余次数、快照起跑线传感器图案
static void init_on_start(bool &was_running, int &kick_remain, bool &ques2_armed)
{
    was_running = true;
    kick_remain = 20;
    ques2_armed = false;
    for (int i = 0; i < 8; i++) ques2_pattern[i] = track.digital_values[i];
    left_motor.dir.set_dir(direction::MOTOR_DIRECTION::forward);
    right_motor.dir.set_dir(direction::MOTOR_DIRECTION::forward);
}

// 停止时: 复位运行标志，PWM 置 0，清零测速缓存
static void reset_on_stop(bool &was_running)
{
    was_running = false;
    left_motor.set_duty(0);
    right_motor.set_duty(0);
    left_actual_rpm  = 0;
    right_actual_rpm = 0;
}

// ques2 任务专用停车检测：
//   同时满足以下任一条件即认为完成一圈并停车:
//     by_enc  = 左/右编码器累计计数值达到设定阈值(±窗口)
//     by_gz   = 转过 >=4 个转角 && 陀螺仪 Z 轴角速度近乎静止(车速已停下)
//   前提: 传感器图案与起跑线快照至少 6/8 路一致
static bool ques2_check(int32_t l_cnt, int32_t r_cnt, bool &armed)
{
    static constexpr int32_t ARM_TH   = 500;
    static constexpr int32_t LEFT_TH  = 48000;
    static constexpr int32_t RIGHT_TH = 38000;
    static constexpr int32_t WINDOW   = 2000;
    static constexpr float   GZ_FLAT  = 50.0f;

    if (!armed && l_cnt > ARM_TH && r_cnt > ARM_TH)
        armed = true;

    if (!armed)
        return false;

    int m = 0;
    for (int i = 0; i < 8; i++)
        if (track.digital_values[i] == ques2_pattern[i]) m++;
    bool pattern_ok = (m >= 6);

    bool by_enc = (l_cnt >= LEFT_TH - WINDOW && r_cnt >= RIGHT_TH - WINDOW);

    bool by_gz  = (corner_count >= 4 && fabsf(imu_gz) < GZ_FLAT);

    if (pattern_ok && (by_enc || by_gz))
    {
        motor_brake();
        g_running = false;
        armed = false;
        return true;
    }
    return false;
}

// 圈数统计: 对陀螺仪 Z 轴角速度积分得到累计偏航角，
// 每累计 ~0.85*2π 判定为一圈并回绕(防止溢出漂移)，同时更新转角计数(每 90° 一个)
static void update_lap(float &lap_yaw_acc)
{
    static constexpr float GZ_SENS = 16.4f;
    static constexpr float GZ_RAD  = (float)M_PI / 180.0f / GZ_SENS;
    static constexpr float TWO_PI  = 2.0f * (float)M_PI;
    static constexpr float LAP_TH  = TWO_PI * 0.85f;

    lap_yaw_acc += imu_gz * GZ_RAD * TS;
    corner_count = static_cast<uint8_t>(fabsf(lap_yaw_acc) / (M_PI / 2.0f));

    if (fabsf(lap_yaw_acc) >= LAP_TH)
    {
        lap_yaw_acc += (lap_yaw_acc > 0.0f) ? -TWO_PI : TWO_PI;
        lap_count++;
        VECPID_DBG("vecpid: lap %d\r\n", lap_count);
    }
}

// ===== 速度环主任务 =====
// 每10ms: 测速 -> 巡线外环(转向修正) -> 速度内环PID -> PWM输出
// 控制链路: eff_base(基础速度×弯道减速) ± track_fix(巡线PD+陀螺仪阻尼+yaw保持)
//           -> 左/右设定转速 -> 速度内环PID(×电压前馈) -> 电机 PWM
static void vector_pid_task_entry(void *pv)
{
    (void)pv;

    left_enc.start();
    right_enc.start();
    left_motor.start();
    right_motor.start();

    float period = static_cast<float>(left_motor.get_period());
    left_motor_pid.set_limits(-period, period);
    right_motor_pid.set_limits(-period, period);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_raw, 1);
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    // ---- 局部状态 ----
    static constexpr int SPD_WIN = 2;
    static constexpr float WIN_S = SPD_WIN * TS;
    int32_t l_hist[SPD_WIN] = {0}, r_hist[SPD_WIN] = {0};
    int  spd_idx  = 0;
    bool spd_ready = false;
    float l_rpm_f = 0, r_rpm_f = 0;
    float prev_err = 0;
    float lap_yaw_acc = 0;
    int  kick_remain = 0;
    bool was_running = false;
    bool ques2_armed = false;
    int  dbg = 0;

    TickType_t last_wake = xTaskGetTickCount();
    static constexpr TickType_t tick = pdMS_TO_TICKS(10);

    while (1)
    {
        vTaskDelayUntil(&last_wake, tick);

        // ---- 启停状态机 ----
        if (g_running && !was_running)
            init_on_start(was_running, kick_remain, ques2_armed);
        if (!g_running && was_running)
            reset_on_stop(was_running);

        if (!g_running)
        {
            left_out_val = right_out_val = 0;
            motor_apply_output(left_motor, 0);
            motor_apply_output(right_motor, 0);
            continue;
        }

        // ---- 起步助推 ----
        if (kick_remain > 0)
        {
            kick_remain--;
            left_motor.set_duty(static_cast<uint32_t>(period / 4));
            right_motor.set_duty(static_cast<uint32_t>(period / 4));
            if (kick_remain == 0)
            {
                left_enc.get_count();  right_enc.get_count();
                for (int i = 0; i < SPD_WIN; i++) l_hist[i] = r_hist[i] = 0;
                spd_idx = 0; spd_ready = false;
                l_rpm_f = r_rpm_f = 0;
                prev_err = 0; lap_yaw_acc = 0;
                target_yaw = imu_yaw;
            }
            continue;
        }

        // ---- 传感器读取 ----
        const float l_cpr = left_enc.lines()  * 4.0f;
        const float r_cpr = right_enc.lines() * 4.0f;
        int32_t l_cnt = left_enc.get_count();
        int32_t r_cnt = right_enc.get_count();
        left_enc_total  = l_cnt;
        right_enc_total = r_cnt;

        // ---- ques2 停车检测 ----
        if (g_mode == task_mode::ques_2 && ques2_check(l_cnt, r_cnt, ques2_armed))
            continue;

        // ---- 速度计算 ----
        int32_t l_old = l_hist[spd_idx], r_old = r_hist[spd_idx];
        l_hist[spd_idx] = l_cnt; r_hist[spd_idx] = r_cnt;
        if (++spd_idx >= SPD_WIN) { spd_idx = 0; spd_ready = true; }

        float l_rpm = spd_ready ? ((l_cnt - l_old) / WIN_S * 60.0f / l_cpr) : 0;
        float r_rpm = spd_ready ? ((r_cnt - r_old) / WIN_S * 60.0f / r_cpr) : 0;
        l_rpm_f += 0.5f * (l_rpm - l_rpm_f);
        r_rpm_f += 0.5f * (r_rpm - r_rpm_f);
        left_actual_rpm  = l_rpm_f;
        right_actual_rpm = r_rpm_f;

        // ---- 电压前馈 ----
        float vin = read_vin();
        vin_actual = vin;
        float ff = V_NOMINAL / vin;
        if (vin < 8.0f || vin > 14.0f) ff = 1.0f;
        if (ff > 1.5f) ff = 1.5f;

        // ---- 圈数/转角计数 ----
        update_lap(lap_yaw_acc);

        // ---- 巡线外环: 转向修正 ----
        float base_avg  = (left_base_rpm + right_base_rpm) * 0.5f;
        float abs_err   = fabsf(err);
        float ratio     = (abs_err > 0.5f) ? fminf((abs_err - 0.5f) / 1.5f, 1.0f) : 0;
        float turn_blend = 1.0f - (1.0f - turn_k) * ratio;
        float eff_base  = base_avg * turn_blend;
        r_e = (abs_err > 0.5f) ? running_event::turning : running_event::straight;

        float err_rate   = (err - prev_err) / TS;
        prev_err = err;
        float track_fix  = (steer_kp * err + steer_kd * err_rate) * base_avg;
        track_fix += gz_k * imu_gz;

        // yaw 修正
        float yaw_err = imu_yaw - target_yaw;
        while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;
        track_fix += yaw_pid.calculate(0.0f, yaw_err) * yaw_gain * base_avg;

        if (track_fix >  base_avg) track_fix =  base_avg;
        if (track_fix < -base_avg) track_fix = -base_avg;

        // ---- 速度内环: PID -> 限幅 -> 输出 ----
        left_setpoint_rpm  = eff_base + track_fix;
        right_setpoint_rpm = eff_base - track_fix;

        float left_out  = left_motor_pid.calculate(left_setpoint_rpm, l_rpm_f) * ff;
        float right_out = right_motor_pid.calculate(right_setpoint_rpm, r_rpm_f) * ff;

        if (left_out  >  period) left_out  =  period;
        if (left_out  < -period) left_out  = -period;
        if (right_out >  period) right_out =  period;
        if (right_out < -period) right_out = -period;

        left_out_val  = left_out;
        right_out_val = right_out;

        motor_apply_output(left_motor, left_out);
        motor_apply_output(right_motor, right_out);

        // ---- 调试打印 (5Hz) ----
        if (++dbg >= 20)
        {
            dbg = 0;
            VECPID_DBG("<l_sp,l_ac,l_out>:%.3f,%.3f,%.3f\r\n",
                       left_setpoint_rpm, l_rpm_f, left_out);
        }
    }
}

void vector_pid_task_create(void)
{
    xTaskCreate(vector_pid_task_entry, NAME, STACK, NULL, PRIO, NULL);
}

void set_left_target_rpm(float rpm)
{
    left_base_rpm = rpm;
}

void set_right_target_rpm(float rpm)
{
    right_base_rpm = rpm;
}

void set_both_target_rpm(float rpm)
{
    left_base_rpm  = rpm;
    right_base_rpm = rpm;
}

// 实际转速：直接返回控制环里缓存的编码器测速值
float get_left_actual_rpm(void)
{
    return left_actual_rpm;
}

float get_right_actual_rpm(void)
{
    return right_actual_rpm;
}
