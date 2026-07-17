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
#include "debug_print.h"

volatile running_event r_e = running_event::straight;
volatile uint16_t lap_count   = 0;
volatile uint8_t  corner_count = 0;
volatile float    target_laps  = 1.0f;

// 双电机速度环：编码器测速 -> PID -> PWM 输出

volatile float left_base_rpm  = 5000.0f;
volatile float right_base_rpm = 5000.0f;
volatile float left_actual_rpm    = 0.0f;
volatile float right_actual_rpm   = 0.0f;
volatile float left_setpoint_rpm  = 0.0f;
volatile float right_setpoint_rpm = 0.0f;
volatile float left_out_val       = 0.0f;
volatile float right_out_val      = 0.0f;

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
static float read_vin(void)
{
    return static_cast<float>(adc_raw) / ADC_FULL * ADC_VREF * DIV_RATIO;
}

direction left_dir(GPIOB, GPIO_PIN_0, GPIO_PIN_1);  // in1:PB1  in2:PB0
direction right_dir(GPIOC, GPIO_PIN_8, GPIO_PIN_9); // in1:PC9      in2:PC8

motor left_motor(&htim2, TIM_CHANNEL_1, left_dir);  // PA0
motor right_motor(&htim2, TIM_CHANNEL_2, right_dir);    // PA1

motor_encoder left_enc(&htim3, 13);     // a: PA6   b: PA7
motor_encoder right_enc(&htim4, 13, true);    // a: PD12   b: PD13

volatile float steer_kp = 0.45f;    // P: err=1时差速占基础转速的比例
volatile float steer_kd = 0.051f;   // D: 阻尼 err 变化率，防振荡
volatile float gz_k     = 0.006f;    // 陀螺仪Z轴角速度阻尼系数
volatile float turn_k   = 0.6f;     // 弯道速度比例 0~1，1=不减速

// 内环
// 注意：限幅在任务里 set_limits() 设置。
// 不能在此用 left_motor.get_period()，静态初始化早于 MX_TIM2_Init，此时 Period 还是 0
// 起始增益按 period=4200(g≈2.45 RPM/count)估算：Ki≈1/(g·20)≈0.02, Kp≈3Ki≈0.05, Kd=0
pid left_motor_pid (0.8f, 0.05f, 0.0f, TS, -100.0f, 100.0f);
pid right_motor_pid(0.8f, 0.05f, 0.0f, TS, -100.0f, 100.0f);
pid yaw_pid(0.68f, 0.0f, 0.0f, TS, -1.0f, 1.0f);

volatile float target_yaw = 0.0f;
volatile float yaw_gain   = 0.0f;    // yaw 先关，巡线+gz 稳了再加

// 把 PID 输出施加到电机：符号决定方向，绝对值作占空比
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

// 速度环任务：每 10ms 测速 -> PID 计算(外环循迹误差输出rpm，内环输入测速rpm+偏差rpm输出占空比) -> 输出
static void vector_pid_task_entry(void *pv)
{
    (void)pv;
    VECPID_DBG("vecpid: enter\r\n");

    left_enc.start();
    right_enc.start();
    VECPID_DBG("vecpid: enc started\r\n");
    left_motor.start();
    right_motor.start();
    VECPID_DBG("vecpid: motor started\r\n");

    // 此时 MX_TIM2_Init 已跑过，Period 有效，按 PWM 满量程设 PID 限幅
    float period = static_cast<float>(left_motor.get_period());
    left_motor_pid.set_limits(-period, period);
    right_motor_pid.set_limits(-period, period);
    VECPID_DBG("vecpid: pid limit +-%d\r\n", (int)period);

    left_motor.dir.set_dir(direction::MOTOR_DIRECTION::forward);
    right_motor.dir.set_dir(direction::MOTOR_DIRECTION::forward);

    // 启动 ADC DMA 循环采集，之后 adc_raw 自动保持最新值
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_raw, 1);
    // 单元素循环+连续转换会让 DMA 完成中断疯狂触发(中断风暴导致卡死)，
    // 循环搬运不需要中断通知，直接读内存即可，故关掉 TC/HT/TE 中断
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
    VECPID_DBG("vecpid: loop begin\r\n");

    // 起步：固定正向 50% 占空比 500ms，等轮子转起来产生编码器反馈
    left_motor.set_duty(static_cast<uint32_t>(period / 2));
    right_motor.set_duty(static_cast<uint32_t>(period / 2));
    vTaskDelay(pdMS_TO_TICKS(500));
    VECPID_DBG("vecpid: kick done\r\n");

    // 编码器计数清零，后续测速以此为基准
    left_enc.get_count();
    right_enc.get_count();

    // 上电锁死当前航向作为直行目标
    target_yaw = imu_yaw;
    VECPID_DBG("vecpid: yaw locked at %d.%02d\r\n",
               (int)target_yaw, (int)(target_yaw * 100.0f) % 100);

    TickType_t last_wake = xTaskGetTickCount();
    static constexpr TickType_t ticks = pdMS_TO_TICKS(10);

    while (1)
    {
        vTaskDelayUntil(&last_wake, ticks);

        // ── 长窗口测速：用 50ms 内累计计数差算转速 ──
        // 量化档 = 60000/(线数×4×窗口ms)，10ms→115RPM，50ms→~23RPM，噪声大幅下降
        static constexpr int   SPD_WIN = 2;              // 2 × 10ms = 20ms 窗口(减滞后)
        static constexpr float WIN_S   = SPD_WIN * TS;   // 0.02s
        const float l_cpr = left_enc.lines()  * 4.0f;    // 每圈计数(四倍频)
        const float r_cpr = right_enc.lines() * 4.0f;

        int32_t l_cnt = left_enc.get_count();   // 累计计数(内部已处理 16 位回绕)
        int32_t r_cnt = right_enc.get_count();

        static int32_t l_hist[SPD_WIN] = {0}, r_hist[SPD_WIN] = {0};
        static int  spd_idx  = 0;
        static bool spd_ready = false;

        int32_t l_old = l_hist[spd_idx];        // SPD_WIN 拍之前的计数
        int32_t r_old = r_hist[spd_idx];
        l_hist[spd_idx] = l_cnt;
        r_hist[spd_idx] = r_cnt;
        if (++spd_idx >= SPD_WIN) { spd_idx = 0; spd_ready = true; }

        // 窗口未填满前速度先按 0，避免开机瞬间的假尖峰
        float l_rpm = spd_ready ?  ((l_cnt - l_old) / WIN_S * 60.0f / l_cpr) : 0.0f;
        float r_rpm = spd_ready ?  ((r_cnt - r_old) / WIN_S * 60.0f / r_cpr) : 0.0f;

        // 轻 EMA 收尾，磨平长窗口后的残余量化跳变
        static float l_rpm_f = 0.0f, r_rpm_f = 0.0f;
        constexpr float ALPHA = 0.5f;    // 加大以减小滞后
        l_rpm_f += ALPHA * (l_rpm - l_rpm_f);
        r_rpm_f += ALPHA * (r_rpm - r_rpm_f);

        left_actual_rpm  = l_rpm_f;   // 缓存供 UI 读取
        right_actual_rpm = r_rpm_f; 
        // 12v -> 10k -> 1k ->gnd
        // 电压前馈系数：实测电压越低，占空比补得越大，保持有效电压一致
        float vin = read_vin();
        vin_actual = vin;
        // vin 读数不在合理电池范围就不补偿：读高了 ff<1 会把占空比
        // 整体打折(如误读 25V -> ff=0.47，5000 只能跑 2000+)，读低了会顶满
        float ff = V_NOMINAL / vin;
        if (vin < 8.0f || vin > 14.0f) ff = 1.0f;
        if (ff > 1.5f) ff = 1.5f;

        // ── 计圈：原始陀螺仪 gz 积分，满 2π 即一圈 ──
        // 直接用 gz 原始值积分，不依赖 IMU 欧拉角（欧拉角内部融合会补偿/重置导致净转角凑不够 2π）
        // GZ_SENS: 陀螺仪灵敏度 LSB/(°/s)，±2000°/s 量程 = 16.4，±500°/s = 65.5
        static constexpr float GZ_SENS = 16.4f;
        static constexpr float GZ_RAD  = (float)M_PI / 180.0f / GZ_SENS;
        static float lap_yaw_acc = 0.0f;
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

        // ── 连续弯道衰减：|err| 越大速度越低，无跳变 ──
        // 最大减速到turn_k = 0.6
        // 1.0 - turn_k表示随着err的增大从0增大到0.4，1.0 - (1.0 - turn_k) * 归一化就是最后的比例
        float base_avg = (left_base_rpm + right_base_rpm) * 0.5f;
        float abs_err = fabsf(err);
        float ratio = (abs_err > 0.5f) ? fminf((abs_err - 0.5f) / 1.5f, 1.0f) : 0.0f;
        float turn_blend = 1.0f - (1.0f - turn_k) * ratio;
        float eff_base = base_avg * turn_blend;
        r_e = (abs_err > 0.5f) ? running_event::turning : running_event::straight;

        // 完成目标圈数后停车
        if (lap_count >= static_cast<uint16_t>(target_laps))
        {
            eff_base = 0.0f;
        }

        // ── 巡线转向 ──
        static float prev_err = 0.0f;
        float err_rate = (err - prev_err) / TS;
        prev_err = err;
        float track_fix = (steer_kp * err + steer_kd * err_rate) * base_avg;

        // ── gz 阻尼 ──
        track_fix += gz_k * imu_gz;

        // ── yaw 航向修正（全时段） ──
        {
            float yaw_err = imu_yaw - target_yaw;
            while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;
            float yaw_out = yaw_pid.calculate(0.0f, yaw_err);
            track_fix += yaw_out * yaw_gain * base_avg;
        }

        // ── 限幅 ──
        if (track_fix > base_avg)  track_fix = base_avg;
        if (track_fix < -base_avg) track_fix = -base_avg;

        left_setpoint_rpm = eff_base + track_fix;
        right_setpoint_rpm = eff_base - track_fix;

        float left_out  = left_motor_pid.calculate(left_setpoint_rpm, l_rpm_f) * ff;
        float right_out = right_motor_pid.calculate(right_setpoint_rpm, r_rpm_f) * ff;

        left_out_val  = left_out;
        right_out_val = right_out;

        // 前馈放大后可能超限，夹回 [-period, period]
        if (left_out  >  period) left_out  =  period;
        if (left_out  < -period) left_out  = -period;
        if (right_out >  period) right_out =  period;
        if (right_out < -period) right_out = -period;

        motor_apply_output(left_motor, left_out);
        motor_apply_output(right_motor, right_out);

        static int dbg = 0;
        if (++dbg >= 20) {
            dbg = 0;
            VECPID_DBG("L cnt=%ld raw=%u rpm=%ld sp=%ld out=%ld ccr=%u d=%d%d\r\n",
                   (long)l_cnt, (unsigned)__HAL_TIM_GET_COUNTER(&htim3),
                   (long)l_rpm_f, (long)left_setpoint_rpm, (long)left_out,
                   (unsigned)TIM2->CCR1,
                   HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0), HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1));
            VECPID_DBG("R cnt=%ld raw=%u rpm=%ld sp=%ld out=%ld ccr=%u d=%d%d vin=%ldmv\r\n",
                   (long)r_cnt, (unsigned)__HAL_TIM_GET_COUNTER(&htim4),
                   (long)r_rpm_f, (long)right_setpoint_rpm, (long)right_out,
                   (unsigned)TIM2->CCR2,
                   HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8), HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9),
                   (long)(vin * 1000));
            VECPID_DBG("lap=%d corner=%d acc=%d.%02d evt=%d\r\n",
                   lap_count, corner_count,
                   (int)lap_yaw_acc, (int)(fabsf(lap_yaw_acc) * 100.0f) % 100,
                   (int)r_e);
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
