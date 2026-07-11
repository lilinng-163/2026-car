#pragma once

#include "arm_math.h"

class pid
{
public:
    pid(float32_t _kp, float32_t _ki, float32_t _kd,
        float32_t _ts,
        float32_t _out_min = 0.0f, float32_t _out_max = 1.0f);

    void reset(void);
    void set_gains(float32_t _kp, float32_t _ki, float32_t _kd);
    void set_ts(float32_t _ts);
    void set_limits(float32_t _min, float32_t _max);
    float32_t calculate(float32_t setpoint, float32_t measurement);
    // 返回内部 CMSIS-DSP 实例副本，供 UI 只读显示 Kp/Ki/Kd
    arm_pid_instance_f32 get_instance(void);
private:
    void sync_instance(void);

    float32_t kp;           // 比例增益（连续/离散相同）
    float32_t ki_cont;      // 连续积分增益 = Kp'/Ti
    float32_t kd_cont;      // 连续微分增益 = Kp'·Td
    float32_t ts;           // 采样周期 (s)
    float32_t ki;           // 预计算 ki_cont * ts
    float32_t kd;           // 预计算 kd_cont / ts

    arm_pid_instance_f32 instance;   // 只复用 state[] 和 UI 显示
    float32_t out_min;
    float32_t out_max;
};
