#pragma once

#include "arm_math.h"

class pid
{
public:
    pid(float32_t _kp, float32_t _ki, float32_t _kd,
        float32_t _out_min = 0.0f, float32_t _out_max = 1.0f);

    void reset(void);
    void set_gains(float32_t _kp, float32_t _ki, float32_t _kd);
    void set_limits(float32_t _min, float32_t _max);
    float32_t calculate(float32_t setpoint, float32_t measurement);

private:
    arm_pid_instance_f32 instance;
    float32_t out_min;
    float32_t out_max;
};
