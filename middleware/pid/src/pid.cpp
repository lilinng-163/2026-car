#include "pid.h"
#include <algorithm>

pid::pid(float32_t _kp, float32_t _ki, float32_t _kd,
         float32_t _out_min, float32_t _out_max)
    : out_min(_out_min), out_max(_out_max)
{
    instance.Kp = _kp;
    instance.Ki = _ki;
    instance.Kd = _kd;
    arm_pid_init_f32(&instance, 1);
}

void pid::reset(void)
{
    arm_pid_reset_f32(&instance);
}

void pid::set_gains(float32_t _kp, float32_t _ki, float32_t _kd)
{
    instance.Kp = _kp;
    instance.Ki = _ki;
    instance.Kd = _kd;
    arm_pid_init_f32(&instance, 0);
}

void pid::set_limits(float32_t _min, float32_t _max)
{
    out_min = _min;
    out_max = _max;
}

float32_t pid::calculate(float32_t setpoint, float32_t measurement)
{
    float32_t error = setpoint - measurement;

    float32_t p_term = instance.Kp * (error - instance.state[0]);
    float32_t d_term = instance.Kd * (error - 2.0f * instance.state[0] + instance.state[1]);

    float32_t out = instance.state[2] + p_term + d_term;

    if ((out >= out_max && error > 0.0f) || (out <= out_min && error < 0.0f))
    {
        instance.state[1] = instance.state[0];
        instance.state[0] = error;
        instance.state[2] = (out > out_max) ? out_max : out_min;
        return instance.state[2];
    }

    out += instance.Ki * error;

    if (out > out_max)
    {
        out = out_max;
    }
        
    else if (out < out_min)
    {
        out = out_min;
    }
        
    instance.state[1] = instance.state[0];
    instance.state[0] = error;
    instance.state[2] = out;

    return out;
}
arm_pid_instance_f32 pid::get_instance(void)
{
    return this->instance;
}