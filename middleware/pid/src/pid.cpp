#include "pid.h"
#include <algorithm>

pid::pid(float32_t _kp, float32_t _ki, float32_t _kd,
         float32_t _ts, float32_t _out_min, float32_t _out_max)
    : kp(_kp), ki_cont(_ki / _ts), kd_cont(_kd * _ts),
      ts(_ts), out_min(_out_min), out_max(_out_max)
{
    reset();
}

void pid::reset(void)
{
    instance.state[0] = 0.0f;
    instance.state[1] = 0.0f;
    instance.state[2] = 0.0f;
    sync_instance();
}

void pid::set_gains(float32_t _kp, float32_t _ki, float32_t _kd)
{
    // _ki, _kd 是当前 Ts 下的离散值，反算连续增益
    kp = _kp;
    ki_cont = _ki / ts;
    kd_cont = _kd * ts;
    sync_instance();
}

void pid::set_ts(float32_t _ts)
{
    ts = _ts;
    sync_instance();
}

void pid::set_limits(float32_t _min, float32_t _max)
{
    out_min = _min;
    out_max = _max;
}

void pid::sync_instance(void)
{
    ki = ki_cont * ts;
    kd = kd_cont / ts;
    instance.Kp = kp;
    instance.Ki = ki;
    instance.Kd = kd;
}

float32_t pid::calculate(float32_t setpoint, float32_t measurement)
{
    float32_t error = setpoint - measurement;

    float32_t p_term = kp * (error - instance.state[0]);
    float32_t d_term = kd * (error - 2.0f * instance.state[0] + instance.state[1]);

    float32_t out = instance.state[2] + p_term + d_term;

    if ((out >= out_max && error > 0.0f) || (out <= out_min && error < 0.0f))
    {
        instance.state[1] = instance.state[0];
        instance.state[0] = error;
        instance.state[2] = (out > out_max) ? out_max : out_min;
        return instance.state[2];
    }

    out += ki * error;

    if (out > out_max)
        out = out_max;
    else if (out < out_min)
        out = out_min;

    instance.state[1] = instance.state[0];
    instance.state[0] = error;
    instance.state[2] = out;

    return out;
}

arm_pid_instance_f32 pid::get_instance(void)
{
    return this->instance;
}