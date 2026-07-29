#pragma once
#include <cstdint>

void servo_task_create(void);

extern volatile float ball_pos;       // 摄像头反馈的钢球位置(cm), 0=中心
extern volatile float ball_setpoint;  // 期望位置(cm)
extern volatile float servo_angle;    // 当前舵机角度(°), 供调试
