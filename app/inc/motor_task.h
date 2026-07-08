#pragma once

#include "pid.h"

extern pid left_motor_pid;
extern pid right_motor_pid;

int motor_task(void *pv);