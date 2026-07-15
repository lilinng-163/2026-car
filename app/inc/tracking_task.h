#pragma once

#include "tracking.h"

extern tracking track;
constexpr float middle = 3.5f;
extern volatile float pos;
extern volatile float err;

void tracking_task_create(void);
