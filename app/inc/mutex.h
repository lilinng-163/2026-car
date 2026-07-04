#pragma once

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

extern SemaphoreHandle_t lvgl_mutex;

int lvgl_mutex_init(void);
