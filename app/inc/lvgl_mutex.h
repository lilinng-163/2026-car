#pragma once

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SemaphoreHandle_t lvgl_mutex;

int lvgl_mutex_init();

#ifdef __cplusplus
}
#endif
