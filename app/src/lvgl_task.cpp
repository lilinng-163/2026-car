#include "lvgl_task.h"
#include "mutex.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl.h"
#include <cstdio>

static constexpr const char *NAME  = "lvgl";
static constexpr configSTACK_DEPTH_TYPE STACK = 2048;
static constexpr UBaseType_t PRIO = 1;

static void lvgl_task(void *pv) {
    (void)pv;
    printf("lvgl start\n");
    while (1) {
        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
        lv_tick_inc(5);
        lv_timer_handler();
        xSemaphoreGive(lvgl_mutex);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void lvgl_task_create() {
    xTaskCreate(lvgl_task, NAME, STACK, NULL, PRIO, NULL);
}
