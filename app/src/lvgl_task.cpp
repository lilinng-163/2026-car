#include "lvgl_task.h"
#include "mutex.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl.h"
#include "debug_print.h"
#include <cstdio>

static constexpr const char *NAME  = "lvgl";
static constexpr configSTACK_DEPTH_TYPE STACK = 2048;
static constexpr UBaseType_t PRIO = 1;

static void lvgl_task(void *pv) {
    (void)pv;
    LVGL_DBG("lvgl start\n");
    TickType_t last = xTaskGetTickCount();
    while (1)
    {
        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
        TickType_t now = xTaskGetTickCount();
        lv_tick_inc((now - last) * portTICK_PERIOD_MS);
        last = now;
        lv_timer_handler();
        xSemaphoreGive(lvgl_mutex);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void lvgl_task_create() {
    xTaskCreate(lvgl_task, NAME, STACK, NULL, PRIO, NULL);
}
