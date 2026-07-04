#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl.h"
#include "mutex.h"
#include "show_pv.h"
#include "beep.h"
#include "lv_obj.h"

static constexpr const char *NAME  = "show_pv";
static constexpr configSTACK_DEPTH_TYPE STACK = 512;
static constexpr UBaseType_t PRIO = 2;

static void show_pv_task(void *pv) {
    (void)pv;
    printf("show_pv start\n");

    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);

    lv_obj_t *screen = lv_screen_active();

    create_pages(screen);

    xSemaphoreGive(lvgl_mutex);

    printf("show_pv done\n");
    while (1) {
        if (dht11_request) {
            dht11_request = false;
            xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
            dht11_update_ui();
            xSemaphoreGive(lvgl_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void show_pv_create() {
    xTaskCreate(show_pv_task, NAME, STACK, NULL, PRIO, NULL);
}
