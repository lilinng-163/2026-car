#include "show_pv.h"
#include "lvgl_mutex.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl.h"
#include <cstdio>

static constexpr const char *NAME  = "show_pv";
static constexpr configSTACK_DEPTH_TYPE STACK = 512;
static constexpr UBaseType_t PRIO = 2;

static void show_pv_task(void *pv) {
    (void)pv;
    printf("show_pv start\n");

    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);

    lv_obj_t *screen = lv_screen_active();

    lv_obj_t *sw1 = lv_switch_create(screen);
    lv_obj_align(sw1, LV_ALIGN_CENTER, -50, -20);
    lv_obj_add_event_cb(sw1, [](lv_event_t *e) {
        lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
        printf("sw1: %d\n", lv_obj_has_state(sw, LV_STATE_CHECKED));
    }, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *sw2 = lv_switch_create(screen);
    lv_obj_align(sw2, LV_ALIGN_CENTER, 50, -20);
    lv_obj_add_event_cb(sw2, [](lv_event_t *e) {
        lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
        printf("sw2: %d\n", lv_obj_has_state(sw, LV_STATE_CHECKED));
    }, LV_EVENT_VALUE_CHANGED, NULL);

    xSemaphoreGive(lvgl_mutex);

    printf("show_pv done\n");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void show_pv_create() {
    xTaskCreate(show_pv_task, NAME, STACK, NULL, PRIO, NULL);
}
