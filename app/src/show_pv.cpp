#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lvgl.h"
#include "mutex.h"
#include "show_pv.h"
#include "beep.h"
#include "lv_obj.h"
#include "debug_print.h"

static constexpr const char *NAME  = "show_pv";
static constexpr configSTACK_DEPTH_TYPE STACK = 512;
static constexpr UBaseType_t PRIO = 2;

static void show_pv_task(void *pv)
{
    (void)pv;
    SHOW_PV_DBG("show_pv start\n");

    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);

    lv_obj_t *screen = lv_screen_active();

    create_pages(screen);

    xSemaphoreGive(lvgl_mutex);

    SHOW_PV_DBG("show_pv done\n");
    while (1)
    {
        // 切换页面 + 刷新 PID 参数显示（读共享 PID 对象，key_task 写、这里读）
        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
        ui_page_apply();
        if (!ui_editing())
        {
            pid_update_ui();
        }
        xSemaphoreGive(lvgl_mutex);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void show_pv_create()
{
    xTaskCreate(show_pv_task, NAME, STACK, NULL, PRIO, NULL);
}
