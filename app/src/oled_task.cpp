#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "main.h"
#include "oled.h"
#include "key_task.h"
#include "oled_task.h"
#include "debug_print.h"

static constexpr const char *NAME  = "oled";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 3;

static void oled_task(void *pv) {
    (void)pv;
    OLED_DBG("oled start\r\n");
    static oled096 o(GPIOB, GPIO_PIN_3, GPIO_PIN_4);

    char buf[32];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));

        o.clear();

        uint32_t ms = stopwatch_elapsed_ms;
            uint32_t seconds = ms / 1000;
            uint32_t minutes = seconds / 60;
            uint32_t sec = seconds % 60;
            uint32_t cs = (ms % 1000) / 10;

            const char *mode_str = "?";
            if (g_mode == task_mode::idle)        mode_str = "IDLE";
            else if (g_mode == task_mode::line_patrol) mode_str = "PATROL";

            snprintf(buf, sizeof(buf), "%c%s",
                     g_running ? '>' : ' ', mode_str);
            o.show_string(buf, 0, 0);

            snprintf(buf, sizeof(buf), "  %02lu:%02lu.%02lu",
                     minutes, sec, cs);
            o.show_string(buf, 0, 24);

            if (g_running)
                snprintf(buf, sizeof(buf), "K2:STOP");
            else
                snprintf(buf, sizeof(buf), "K0:MODE K1:GO");
            o.show_string(buf, 0, 48);

        o.refresh();
    }
}

void oled_task_create() {
    xTaskCreate(oled_task, NAME, STACK, NULL, PRIO, NULL);
}
