#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "main.h"
#include "oled.h"
#include "vector_pid_task.h"
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

        // Line 0: setpoint (SP)
        snprintf(buf, sizeof(buf), "SP L%-4d R%-4d",
                 static_cast<int>(left_setpoint_rpm),
                 static_cast<int>(right_setpoint_rpm));
        o.show_string(buf, 0, 0);

        // Line 1: actual (AC)
        snprintf(buf, sizeof(buf), "AC L%-4d R%-4d",
                 static_cast<int>(left_actual_rpm),
                 static_cast<int>(right_actual_rpm));
        o.show_string(buf, 0, 16);

        // Line 2: output (OT)
        snprintf(buf, sizeof(buf), "OT L%-4d R%-4d",
                 static_cast<int>(left_out_val),
                 static_cast<int>(right_out_val));
        o.show_string(buf, 0, 32);

        // Line 3: voltage + track_pid Kp
        float tkp = track_pid.get_instance().Kp;
        snprintf(buf, sizeof(buf), "V%d.%dV %d.%02d",
                 static_cast<int>(vin_actual),
                 static_cast<int>(vin_actual * 10.0f) % 10,
                 static_cast<int>(tkp),
                 static_cast<int>(tkp * 100.0f) % 100);
        o.show_string(buf, 0, 48);

        o.refresh();
    }
}

void oled_task_create() {
    xTaskCreate(oled_task, NAME, STACK, NULL, PRIO, NULL);
}
