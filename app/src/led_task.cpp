#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "main.h"
#include "led.h"
#include "led_task.h"

static constexpr const char *NAME  = "led";
static constexpr configSTACK_DEPTH_TYPE STACK = 128;
static constexpr UBaseType_t PRIO = 3;

static void led_task(void *pv) {
    (void)pv;
    printf("led start\n");
    led l0(LED0_GPIO_Port, LED0_Pin);
    while (1) {
        // printf("led toggle\n");
        l0.toggle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void led_task_create() {
    xTaskCreate(led_task, NAME, STACK, NULL, PRIO, NULL);
}
