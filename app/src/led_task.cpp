#include "led_task.h"
#include "main.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include <cstdio>

static constexpr const char *NAME  = "led";
static constexpr configSTACK_DEPTH_TYPE STACK = 128;
static constexpr UBaseType_t PRIO = 3;

static void led_task(void *pv) {
    (void)pv;
    printf("led start\n");
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    while (1) {
        // printf("led toggle\n");
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

extern "C" void led_task_create() {
    xTaskCreate(led_task, NAME, STACK, NULL, PRIO, NULL);
}
