#include <cstdio>
#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "key.h"
#include "vector_pid_task.h"
#include "key_task.h"
#include "debug_print.h"

static constexpr const char *NAME  = "key";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 3;

volatile task_mode g_mode = task_mode::idle;
volatile bool g_running = false;
volatile uint32_t stopwatch_elapsed_ms = 0;
static uint32_t stopwatch_start_tick = 0;

[[maybe_unused]] static const char *mode_name(task_mode m)
{
    switch (m) {
        case task_mode::idle:        return "IDLE";
        case task_mode::line_patrol: return "PATROL";
        case task_mode::ques_2:      return "QUES2";
        default: return "?";
    }
}

static void key_task(void *pv)
{
    (void)pv;
    key k0(GPIOF, GPIO_PIN_9);
    key k1(GPIOF, GPIO_PIN_8);
    key k2(GPIOF, GPIO_PIN_7);
    KEY_DBG("key_task start\r\n");

    while (1)
    {
        // K0: 切换模式 (仅在非运行状态)
        if (!g_running && k0.key_tick() == key_event::click)
        {
            g_mode = static_cast<task_mode>(
                (static_cast<uint8_t>(g_mode) + 1) % static_cast<uint8_t>(task_mode::mode_count));
            KEY_DBG("mode -> %s\r\n", mode_name(g_mode));
        }

        // K1: 启动当前模式
        if (!g_running && g_mode != task_mode::idle && k1.key_tick() == key_event::click)
        {
            g_running = true;
            lap_count = 0;
            stopwatch_start_tick = HAL_GetTick();
            stopwatch_elapsed_ms = 0;
            KEY_DBG("START %s\r\n", mode_name(g_mode));
        }

        // K2: 停止
        if (g_running && k2.key_tick() == key_event::click)
        {
            g_running = false;
            KEY_DBG("STOP %lu ms\r\n", stopwatch_elapsed_ms);
        }

        if (g_running)
        {
            stopwatch_elapsed_ms = HAL_GetTick() - stopwatch_start_tick;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void key_task_create()
{
    xTaskCreate(key_task, NAME, STACK, NULL, PRIO, NULL);
}
