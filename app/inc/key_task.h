#pragma once
#include <cstdint>

void key_task_create();

enum class task_mode : uint8_t {
    idle = 0,
    line_patrol,
    ques_2,
    mode_count
};

extern volatile task_mode g_mode;
extern volatile bool g_running;
extern volatile uint32_t stopwatch_elapsed_ms;
