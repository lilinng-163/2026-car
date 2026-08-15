#pragma once
#include <cstdint>

void key_task_create();

// 运行模式: idle 待机 / line_patrol 巡线 / ques_2 第二问
enum class task_mode : uint8_t {
    idle = 0,
    line_patrol,
    ques_2,
    mode_count
};

extern volatile task_mode g_mode;        // 当前模式(K0 切换, K1 启动)
extern volatile bool g_running;          // 运行标志(K1 置位, K2 清除)
extern volatile uint32_t stopwatch_elapsed_ms;  // 本次运行已用时长(ms, 供 OLED 显示)
