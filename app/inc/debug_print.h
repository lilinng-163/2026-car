#pragma once

#include <cstdio>

// ── 各任务调试打印总开关：1 开启 / 0 关闭 ──
#define DEBUG_IMU_TASK      0
#define DEBUG_KEY_TASK      1
#define DEBUG_LED_TASK      0
#define DEBUG_LVGL_TASK     0
#define DEBUG_MUTEX         0
#define DEBUG_OLED_TASK     0
#define DEBUG_SHOW_PV       0
#define DEBUG_TRACKING_TASK 1
#define DEBUG_VECPID_TASK   0

//__VA_ARGS__: C/C++ 变参宏的占位符，代表宏调用时传入的所有参数（... 部分）

#if DEBUG_IMU_TASK
#define IMU_DBG(...)        printf(__VA_ARGS__)
#else
#define IMU_DBG(...)        ((void)0)
#endif

#if DEBUG_KEY_TASK
#define KEY_DBG(...)        printf(__VA_ARGS__)
#else
#define KEY_DBG(...)        ((void)0)
#endif

#if DEBUG_LED_TASK
#define LED_DBG(...)        printf(__VA_ARGS__)
#else
#define LED_DBG(...)        ((void)0)
#endif

#if DEBUG_LVGL_TASK
#define LVGL_DBG(...)       printf(__VA_ARGS__)
#else
#define LVGL_DBG(...)       ((void)0)
#endif

#if DEBUG_MUTEX
#define MUTEX_DBG(...)      printf(__VA_ARGS__)
#else
#define MUTEX_DBG(...)      ((void)0)
#endif

#if DEBUG_OLED_TASK
#define OLED_DBG(...)       printf(__VA_ARGS__)
#else
#define OLED_DBG(...)       ((void)0)
#endif

#if DEBUG_SHOW_PV
#define SHOW_PV_DBG(...)    printf(__VA_ARGS__)
#else
#define SHOW_PV_DBG(...)    ((void)0)
#endif

#if DEBUG_TRACKING_TASK
#define TRACKING_DBG(...)   printf(__VA_ARGS__)
#else
#define TRACKING_DBG(...)   ((void)0)
#endif

#if DEBUG_VECPID_TASK
#define VECPID_DBG(...)     printf(__VA_ARGS__)
#else
#define VECPID_DBG(...)     ((void)0)
#endif
