#include "mutex.h"
#include "debug_print.h"
#include <cstdio>

SemaphoreHandle_t lvgl_mutex = NULL;

int lvgl_mutex_init(void)   // 控件创建和lvgl定时回调之间的互斥锁
{
    lvgl_mutex = xSemaphoreCreateMutex();
    if (lvgl_mutex == NULL) {
        MUTEX_DBG("lvgl_mutex create failed\n");
        return -1;
    }
    MUTEX_DBG("lvgl_mutex ok\n");
    return 0;
}
