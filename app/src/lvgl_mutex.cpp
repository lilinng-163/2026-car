#include "lvgl_mutex.h"
#include <cstdio>

extern "C" {
SemaphoreHandle_t lvgl_mutex = NULL;
}

extern "C" int lvgl_mutex_init() {
    lvgl_mutex = xSemaphoreCreateMutex();
    if (lvgl_mutex == NULL) {
        printf("lvgl_mutex create failed\n");
        return -1;
    }
    printf("lvgl_mutex ok\n");
    return 0;
}
