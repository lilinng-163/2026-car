#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "main.h"
#include "oled.h"
#include "oled_task.h"

static constexpr const char *NAME  = "oled";
static constexpr configSTACK_DEPTH_TYPE STACK = 512;
static constexpr UBaseType_t PRIO = 2;

static void oled_task(void *pv) {
    (void)pv;
    printf("oled start\n");
    static oled096 o(GPIOB, GPIO_PIN_3, GPIO_PIN_4);

    int counter = 0;
    while (1) {
        // --- 测试 1: 字符串显示 (每行 8x16) ---
        o.clear();
        o.show_string("Hello World", 0, 0);
        o.show_string("0123456789", 0, 16);
        o.show_string("ABCabc!@#$%", 0, 32);
        o.show_string("STM32F407", 0, 48);
        o.refresh();
        vTaskDelay(pdMS_TO_TICKS(2000));

        // --- 测试 2: 数字显示 (自增/负数) ---
        o.clear();
        o.show_string("num:", 0, 0);
        o.show_num(counter, 40, 0);
        o.show_string("neg:", 0, 16);
        o.show_num(-counter, 40, 16);
        o.refresh();
        vTaskDelay(pdMS_TO_TICKS(2000));

        // --- 测试 3: 像素 —— 画边框 + 对角线 ---
        o.clear();
        for (uint16_t x = 0; x < 128; x++) {
            o.set_pixel(x, 0);
            o.set_pixel(x, 63);
        }
        for (uint16_t y = 0; y < 64; y++) {
            o.set_pixel(0, y);
            o.set_pixel(127, y);
        }
        for (uint16_t i = 0; i < 64; i++) {
            o.set_pixel(i * 2, i);
        }
        o.refresh();
        vTaskDelay(pdMS_TO_TICKS(2000));

        // --- 测试 4: clear_pixel —— 在实心块上挖洞 ---
        o.clear();
        for (uint16_t x = 20; x < 108; x++)
            for (uint16_t y = 16; y < 48; y++)
                o.set_pixel(x, y);
        for (uint16_t x = 40; x < 88; x++)
            for (uint16_t y = 24; y < 40; y++)
                o.clear_pixel(x, y);
        o.refresh();
        vTaskDelay(pdMS_TO_TICKS(2000));

        // --- 测试 5: 清屏 ---
        o.clear();
        o.show_string("Clear...", 0, 24);
        o.refresh();
        vTaskDelay(pdMS_TO_TICKS(1000));

        counter += 123;
    }
}

void oled_task_create() {
    xTaskCreate(oled_task, NAME, STACK, NULL, PRIO, NULL);
}
