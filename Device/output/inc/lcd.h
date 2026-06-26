#pragma once

#ifdef __cplusplus
#include <cstdint>

extern "C" {
#endif

#include "lvgl.h"

#ifdef __cplusplus
}
#endif

// ============================================================
// C++ 接口
// ============================================================
#ifdef __cplusplus
class Lcd {
public:
    static Lcd& instance();

    void init();
    void backlight_on();
    void backlight_off();

private:
    Lcd() = default;

    void write_cmd(uint8_t cmd);
    void write_data8(uint8_t data);
    void write_data16(uint16_t data);
    void set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void ili9341_init_seq();

    static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);

    static constexpr uint16_t WIDTH  = 240;
    static constexpr uint16_t HEIGHT = 320;

    lv_display_t *display_ = nullptr;

    static uint8_t *buf1_;
    static uint8_t *buf2_;
};
#endif // __cplusplus

// ============================================================
// C 兼容接口
// ============================================================
#ifdef __cplusplus
extern "C" {
#endif

void lcd_init(void);

#ifdef __cplusplus
}
#endif
