#include "lcd.h"
#include "main.h"
#include "gpio.h"
#include "stm32f407xx.h"

#define LCD_CMD  (*(volatile uint16_t *)(0x6C000000 | (0 << 13)))
#define LCD_DATA (*(volatile uint16_t *)(0x6C000000 | (1 << 13)))

uint8_t *Lcd::buf1_ = nullptr;
uint8_t *Lcd::buf2_ = nullptr;

Lcd& Lcd::instance() {
    static Lcd inst;
    return inst;
}

void Lcd::write_cmd(uint8_t cmd) {
    LCD_CMD = cmd;
}

void Lcd::write_data8(uint8_t data) {
    LCD_DATA = data;
}

void Lcd::write_data16(uint16_t data) {
    LCD_DATA = data;
}

void Lcd::set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    write_cmd(0x2A);
    write_data8(x1 >> 8); write_data8(x1 & 0xFF);
    write_data8(x2 >> 8); write_data8(x2 & 0xFF);
    write_cmd(0x2B);
    write_data8(y1 >> 8); write_data8(y1 & 0xFF);
    write_data8(y2 >> 8); write_data8(y2 & 0xFF);
    write_cmd(0x2C);
}

void Lcd::ili9341_init_seq() {
    write_cmd(0xCF);
    write_data8(0x00); write_data8(0xC1); write_data8(0x30);

    write_cmd(0xED);
    write_data8(0x64); write_data8(0x03); write_data8(0x12); write_data8(0x81);

    write_cmd(0xE8);
    write_data8(0x85); write_data8(0x10); write_data8(0x7A);

    write_cmd(0xCB);
    write_data8(0x39); write_data8(0x2C); write_data8(0x00);
    write_data8(0x34); write_data8(0x02);

    write_cmd(0xF7);
    write_data8(0x20);

    write_cmd(0xEA);
    write_data8(0x00); write_data8(0x00);

    write_cmd(0xC0);
    write_data8(0x1B);

    write_cmd(0xC1);
    write_data8(0x01);

    write_cmd(0xC5);
    write_data8(0x30); write_data8(0x30);

    write_cmd(0xC7);
    write_data8(0xB7);

    write_cmd(0x36);
    write_data8(0x08);

    write_cmd(0x3A);
    write_data8(0x55);

    write_cmd(0xB1);
    write_data8(0x00); write_data8(0x1A);

    write_cmd(0xB6);
    write_data8(0x0A); write_data8(0xA2);

    write_cmd(0xF2);
    write_data8(0x00);

    write_cmd(0x26);
    write_data8(0x01);

    write_cmd(0xE0);
    write_data8(0x0F); write_data8(0x2A); write_data8(0x28);
    write_data8(0x08); write_data8(0x0E); write_data8(0x08);
    write_data8(0x54); write_data8(0xA9); write_data8(0x43);
    write_data8(0x0A); write_data8(0x0F); write_data8(0x00);
    write_data8(0x00); write_data8(0x00); write_data8(0x00);

    write_cmd(0xE1);
    write_data8(0x00); write_data8(0x15); write_data8(0x17);
    write_data8(0x07); write_data8(0x11); write_data8(0x06);
    write_data8(0x2B); write_data8(0x56); write_data8(0x3C);
    write_data8(0x05); write_data8(0x10); write_data8(0x0F);
    write_data8(0x3F); write_data8(0x3F); write_data8(0x0F);

    write_cmd(0x2B);
    write_data8(0x00); write_data8(0x00);
    write_data8(0x01); write_data8(0x3F);

    write_cmd(0x2A);
    write_data8(0x00); write_data8(0x00);
    write_data8(0x00); write_data8(0xEF);

    write_cmd(0x11);
    HAL_Delay(120);
    write_cmd(0x29);
    HAL_Delay(50);
}

void Lcd::flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    Lcd &self = instance();
    self.set_window(area->x1, area->y1, area->x2, area->y2);

    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    uint16_t *pixels = reinterpret_cast<uint16_t *>(px_map);

    for (uint32_t i = 0; i < w * h; i++) {
        self.write_data16(pixels[i]);
    }

    lv_display_flush_ready(disp);
}

void Lcd::backlight_on() {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
}

void Lcd::backlight_off() {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
}

void Lcd::init() {
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef cfg = {};
    cfg.Pin   = GPIO_PIN_10;
    cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    cfg.Pull  = GPIO_PULLUP;
    cfg.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOF, &cfg);
    backlight_off();

    HAL_Delay(50);

    FSMC_Bank1E->BWTR[6] &= ~(0xF << 0);
    FSMC_Bank1E->BWTR[6] &= ~(0xF << 8);
    FSMC_Bank1E->BWTR[6] |= 3 << 0;
    FSMC_Bank1E->BWTR[6] |= 2 << 8;

    ili9341_init_seq();
    backlight_on();

    const size_t buf_size = WIDTH * 50 * 2;
    buf1_ = static_cast<uint8_t *>(lv_malloc(buf_size));
    buf2_ = static_cast<uint8_t *>(lv_malloc(buf_size));

    display_ = lv_display_create(WIDTH, HEIGHT);
    lv_display_set_flush_cb(display_, flush_cb);
    lv_display_set_buffers(display_, buf1_, buf2_, buf_size,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_color_format(display_, LV_COLOR_FORMAT_RGB565);
}

extern "C" void lcd_init(void) {
    Lcd::instance().init();
}
