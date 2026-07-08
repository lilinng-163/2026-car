#pragma once

#include <string>
#include <cstdint>
#include "stm32f407xx.h"
#include "soft_i2c.h"

class oled096
{
public:
    oled096(GPIO_TypeDef *_gpiox, uint16_t _scl, uint16_t _sda);
    int set_pixel(uint16_t x, uint16_t y);
    int clear_pixel(uint16_t x, uint16_t y);
    int clear(void);
    int refresh(void);
    int show_string(std::string str, uint16_t x, uint16_t y);
    int show_num(int num, uint16_t x, uint16_t y);

private:
    soft_i2c i2c;
    unsigned char buffer[1024];

    int write_cmd(unsigned char cmd);
    int write_data(unsigned char data);
    int setcursor(unsigned char x, unsigned char y);
};
