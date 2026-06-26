#ifndef __ILI9341_H
#define __ILI9341_H

#include <stdint.h>

#define ILI9341_WIDTH  240
#define ILI9341_HEIGHT 320

void ILI9341_Init(void);
void ili9341_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint8_t *data);

#endif
