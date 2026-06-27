#ifndef __XPT2046_H__
#define __XPT2046_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Touch controller pins
// PEN  = PF11 (input, touch interrupt)
// DOUT = PB2  (input, MISO)
// TDIN = PB1  (output, MOSI)
// TCLK = PA5  (output, CLK)
// TCS  = PB0  (output, CS)

#define XPT2046_CMD_RDX 0xD0
#define XPT2046_CMD_RDY 0x90

extern uint16_t Xdown;
extern uint16_t Ydown;
extern uint16_t Xup;
extern uint16_t Yup;

extern float xFactor;
extern float yFactor;
extern int16_t xOffset;
extern int16_t yOffset;

void XPT2046_Init(void);
void XPT2046_Scan(uint8_t tp);

#ifdef __cplusplus
}
#endif

#endif
