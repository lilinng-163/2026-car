#include "xpt2046.h"
#include "main.h"
#include "stm32f407xx.h"

// Touch controller pins
// PEN  = PF11 (input, touch interrupt)
// DOUT = PB2  (input, MISO)
// TDIN = PB1  (output, MOSI)
// TCLK = PA5  (output, CLK)
// TCS  = PB0  (output, CS)

static uint8_t  CMD_RDX = XPT2046_CMD_RDX;
static uint8_t  CMD_RDY = XPT2046_CMD_RDY;

uint16_t Xdown = 0;
uint16_t Ydown = 0;
uint16_t Xup   = 0;
uint16_t Yup   = 0;

float   xFactor = 0.06671114f;
float   yFactor = 0.09117551f;
int16_t xOffset = -11;
int16_t yOffset = -18;

static uint16_t x_val;
static uint16_t y_val;
static uint8_t  press_time;

static void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 28; i++) {
        __ASM volatile ("nop");
    }
}

static void SPI_Write_Byte(uint8_t num) {
    for (uint8_t count = 0; count < 8; count++) {
        if (num & 0x80)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        num <<= 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        delay_us(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
}

static uint16_t SPI_Read_AD(uint8_t CMD) {
    uint16_t Num = 0;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    SPI_Write_Byte(CMD);
    delay_us(6);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    for (uint8_t count = 0; count < 16; count++) {
        Num <<= 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        delay_us(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2))
            Num++;
    }

    Num >>= 4;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    return Num;
}

static uint16_t RTouch_Read_XorY(uint8_t xy) {
    uint16_t buf[5];
    uint16_t sum, temp;

    for (uint8_t i = 0; i < 5; i++)
        buf[i] = SPI_Read_AD(xy);

    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = i + 1; j < 5; j++) {
            if (buf[i] > buf[j]) {
                temp   = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }

    sum = 0;
    for (uint8_t i = 1; i < 4; i++)
        sum += buf[i];

    return sum / 3;
}

static uint8_t RTouch_Read_XY(uint16_t *x, uint16_t *y) {
    uint16_t xtemp = RTouch_Read_XorY(CMD_RDX);
    uint16_t ytemp = RTouch_Read_XorY(CMD_RDY);

    if (xtemp < 50 || ytemp < 50)
        return 0;

    *x = xtemp;
    *y = ytemp;
    return 1;
}

static uint8_t RTouch_Read_XY2(uint16_t *x, uint16_t *y) {
    uint16_t x1, y1, x2, y2;
    uint8_t flag;

    flag = RTouch_Read_XY(&x1, &y1);
    if (flag == 0) return 0;

    flag = RTouch_Read_XY(&x2, &y2);
    if (flag == 0) return 0;

    if (((x2 <= x1 && x1 < x2 + 50) || (x1 <= x2 && x2 < x1 + 50))
        && ((y2 <= y1 && y1 < y2 + 50) || (y1 <= y2 && y2 < y1 + 50)))
    {
        *x = (x1 + x2) / 2;
        *y = (y1 + y2) / 2;
        return 1;
    }
    return 0;
}

void XPT2046_Scan(uint8_t tp) {
    Xup = 0xFFFF;
    Yup = 0xFFFF;

    if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11) == GPIO_PIN_RESET) {
        if (tp)
            RTouch_Read_XY2(&x_val, &y_val);
        else if (RTouch_Read_XY2(&x_val, &y_val)) {
            x_val = (uint16_t)(xFactor * x_val + xOffset);
            y_val = (uint16_t)(yFactor * y_val + yOffset);
        }
        Xdown = x_val;
        Ydown = y_val;
        press_time++;
    } else {
        if (press_time > 2) {
            Xup = x_val;
            Yup = y_val;
        }
        press_time = 0;
        Xdown = 0xFFFF;
        Ydown = 0xFFFF;
    }
}

void XPT2046_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // PEN  = PF11 input pull-up
    GPIO_InitTypeDef cfg = {};
    cfg.Pin  = GPIO_PIN_11;
    cfg.Mode = GPIO_MODE_INPUT;
    cfg.Pull = GPIO_PULLUP;
    cfg.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOF, &cfg);

    // DOUT = PB2 input pull-up
    cfg.Pin  = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &cfg);

    // TCS  = PB0, TDIN = PB1 output push-pull
    cfg.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    cfg.Mode = GPIO_MODE_OUTPUT_PP;
    cfg.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &cfg);

    // TCLK = PA5 output push-pull
    cfg.Pin  = GPIO_PIN_5;
    cfg.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &cfg);

    // Set CS high (inactive)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}
