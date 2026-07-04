#include "beep.h"
#include "main.h"
#include "gpio.h"
#include "stm32f407xx.h"

#define BEEP_PIN   GPIO_PIN_7
#define BEEP_PORT  GPIOG

Beep& Beep::instance() {
    static Beep inst;
    return inst;
}

void Beep::on() {
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_SET);
}

void Beep::off() {
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET);
}

void Beep::toggle() {
    HAL_GPIO_TogglePin(BEEP_PORT, BEEP_PIN);
}

