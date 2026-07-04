#include "dht11.h"
#include "main.h"
#include "stm32f407xx.h"

#define DHT11_PORT  GPIOD
#define DHT11_PIN   GPIO_PIN_3

static void dwt_init() {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t dwt_tick() {
    return DWT->CYCCNT;
}

void Dht11::delay_us(uint32_t us) {
    static bool init = false;
    if (!init) {
        dwt_init();
        init = true;
    }
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t start = dwt_tick();
    while ((dwt_tick() - start) < ticks) {
        __NOP();
    }
}

void Dht11::set_pin_output() {
    GPIO_InitTypeDef cfg = {};
    cfg.Pin   = DHT11_PIN;
    cfg.Mode  = GPIO_MODE_OUTPUT_OD;
    cfg.Pull  = GPIO_PULLUP;
    cfg.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_PORT, &cfg);
}

void Dht11::set_pin_input() {
    GPIO_InitTypeDef cfg = {};
    cfg.Pin   = DHT11_PIN;
    cfg.Mode  = GPIO_MODE_INPUT;
    cfg.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &cfg);
}

Dht11& Dht11::instance() {
    static Dht11 inst;
    return inst;
}

uint8_t Dht11::read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;

        uint32_t timeout = 0;
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) {
            if (++timeout > 500) return 0;
        }

        delay_us(40);

        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
            byte |= 1;
        }

        timeout = 0;
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
            if (++timeout > 500) break;
        }
    }
    return byte;
}

bool Dht11::read() {
    uint8_t data[5] = {0};
    uint32_t timeout;

    set_pin_output();

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    delay_us(20000);

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);

    set_pin_input();

    __disable_irq();

    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        if (++timeout > 500) {
            __enable_irq();
            return false;
        }
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) {
        if (++timeout > 500) {
            __enable_irq();
            return false;
        }
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        if (++timeout > 500) {
            __enable_irq();
            return false;
        }
    }

    for (int i = 0; i < 5; i++) {
        data[i] = read_byte();
    }

    __enable_irq();

    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4]) {
        return false;
    }

    humidity_    = data[0];
    temperature_ = data[2];
    return true;
}
