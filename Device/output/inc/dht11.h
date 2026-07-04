#pragma once
#include <cstdint>

class Dht11 {
public:
    static Dht11& instance();

    bool read();

    uint8_t get_humidity() const { return humidity_; }
    uint8_t get_temperature() const { return temperature_; }

private:
    Dht11() = default;

    static void delay_us(uint32_t us);
    static void set_pin_output();
    static void set_pin_input();
    static uint8_t read_byte();

    uint8_t humidity_{0};
    uint8_t temperature_{0};
};
