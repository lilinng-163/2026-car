#pragma once

#include <cstdint>
#include "etl/vector.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

// 亚博8路循迹(YB-MUX04-2.0) 串口解析
// 数字: $D,x1:1,x2:1,...,x8:0#
// 模拟: $A,x1:3794,x2:4025,...,x8:3845#
class tracking
{
public:
    int feed_byte(uint8_t byte);

    uint8_t digital_values[8] = {0};
    uint16_t analog_values[8] = {0};
    bool digital = true;
    enum class tracking_event : uint8_t
    {
        MIDDLE,
        LEFT,
        RIGHT,
        LOST,
    };
    tracking_event e;
private:
    void parse_frame(void);
    void sync_to_header(void);

    static constexpr uint8_t HEAD = '$';
    static constexpr uint8_t TAIL = '#';
    static constexpr size_t BUF_SIZE = 128;

    etl::vector<uint8_t, BUF_SIZE> rx_buf;
};
