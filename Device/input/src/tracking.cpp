/**
 * @file    tracking.cpp
 * @brief   亚博 8 路循迹(YB-MUX04)串口协议解析实现
 *
 *          数字帧: $D,x1:1,x2:1,...,x8:0#
 *          模拟帧: $A,x1:3794,x2:4025,...,x8:3845#
 *          feed_byte 以 '#' 为帧尾触发解析，非法时滑动到 '$' 重新同步。
 */

#include "tracking.h"

// 逐字节喂入; 遇到帧尾 '#' 时解析一帧并清缓冲
// 返回 0=正常(或组帧中), -1=帧头非法已清缓冲
int tracking::feed_byte(uint8_t byte)
{
    if (rx_buf.full())
        rx_buf.clear();

    rx_buf.push_back(byte);

    if (byte != TAIL)
        return 0;

    if (rx_buf.empty() || rx_buf.at(0) != HEAD)
    {
        sync_to_header();
        if (rx_buf.empty() || rx_buf.at(0) != HEAD)
        {
            rx_buf.clear();
            return -1;
        }
    }

    parse_frame();
    rx_buf.clear();
    return 0;
}

// 滑动缓冲到下一个 '$' 帧头位置
void tracking::sync_to_header(void)
{
    for (size_t i = 0; i < rx_buf.size(); i++)
    {
        if (rx_buf.at(i) == HEAD)
        {
            etl::vector<uint8_t, BUF_SIZE> tmp;
            for (size_t j = i; j < rx_buf.size(); j++)
            {
                if (!tmp.full())
                    tmp.push_back(rx_buf.at(j));
            }
            rx_buf = tmp;
            return;
        }
    }
    rx_buf.clear();
}

// 解析整帧: 逐通道提取 "chN:" 后的值(数字=0/1, 模拟=十进制数)存入对应数组
void tracking::parse_frame(void)
{
    if (rx_buf.size() < 4)
        return;

    if (rx_buf.at(0) != HEAD)
        return;

    char mode = static_cast<char>(rx_buf.at(1));
    if (mode != 'A' && mode != 'D')
        return;

    if (rx_buf.at(2) != ',')
        return;

    size_t pos = 3;

    if (mode == 'D')
    {
        for (int ch = 0; ch < 8; ch++)
        {
            while (pos < rx_buf.size() && rx_buf.at(pos) != ':')
                pos++;
            pos++;
            digital_values[ch] = (pos < rx_buf.size() && rx_buf.at(pos) == '1') ? 1 : 0;
            while (pos < rx_buf.size() && rx_buf.at(pos) != ',' && rx_buf.at(pos) != TAIL)
                pos++;
            pos++;
        }
        digital = true;
    }
    else
    {
        for (int ch = 0; ch < 8; ch++)
        {
            while (pos < rx_buf.size() && rx_buf.at(pos) != ':')
                pos++;
            pos++;
            uint16_t val = 0;
            while (pos < rx_buf.size() && rx_buf.at(pos) != ',' && rx_buf.at(pos) != TAIL)
            {
                if (rx_buf.at(pos) >= '0' && rx_buf.at(pos) <= '9')
                    val = val * 10 + static_cast<uint16_t>(rx_buf.at(pos) - '0');
                pos++;
            }
            analog_values[ch] = val;
            pos++;
        }
        digital = false;
    }
}
