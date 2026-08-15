/**
 * @file    imu.cpp
 * @brief   IMU(10轴串口姿态传感器)协议驱动实现
 *
 *          帧格式: 0x7E 0x23 [长度] [功能字] [数据...] [校验和]
 *          校验和 = 帧头起至校验位前所有字节累加取最低字节。
 *          支持功能字: 0x04 原始三轴数据 / 0x16 四元数 / 0x26 欧拉角。
 *          小端字节序，详见 README.md "IMU" 一节。
 */

#include <cstring>
#include "stm32f4xx_hal.h"
#include "imu.h"

imu_9::imu_9(UART_HandleTypeDef *_huart)
    : huart(_huart), raw_data{}, q_data{}, angles{}
{
}

// 逐字节喂入组帧; 返回 0=解析出一帧, -1=数据不足/校验失败/需同步
// 内部流程: 满则清 -> 追加 -> 校验头/长度 -> 凑齐整帧 -> 校验和 -> parse_frame
int imu_9::feed_byte(uint8_t byte)
{
    if (rx_buf.full())
    {
        rx_buf.clear();
    }
    rx_buf.push_back(byte);

    if (rx_buf.size() < 3)
        return -1;

    if (rx_buf.at(0) != HEADER1 || rx_buf.at(1) != HEADER2)
    {
        sync_to_header();
        return -1;
    }

    uint8_t frame_len = rx_buf.at(2);
    if (frame_len < 5 || frame_len > RX_BUF_SIZE)
    {
        sync_to_header();
        return -1;
    }

    if (rx_buf.size() < frame_len)
        return -1;

    uint8_t csum = 0;
    for (uint8_t i = 0; i < frame_len - 1; i++)
        csum += rx_buf.at(i);
    if (csum != rx_buf.at(frame_len - 1))
    {
        sync_to_header();
        return -1;
    }

    etl::span<uint8_t> frame(rx_buf.data(), frame_len);
    int ret = parse_frame(frame);
    rx_buf.clear();
    return ret;
}

// 丢失同步时把缓冲滑动到下一个 0x7E 包头位置
void imu_9::sync_to_header(void)
{
    for (size_t i = 0; i < rx_buf.size(); i++)
    {
        if (rx_buf.at(i) == HEADER1)
        {
            etl::vector<uint8_t, RX_BUF_SIZE> tmp;
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

// 小端 16 位有符号整型
static int16_t le16(const uint8_t *p)
{
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

// 小端 32 位 IEEE754 浮点
static float le32f(const uint8_t *p)
{
    int32_t bits = static_cast<int32_t>(
        p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
    float f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

// 按功能字分发解析，写入对应成员(raw_data / q_data / angles)
int imu_9::parse_frame(etl::span<uint8_t> frame)
{
    uint8_t func = frame[3];
    const uint8_t *d = frame.data() + 4;

    switch (func)
    {
    case 0x04:
        raw_data.ax = le16(d + 0);
        raw_data.ay = le16(d + 2);
        raw_data.az = le16(d + 4);
        raw_data.gx = le16(d + 6);
        raw_data.gy = le16(d + 8);
        raw_data.gz = le16(d + 10);
        raw_data.mx = le16(d + 12);
        raw_data.my = le16(d + 14);
        raw_data.mz = le16(d + 16);
        return 0;

    case 0x16:
        q_data.q0 = le32f(d + 0);
        q_data.q1 = le32f(d + 4);
        q_data.q2 = le32f(d + 8);
        q_data.q3 = le32f(d + 12);
        return 0;

    case 0x26:
        angles.roll  = le32f(d + 0);
        angles.pitch = le32f(d + 4);
        angles.yaw   = le32f(d + 8);
        return 0;

    default:
        return -1;
    }
}
