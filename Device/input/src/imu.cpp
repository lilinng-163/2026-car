#include <cstdio>
#include <cstring>
#include "stm32f4xx_hal.h"
#include "imu.h"

imu_10::imu_10(UART_HandleTypeDef *_huart)
    : huart(_huart), ver_ready(false), data_sem(nullptr)
{
}

void imu_10::send_frame(const uint8_t *data, uint8_t len)
{
    if(len < 2)
    {
        return;
    }
    uint8_t frame[64];
    std::memcpy(frame, data, len);

    uint8_t csum = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        csum += frame[i];
    }
    frame[len] = csum;

    uint8_t total_len = len + 1;
    frame[2] = total_len;

    HAL_UART_Transmit(huart, frame, total_len, 10);
}

int imu_10::get_version(void)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x80, 0x01, 0x00};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_10::calibration_6(void)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x70, 0x01, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_10::calibration_3(void)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x71, 0x01, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_10::calibration_temp(int16_t temp_100x)
{
    uint8_t data[] = {
        HEADER1, HEADER2, 0x00, 0x73,
        static_cast<uint8_t>(temp_100x & 0xFF),
        static_cast<uint8_t>((temp_100x >> 8) & 0xFF),
        0x5F
    };
    send_frame(data, sizeof(data));
    return 0;
}

int imu_10::set_freq(uint8_t freq)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x60, freq, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_10::set_calculation(uint8_t method)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x61, method, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_10::reset_data(void)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0xA0, 0x01, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

void imu_10::feed_byte(uint8_t byte)
{
    if(rx_buf.full())
    {
        rx_buf.clear();
    }
    rx_buf.push_back(byte);

    if(rx_buf.size() < 3)
    {
        return;
    }

    if(rx_buf.at(0) != HEADER1 || rx_buf.at(1) != HEADER2)
    {
        sync_to_header();
        return;
    }

    uint8_t frame_len = rx_buf.at(2);
    if(frame_len < 4 || frame_len > RX_BUF_SIZE)
    {
        sync_to_header();
        return;
    }

    if(rx_buf.size() < frame_len)
    {
        return;
    }

    uint8_t csum = 0;
    for(uint8_t i = 0; i < frame_len - 1; i++)
    {
        csum += rx_buf.at(i);
    }
    if(csum != rx_buf.at(frame_len - 1))
    {
        rx_buf.clear();
        return;
    }

    etl::span<uint8_t> frame(rx_buf.data(), frame_len);
    parse_frame(frame);
    rx_buf.clear();
}

void imu_10::sync_to_header(void)
{
    for(size_t i = 0; i < rx_buf.size(); i++)
    {
        if(rx_buf.at(i) == HEADER1)
        {
            etl::vector<uint8_t, RX_BUF_SIZE> tmp;
            for(size_t j = i; j < rx_buf.size(); j++)
            {
                if(!tmp.full())
                {
                    tmp.push_back(rx_buf.at(j));
                }
            }
            rx_buf = tmp;
            return;
        }
    }
    rx_buf.clear();
}

static int16_t le16(const uint8_t *p)
{
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

void imu_10::parse_frame(etl::span<uint8_t> frame)
{
    uint8_t func = frame[3];
    const uint8_t *d = frame.data() + 4;

    switch(func)
    {
    case 0x01:
        ver_major = d[0];
        ver_minor = d[1];
        ver_patch = d[2];
        ver_ready = true;
        break;

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
        if(data_sem)
        {
            xSemaphoreGive(data_sem);
        }
        break;

    case 0x16:
    {
        auto le32f = [](const uint8_t *p) -> float {
            int32_t bits = static_cast<int32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
            float f;
            std::memcpy(&f, &bits, sizeof(f));
            return f;
        };
        q_data.q0 = le32f(d + 0);
        q_data.q1 = le32f(d + 4);
        q_data.q2 = le32f(d + 8);
        q_data.q3 = le32f(d + 12);
        if(data_sem)
        {
            xSemaphoreGive(data_sem);
        }
        break;
    }

    case 0x26:
    {
        auto le32f = [](const uint8_t *p) -> float {
            int32_t bits = static_cast<int32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
            float f;
            std::memcpy(&f, &bits, sizeof(f));
            return f;
        };
        angles.roll  = le32f(d + 0);
        angles.pitch = le32f(d + 4);
        angles.yaw   = le32f(d + 8);
        if(data_sem)
        {
            xSemaphoreGive(data_sem);
        }
        break;
    }

    default:
        break;
    }
}

bool imu_10::version_ready(void)
{
    return ver_ready;
}
