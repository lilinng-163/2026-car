#include <cstdio>
#include <cstring>
#include "stm32f4xx_hal.h"
#include "imu.h"

// 亚博科技的imu9
imu_9::imu_9(UART_HandleTypeDef *_huart)
    : huart(_huart), ver_ready(false), data_sem(nullptr)
{
}

void imu_9::send_frame(const uint8_t *data, uint8_t len)
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

int imu_9::get_version(void)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x80, 0x01, 0x00};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_9::calibration_6(void)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x70, 0x01, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_9::calibration_3(void)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x71, 0x01, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_9::calibration_temp(int16_t temp_100x)
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

int imu_9::set_freq(uint8_t freq)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x60, freq, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_9::set_calculation(uint8_t method)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0x61, method, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

int imu_9::reset_data(void)
{
    uint8_t data[] = {HEADER1, HEADER2, 0x00, 0xA0, 0x01, 0x5F};
    send_frame(data, sizeof(data));
    return 0;
}

void imu_9::feed_byte(uint8_t byte)
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

void imu_9::sync_to_header(void)
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

void imu_9::parse_frame(etl::span<uint8_t> frame)
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

bool imu_9::version_ready(void)
{
    return ver_ready;
}


imu_6::imu_6(GPIO_TypeDef *_gpiox, uint16_t _scl, uint16_t _sda)
: i2c(_gpiox, _scl, _sda, 500)
{
    init();
}
void imu_6::set_dev_id(unsigned char _addr_w, unsigned char _addr_r)
{
    addr_w = _addr_w;
    addr_r = _addr_r;
}
int imu_6::get_data(mpu_6050_data_t &_data_t)
{
    unsigned char buf[14];
    if(i2c.receive_data(addr_w, ACCEL_XOUT_H, buf, 14) != 0)
    {
        return -1;
    }
        
    _data_t.ax = (int16_t)((buf[0]  << 8) | buf[1]);
    _data_t.ay = (int16_t)((buf[2]  << 8) | buf[3]);
    _data_t.az = (int16_t)((buf[4]  << 8) | buf[5]);
    _data_t.temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    _data_t.gx  = (int16_t)((buf[8]  << 8) | buf[9]);
    _data_t.gy  = (int16_t)((buf[10] << 8) | buf[11]);
    _data_t.gz  = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;

}
int imu_6::who_am_i(unsigned char &id)
{
    id = 0;
    return i2c.receive_data(addr_w, WHO_AM_I, &id, 1);
}
int imu_6::init(void)
{
    unsigned char val;

    // 1. 唤醒设备，选择内部时钟源
    val = 0x01;
    if(i2c.send_data(addr_w, PWR_MGMT_1, &val, 1) != 0)
        return -1;

    // 2. 使能所有轴
    val = 0x00;
    if(i2c.send_data(addr_w, PWR_MGMT_2, &val, 1) != 0)
        return -2;

    // 3. 配置采样率：1kHz (8kHz / (9+1) = 800Hz)
    val = 0x09;
    if(i2c.send_data(addr_w, SMPLRT_DIV, &val, 1) != 0)
        return -3;

    // 4. 配置数字低通滤波器(DLPF)
    val = 0x06;
    if(i2c.send_data(addr_w, CONFIG, &val, 1) != 0)
        return -4;

    // 5. 配置陀螺仪量程：±2000 °/s
    val = 0x18;
    if(i2c.send_data(addr_w, GYRO_CONFIG, &val, 1) != 0)
        return -5;

    // 6. 配置加速度计量程：±16g
    val = 0x18;
    if(i2c.send_data(addr_w, ACCEL_CONFIG, &val, 1) != 0)
        return -6;

    return 0;
}

