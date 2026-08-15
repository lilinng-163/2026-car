#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "etl/vector.h"
#include "etl/span.h"

// 原始三轴数据 (0x04): accel/gyro/mag, 各 int16 小端
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t mx;
    int16_t my;
    int16_t mz;
} a_g_m;

// 四元数 (0x16)
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion;

// 欧拉角 (0x26), 单位弧度
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} posture_angles;

// IMU 串口协议解析器: feed_byte 逐字节喂入，帧对齐/校验/解析全在此类完成
class imu_9
{
public:
    imu_9(UART_HandleTypeDef *_huart);

    int feed_byte(uint8_t byte);   // 0=解析成功一帧, -1=未完成/失败

    const a_g_m &get_raw(void) { return raw_data; }
    const quaternion &get_quat(void) { return q_data; }
    const posture_angles &get_angles(void) { return angles; }

private:
    UART_HandleTypeDef *huart;
    void send_frame(const uint8_t *data, uint8_t len);
    int parse_frame(etl::span<uint8_t> frame);
    void sync_to_header(void);

    static constexpr uint8_t HEADER1 = 0x7E;
    static constexpr uint8_t HEADER2 = 0x23;
    static constexpr size_t RX_BUF_SIZE = 64;

    etl::vector<uint8_t, RX_BUF_SIZE> rx_buf;

    a_g_m raw_data;
    quaternion q_data;
    posture_angles angles;
};
