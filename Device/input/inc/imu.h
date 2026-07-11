#pragma once

#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "stm32f4xx_hal.h"
#include "etl/vector.h"
#include "etl/span.h"

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

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} posture_angles;

class imu_9
{
public:
    imu_9(UART_HandleTypeDef *_huart);

    int get_version(void);
    int calibration_6(void);
    int calibration_3(void);
    int calibration_temp(int16_t temp_100x);
    int set_freq(uint8_t freq);
    int set_calculation(uint8_t method);
    int reset_data(void);

    void feed_byte(uint8_t byte);

    const a_g_m &get_raw(void) { return raw_data; }
    const quaternion &get_quat(void) { return q_data; }
    const posture_angles &get_angles(void) { return angles; }

    bool version_ready(void);
    uint8_t ver_major;
    uint8_t ver_minor;
    uint8_t ver_patch;

    SemaphoreHandle_t data_sem;

private:
    UART_HandleTypeDef *huart;
    void send_frame(const uint8_t *data, uint8_t len);
    void parse_frame(etl::span<uint8_t> frame);
    void sync_to_header(void);

    static constexpr uint8_t HEADER1 = 0x7E;
    static constexpr uint8_t HEADER2 = 0x23;
    static constexpr size_t RX_BUF_SIZE = 64;

    etl::vector<uint8_t, RX_BUF_SIZE> rx_buf;

    a_g_m raw_data;
    quaternion q_data;
    posture_angles angles;

    bool ver_ready;
};


// mpu6050

#include "soft_i2c.h"

// ======================== 寄存器地址 ========================

#define SMPLRT_DIV 		0x19    /**< 采样率分频器 */
#define CONFIG 	   		0x1A    /**< 数字低通滤波器配置 */
#define GYRO_CONFIG 	0x1B    /**< 陀螺仪量程配置 */
#define ACCEL_CONFIG    0x1C    /**< 加速度计量程配置 */
#define ACCEL_XOUT_H 	0x3B    /**< 加速度 X 轴高字节 */
#define ACCEL_XOUT_L 	0x3C    /**< 加速度 X 轴低字节 */
#define ACCEL_YOUT_H 	0x3D    /**< 加速度 Y 轴高字节 */
#define ACCEL_YOUT_L	0x3E    /**< 加速度 Y 轴低字节 */
#define ACCEL_ZOUT_H 	0x3F    /**< 加速度 Z 轴高字节 */
#define ACCEL_ZOUT_L	0x40    /**< 加速度 Z 轴低字节 */
#define TEMP_OUT_H 		0x41    /**< 温度高字节 */
#define TEMP_OUT_L		0x42    /**< 温度低字节 */
#define GYRO_XOUT_H		0x43    /**< 陀螺仪 X 轴高字节 */
#define GYRO_XOUT_L		0x44    /**< 陀螺仪 X 轴低字节 */
#define GYRO_YOUT_H		0x45    /**< 陀螺仪 Y 轴高字节 */
#define GYRO_YOUT_L		0x46    /**< 陀螺仪 Y 轴低字节 */
#define GYRO_ZOUT_H		0x47    /**< 陀螺仪 Z 轴高字节 */
#define GYRO_ZOUT_L		0x48    /**< 陀螺仪 Z 轴低字节 */

// ======================== 电源管理寄存器 ========================

#define PWR_MGMT_1 0x6B          /**< 电源管理 1 (唤醒/时钟源) */
#define PWR_MGMT_2 0x6C          /**< 电源管理 2 (轴使能) */

#define WHO_AM_I   0x75          /**< 设备 ID (应返回 0x68) */

// ======================== 量程配置值 ========================

#define ACCEL_RANGE_2G  0x00     /**< 加速度 ±2g */
#define ACCEL_RANGE_4G  0x08     /**< 加速度 ±4g */
#define ACCEL_RANGE_8G  0x10     /**< 加速度 ±8g */
#define ACCEL_RANGE_16G 0x18     /**< 加速度 ±16g */

#define GYRO_RANGE_250  0x00     /**< 陀螺仪 ±250°/s */
#define GYRO_RANGE_500  0x08     /**< 陀螺仪 ±500°/s */
#define GYRO_RANGE_1000 0x10     /**< 陀螺仪 ±1000°/s */
#define GYRO_RANGE_2000 0x18     /**< 陀螺仪 ±2000°/s */


typedef struct
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float temp;
}mpu_6050_data_t;

class imu_6
{
public:
    imu_6(GPIO_TypeDef *_gpiox, uint16_t _scl, uint16_t _sda);
    void set_dev_id(unsigned char _addr_w, unsigned char _addr_r);
    int get_data(mpu_6050_data_t &_data_t);
    int init(void);
    int who_am_i(unsigned char &id);
private:
    mpu_6050_data_t data_t;
    posture_angles angles;
    soft_i2c i2c;
    unsigned char addr_w = 0xD0;
    unsigned char addr_r = 0xD1;
};