/**
 * @file    soft_i2c.h
 * @brief   软件 I2C (GPIO 模拟) 主设备驱动声明 —— SoftDrivers 层
 *
 * @details 通过 GPIO 模拟标准 I2C 时序，提供 START/STOP/字节收发/ACK 处理，
 *          以及面向寄存器的写/读事务。HAL 调用集中于本类，上层设备(如 oled096)
 *          通过组合本类访问 I2C 总线。
 *
 * @note    引脚模式(开漏/推挽)由 CubeMX 生成的 MX_GPIO_Init() 完成，本类只做电平读写。
 */

#pragma once

#include <cstdint>
#include "stm32f407xx.h"

class soft_i2c
{
public:
    soft_i2c(GPIO_TypeDef *_gpiox, uint16_t _scl, uint16_t _sda, uint32_t _delay = 5);

    /**
     * @brief I2C 写事务: START + 设备地址(W) + 寄存器地址 + 数据... + STOP
     * @return 0=成功, -1=ACK 错误
     */
    int send_data(unsigned char dev_addr_w, unsigned char reg_addr,
                  unsigned char *data, int len);

    /**
     * @brief I2C 读事务: START + 设备地址(W) + 寄存器地址 + RESTART + 设备地址(R) + 数据... + NACK + STOP
     * @return 0=成功, -1=ACK 错误
     */
    int receive_data(unsigned char dev_addr_w, unsigned char reg_addr,
                     unsigned char *data, int len);

private:
    GPIO_TypeDef *gpiox;
    uint16_t scl_pin;
    uint16_t sda_pin;
    uint32_t delay_cycles;

    void i2c_delay(void);
    void scl_write(int level);
    void sda_write(int level);
    int  sda_read(void);

    void start(void);
    void stop(void);
    unsigned char send_byte(unsigned char byte);
    unsigned char receive_byte(void);
    void send_ack(unsigned char ack);
    unsigned char receive_ack(void);
};
