/**
 * @file    soft_i2c.cpp
 * @brief   软件 I2C (GPIO 模拟) 主设备驱动实现 —— SoftDrivers 层
 *
 * @note    时序: START/STOP/字节收发/ACK。SCL/SDA 通过 HAL_GPIO 操作。
 *          - send_data():    START + 设备地址(W) + 寄存器地址 + 数据... + STOP
 *          - receive_data(): START + 设备地址(W) + 寄存器地址 + RESTART + 设备地址(R) + 数据... + NACK + STOP
 */

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "soft_i2c.h"

//==============================================================================
//  总线延时 —— 决定 SCL 频率。F4@168MHz 下软件 I2C 需要足够的建立/保持时间，
//  以便开漏 + 上拉的 SCL/SDA 电平能被从机正确采样 (~几十~百 kHz)。
//==============================================================================

void soft_i2c::i2c_delay(void)
{
    for(volatile uint32_t i = 0; i < delay_cycles; i++)
    {
        __NOP();
    }
}

//==============================================================================
//  GPIO 底层操作
//==============================================================================

void soft_i2c::scl_write(int level)
{
    HAL_GPIO_WritePin(gpiox, scl_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    i2c_delay();
}

void soft_i2c::sda_write(int level)
{
    HAL_GPIO_WritePin(gpiox, sda_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    i2c_delay();
}

int soft_i2c::sda_read(void)
{
    return (HAL_GPIO_ReadPin(gpiox, sda_pin) == GPIO_PIN_SET) ? 1 : 0;
}

//==============================================================================
//  构造
//==============================================================================

soft_i2c::soft_i2c(GPIO_TypeDef *_gpiox, uint16_t _scl, uint16_t _sda, uint32_t _delay)
: gpiox(_gpiox), scl_pin(_scl), sda_pin(_sda), delay_cycles(_delay)
{
    scl_write(1);
    sda_write(1);
}

//==============================================================================
//  写事务
//==============================================================================

int soft_i2c::send_data(unsigned char dev_addr_w, unsigned char reg_addr,
                        unsigned char *data, int len)
{
    start();

    if(send_byte(dev_addr_w))  
    { 
        stop(); 
        return -1; 
    }
    if(send_byte(reg_addr))    
    { 
        stop(); 
        return -1; 
    }

    for(int i = 0; i < len; i++)
    {
        if(send_byte(data[i]))
        { 
            stop();
            return -1; 
        }
    }

    stop();
    return 0;
}

//==============================================================================
//  读事务
//==============================================================================

int soft_i2c::receive_data(unsigned char dev_addr_w, unsigned char reg_addr,
                           unsigned char *data, int len)
{
    start();

    if(send_byte(dev_addr_w))         
    { 
        stop(); 
        return -1; 
    }
    if(send_byte(reg_addr))           
    { 
        stop(); 
        return -1; 
    }

    start();

    if(send_byte(dev_addr_w | 0x01))  
    { 
        stop(); 
        return -1; 
    }

    for(int i = 0; i < len; i++)
    {
        data[i] = receive_byte();
        send_ack(i == len - 1 ? 1 : 0);
    }

    stop();
    return 0;
}

//==============================================================================
//  底层时序
//==============================================================================

void soft_i2c::start(void)
{
    sda_write(1);
    scl_write(1);
    sda_write(0);
    scl_write(0);
}

void soft_i2c::stop(void)
{
    sda_write(0);
    scl_write(0);
    scl_write(1);
    sda_write(1);
}

unsigned char soft_i2c::send_byte(unsigned char byte)
{
    scl_write(0);
    for(int i = 0; i < 8; i++)
    {
        scl_write(0);
        sda_write((byte & (0x80 >> i)) ? 1 : 0);
        scl_write(1);
        scl_write(0);
    }
    sda_write(1);
    return receive_ack();
}

unsigned char soft_i2c::receive_byte(void)
{
    unsigned char data = 0x00;

    sda_write(1);
    for(int i = 0; i < 8; i++)
    {
        scl_write(1);
        if(sda_read())
        {
            data |= (0x80 >> i);
        }
        scl_write(0);
    }

    return data;
}

void soft_i2c::send_ack(unsigned char ack)
{
    scl_write(0);
    sda_write(ack ? 1 : 0);
    scl_write(1);
    scl_write(0);
}

unsigned char soft_i2c::receive_ack(void)
{
    unsigned char ack;

    scl_write(0);
    sda_write(1);
    scl_write(1);
    ack = sda_read();
    scl_write(0);

    return ack;  // 0=ACK, 1=NACK
}
