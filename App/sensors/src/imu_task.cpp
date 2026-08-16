/**
 * @file    imu_task.cpp
 * @brief   IMU(10轴串口姿态传感器)任务 (imu)
 *
 *          每 10ms 取一次串口队列喂给 imu_9::feed_byte 组帧解析，
 *          并把欧拉角/角速度发布到全局 volatile 变量供其他任务读取
 *          (巡线外环的陀螺仪阻尼、yaw 保持、圈数统计都用到这里的数据)。
 *
 *          注: 本任务当前仍使用 UART2 硬件，实际接收路由以 uart_isr.cpp 为准。
 */

#include <cstdio>
#include <cmath>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "imu.h"
#include "imu_task.h"
#include "debug_print.h"

static constexpr const char *NAME  = "imu";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 3;
static constexpr UBaseType_t QUEUE_LEN = 256;

QueueHandle_t imu_queue = NULL;
uint8_t imu_rx_byte;

volatile float imu_roll = 0.0f;
volatile float imu_pitch = 0.0f;
volatile float imu_yaw = 0.0f;
volatile float imu_gz  = 0.0f;

static void imu_task(void *pv)
{
    (void)pv;
    IMU_DBG("imu_task start\r\n");

    imu_9 imu(&huart2);
    HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);

    uint32_t print_cnt = 0;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        // 取尽队列字节并逐字节组帧
        uint8_t byte;
        while (xQueueReceive(imu_queue, &byte, 0) == pdTRUE)
        {
            imu.feed_byte(byte);
        }

        // 发布欧拉角(rad)与陀螺仪 Z 轴角速度到全局
        const auto &a = imu.get_angles();
        imu_yaw = a.yaw;
        imu_gz  = static_cast<float>(imu.get_raw().gz);

        print_cnt++;
        if (print_cnt >= 20)
        {
            print_cnt = 0;
            float roll_deg  = a.roll  * 180.0f / 3.14159265f;
            float pitch_deg = a.pitch * 180.0f / 3.14159265f;
            float yaw_deg   = a.yaw   * 180.0f / 3.14159265f;
            IMU_DBG("gz=%d\r\n", (int)imu_gz);
            IMU_DBG("Roll: %.2f  Pitch: %.2f  Yaw: %.2f\r\n",
                    roll_deg, pitch_deg, yaw_deg);
        }
    }
}

void imu_task_create()
{
    imu_queue = xQueueCreate(QUEUE_LEN, sizeof(uint8_t));
    xTaskCreate(imu_task, NAME, STACK, NULL, PRIO, NULL);
}
