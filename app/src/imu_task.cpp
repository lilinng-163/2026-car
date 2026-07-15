#include <cstdio>
#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "imu.h"
#include "imu_task.h"
#include "debug_print.h"
#include "etl/vector.h"

static constexpr const char *NAME  = "imu";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 3;
// static constexpr UBaseType_t QUEUE_LEN = 512;

// static QueueHandle_t imu_queue = NULL;
// static SemaphoreHandle_t imu_sem = NULL;
// static imu_9 imu(&huart2);

// extern "C"
// {
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if(huart == &huart2)
//     {
//         static uint8_t rx_byte;
//         HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

//         BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//         xQueueSendFromISR(imu_queue, &rx_byte, &xHigherPriorityTaskWoken);
//         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//     }
// }
// }

static void imu_task(void *pv)
{
    // imu.get_version();

    // vTaskDelay(pdMS_TO_TICKS(500));
    // imu.calibration_6();
    // vTaskDelay(pdMS_TO_TICKS(3000));
    // imu.calibration_3();
    // vTaskDelay(pdMS_TO_TICKS(3000));

    imu_6 mpu6050(GPIOC, GPIO_PIN_6, GPIO_PIN_7);
    mpu_6050_data_t data;
    memset(&data, 0, sizeof(data));

    unsigned char id = 0;
    int wret = mpu6050.who_am_i(id);
    IMU_DBG("who_am_i ret = %d, id = 0x%02X (expect 0x68)\r\n", wret, id);

    int ret = mpu6050.init();
    IMU_DBG("mpu6050 init ret = %d\r\n", ret);
    while(ret != 0)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = mpu6050.init();
        IMU_DBG("mpu6050 retry init ret = %d\r\n", ret);
    }

    while(1)
    {
        int r = mpu6050.get_data(data);
        IMU_DBG("get_data ret = %d\r\n", r);
        IMU_DBG("ax: %f\r\n", data.ax);
        IMU_DBG("ay: %f\r\n", data.ay);
        IMU_DBG("az: %f\r\n", data.az);
        IMU_DBG("gx: %f\r\n", data.gx);
        IMU_DBG("gy: %f\r\n", data.gy);
        IMU_DBG("gz: %f\r\n", data.gz);

        vTaskDelay(pdMS_TO_TICKS(500));
        // uint8_t byte;
        // if(xQueueReceive(imu_queue, &byte, portMAX_DELAY) == pdTRUE)
        // {
        //     imu.feed_byte(byte);
        // }

        // while(xSemaphoreTake(imu_sem, 0) == pdTRUE)
        // {
        //     const auto &r = imu.get_raw();
        //     const auto &q = imu.get_quat();
        //     const auto &a = imu.get_angles();
        //     printf("IMU raw: a(%d,%d,%d) g(%d,%d,%d) m(%d,%d,%d)\r\n",
        //            r.ax, r.ay, r.az, r.gx, r.gy, r.gz, r.mx, r.my, r.mz);
        //     printf("IMU quat: w=%.4f x=%.4f y=%.4f z=%.4f\r\n",
        //            q.q0, q.q1, q.q2, q.q3);
        //     printf("IMU euler: roll=%.1fdeg pitch=%.1fdeg yaw=%.1fdeg\r\n",
        //            a.roll * 57.29578f, a.pitch * 57.29578f, a.yaw * 57.29578f);

        //     if(imu.version_ready())
        //     {
        //         printf("IMU ver: %d.%d.%d\r\n",
        //                imu.ver_major, imu.ver_minor, imu.ver_patch);
        //     }
        // }
    }
}

void imu_task_create()
{
    // imu_sem = xSemaphoreCreateBinary();
    // imu.data_sem = imu_sem;

    // imu_queue = xQueueCreate(QUEUE_LEN, sizeof(uint8_t));
    xTaskCreate(imu_task, NAME, STACK, NULL, PRIO, NULL);

    // static uint8_t first_byte;
    // HAL_UART_Receive_IT(&huart2, &first_byte, 1);
}
