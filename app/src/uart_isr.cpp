#include "stm32f4xx_hal.h"
#include "usart.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"

extern QueueHandle_t track_queue;
extern uint8_t track_rx_byte;

extern QueueHandle_t imu_queue;
extern uint8_t imu_rx_byte;

extern "C"
{

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart3)
    {
        xQueueSendFromISR(track_queue, &track_rx_byte, &xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart3, &track_rx_byte, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else if (huart == &huart2)
    {
        xQueueSendFromISR(imu_queue, &imu_rx_byte, &xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        __HAL_UART_CLEAR_FEFLAG(&huart3);
        __HAL_UART_CLEAR_NEFLAG(&huart3);
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        HAL_UART_Receive_IT(&huart3, &track_rx_byte, 1);
    }
    else if (huart == &huart2)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        __HAL_UART_CLEAR_FEFLAG(&huart2);
        __HAL_UART_CLEAR_NEFLAG(&huart2);
        __HAL_UART_CLEAR_PEFLAG(&huart2);
        HAL_UART_Receive_IT(&huart2, &imu_rx_byte, 1);
    }
}

}
