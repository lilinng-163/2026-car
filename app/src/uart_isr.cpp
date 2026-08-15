/**
 * @file    uart_isr.cpp
 * @brief   UART 中断回调集中实现
 *
 *          三路串口共用 HAL_UART_RxCpltCallback，把每字节推入各自队列后
 *          立即重新开启中断接收；错误回调负责清标志并重开接收。
 *
 *          通道分配:
 *            USART1 -> uart1_queue    调试命令行 (uart_cmd_task)
 *            USART2 -> pid_uart_queue 视觉帧    (pid_task)
 *            USART3 -> track_queue    循迹数据  (tracking_task)
 *          注: 本回调按硬件实例分发，实际生效路由以这里为准。
 */

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"

extern QueueHandle_t track_queue;
extern uint8_t track_rx_byte;

extern QueueHandle_t imu_queue;
extern uint8_t imu_rx_byte;

extern QueueHandle_t pid_uart_queue;
extern uint8_t pid_uart_rx_byte;

extern QueueHandle_t uart1_queue;
extern uint8_t uart1_rx_byte;

extern "C"
{

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 收到一字节 -> 推入对应队列 -> 立即重开中断接收(单字节轮询式)
    if (huart == &huart1)
    {
        xQueueSendFromISR(uart1_queue, &uart1_rx_byte, &xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else if (huart == &huart2)
    {
        xQueueSendFromISR(pid_uart_queue, &pid_uart_rx_byte, &xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart2, &pid_uart_rx_byte, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else if (huart == &huart3)
    {
        xQueueSendFromISR(track_queue, &track_rx_byte, &xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart3, &track_rx_byte, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// 串口错误处理: 清除溢出/帧/噪声/奇偶错误标志后重开接收，防止接收停摆
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    else if (huart == &huart2)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        __HAL_UART_CLEAR_FEFLAG(&huart2);
        __HAL_UART_CLEAR_NEFLAG(&huart2);
        __HAL_UART_CLEAR_PEFLAG(&huart2);
        HAL_UART_Receive_IT(&huart2, &pid_uart_rx_byte, 1);
    }
    else if (huart == &huart3)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        __HAL_UART_CLEAR_FEFLAG(&huart3);
        __HAL_UART_CLEAR_NEFLAG(&huart3);
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        HAL_UART_Receive_IT(&huart3, &track_rx_byte, 1);
    }
}

}
