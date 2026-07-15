#include <cstdio>
#include <string_view>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "tracking.h"
#include "tracking_task.h"
#include "debug_print.h"

static constexpr std::string_view calibration = "$1,0,0#";
static constexpr std::string_view digital_cmd = "$0,0,1#";
static constexpr std::string_view analog_cmd = "$0,1,0#";
static constexpr std::string_view both_cmd = "$0,1,1#";

static constexpr const char *NAME  = "tracking";
static constexpr configSTACK_DEPTH_TYPE STACK = 512;
static constexpr UBaseType_t PRIO = 3;
static constexpr UBaseType_t QUEUE_LEN = 256;

static QueueHandle_t track_queue = NULL;
static uint8_t rx_byte;

tracking track;
volatile float pos = 0.0f;
volatile float err = 0.0f;

extern "C"
{
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(track_queue, &rx_byte, &xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}
}

static void tracking_task(void *pv)
{
    (void)pv;
    TRACKING_DBG("tracking_task start\r\n");

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    // 发送模拟命令启动数据流
    HAL_UART_Transmit(&huart3, (const uint8_t *)digital_cmd.data(),
                      digital_cmd.size(), 100);

    while (1)
    {
        uint8_t byte;
        if (xQueueReceive(track_queue, &byte, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            if (track.feed_byte(byte) == 0)
            {
                float sum = 0, weithted = 0;
                for(int i = 0; i < 8; i++)
                {
                    float v = track.digital_values[i];
                    sum += v;
                    weithted += v * i;
                }
                if(sum < 0.01f)
                {
                    track.e = tracking::tracking_event::LOST;
                }
                else
                {
                    pos = weithted / sum;
                    err = pos - middle;

                    if(err > 0.5f)
                    {
                        track.e = tracking::tracking_event::RIGHT;
                    }
                    else if(err < -0.5f)
                    {
                        track.e = tracking::tracking_event::LEFT;
                    }
                    else
                    {
                        track.e = tracking::tracking_event::MIDDLE;
                    }
                }

                // 20 cnt打印一次
                static uint32_t cnt = 0;
                cnt++;
                if(cnt >= 20)
                {
                cnt = 0;
                if (track.digital)
                {
                    TRACKING_DBG("D: %d,%d,%d,%d,%d,%d,%d,%d\r\n",
                           track.digital_values[0], track.digital_values[1],
                           track.digital_values[2], track.digital_values[3],
                           track.digital_values[4], track.digital_values[5],
                           track.digital_values[6], track.digital_values[7]);
                }
                else
                {
                    TRACKING_DBG("A: %d,%d,%d,%d,%d,%d,%d,%d\r\n",
                           track.analog_values[0], track.analog_values[1],
                           track.analog_values[2], track.analog_values[3],
                           track.analog_values[4], track.analog_values[5],
                           track.analog_values[6], track.analog_values[7]);
                }

                }
            }
        }
    }
}

void tracking_task_create(void)
{
    track_queue = xQueueCreate(QUEUE_LEN, sizeof(uint8_t));
    xTaskCreate(tracking_task, NAME, STACK, NULL, PRIO, NULL);
}
