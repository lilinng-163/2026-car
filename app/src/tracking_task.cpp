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

QueueHandle_t track_queue = NULL;
uint8_t track_rx_byte;

tracking track;
volatile float pos = 0.0f;
volatile float err = 0.0f;

// 解析出一帧后更新 pos/err/事件
static void process_frame(void)
{
    int black_idx[8];
    int black_count = 0;
    for (int i = 0; i < 8; i++)
    {
        if (track.digital_values[i] == 0)   // 黑=0
            black_idx[black_count++] = i;
    }

    if (black_count == 0)
    {
        track.e = tracking::tracking_event::LOST;
        return;
    }

    // 黑灯必须连续，排除路面不平导致的散点噪声
    bool contiguous = true;
    for (int i = 1; i < black_count; i++)
    {
        if (black_idx[i] - black_idx[i - 1] > 1)
        {
            contiguous = false;
            break;
        }
    }
    if (!contiguous)
        return;

    // 异常大片 (起跑线最多4~5个)
    if (black_count > 5)
        return;

    // 正常: 计算黑色重心
    float sum = 0, weighted = 0;
    for (int i = 0; i < 8; i++)
    {
        float v = 1.0f - track.digital_values[i];
        sum += v;
        weighted += v * i;
    }

    pos = weighted / sum;
    err = pos - middle;

    // 限制单帧跳变，防止噪声拉偏
    static float last_err = 0;
    static bool  first = true;
    if (first) { last_err = err; first = false; }
    float delta = err - last_err;
    if (delta >  1.0f) err = last_err + 1.0f;
    if (delta < -1.0f) err = last_err - 1.0f;
    last_err = err;

    if (err > 0.5f)       track.e = tracking::tracking_event::RIGHT;
    else if (err < -0.5f) track.e = tracking::tracking_event::LEFT;
    else                  track.e = tracking::tracking_event::MIDDLE;

    // 20 cnt打印一次
    static uint32_t cnt = 0;
    cnt++;
    if (cnt >= 20)
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

static void tracking_task(void *pv)
{
    (void)pv;
    TRACKING_DBG("tracking_task start\r\n");

    HAL_UART_Receive_IT(&huart3, &track_rx_byte, 1);

    // 发送数字命令启动数据流
    HAL_UART_Transmit(&huart3, (const uint8_t *)digital_cmd.data(),
                      digital_cmd.size(), 100);

    while (1)
    {
        // 10ms 批处理一次，与速度环周期同步：每个 PID 周期都拿到最新 pos/err
        vTaskDelay(pdMS_TO_TICKS(10));

        uint8_t byte;
        while (xQueueReceive(track_queue, &byte, 0) == pdTRUE)
        {
            if (track.feed_byte(byte) == 0)
            {
                process_frame();
            }
        }
    }
}

void tracking_task_create(void)
{
    track_queue = xQueueCreate(QUEUE_LEN, sizeof(uint8_t));
    xTaskCreate(tracking_task, NAME, STACK, NULL, PRIO, NULL);
}
