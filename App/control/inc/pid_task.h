#pragma once

#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"

// ===== 管道滚球平衡任务 (ballpid) 共享接口 =====
// 上位机视觉帧结构: cx/cy 球心坐标, errx/erry 像素偏差,
//                   vx/vy 球速度, ax/ay 球加速度(单位由上位机定义), tick_ms 接收时刻。

typedef struct
{
    int32_t cx;
    int32_t cy;
    int32_t errx;
    int32_t erry;
    int32_t vx;
    int32_t vy;
    int32_t ax;
    int32_t ay;
    uint32_t tick_ms;
} vision_frame_t;

void pid_task_create(void);

extern QueueHandle_t pid_uart_queue;   // UART2 字节队列(ISR -> pid_task)
extern uint8_t pid_uart_rx_byte;

extern volatile vision_frame_t vision_frame;  // 最新解析出的视觉帧(供调试/上层读取)
extern volatile float ball_target_pitch;      // 目标管道倾角(rad)
extern volatile float ball_servo_angle;       // 舵机实际角度(°)
extern volatile float ball_pid_output;        // 内环 PID 输出(舵机修正量, °)
