#pragma once

#include <cstdint>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"

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

extern QueueHandle_t pid_uart_queue;
extern uint8_t pid_uart_rx_byte;

extern volatile vision_frame_t vision_frame;
extern volatile float ball_target_pitch;
extern volatile float ball_servo_angle;
extern volatile float ball_pid_output;
