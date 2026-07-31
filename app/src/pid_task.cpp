#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "imu_task.h"
#include "pid.h"
#include "servo.h"
#include "pid_task.h"

static constexpr const char *NAME = "ballpid";
static constexpr configSTACK_DEPTH_TYPE STACK = 768;
static constexpr UBaseType_t PRIO = 4;
static constexpr UBaseType_t QUEUE_LEN = 256;
static constexpr float TS = 0.01f;
static constexpr float PI_F = 3.14159265f;

static constexpr float SERVO_CENTER_DEG = 135.0f;
static constexpr float SERVO_MIN_DEG = 90.0f;
static constexpr float SERVO_MAX_DEG = 180.0f;
static constexpr float SERVO_DIR = 1.0f; // If correction is reversed, change this to -1.0f.

QueueHandle_t pid_uart_queue = NULL;
uint8_t pid_uart_rx_byte = 0;

volatile vision_frame_t vision_frame = {};
volatile float ball_target_pitch = 0.0f;
volatile float ball_servo_angle = SERVO_CENTER_DEG;
volatile float ball_pid_output = 0.0f;

static int hex_value(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

static bool parse_i32_field(const char *payload, const char *key, int32_t &out)
{
    const char *p = std::strstr(payload, key);
    if (p == nullptr)
        return false;

    p += std::strlen(key);
    char *end = nullptr;
    long value = std::strtol(p, &end, 10);
    if (end == p)
        return false;

    out = static_cast<int32_t>(value);
    return true;
}

static bool parse_vision_line(char *line, vision_frame_t &frame)
{
    char *last_sep = std::strrchr(line, ';');
    if (last_sep == nullptr || std::strlen(last_sep + 1) != 2)
        return false;

    int hi = hex_value(last_sep[1]);
    int lo = hex_value(last_sep[2]);
    if (hi < 0 || lo < 0)
        return false;

    uint8_t expected = static_cast<uint8_t>((hi << 4) | lo);
    uint8_t actual = 0;
    for (char *p = line; p < last_sep; p++)
        actual ^= static_cast<uint8_t>(*p);
    if (actual != expected)
        return false;

    *last_sep = '\0';
    if (!parse_i32_field(line, "cx:", frame.cx) ||
        !parse_i32_field(line, "cy:", frame.cy) ||
        !parse_i32_field(line, "errx:", frame.errx) ||
        !parse_i32_field(line, "erry:", frame.erry) ||
        !parse_i32_field(line, "vx:", frame.vx) ||
        !parse_i32_field(line, "vy:", frame.vy) ||
        !parse_i32_field(line, "ax:", frame.ax) ||
        !parse_i32_field(line, "ay:", frame.ay))
    {
        return false;
    }

    frame.tick_ms = HAL_GetTick();
    return true;
}

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static void pid_task_entry(void *pv)
{
    (void)pv;

    servo pipe_servo(&htim9, TIM_CHANNEL_1, 270.0f);
    pipe_servo.start();
    pipe_servo.set_angle(SERVO_CENTER_DEG);

    pid pos_pid(0.020f, 0.0f, 0.004f, TS, -8.0f, 8.0f);       // pixel error -> target pipe pitch(deg)
    pid pitch_pid(45.0f, 0.0f, 3.0f, TS, -35.0f, 35.0f);      // pitch(rad) -> servo correction(deg)

    HAL_UART_Receive_IT(&huart2, &pid_uart_rx_byte, 1);

    char line[160] = {0};
    size_t line_len = 0;
    vision_frame_t local_frame = {};
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t dbg_cnt = 0;

    printf("ball pid start: UART2 vision + imu_pitch + servo TIM9_CH1\r\n");

    while (1)
    {
        uint8_t byte = 0;
        while (xQueueReceive(pid_uart_queue, &byte, 0) == pdTRUE)
        {
            if (byte == '\n')
            {
                line[line_len] = '\0';
                if (line_len > 0 && line[line_len - 1] == '\r')
                    line[line_len - 1] = '\0';

                vision_frame_t parsed = {};
                if (parse_vision_line(line, parsed))
                {
                    local_frame = parsed;
                    vision_frame.cx = parsed.cx;
                    vision_frame.cy = parsed.cy;
                    vision_frame.errx = parsed.errx;
                    vision_frame.erry = parsed.erry;
                    vision_frame.vx = parsed.vx;
                    vision_frame.vy = parsed.vy;
                    vision_frame.ax = parsed.ax;
                    vision_frame.ay = parsed.ay;
                    vision_frame.tick_ms = parsed.tick_ms;
                }
                line_len = 0;
            }
            else if (line_len < sizeof(line) - 1)
            {
                line[line_len++] = static_cast<char>(byte);
            }
            else
            {
                line_len = 0;
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        float target_pitch_deg = pos_pid.calculate(0.0f, static_cast<float>(local_frame.errx));
        ball_target_pitch = target_pitch_deg * PI_F / 180.0f;

        ball_pid_output = pitch_pid.calculate(ball_target_pitch, imu_pitch);
        ball_servo_angle = clampf(SERVO_CENTER_DEG + SERVO_DIR * ball_pid_output,
                                  SERVO_MIN_DEG, SERVO_MAX_DEG);
        pipe_servo.set_angle(ball_servo_angle);

        if (++dbg_cnt >= 50)
        {
            dbg_cnt = 0;
            printf("ball errx=%ld vx=%ld pitch=%.3f tgt=%.3f out=%.2f servo=%.1f\r\n",
                   static_cast<long>(local_frame.errx), static_cast<long>(local_frame.vx),
                   imu_pitch, ball_target_pitch, ball_pid_output, ball_servo_angle);
        }
    }
}

void pid_task_create(void)
{
    pid_uart_queue = xQueueCreate(QUEUE_LEN, sizeof(uint8_t));
    xTaskCreate(pid_task_entry, NAME, STACK, NULL, PRIO, NULL);
}
