#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "vector_pid_task.h"
#include "uart_cmd_task.h"

static constexpr const char *NAME  = "uart_cmd";
static constexpr configSTACK_DEPTH_TYPE STACK = 2048;
static constexpr UBaseType_t PRIO = 3;
static constexpr UBaseType_t QUEUE_LEN = 256;
static constexpr size_t LINE_BUFSIZ = 64;

QueueHandle_t uart1_queue = NULL;
uint8_t uart1_rx_byte;

struct cmd_entry {
    const char *key;
    void (*setter)(const char *val);
};

static float parse_float(const char *s)
{
    return strtof(s, NULL);
}

static void cmd_left_sp(const char *val)
{
    left_base_rpm  = parse_float(val);
    //printf("[cmd] left_sp = %.2f\r\n", (double)left_base_rpm);
}

static void cmd_right_sp(const char *val)
{
    right_base_rpm = parse_float(val);
    //printf("[cmd] right_sp = %.2f\r\n", (double)right_base_rpm);
}

static void cmd_both_sp(const char *val)
{
    float v = parse_float(val);
    left_base_rpm = right_base_rpm = v;
    //printf("[cmd] both_sp = %.2f\r\n", (double)v);
}

static void cmd_l_kp(const char *val)
{
    arm_pid_instance_f32 inst = left_motor_pid.get_instance();
    float v = parse_float(val);
    left_motor_pid.set_gains(v, inst.Ki, inst.Kd);
    //printf("[cmd] l_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)v, (double)inst.Ki, (double)inst.Kd);
}

static void cmd_l_ki(const char *val)
{
    arm_pid_instance_f32 inst = left_motor_pid.get_instance();
    float v = parse_float(val);
    left_motor_pid.set_gains(inst.Kp, v, inst.Kd);
    //printf("[cmd] l_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)inst.Kp, (double)v, (double)inst.Kd);
}

static void cmd_l_kd(const char *val)
{
    arm_pid_instance_f32 inst = left_motor_pid.get_instance();
    float v = parse_float(val);
    left_motor_pid.set_gains(inst.Kp, inst.Ki, v);
    //printf("[cmd] l_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)inst.Kp, (double)inst.Ki, (double)v);
}

static void cmd_r_kp(const char *val)
{
    arm_pid_instance_f32 inst = right_motor_pid.get_instance();
    float v = parse_float(val);
    right_motor_pid.set_gains(v, inst.Ki, inst.Kd);
    //printf("[cmd] r_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)v, (double)inst.Ki, (double)inst.Kd);
}

static void cmd_r_ki(const char *val)
{
    arm_pid_instance_f32 inst = right_motor_pid.get_instance();
    float v = parse_float(val);
    right_motor_pid.set_gains(inst.Kp, v, inst.Kd);
    //printf("[cmd] r_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)inst.Kp, (double)v, (double)inst.Kd);
}

static void cmd_r_kd(const char *val)
{
    arm_pid_instance_f32 inst = right_motor_pid.get_instance();
    float v = parse_float(val);
    right_motor_pid.set_gains(inst.Kp, inst.Ki, v);
    //printf("[cmd] r_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)inst.Kp, (double)inst.Ki, (double)v);
}

static void cmd_steer_kp(const char *val)
{
    steer_kp = parse_float(val);
    //printf("[cmd] steer_kp = %.4f\r\n", (double)steer_kp);
}

static void cmd_steer_kd(const char *val)
{
    steer_kd = parse_float(val);
    //printf("[cmd] steer_kd = %.4f\r\n", (double)steer_kd);
}

static void cmd_gz_k(const char *val)
{
    gz_k = parse_float(val);
    //printf("[cmd] gz_k = %.4f\r\n", (double)gz_k);
}

static void cmd_turn_k(const char *val)
{
    turn_k = parse_float(val);
    //printf("[cmd] turn_k = %.4f\r\n", (double)turn_k);
}

static void cmd_y_kp(const char *val)
{
    arm_pid_instance_f32 inst = yaw_pid.get_instance();
    float v = parse_float(val);
    yaw_pid.set_gains(v, inst.Ki, inst.Kd);
    //printf("[cmd] y_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)v, (double)inst.Ki, (double)inst.Kd);
}

static void cmd_y_ki(const char *val)
{
    arm_pid_instance_f32 inst = yaw_pid.get_instance();
    float v = parse_float(val);
    yaw_pid.set_gains(inst.Kp, v, inst.Kd);
    //printf("[cmd] y_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)inst.Kp, (double)v, (double)inst.Kd);
}

static void cmd_y_kd(const char *val)
{
    arm_pid_instance_f32 inst = yaw_pid.get_instance();
    float v = parse_float(val);
    yaw_pid.set_gains(inst.Kp, inst.Ki, v);
    //printf("[cmd] y_pid: kp=%.4f ki=%.4f kd=%.4f\r\n", (double)inst.Kp, (double)inst.Ki, (double)v);
}

static void cmd_y_gain(const char *val)
{
    yaw_gain = parse_float(val);
    //printf("[cmd] y_gain = %.4f\r\n", (double)yaw_gain);
}

static void cmd_t_laps(const char *val)
{
    target_laps = parse_float(val);
    //printf("[cmd] t_laps = %.2f\r\n", (double)target_laps);
}

static const cmd_entry cmds[] = {
    {"left_sp:",  cmd_left_sp},
    {"right_sp:", cmd_right_sp},
    {"both_sp:",  cmd_both_sp},
    {"l_kp:",     cmd_l_kp},
    {"l_ki:",     cmd_l_ki},
    {"l_kd:",     cmd_l_kd},
    {"r_kp:",     cmd_r_kp},
    {"r_ki:",     cmd_r_ki},
    {"r_kd:",     cmd_r_kd},
    {"steer_kp:", cmd_steer_kp},
    {"steer_kd:", cmd_steer_kd},
    {"gz_k:",     cmd_gz_k},
    {"turn_k:",   cmd_turn_k},
    {"y_kp:",     cmd_y_kp},
    {"y_ki:",     cmd_y_ki},
    {"y_kd:",     cmd_y_kd},
    {"y_gain:",   cmd_y_gain},
    {"t_laps:",   cmd_t_laps},
};

static void process_command(const char *line)
{
    for (const auto &c : cmds)
    {
        size_t klen = strlen(c.key);
        if (strncmp(line, c.key, klen) == 0)
        {
            c.setter(line + klen);
            return;
        }
    }
}

static void uart_cmd_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(500));

    HAL_StatusTypeDef rc = HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    //printf("[cmd] rx_init: ret=%d RxState=%d\r\n",
           //(int)rc, (int)huart1.RxState);

    char line[LINE_BUFSIZ];
    size_t idx = 0;

    while (1)
    {
        uint8_t byte;
        if (xQueueReceive(uart1_queue, &byte, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (byte == '\r' || byte == '\n')
            {
                printf("\r\n");
                if (idx > 0)
                {
                    line[idx] = '\0';
                    process_command(line);
                    idx = 0;
                }
            }
            else
            {
                printf("%c", byte);
                if (idx < LINE_BUFSIZ - 1)
                {
                    line[idx++] = (char)byte;
                }
            }
        }
    }
}

void uart_cmd_task_create(void)
{
    uart1_queue = xQueueCreate(QUEUE_LEN, sizeof(uint8_t));
    if (xTaskCreate(uart_cmd_task, NAME, STACK, NULL, PRIO, NULL) != pdPASS)
    {
        printf("[err] uart_cmd task create failed\r\n");
    }
}
