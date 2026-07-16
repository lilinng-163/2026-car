#include <cstdio>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "main.h"
#include "oled.h"
#include "vector_pid_task.h"
#include "tracking_task.h"
#include "tune_ui.h"
#include "oled_task.h"
#include "debug_print.h"

static constexpr const char *NAME  = "oled";
static constexpr configSTACK_DEPTH_TYPE STACK = 1024;
static constexpr UBaseType_t PRIO = 3;

// 页0：SP 调参页（k1 选择 L/R，k0/k2 增减，'>' 为选中标记）
static void show_page0(oled096 &o, char *buf, size_t len)
{
    // Line 0: setpoint (SP)，选中项前带 '>'
    snprintf(buf, len, "SP%cL%-5d%cR%-5d",
             (tune_select == TUNE_SP_L) ? '>' : ' ',
             static_cast<int>(left_base_rpm),
             (tune_select == TUNE_SP_R) ? '>' : ' ',
             static_cast<int>(right_base_rpm));
    o.show_string(buf, 0, 0);

    // Line 1: actual (AC)
    snprintf(buf, len, "AC L%-5d R%-5d",
             static_cast<int>(left_actual_rpm),
             static_cast<int>(right_actual_rpm));
    o.show_string(buf, 0, 16);

    // Line 2: output (OT)
    snprintf(buf, len, "OT L%-5d R%-5d",
             static_cast<int>(left_out_val),
             static_cast<int>(right_out_val));
    o.show_string(buf, 0, 32);

    // Line 3: voltage + page
    snprintf(buf, len, "V%d.%dV       P1",
             static_cast<int>(vin_actual),
             static_cast<int>(vin_actual * 10.0f) % 10);
    o.show_string(buf, 0, 48);
}

// 页1：双电机 PID 增益调节（k1 选择，'>' 为选中标记）
static void show_page1(oled096 &o, char *buf, size_t len)
{
    arm_pid_instance_f32 l = left_motor_pid.get_instance();
    arm_pid_instance_f32 r = right_motor_pid.get_instance();

    snprintf(buf, len, "PID Kp   Ki   Kd");
    o.show_string(buf, 0, 0);

    snprintf(buf, len, "L%c%d.%02d%c%d.%02d%c%d.%02d",
             (tune_select == TUNE_L_KP) ? '>' : ' ',
             static_cast<int>(l.Kp), static_cast<int>(l.Kp * 100.0f) % 100,
             (tune_select == TUNE_L_KI) ? '>' : ' ',
             static_cast<int>(l.Ki), static_cast<int>(l.Ki * 100.0f) % 100,
             (tune_select == TUNE_L_KD) ? '>' : ' ',
             static_cast<int>(l.Kd), static_cast<int>(l.Kd * 100.0f) % 100);
    o.show_string(buf, 0, 16);

    snprintf(buf, len, "R%c%d.%02d%c%d.%02d%c%d.%02d",
             (tune_select == TUNE_R_KP) ? '>' : ' ',
             static_cast<int>(r.Kp), static_cast<int>(r.Kp * 100.0f) % 100,
             (tune_select == TUNE_R_KI) ? '>' : ' ',
             static_cast<int>(r.Ki), static_cast<int>(r.Ki * 100.0f) % 100,
             (tune_select == TUNE_R_KD) ? '>' : ' ',
             static_cast<int>(r.Kd), static_cast<int>(r.Kd * 100.0f) % 100);
    o.show_string(buf, 0, 32);

    snprintf(buf, len, "             P2");
    o.show_string(buf, 0, 48);
}

// 页2：STEER_KP / STEER_KD / GZ_K + 当前位置/误差
static void show_page2(oled096 &o, char *buf, size_t len)
{
    snprintf(buf, len, "%cSTEER_KP %d.%02d",
             (tune_select == TUNE_T_KP) ? '>' : ' ',
             static_cast<int>(steer_kp), static_cast<int>(steer_kp * 100.0f) % 100);
    o.show_string(buf, 0, 0);

    snprintf(buf, len, "%cSTEER_KD %d.%03d",
             (tune_select == TUNE_T_KD) ? '>' : ' ',
             static_cast<int>(steer_kd), static_cast<int>(steer_kd * 1000.0f) % 1000);
    o.show_string(buf, 0, 16);

    snprintf(buf, len, "%cGZ_K    %d.%04d",
             (tune_select == TUNE_GZ_K) ? '>' : ' ',
             static_cast<int>(gz_k), static_cast<int>(gz_k * 10000.0f) % 10000);
    o.show_string(buf, 0, 32);

    snprintf(buf, len, "             P3");
    o.show_string(buf, 0, 48);
}

// 页3：YAW PID 增益 + 当前yaw/hdg
static void show_page3(oled096 &o, char *buf, size_t len)
{
    arm_pid_instance_f32 y = yaw_pid.get_instance();

    snprintf(buf, len, "%cY_KP  %d.%02d",
             (tune_select == TUNE_Y_KP) ? '>' : ' ',
             static_cast<int>(y.Kp), static_cast<int>(y.Kp * 100.0f) % 100);
    o.show_string(buf, 0, 0);

    snprintf(buf, len, "%cY_KI  %d.%02d",
             (tune_select == TUNE_Y_KI) ? '>' : ' ',
             static_cast<int>(y.Ki), static_cast<int>(y.Ki * 100.0f) % 100);
    o.show_string(buf, 0, 16);

    snprintf(buf, len, "%cY_KD  %d.%02d",
             (tune_select == TUNE_Y_KD) ? '>' : ' ',
             static_cast<int>(y.Kd), static_cast<int>(y.Kd * 100.0f) % 100);
    o.show_string(buf, 0, 32);

    snprintf(buf, len, "%cY_GAIN %d.%02d",
             (tune_select == TUNE_Y_GAIN) ? '>' : ' ',
             static_cast<int>(yaw_gain), static_cast<int>(yaw_gain * 100.0f) % 100);
    o.show_string(buf, 0, 32);

    snprintf(buf, len, "             P4");
    o.show_string(buf, 0, 48);
}

static void oled_task(void *pv) {
    (void)pv;
    OLED_DBG("oled start\r\n");
    static oled096 o(GPIOB, GPIO_PIN_3, GPIO_PIN_4);

    char buf[32];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));

        o.clear();

        switch (tune_page)
        {
        case 0:
            show_page0(o, buf, sizeof(buf));
            break;
        case 1:
            show_page1(o, buf, sizeof(buf));
            break;
        case 2:
            show_page2(o, buf, sizeof(buf));
            break;
        case 3:
            show_page3(o, buf, sizeof(buf));
            break;
        default:
            break;
        }

        o.refresh();
    }
}

void oled_task_create() {
    xTaskCreate(oled_task, NAME, STACK, NULL, PRIO, NULL);
}
