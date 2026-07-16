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

// 页2：循迹 PID 增益调节 + 当前位置/误差
static void show_page2(oled096 &o, char *buf, size_t len)
{
    arm_pid_instance_f32 t = track_pid.get_instance();

    snprintf(buf, len, "%cKp %d",
             (tune_select == TUNE_T_KP) ? '>' : ' ',
             static_cast<int>(t.Kp));
    o.show_string(buf, 0, 0);

    snprintf(buf, len, "%cKi %d.%02d",
             (tune_select == TUNE_T_KI) ? '>' : ' ',
             static_cast<int>(t.Ki), static_cast<int>(t.Ki * 100.0f) % 100);
    o.show_string(buf, 0, 16);

    snprintf(buf, len, "%cKd %d.%02d",
             (tune_select == TUNE_T_KD) ? '>' : ' ',
             static_cast<int>(t.Kd), static_cast<int>(t.Kd * 100.0f) % 100);
    o.show_string(buf, 0, 32);

    // 当前位置(0~7)与误差(pos-3.5)
    float p = pos;
    float e = err;
    char es = (e < 0.0f) ? '-' : '+';
    float ea = (e < 0.0f) ? -e : e;
    snprintf(buf, len, "P%d.%02d E%c%d.%02d P3",
             static_cast<int>(p), static_cast<int>(p * 100.0f) % 100,
             es,
             static_cast<int>(ea), static_cast<int>(ea * 100.0f) % 100);
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
        default:
            break;
        }

        o.refresh();
    }
}

void oled_task_create() {
    xTaskCreate(oled_task, NAME, STACK, NULL, PRIO, NULL);
}
