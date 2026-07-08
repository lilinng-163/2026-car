#include <cstdio>
#include "lvgl.h"
#include "lv_obj.h"
#include "motor_task.h"
#include "beep.h"
#include "dht11.h"
#include "led.h"
#include "main.h"

// 6 个 PID 参数的值 label：[0..2]=左 Kp/Ki/Kd，[3..5]=右 Kp/Ki/Kd
static lv_obj_t *pid_labels[6] = {nullptr};

// 页面容器 + 当前实际显示页 + 目标页标志位
static lv_obj_t *pages[UI_PAGE_NUM] = {nullptr};
static uint8_t current_page = 0;
static volatile uint8_t target_page = 0;

// DHT11
volatile bool dht11_request = false;
static lv_obj_t *dht11_label = nullptr;

void dht11_update_ui()
{
    if (!dht11_label)
    {
        return;
    }
    static char buf[32];

    if (Dht11::instance().read())
    {
        std::snprintf(buf, sizeof(buf), "DHT11: %d\xC2\xB0""C  %d%%",
            Dht11::instance().get_temperature(),
            Dht11::instance().get_humidity());
    }
    else
    {
        std::snprintf(buf, sizeof(buf), "DHT11: ERR  ERR");
    }
    lv_label_set_text(dht11_label, buf);
}

// 刷新 6 个 PID 参数值（定点整数运算，避免浮点 printf）
void pid_update_ui()
{
    if (!pid_labels[0])
    {
        return;
    }
    static char buf[16];

    arm_pid_instance_f32 l = left_motor_pid.get_instance();
    arm_pid_instance_f32 r = right_motor_pid.get_instance();

    const float32_t vals[6] = { l.Kp, l.Ki, l.Kd, r.Kp, r.Ki, r.Kd };

    for (int i = 0; i < 6; i++)
    {
        int scaled = (int)(vals[i] * 1000.0f + (vals[i] >= 0 ? 0.5f : -0.5f));
        int ip = scaled / 1000;
        int fp = scaled % 1000;
        if (fp < 0)
        {
            fp = -fp;
        }
        std::snprintf(buf, sizeof(buf), "%d.%03d", ip, fp);
        lv_label_set_text(pid_labels[i], buf);
    }
}

// 一个电机的参数卡片：标题 + 3 行 (Kp/Ki/Kd) 键值对
static void motor_card(lv_obj_t *parent, const char *title, lv_color_t color, int slot_base)
{
    static const char *keys[3] = { "Kp", "Ki", "Kd" };

    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_width(card, LV_PCT(100));
    lv_obj_set_height(card, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(card, 10, 0);
    lv_obj_set_style_pad_row(card, 6, 0);
    lv_obj_set_style_radius(card, 8, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, color, 0);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x1e293b), 0);

    // 卡片标题
    lv_obj_t *head = lv_label_create(card);
    lv_label_set_text(head, title);
    lv_obj_set_style_text_color(head, color, 0);

    // 分隔线
    lv_obj_t *line = lv_obj_create(card);
    lv_obj_set_size(line, LV_PCT(100), 2);
    lv_obj_set_style_bg_color(line, color, 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_set_style_radius(line, 0, 0);

    // 3 行键值对
    for (int i = 0; i < 3; i++)
    {
        lv_obj_t *row = lv_obj_create(card);
        lv_obj_set_width(row, LV_PCT(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_all(row, 2, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);

        lv_obj_t *key = lv_label_create(row);
        lv_label_set_text(key, keys[i]);
        lv_obj_set_style_text_color(key, lv_color_hex(0x94a3b8), 0);

        lv_obj_t *val = lv_label_create(row);
        lv_label_set_text(val, "--");
        lv_obj_set_style_text_color(val, lv_color_hex(0xffffff), 0);
        pid_labels[slot_base + i] = val;
    }
}

// 第一页：两个电机的 6 个 PID 参数
static lv_obj_t *page1(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(cont, 8, 0);
    lv_obj_set_style_pad_row(cont, 10, 0);
    lv_obj_set_style_border_width(cont, 0, 0);

    // 页面标题
    lv_obj_t *title = lv_label_create(cont);
    lv_label_set_text(title, "PID Parameters");
    lv_obj_set_style_text_color(title, lv_color_hex(0xf1f5f9), 0);

    // 左右电机卡片
    motor_card(cont, "Left Motor",  lv_color_hex(0x22c55e), 0);
    motor_card(cont, "Right Motor", lv_color_hex(0x3b82f6), 3);

    pid_update_ui();
    return cont;
}

// 开关样式
static void apply_switch_style(lv_obj_t *sw)
{
    lv_obj_set_size(sw, 60, 30);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xef4444), LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0x22c55e),
        static_cast<uint32_t>(LV_PART_INDICATOR) | static_cast<uint32_t>(LV_STATE_CHECKED));
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xffffff), LV_PART_KNOB);
}

// 一行：左标签 + 右侧控件
static lv_obj_t *create_row(lv_obj_t *parent)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END);
    lv_obj_set_style_pad_all(row, 6, 0);
    lv_obj_set_style_pad_column(row, 10, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    return row;
}

// 第二页：LED1 / LED2 / 蜂鸣器 / DHT11
static lv_obj_t *page2(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(cont, 10, 0);
    lv_obj_set_style_pad_row(cont, 4, 0);
    lv_obj_set_style_border_width(cont, 0, 0);

    // LED1
    {
        lv_obj_t *row = create_row(cont);
        lv_obj_t *label = lv_label_create(row);
        lv_label_set_text(label, "LED1: OFF");
        lv_obj_set_flex_grow(label, 1);

        lv_obj_t *sw = lv_switch_create(row);
        apply_switch_style(sw);

        lv_obj_add_event_cb(sw, [](lv_event_t *e)
        {
            lv_obj_t *sw_ = lv_event_get_target_obj(e);
            lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
            bool on = lv_obj_has_state(sw_, LV_STATE_CHECKED);

            static led l1(LED1_GPIO_Port, LED1_Pin);
            if (on)
            {
                l1.on();
                lv_label_set_text(lbl, "LED1: ON");
            }
            else
            {
                l1.off();
                lv_label_set_text(lbl, "LED1: OFF");
            }
        }, LV_EVENT_VALUE_CHANGED, label);
    }

    // LED2
    {
        lv_obj_t *row = create_row(cont);
        lv_obj_t *label = lv_label_create(row);
        lv_label_set_text(label, "LED2: OFF");
        lv_obj_set_flex_grow(label, 1);

        lv_obj_t *sw = lv_switch_create(row);
        apply_switch_style(sw);

        lv_obj_add_event_cb(sw, [](lv_event_t *e)
        {
            lv_obj_t *sw_ = lv_event_get_target_obj(e);
            lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
            bool on = lv_obj_has_state(sw_, LV_STATE_CHECKED);

            static led l2(LED2_GPIO_Port, LED2_Pin);
            if (on)
            {
                l2.on();
                lv_label_set_text(lbl, "LED2: ON");
            }
            else
            {
                l2.off();
                lv_label_set_text(lbl, "LED2: OFF");
            }
        }, LV_EVENT_VALUE_CHANGED, label);
    }

    // BEEP
    {
        lv_obj_t *row = create_row(cont);
        lv_obj_t *label = lv_label_create(row);
        lv_label_set_text(label, "BEEP: OFF");
        lv_obj_set_flex_grow(label, 1);

        lv_obj_t *sw = lv_switch_create(row);
        apply_switch_style(sw);

        lv_obj_add_event_cb(sw, [](lv_event_t *e)
        {
            lv_obj_t *sw_ = lv_event_get_target_obj(e);
            lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
            bool on = lv_obj_has_state(sw_, LV_STATE_CHECKED);

            if (on)
            {
                Beep::instance().on();
                lv_label_set_text(lbl, "BEEP: ON");
            }
            else
            {
                Beep::instance().off();
                lv_label_set_text(lbl, "BEEP: OFF");
            }
        }, LV_EVENT_VALUE_CHANGED, label);
    }

    // DHT11
    {
        lv_obj_t *row = create_row(cont);
        lv_obj_t *label = lv_label_create(row);
        lv_label_set_text(label, "DHT11: --\xC2\xB0""C  --%");
        lv_obj_set_flex_grow(label, 1);
        dht11_label = label;

        lv_obj_t *btn = lv_btn_create(row);
        lv_obj_set_size(btn, 50, 30);
        lv_obj_t *btn_lbl = lv_label_create(btn);
        lv_label_set_text(btn_lbl, LV_SYMBOL_REFRESH);
        lv_obj_center(btn_lbl);

        lv_obj_add_event_cb(btn, [](lv_event_t *e)
        {
            dht11_request = true;
        }, LV_EVENT_CLICKED, nullptr);
    }
    return cont;
}

// 请求下一页：只改标志位（key_task 在临界区里调用）
void ui_page_next(void)
{
    target_page = (target_page + 1) % UI_PAGE_NUM;
}

// 请求上一页：只改标志位（key_task 在临界区里调用）
void ui_page_prev(void)
{
    target_page = (target_page + UI_PAGE_NUM - 1) % UI_PAGE_NUM;
}

// 在 LVGL 任务里调用：目标页与当前页不同才切换（持有 lvgl_mutex）
void ui_page_apply(void)
{
    uint8_t t = target_page;
    if (t == current_page)
    {
        return;
    }
    current_page = t;

    for (uint8_t i = 0; i < UI_PAGE_NUM; i++)
    {
        if (!pages[i])
        {
            continue;
        }
        if (i == t)
        {
            lv_obj_remove_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

// 创建所有页面
void create_pages(lv_obj_t *parent)
{
    pages[0] = page1(parent);
    pages[1] = page2(parent);

    // 默认显示第一页，其余隐藏
    current_page = 0;
    target_page = 0;
    for (uint8_t i = 1; i < UI_PAGE_NUM; i++)
    {
        if (pages[i])
        {
            lv_obj_add_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}
