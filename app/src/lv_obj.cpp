#include <cstdio>
#include <cstdlib>
#include "lvgl.h"
#include "lv_obj.h"
#include "vector_pid_task.h"
#include "beep.h"
#include "led.h"
#include "main.h"

// 每个电机 6 个值 label：Kp/Ki/Kd/RPM/ERR/OUT
// [0..5]=左，[6..11]=右
static lv_obj_t *pid_labels[12] = {nullptr};

// 页面容器 + 当前实际显示页 + 目标页标志位
static lv_obj_t *pages[UI_PAGE_NUM] = {nullptr};
static uint8_t current_page = 0;
static volatile uint8_t target_page = 0;

// 刷新 12 个值（定点整数运算，避免浮点 printf；保留正负号）
void pid_update_ui()
{
    if (!pid_labels[0])
    {
        return;
    }
    static char buf[16];

    arm_pid_instance_f32 l = left_motor_pid.get_instance();
    arm_pid_instance_f32 r = right_motor_pid.get_instance();

    // 每个电机: Kp, Ki, Kd, RPM, ERR, OUT
    const float32_t vals[12] = {
        l.Kp, l.Ki, l.Kd, get_left_actual_rpm(),  l.state[0], l.state[2],
        r.Kp, r.Ki, r.Kd, get_right_actual_rpm(), r.state[0], r.state[2]
    };

    for (int i = 0; i < 12; i++)
    {
        // 定点格式化：×1000 转成整数，避免用浮点 printf(体积/性能)
        // 例 -0.5 -> neg=true, scaled=500 -> "-0.500"，单独留符号防丢负号
        float32_t v = vals[i];
        bool neg = (v < 0.0f);
        int scaled = (int)(v * 1000.0f + (neg ? -0.5f : 0.5f));  // 四舍五入
        if (scaled < 0)
        {
            scaled = -scaled;    // 取绝对值，符号交给 neg
        }
        int ip = scaled / 1000;  // 整数部分
        int fp = scaled % 1000;  // 3 位小数部分
        std::snprintf(buf, sizeof(buf), "%s%d.%03d", neg ? "-" : "", ip, fp);
        lv_label_set_text(pid_labels[i], buf);
    }
}

// 创建一个表格行容器（统一样式 + 斑马纹）
static lv_obj_t *make_row(lv_obj_t *parent, bool zebra)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_ver(row, 3, 0);
    lv_obj_set_style_pad_hor(row, 8, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_radius(row, 4, 0);
    lv_obj_set_style_bg_color(row, zebra ? lv_color_hex(0x334155)
                                         : lv_color_hex(0x1e293b), 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
    return row;
}

// 只读表格行：3 个等宽单元格 [名称 | 左值 | 右值]
// left_slot/right_slot 传非空指针时该格是“数值格”(白字，*slot 回填 label 供刷新)
// 传 nullptr 时该格是“表头格”(显示 Left/Right 彩色文字)
static void table_row(lv_obj_t *parent, const char *name,
                      lv_obj_t **left_slot, lv_obj_t **right_slot,
                      lv_color_t name_color, bool zebra)
{
    lv_obj_t *row = make_row(parent, zebra);

    lv_obj_t *c0 = lv_label_create(row);
    lv_label_set_text(c0, name);
    lv_obj_set_flex_grow(c0, 1);
    lv_obj_set_style_text_color(c0, name_color, 0);

    lv_obj_t *c1 = lv_label_create(row);
    lv_label_set_text(c1, left_slot ? "--" : "Left");
    lv_obj_set_flex_grow(c1, 1);
    lv_obj_set_style_text_align(c1, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(c1, left_slot ? lv_color_hex(0xffffff)
                                              : lv_color_hex(0x22c55e), 0);

    lv_obj_t *c2 = lv_label_create(row);
    lv_label_set_text(c2, right_slot ? "--" : "Right");
    lv_obj_set_flex_grow(c2, 1);
    lv_obj_set_style_text_align(c2, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(c2, right_slot ? lv_color_hex(0xffffff)
                                               : lv_color_hex(0x3b82f6), 0);

    if (left_slot)  *left_slot  = c1;
    if (right_slot) *right_slot = c2;
}

// ============================================================================
//  触屏编辑 Kp/Ki/Kd 功能
//  流程：点击黄色数值 -> value_click_cb 弹出数字键盘 -> 输入 ->
//        键盘按 OK 触发 kb_event_cb(READY) -> editor_apply 写回真实 PID
//        键盘按 X  触发 kb_event_cb(CANCEL) -> editor_close 只关闭不改
// ============================================================================

// 一个可编辑数值“记住”它对应哪个 PID、哪个增益(0=Kp,1=Ki,2=Kd)
// 通过 lv 事件的 user_data 传给回调，回调据此知道要改谁
struct gain_target
{
    pid *p;          // 指向真实 PID 对象(&left_motor_pid / &right_motor_pid)
    uint8_t which;   // 0=Kp 1=Ki 2=Kd
};
static gain_target g_targets[6];   // 6 个可编辑格子(左右各 3)的目标描述
static int g_target_n = 0;         // 已分配个数

// 数字键盘弹窗的运行时状态(同一时刻只有一个弹窗)
static lv_obj_t *kb_modal  = nullptr;   // 全屏遮罩(承载输入框+键盘)
static lv_obj_t *kb_ta     = nullptr;   // 输入框
static gain_target *kb_target = nullptr;// 当前正在编辑的目标

// 是否正在弹键盘编辑 —— 供 show_pv 任务查询，编辑时暂停后台刷新避免抢重绘
bool ui_editing(void)
{
    return kb_modal != nullptr;
}

// 关闭弹窗：删掉整个遮罩(连带输入框、键盘)并清空状态
static void editor_close(void)
{
    if (kb_modal)
    {
        lv_obj_del(kb_modal);
        kb_modal = nullptr;
        kb_ta = nullptr;
        kb_target = nullptr;
    }
}

// 按 OK 后调用：把输入框文本解析成数值，写回目标 PID 的对应增益
static void editor_apply(void)
{
    if (!kb_target || !kb_ta)
    {
        return;
    }
    // 1. 取输入框文本 -> float，负值夹到 0
    const char *txt = lv_textarea_get_text(kb_ta);
    float32_t v = strtof(txt, nullptr);
    if (v < 0.0f)
    {
        v = 0.0f;
    }

    // 2. 先读回当前 3 个增益，只替换被编辑的那一个(其余保持不变)
    arm_pid_instance_f32 inst = kb_target->p->get_instance();
    float32_t kp = inst.Kp, ki = inst.Ki, kd = inst.Kd;

    if (kb_target->which == 0)      kp = v;
    else if (kb_target->which == 1) ki = v;
    else                            kd = v;

    // 3. 写回真实 PID -> 下一控制周期 motor_task 立即用新参数
    kb_target->p->set_gains(kp, ki, kd);
    pid_update_ui();   // 立刻刷新界面数值
}

// 键盘事件：OK(READY)=确认写回并关闭；X(CANCEL)=只关闭
static void kb_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)
    {
        editor_apply();
        editor_close();
    }
    else if (code == LV_EVENT_CANCEL)
    {
        editor_close();
    }
}

// 点击某个数值 label -> 弹出数字键盘编辑该值
static void value_click_cb(lv_event_t *e)
{
    if (kb_modal)   // 已有弹窗则忽略，防重复
    {
        return;
    }
    // user_data 里就是这个格子的 gain_target，记下当前编辑目标
    kb_target = static_cast<gain_target *>(lv_event_get_user_data(e));
    lv_obj_t *lbl = lv_event_get_target_obj(e);   // 被点的数值 label

    // 全屏遮罩层：放在最顶层，盖住整页
    // 用不透明背景(非半透明)——半透明会让键盘每次重绘都重新合成下层，很卡
    kb_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(kb_modal, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_border_width(kb_modal, 0, 0);
    lv_obj_set_style_radius(kb_modal, 0, 0);
    lv_obj_set_style_pad_all(kb_modal, 0, 0);
    lv_obj_set_style_bg_color(kb_modal, lv_color_hex(0x0f172a), 0);
    lv_obj_set_style_bg_opa(kb_modal, LV_OPA_COVER, 0);
    lv_obj_remove_flag(kb_modal, LV_OBJ_FLAG_SCROLLABLE);

    // 输入框：预填当前显示值，方便在原值基础上改
    kb_ta = lv_textarea_create(kb_modal);
    lv_textarea_set_one_line(kb_ta, true);
    lv_textarea_set_text(kb_ta, lv_label_get_text(lbl));
    lv_obj_set_width(kb_ta, LV_PCT(80));
    lv_obj_align(kb_ta, LV_ALIGN_TOP_MID, 0, 10);

    // 数字键盘：NUMBER 模式自带数字/小数点和 OK、X 按钮
    lv_obj_t *kb = lv_keyboard_create(kb_modal);
    lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(kb, kb_ta);   // 键盘输入自动进 kb_ta
    lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_READY,  nullptr);  // OK
    lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_CANCEL, nullptr);  // X
}

// 创建一个“可编辑数值 label”：黄色带下划线、点击弹键盘
// 返回 label 指针，供 pid_labels[] 保存以便周期刷新数值
static lv_obj_t *make_editable_value(lv_obj_t *row, pid *p, uint8_t which)
{
    // 分配并填好这个格子的目标描述(改谁的哪个增益)
    gain_target *t = &g_targets[g_target_n++];
    t->p = p;
    t->which = which;

    lv_obj_t *val = lv_label_create(row);
    lv_label_set_text(val, "--");
    lv_obj_set_flex_grow(val, 1);
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_pad_ver(val, 4, 0);
    // 黄色 + 下划线，暗示“可点击编辑”
    lv_obj_set_style_text_color(val, lv_color_hex(0xfacc15), 0);
    lv_obj_set_style_text_decor(val, LV_TEXT_DECOR_UNDERLINE, 0);
    // label 默认不可点，这里开启点击并绑定回调(user_data 传 t)
    lv_obj_add_flag(val, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(val, value_click_cb, LV_EVENT_CLICKED, t);
    return val;
}

// 可编辑增益行：名称 | 左值(可点) | 右值(可点)
// 左值存入 pid_labels[which]，右值存入 pid_labels[6+which]
static void gain_row(lv_obj_t *parent, const char *name, uint8_t which, bool zebra)
{
    lv_obj_t *row = make_row(parent, zebra);

    lv_obj_t *c0 = lv_label_create(row);
    lv_label_set_text(c0, name);
    lv_obj_set_flex_grow(c0, 1);
    lv_obj_set_style_text_color(c0, lv_color_hex(0xcbd5e1), 0);

    pid_labels[which]     = make_editable_value(row, &left_motor_pid,  which);
    pid_labels[6 + which] = make_editable_value(row, &right_motor_pid, which);
}

// 第一页：一张表显示两个电机的 Kp/Ki/Kd/RPM/ERR/OUT
static lv_obj_t *page1(lv_obj_t *parent)
{
    static const char *keys[6] = { "Kp", "Ki", "Kd", "RPM", "ERR", "OUT" };

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(cont, 6, 0);
    lv_obj_set_style_pad_row(cont, 2, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x0f172a), 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);

    // 页面标题
    lv_obj_t *title = lv_label_create(cont);
    lv_label_set_text(title, "PID Monitor");
    lv_obj_set_style_text_color(title, lv_color_hex(0xf1f5f9), 0);

    // 表头：名称 | Left | Right
    table_row(cont, "", nullptr, nullptr, lv_color_hex(0x94a3b8), false);

    // 前 3 行可编辑增益（点击弹键盘），后 3 行只读
    for (int i = 0; i < 6; i++)
    {
        bool zebra = (i & 1) != 0;
        if (i < 3)
        {
            gain_row(cont, keys[i], static_cast<uint8_t>(i), zebra);
        }
        else
        {
            table_row(cont, keys[i], &pid_labels[i], &pid_labels[6 + i],
                      lv_color_hex(0xcbd5e1), zebra);
        }
    }

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
