#include <cstdio>
#include "lvgl.h"
#include "beep.h"
#include "dht11.h"
#include "led.h"
#include "main.h"
#include "lv_obj.h"

// ---------------------------------------------------------------------------
// DHT11 shared state
// ---------------------------------------------------------------------------
volatile bool dht11_request = false;
static lv_obj_t *dht11_label = nullptr;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void apply_switch_style(lv_obj_t *sw)
{
    lv_obj_set_size(sw, 60, 30);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xef4444), LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0x22c55e),
        static_cast<uint32_t>(LV_PART_INDICATOR) | static_cast<uint32_t>(LV_STATE_CHECKED));
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xffffff), LV_PART_KNOB);
}

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

// ---------------------------------------------------------------------------
// DHT11 UI update (called from show_pv task)
// ---------------------------------------------------------------------------
void dht11_update_ui()
{
    if (!dht11_label) return;
    static char buf[32];

    if (Dht11::instance().read()) {
        std::snprintf(buf, sizeof(buf), "DHT11: %d°C  %d%%",
            Dht11::instance().get_temperature(),
            Dht11::instance().get_humidity());
    } else {
        std::snprintf(buf, sizeof(buf), "DHT11: ERR  ERR");
    }
    lv_label_set_text(dht11_label, buf);
}

// ---------------------------------------------------------------------------
// Content (page 1)
// ---------------------------------------------------------------------------
static void create_content(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(cont, 10, 0);
    lv_obj_set_style_pad_row(cont, 4, 0);
    lv_obj_set_style_pad_bottom(cont, 48, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);

    // LED1
    {
        lv_obj_t *row = create_row(cont);
        lv_obj_t *label = lv_label_create(row);
        lv_label_set_text(label, "LED1: OFF");
        lv_obj_set_flex_grow(label, 1);

        lv_obj_t *sw = lv_switch_create(row);
        apply_switch_style(sw);

        lv_obj_add_event_cb(sw, [](lv_event_t *e) {
            lv_obj_t *sw_ = lv_event_get_target_obj(e);
            lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
            bool on = lv_obj_has_state(sw_, LV_STATE_CHECKED);

            static led l1(LED1_GPIO_Port, LED1_Pin);
            if (on) { l1.on();  lv_label_set_text(lbl, "LED1: ON");  }
            else    { l1.off(); lv_label_set_text(lbl, "LED1: OFF"); }
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

        lv_obj_add_event_cb(sw, [](lv_event_t *e) {
            lv_obj_t *sw_ = lv_event_get_target_obj(e);
            lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
            bool on = lv_obj_has_state(sw_, LV_STATE_CHECKED);

            static led l2(LED2_GPIO_Port, LED2_Pin);
            if (on) { l2.on();  lv_label_set_text(lbl, "LED2: ON");  }
            else    { l2.off(); lv_label_set_text(lbl, "LED2: OFF"); }
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

        lv_obj_add_event_cb(sw, [](lv_event_t *e) {
            lv_obj_t *sw_ = lv_event_get_target_obj(e);
            lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
            bool on = lv_obj_has_state(sw_, LV_STATE_CHECKED);

            if (on) { Beep::instance().on();  lv_label_set_text(lbl, "BEEP: ON");  }
            else    { Beep::instance().off(); lv_label_set_text(lbl, "BEEP: OFF"); }
        }, LV_EVENT_VALUE_CHANGED, label);
    }

    // DHT11
    {
        lv_obj_t *row = create_row(cont);
        lv_obj_t *label = lv_label_create(row);
        lv_label_set_text(label, "DHT11: --°C  --%");
        lv_obj_set_flex_grow(label, 1);
        dht11_label = label;

        lv_obj_t *btn = lv_btn_create(row);
        lv_obj_set_size(btn, 50, 30);
        lv_obj_t *btn_lbl = lv_label_create(btn);
        lv_label_set_text(btn_lbl, LV_SYMBOL_REFRESH);
        lv_obj_center(btn_lbl);

        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            dht11_request = true;
        }, LV_EVENT_CLICKED, nullptr);
    }
}

// ---------------------------------------------------------------------------
// Bottom nav bar
// ---------------------------------------------------------------------------
static constexpr int PAGE_NUM = 4;

void create_pages(lv_obj_t *parent)
{
    create_content(parent);

    lv_obj_t *nav = lv_obj_create(parent);
    lv_obj_set_size(nav, LV_PCT(100), 44);
    lv_obj_align(nav, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(nav, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(nav, 8, 0);
    lv_obj_set_style_border_width(nav, 0, 0);

    for (int i = 0; i < PAGE_NUM; i++) {
        lv_obj_t *btn = lv_btn_create(nav);
        lv_obj_set_size(btn, 36, 36);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text_fmt(lbl, "%d", i + 1);
        lv_obj_center(lbl);

        if (i == 0) lv_obj_add_state(btn, LV_STATE_CHECKED);
    }
}
