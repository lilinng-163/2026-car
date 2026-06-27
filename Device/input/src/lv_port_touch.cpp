#include "lv_port_touch.h"
#include "xpt2046.h"

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    XPT2046_Scan(0);

    if (Xdown != 0xFFFF && Ydown != 0xFFFF) {
        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = Xdown;
        data->point.y = Ydown;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void lv_port_touch_init(void) {
    XPT2046_Init();

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);
}
