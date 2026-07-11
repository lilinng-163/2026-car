#pragma once

#include <cstdint>
#include "lvgl.h"

// 页面数量
static constexpr uint8_t UI_PAGE_NUM = 2;

// 创建所有页面（第一页 PID 参数，第二页 LED/蜂鸣器）
void create_pages(lv_obj_t *parent);

// 页面切换请求：key_task 在临界区里改标志位，LVGL 任务读标志位刷新
void ui_page_next(void);   // 请求下一页（page++）
void ui_page_prev(void);   // 请求上一页（page--）
void ui_page_apply(void);  // 在 LVGL 任务里调用：按标志位切换实际页面

// 刷新第一页的 6 个 PID 参数值
void pid_update_ui();

// 是否正在弹数字键盘编辑（编辑时 show_pv 暂停后台刷新）
bool ui_editing(void);
