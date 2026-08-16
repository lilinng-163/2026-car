#pragma once

#include "tracking.h"

// ===== 循迹任务 (tracking) 共享接口 =====
// track       循迹协议解析器实例
// middle      8 路传感器对称中心索引(0~7 中心=3.5)
// pos         黑色线重心位置(0~7，加权重心)
// err         重心相对中心的偏差 (pos - middle)，供巡线外环使用

extern tracking track;
constexpr float middle = 3.5f;
extern volatile float pos;
extern volatile float err;

void tracking_task_create(void);
