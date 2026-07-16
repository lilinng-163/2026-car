#pragma once

#include <cstdint>

// OLED 调参 UI 共享状态 (key_task 写, oled_task 读)
inline constexpr uint8_t TUNE_NUM_PAGES = 4;   // 总页数

// 页0可调参数索引 (SP)
inline constexpr uint8_t TUNE_SP_L = 0;
inline constexpr uint8_t TUNE_SP_R = 1;
inline constexpr uint8_t TUNE_P0_PARAMS = 2;

// 页1可调参数索引 (双电机 PID 增益)
inline constexpr uint8_t TUNE_L_KP = 0;
inline constexpr uint8_t TUNE_L_KI = 1;
inline constexpr uint8_t TUNE_L_KD = 2;
inline constexpr uint8_t TUNE_R_KP = 3;
inline constexpr uint8_t TUNE_R_KI = 4;
inline constexpr uint8_t TUNE_R_KD = 5;
inline constexpr uint8_t TUNE_P1_PARAMS = 6;

// 页2可调参数索引 (STEER_KP / STEER_KD / GZ_K)
inline constexpr uint8_t TUNE_T_KP = 0;
inline constexpr uint8_t TUNE_T_KD = 1;
inline constexpr uint8_t TUNE_GZ_K = 2;
inline constexpr uint8_t TUNE_P2_PARAMS = 3;

// 页3可调参数索引 (YAW_KP / YAW_KI / YAW_KD / YAW_GAIN)
inline constexpr uint8_t TUNE_Y_KP   = 0;
inline constexpr uint8_t TUNE_Y_KI   = 1;
inline constexpr uint8_t TUNE_Y_KD   = 2;
inline constexpr uint8_t TUNE_Y_GAIN = 3;
inline constexpr uint8_t TUNE_P3_PARAMS = 4;

extern volatile uint8_t tune_page;     // 当前页 (k3 切换)
extern volatile uint8_t tune_select;   // 当前页内选中的参数 (k1 切换)
