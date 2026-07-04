cmake_minimum_required(VERSION 3.22)

add_library(device_input OBJECT
    src/xpt2046.cpp
    src/lv_port_touch.cpp
    src/encoder.cpp
)

target_include_directories(device_input PUBLIC
    inc
)

target_link_libraries(device_input PUBLIC
    stm32cubemx
    lvgl
)
