cmake_minimum_required(VERSION 3.22)

add_library(device_output OBJECT
    src/lcd.cpp
    src/led.cpp
    src/beep.cpp
    src/dht11.cpp
    src/motor.cpp
)

target_include_directories(device_output PUBLIC
    inc
)

target_link_libraries(device_output PUBLIC
    stm32cubemx
    lvgl
)
