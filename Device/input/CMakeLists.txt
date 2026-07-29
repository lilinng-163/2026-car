cmake_minimum_required(VERSION 3.22)

add_library(device_input OBJECT
    src/encoder.cpp
    src/key.cpp
    src/imu.cpp
    src/tracking.cpp
)

target_include_directories(device_input PUBLIC
    inc
)

target_link_libraries(device_input PUBLIC
    stm32cubemx
    etl
    FreeRTOS
    soft_drivers
)
