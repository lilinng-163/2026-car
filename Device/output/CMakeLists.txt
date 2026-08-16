cmake_minimum_required(VERSION 3.22)

add_library(device_output OBJECT
    src/led.cpp
    src/beep.cpp
    src/motor.cpp
    src/servo.cpp
    src/stepper.cpp
    src/oled.cpp
)

target_include_directories(device_output PUBLIC
    inc
)

target_link_libraries(device_output PUBLIC
    stm32cubemx
    soft_drivers
)
