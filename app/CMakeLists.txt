add_library(app OBJECT
    src/led_task.cpp
    src/servo_task.cpp
    src/vector_pid_task.cpp
    src/key_task.cpp
    src/oled_task.cpp
    src/imu_task.cpp
    src/tracking_task.cpp
    src/uart_isr.cpp
)

target_include_directories(app PUBLIC
    inc
)

target_link_libraries(app PUBLIC
    FreeRTOS
    stm32cubemx
    device_input
    device_output
    etl
    pid
)
