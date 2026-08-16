add_library(app OBJECT
    sensors/src/imu_task.cpp
    sensors/src/tracking_task.cpp
    control/src/pid_task.cpp
    control/src/servo_task.cpp
    control/src/vector_pid_task.cpp
    ui/src/oled_task.cpp
    comm/src/uart_cmd_task.cpp
    system/src/key_task.cpp
    system/src/led_task.cpp
)

target_include_directories(app PUBLIC
    common/inc
    sensors/inc
    control/inc
    ui/inc
    comm/inc
    system/inc
)

target_link_libraries(app PUBLIC
    FreeRTOS
    stm32cubemx
    device_input
    device_output
    etl
    pid
)
