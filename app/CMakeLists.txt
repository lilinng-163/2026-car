add_library(app OBJECT
    src/lvgl_task.cpp
    src/led_task.cpp
    src/mutex.cpp
    src/show_pv.cpp
    src/lv_obj.cpp
    src/motor_task.cpp
    src/servo_task.cpp
    src/key_task.cpp
)

target_include_directories(app PUBLIC
    inc
)

target_link_libraries(app PUBLIC
    lvgl
    FreeRTOS
    stm32cubemx
    device_input
    device_output
    pid
)
