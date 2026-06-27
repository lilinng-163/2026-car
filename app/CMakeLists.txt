add_library(app STATIC
    src/lvgl_task.cpp
    src/led_task.cpp
    src/lvgl_mutex.cpp
    src/show_pv.cpp
)

target_include_directories(app PUBLIC
    inc
)

target_link_libraries(app PUBLIC
    lvgl
    FreeRTOS
    stm32cubemx
)
