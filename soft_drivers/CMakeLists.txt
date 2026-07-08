cmake_minimum_required(VERSION 3.22)

add_library(soft_drivers OBJECT
    src/soft_i2c.cpp
)

target_include_directories(soft_drivers PUBLIC
    inc
)

target_link_libraries(soft_drivers PUBLIC
    stm32cubemx
)
