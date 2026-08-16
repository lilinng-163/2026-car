cmake_minimum_required(VERSION 3.22)

add_library(pid STATIC
    src/pid.cpp
)

target_include_directories(pid PUBLIC
    inc
)

target_link_libraries(pid PUBLIC
    CMSISDSP
)
