cmake_minimum_required(VERSION 3.22)

file(GLOB FreeRTOS_Sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

set(FreeRTOS_Portable_Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/portable/port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/portable/heap_4.c
)

set(FreeRTOS_Include_Dir
    ${CMAKE_CURRENT_SOURCE_DIR}/portable
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/FreeRTOS)

add_library(FreeRTOS STATIC)
target_sources(FreeRTOS PRIVATE ${FreeRTOS_Sources} ${FreeRTOS_Portable_Sources})
target_include_directories(FreeRTOS PUBLIC ${FreeRTOS_Include_Dir})