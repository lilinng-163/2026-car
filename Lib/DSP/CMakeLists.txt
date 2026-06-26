cmake_minimum_required(VERSION 3.22)

set(FASTBUILD ON CACHE BOOL "" FORCE)
set(DISABLEFLOAT16 ON CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cmsis_dsp_sdk/Source ${CMAKE_CURRENT_BINARY_DIR}/cmsisdsp)

target_include_directories(CMSISDSP PUBLIC ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include)
target_compile_definitions(CMSISDSP PUBLIC ARM_MATH_CM4 ARM_MATH_LOOPUNROLL)
