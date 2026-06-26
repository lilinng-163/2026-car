cmake_minimum_required(VERSION 3.22)

set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/etl_sdk ${CMAKE_CURRENT_BINARY_DIR}/etl)
