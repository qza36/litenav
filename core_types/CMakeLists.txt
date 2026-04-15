cmake_minimum_required(VERSION 3.16)
project(core_types LANGUAGES CXX)

add_library(core_types INTERFACE)

target_include_directories(core_types INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
