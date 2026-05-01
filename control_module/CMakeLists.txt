cmake_minimum_required(VERSION 3.16)
project(control_module LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT TARGET core_types)
    add_subdirectory(../core_types ${CMAKE_CURRENT_BINARY_DIR}/core_types_build)
endif()

add_library(control_module
    src/pid_path_follower.cpp
)

target_link_libraries(control_module
    PUBLIC
    core_types
)
target_include_directories(control_module PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
