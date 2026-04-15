cmake_minimum_required(VERSION 3.16)
project(planning_module LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT TARGET core_types)
    add_subdirectory(../core_types ${CMAKE_CURRENT_BINARY_DIR}/core_types_build)
endif()

add_library(planning_module
    src/planning.cpp
)

target_link_libraries(planning_module
    PUBLIC
    core_types
)
target_include_directories(planning_module PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
