cmake_minimum_required(VERSION 3.16)
project(map_module LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT TARGET core_types)
    add_subdirectory(../core_types ${CMAKE_CURRENT_BINARY_DIR}/core_types_build)
endif()

find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)

pkg_check_modules(GRAPHICSMAGICKPP QUIET IMPORTED_TARGET GraphicsMagick++)
pkg_check_modules(IMAGEMAGICKPP QUIET IMPORTED_TARGET Magick++)

if(GRAPHICSMAGICKPP_FOUND)
    set(MAGICK_TARGET PkgConfig::GRAPHICSMAGICKPP)
elseif(IMAGEMAGICKPP_FOUND)
    set(MAGICK_TARGET PkgConfig::IMAGEMAGICKPP)
else()
    message(FATAL_ERROR "Could not find GraphicsMagick++ or ImageMagick Magick++ via pkg-config.")
endif()

add_library(map_module src/map_io.cpp)

target_link_libraries(map_module PUBLIC
    yaml-cpp::yaml-cpp
    core_types
)

target_link_libraries(map_module PRIVATE
    ${MAGICK_TARGET}
)

target_include_directories(map_module PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
