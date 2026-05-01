cmake_minimum_required(VERSION 3.16)
project(app_module LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GRAPHICSMAGICKPP QUIET IMPORTED_TARGET GraphicsMagick++)
pkg_check_modules(IMAGEMAGICKPP QUIET IMPORTED_TARGET Magick++)

if(GRAPHICSMAGICKPP_FOUND)
    set(MAGICK_TARGET PkgConfig::GRAPHICSMAGICKPP)
elseif(IMAGEMAGICKPP_FOUND)
    set(MAGICK_TARGET PkgConfig::IMAGEMAGICKPP)
else()
    message(FATAL_ERROR "Could not find GraphicsMagick++ or ImageMagick Magick++ via pkg-config.")
endif()

add_subdirectory(../map_module ${CMAKE_CURRENT_BINARY_DIR}/map_module_build)
add_subdirectory(../plan_module ${CMAKE_CURRENT_BINARY_DIR}/plan_module_build)
add_subdirectory(../control_module ${CMAKE_CURRENT_BINARY_DIR}/control_module_build)

add_library(app_module
    src/navigation_app.cpp
    src/app_visualization.cpp
)

target_include_directories(app_module PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(app_module PUBLIC
    planning_module
    control_module
)

target_link_libraries(app_module PRIVATE
    map_module
    ${MAGICK_TARGET}
)

add_executable(app_module_demo
    src/main.cpp
)

target_link_libraries(app_module_demo PRIVATE app_module)
