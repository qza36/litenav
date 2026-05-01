cmake_minimum_required(VERSION 3.20)
project(litenav_ros2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Pull in existing litenav modules (same pattern as app_module)
if(NOT TARGET core_types)
  add_subdirectory(../core_types ${CMAKE_CURRENT_BINARY_DIR}/core_types_build)
endif()
if(NOT TARGET map_module)
  add_subdirectory(../map_module ${CMAKE_CURRENT_BINARY_DIR}/map_module_build)
endif()
if(NOT TARGET planning_module)
  add_subdirectory(../plan_module ${CMAKE_CURRENT_BINARY_DIR}/plan_module_build)
endif()
if(NOT TARGET control_module)
  add_subdirectory(../control_module ${CMAKE_CURRENT_BINARY_DIR}/control_module_build)
endif()

add_executable(litenav_node src/litenav_node.cpp)

ament_target_dependencies(litenav_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  visualization_msgs
)

target_link_libraries(litenav_node
  map_module
  planning_module
  control_module
)

install(TARGETS litenav_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
