#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Default map: test.yaml from the sibling map_module
    project_root = os.path.join(
        get_package_share_directory("litenav_ros2"), "..", "..", "..", ".."
    )
    default_map = os.path.join(
        project_root, "map_module", "maps", "test.yaml"
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value=default_map,
        description="Path to the YAML map metadata file",
    )

    rviz_config = os.path.join(
        get_package_share_directory("litenav_ros2"), "rviz", "litenav.rviz"
    )

    litenav_node = Node(
        package="litenav_ros2",
        executable="litenav_node",
        name="litenav_node",
        parameters=[{"map_yaml": LaunchConfiguration("map_yaml")}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([map_yaml_arg, litenav_node, rviz_node])
