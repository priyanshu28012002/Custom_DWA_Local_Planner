#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    dwa_node = Node(
        package='custom_dwa_planner',
        executable='dwa_planner_node',
        output='screen'
    )

    return LaunchDescription([

       dwa_node
    ])