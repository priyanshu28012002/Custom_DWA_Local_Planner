#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    costmap = Node(
        package='custom_dwa_planner',
        executable='costmap_node',
        output='screen'
    )
    global_panner = Node(
        package='custom_dwa_planner',
        executable='global_path_planner_node',
        output='screen'
    )


    return LaunchDescription([
        costmap,
        global_panner
  
    ])