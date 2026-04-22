#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # Launch argument: real or sim
    environment = LaunchConfiguration('environment')

    # Path to your existing SLAM launch file
    slam_launch_path = os.path.join(
        get_package_share_directory('ele434_team15_2026'),
        'launch',
        'slam.launch.py'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'environment',
            default_value='real',
            description="Set to 'real' or 'sim'"
        ),

        # Start SLAM (this also launches RViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={'environment': environment}.items()
        ),

        # Start your map saver node
        Node(
            package='ele434_team15_2026',
            executable='Periodic_Map_Saver.py',
            name='map_saver',
            output='screen'
        ),
    ])