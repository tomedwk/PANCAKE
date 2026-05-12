#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    environment = LaunchConfiguration('environment')

    launch_dir = os.path.join(
        get_package_share_directory('ele434_team15_2026'),
        'launch'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'environment',
            default_value='real',
            description="Set to 'real' or 'sim'"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    launch_dir,
                    'Mapping.launch.py'
                )
            ),
            launch_arguments={'environment': environment}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    launch_dir,
                    'FSM.launch.py'
                )
            )
        )
    ])