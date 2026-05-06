#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_dir = os.path.join(
        get_package_share_directory('ele434_team15_2026'),
        'launch'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    launch_dir,
                    'Mapping.launch.py'
                )
            )
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
