#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    return LaunchDescription([
    # Empty launch node
        Node(
            package='ele434_team15_2026',
            executable='',
            name=''
        ),
    ])
