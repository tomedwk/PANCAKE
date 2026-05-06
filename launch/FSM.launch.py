#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    manager_node = Node(
        package='ele434_team15_2026',
        executable='Manager.py',
        name='manager_node',
        output='screen'
    )

    waypoint_node = Node(
        package='ele434_team15_2026',
        executable='waypoint.py',
        name='waypoint_node',
        output='screen'
    )

    return LaunchDescription([
        manager_node,
        waypoint_node
    ])