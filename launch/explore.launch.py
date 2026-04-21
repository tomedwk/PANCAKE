from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'ele434_team15_2026',
            executable='',
            name=''
        )
    ])

# Use ros2 launch ele434_team15_2026 explore.launch.py