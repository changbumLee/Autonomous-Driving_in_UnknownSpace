from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_explorer',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[
                {'min_frontier_size': 0.05}
            ]
        )
    ])
