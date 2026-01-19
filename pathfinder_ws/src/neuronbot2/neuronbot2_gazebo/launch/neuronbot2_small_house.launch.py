import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('aws_robomaker_small_house_world'),
        'worlds',
        'small_house.world'  # 실제 world 파일 이름 확인 필요
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'neuronbot2'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[os.path.join(
            get_package_share_directory('neuronbot2_description'),
            'urdf',
            'neuronbot2.urdf'
        )]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity
    ])

