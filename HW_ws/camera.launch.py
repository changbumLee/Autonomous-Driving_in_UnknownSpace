#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    # PX4-Autopilot 디렉터리에서 명령 실행
    
    workspace_directory = str(Path.home() / "final_ws/main_ws")
    yolo_directory = str(Path(workspace_directory) / "yolo_ros/yolo_bringup/launch")
    cmd_directory = str(Path(workspace_directory) / "cmd")  # 경로 수정 가능
    config_file = os.path.join(workspace_directory, 'ros_gz_bridge/config')
    px4_sensor_directory = str(Path(workspace_directory) / 'src'/'px4_sensor'/'px4_sensor')



    return LaunchDescription([


        # camer launch cmd
        ExecuteProcess(
            cmd=["ros2", "run", "yahboom_esp32_camera", "sub_img"],
            cwd=str(workspace_directory)
        ),
        # yolo detection cmd
        ExecuteProcess(
            cmd=["ros2", "launch", "yolo_bringup", "yolov8.launch.py"],
            cwd=str(workspace_directory),
            output="screen"
        ),

        ExecuteProcess(
            cmd=["rviz2"],
            output="screen"
        ),

        # ExecuteProcess(
        #     cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", "config_file:=GZ_to_ROS.yaml"],
        #     output="screen",
        #     cwd=config_file
        # ),
    ])
