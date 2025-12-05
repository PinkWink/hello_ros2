#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 1) turtlesim 기본 노드 실행
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # 2) turtle1을 원호로 움직이는 node
        Node(
            package='hello_ros2',
            executable='turtle_circle',
            name='turtle_circle'
        ),

        # 3) pose를 실시간으로 모니터링하는 matplotlib 노드
        Node(
            package='hello_ros2',
            executable='turtle_pose_live_monitor',
            name='turtle_pose_live_monitor',
            output='screen'
        ),

    ])
