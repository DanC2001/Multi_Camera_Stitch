#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # argument to pass the bag file.
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/workspaces/Abyss_Solutions_Test_DanielCook/bag_files/case_study/case_study.db3',
        description='Path to the recorded ROS bag file'
    )

    # Argument to pass the perspective file
    perspective_arg = DeclareLaunchArgument(
        'perspective_file',
        default_value='/workspaces/Abyss_Solutions_Test_DanielCook/dc_abyss_solutions_test/perspectives/ImageView.perspective',
        description='Path to an rqt perspective file to load'
    )

    # Play the bag file
    bag_play_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--loop'
        ],
        name='bag_player',
        output='screen'
    )

    # Launch the node
    multi_camera_node = Node(
        package='dc_abyss_solutions_test',
        executable='multi_camera_node',
        name='multi_camera_node',
        output='screen',
        remappings=[
            ('/camera_1/image', '/platypus/camera_1/dec/manual_white_balance'),
            ('/camera_2/image', '/platypus/camera_2/dec/manual_white_balance'),
            ('/camera_3/image', '/platypus/camera_3/dec/manual_white_balance'),
        ]
    )

    # rqt with a perspective file
    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_gui',
        output='screen',
        arguments=[
            '--perspective-file',
            LaunchConfiguration('perspective_file')
        ]
    )

    return LaunchDescription([
        bag_path_arg,
        perspective_arg,
        bag_play_process,
        multi_camera_node,
        rqt_node
    ])
