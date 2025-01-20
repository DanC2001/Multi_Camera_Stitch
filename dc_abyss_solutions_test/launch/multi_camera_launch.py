# multi_camera_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    camera_file_arg = DeclareLaunchArgument(
        'camera_file',
        default_value='/workspaces/Abyss_Solutions_Test_DanielCook/dc_abyss_solutions_test/intrinsics_extrinsics.json',
        description='Path to JSON file with camera intrinsics/extrinsics'
    )

    multi_cam_node = Node(
        package='dc_abyss_solutions_test',
        executable='multi_camera_node',
        name='multi_camera_node',
        output='screen',
        parameters=[{
            'intrinsics_extrinsics_file': LaunchConfiguration('camera_file')
        }],
    )

    return LaunchDescription([
        camera_file_arg,
        multi_cam_node
    ])