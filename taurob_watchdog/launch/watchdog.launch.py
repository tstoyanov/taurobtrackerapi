import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='taurob_watchdog',
            executable='taurob_watchdog_node',
            name='watchdog_node',
            output='screen',
            parameters=[{
                'supply_watchdog_after_coms_loss_secs': 90,
                'watchdog_detect_after_coms_loss_tolerance_secs': 30,
            }],
        ),
    ])
