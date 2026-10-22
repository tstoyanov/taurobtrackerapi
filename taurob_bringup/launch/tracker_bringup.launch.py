import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'watchdog_enabled',
            default_value='true',
            description='Gate all actuation on the safety watchdog.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_ip',
            default_value='10.0.0.2',
            description='IP address of the robot base platform ECU.'))

    watchdog_enabled = LaunchConfiguration('watchdog_enabled')
    base_ip = LaunchConfiguration('base_ip')

    taurob_description_share = get_package_share_directory('taurob_tracker_description')
    bringup_share = get_package_share_directory('taurob_bringup')

    xacro_file = os.path.join(taurob_description_share, 'urdf', 'tracker.urdf.xacro')
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' watchdog_enabled:=', watchdog_enabled,
            ' base_ip:=', base_ip,
        ])
    robot_description = {'robot_description': robot_description_content}

    controllers_file = os.path.join(bringup_share, 'config', 'tracker_controllers.yaml')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='both',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    flipper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['flipper_controller', '--controller-manager', '/controller_manager'],
    )

    claw_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['claw_controller', '--controller-manager', '/controller_manager'],
    )

    watchdog_node = Node(
        package='taurob_watchdog',
        executable='taurob_watchdog_node',
        output='both',
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            control_node,
            watchdog_node,
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
            arm_spawner,
            flipper_spawner,
            claw_spawner,
        ])
