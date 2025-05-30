#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    pkg_control = FindPackageShare('clearpath_control')

    # Launch Configurations
    enable_ekf = LaunchConfiguration('enable_ekf')
    # setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    arg_enable_ekf = DeclareLaunchArgument(
        'enable_ekf',
        default_value='true',
        choices=['true', 'false'],
        description='Enable localization via EKF node'
    )
    arg_setup_path = DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation time'
    )

    node_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_control, 'config', 'a200', 'localization.yaml'])],
        remappings=[
            ('odometry/filtered', 'platform/odom/filtered'),
            ('/diagnostics', 'diagnostics'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=IfCondition(enable_ekf),
    )

    return LaunchDescription([
        arg_enable_ekf,
        arg_use_sim_time,
        node_localization,
    ])
