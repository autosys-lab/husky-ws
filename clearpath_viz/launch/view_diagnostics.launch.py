#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Launch Configurations
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    arg_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')

    group_view_model = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='rqt_robot_monitor',
            executable='rqt_robot_monitor',
            output='screen',
            remappings=[
                ('/diagnostics', 'diagnostics'),
                ('/diagnostics_agg', 'diagnostics_agg')
            ],
        )
    ])

    return LaunchDescription([
        arg_namespace,
        group_view_model
    ])
