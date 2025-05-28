#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Configurations
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    arg_rviz_config = DeclareLaunchArgument(
        name='config',
        default_value='robot.rviz'
    )

    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    config_rviz = PathJoinSubstitution([
        pkg_clearpath_viz, 'rviz', LaunchConfiguration('config')
    ])

    group_view_model = GroupAction([
        PushRosNamespace(namespace),
        Node(package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', config_rviz],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            output='screen')
    ])

    return LaunchDescription([
        arg_namespace,
        arg_rviz_config,
        arg_use_sim_time,
        group_view_model
    ])
