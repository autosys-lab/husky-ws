#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    pkg_viz = FindPackageShare('clearpath_viz')

    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = LaunchConfiguration('config')

    # Launch Arguments
    arg_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    arg_rviz_config = DeclareLaunchArgument('config', default_value='nav2.rviz')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    group_view_model = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_viz, 'rviz', config])],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
        )
    ])

    return LaunchDescription([
        arg_namespace,
        arg_rviz_config,
        arg_use_sim_time,
        group_view_model,
    ])
