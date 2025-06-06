#!/usr/bin/env python3

import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from clearpath_config.clearpath_config import ClearpathConfig


def launch_setup(context, *args, **kwargs):
    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # setup_path_context = setup_path.perform(context)

    # namespace = ClearpathConfig(os.path.join(setup_path_context, 'robot.yaml')).get_namespace()

    # Robot Description
    robot_description = {
        'robot_description': xacro.process_file(
            '/etc/clearpath/robot.urdf.xacro'
        ).toxml()
    }

    # Semantic Robot Description
    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            '/etc/clearpath/robot.srdf'
        ).toxml()
    }

    return [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='log',
            namespace='husky',
            parameters=[
                '/etc/clearpath/manipulators/config/moveit.yaml',
                robot_description,
                robot_description_semantic,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states'),
            ]
        )
    ]


def generate_launch_description():
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/',
        description='Clearpath setup path'
    )
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    )
    return LaunchDescription([
        arg_setup_path,
        arg_use_sim_time,
        OpaqueFunction(function=launch_setup)
    ])
