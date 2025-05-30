#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from clearpath_config.common.utils.dictionary import unflatten_dict
from clearpath_config.common.utils.yaml import read_yaml


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    # setup_path = LaunchConfiguration('setup_path')

    pkg_clearpath_control = FindPackageShare('clearpath_manipulators')

    # Controllers
    config_control = PathJoinSubstitution([pkg_clearpath_control, 'config', 'control.yaml'])

    context_control = unflatten_dict(read_yaml(config_control.perform(context)))

    controllers = [
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_control],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            remappings=[
                ('~/robot_description', 'robot_description'),
                ('dynamic_joint_states', PathJoinSubstitution(['/', namespace, 'platform', 'dynamic_joint_states'])),
                ('joint_states', PathJoinSubstitution(['/', namespace, 'platform', 'joint_states'])),
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager-timeout',
                '60',
            ],
            output='screen',
        )
    ]

    # If Simulation, Add All Listed Controllers
    for namespace in context_control:
        for controller in context_control[namespace]:
            if ('controller' not in controller or
                'manager' in controller or
                'platform' in controller):
                continue
            controllers.append(Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller,
                    '--controller-manager-timeout',
                    '60',
                ],
                output='screen',
            ))
    return [GroupAction(controllers)]


def generate_launch_description():
    # Launch Configurations
    arg_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    arg_setup_path = DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/')

    return LaunchDescription([
        arg_namespace,
        arg_setup_path,
        OpaqueFunction(function=launch_setup),
    ])
