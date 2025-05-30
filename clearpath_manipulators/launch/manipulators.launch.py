#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Packages
    pkg_mnp_desc = FindPackageShare('clearpath_manipulators_description')
    pkg_manipulators = FindPackageShare('clearpath_manipulators')

    # Launch Arguments
    arg_setup_path = DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', choices=['true', 'false'], default_value='false', description='Use simulation time'
    )
    arg_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    arg_launch_moveit = DeclareLaunchArgument(
        'launch_moveit', choices=['true', 'false'], default_value='false', description='Launch MoveIt'
    )
    arg_control_delay = DeclareLaunchArgument(
        'control_delay', default_value='0.0', description='Control launch delay in seconds.'
    )
    arg_moveit_delay = DeclareLaunchArgument(
        'moveit_delay', default_value='1.0', description='MoveIt launch delay in seconds.'
    )

    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    launch_moveit = LaunchConfiguration('launch_moveit')
    control_delay = LaunchConfiguration('control_delay')
    moveit_delay = LaunchConfiguration('moveit_delay')

    # Launch files
    launch_file_mnp_desc = PathJoinSubstitution([pkg_mnp_desc, 'launch', 'description.launch.py'])
    launch_file_control = PathJoinSubstitution([pkg_manipulators, 'launch', 'control.launch.py'])
    launch_file_moveit = PathJoinSubstitution([pkg_manipulators, 'launch', 'moveit.launch.py'])

    group_manipulators_action = GroupAction(
        actions=[
            PushRosNamespace(PathJoinSubstitution([namespace, 'manipulators'])),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_mnp_desc),
                launch_arguments=[
                    ('namespace', namespace),
                    ('setup_path', setup_path),
                    ('use_sim_time', use_sim_time),
                ],
            ),
            # Launch clearpath_control/control.launch.py which is just robot_localization.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_control),
                launch_arguments=[
                    ('namespace', namespace),
                    ('setup_path', setup_path),
                    ('use_sim_time', use_sim_time),
                ],
            ),
        ]
    )

    control_delayed = TimerAction(period=control_delay, actions=[group_manipulators_action])

    # Launch MoveIt
    moveit_node_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_moveit),
        launch_arguments=[
            ('setup_path', setup_path),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(launch_moveit)
    )

    moveit_delayed = TimerAction(period=moveit_delay, actions=[moveit_node_action])

    return LaunchDescription([
        arg_setup_path,
        arg_use_sim_time,
        arg_namespace,
        arg_launch_moveit,
        arg_control_delay,
        arg_moveit_delay,
        control_delayed,
        moveit_delayed,
    ])
