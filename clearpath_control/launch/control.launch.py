#!/usr/bin/env python3

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from clearpath_config.common.utils.dictionary import unflatten_dict
from clearpath_config.common.utils.yaml import read_yaml

REMAPPINGS = [
    ('joint_states', 'platform/joint_states'),
    ('dynamic_joint_states', 'platform/dynamic_joint_states'),
    ('platform_velocity_controller/odom', 'platform/odom'),
    ('platform_velocity_controller/cmd_vel_unstamped', 'platform/cmd_vel_unstamped'),
    ('platform_velocity_controller/reference', 'platform/cmd_vel_unstamped'),
    ('/diagnostics', 'diagnostics'),
    ('/tf', 'tf'),
    ('/tf_static', 'tf_static'),
    ('~/robot_description', 'robot_description'),
]


def launch_setup(context, *args, **kwargs):
    # Packages
    pkg_control = FindPackageShare('clearpath_control')

    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Controllers
    config_control = PathJoinSubstitution([pkg_control, 'config', 'a200', 'control.yaml'])
    context_control = unflatten_dict(read_yaml(config_control.perform(context)))

    controllers = [
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_control],
            output={'stdout': 'screen', 'stderr': 'screen'},
            remappings=REMAPPINGS,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['--controller-manager-timeout', '60', 'joint_state_broadcaster'],
            output='screen',
            additional_env={'ROS_SUPER_CLIENT': 'True'},
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['--controller-manager-timeout', '60', 'platform_velocity_controller'],
            output='screen',
            additional_env={'ROS_SUPER_CLIENT': 'True'},
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
                name=controller,
                package='controller_manager',
                executable='spawner',
                arguments=['--controller-manager-timeout', '60', controller],
                output='screen',
                additional_env={'ROS_SUPER_CLIENT': 'True'},
                condition=IfCondition(use_sim_time),
            ))
    return [GroupAction(controllers)]


def generate_launch_description():
    # Packages
    pkg_control = FindPackageShare('clearpath_control')

    # Launch Configurations
    arg_setup_path = DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', choices=['true', 'false'], default_value='false', description='Use simulation time'
    )

    lc = LaunchContext()
    ld = LaunchDescription([arg_use_sim_time])

    # TODO think about moving the IMU to customization package (like in clearpath_customization)
    primary_imu_enable = EnvironmentVariable('CPR_IMU', default_value='false')
    if (primary_imu_enable.perform(lc)) == 'true':
        config_imu_filter = PathJoinSubstitution([pkg_control, 'config', 'a200', 'imu_filter.yaml'])
        node_imu_filter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[config_imu_filter]
        )
        ld.add_action(node_imu_filter)

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
