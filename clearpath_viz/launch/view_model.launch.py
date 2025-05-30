#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    pkg_pf_description = FindPackageShare('clearpath_platform_description')
    pkg_viz = FindPackageShare('clearpath_viz')

    # Launch Arguments
    arg_setup_path = DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/')
    arg_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    arg_rviz_config = DeclareLaunchArgument('config', default_value='model.rviz')

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    # Launch Configurations
    rviz_config = LaunchConfiguration('config')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')

    group_view_model = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([pkg_viz, 'rviz', rviz_config])],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            remappings=[
                ("joint_states", "platform/joint_states")
            ]
        ),
        # Load Robot Description
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg_pf_description, 'launch', 'description.launch.py'])
        ),
        # Live Updater
        Node(
            package='clearpath_config_live',
            executable='clearpath_config_live',
            parameters=[{'setup_path': setup_path}]
        ),
    ])

    # Generate Initial Description
    node_generate_description = Node(
        package='clearpath_generator_common',
        executable='generate_description',
        name='generate_description',
        output='screen',
        arguments=['-s', setup_path],
    )

    event_generate_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_description,
            on_exit=[group_view_model]
        )
    )

    return LaunchDescription([
        arg_setup_path,
        arg_namespace,
        arg_rviz_config,
        arg_use_sim_time,
        node_generate_description,
        event_generate_description
    ])
