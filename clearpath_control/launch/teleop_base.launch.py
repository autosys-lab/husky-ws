#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    pkg_control = FindPackageShare('clearpath_control')

    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        output='screen',
        remappings=[
            ('cmd_vel', 'twist_marker_server/cmd_vel'),
            ('twist_server/feedback', 'twist_marker_server/feedback'),
            ('twist_server/update', 'twist_marker_server/update'),
        ],
        parameters=[
            PathJoinSubstitution([pkg_control, 'config', 'a200', 'teleop_interactive_markers.yaml'])
        ],
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[
            ('cmd_vel_out', 'platform/cmd_vel_unstamped'),
            ('/diagnostics', 'diagnostics'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[
            PathJoinSubstitution([pkg_control, 'config', 'twist_mux.yaml'])
        ],
    )

    return LaunchDescription([
        node_interactive_marker_twist_server,
        node_twist_mux
    ])
