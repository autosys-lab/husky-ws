#!/usr/bin/env python3

from pprint import pprint
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from clearpath_config.common.utils.yaml import read_yaml, write_yaml

MARKER = 'rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/'

MOVEIT_TOPICS = [
    'attached_collision_object',
    'compute_cartesian_path',
    'display_planned_path',
    'get_planner_params',
    'monitored_planning_scene',
    'move_action',
    'query_planner_interface',
    'set_planner_params',
    'trajectory_execution_event',
    MARKER + 'feedback',
    MARKER + 'get_interactive_markers',
    MARKER + 'update',
]


def launch_setup(context, *args, **kwargs):
    # Packages
    pkg_viz = FindPackageShare('clearpath_viz')

    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    arm_count = LaunchConfiguration('arm_count')

    # Apply namespace to RViz Configuration
    context_namespace = namespace.perform(context)

    # RViz Configuration
    default_config = PathJoinSubstitution([pkg_viz, 'rviz', 'moveit.rviz'])

    context_rviz = default_config.perform(context)
    content_rviz = read_yaml(context_rviz)

    content_rviz['Visualization Manager']['Displays'][1]['Move Group Namespace'] = '/' + context_namespace

    namespaced_config = '/tmp/moveit.rviz'
    write_yaml(namespaced_config, content_rviz)

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # Remappings for MoveIt!
    for topic in MOVEIT_TOPICS:
        remappings.append(('/%s' % topic, topic))  # Standard Topics
        remappings.append(('/%s/%s/' % (context_namespace, context_namespace) + topic, topic))  # Doubled Topics

    # Arm Kinematics
    parameters = {'use_sim_time': use_sim_time, 'robot_description_kinematics': {}}
    for i in range(int(arm_count.perform(context))):
        parameters['robot_description_kinematics'].update({
            'arm_%s' % i: {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.005,
                'kinematics_solver_timeout': 0.005,
            }
        })

    return [
        GroupAction([
            PushRosNamespace(namespace),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', namespaced_config],
                parameters=[parameters],
                remappings=remappings,
                output='screen'
            )
        ])
    ]


def generate_launch_description():
    # Launch Arguments
    arg_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )
    arg_arm_count = DeclareLaunchArgument(
        'arm_count', default_value='1', description='Number of arms to add kinematics for.'
    )

    return LaunchDescription([
        arg_namespace,
        arg_use_sim_time,
        arg_arm_count,
        OpaqueFunction(function=launch_setup)
    ])
