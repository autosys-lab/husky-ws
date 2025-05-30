#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    pkg_control = FindPackageShare('clearpath_control')
    pkg_pf_description = FindPackageShare('clearpath_platform_description')
    pkg_bringup = FindPackageShare('husky_bringup')

    # Launch Arguments
    arg_setup_path = DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', choices=['true', 'false'], default_value='false', description='Use simulation time'
    )
    arg_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    arg_enable_ekf = DeclareLaunchArgument(
        'enable_ekf', default_value='true', choices=['true', 'false'], description='Enable localization via EKF node'
    )

    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    enable_ekf = LaunchConfiguration('enable_ekf')

    # TODO check if this is necessary (we already publish robot state in clearpath_control?)
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        ' ',
        PathJoinSubstitution([pkg_pf_description, 'urdf', 'husky.urdf.xacro']),
        ' ',
        'name:=husky',
        ' ',
        "prefix:=''",
        ' ',
        'namespace:=',
        namespace
    ])

    # TODO check if this is necessary (we already publish robot state in clearpath_control?)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }],
    )

    # Launch files
    launch_file_platform_description = PathJoinSubstitution([pkg_pf_description, 'launch', 'description.launch.py'])
    launch_file_control = PathJoinSubstitution([pkg_control, 'launch', 'control.launch.py'])
    launch_file_localization = PathJoinSubstitution([pkg_control, 'launch', 'localization.launch.py'])
    launch_file_teleop_base = PathJoinSubstitution([pkg_control, 'launch', 'teleop_base.launch.py'])
    launch_file_teleop_joy = PathJoinSubstitution([pkg_control, 'launch', 'teleop_joy.launch.py'])
    launch_file_accessories = PathJoinSubstitution([pkg_bringup, 'launch', 'accessories.launch.py'])

    group_platform_action = GroupAction([
        PushRosNamespace(namespace),

        # TODO check if this is necessary (we already publish robot state in clearpath_control?)
        node_robot_state_publisher,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_platform_description),
            launch_arguments=[
                # ('setup_path', setup_path),
                ('use_sim_time', use_sim_time),
                ('namespace', namespace),
            ]
        ),

        # Launch clearpath_control/control.launch.py which is just robot_localization.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_control),
            launch_arguments=[
                # ('setup_path', setup_path),
                ('use_sim_time', use_sim_time),
            ]
        ),

        # Launch localization (ekf node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_localization),
            launch_arguments=[
                # ('setup_path', setup_path),
                ('use_sim_time', use_sim_time),
                ('enable_ekf', enable_ekf)
            ]
        ),

        # Launch clearpath_control/teleop_base.launch.py which is various ways to tele-op
        # the robot but does not include the joystick. Also, has a twist mux.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_teleop_base),
            launch_arguments=[
                # ('setup_path', setup_path),
                ('use_sim_time', use_sim_time),
            ]
        ),

        # Launch clearpath_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_teleop_joy),
            launch_arguments=[
                # ('setup_path', setup_path),
                ('use_sim_time', use_sim_time),
            ]
        ),

        # Launch husky_bringup/accessories.launch.py which is the sensors commonly used on the Husky.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_accessories)
        ),
    ])

    return LaunchDescription([
        # arg_setup_path,
        arg_use_sim_time,
        arg_namespace,
        arg_enable_ekf,
        group_platform_action
    ])
