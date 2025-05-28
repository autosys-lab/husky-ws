from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lc = LaunchContext()
    joy_type = EnvironmentVariable('CPR_JOY_TYPE', default_value='logitech')

    filepath_config_joy = PathJoinSubstitution([
        FindPackageShare('clearpath_control'), 'config', 'a200', ('teleop_' + joy_type.perform(lc) + '.yaml')
    ])

    node_joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_joy],
        remappings=[
            ('/diagnostics', 'diagnostics'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('joy', 'joy_teleop/joy'),
            ('joy/set_feedback', 'joy_teleop/joy/set_feedback'),
        ]
    )

    node_teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        parameters=[filepath_config_joy],
        remappings=[
            ('joy', 'joy_teleop/joy'),
            ('cmd_vel', 'joy_teleop/cmd_vel'),
        ]
    )

    return LaunchDescription([
        node_joy,
        node_teleop_twist_joy
    ])
