"""Joystick manual-control input (joy + joy_teleop). Not included by
stack_bringup.launch.py by default -- run standalone
(`ros2 launch f1tenth_control joy.launch.py`) when manual joystick control is
needed; autonomous runs don't need it.
"""

from f1tenth_params.param_defaults import get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_teleop_config, joy_config_desc = get_path_default('joy_config')
    joy_la = DeclareLaunchArgument(
        'joy_config', default_value=joy_teleop_config, description=joy_config_desc)

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')],
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')],
    )

    return LaunchDescription([joy_la, joy_node, joy_teleop_node])
