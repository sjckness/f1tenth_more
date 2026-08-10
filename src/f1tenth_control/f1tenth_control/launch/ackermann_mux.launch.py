"""Ackermann command mux: arbitrates between drive-command sources (MPC,
teleop, startup sequence) by priority, publishing the single /ackermann_drive
consumed by vesc_ackermann's ackermann_to_vesc_node.

Not the upstream ackermann_mux package's own launch file: that one loads
three separate locks/topics/joystick config files and a different remap
target than this stack uses, so it's not a drop-in here -- mux_config below
is our single combined config (f1tenth_bringup/config/mux.yaml).
"""

from f1tenth_params.param_defaults import get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mux_config, mux_config_desc = get_path_default('mux_config')
    mux_la = DeclareLaunchArgument(
        'mux_config', default_value=mux_config, description=mux_config_desc)

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd', 'ackermann_drive')],
    )

    return LaunchDescription([mux_la, ackermann_mux_node])
