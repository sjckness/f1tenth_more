"""Launches diagnostics_server_node -- continuous battery-voltage safety monitoring
(independent of enable_sys_obs, see that node's own module docstring) plus the
on-demand ~/run_diagnostics service. Included unconditionally by
stack_bringup.launch.py / components.yaml's diagnostics component, alongside
system_observer.launch.py -- which, unlike this file, IS gated behind enable_sys_obs.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    min_battery_voltage_default, min_battery_voltage_desc = get_default('min_battery_voltage')
    min_battery_voltage_arg = DeclareLaunchArgument(
        'min_battery_voltage', default_value=str(min_battery_voltage_default),
        description=min_battery_voltage_desc)
    battery_check_rate_default, battery_check_rate_desc = get_default('battery_check_rate_hz')
    battery_check_rate_arg = DeclareLaunchArgument(
        'battery_check_rate_hz', default_value=str(battery_check_rate_default),
        description=battery_check_rate_desc)

    diagnostics_server_node = Node(
        package='f1tenth_diagnostics',
        executable='diagnostics_server_node',
        name='diagnostics_server_node',
        output='screen',
        parameters=[{
            'min_battery_voltage': LaunchConfiguration('min_battery_voltage'),
            'battery_check_rate_hz': LaunchConfiguration('battery_check_rate_hz'),
        }],
    )

    return LaunchDescription([
        min_battery_voltage_arg,
        battery_check_rate_arg,
        diagnostics_server_node,
    ])
