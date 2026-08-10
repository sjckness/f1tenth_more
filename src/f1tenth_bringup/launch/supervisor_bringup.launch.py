"""Parallel bringup path: launches component_supervisor_node, which spawns every
component itself via subprocess (see component_supervisor_node.py's own docstring)
and exposes /restart_component (f1tenth_messages/srv/RestartComponent) and
~/control_component (f1tenth_messages/srv/ComponentControl) to restart, shut down,
or start any one of them independently. This file's only job is starting that one
node -- it is NOT a rewrite of stack_bringup.launch.py's grouping logic, and
stack_bringup.launch.py is untouched and still works as a single-process fallback.

Example:
  ros2 launch f1tenth_bringup supervisor_bringup.launch.py
  ros2 service call /restart_component f1tenth_messages/srv/RestartComponent \\
      "{component_name: 'navigation'}"
  ros2 service call /component_supervisor_node/control_component \\
      f1tenth_messages/srv/ComponentControl "{component_name: 'navigation', action: 0}"
"""

from f1tenth_params.param_defaults import get_default, get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    components_config, components_config_desc = get_path_default('components_config')
    components_config_la = DeclareLaunchArgument(
        'components_config', default_value=components_config,
        description=components_config_desc)
    restart_timeout_default, restart_timeout_desc = get_default('restart_timeout_sec')
    restart_timeout_la = DeclareLaunchArgument(
        'restart_timeout_sec', default_value=str(restart_timeout_default),
        description=restart_timeout_desc)
    log_dir_default, log_dir_desc = get_default('log_dir')
    log_dir_la = DeclareLaunchArgument(
        'log_dir', default_value=str(log_dir_default), description=log_dir_desc)
    watchdog_period_default, watchdog_period_desc = get_default('watchdog_period_sec')
    watchdog_period_la = DeclareLaunchArgument(
        'watchdog_period_sec', default_value=str(watchdog_period_default),
        description=watchdog_period_desc)
    max_auto_restarts_default, max_auto_restarts_desc = get_default('max_auto_restarts')
    max_auto_restarts_la = DeclareLaunchArgument(
        'max_auto_restarts', default_value=str(max_auto_restarts_default),
        description=max_auto_restarts_desc)
    restart_budget_window_default, restart_budget_window_desc = get_default(
        'restart_budget_window_sec')
    restart_budget_window_la = DeclareLaunchArgument(
        'restart_budget_window_sec', default_value=str(restart_budget_window_default),
        description=restart_budget_window_desc)

    component_supervisor_node = Node(
        package='f1tenth_bringup',
        executable='component_supervisor_node',
        name='component_supervisor_node',
        output='screen',
        parameters=[{
            'components_config': LaunchConfiguration('components_config'),
            'restart_timeout_sec': LaunchConfiguration('restart_timeout_sec'),
            'log_dir': LaunchConfiguration('log_dir'),
            'watchdog_period_sec': LaunchConfiguration('watchdog_period_sec'),
            'max_auto_restarts': LaunchConfiguration('max_auto_restarts'),
            'restart_budget_window_sec': LaunchConfiguration('restart_budget_window_sec'),
        }],
    )

    return LaunchDescription([
        components_config_la,
        restart_timeout_la,
        log_dir_la,
        watchdog_period_la,
        max_auto_restarts_la,
        restart_budget_window_la,
        component_supervisor_node,
    ])
