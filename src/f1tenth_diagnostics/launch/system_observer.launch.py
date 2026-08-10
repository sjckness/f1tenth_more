"""Launches system_observer_node standalone -- publishes
f1tenth_messages/SystemStatus (CPU/RAM via psutil, Jetson GPU/EMC/temps via
jtop) on /diagnostics/system_status. See system_observer_node.py's module docstring
for jtop connection/fallback behavior.

Self-gated behind enable_sys_obs (read via get_value() as a plain Python value at
parse time, same mechanism the 6 stack-wide branching args use -- see
f1tenth_params/config/stack_params.yaml's enable_sys_obs comment): when false, this
file returns a LaunchDescription with no Node at all, so system_observer_node is fully
skippable regardless of where this launch file is included from (stack_bringup's
diagnostics section, components.yaml's diagnostics component, or standalone). Battery
monitoring is unaffected either way -- see diagnostics_server.launch.py, which is
never gated behind this.
"""

from f1tenth_params.param_defaults import get_default, get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    if not get_value('enable_sys_obs'):
        return LaunchDescription([
            LogInfo(msg='[system_observer] enable_sys_obs is false -- '
                        'system_observer_node will NOT be launched.'),
        ])

    publish_rate_default, publish_rate_desc = get_default('publish_rate_hz')
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz', default_value=str(publish_rate_default),
        description=publish_rate_desc)

    system_observer_node = Node(
        package='f1tenth_diagnostics',
        executable='system_observer_node',
        name='system_observer_node',
        output='screen',
        parameters=[{
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }],
    )

    return LaunchDescription([
        publish_rate_arg,
        system_observer_node,
    ])
