"""
wifi_coverage_node and link_manager (f1tenth_comms): the car's side of the link to the base.

Registered as the `comms` component in f1tenth_bringup/config/components.yaml.
The supervisor starts it only with the launch argument enable_comms:=true
(default false, from stack_params.yaml); stack_bringup.launch.py includes it
on the same argument. With the stack already up, on demand:

    ./scripts/stackctl.py start comms

or alone:

    ros2 launch f1tenth_comms comms.launch.py

Parameters come from config/comms.yaml; comms_params_file:=<file> replaces
it. comms_tailscale_host:=... and comms_local_host:=... override the base's
addresses without a new file (e.g. the tpad as the base). The nodes publish
/comms/* only: nothing in them drives, stops or aborts the car. link_manager
may run `nmcli connection up <hotspot>` while the link is LOST
(link_manager_node.py, reconnect.py); comms_reconnect_dry_run:=true logs
what it would run instead.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _link_manager(context):
    """Return link_manager: the yaml, plus the overrides that are not empty."""
    overrides = {}
    for arg, param in (('comms_tailscale_host', 'tailscale_host'),
                       ('comms_local_host', 'local_host')):
        value = LaunchConfiguration(arg).perform(context).strip()
        if value:
            overrides[param] = value
    if LaunchConfiguration('comms_reconnect_dry_run').perform(context).lower() == 'true':
        overrides['reconnect_dry_run'] = True
    return [Node(
        package='f1tenth_comms',
        executable='link_manager',
        name='link_manager',
        output='screen',
        parameters=[LaunchConfiguration('comms_params_file'), overrides],
        condition=IfCondition(LaunchConfiguration('enable_link_manager')),
    )]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('f1tenth_comms'), 'config', 'comms.yaml')
    params_file = LaunchConfiguration('comms_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'comms_params_file', default_value=default_params,
            description='Parameter file for wifi_coverage_node and link_manager.'),
        DeclareLaunchArgument(
            'enable_wifi_coverage', default_value='true',
            description='Run wifi_coverage_node (/comms/coverage_*).'),
        DeclareLaunchArgument(
            'enable_link_manager', default_value='true',
            description='Run link_manager (/comms/link_status, heartbeat, reconnect).'),
        DeclareLaunchArgument(
            'comms_tailscale_host', default_value='',
            description="The base's Tailscale IP or MagicDNS name; '' keeps the yaml's."),
        DeclareLaunchArgument(
            'comms_local_host', default_value='',
            description="The base's hotspot address; '' keeps the yaml's."),
        DeclareLaunchArgument(
            'comms_reconnect_dry_run', default_value='false',
            description='true: link_manager only logs the nmcli command it would run.'),
        Node(
            package='f1tenth_comms',
            executable='wifi_coverage_node',
            name='wifi_coverage_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('enable_wifi_coverage')),
        ),
        OpaqueFunction(function=_link_manager),
    ])
