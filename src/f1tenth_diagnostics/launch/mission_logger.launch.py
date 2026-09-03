"""Launches mission_logger_node -- automatic per-mission rosbag2 recording,
started/stopped by f1tenth_behavior's own /mission/status transitions (see that
node's own module docstring for the full design, including the loader.py
mission-end event fix it depends on).

Included by components.yaml's `diagnostics` component alongside
system_observer.launch.py / diagnostics_server.launch.py. Not gated behind
enable_sys_obs: that flag governs CPU/GPU/RAM telemetry, an unrelated concern,
and a mission logger that silently stops existing because an unrelated
telemetry toggle was flipped is exactly the "recording was off when it
mattered" failure this node exists to remove. Gate it with its own
enable_mission_logger instead (stack_params.yaml), which defaults true.

Launch args deliberately follow this file's OWN local convention -- plain
hardcoded DeclareLaunchArgument defaults for the recorder's operational knobs
(bag_root/storage_id/sweep_*), stack_params.yaml only for the one value that
is a stack-wide policy decision (enable_mission_logger). Same reasoning
costmap.launch.py documents for its own mixed set: the bag root and sweep
cadence are machine/deployment details, not stack-wide behaviour every other
package needs to agree on.
"""

import os

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_default, enable_desc = get_default('enable_mission_logger')
    enable_la = DeclareLaunchArgument(
        'enable_mission_logger', default_value=str(enable_default).lower(),
        description=enable_desc)

    bag_root_la = DeclareLaunchArgument(
        'mission_bag_root',
        default_value=os.path.join(os.path.expanduser('~'), '.ros', 'mission_bags'),
        description="Directory holding one bag directory per mission run, plus "
                    "each run's own .manifest.json/.params.yaml sidecars.")
    storage_id_la = DeclareLaunchArgument(
        'mission_bag_storage_id', default_value='mcap',
        description="rosbag2 storage plugin. 'mcap' is preferred (Foxglove reads "
                    "it directly, no conversion step) but is NOT installed by "
                    "default on this box -- the node checks the registered "
                    "writers at startup and falls back to sqlite3 with a loud "
                    "warning naming the package to install.")
    sweep_period_la = DeclareLaunchArgument(
        'mission_bag_sweep_period_sec', default_value='300.0',
        description="How often to sweep bag directories left with no "
                    "metadata.yaml (recorder died mid-record). Swept at startup "
                    "AND on this period -- startup-only sweeping is precisely "
                    "why /dev/shm orphans still accumulated to the point of "
                    "breaking DDS discovery within one session.")
    sweep_action_la = DeclareLaunchArgument(
        'mission_bag_sweep_action', default_value='move',
        description="'move' (default): relocate incomplete bags into "
                    "<bag_root>/incomplete/. 'delete': remove them outright -- "
                    "opt-in, since a partial bag is often still readable "
                    "directly and run data cannot be re-collected.")

    mission_logger_node = Node(
        package='f1tenth_diagnostics',
        executable='mission_logger_node',
        name='mission_logger_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mission_logger')),
        parameters=[{
            'bag_root': LaunchConfiguration('mission_bag_root'),
            'storage_id': LaunchConfiguration('mission_bag_storage_id'),
            'sweep_period_sec': LaunchConfiguration('mission_bag_sweep_period_sec'),
            'sweep_action': LaunchConfiguration('mission_bag_sweep_action'),
        }],
    )

    return LaunchDescription([
        enable_la,
        bag_root_la,
        storage_id_la,
        sweep_period_la,
        sweep_action_la,
        mission_logger_node,
    ])
