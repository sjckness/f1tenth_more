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
enable_mission_logger instead (stack_params.yaml), which defaults FALSE since
2026-10-16: recording is not free on this box (see that parameter's own
description) and a run that wants a bag asks for it with
enable_mission_logger:=true. With true, everything here is as it was.

Launch args mostly follow this file's OWN local convention -- plain hardcoded
DeclareLaunchArgument defaults for the recorder's operational knobs
(storage_id/sweep_*), stack_params.yaml only for values that are stack-wide
policy decisions. Same reasoning costmap.launch.py documents for its own mixed
set: the sweep cadence is a machine/deployment detail, not stack-wide
behaviour every other package needs to agree on.

The run root moved OUT of that local set and into stack_params.yaml as
mission_logger_runs_dir: f1tenth-archive.service syncs <dir>/complete/ and the
runs CLI reads the same tree, so it stopped being a detail only this launch
file knew and became something three consumers have to agree on.
"""

import os

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_default, enable_desc = get_default('enable_mission_logger')
    enable_la = DeclareLaunchArgument(
        'enable_mission_logger', default_value=str(enable_default).lower(),
        description=enable_desc)

    runs_dir_default, runs_dir_desc = get_default('mission_logger_runs_dir')
    runs_dir_la = DeclareLaunchArgument(
        'mission_logger_runs_dir',
        default_value=os.path.expanduser(str(runs_dir_default)),
        description=runs_dir_desc)
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
                    "<runs_dir>/incomplete/. 'delete': remove them outright -- "
                    "opt-in, since a partial bag is often still readable "
                    "directly and run data cannot be re-collected.")

    annotated_default, annotated_desc = get_default('mission_logger_record_annotated_image')
    annotated_la = DeclareLaunchArgument(
        'mission_logger_record_annotated_image', default_value=str(annotated_default),
        description=annotated_desc)

    depth_default, depth_desc = get_default('mission_logger_record_depth')
    depth_la = DeclareLaunchArgument(
        'mission_logger_record_depth', default_value=str(depth_default),
        description=depth_desc)
    logger_on = LaunchConfiguration('enable_mission_logger')
    depth_on = LaunchConfiguration('mission_logger_record_depth')
    # 5 Hz copy of the ZED depth for the bags (mission_logger_record_depth).
    # topic_tools relays serialized messages, so the 30 Hz input is not
    # deserialized here.
    depth_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='depth_throttle_5hz',
        output='screen',
        arguments=['messages', '/zed2/zed_node/depth/depth_registered', '5.0',
                   '/zed2/zed_node/depth/depth_registered_5hz'],
        condition=IfCondition(PythonExpression([
            "'", logger_on, "'.lower() == 'true' and '", depth_on, "'.lower() == 'true'"])),
    )

    mission_logger_node = Node(
        package='f1tenth_logger',
        executable='mission_logger_node',
        name='mission_logger_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mission_logger')),
        parameters=[{
            'runs_dir': LaunchConfiguration('mission_logger_runs_dir'),
            'storage_id': LaunchConfiguration('mission_bag_storage_id'),
            'sweep_period_sec': LaunchConfiguration('mission_bag_sweep_period_sec'),
            'sweep_action': LaunchConfiguration('mission_bag_sweep_action'),
            'record_depth': ParameterValue(depth_on, value_type=bool),
            'record_annotated_image': ParameterValue(
                LaunchConfiguration('mission_logger_record_annotated_image'), value_type=bool),
        }],
    )

    return LaunchDescription([
        enable_la,
        runs_dir_la,
        storage_id_la,
        sweep_period_la,
        sweep_action_la,
        annotated_la,
        depth_la,
        depth_throttle,
        mission_logger_node,
    ])
