"""Localization/TF group: map -> odom (EKF or raw fallback) + static sensor TFs, as
one restartable unit for component_supervisor_node (see f1tenth_bringup/
component_supervisor_node.py). Self-resolves localization_source the same way
f1tenth_perception/camera.launch.py self-resolves camera_source -- a plain Python
value read directly from stack_params.yaml, not a DeclareLaunchArgument, so this file
needs no args passed in and stays consistent regardless of which top-level bringup
includes it.

Mirrors stack_bringup.launch.py's own "3. LOCALIZATION / TF" section, minus the
calibration-ordering deferral that only applies there (stack_bringup.launch.py defers
this whole group behind is_calibration_disabled since vesc.launch.py's calibration:=
true path releases it itself; this file has no such deferral -- it's meant to be
launched directly, once hardware is already up, whether that's via
component_supervisor_node's own ordering or manually).

Static odom -> base_link (identity) TF: moved here from f1tenth_navigation/
nav2.launch.py originally, published unconditionally at the time -- odom and
base_link coincide at startup; the (then single) EKF's own map -> odom output
(world_frame: map) needed this existing odom -> base_link edge to compose
that correction against. Previously this only got published as a side effect
of nav2.launch.py (gated on enable_nav2), so with enable_nav2:=false NOTHING
published odom -> base_link at all -- a real bug, found while making
f1tenth_behavior's BT work without Nav2 (see behavior_bringup.launch.py).

NOW CONDITIONAL on localization_source == 'raw_odom' only (dual-EKF pass,
see f1tenth_bringup/config/ekf.yaml's own docstring for the full design):
with localization_source == 'ekf', the LOCAL EKF instance itself now owns
odom -> base_link directly (ekf.yaml's own world_frame reverted from map to
odom, publish_tf stays true) -- publishing this static identity edge
UNCONDITIONALLY as before would now collide with that real, non-identity
estimate (two publishers on the same TF edge). raw_odom mode still needs it
exactly as before (raw_odom_map_tf_node only mirrors /odom into map -> odom,
it does not publish odom -> base_link itself) -- so this file remains the one
place that edge comes from in that mode, unaffected by this change.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_value

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_source = get_value('localization_source')

    actions = []

    # ekf_cost_observer_node: measures each EKF instance's per-update COST
    # directly, because robot_localization's own "Failed to meet update rate!"
    # test compares loop_elapsed against 1/frequency (ros_filter.cpp:2206) and
    # is therefore self-referential -- lowering frequency raises the bar the
    # loop is judged against, and a cost regression that used to warn goes
    # silent. See that node's own module docstring.
    #
    # Hardcoded launch-arg defaults rather than stack_params.yaml keys, the
    # same convention costmap.launch.py's own render/extraction rates use: this
    # is an instrument, not a stack-shaping parameter, and nothing outside this
    # file needs to resolve its values.
    #
    # cpu_affinity default deliberately EXCLUDES cores 0,1. Those are reserved
    # for the two EKF instances (see ekf.launch.py's own cpu_affinity comment),
    # and an observer that competes for the budget it is measuring would
    # perturb the very number it exists to report.
    actions.append(DeclareLaunchArgument(
        'enable_ekf_cost_observer', default_value='true',
        description='Run ekf_cost_observer_node alongside the EKF pair, '
                    'publishing per-update cost on /diagnostics. Only has an '
                    'effect when localization_source is ekf.'))
    actions.append(DeclareLaunchArgument(
        'ekf_cost_observer_cpu_affinity', default_value='2-11',
        description="Cores for ekf_cost_observer_node's 'taskset -c' prefix. "
                    'Must exclude 0,1: those are the EKF pair\'s reserved '
                    'budget and this node measures that budget.'))

    if localization_source == 'ekf':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('f1tenth_localization'),
                'launch', 'ekf.launch.py'))
        ))
        # GLOBAL EKF (dual-EKF pass) -- fuses the local EKF's own
        # /odometry/filtered + slam_toolbox's pose, publishes map -> odom. Only
        # meaningful in 'ekf' mode -- see ekf_global.launch.py's own docstring
        # for why raw_odom mode doesn't get one.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('f1tenth_localization'),
                'launch', 'ekf_global.launch.py'))
        ))
        # Only in 'ekf' mode: raw_odom mode runs no EKF instance for this to
        # measure, and the node would sit reporting "not found in /proc".
        actions.append(Node(
            package='f1tenth_diagnostics',
            executable='ekf_cost_observer_node',
            name='ekf_cost_observer_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_ekf_cost_observer')),
            prefix=['taskset -c ',
                    LaunchConfiguration('ekf_cost_observer_cpu_affinity')],
        ))
    elif localization_source == 'raw_odom':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('f1tenth_localization'),
                'launch', 'raw_odom.launch.py'))
        ))
        # Static odom -> base_link (identity) TF -- raw_odom mode ONLY (dual-
        # EKF pass, see module docstring's own "NOW CONDITIONAL" paragraph):
        # 'ekf' mode's local EKF instance now publishes this edge itself
        # (ekf.yaml's own world_frame: odom, publish_tf: true) -- publishing
        # this static identity version too would collide with that real
        # estimate. raw_odom_map_tf_node (just included above) only ever
        # mirrors /odom into map -> odom directly, so this file remains the
        # one place odom -> base_link comes from in that mode.
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_tf',
            output='screen',
            arguments=['0.0', '0.0', '0.0',
                       '0.0', '0.0', '0.0', '1.0',
                       'odom', 'base_link'],
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('f1tenth_description'),
            'launch', 'description.launch.py'))
    ))

    return LaunchDescription(actions)
