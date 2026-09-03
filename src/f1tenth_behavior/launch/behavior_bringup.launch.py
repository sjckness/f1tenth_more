"""py_trees emergency-stop + safety-stop supervisor + Twist->Ackermann bridge -- the
part that actually drives.

Included from stack_bringup.launch.py only when use_behavior_tree:=true (mutually
exclusive with mpc.launch.py -- see stack_bringup.launch.py). This file does NOT
launch the Nav2 stack itself -- see navigation.launch.py, which brings up either
nav2.launch.py or mpc_corr.launch.py depending on enable_nav2, entirely independent
of use_behavior_tree.

behavior_executor_node owns the outer BT (emergency > obstacle-stop > navigate -- see
its own module docstring). Only the third lane, navigation (HasGoalPose ->
NavigateThroughPosesClient), depends on Nav2's navigate_through_poses action server;
emergency and handle_obstacle publish straight onto the mux's safety_stop lane and
work regardless of enable_nav2. NavigateThroughPosesClient's own setup() only
constructs a plain rclpy ActionClient (see its docstring) -- it does not block
waiting for the server to exist, so tree.setup() itself never fails just because
Nav2 isn't running; it fails only on a genuine timeout building the tree/node itself.
twist_to_ackermann_node bridges nav2_regulated_pure_pursuit_controller's Twist output
onto the ackermann_mux's "navigation" lane -- harmlessly idle if Nav2 isn't running,
since nothing ever publishes to its input topic.

Self-resolves enable_nav2 the same way navigation.launch.py/localization.launch.py
do (a plain Python value read directly from stack_params.yaml, not a
DeclareLaunchArgument) to decide whether to gate startup on Nav2 readiness:

  enable_nav2 true: Nav2 readiness gate (fixes behavior_executor_node racing Nav2's
    lifecycle bringup -- map_server -> full stack -> lifecycle_manager_navigation can
    take longer than a fixed delay under load, especially on Jetson).
    wait_for_trigger_service_node (generic, see its own docstring) polls
    lifecycle_manager_navigation's built-in aggregate readiness service,
    /lifecycle_manager_navigation/is_active (std_srvs/Trigger, confirmed via the
    installed nav2_lifecycle_manager library -- returns success=true only once ALL
    managed nodes (5 by default; nav2.launch.py's enable_nav2_* keys can trim this
    list), including bt_navigator and controller_server, are ACTIVE) until it succeeds
    or times out. behavior_executor_node/twist_to_ackermann_node are only added to the
    launch tree via RegisterEventHandler(OnProcessExit(...)) inspecting that node's
    exit code -- same event-driven, returncode-branching idiom already used in
    vesc.launch.py's calibration/battery-check sequencing, not a fixed-duration
    TimerAction guess. tree.setup()'s own timeout (bt_setup_timeout_sec, see
    behavior_executor_node.py) is a second, independent safety margin on top of this.

  enable_nav2 false: there is no lifecycle_manager_navigation to ever wait for (see
    navigation.launch.py) -- gating on it would time out every single run (this was
    a real, observed bug: behavior_bringup previously always waited on Nav2 readiness
    regardless of enable_nav2, so with enable_nav2:=false it reliably failed its own
    gate after nav2_readiness_timeout_sec and never started at all). Instead,
    behavior_executor_node/twist_to_ackermann_node launch immediately -- the BT's
    emergency/handle_obstacle lanes still protect the car; the navigation lane simply
    stays FAILURE (no goal pose, and even if one arrived, no action server to send it
    to) until enable_nav2 is turned back on.
"""

import os

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default, get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    behavior_share = get_package_share_directory('f1tenth_behavior')

    twist_to_ackermann_yaml = os.path.join(
        behavior_share, 'config', 'twist_to_ackermann.yaml')

    nav2_readiness_service_default, nav2_readiness_service_desc = get_default(
        'nav2_readiness_service')
    nav2_readiness_service_la = DeclareLaunchArgument(
        'nav2_readiness_service', default_value=str(nav2_readiness_service_default),
        description=nav2_readiness_service_desc)
    nav2_readiness_timeout_default, nav2_readiness_timeout_desc = get_default(
        'nav2_readiness_timeout_sec')
    nav2_readiness_timeout_la = DeclareLaunchArgument(
        'nav2_readiness_timeout_sec', default_value=str(nav2_readiness_timeout_default),
        description=nav2_readiness_timeout_desc)
    bt_setup_timeout_default, bt_setup_timeout_desc = get_default('bt_setup_timeout_sec')
    bt_setup_timeout_la = DeclareLaunchArgument(
        'bt_setup_timeout_sec', default_value=str(bt_setup_timeout_default),
        description=bt_setup_timeout_desc)
    # IsSystemOverheated's shared CPU/GPU thresholds -- only actually used if
    # enable_sys_obs is true (a separate, plain-value read via get_value(), not a
    # DeclareLaunchArgument here -- see behavior_executor_node.create_root()'s
    # docstring). Declared/forwarded either way since they're harmless no-ops when
    # sys_obs is disabled.
    sys_obs_max_temp_default, sys_obs_max_temp_desc = get_default('sys_obs_max_temp_c')
    sys_obs_max_temp_la = DeclareLaunchArgument(
        'sys_obs_max_temp_c', default_value=str(sys_obs_max_temp_default),
        description=sys_obs_max_temp_desc)
    sys_obs_max_load_default, sys_obs_max_load_desc = get_default(
        'sys_obs_max_load_percent')
    sys_obs_max_load_la = DeclareLaunchArgument(
        'sys_obs_max_load_percent', default_value=str(sys_obs_max_load_default),
        description=sys_obs_max_load_desc)
    # Safety-margin unification pass: IsObstacleDetected's corridor half-width/
    # height and IsProximityTooClose's two thresholds are derived from these three
    # (see behavior_executor_node.create_root()'s own comments and
    # stack_params.yaml's car_radius comment for the full picture) instead of being
    # hardcoded, same "declared here, bootstrap-read in main() before tree.node
    # exists" pattern as the two sys_obs_* args above.
    car_radius_default, car_radius_desc = get_default('car_radius')
    car_radius_la = DeclareLaunchArgument(
        'car_radius', default_value=str(car_radius_default), description=car_radius_desc)
    obstacle_safety_margin_default, obstacle_safety_margin_desc = get_default(
        'obstacle_safety_margin_m')
    obstacle_safety_margin_la = DeclareLaunchArgument(
        'obstacle_safety_margin_m', default_value=str(obstacle_safety_margin_default),
        description=obstacle_safety_margin_desc)
    proximity_front_extra_margin_default, proximity_front_extra_margin_desc = get_default(
        'proximity_front_extra_margin_m')
    proximity_front_extra_margin_la = DeclareLaunchArgument(
        'proximity_front_extra_margin_m',
        default_value=str(proximity_front_extra_margin_default),
        description=proximity_front_extra_margin_desc)
    # Obstacle-avoidance test configuration -- same "declared here,
    # bootstrap-read in main() before tree.node exists" pattern as the
    # sys_obs_*/car_radius args above. See stack_params.yaml's own comments
    # (and behavior_executor_node.create_root()'s docstring) for what each
    # changes; every one is a real safety-behaviour change, so they are
    # launch args rather than buried constants.
    #
    # value_type is passed EXPLICITLY rather than relying on launch_ros's
    # string type-inference, because two of these are booleans and that is
    # exactly where inference is worth not gambling on: a LaunchConfiguration
    # evaluates to the string 'False', and any path that hands that to the node
    # without coercion yields a non-empty (therefore TRUE) value -- which would
    # silently ENABLE the camera stop this pass exists to disable, and enable
    # the load trip it exists to stop trusting. Both failure modes are silent
    # and both are backwards, so the types are spelled out.
    _avoidance_args = [
        ('enable_camera_obstacle_stop', bool),
        ('enable_lidar_safety_stop', bool),
        ('proximity_front_threshold_m', float),
        ('proximity_side_threshold_m', float),
        ('enable_sys_obs_load_trip', bool),
        ('sys_obs_load_trip_consecutive_samples', int),
    ]
    avoidance_las = []
    for _name, _type in _avoidance_args:
        _default, _desc = get_default(_name)
        avoidance_las.append(DeclareLaunchArgument(
            _name, default_value=str(_default), description=_desc))
    avoidance_params = {
        _name: ParameterValue(LaunchConfiguration(_name), value_type=_type)
        for _name, _type in _avoidance_args
    }

    # Bare filename under f1tenth_behavior/missions/, or '' for no mission at
    # startup -- resolved against the package's own share dir by MissionLoader
    # itself (mission/loader.py), not here, since '' has to stay '' (not become a
    # directory path) for that "no mission" default to keep working.
    mission_file_name_default, mission_file_name_desc = get_default('mission_file_name')
    mission_file_name_la = DeclareLaunchArgument(
        'mission_file_name', default_value=str(mission_file_name_default),
        description=mission_file_name_desc)
    # cpu_affinity/nice -- added by the stack-wide CPU-budget investigation
    # (found this node at ~45% CPU / 23 threads, unpinned -- lighter than
    # the perception nodes, but still real). Same "hardcoded default
    # directly in the launch file, not stack_params.yaml" convention every
    # other pinned node in this stack already follows (the right core ids
    # are machine-specific).
    #
    # CORRECTED (core-remap pass, following a live CPU-contention
    # investigation -- see that pass's own report): this comment previously
    # claimed "combined cost of all three [this node + ekf_node +
    # foxglove_bridge] is well under one core" -- wrong, and didn't even
    # account for the SECOND ekf_node instance the later dual-EKF pass
    # added to the same pair. Live measurement (at rest) found this node at
    # 24.7% CPU (idle) -- consistent with the documented ~45% under real BT
    # activity above, not a contradiction, just a different load state --
    # while the pair as a whole (this node + both EKF instances +
    # foxglove_bridge) carried ~152% combined demand on a 200% (2-core)
    # budget, confirmed saturated live (cpu0/cpu1 both 99%+ busy) and
    # producing ekf_node's own "Failed to meet update rate!" warnings (see
    # ekf.launch.py's own matching comment). Moved to its own core (4),
    # off the EKF pair entirely -- ekf.launch.py/ekf_global.launch.py now
    # reserve cores 0,1 for just the two EKF instances, foxglove_bridge
    # moved to core 3 (see that file's own matching comment).
    #
    # CPU AFFINITY NOW A taskset -c LAUNCH PREFIX, NOT self-pinning (thread-
    # pinning-leak fix, Step 6 reintroduction investigation): the previous
    # mechanism -- behavior_executor_node.py's own _apply_cpu_affinity_and_
    # priority() calling os.sched_setaffinity(0, cores) once, in-process,
    # from __init__ -- only ever restricted the ONE thread that happened to
    # be executing that call (confirmed live: 21 of this node's 22 threads
    # showed full 0-11 affinity, with 2 actually caught executing on cpu1 --
    # one of the EKF pair's own reserved cores -- under Stage 4 load). Worse,
    # that call runs AFTER tree.setup() (py_trees_ros' own executor/action-
    # client machinery), so most of this node's threads already exist,
    # unpinned, before the call ever fires -- pinning later wouldn't fix that
    # even if os.sched_setaffinity applied process-wide (it doesn't; pid=0
    # means the calling thread only). taskset -c sets the affinity mask
    # BEFORE this node's own code starts running at all, so it applies to
    # the process's very first thread and everything it (or any library)
    # spawns afterward inherits it -- confirmed empirically across every
    # OTHER pinned node in this stack (ekf_node x2, slam_toolbox,
    # foxglove_bridge, all already taskset-prefixed): 100% of every one of
    # their threads stayed on their assigned cores through Step 6's full
    # reintroduction sequence, including Stage 4's saturated load, while
    # every self-pinning node leaked. behavior_cpu_affinity_la still exists
    # unchanged below -- it now feeds the launch-level prefix= argument
    # instead of a ROS param this node reads on itself.
    behavior_cpu_affinity_la = DeclareLaunchArgument(
        'behavior_cpu_affinity', default_value='4',
        description="Comma-separated core ids to pin behavior_executor_node "
                    "to via a 'taskset -c' launch prefix. Own dedicated core "
                    "(moved off the EKF pair, 0,1 -- core-remap pass, see "
                    "this file's own comment above). Must stay a valid, "
                    "non-empty core list -- 'taskset -c' with no core list "
                    "is a shell-level error, not a graceful no-op (same "
                    "caveat as ekf.launch.py's own matching argument); "
                    "remove this Node's prefix= argument instead to fully "
                    "disable pinning.")
    behavior_nice_la = DeclareLaunchArgument(
        'behavior_nice', default_value='0',
        description="Process niceness for behavior_executor_node. 0: no-op.")

    behavior_executor_node = Node(
        package='f1tenth_behavior',
        executable='behavior_executor_node',
        name='behavior_executor_node',
        output='screen',
        prefix=['taskset -c ', LaunchConfiguration('behavior_cpu_affinity')],
        parameters=[{
            'bt_setup_timeout_sec': LaunchConfiguration('bt_setup_timeout_sec'),
            'sys_obs_max_temp_c': LaunchConfiguration('sys_obs_max_temp_c'),
            'sys_obs_max_load_percent': LaunchConfiguration('sys_obs_max_load_percent'),
            'car_radius': LaunchConfiguration('car_radius'),
            'obstacle_safety_margin_m': LaunchConfiguration('obstacle_safety_margin_m'),
            'proximity_front_extra_margin_m': LaunchConfiguration(
                'proximity_front_extra_margin_m'),
            'mission_file_name': LaunchConfiguration('mission_file_name'),
            'nice': LaunchConfiguration('behavior_nice'),
            **avoidance_params,
        }],
    )
    twist_to_ackermann_node = Node(
        package='f1tenth_behavior',
        executable='twist_to_ackermann_node',
        name='twist_to_ackermann_node',
        output='screen',
        parameters=[twist_to_ackermann_yaml],
    )

    declared_args = [
        nav2_readiness_service_la,
        nav2_readiness_timeout_la,
        bt_setup_timeout_la,
        sys_obs_max_temp_la,
        sys_obs_max_load_la,
        car_radius_la,
        obstacle_safety_margin_la,
        proximity_front_extra_margin_la,
        *avoidance_las,
        mission_file_name_la,
        behavior_cpu_affinity_la, behavior_nice_la,
    ]

    if not get_value('enable_nav2'):
        # No lifecycle_manager_navigation to ever wait for -- launch immediately. See
        # module docstring's "enable_nav2 false" paragraph.
        return LaunchDescription(declared_args + [
            LogInfo(msg='[behavior_bringup] enable_nav2 is false -- skipping the Nav2 '
                        'readiness gate, launching behavior_executor_node and '
                        'twist_to_ackermann_node immediately (navigation lane will '
                        'stay idle until enable_nav2 is turned back on).'),
            behavior_executor_node,
            twist_to_ackermann_node,
        ])

    wait_for_nav2_node = Node(
        package='f1tenth_behavior',
        executable='wait_for_trigger_service_node',
        name='wait_for_nav2_ready',
        output='screen',
        parameters=[{
            'service_name': LaunchConfiguration('nav2_readiness_service'),
            'timeout_sec': LaunchConfiguration('nav2_readiness_timeout_sec'),
        }],
    )

    def _on_readiness_exit(event, context):
        if event.returncode != 0:
            return [LogInfo(
                msg='[behavior_bringup] Nav2 readiness check failed -- '
                    'behavior_executor_node/twist_to_ackermann_node will NOT launch. '
                    'Is lifecycle_manager_navigation active? Check: '
                    'ros2 service call /lifecycle_manager_navigation/is_active '
                    'std_srvs/srv/Trigger -- and: ros2 lifecycle get /bt_navigator')]
        return [
            LogInfo(msg='[behavior_bringup] Nav2 ready -- launching behavior_executor_node '
                        'and twist_to_ackermann_node.'),
            behavior_executor_node,
            twist_to_ackermann_node,
        ]

    readiness_exit_handler = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_nav2_node, on_exit=_on_readiness_exit))

    return LaunchDescription(declared_args + [
        wait_for_nav2_node,
        readiness_exit_handler,
    ])
