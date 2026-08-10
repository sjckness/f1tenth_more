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
    # Bare filename under f1tenth_behavior/missions/, or '' for no mission at
    # startup -- resolved against the package's own share dir by MissionLoader
    # itself (mission/loader.py), not here, since '' has to stay '' (not become a
    # directory path) for that "no mission" default to keep working.
    mission_file_name_default, mission_file_name_desc = get_default('mission_file_name')
    mission_file_name_la = DeclareLaunchArgument(
        'mission_file_name', default_value=str(mission_file_name_default),
        description=mission_file_name_desc)

    behavior_executor_node = Node(
        package='f1tenth_behavior',
        executable='behavior_executor_node',
        name='behavior_executor_node',
        output='screen',
        parameters=[{
            'bt_setup_timeout_sec': LaunchConfiguration('bt_setup_timeout_sec'),
            'sys_obs_max_temp_c': LaunchConfiguration('sys_obs_max_temp_c'),
            'sys_obs_max_load_percent': LaunchConfiguration('sys_obs_max_load_percent'),
            'car_radius': LaunchConfiguration('car_radius'),
            'obstacle_safety_margin_m': LaunchConfiguration('obstacle_safety_margin_m'),
            'proximity_front_extra_margin_m': LaunchConfiguration(
                'proximity_front_extra_margin_m'),
            'mission_file_name': LaunchConfiguration('mission_file_name'),
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
        mission_file_name_la,
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
