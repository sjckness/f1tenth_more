"""F1TENTH stack bringup -- thin orchestrator.

Every node lives in its owning package's own launch file; this file only
declares the truly stack-wide arguments (ones more than one included file
needs) and includes those launch files. The one exception is foxglove_bridge:
a generic dev/visualization bridge that isn't "owned" by any f1tenth_*
package, so it stays inline here.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Camera source selection. The two paths are mutually exclusive: exactly one
    # of {v4l2 webcam, ZED2 wrapper} is brought up (f1tenth_perception/camera.launch.py),
    # and both publish onto the canonical /camera/image_raw + /camera/camera_info so
    # downstream nodes (detection.launch.py) are agnostic to which camera is active.
    # Stack-wide: forwarded into both camera.launch.py and detection.launch.py, since
    # detection_3d_node also needs to know whether ZED depth exists.
    camera_source_la = DeclareLaunchArgument(
        'camera_source', default_value='zed',
        description="Camera source: 'zed' (ZED2 wrapper) or 'webcam' (v4l2 UVC "
                    'on /dev/video0). Selects exactly one; the other is not '
                    'launched at all.')
    # Reactive safety layer: watches Detection3DArray and zeroes the MPC's
    # v_ref parameter when an obstacle enters a fixed corridor, restoring it
    # once clear. Off by default -- new/not yet road-tested, and it actively
    # overrides MPC speed, so it's an explicit opt-in per run. Corridor/timing
    # parameters live in safety_stop_controller's own launch file.
    enable_safety_stop_la = DeclareLaunchArgument(
        'enable_safety_stop', default_value='false',
        description='Launch the safety_stop_controller package (see its own '
                    'launch/safety_stop.launch.py for corridor/timing args). '
                    'Default false: opt-in per run.')
    # Selects the autonomous drive-command source, mutually exclusive with mpc_launch.py:
    # both would otherwise publish onto the same ackermann_mux "navigation" lane (topic
    # "drive"), and only one drive-command source should own it at a time -- same
    # mutual-exclusion pattern as camera_source above.
    use_behavior_tree_la = DeclareLaunchArgument(
        'use_behavior_tree', default_value='True',
        description='false: mpc_launch.py drives the "drive" mux lane. true (default): '
                    "launch f1tenth_behavior's BT safety-stop supervisor + "
                    'Twist->Ackermann bridge instead (behavior_bringup.launch.py), '
                    'which drives the mux lane via the Nav2 stack that '
                    'f1tenth_navigation/nav2_bringup.launch.py brings up (see enable_nav2 '
                    'to control that independently).')

    is_safety_stop_enabled = IfCondition(LaunchConfiguration('enable_safety_stop'))
    is_mpc_enabled = UnlessCondition(LaunchConfiguration('use_behavior_tree'))
    is_behavior_tree_enabled = IfCondition(LaunchConfiguration('use_behavior_tree'))
    # enable_nav2 is declared in f1tenth_localization/launch/ekf_launch.py (see its
    # docstring), not here -- ekf_bringup below must be included before this condition is
    # evaluated so the argument's default is registered first.
    is_nav2_enabled = IfCondition(LaunchConfiguration('enable_nav2'))

    def include(package, launch_file, condition=None, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package), 'launch', launch_file)
            ),
            launch_arguments=launch_arguments.items(),
            condition=condition,
        )

    # ---- VESC hardware chain (ackermann->VESC, odometry, driver, IMU TF) ---
    vesc_bringup = include('f1tenth_hardware', 'vesc_launch.py')
    # ---- ackermann command mux ------------------------------------------
    ackermann_mux_bringup = include('f1tenth_control', 'ackermann_mux_launch.py')
    # ---- MPC controller (only when NOT use_behavior_tree) ------------------
    mpc_bringup = include('f1tenth_control', 'mpc_launch.py', condition=is_mpc_enabled)
    # ---- EKF (odom + IMU fusion, odom -> base_link TF) ---------------------
    ekf_bringup = include('f1tenth_localization', 'ekf_launch.py')
    # ---- static base_link -> laser TF --------------------------------------
    sensor_tf_bringup = include('f1tenth_description', 'sensor_tf_launch.py')
    # ---- camera (ZED2 or webcam, mutually exclusive) + ZED2 static TF ------
    camera_bringup = include(
        'f1tenth_perception', 'camera.launch.py',
        camera_source=LaunchConfiguration('camera_source'))
    # ---- YOLO 2D detector + 2D-to-3D detection fusion -----------------------
    detection_bringup = include(
        'f1tenth_perception', 'detection.launch.py',
        camera_source=LaunchConfiguration('camera_source'))
    # ---- safety-stop supervisor (opt-in) ------------------------------------
    safety_stop_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('safety_stop_controller'),
                          'launch', 'safety_stop.launch.py')
        ),
        condition=is_safety_stop_enabled,
    )

    # ---- f1tenth_navigation: map_server + full Nav2 stack (planner/controller/ -----
    # ---- costmaps/bt_navigator), one unified lifecycle_manager_navigation ----------
    # Gated by enable_nav2 (declared in f1tenth_localization/ekf_launch.py, default
    # true), independent of use_behavior_tree: these nodes idle with no active goal, so
    # running them doesn't put any drive command on the mux by itself -- only
    # behavior_bringup's twist_to_ackermann_node (gated below) ever turns a Nav2 plan
    # into one. f1tenth_behavior's own launch file depends on this already being up
    # rather than re-including it, to avoid duplicate Nav2 nodes. map_server used to be
    # its own always-on include here with its own lifecycle manager; it's now merged
    # into nav2_bringup.launch.py, so it's gated by enable_nav2 too.
    navigation_bringup = include(
        'f1tenth_navigation', 'nav2_bringup.launch.py',
        condition=is_nav2_enabled)
    # ---- BT safety-stop supervisor + Twist->Ackermann bridge (opt-in) -------
    behavior_bringup = include(
        'f1tenth_behavior', 'behavior_bringup.launch.py',
        condition=is_behavior_tree_enabled)

    # foxglove_bridge: generic dev/visualization tool, not owned by any
    # f1tenth_* package, so it's the one node kept inline here.
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
        }]
    )

    return LaunchDescription([
        camera_source_la,
        enable_safety_stop_la,
        use_behavior_tree_la,
        vesc_bringup,
        ackermann_mux_bringup,
        #mpc_bringup,
        ekf_bringup,
        sensor_tf_bringup,
        camera_bringup,
        detection_bringup,
        safety_stop_bringup,
        navigation_bringup,
        behavior_bringup,
        foxglove_bridge_node,
    ])
