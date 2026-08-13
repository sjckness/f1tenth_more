"""Hokuyo LiDAR (urg_node) -> /scan, frame `laser`.

Extracted out of perception.launch.py's inline urg_node block so
f1tenth_behavior/behavior_bringup.launch.py (which needs /scan for Nav2's costmap
obstacle layer) can reuse it without a third hand-copied params dict -- this file loads
f1tenth_bringup/config/sensors.yaml directly, the same shared-config convention
vesc.launch.py/ackermann_mux.launch.py already use.

perception.launch.py now includes this file directly instead of keeping its own
inline urg_node duplicate (previously a pre-existing, separate hand-copied params
dict with a hardcoded IP) -- resolved, single source of truth here.

Also launches lidar_boundary_node (left/right hard-boundary line fits for
mpc_controller's OSQP/RTI solver -- see that node's own module docstring),
gated behind the SAME use_lidar condition as urg_node itself (structurally
needs /scan, no lidar = nothing to fit). Temporal-tracking parameters
(ema_alpha/min_frames_to_publish/hold_frames -- see lidar_boundary_node's
own "Temporal tracking" docstring section) are wired here from
stack_params.yaml's lidar_boundary_* keys, added by the "Boundary detection
hardening" pass -- previously this Node() launched with no parameters=[...]
at all, so every value was that node's own in-code declare_parameter
default; now overridable the same way wall_detector_node's own params
already are (see detection.launch.py).
"""

from f1tenth_params.param_defaults import get_default, get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensors_config, sensors_config_desc = get_path_default('sensors_config')
    use_lidar_default, use_lidar_desc = get_default('use_lidar')

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value=str(use_lidar_default), description=use_lidar_desc)
    sensors_config_arg = DeclareLaunchArgument(
        'sensors_config', default_value=sensors_config, description=sensors_config_desc)

    lidar_boundary_ema_alpha_default, lidar_boundary_ema_alpha_desc = get_default(
        'lidar_boundary_ema_alpha')
    lidar_boundary_ema_alpha_la = DeclareLaunchArgument(
        'lidar_boundary_ema_alpha', default_value=str(lidar_boundary_ema_alpha_default),
        description=lidar_boundary_ema_alpha_desc)
    lidar_boundary_min_frames_to_publish_default, lidar_boundary_min_frames_to_publish_desc = (
        get_default('lidar_boundary_min_frames_to_publish'))
    lidar_boundary_min_frames_to_publish_la = DeclareLaunchArgument(
        'lidar_boundary_min_frames_to_publish',
        default_value=str(lidar_boundary_min_frames_to_publish_default),
        description=lidar_boundary_min_frames_to_publish_desc)
    lidar_boundary_hold_frames_default, lidar_boundary_hold_frames_desc = get_default(
        'lidar_boundary_hold_frames')
    lidar_boundary_hold_frames_la = DeclareLaunchArgument(
        'lidar_boundary_hold_frames', default_value=str(lidar_boundary_hold_frames_default),
        description=lidar_boundary_hold_frames_desc)

    urg_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[LaunchConfiguration('sensors_config')],
    )

    lidar_boundary_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='f1tenth_perception',
        executable='lidar_boundary_node',
        name='lidar_boundary_node',
        output='screen',
        parameters=[{
            'ema_alpha': LaunchConfiguration('lidar_boundary_ema_alpha'),
            'min_frames_to_publish': LaunchConfiguration(
                'lidar_boundary_min_frames_to_publish'),
            'hold_frames': LaunchConfiguration('lidar_boundary_hold_frames'),
        }],
    )

    return LaunchDescription([
        use_lidar_arg, sensors_config_arg,
        lidar_boundary_ema_alpha_la, lidar_boundary_min_frames_to_publish_la,
        lidar_boundary_hold_frames_la,
        urg_node, lidar_boundary_node,
    ])
