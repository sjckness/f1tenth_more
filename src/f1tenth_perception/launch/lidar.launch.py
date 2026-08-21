"""Hokuyo LiDAR (urg_node) -> /scan, frame `laser`.

Extracted out of perception.launch.py's inline urg_node block so
f1tenth_behavior/behavior_bringup.launch.py (which needs /scan for Nav2's costmap
obstacle layer) can reuse it without a third hand-copied params dict -- this file loads
f1tenth_bringup/config/sensors.yaml directly, the same shared-config convention
vesc.launch.py/ackermann_mux.launch.py already use.

perception.launch.py now includes this file directly instead of keeping its own
inline urg_node duplicate (previously a pre-existing, separate hand-copied params
dict with a hardcoded IP) -- resolved, single source of truth here.

lidar_boundary_node (left/right hard-boundary line fits for mpc_controller's
OSQP/RTI solver, raw /scan-based) previously launched here, gated behind the
same use_lidar condition as urg_node -- RETIRED by the dual-EKF + costmap-
derived-MPC-boundaries pass in favor of f1tenth_costmap's costmap_boundary_
node.py (nearest-occupied-cell extraction from slam_toolbox's own /slam/map,
front/left/right in one node -- see that node's own module docstring).
Deleted entirely here (node file, its 3 lidar_boundary_* stack_params.yaml
keys, this file's own launch-argument declarations/Node() registration, and
its own test file) -- confirmed via a codebase-wide grep before deleting
that nothing else held a functional dependency on it.
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

    urg_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[LaunchConfiguration('sensors_config')],
    )

    return LaunchDescription([
        use_lidar_arg, sensors_config_arg,
        urg_node,
    ])
