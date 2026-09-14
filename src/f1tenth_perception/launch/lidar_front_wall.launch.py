"""lidar_front_wall_node -> /perception/lidar_front_wall, /perception/lidar_front_wall_virtual.

THIS FILE IS ITS OWN SUPERVISOR COMPONENT ON PURPOSE. DO NOT MOVE THE NODE INTO
lidar.launch.py, AND DO NOT ADD THIS FILE TO THE `perception` COMPONENT.
-------------------------------------------------------------------------------
component_supervisor_node restarts a component by SIGINTing EVERY launch file
registered under it (_stop_component in component_supervisor_node.py). The
`perception` component owns lidar.launch.py; lidar.launch.py owns urg_node;
urg_node is the only publisher of /scan; and /scan is what the emergency stop,
f1tenth_behavior's IsProximityTooClose, reads. Anything that restarts this node
alongside urg_node -- a crash loop, a parameter retune, a routine restart --
would leave the e-stop without its sensor for as long as that restart takes.
As its own component (`lidar_front_wall` in f1tenth_bringup/config/
components.yaml) it restarts alone and urg_node never notices.

Folding this into lidar.launch.py looks like a tidy-up. It is the incident this
comment exists to prevent. test_lidar_front_wall_launch.py (this package) and
test_lidar_front_wall_component.py (f1tenth_bringup) fail if either placement
changes.

Gated on use_lidar, the same flag as urg_node: without /scan there is nothing
to fit.

Every lidar_front_wall_* default and its description comes from
f1tenth_params/config/stack_params.yaml, and the node's module docstring
carries the measurements behind them. Each is passed with an explicit
value_type rather than launch_ros's string inference, for the reason
detection.launch.py spells out: a CLI override like stale_age_sec:=3 would
otherwise arrive as an int and be rejected by the node's double parameter.

cpu_affinity/nice follow the stack's taskset-prefix convention (see
detection.launch.py's module docstring for why affinity is a launch prefix and
not self-pinning) and, being machine-specific, are not in stack_params.yaml.
Core 5 is the one core no pinned node reserves on this Jetson (EKFs 0,1,
slam_toolbox 2, foxglove 3, behavior_executor_node 4, detection_3d/obstacle
projector 6,7, YOLO 8,9, mpc_corr 10,11) -- and in particular not the behavior
executor's core 4, which hosts the e-stop.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name without the lidar_front_wall_ prefix, value type). The node takes the
# bare names; the prefix only exists to keep them unambiguous in
# stack_params.yaml's workspace-wide flat namespace.
LIDAR_FRONT_WALL_ARGS = [
    ('sector_half_angle_deg', float),
    ('inlier_distance_m', float),
    ('min_inlier_fraction', float),
    ('min_inlier_count_ratio', float),
    ('oblique_max_deg', float),
    ('max_hypothesis_pairs', int),
    ('wrong_surface_gate_m', float),
    ('stale_age_sec', float),
    ('odom_topic', str),
    ('odom_max_age_sec', float),
    ('seed_max_age_sec', float),
    ('seed_max_valid_m', float),
]


def generate_launch_description():
    use_lidar_default, use_lidar_desc = get_default('use_lidar')
    declared = [DeclareLaunchArgument(
        'use_lidar', default_value=str(use_lidar_default), description=use_lidar_desc)]

    params = {}
    for name, value_type in LIDAR_FRONT_WALL_ARGS:
        default, description = get_default('lidar_front_wall_' + name)
        declared.append(DeclareLaunchArgument(
            'lidar_front_wall_' + name, default_value=str(default),
            description=description))
        params[name] = ParameterValue(
            LaunchConfiguration('lidar_front_wall_' + name), value_type=value_type)

    declared.append(DeclareLaunchArgument(
        'lidar_front_wall_cpu_affinity', default_value='5',
        description="Comma-separated core ids to pin lidar_front_wall_node to via a "
                    "'taskset -c' launch prefix. Must stay a non-empty core list -- "
                    "'taskset -c' with none is a shell error, not a no-op; remove the "
                    "Node's prefix= argument instead to disable pinning. Keep it off "
                    "behavior_executor_node's core (4), which hosts the e-stop."))
    declared.append(DeclareLaunchArgument(
        'lidar_front_wall_nice', default_value='0',
        description='Process niceness for lidar_front_wall_node. 0: no-op.'))
    params['nice'] = ParameterValue(
        LaunchConfiguration('lidar_front_wall_nice'), value_type=int)

    node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='f1tenth_perception',
        executable='lidar_front_wall_node',
        name='lidar_front_wall_node',
        output='screen',
        prefix=['taskset -c ', LaunchConfiguration('lidar_front_wall_cpu_affinity')],
        parameters=[params],
    )

    return LaunchDescription([*declared, node])
