"""wall_distance_node -> /perception/d_wall, /perception/d_wall/segment,
/perception/d_wall/psi_correction, /perception/d_wall/gate_margin,
/perception/d_wall/swept_arc.

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
As its own component (`wall_distance` in f1tenth_bringup/config/components.yaml)
it restarts alone and urg_node never notices.

This node is a MORE likely restart candidate than lidar_front_wall, not less:
it owns the glass detector in-process, it is brand new, and it is the one whose
gates will be retuned during the first powered session. Folding it in looks like
a tidy-up; it is the incident lidar_front_wall.launch.py's comment exists to
prevent, and test_wall_distance_launch.py (this package) and
test_wall_distance_component.py (f1tenth_bringup) fail if either placement
changes.

Gated on use_lidar, the same flag as urg_node: without /scan there is nothing to
fit.

THREE PARAMETER GROUPS, THREE PREFIXES, AND THE REASON FOR EACH
---------------------------------------------------------------
  wall_distance_*   this node's own keys: topics, the phase machine, the coast
                    caps, the control law.
  glass_*           glass_detect.py's detector gates, assembled here into a
                    DetectorConfig and a GlassTracker. They carry the glass_
                    prefix rather than wall_distance_ because DetectorConfig's
                    docstring has always promised "stack_params.yaml's glass_*
                    values" -- until this pass that was false, there were zero
                    such keys and every gate was a Python default. The prefix
                    keeps them reusable if glass_detect ever gets a node of its
                    own; it has none today.
  swept_clearance_* the FOOTPRINT, forwarded to this node under swept_* names.
                    Deliberately NOT a second copy: the swept_arc debug value
                    runs the same swept_corridor.clearance() on the same
                    rectangle swept_clearance_node uses, and a car with two
                    footprints in two nodes is a car whose clearances disagree
                    for reasons nobody can find. One source, two readers.

Each is passed with an explicit value_type rather than launch_ros's string
inference, for the reason detection.launch.py spells out: a CLI override like
publish_rate_hz:=10 would otherwise arrive as an int and be rejected by the
node's double parameter.

cpu_affinity/nice follow the stack's taskset-prefix convention (see
detection.launch.py's module docstring for why affinity is a launch prefix and
not self-pinning) and, being machine-specific, are not in stack_params.yaml.
Core 5 is the one core no pinned node reserves on this Jetson (EKFs 0,1,
slam_toolbox 2, foxglove 3, behavior_executor_node 4, detection_3d/obstacle
projector 6,7, YOLO 8,9, mpc_corr 10,11) -- and in particular NOT the behavior
executor's core 4, which hosts the e-stop. It is shared with
lidar_front_wall_node and swept_clearance_node, and this node is the heaviest of
the three because it runs the glass detector per scan: core 5 contention is on
the UNVALIDATED list in f1tenth_perception/README.md, to be measured in Stage 0
of docs/bringup_checklist.md.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name without the wall_distance_ prefix, value type). The node takes the bare
# names; the prefix only exists to keep them unambiguous in stack_params.yaml's
# workspace-wide flat namespace.
WALL_DISTANCE_ARGS = [
    ('scan_topic', str),
    ('odom_topic', str),
    ('odom_max_age_sec', float),
    ('wall_track_topic', str),
    ('output_topic', str),
    ('publish_rate_hz', float),
    ('wall_track_silence_ticks', int),
    ('match_endpoint_tol', float),
    ('match_angle_tol_deg', float),
    ('max_coast_distance', float),
    ('max_coast_yaw', float),
    ('fade_start_age', float),
    ('fade_zero_age', float),
    ('stale_inflate_per_s', float),
    ('d_ref', float),
    ('convergence_length_m', float),
    ('max_psi_correction', float),
    ('max_psi_rate', float),
    ('deadband_floor', float),
    ('deadband_k', float),
    ('fov_half_angle_rad', float),
    ('expect_return_tol_deg', float),
    ('max_range_m', float),
    ('persistence_window', int),
    ('persistence_hits', int),
    ('geometry_only_multiplier', int),
    ('track_timeout_sec', float),
    ('geom_enable', bool),
    ('geom_min_inliers', int),
    ('geom_min_span_m', float),
    ('geom_min_distance_m', float),
    ('geom_max_candidates', int),
    ('geom_inlier_distance_m', float),
]

# (name without the glass_ prefix, value type). Forwarded under the bare name,
# which is also the DetectorConfig field name -- wall_distance_node builds the
# dataclass straight from them.
GLASS_ARGS = [
    ('cos_theta_floor', float),
    ('normal_window', int),
    ('intensity_range_exponent', float),
    ('spike_factor', float),
    ('spike_window', int),
    ('max_spike_width', int),
    ('min_gradient_ratio', float),
    ('min_void_beams', int),
    ('void_range_jump', float),
    ('support_window_beams', int),
    ('see_through_max_gap_m', float),
    ('min_line_inliers', int),
    ('line_inlier_dist', float),
    ('min_segment_length', float),
    ('max_segment_length', float),
    ('min_range_m', float),
    ('normal_tolerance_deg', float),
    ('on_line_min_beams', int),
    ('geom_min_length_m', float),
    ('geom_min_voids', int),
    ('max_candidates', int),
    ('point_spacing', float),
]

# stack_params key -> the node's own parameter name. The footprint, shared with
# swept_clearance_node rather than duplicated. See the module docstring.
SWEPT_GEOMETRY_ARGS = [
    ('swept_clearance_wheelbase_m', 'swept_wheelbase_m', float),
    ('swept_clearance_body_front_x_m', 'swept_body_front_x_m', float),
    ('swept_clearance_body_rear_x_m', 'swept_body_rear_x_m', float),
    ('swept_clearance_body_half_width_m', 'swept_body_half_width_m', float),
    ('swept_clearance_margin_m', 'swept_margin_m', float),
    ('swept_clearance_max_range_m', 'swept_max_range_m', float),
    ('swept_clearance_absolute_min_clearance_m', 'swept_absolute_min_clearance_m', float),
    ('swept_clearance_rear_axle_x_m', 'swept_rear_axle_x_m', float),
]


def generate_launch_description():
    use_lidar_default, use_lidar_desc = get_default('use_lidar')
    declared = [DeclareLaunchArgument(
        'use_lidar', default_value=str(use_lidar_default), description=use_lidar_desc)]

    params = {}
    for prefix, arg_list in (('wall_distance_', WALL_DISTANCE_ARGS),
                             ('glass_', GLASS_ARGS)):
        for name, value_type in arg_list:
            default, description = get_default(prefix + name)
            declared.append(DeclareLaunchArgument(
                prefix + name, default_value=str(default), description=description))
            params[name] = ParameterValue(
                LaunchConfiguration(prefix + name), value_type=value_type)

    # The footprint keys keep their swept_clearance_ launch-arg names, so a CLI
    # override moves BOTH nodes at once rather than silently desynchronising
    # them. They are declared here too because a launch file that reads an
    # argument it never declared cannot be overridden on the command line at
    # all, and this file can be launched on its own.
    for key, node_name, value_type in SWEPT_GEOMETRY_ARGS:
        default, description = get_default(key)
        declared.append(DeclareLaunchArgument(
            key, default_value=str(default), description=description))
        params[node_name] = ParameterValue(
            LaunchConfiguration(key), value_type=value_type)

    declared.append(DeclareLaunchArgument(
        'wall_distance_cpu_affinity', default_value='5',
        description="Comma-separated core ids to pin wall_distance_node to via a "
                    "'taskset -c' launch prefix. Must stay a non-empty core list -- "
                    "'taskset -c' with none is a shell error, not a no-op; remove the "
                    "Node's prefix= argument instead to disable pinning. Keep it off "
                    "behavior_executor_node's core (4), which hosts the e-stop."))
    declared.append(DeclareLaunchArgument(
        'wall_distance_nice', default_value='0',
        description='Process niceness for wall_distance_node. 0: no-op.'))
    params['nice'] = ParameterValue(
        LaunchConfiguration('wall_distance_nice'), value_type=int)

    node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='f1tenth_perception',
        executable='wall_distance_node',
        name='wall_distance_node',
        output='screen',
        prefix=['taskset -c ', LaunchConfiguration('wall_distance_cpu_affinity')],
        parameters=[params],
    )

    return LaunchDescription([*declared, node])
