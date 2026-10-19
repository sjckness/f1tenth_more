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

from f1tenth_params.param_defaults import get_default, get_value

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
    ('detection_margin_deg', float),
    ('detection_max_age_sec', float),
    ('wall_min_span_m', float),
    ('wall_planes_half_angle_deg', float),
    ('wall_planes_rate_hz', float),
    ('wall_planes_max_lines', int),
    ('wall_planes_min_inliers', int),
    ('virtual_disagree_m', float),
    ('stereo_max_age_sec', float),
    ('wall_tracker_front_max_coast_sec', float),
    # Step 3: the per-cycle LiDAR/stereo fusion.
    ('stereo_profile_max_age_s', float),
    ('stereo_profile_min_weight', float),
    ('wall_obs_tol_base_m', float),
    ('wall_obs_tol_quad_per_m2', float),
    ('wall_obs_disagree_reject_frac', float),
    ('wall_obs_stereo_min_columns', int),
    ('wall_obs_stereo_min_rho_m', float),
    ('wall_default_height_m', float),
    # Step 4: the tracker's observation model and FRONT rule.
    ('lidar_sigma_base_m', float),
    ('lidar_sigma_per_m', float),
    ('stereo_sigma_base_m', float),
    ('stereo_sigma_quad_per_m2', float),
    ('min_span_for_angle_m', float),
    ('stereo_confirm_hits', int),
    ('stereo_confirm_window', int),
    ('role_hysteresis_rad', float),
    ('stereo_front_max_normal_rad', float),
    # Block 3 (2026-10-16): the travel strip and what FRONT requires.
    ('front_strip_margin_m', float),
    ('front_min_confidence', float),
    ('front_lidar_recent_cycles', int),
    ('front_contradict_margin_m', float),
    ('front_contradict_cycles', int),
    ('front_clear_range_m', float),
    ('wall_marker_thickness_m', float),
    # Minimum width for a LiDAR wall (multi_wall_tracker's span gate).
    ('wall_min_span_birth_m', float),
    ('wall_min_span_update_m', float),
    # E1: the extraction's own quality gates.
    ('wall_extract_quality_gates', bool),
    ('wall_extract_backend', str),
    ('wall_tracker_scan_decimation', int),
    # cpu-cleanup R2: rate policy (EKF at 20 Hz, virtual wall on new estimates).
    ('pose_max_extrapolation_sec', float),
    ('virtual_wall_rate_hz', float),
    # E2: the rectangle verification's decision thresholds.
    ('wall_rect_verify', bool),
    ('wall_rect_coverage_min', float),
    ('wall_rect_support_verify', float),
    ('wall_rect_support_reject', float),
    ('wall_rect_glass_beyond_share', float),
    ('wall_rect_max_range_m', float),
    ('wall_debug_tracks', bool),
    ('wall_rect_reject_cycles', int),
    # E3: the geometric roles, the first obstacle, the negative evidence and
    # the two coasting limits.
    ('wall_role_back_x_m', float),
    ('wall_role_side_x_min_m', float),
    ('wall_role_side_x_max_m', float),
    ('wall_role_side_max_normal_deg', float),
    ('wall_role_hysteresis_m', float),
    ('wall_occlusion_margin_m', float),
    ('wall_negative_tol_m', float),
    ('wall_negative_min_beams', int),
    ('wall_negative_pass_frac', float),
    ('wall_negative_miss_to_kill', int),
    ('wall_tracker_delete_after_sec', float),
    ('wall_coast_max_odom_m', float),
    # Block 3: the measured extent and the FRONT coast on odometry. (The
    # collinear gap is door_width_min_m since door-detection.)
    ('wall_extent_decay_sec', float),
    ('wall_tracker_front_max_coast_m', float),
    # The confidence model's range scaling (wall_fusion.confidence, RANGE).
    ('wall_conf_range_ref_m', float),
    ('wall_conf_min_inlier_ref', float),
    ('wall_conf_max_rms_scale', float),
    # The camera observation gates and the camera side zone (Q2 / P3).
    ('camera_seg_min_ray_angle_deg', float),
    ('camera_seg_min_bearing_span_deg', float),
    ('stereo_max_rms_m', float),
    ('stereo_min_vertical_extent_m', float),
    ('stereo_min_length_m', float),
    ('camera_confirm_fresh_only', bool),
    ('camera_side_x_max_m', float),
    ('camera_side_max_normal_deg', float),
    ('camera_side_offset_min_m', float),
    ('camera_side_offset_max_m', float),
    # Door detection (perception only) and the fixed wall marker height.
    ('wall_viz_height_m', float),
    ('door_detection_enabled', bool),
    ('door_collinear_angle_deg', float),
    ('door_collinear_offset_m', float),
    ('door_width_min_m', float),
    ('door_width_max_m', float),
    ('door_behind_margin_m', float),
    ('door_min_beams', int),
    ('door_min_beam_fraction', float),
    ('door_confirm_hits', int),
    ('door_confirm_window', int),
    ('door_delete_after_sec', float),
    ('door_jamb_viz_diameter_m', float),
    # The open leaf and the stereo glass veto (door-detection step 2).
    ('door_core_fraction', float),
    ('door_leaf_max_len_m', float),
    ('door_leaf_hinge_tol_m', float),
    ('door_stereo_glass_veto', bool),
    ('door_stereo_plane_tol_m', float),
    ('door_stereo_wall_veto_frac', float),
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
    # The stack-wide scene-labels switch: a plain get_value like the MPC and
    # the BT read it, so the producer and its consumers cannot disagree.
    params['enable_scene_labels'] = bool(get_value('enable_scene_labels'))
    params['use_wall_tracker'] = bool(get_value('use_wall_tracker'))

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
