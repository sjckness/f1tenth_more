"""YOLO 2D detector + 2D-to-3D detection fusion, plus front_depth_monitor_node
and wall_detector_node.

Source-agnostic: subscribes to the canonical /camera/image_raw (published by
whichever camera stack_bringup brought up) and publishes vision_msgs
Detection2DArray/Detection3DArray + a MarkerArray for Foxglove/RViz.

detection_3d_node/obstacle_projector_node/front_depth_monitor_node/
wall_detector_node all need ZED depth or the ZED point cloud, which only
exist in ZED mode, so they're only built at all when camera_source == 'zed'
-- camera_source is one of the 6 stack-wide branching args (see
f1tenth_params/config/stack_params.yaml), a plain Python value here, not a
DeclareLaunchArgument/LaunchConfiguration.

front_depth_monitor_node is deliberately unrelated to the YOLO/detection
pipeline above it in this file -- it reads raw ZED depth directly, with no
dependency on yolo_detector_node/detection_3d_node's output, by design (see
its own docstring: it's the front half of f1tenth_behavior's IsProximityTooClose
last-resort emergency-stop condition, which must keep working even if YOLO is
degraded). It lives in this file anyway rather than a separate one because the
is_zed gating condition is identical and this package doesn't otherwise split
one launch file per node.

wall_detector_node is likewise independent of the YOLO/detection_3d/
obstacle_projector chain -- it consumes the ZED point cloud directly (RANSAC
plane segmentation for wall/corner detection, f1tenth_messages/WallArray on
/perception/wall_detections, Float32 on /perception/front_clearance for the
mission runtime's front_clearance stop_condition, MarkerArray on
/perception/wall_markers for Foxglove). Additionally gated on
enable_wall_detector (a plain stack_params.yaml value, same as
enable_sys_obs/enable_foxglove) -- see that key's own comment.

cpu_affinity/nice args (perception-optimization pass, following the latency
audit that root-caused /camera/detections' ~278ms lag behind /camera/image_raw
to CPU/executor contention, not transport/QoS): one pair per node, same
mechanism as mpc_controller/MPC_corr.py's own cpu_affinity param (see
f1tenth_perception/cpu_affinity.py, shared by all three nodes here). Not
sourced from stack_params.yaml -- like mpc_corr.launch.py's own cpu_affinity,
the right core ids are machine-specific, not a stack-wide default. Defaults
below reserve yolo_detector_node (clearest contention signature in the audit)
its own pair, and detection_3d_node/obstacle_projector_node (less severe:
75-82% and ~56-59% CPU with no pinning) a shared "perception" pair -- both
away from mpc_corr's own 10,11. Empty/0 (the underlying params' own defaults)
are no-ops, matching every other cpu_affinity/nice arg in this workspace.
"""

import os

from f1tenth_params.param_defaults import get_default, get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # src/ dir: src/f1tenth_perception/launch/<this file> -> ../.. -> src/.
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__))))

    confidence_threshold_default, confidence_threshold_desc = get_default(
        'confidence_threshold')
    confidence_threshold_la = DeclareLaunchArgument(
        'confidence_threshold', default_value=str(confidence_threshold_default),
        description=confidence_threshold_desc)
    yolo_device_default, yolo_device_desc = get_default('yolo_device')
    yolo_device_la = DeclareLaunchArgument(
        'yolo_device', default_value=str(yolo_device_default), description=yolo_device_desc)
    yolo_model_default, yolo_model_desc = get_default('yolo_model')
    yolo_model_la = DeclareLaunchArgument(
        'yolo_model', default_value=str(yolo_model_default), description=yolo_model_desc)
    obstacle_z_min_default, obstacle_z_min_desc = get_default('obstacle_z_min')
    obstacle_z_min_la = DeclareLaunchArgument(
        'obstacle_z_min', default_value=str(obstacle_z_min_default),
        description=obstacle_z_min_desc)
    obstacle_z_max_default, obstacle_z_max_desc = get_default('obstacle_z_max')
    obstacle_z_max_la = DeclareLaunchArgument(
        'obstacle_z_max', default_value=str(obstacle_z_max_default),
        description=obstacle_z_max_desc)

    # wall_detector_node (RANSAC plane segmentation, is_zed-gated below same as
    # obstacle_z_min/_max above) -- see stack_params.yaml's wall_* keys for the
    # full reasoning behind each default.
    wall_input_topic_default, wall_input_topic_desc = get_default('wall_input_topic')
    wall_input_topic_la = DeclareLaunchArgument(
        'wall_input_topic', default_value=str(wall_input_topic_default),
        description=wall_input_topic_desc)
    wall_roi_x_max_default, wall_roi_x_max_desc = get_default('wall_roi_x_max')
    wall_roi_x_max_la = DeclareLaunchArgument(
        'wall_roi_x_max', default_value=str(wall_roi_x_max_default),
        description=wall_roi_x_max_desc)
    wall_roi_y_half_width_default, wall_roi_y_half_width_desc = get_default(
        'wall_roi_y_half_width')
    wall_roi_y_half_width_la = DeclareLaunchArgument(
        'wall_roi_y_half_width', default_value=str(wall_roi_y_half_width_default),
        description=wall_roi_y_half_width_desc)
    wall_verticality_max_deg_default, wall_verticality_max_deg_desc = get_default(
        'wall_verticality_max_deg')
    wall_verticality_max_deg_la = DeclareLaunchArgument(
        'wall_verticality_max_deg', default_value=str(wall_verticality_max_deg_default),
        description=wall_verticality_max_deg_desc)
    wall_front_facing_max_deg_default, wall_front_facing_max_deg_desc = get_default(
        'wall_front_facing_max_deg')
    wall_front_facing_max_deg_la = DeclareLaunchArgument(
        'wall_front_facing_max_deg', default_value=str(wall_front_facing_max_deg_default),
        description=wall_front_facing_max_deg_desc)
    # front_clearance selection eligibility -- see wall_detector_node.py's own
    # module docstring and stack_params.yaml's own comments for the full
    # investigation writeup behind these two.
    (wall_front_clearance_min_track_frames_default,
     wall_front_clearance_min_track_frames_desc) = get_default(
        'wall_front_clearance_min_track_frames')
    wall_front_clearance_min_track_frames_la = DeclareLaunchArgument(
        'wall_front_clearance_min_track_frames',
        default_value=str(wall_front_clearance_min_track_frames_default),
        description=wall_front_clearance_min_track_frames_desc)
    wall_front_clearance_min_inliers_default, wall_front_clearance_min_inliers_desc = (
        get_default('wall_front_clearance_min_inliers'))
    wall_front_clearance_min_inliers_la = DeclareLaunchArgument(
        'wall_front_clearance_min_inliers',
        default_value=str(wall_front_clearance_min_inliers_default),
        description=wall_front_clearance_min_inliers_desc)
    wall_corner_perp_tolerance_deg_default, wall_corner_perp_tolerance_deg_desc = get_default(
        'wall_corner_perp_tolerance_deg')
    wall_corner_perp_tolerance_deg_la = DeclareLaunchArgument(
        'wall_corner_perp_tolerance_deg',
        default_value=str(wall_corner_perp_tolerance_deg_default),
        description=wall_corner_perp_tolerance_deg_desc)

    # Plane merge (duplicate/near-coplanar RANSAC split fix) + per-wall
    # tracking/EMA smoothing (frame-to-frame jitter fix) -- two independent
    # fixes, see wall_detector_node.py's own module docstring for the full
    # pipeline description and stack_params.yaml for the reasoning behind
    # each default.
    wall_merge_normal_cos_thresh_default, wall_merge_normal_cos_thresh_desc = get_default(
        'wall_merge_normal_cos_thresh')
    wall_merge_normal_cos_thresh_la = DeclareLaunchArgument(
        'wall_merge_normal_cos_thresh',
        default_value=str(wall_merge_normal_cos_thresh_default),
        description=wall_merge_normal_cos_thresh_desc)
    wall_merge_distance_thresh_m_default, wall_merge_distance_thresh_m_desc = get_default(
        'wall_merge_distance_thresh_m')
    wall_merge_distance_thresh_m_la = DeclareLaunchArgument(
        'wall_merge_distance_thresh_m',
        default_value=str(wall_merge_distance_thresh_m_default),
        description=wall_merge_distance_thresh_m_desc)
    wall_min_distinct_separation_m_default, wall_min_distinct_separation_m_desc = get_default(
        'wall_min_distinct_separation_m')
    wall_min_distinct_separation_m_la = DeclareLaunchArgument(
        'wall_min_distinct_separation_m',
        default_value=str(wall_min_distinct_separation_m_default),
        description=wall_min_distinct_separation_m_desc)
    wall_ambiguous_confirm_frames_default, wall_ambiguous_confirm_frames_desc = get_default(
        'wall_ambiguous_confirm_frames')
    wall_ambiguous_confirm_frames_la = DeclareLaunchArgument(
        'wall_ambiguous_confirm_frames',
        default_value=str(wall_ambiguous_confirm_frames_default),
        description=wall_ambiguous_confirm_frames_desc)
    wall_track_assoc_distance_thresh_m_default, wall_track_assoc_distance_thresh_m_desc = (
        get_default('wall_track_assoc_distance_thresh_m'))
    wall_track_assoc_distance_thresh_m_la = DeclareLaunchArgument(
        'wall_track_assoc_distance_thresh_m',
        default_value=str(wall_track_assoc_distance_thresh_m_default),
        description=wall_track_assoc_distance_thresh_m_desc)
    wall_track_assoc_bearing_thresh_deg_default, wall_track_assoc_bearing_thresh_deg_desc = (
        get_default('wall_track_assoc_bearing_thresh_deg'))
    wall_track_assoc_bearing_thresh_deg_la = DeclareLaunchArgument(
        'wall_track_assoc_bearing_thresh_deg',
        default_value=str(wall_track_assoc_bearing_thresh_deg_default),
        description=wall_track_assoc_bearing_thresh_deg_desc)
    wall_track_hold_frames_default, wall_track_hold_frames_desc = get_default(
        'wall_track_hold_frames')
    wall_track_hold_frames_la = DeclareLaunchArgument(
        'wall_track_hold_frames', default_value=str(wall_track_hold_frames_default),
        description=wall_track_hold_frames_desc)
    wall_ema_alpha_default, wall_ema_alpha_desc = get_default('wall_ema_alpha')
    wall_ema_alpha_la = DeclareLaunchArgument(
        'wall_ema_alpha', default_value=str(wall_ema_alpha_default),
        description=wall_ema_alpha_desc)

    # Confidence-based pruning -- see wall_detector_node.py's own module
    # docstring ('Confidence-based pruning' section) and stack_params.yaml's
    # own comments for the full reasoning behind each default.
    wall_prune_conflict_radius_m_default, wall_prune_conflict_radius_m_desc = get_default(
        'wall_prune_conflict_radius_m')
    wall_prune_conflict_radius_m_la = DeclareLaunchArgument(
        'wall_prune_conflict_radius_m',
        default_value=str(wall_prune_conflict_radius_m_default),
        description=wall_prune_conflict_radius_m_desc)
    (wall_prune_min_frames_before_eligible_default,
     wall_prune_min_frames_before_eligible_desc) = get_default(
        'wall_prune_min_frames_before_eligible')
    wall_prune_min_frames_before_eligible_la = DeclareLaunchArgument(
        'wall_prune_min_frames_before_eligible',
        default_value=str(wall_prune_min_frames_before_eligible_default),
        description=wall_prune_min_frames_before_eligible_desc)
    wall_prune_inlier_ratio_floor_default, wall_prune_inlier_ratio_floor_desc = get_default(
        'wall_prune_inlier_ratio_floor')
    wall_prune_inlier_ratio_floor_la = DeclareLaunchArgument(
        'wall_prune_inlier_ratio_floor',
        default_value=str(wall_prune_inlier_ratio_floor_default),
        description=wall_prune_inlier_ratio_floor_desc)
    (wall_prune_match_streak_ratio_floor_default,
     wall_prune_match_streak_ratio_floor_desc) = get_default(
        'wall_prune_match_streak_ratio_floor')
    wall_prune_match_streak_ratio_floor_la = DeclareLaunchArgument(
        'wall_prune_match_streak_ratio_floor',
        default_value=str(wall_prune_match_streak_ratio_floor_default),
        description=wall_prune_match_streak_ratio_floor_desc)

    # cpu_affinity/nice, one pair per node -- see module docstring.
    yolo_cpu_affinity_la = DeclareLaunchArgument(
        'yolo_cpu_affinity', default_value='8,9',
        description="Comma-separated core ids to pin yolo_detector_node to. "
                    "Empty: inherit the OS default affinity (no-op).")
    yolo_nice_la = DeclareLaunchArgument(
        'yolo_nice', default_value='0',
        description="Process niceness for yolo_detector_node. 0: no-op.")
    detection_3d_cpu_affinity_la = DeclareLaunchArgument(
        'detection_3d_cpu_affinity', default_value='6,7',
        description="Comma-separated core ids to pin detection_3d_node to. "
                    "Empty: inherit the OS default affinity (no-op).")
    detection_3d_nice_la = DeclareLaunchArgument(
        'detection_3d_nice', default_value='0',
        description="Process niceness for detection_3d_node. 0: no-op.")
    obstacle_projector_cpu_affinity_la = DeclareLaunchArgument(
        'obstacle_projector_cpu_affinity', default_value='6,7',
        description="Comma-separated core ids to pin obstacle_projector_node "
                    "to. Empty: inherit the OS default affinity (no-op).")
    obstacle_projector_nice_la = DeclareLaunchArgument(
        'obstacle_projector_nice', default_value='0',
        description="Process niceness for obstacle_projector_node. 0: no-op.")

    is_zed = get_value('camera_source') == 'zed'

    yolo_detector_node = Node(
        package='f1tenth_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'detections_topic': '/camera/detections',
            'annotated_topic': '/camera/image_annotated',
            'model_path': PathJoinSubstitution([
                src_dir, 'f1tenth_perception', 'models',
                LaunchConfiguration('yolo_model')]),
            'device': LaunchConfiguration('yolo_device'),
            # Previously only reached detection_3d_node below -- see
            # yolo_detector_node.py's own docstring for why that left this
            # node's own output unfiltered regardless of this value.
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'cpu_affinity': LaunchConfiguration('yolo_cpu_affinity'),
            'nice': LaunchConfiguration('yolo_nice'),
        }],
    )

    actions = [
        confidence_threshold_la, yolo_device_la, yolo_model_la,
        obstacle_z_min_la, obstacle_z_max_la,
        wall_input_topic_la, wall_roi_x_max_la, wall_roi_y_half_width_la,
        wall_verticality_max_deg_la, wall_front_facing_max_deg_la,
        wall_front_clearance_min_track_frames_la, wall_front_clearance_min_inliers_la,
        wall_corner_perp_tolerance_deg_la,
        wall_merge_normal_cos_thresh_la, wall_merge_distance_thresh_m_la,
        wall_min_distinct_separation_m_la, wall_ambiguous_confirm_frames_la,
        wall_track_assoc_distance_thresh_m_la, wall_track_assoc_bearing_thresh_deg_la,
        wall_track_hold_frames_la, wall_ema_alpha_la,
        wall_prune_conflict_radius_m_la, wall_prune_min_frames_before_eligible_la,
        wall_prune_inlier_ratio_floor_la, wall_prune_match_streak_ratio_floor_la,
        yolo_cpu_affinity_la, yolo_nice_la,
        detection_3d_cpu_affinity_la, detection_3d_nice_la,
        obstacle_projector_cpu_affinity_la, obstacle_projector_nice_la,
        yolo_detector_node,
    ]

    if is_zed:
        actions.append(Node(
            package='f1tenth_perception',
            executable='detection_3d_node',
            name='detection_3d_node',
            output='screen',
            parameters=[{
                'detections_topic': '/camera/detections',
                'depth_topic': '/zed2/zed_node/depth/depth_registered',
                'depth_info_topic': '/zed2/zed_node/depth/camera_info',
                'detections_3d_topic': '/camera/detections_3d',
                'markers_topic': '/camera/detection_markers',
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'cpu_affinity': LaunchConfiguration('detection_3d_cpu_affinity'),
                'nice': LaunchConfiguration('detection_3d_nice'),
            }],
        ))

        # 3D detections -> 2D obstacles for mpc_controller's MPC_corr.py. Depends
        # on detection_3d_node's own output, so it lives behind the same is_zed
        # gate (see obstacle_projector_node.py's own docstring for the tf2/frame
        # rationale).
        actions.append(Node(
            package='f1tenth_perception',
            executable='obstacle_projector_node',
            name='obstacle_projector_node',
            output='screen',
            parameters=[{
                'detections_3d_topic': '/camera/detections_3d',
                'obstacles_topic': '/perception/obstacles_2d',
                'output_frame': 'base_link',
                'obstacle_z_min': LaunchConfiguration('obstacle_z_min'),
                'obstacle_z_max': LaunchConfiguration('obstacle_z_max'),
                'cpu_affinity': LaunchConfiguration('obstacle_projector_cpu_affinity'),
                'nice': LaunchConfiguration('obstacle_projector_nice'),
            }],
        ))

        # See module docstring: independent of the YOLO/detection nodes above
        # despite living in this file -- last-resort proximity input for
        # f1tenth_behavior's IsProximityTooClose (emergency lane), and, as a
        # side effect of finally having a real publisher, MPC_corr.py's own
        # front_distance-based corridor-length logic.
        actions.append(Node(
            package='f1tenth_perception',
            executable='front_depth_monitor_node',
            name='front_depth_monitor_node',
            output='screen',
            parameters=[{
                'depth_topic': '/zed2/zed_node/depth/depth_registered',
                'front_distance_topic': '/perception/front_distance',
            }],
        ))

        # RANSAC plane segmentation for wall/corner detection -- consumes the
        # ZED point cloud directly (is_zed-gated for that reason, same as the
        # three nodes above), independent of the YOLO/detection_3d/obstacle_
        # projector chain. enable_wall_detector is a plain stack_params.yaml
        # value read here, not a DeclareLaunchArgument -- same pattern as
        # enable_sys_obs/enable_foxglove (see stack_params.yaml's own comment
        # on that key): opts out of the node entirely rather than a no-op
        # runtime flag on it.
        if get_value('enable_wall_detector'):
            actions.append(Node(
                package='f1tenth_perception',
                executable='wall_detector_node',
                name='wall_detector_node',
                output='screen',
                parameters=[{
                    'input_topic': LaunchConfiguration('wall_input_topic'),
                    'roi_x_max': LaunchConfiguration('wall_roi_x_max'),
                    'roi_y_half_width': LaunchConfiguration('wall_roi_y_half_width'),
                    'verticality_max_deg': LaunchConfiguration('wall_verticality_max_deg'),
                    'front_facing_max_deg': LaunchConfiguration('wall_front_facing_max_deg'),
                    'front_clearance_min_track_frames': LaunchConfiguration(
                        'wall_front_clearance_min_track_frames'),
                    'front_clearance_min_inliers': LaunchConfiguration(
                        'wall_front_clearance_min_inliers'),
                    'corner_perp_tolerance_deg': LaunchConfiguration(
                        'wall_corner_perp_tolerance_deg'),
                    'merge_normal_cos_thresh': LaunchConfiguration(
                        'wall_merge_normal_cos_thresh'),
                    'merge_distance_thresh_m': LaunchConfiguration(
                        'wall_merge_distance_thresh_m'),
                    'min_distinct_separation_m': LaunchConfiguration(
                        'wall_min_distinct_separation_m'),
                    'ambiguous_confirm_frames': LaunchConfiguration(
                        'wall_ambiguous_confirm_frames'),
                    'track_assoc_distance_thresh_m': LaunchConfiguration(
                        'wall_track_assoc_distance_thresh_m'),
                    'track_assoc_bearing_thresh_deg': LaunchConfiguration(
                        'wall_track_assoc_bearing_thresh_deg'),
                    'track_hold_frames': LaunchConfiguration('wall_track_hold_frames'),
                    'ema_alpha': LaunchConfiguration('wall_ema_alpha'),
                    'prune_conflict_radius_m': LaunchConfiguration(
                        'wall_prune_conflict_radius_m'),
                    'prune_min_frames_before_eligible': LaunchConfiguration(
                        'wall_prune_min_frames_before_eligible'),
                    'prune_inlier_ratio_floor': LaunchConfiguration(
                        'wall_prune_inlier_ratio_floor'),
                    'prune_match_streak_ratio_floor': LaunchConfiguration(
                        'wall_prune_match_streak_ratio_floor'),
                }],
            ))

    return LaunchDescription(actions)
