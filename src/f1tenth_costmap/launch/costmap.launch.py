"""Two-layer costmap bringup: semantic_layer_node.py (accumulates /camera/
detections_3d into a persistent map-frame semantic layer, using slam_
toolbox's own /slam/pose -- see that node's own module docstring) +
costmap_renderer_node.py (combines that layer with slam_toolbox's own
/slam/map into one rgb8 image for a Foxglove Image panel -- see that node's
own module docstring) + costmap_boundary_node.py (nearest-occupied-cell
hard boundary extraction from /slam/map + the global EKF's own map-frame
pose, retiring f1tenth_perception's wall_detector_node.py/lidar_boundary_
node.py -- see that node's own module docstring for the full design,
including the periodic-publish pass: publishes real boundaries/clearance
on every timer tick from whatever's currently cached, regardless of
upstream recency -- empty/withheld only if a message of that kind has
never arrived at all).

costmap_boundary_node's own launch args (front_facing_max_deg/side_window_
min_deg/side_window_max_deg/occupied_threshold/max_range_m/extraction_
rate_hz -- map_stale_timeout_sec/pose_stale_timeout_sec removed by the
periodic-publish pass, see that node's own module docstring: staleness no
longer gates the publish at all) follow this launch file's OWN existing
local convention -- plain hardcoded DeclareLaunchArgument
defaults (semantic_score_threshold/semantic_merge_distance_m/costmap_
render_rate_hz above), NOT stack_params.yaml-registered get_default() calls
the way most OTHER packages in this workspace do it -- for consistency
within this one file (mixing both conventions across three nodes in the
same launch file would be more confusing than either convention alone),
not an oversight of the codebase-wide pattern.

Gated on the SAME enable_slam flag slam.launch.py itself uses (not a second,
independent toggle) -- deliberately: both nodes here are structurally
dependent on slam_toolbox's own output (/slam/map, /slam/pose), so running
them with enable_slam:=false would just mean permanently-empty output, not
a meaningful independent on/off state worth its own flag. Bundled as a
second launches-list entry under components.yaml's own 'slam' component
(see that file) -- restarted together with slam.launch.py as one logical
unit, not a separate top-level component.

Same DeclareLaunchArgument + IfCondition mechanism slam.launch.py itself
uses for this exact flag (own default sourced from stack_params.yaml,
re-declared here rather than shared some other way -- launch has no
"already declared elsewhere" import across separate `ros2 launch` process
trees) -- NOT the plain get_value()-and-branch pattern detection.launch.py
still uses for its own is_zed gating (`get_value('camera_source') ==
'zed'`, a plain Python branch, not a re-declared/overridable launch
argument -- appropriate there since camera_source is already declared
once, upstream, by camera.launch.py itself). Deliberate, not copied
carelessly: a first
draft of this file used the plain-value pattern, which silently made
enable_slam NOT CLI-overridable on this file specifically (`ros2 launch
f1tenth_costmap costmap.launch.py enable_slam:=true` launched nothing,
caught live testing this pass) -- inconsistent with slam.launch.py's own
CLI-overridable behavior for what's meant to be the exact same flag. Fixed
by matching that file's mechanism exactly instead.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_default, enable_desc = get_default('enable_slam')
    enable_la = DeclareLaunchArgument(
        'enable_slam', default_value=str(enable_default), description=enable_desc)

    score_threshold_la = DeclareLaunchArgument(
        'semantic_score_threshold', default_value='0.5',
        description="Minimum vision_msgs detection score for semantic_layer_"
                    "node to accumulate a detection at all.")
    merge_distance_la = DeclareLaunchArgument(
        'semantic_merge_distance_m', default_value='0.5',
        description="Max distance (m, map frame) for a new same-class "
                    "detection to match an existing track's PREDICTED "
                    "position (real per-frame tracking pass) rather than "
                    "spawning a new one. Name kept from the pre-tracking-"
                    "pass merge_distance_m for launch-config continuity -- "
                    "same 'max association distance' concept, see "
                    "semantic_layer.py's own module docstring.")
    confirm_hit_count_la = DeclareLaunchArgument(
        'semantic_confirm_hit_count', default_value='3',
        description="Real per-frame tracking pass: a track only starts "
                    "publishing a marker once this many CONSECUTIVE frames "
                    "matched it -- keeps a one-off false detection from "
                    "ever becoming a lingering phantom object. Initial, "
                    "reasoned-but-not-yet-live-tuned value in units of "
                    "detection batches (roughly camera FPS), not seconds "
                    "-- see semantic_layer_node.py's own module docstring.")
    lost_miss_count_la = DeclareLaunchArgument(
        'semantic_lost_miss_count', default_value='5',
        description="Real per-frame tracking pass: a track is dropped "
                    "entirely once this many CONSECUTIVE frames failed to "
                    "match it (the 'is it still there' half of the "
                    "tracking ask). Same units/tuning-status caveat as "
                    "semantic_confirm_hit_count above.")
    render_rate_la = DeclareLaunchArgument(
        'costmap_render_rate_hz', default_value='2.0',
        description="Fixed periodic rate costmap_renderer_node re-renders "
                    "/costmap/visualization at -- see that node's own module "
                    "docstring for why a plain timer (not on-change) at the "
                    "low end of the task's own suggested 2-5Hz range.")

    # ---- costmap_boundary_node -- see that node's own module docstring for
    # the full reasoning behind each default below. ----
    boundary_front_max_la = DeclareLaunchArgument(
        'costmap_boundary_front_facing_max_deg', default_value='35.0',
        description="Half-angle (deg, car frame) of costmap_boundary_node's "
                    "front cone -- reuses wall_detector_node's own retired "
                    "front_facing_max_deg value/precedent.")
    boundary_side_min_la = DeclareLaunchArgument(
        'costmap_boundary_side_window_min_deg', default_value='45.0',
        description="Inner edge (deg, car frame) of costmap_boundary_node's "
                    "left/right windows -- reuses lidar_boundary_node's own "
                    "retired side_window_min_deg value/precedent.")
    boundary_side_max_la = DeclareLaunchArgument(
        'costmap_boundary_side_window_max_deg', default_value='135.0',
        description="Outer edge (deg, car frame) of costmap_boundary_node's "
                    "left/right windows -- reuses lidar_boundary_node's own "
                    "retired side_window_max_deg value/precedent.")
    boundary_occupied_threshold_la = DeclareLaunchArgument(
        'costmap_boundary_occupied_threshold', default_value='65',
        description="OccupancyGrid.data value (0-100) at/above which "
                    "costmap_boundary_node treats a cell as occupied -- "
                    "REASONED STARTING POINT, not tuned against real SLAM "
                    "output yet (see that node's own module docstring).")
    boundary_max_range_la = DeclareLaunchArgument(
        'costmap_boundary_max_range_m', default_value='5.0',
        description="Max search distance (m) costmap_boundary_node looks "
                    "for a nearest occupied cell per direction -- REASONED "
                    "STARTING POINT, roughly matching wall_detector_node's "
                    "own retired roi_x_max ballpark.")
    boundary_rate_la = DeclareLaunchArgument(
        'costmap_boundary_extraction_rate_hz', default_value='20.0',
        description="Periodic publish rate for costmap_boundary_node -- "
                    "bumped from 5.0 by the periodic-publish pass. This is "
                    "now the ONLY thing governing how often /costmap/"
                    "boundaries and /costmap/front_clearance are published "
                    "-- see that node's own module docstring's 'PERIODIC-"
                    "PUBLISH pass' paragraph for why upstream (/slam/map, "
                    "/ekf_global/odometry/filtered) recency no longer gates "
                    "the publish at all.")

    semantic_layer_node = Node(
        package='f1tenth_costmap',
        executable='semantic_layer_node',
        name='semantic_layer_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam')),
        parameters=[{
            'score_threshold': LaunchConfiguration('semantic_score_threshold'),
            'merge_distance_m': LaunchConfiguration('semantic_merge_distance_m'),
            'confirm_hit_count': LaunchConfiguration('semantic_confirm_hit_count'),
            'lost_miss_count': LaunchConfiguration('semantic_lost_miss_count'),
        }],
    )

    costmap_renderer_node = Node(
        package='f1tenth_costmap',
        executable='costmap_renderer_node',
        name='costmap_renderer_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam')),
        parameters=[{
            'render_rate_hz': LaunchConfiguration('costmap_render_rate_hz'),
        }],
    )

    # Gated the SAME way as the two nodes above -- enable_slam, not a
    # separate toggle -- for the identical reason: this node is
    # structurally dependent on /slam/map existing at all (see its own
    # module docstring).
    costmap_boundary_node = Node(
        package='f1tenth_costmap',
        executable='costmap_boundary_node',
        name='costmap_boundary_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam')),
        parameters=[{
            'front_facing_max_deg': LaunchConfiguration(
                'costmap_boundary_front_facing_max_deg'),
            'side_window_min_deg': LaunchConfiguration(
                'costmap_boundary_side_window_min_deg'),
            'side_window_max_deg': LaunchConfiguration(
                'costmap_boundary_side_window_max_deg'),
            'occupied_threshold': LaunchConfiguration(
                'costmap_boundary_occupied_threshold'),
            'max_range_m': LaunchConfiguration('costmap_boundary_max_range_m'),
            'extraction_rate_hz': LaunchConfiguration(
                'costmap_boundary_extraction_rate_hz'),
        }],
    )

    return LaunchDescription([
        enable_la, score_threshold_la, merge_distance_la,
        confirm_hit_count_la, lost_miss_count_la, render_rate_la,
        boundary_front_max_la, boundary_side_min_la, boundary_side_max_la,
        boundary_occupied_threshold_la, boundary_max_range_la, boundary_rate_la,
        semantic_layer_node, costmap_renderer_node, costmap_boundary_node,
    ])
