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
never arrived at all) + freespace_door_node.py (the THIRD door detector:
bottlenecks in the free space of that same /slam/map, no walls involved --
see freespace_doors.py's own module docstring, and stack_params.yaml's
door_source for how the three detectors exclude each other).

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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: The thread-pool variables a *_cpu_threads launch argument sets (the same four
#: as f1tenth_perception's detection.launch.py, yolo_cpu_threads).
CPU_THREAD_ENV_VARS = ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                       'OPENCV_FOR_THREADS_NUM')


def cpu_thread_env(threads):
    """Return the process env for a *_cpu_threads value: {} for 0 (library defaults)."""
    if threads <= 0:
        return {}
    return {var: str(threads) for var in CPU_THREAD_ENV_VARS}


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
        'costmap_boundary_extraction_rate_hz', default_value='10.0',
        description="Periodic publish rate for costmap_boundary_node -- "
                    "10 Hz since cpu-cleanup (was 20; 5 before the periodic-"
                    "publish pass): mpc_corr reads it at 10 Hz and every "
                    "downstream staleness gate is >= 0.5 s. This is "
                    "now the ONLY thing governing how often /costmap/"
                    "boundaries and /costmap/front_clearance are published "
                    "-- see that node's own module docstring's 'PERIODIC-"
                    "PUBLISH pass' paragraph for why upstream (/slam/map, "
                    "/ekf_global/odometry/filtered) recency no longer gates "
                    "the publish at all.")

    semantic_rate_la = DeclareLaunchArgument(
        'semantic_layer_rate_hz', default_value='5.0',
        description="Max rate semantic_layer_node rebuilds its wall layer "
                    "(/costmap/wall_cells, /costmap/wall_cell_ids) from the "
                    "NEWEST /perception/walls message -- cpu-cleanup, was "
                    "every message (~10 Hz). Neither output has a live "
                    "consumer (logger only / none). Detections are still "
                    "processed per batch: the track lifecycle counts batches.")

    # semantic_layer_node's thread env (R3, 2026-10-23): the yolo_cpu_threads
    # pattern of f1tenth_perception's detection.launch.py, default from
    # stack_params.yaml (semantic_layer_cpu_threads) rather than this file's
    # hardcoded-default convention, since the value is shared stack tuning.
    semantic_cpu_threads_default, semantic_cpu_threads_desc = get_default(
        'semantic_layer_cpu_threads')
    semantic_cpu_threads_la = DeclareLaunchArgument(
        'semantic_layer_cpu_threads', default_value=str(semantic_cpu_threads_default),
        description=semantic_cpu_threads_desc)

    # The door switches, by launch argument. Every OTHER semantic_doors_* /
    # freespace_doors_* value reaches its node straight from stack_params.yaml
    # through that node's own declare_parameter default, and is tuned by
    # editing that file (or with `ros2 param set`); these three are here
    # because they are what a run turns on and off, and because door_source
    # must reach lidar_front_wall_node, semantic_layer_node and
    # freespace_door_node with the SAME value -- one argument, three nodes.
    doors_enabled_default, doors_enabled_desc = get_default('semantic_doors_enabled')
    door_source_default, door_source_desc = get_default('door_source')
    freespace_enabled_default, freespace_enabled_desc = get_default(
        'freespace_doors_enabled')
    doors_enabled_la = DeclareLaunchArgument(
        'semantic_doors_enabled', default_value=str(doors_enabled_default),
        description=doors_enabled_desc)
    door_source_la = DeclareLaunchArgument(
        'door_source', default_value=str(door_source_default),
        description=door_source_desc)
    freespace_enabled_la = DeclareLaunchArgument(
        'freespace_doors_enabled', default_value=str(freespace_enabled_default),
        description=freespace_enabled_desc)
    freespace_rate_default, freespace_rate_desc = get_default('freespace_doors_rate_hz')
    freespace_window_default, freespace_window_desc = get_default('freespace_doors_window_m')
    freespace_rate_la = DeclareLaunchArgument(
        'freespace_doors_rate_hz', default_value=str(freespace_rate_default),
        description=freespace_rate_desc)
    freespace_window_la = DeclareLaunchArgument(
        'freespace_doors_window_m', default_value=str(freespace_window_default),
        description=freespace_window_desc)

    def _semantic_layer_node(context):
        # An OpaqueFunction only for the thread env: semantic_layer_cpu_threads
        # 0 (the default) must leave OMP/MKL/OPENBLAS/OPENCV unset, exactly the
        # env the node had, which a substitution cannot express. The door
        # parameters ride along as ordinary parameters -- they need no context.
        return [Node(
            package='f1tenth_costmap',
            executable='semantic_layer_node',
            name='semantic_layer_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_slam')),
            additional_env=cpu_thread_env(
                int(LaunchConfiguration('semantic_layer_cpu_threads').perform(context))),
            parameters=[{
                'wall_layer_rate_hz': ParameterValue(
                    LaunchConfiguration('semantic_layer_rate_hz'), value_type=float),
                'score_threshold': LaunchConfiguration('semantic_score_threshold'),
                'merge_distance_m': LaunchConfiguration('semantic_merge_distance_m'),
                'confirm_hit_count': LaunchConfiguration('semantic_confirm_hit_count'),
                'lost_miss_count': LaunchConfiguration('semantic_lost_miss_count'),
                'semantic_doors_enabled': ParameterValue(
                    LaunchConfiguration('semantic_doors_enabled'), value_type=bool),
                'door_source': ParameterValue(
                    LaunchConfiguration('door_source'), value_type=str),
            }],
        )]

    semantic_layer_node = OpaqueFunction(function=_semantic_layer_node)

    # The grid door detector. Gated on enable_slam like everything else in
    # this file and for the identical reason -- it reads /slam/map, so with
    # SLAM off it would be a permanently idle subscriber, not a meaningful
    # independent on/off state. It self-gates on door_source as well
    # (freespace_doors_enabled AND door_source == 'freespace'), so a bare
    # `ros2 run f1tenth_costmap freespace_door_node` with another source
    # selected is a no-op too, exactly the two-sided discipline the polytope
    # and component-filter flags below already follow. Rate and window are
    # launch arguments because they are the two knobs a run that cannot hold
    # 2 Hz on the Orin reaches for first; everything else comes from
    # stack_params.yaml.
    freespace_door_node = Node(
        package='f1tenth_costmap',
        executable='freespace_door_node',
        name='freespace_door_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam')),
        parameters=[{
            'freespace_doors_enabled': ParameterValue(
                LaunchConfiguration('freespace_doors_enabled'), value_type=bool),
            'door_source': ParameterValue(
                LaunchConfiguration('door_source'), value_type=str),
            'freespace_doors_rate_hz': ParameterValue(
                LaunchConfiguration('freespace_doors_rate_hz'), value_type=float),
            'freespace_doors_window_m': ParameterValue(
                LaunchConfiguration('freespace_doors_window_m'), value_type=float),
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

    # ---- convex safe corridor (safe_corridor.py) -- BUILT, NOT ENABLED.
    # Default 'false' here mirrors the node's own declare_parameter default;
    # both have to say false for the flag to actually be off under a bare
    # `ros2 run` as well as under this launch file. See the node's own
    # module docstring section "CONVEX SAFE CORRIDOR" for what turning it on
    # changes. Corrected 2026-09-09: this is the ONLY guard. MPC_corr.py's
    # use_hard_boundary_constraints is TRUE, so a published polytope WOULD
    # reach the solve -- and at boundary_max_sources 3 against up to 8 faces
    # it would be silently truncated. Raise that key to match
    # costmap_boundary_polytope_max_faces in the same change that sets this
    # to true.
    boundary_use_polytope_la = DeclareLaunchArgument(
        'costmap_boundary_use_convex_polytope', default_value='false',
        description="true: costmap_boundary_node publishes the faces of one "
                    "convex polytope inflated around the MPC's reference "
                    "centreline on /costmap/boundaries, instead of three "
                    "nearest-occupied-cell half-planes. Default false -- built "
                    "but not enabled. /costmap/front_clearance is unaffected "
                    "either way.")
    boundary_polytope_r_local_la = DeclareLaunchArgument(
        'costmap_boundary_polytope_r_local_m', default_value='3.0',
        description="Radius (m) of the local window the polytope is carved out "
                    "of, and the arc length of reference centreline used to "
                    "seed it. Only read when the polytope path is enabled.")
    boundary_polytope_max_faces_la = DeclareLaunchArgument(
        'costmap_boundary_polytope_max_faces', default_value='8',
        description="Max faces in the polytope. Each face is one QP row at "
                    "EVERY horizon stage, which is why there is a cap; hitting "
                    "it is reported on /costmap/safe_corridor_report, not "
                    "silently accepted.")

    # ---- COSTMAP-MODE CLEANUP -- BUILT, DEFAULT OFF, same discipline as the
    # polytope flag above. Both switches default false here AND in the node, so
    # the filter is off under a bare `ros2 run` too. See costmap_boundary.py's
    # own "COSTMAP-MODE CLEANUP" section for the failure these address: one
    # occupied cell -- a map artefact, a person, a single spurious return -- is
    # enough to produce a half-plane across the MPC's whole horizon, and the
    # extraction cannot tell any of those from a wall.
    #
    # NEITHER TOUCHES /costmap/front_clearance, whichever way they are set:
    # that scalar is recomputed from the raw grid on every tick because it
    # feeds a live mission-stopping behaviour-tree condition.
    boundary_component_filter_la = DeclareLaunchArgument(
        'costmap_boundary_component_filter', default_value='false',
        description="true: costmap_boundary_node ignores occupied connected "
                    "components shorter than "
                    "costmap_boundary_min_component_length_m, or seen in fewer "
                    "than costmap_boundary_min_component_scans consecutive "
                    "extraction ticks, when it builds /costmap/boundaries. "
                    "Default false -- current behaviour.")
    boundary_min_component_length_la = DeclareLaunchArgument(
        'costmap_boundary_min_component_length_m', default_value='0.5',
        description="Shortest occupied connected component (m, bounding-box "
                    "diagonal) that may still produce a boundary row. Only "
                    "read with costmap_boundary_component_filter true.")
    boundary_min_component_scans_la = DeclareLaunchArgument(
        'costmap_boundary_min_component_scans', default_value='2',
        description="Consecutive EXTRACTION TICKS a component must be seen in "
                    "before it may produce a boundary row. Ticks, not scans: "
                    "this node reads the latched /slam/map on its own timer "
                    "and never sees a scan. 1 disables the test.")
    boundary_exclude_object_cells_la = DeclareLaunchArgument(
        'costmap_boundary_exclude_tracked_object_cells', default_value='false',
        description="true: occupied cells inside a tracked object's footprint "
                    "plus costmap_boundary_object_footprint_margin_m "
                    "(/perception/obstacles_2d) never produce a boundary row. "
                    "They are already handled by the MPC's soft w_obs "
                    "deflection cost, and the hard-ish copy is the one that "
                    "can stop the car dead. Default false.")
    boundary_object_margin_la = DeclareLaunchArgument(
        'costmap_boundary_object_footprint_margin_m', default_value='0.12',
        description="Margin (m) grown around each tracked object's own radius "
                    "for the exclusion above -- the stack's shared "
                    "obstacle_safety_margin_m, not a new number.")

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
            'use_convex_polytope': LaunchConfiguration(
                'costmap_boundary_use_convex_polytope'),
            'polytope_r_local_m': LaunchConfiguration(
                'costmap_boundary_polytope_r_local_m'),
            'polytope_max_faces': LaunchConfiguration(
                'costmap_boundary_polytope_max_faces'),
            'component_filter': LaunchConfiguration(
                'costmap_boundary_component_filter'),
            'min_component_length_m': LaunchConfiguration(
                'costmap_boundary_min_component_length_m'),
            'min_component_scans': LaunchConfiguration(
                'costmap_boundary_min_component_scans'),
            'exclude_tracked_object_cells': LaunchConfiguration(
                'costmap_boundary_exclude_tracked_object_cells'),
            'object_footprint_margin_m': LaunchConfiguration(
                'costmap_boundary_object_footprint_margin_m'),
        }],
    )

    return LaunchDescription([
        enable_la, score_threshold_la, merge_distance_la,
        confirm_hit_count_la, lost_miss_count_la, render_rate_la,
        boundary_front_max_la, boundary_side_min_la, boundary_side_max_la,
        boundary_occupied_threshold_la, boundary_max_range_la, boundary_rate_la,
        boundary_use_polytope_la, boundary_polytope_r_local_la,
        boundary_polytope_max_faces_la, semantic_rate_la,
        boundary_component_filter_la, boundary_min_component_length_la,
        boundary_min_component_scans_la, boundary_exclude_object_cells_la,
        boundary_object_margin_la, semantic_cpu_threads_la,
        doors_enabled_la, door_source_la, freespace_enabled_la,
        freespace_rate_la, freespace_window_la,
        semantic_layer_node, costmap_renderer_node, costmap_boundary_node,
        freespace_door_node,
    ])
