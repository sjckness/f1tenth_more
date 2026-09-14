"""YOLO 2D detector + 2D-to-3D detection fusion, plus front_clearance_node.

Source-agnostic: subscribes to the canonical /camera/image_raw (published by
whichever camera stack_bringup brought up) and publishes vision_msgs
Detection2DArray/Detection3DArray + a MarkerArray for Foxglove/RViz.

detection_3d_node/obstacle_projector_node/front_clearance_node all need
ZED depth or the ZED point cloud, which only exist in ZED mode, so they're
only built at all when camera_source == 'zed' -- camera_source is one of
the 6 stack-wide branching args (see f1tenth_params/config/stack_params.yaml),
a plain Python value here, not a DeclareLaunchArgument/LaunchConfiguration.

front_clearance_node REPLACES front_depth_monitor_node, which used to be
launched here and is no longer launched at all. Both publish /perception/
front_distance, so running the two together would put two publishers on one
topic -- the replacement is why that block is gone, not an oversight. The
node module and its console_scripts entry point are both still present and
still work if launched by hand; only this file stopped starting it.

What changes for the existing /perception/front_distance consumer
(MPC_corr.py, which subscribes it into self.front_distance): the value is now
EMA-smoothed rather than a raw per-frame percentile, and objects detected by
YOLO are excluded from it rather than included. Its semantics are narrowed to
"distance to the background/wall". MPC_corr's use of it is telemetry only --
d_front reaches the corridor debug dict as "dFront" and one log line, and the
corridor length L is derived from the goal distance clipped to corr_L_base --
so this is a change in what that telemetry MEANS, not in what the car does.
Several docstrings around the stack still describe MPC_corr as having
"front_distance-based corridor-length logic"; that coupling is not in the
code.

front_clearance_node is only PARTLY independent of the YOLO pipeline above
it, which is the one real difference from the node it replaces: it reads raw
ZED depth directly (so it keeps publishing whether or not detections flow),
but it also consumes /camera/detections, /camera/detection_masks and
/perception/obstacles_2d to exclude objects from the background estimate and
to compute front_clearance. Every one of those is optional and degrades
rather than blocks -- publishing is driven by the depth frame alone, never by
a synchronizer. It lives in this file rather than a separate one because the
is_zed gating condition is identical and this package doesn't otherwise split
one launch file per node.

wall_detector_node (RANSAC plane segmentation for wall/corner detection, ZED
point-cloud-based) previously lived in this file, independent of the YOLO/
detection_3d/obstacle_projector chain above -- RETIRED by the dual-EKF +
costmap-derived-MPC-boundaries pass in favor of f1tenth_costmap's costmap_
boundary_node.py (nearest-occupied-cell extraction from slam_toolbox's own
/slam/map, see that node's own module docstring). Deleted entirely here
(node file, its ~24 wall_* stack_params.yaml keys, this file's own wall_*
launch-argument declarations/cpu_affinity block/Node() registration, and its
own test file) -- confirmed via a codebase-wide grep before deleting that
nothing else held a functional dependency on it (only historical/precedent
comment mentions remain elsewhere, left as accurate documentation of where
those design patterns came from). Lost as a result: the Foxglove /perception/
wall_markers visualization (individual wall-surface MarkerArray) has no
replacement in this pass -- costmap_boundary_node.py publishes hard boundary
constraints and a scalar front_clearance, not a per-wall visualization; the
existing /costmap/visualization PNG render (costmap_renderer_node.py) shows
the raw occupancy grid, which does NOT yet distinguish individual wall
surfaces as distinctly as the retired per-wall Foxglove markers did -- flagged
explicitly, not fixed this pass (out of scope; see this pass's own final
report).

cpu_affinity/nice args (perception-optimization pass, following the latency
audit that root-caused /camera/detections' ~278ms lag behind /camera/image_raw
to CPU/executor contention, not transport/QoS): one pair per node. Not
sourced from stack_params.yaml -- like mpc_corr.launch.py's own cpu_affinity,
the right core ids are machine-specific, not a stack-wide default. Defaults
below reserve yolo_detector_node (clearest contention signature in the audit)
its own pair, and detection_3d_node/obstacle_projector_node (less severe:
75-82% and ~56-59% CPU with no pinning) a shared "perception" pair -- both
away from mpc_corr's own 10,11.

CPU AFFINITY NOW A taskset -c LAUNCH PREFIX PER NODE, NOT self-pinning
(thread-pinning-leak fix, Step 6 reintroduction investigation): each
*_cpu_affinity arg used to be read by the node itself (f1tenth_perception/
cpu_affinity.py's apply_cpu_affinity_and_priority(), calling
os.sched_setaffinity(0, cores) once, in-process, from __init__) --
confirmed live, under Stage 4 load, that this only ever restricted the ONE
thread executing that call: yolo_detector_node had 32 of 36 threads fully
unpinned (0-11), detection_3d_node 32 of 33, obstacle_projector_node 21 of
22 -- with threads from multiple nodes actually caught executing on cores
0-4 (the EKF pair, slam_toolbox, and behavior_executor_node's own reserved
cores) at the moment of checking, not just theoretically able to. Each
`prefix=['taskset -c ', ...]` below sets the affinity mask before that
node's first instruction runs, so every thread it or any library (CUDA/
TensorRT inference threads very much included) ever spawns inherits it,
with no in-process code needed at all -- the same mechanism ekf.launch.py/
foxglove_bridge.launch.py/slam.launch.py already use, and the only pinned
nodes in this stack that stayed 100% on their assigned cores through
Step 6's full reintroduction sequence, including Stage 4's saturated load.
cpu_affinity.py's shared helper is now nice-only (declare_nice_param/
apply_nice) -- see that module's own docstring.
"""

import os

from f1tenth_params.param_defaults import get_default, get_value

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    yolo_model_task_default, yolo_model_task_desc = get_default('yolo_model_task')
    yolo_model_task_la = DeclareLaunchArgument(
        'yolo_model_task', default_value=str(yolo_model_task_default),
        description=yolo_model_task_desc)
    # Live-deployable yolo-seg pass: default_value is a PythonExpression, not
    # str(use_mask_depth_default) like every other arg here -- deliberately,
    # so selecting a seg-family yolo_model (e.g. yolo26s-seg.pt) is a SINGLE
    # switch, not two independent ones to remember (mask-based fusion is the
    # whole point of a seg model; leaving it off by default against one would
    # silently waste the mask decode and fall back to box-region sampling).
    # Evaluated against whatever yolo_model actually resolves to at launch
    # time (its own override included, not just its stack_params.yaml
    # default) -- 'yolo_model_la' is declared just above so this reference is
    # always valid. stack_params.yaml's own use_mask_depth default (false) is
    # unreachable via this expression today (no shipped filename contains
    # '-seg' except the seg models themselves) but is exactly what a
    # non-seg yolo_model (the deployed default, yolo26s.engine) still
    # produces -- '-seg' not in 'yolo26s.engine' is False, same false
    # default as before this pass, zero behavior change for the default
    # launch. Explicit `use_mask_depth:=false` on the command line still
    # overrides this (or `:=true`), same as any other launch arg -- this
    # only changes what happens when it's left unset.
    use_mask_depth_default, use_mask_depth_desc = get_default('use_mask_depth')
    use_mask_depth_la = DeclareLaunchArgument(
        'use_mask_depth',
        default_value=PythonExpression(
            ["'-seg' in '", LaunchConfiguration('yolo_model'), "'"]),
        description=(
            use_mask_depth_desc +
            " Default auto-derived from yolo_model: true whenever its "
            "filename contains '-seg' (e.g. yolo26s-seg.pt), false "
            "otherwise (matches stack_params.yaml's own false default for "
            "every non-seg yolo_model, including the deployed default) -- "
            "override explicitly to decouple mask-based fusion from model "
            "selection if ever needed."))
    # Car's-own-LiDAR exclusion (self-occlusion filter) -- see yolo_detector_
    # node.py's own module docstring and stack_params.yaml's lidar_exclusion_
    # x_min comment for the full rationale/trade-off/calibration source. All
    # 5 forwarded straight through to yolo_detector_node below; screenshot-
    # calibrated with margin, not yet measured against a real-resolution
    # live frame.
    lidar_exclusion_x_min_default, lidar_exclusion_x_min_desc = get_default(
        'lidar_exclusion_x_min')
    lidar_exclusion_x_min_la = DeclareLaunchArgument(
        'lidar_exclusion_x_min', default_value=str(lidar_exclusion_x_min_default),
        description=lidar_exclusion_x_min_desc)
    lidar_exclusion_x_max_default, lidar_exclusion_x_max_desc = get_default(
        'lidar_exclusion_x_max')
    lidar_exclusion_x_max_la = DeclareLaunchArgument(
        'lidar_exclusion_x_max', default_value=str(lidar_exclusion_x_max_default),
        description=lidar_exclusion_x_max_desc)
    lidar_exclusion_y_min_default, lidar_exclusion_y_min_desc = get_default(
        'lidar_exclusion_y_min')
    lidar_exclusion_y_min_la = DeclareLaunchArgument(
        'lidar_exclusion_y_min', default_value=str(lidar_exclusion_y_min_default),
        description=lidar_exclusion_y_min_desc)
    lidar_exclusion_y_max_default, lidar_exclusion_y_max_desc = get_default(
        'lidar_exclusion_y_max')
    lidar_exclusion_y_max_la = DeclareLaunchArgument(
        'lidar_exclusion_y_max', default_value=str(lidar_exclusion_y_max_default),
        description=lidar_exclusion_y_max_desc)
    lidar_exclusion_overlap_threshold_default, lidar_exclusion_overlap_threshold_desc = (
        get_default('lidar_exclusion_overlap_threshold'))
    lidar_exclusion_overlap_threshold_la = DeclareLaunchArgument(
        'lidar_exclusion_overlap_threshold',
        default_value=str(lidar_exclusion_overlap_threshold_default),
        description=lidar_exclusion_overlap_threshold_desc)

    # front_clearance_node's own tuning params. Declared with an explicit
    # value_type per entry rather than relying on launch_ros's string
    # type-inference -- the same reasoning behavior_bringup.launch.py's own
    # _avoidance_args block spells out, and publish_debug_raw is exactly the
    # case it warns about: a LaunchConfiguration evaluates to the STRING
    # 'False', which is non-empty and therefore true, so inference would
    # silently turn the debug topics ON at their default. The integer counts
    # matter for the same reason in the other direction -- a float pixel
    # threshold or dwell count would be a type error at the node, not a
    # silent wrong value, but spelling every type out is cheaper than
    # remembering which ones are safe.
    #
    # Names carry the front_clearance_ prefix in stack_params.yaml (and
    # therefore as launch args) but are passed to the node under their bare
    # names -- several of them (distance_ema_alpha, corridor_half_width_m)
    # are far too generic to sit unprefixed in a workspace-wide flat
    # namespace shared with every other package's args.
    _front_clearance_args = [
        ('roi_half_width_px', int),
        ('roi_half_height_px', int),
        ('min_bg_pixels_for_reading', int),
        ('too_close_clearance_m', float),
        ('too_close_min_pixel_fraction', float),
        ('wall_enter_px', int),
        ('wall_exit_px', int),
        ('wall_min_dwell_frames', int),
        ('distance_ema_alpha', float),
        ('mask_time_constant', float),
        ('background_weight_threshold', float),
        ('background_percentile_low', float),
        ('background_percentile_high', float),
        ('corridor_half_width_m', float),
        ('obstacle_max_age_s', float),
        ('clearance_enter_m', float),
        ('clearance_exit_m', float),
        ('clearance_min_dwell_frames', int),
        ('publish_debug_raw', bool),
    ]
    front_clearance_las = []
    for _name, _type in _front_clearance_args:
        _default, _desc = get_default('front_clearance_' + _name)
        front_clearance_las.append(DeclareLaunchArgument(
            'front_clearance_' + _name, default_value=str(_default),
            description=_desc))
    front_clearance_params = {
        _name: ParameterValue(
            LaunchConfiguration('front_clearance_' + _name), value_type=_type)
        for _name, _type in _front_clearance_args
    }

    obstacle_z_min_default, obstacle_z_min_desc = get_default('obstacle_z_min')
    obstacle_z_min_la = DeclareLaunchArgument(
        'obstacle_z_min', default_value=str(obstacle_z_min_default),
        description=obstacle_z_min_desc)
    obstacle_z_max_default, obstacle_z_max_desc = get_default('obstacle_z_max')
    obstacle_z_max_la = DeclareLaunchArgument(
        'obstacle_z_max', default_value=str(obstacle_z_max_default),
        description=obstacle_z_max_desc)

    # cpu_affinity/nice, one pair per node -- see module docstring. cpu_affinity
    # now feeds a 'taskset -c' launch prefix (not a self-pin ROS param) -- an
    # EMPTY value is NOT a graceful no-op here (unlike nice): 'taskset -c' with
    # no core list is a shell-level error. Must stay a valid, non-empty core
    # list; to fully disable pinning, remove that Node's prefix= argument
    # instead of emptying this value (same caveat as ekf.launch.py/
    # foxglove_bridge.launch.py/slam.launch.py's own matching arguments).
    yolo_cpu_affinity_la = DeclareLaunchArgument(
        'yolo_cpu_affinity', default_value='8,9',
        description="Comma-separated core ids to pin yolo_detector_node to "
                    "via a 'taskset -c' launch prefix.")
    yolo_nice_la = DeclareLaunchArgument(
        'yolo_nice', default_value='0',
        description="Process niceness for yolo_detector_node. 0: no-op.")
    detection_3d_cpu_affinity_la = DeclareLaunchArgument(
        'detection_3d_cpu_affinity', default_value='6,7',
        description="Comma-separated core ids to pin detection_3d_node to "
                    "via a 'taskset -c' launch prefix.")
    detection_3d_nice_la = DeclareLaunchArgument(
        'detection_3d_nice', default_value='0',
        description="Process niceness for detection_3d_node. 0: no-op.")
    obstacle_projector_cpu_affinity_la = DeclareLaunchArgument(
        'obstacle_projector_cpu_affinity', default_value='6,7',
        description="Comma-separated core ids to pin obstacle_projector_node "
                    "to via a 'taskset -c' launch prefix.")
    obstacle_projector_nice_la = DeclareLaunchArgument(
        'obstacle_projector_nice', default_value='0',
        description="Process niceness for obstacle_projector_node. 0: no-op.")
    # No taskset prefix for front_clearance_node, unlike the three nodes
    # above: it inherits the (unpinned) default the node it replaces,
    # front_depth_monitor_node, also ran with. Its per-frame work is a
    # percentile over a small ROI plus one whole-image EMA -- nothing like
    # the CUDA/TensorRT thread population the pinning exists to contain --
    # so assigning it cores would be a guess at a budget nobody has
    # measured. Measure it under Stage 4 load first.
    front_clearance_nice_la = DeclareLaunchArgument(
        'front_clearance_nice', default_value='0',
        description="Process niceness for front_clearance_node. 0: no-op.")

    is_zed = get_value('camera_source') == 'zed'

    yolo_detector_node = Node(
        package='f1tenth_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        prefix=['taskset -c ', LaunchConfiguration('yolo_cpu_affinity')],
        parameters=[{
            'image_topic': '/camera/image_raw',
            'detections_topic': '/camera/detections',
            'annotated_topic': '/camera/image_annotated',
            'model_path': PathJoinSubstitution([
                src_dir, 'f1tenth_perception', 'models',
                LaunchConfiguration('yolo_model')]),
            'device': LaunchConfiguration('yolo_device'),
            # Only consulted for a *.engine yolo_model -- see stack_params.yaml's
            # own yolo_model_task entry and yolo_detector_node.py's "Task
            # resolution" docstring paragraph. Ignored for a *.pt yolo_model
            # (e.g. yolo26s-seg.pt), which carries its own task in the checkpoint.
            'model_task': LaunchConfiguration('yolo_model_task'),
            # Only ever published when the loaded model resolves to task
            # 'segment' -- see yolo_detector_node.py's own docstring.
            'masks_topic': '/camera/detection_masks',
            # Previously only reached detection_3d_node below -- see
            # yolo_detector_node.py's own docstring for why that left this
            # node's own output unfiltered regardless of this value.
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            # Car's-own-LiDAR exclusion -- see the declare block above.
            'lidar_exclusion_x_min': LaunchConfiguration('lidar_exclusion_x_min'),
            'lidar_exclusion_x_max': LaunchConfiguration('lidar_exclusion_x_max'),
            'lidar_exclusion_y_min': LaunchConfiguration('lidar_exclusion_y_min'),
            'lidar_exclusion_y_max': LaunchConfiguration('lidar_exclusion_y_max'),
            'lidar_exclusion_overlap_threshold': LaunchConfiguration(
                'lidar_exclusion_overlap_threshold'),
            'nice': LaunchConfiguration('yolo_nice'),
        }],
    )

    actions = [
        confidence_threshold_la, yolo_device_la, yolo_model_la,
        yolo_model_task_la, use_mask_depth_la,
        lidar_exclusion_x_min_la, lidar_exclusion_x_max_la,
        lidar_exclusion_y_min_la, lidar_exclusion_y_max_la,
        lidar_exclusion_overlap_threshold_la,
        obstacle_z_min_la, obstacle_z_max_la,
        yolo_cpu_affinity_la, yolo_nice_la,
        detection_3d_cpu_affinity_la, detection_3d_nice_la,
        obstacle_projector_cpu_affinity_la, obstacle_projector_nice_la,
        front_clearance_nice_la,
        *front_clearance_las,
        yolo_detector_node,
    ]

    if is_zed:
        actions.append(Node(
            package='f1tenth_perception',
            executable='detection_3d_node',
            name='detection_3d_node',
            output='screen',
            prefix=['taskset -c ', LaunchConfiguration('detection_3d_cpu_affinity')],
            parameters=[{
                'detections_topic': '/camera/detections',
                'depth_topic': '/zed2/zed_node/depth/depth_registered',
                'depth_info_topic': '/zed2/zed_node/depth/camera_info',
                'detections_3d_topic': '/camera/detections_3d',
                'markers_topic': '/camera/detection_markers',
                # Matches yolo_detector_node's own masks_topic above -- only
                # actually subscribed when use_mask_depth is true (see
                # detection_3d_node.py's own docstring); harmless to always
                # pass the topic name either way.
                'masks_topic': '/camera/detection_masks',
                'use_mask_depth': LaunchConfiguration('use_mask_depth'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
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
            prefix=['taskset -c ', LaunchConfiguration('obstacle_projector_cpu_affinity')],
            parameters=[{
                'detections_3d_topic': '/camera/detections_3d',
                'obstacles_topic': '/perception/obstacles_2d',
                'output_frame': 'base_link',
                'obstacle_z_min': LaunchConfiguration('obstacle_z_min'),
                'obstacle_z_max': LaunchConfiguration('obstacle_z_max'),
                'nice': LaunchConfiguration('obstacle_projector_nice'),
            }],
        ))

        # Jitter-hardened front wall / front clearance. REPLACES
        # front_depth_monitor_node, which this file no longer launches -- both
        # publish /perception/front_distance and two publishers on one topic
        # is not a thing to leave running. See the module docstring for what
        # that swap changes for MPC_corr.py, the existing subscriber.
        #
        # depth_topic is passed EXPLICITLY as the ZED topic here, exactly as
        # detection_3d_node above does, even though the node's own default is
        # the canonical /camera/depth/image_raw. That canonical name does not
        # exist in this stack: camera.launch.py's GroupAction remaps the ZED
        # wrapper's RGB and camera_info onto /camera/* but NOT its depth, and
        # adding a depth SetRemap there would rename the wrapper's published
        # topic out from under detection_3d_node, which subscribes to
        # /zed2/zed_node/depth/depth_registered by name. Making the canonical
        # depth topic real is a camera.launch.py change with its own blast
        # radius, not a side effect of adding this node.
        actions.append(Node(
            package='f1tenth_perception',
            executable='front_clearance_node',
            name='front_clearance_node',
            output='screen',
            parameters=[{
                'depth_topic': '/zed2/zed_node/depth/depth_registered',
                'detections_topic': '/camera/detections',
                # Only ever published by a segment-task yolo_model; the node
                # detects that per frame by header stamp and falls back to
                # bounding boxes when no mask matches, so passing the name
                # unconditionally is correct for both model types (same
                # reasoning as detection_3d_node's masks_topic above).
                'masks_topic': '/camera/detection_masks',
                'obstacles_topic': '/perception/obstacles_2d',
                # Same four fractional edges yolo_detector_node gets -- one
                # calibration of the same physical housing, two consumers.
                # The overlap threshold is NOT passed: it is a per-detection
                # suppression rule with no meaning for a depth ROI cut.
                'lidar_exclusion_x_min': LaunchConfiguration('lidar_exclusion_x_min'),
                'lidar_exclusion_x_max': LaunchConfiguration('lidar_exclusion_x_max'),
                'lidar_exclusion_y_min': LaunchConfiguration('lidar_exclusion_y_min'),
                'lidar_exclusion_y_max': LaunchConfiguration('lidar_exclusion_y_max'),
                'nice': LaunchConfiguration('front_clearance_nice'),
                **front_clearance_params,
            }],
        ))

    return LaunchDescription(actions)
