"""YOLO 2D detector + 2D-to-3D detection fusion, plus front_depth_monitor_node.

Source-agnostic: subscribes to the canonical /camera/image_raw (published by
whichever camera stack_bringup brought up) and publishes vision_msgs
Detection2DArray/Detection3DArray + a MarkerArray for Foxglove/RViz.

detection_3d_node/obstacle_projector_node/front_depth_monitor_node all need
ZED depth or the ZED point cloud, which only exist in ZED mode, so they're
only built at all when camera_source == 'zed' -- camera_source is one of
the 6 stack-wide branching args (see f1tenth_params/config/stack_params.yaml),
a plain Python value here, not a DeclareLaunchArgument/LaunchConfiguration.

front_depth_monitor_node is deliberately unrelated to the YOLO/detection
pipeline above it in this file -- it reads raw ZED depth directly, with no
dependency on yolo_detector_node/detection_3d_node's output, by design (see
its own docstring: it's the front half of f1tenth_behavior's IsProximityTooClose
last-resort emergency-stop condition, which must keep working even if YOLO is
degraded). It lives in this file anyway rather than a separate one because the
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
            # Previously only reached detection_3d_node below -- see
            # yolo_detector_node.py's own docstring for why that left this
            # node's own output unfiltered regardless of this value.
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'nice': LaunchConfiguration('yolo_nice'),
        }],
    )

    actions = [
        confidence_threshold_la, yolo_device_la, yolo_model_la,
        obstacle_z_min_la, obstacle_z_max_la,
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
            prefix=['taskset -c ', LaunchConfiguration('detection_3d_cpu_affinity')],
            parameters=[{
                'detections_topic': '/camera/detections',
                'depth_topic': '/zed2/zed_node/depth/depth_registered',
                'depth_info_topic': '/zed2/zed_node/depth/camera_info',
                'detections_3d_topic': '/camera/detections_3d',
                'markers_topic': '/camera/detection_markers',
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

    return LaunchDescription(actions)
