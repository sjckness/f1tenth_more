"""First SLAM implementation for this stack: slam_toolbox (lidar-primary,
async/mapping mode) -- chosen over RTAB-Map after two feasibility NO-GOs on
this Orin box (CPU-only RTAB-Map's realistic RGBD+lidar footprint had no
plausible core placement without degrading an already-tuned node, see that
investigation's own reports; slam_toolbox is lidar-only, structurally much
lighter -- no RGBD processing, no cross-modal graph optimization).

Toggleable via enable_slam (default false, f1tenth_params/config/
stack_params.yaml) -- a normal per-file IfCondition toggle, NOT one of the 5
stack-wide branching args, same pattern as enable_foxglove (see that key's
own comment). component_supervisor_node.py always auto-starts the 'slam'
component itself (same as dev_tools/enable_foxglove) -- this flag alone
decides whether the Node below actually launches.

Consumes /scan directly (now front-facing post lidar-remount -- see
f1tenth_description/launch/description.launch.py's own module docstring for
that remount's details; slam_toolbox itself has no notion of which physical
direction the lidar faces, it just fits scan-to-scan geometry, so the remount
doesn't otherwise change anything here).

Pose-output only in this pass -- NOT wired into localization_source/EKF/
mpc_corr, a separate, deliberate future decision:
  - transform_publish_period is set to 0.0 in slam_toolbox_params.yaml (see
    that file's own header comment) -- slam_toolbox will NEVER publish
    map->odom (or anything on /tf at all). This is the actual mechanism
    enforcing the scope boundary, not just a documentation promise: map->odom
    is ALREADY owned by the EKF or raw_odom mirror, per localization_source
    (f1tenth_localization/launch/localization.launch.py) -- letting
    slam_toolbox ALSO claim that same TF edge would be a direct, silent
    authority collision.
  - /map and /pose (slam_toolbox's own default topic names) are remapped to
    /slam/map and /slam/pose specifically because f1tenth_navigation's own
    map_server (enable_nav2:=true path, or the enable_nav2:=false map_only.
    launch.py path -- see navigation.launch.py) ALSO publishes bare /map,
    from a pre-built static map file. Running enable_slam and enable_nav2
    both true at once would otherwise mean two unrelated, disagreeing
    sources of truth silently fighting over the exact same topic name -- a
    real, not hypothetical, collision this remap avoids entirely regardless
    of which combination of toggles is active. /pose doesn't collide with
    anything else in this stack today, remapped anyway for the same
    /slam/... namespacing consistency (matches this codebase's own
    /perception/..., /mpc/... per-subsystem topic prefix convention).
  - Both native topics (/slam/map as nav_msgs/OccupancyGrid, /slam/pose as
    geometry_msgs/PoseWithCovarianceStamped) stay available for later use
    (e.g. the two-layer costmap's own occupancy layer, see costmap_
    renderer_node.py) -- this file does not do anything with them itself.

cpu_affinity: async_slam_toolbox_node is a VENDORED binary (ros-humble-slam-
toolbox, not this workspace's own source) -- pinned via a 'taskset -c' launch
prefix, the SAME mechanism ekf_node/foxglove_bridge (and, after its own
per-thread-affinity bug fix, wall_detector_node) use for exactly this reason.
Confirmed live (idle, no /scan data flowing -- see this pass's own
feasibility report for why full live verification against real scan-matching
load is deferred to once the physical lidar remount + reconnection is done)
that EVERY thread this binary spawns (13, idle) correctly inherits the outer
taskset's restriction -- unlike wall_detector_node's own Open3D/libgomp bug,
no additional per-library env-var workaround was found necessary here. '2'
(a single core) reflects this pass's own feasibility snapshot: core 2 is the
one genuinely free core stack-wide under real full-sensor load (the shared
0,1/6,7/8,9/10,11 pairs all already carry a tuned consumer with real but
modest slack, not free capacity a new heavy process should be layered onto)
-- slam_toolbox's OWN real cost under active scan-matching is not yet
live-measured (both the ZED and the lidar itself were down for what's
plausibly the concurrent physical remount at the time of this pass), so this
single-core budget is a REASONED STARTING POINT pending that live
verification, not a validated one -- flagged deliberately, same discipline
wall_detector_node's own node-local gate parameters already established.
"""

from ament_index_python.packages import get_package_share_directory

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('f1tenth_navigation')
    params_file = f'{slam_share}/config/slam_toolbox_params.yaml'

    enable_default, enable_desc = get_default('enable_slam')
    enable_la = DeclareLaunchArgument(
        'enable_slam', default_value=str(enable_default), description=enable_desc)

    # cpu_affinity -- see module docstring's own "cpu_affinity" section for
    # the reasoning behind '2' and why this is a launch prefix (vendored
    # binary), not the self-pin ROS-param mechanism this workspace's own
    # source nodes use. Same non-empty-value caveat as ekf_cpu_affinity/
    # foxglove_cpu_affinity/wall_detector_cpu_affinity: 'taskset -c' with an
    # empty value is a shell-level error, not a graceful no-op -- remove the
    # prefix= argument entirely to disable pinning, don't empty this value.
    slam_cpu_affinity_la = DeclareLaunchArgument(
        'slam_cpu_affinity', default_value='2',
        description="Comma-separated core ids to pin async_slam_toolbox_node "
                    "to via a 'taskset -c' launch prefix (vendored binary, "
                    "can't self-pin). Must stay a valid, non-empty core "
                    "list -- see this file's own module docstring.")

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam')),
        prefix=['taskset -c ', LaunchConfiguration('slam_cpu_affinity')],
        parameters=[params_file],
        remappings=[
            ('/map', '/slam/map'),
            ('/map_metadata', '/slam/map_metadata'),
            ('/pose', '/slam/pose'),
        ],
    )

    return LaunchDescription([
        enable_la,
        slam_cpu_affinity_la,
        slam_toolbox_node,
    ])
