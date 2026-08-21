"""robot_localization EKF -- GLOBAL instance (dual-EKF pass): fuses the LOCAL
EKF's own /odometry/filtered (f1tenth_localization/launch/ekf.launch.py, now
world_frame: odom) with slam_toolbox's /slam/pose (via slam_pose_relay_node's
covariance-stamped /slam/pose_calibrated -- see that node's own docstring for
why the raw topic isn't fused directly), publishing map -> odom. See
f1tenth_bringup/config/ekf_global.yaml's own docstring for the full two-layer
design.

Only included when localization_source == 'ekf' (see localization.launch.py)
-- raw_odom mode has no /odometry/filtered topic to feed this filter's own
odom0 at all (raw_odom_map_tf_node only ever mirrors /odom directly into
map -> odom itself, no local EKF involved in that mode), so a global EKF
instance would have nothing meaningful to fuse there.

cpu_affinity: ekf_global_filter_node is the SAME vendored robot_localization
binary as the local EKF (f1tenth_localization/launch/ekf.launch.py) -- same
'taskset -c' launch-prefix mechanism, same reasoning (can't self-pin). Shares
the local EKF's own pair (0,1) -- NOW live-measured (core-remap pass,
following a live CPU-contention investigation -- see that pass's own
report): 37.1% CPU at rest, close to the local EKF's own 38.0%, not the
"plausibly lighter still" guess this comment previously made. That guess
also undersold the pair as a whole: cores 0,1 previously ALSO carried
foxglove_bridge (52.2%) and behavior_executor_node (24.7% idle / ~45%
documented under real BT load) at the same time -- ~152% combined demand
on a 200% budget, confirmed saturated live (cpu0/cpu1 both 99%+ busy).
Both of those have since been moved to their own cores (see foxglove_
bridge.launch.py/behavior_bringup.launch.py's own matching comments) --
this pair is now reserved for just the two EKF instances. Kept as a
SHARED pair rather than split one-core-each deliberately: the two
instances run in lockstep (both 50Hz, dual-EKF pass), so letting the OS
scheduler balance them across both cores tolerates either one's load
spiking without a hard per-core ceiling. See ekf.launch.py's own matching
comment for the full measurement and scripts/check_ekf_update_rate.py for
the live re-verification this still needs under real motion (this
correction pass had no live hardware).

slam_pose_relay_node is this workspace's own source (f1tenth_localization) --
no taskset prefix needed (self-pins via the standard cpu_affinity/nice ROS
params if a future pass ever finds it needs pinning; left at the default
no-op for now -- a plain covariance-stamping relay is not expected to be a
real CPU consumer, unlike this workspace's actually-measured heavy nodes).

REAL BUG FOUND WHILE BUILDING costmap_boundary_node (Part 4 of this pass,
not assumed/pre-empted, caught by re-reading this file before wiring a new
consumer to "the global EKF's output topic"): robot_localization's ekf_node
always publishes its fused nav_msgs/Odometry on the RELATIVE topic
'odometry/filtered' (confirmed against this workspace's own ekf.yaml/
ekf_global.yaml comments, which already document '/odometry/filtered' as
the LOCAL EKF's real, unremapped, unnamespaced output topic) -- there is no
YAML parameter that changes this, only launch-level remapping/namespacing.
Neither this node's own Node() action NOR the local EKF's (f1tenth_
localization/launch/ekf.launch.py) sets a namespace, so left alone, BOTH
instances would resolve to the exact same global topic, '/odometry/filtered'
-- not merely a naming collision but a real self-loop: this filter's own
odom0 (below) already reads '/odometry/filtered' (meaning the LOCAL EKF's
output, correctly), so once this filter ALSO started publishing there
itself, its own future ticks would start fusing its own recursive output
back in as odom0. The local EKF (ekf.launch.py) is left completely alone: it
is the one instance meant to own the plain '/odometry/filtered' name (every
other consumer in this stack, and this filter's own odom0, already expect it
there).

SECOND REAL BUG FOUND, LIVE, ONE PASS LATER (confirmed via `ros2 node info
/ekf_global_filter_node` against the actually-running stack: its own
Subscribers list showed '/ekf_global/odometry/filtered', not
'/odometry/filtered', with odom0 therefore silently starved since launch):
the first bug's own original fix -- `remappings=[('odometry/filtered',
'/ekf_global/odometry/filtered')]` -- was itself broken. A launch remapping
matches purely by RESOLVED topic-name STRING, blind to which call site asked
for it and blind to relative-vs-absolute origin. This node has no namespace,
so the remap's relative 'from' side ('odometry/filtered') resolves to
exactly '/odometry/filtered' -- the SAME string odom0 (below) independently
requests, already absolute, straight from ekf_global.yaml. The remap can't
tell those two requests apart: it silently caught odom0's subscription too,
rerouting it onto this node's own (then-unpublished) output -- a genuine
self-loop, just on the input side instead of the output side the first fix
addressed. Giving odom0 an absolute path (which it already had) does NOT
protect it either -- confirmed live, that was already the case when this
broke; remapping does not distinguish absolute origin from a relatively-
resolved match reaching the same resolved string.

Fixed structurally this time, not just patched around this one instance:
`namespace='ekf_global'` below, no `remappings=` at all. A namespace only
ever rewrites RELATIVE names -- it cannot touch an already-absolute one
(confirmed empirically, not assumed: a standalone rclpy probe against a
namespaced node showed a bare, unqualified YAML params key failing to match
at all, and an absolute topic name is resolved the same context-free way).
So this node's own default output ('odometry/filtered', relative) now
becomes '/ekf_global/odometry/filtered' via the namespace alone -- same
final name as before, zero downstream change (costmap_boundary_node.py and
everything else that already reads '/ekf_global/odometry/filtered' keeps
working unmodified) -- while odom0's own absolute '/odometry/filtered' is
structurally unreachable by that namespace rewrite, by construction, not by
convention some future edit could silently re-break. /tf, /diagnostics,
/rosout, and /parameter_events are unaffected by the namespace too --
confirmed against tf2_ros/transform_broadcaster.hpp and diagnostic_updater/
diagnostic_updater.hpp: both hardcode their own topic with a leading slash
('/tf', '/diagnostics'), same standard convention /rosout and
/parameter_events already follow at the rclcpp level, so map -> odom still
broadcasts on plain /tf and diagnostics still lands on the stack-wide shared
/diagnostics topic, not a namespaced fork of either. /set_pose (topic AND
service), /enable, /toggle DO move under /ekf_global/ (this instance's own
utility interfaces) -- confirmed nothing else in this workspace calls any of
them by their old, unnamespaced names.

A namespace changes how this node's params file must be keyed, though --
see ekf_global.yaml's own updated top-level key for
ekf_global_filter_node (now the fully-qualified '/ekf_global/
ekf_global_filter_node', not a bare 'ekf_global_filter_node') and that
file's own comment for why -- confirmed empirically that a bare key
silently fails to match a namespaced node (params fall back to
robot_localization's own built-in defaults with NO error raised), which
would have been a far worse, fully-silent regression than the bug being
fixed here.
"""

from f1tenth_params.param_defaults import get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ekf_global_config, ekf_global_config_desc = get_path_default('ekf_global_config')
    ekf_global_la = DeclareLaunchArgument(
        'ekf_global_config', default_value=ekf_global_config,
        description=ekf_global_config_desc)
    ekf_global_cpu_affinity_la = DeclareLaunchArgument(
        'ekf_global_cpu_affinity', default_value='0,1',
        description="Comma-separated core ids to pin ekf_global_filter_node "
                    "to via a 'taskset -c' launch prefix (vendored binary, "
                    "can't self-pin the way this workspace's own nodes do). "
                    "Unlike the self-pinning cpu_affinity params elsewhere "
                    "in this stack, an EMPTY value here is NOT a graceful "
                    "no-op -- 'taskset -c' with no core list is a shell-"
                    "level error, not an inherit-default-affinity fallback. "
                    "Must stay a valid, non-empty core list; to fully "
                    "disable pinning, remove this Node's prefix= argument "
                    "instead of emptying this value.")

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_filter_node',
        # namespace, NOT remappings= -- see module docstring's "SECOND REAL
        # BUG FOUND..." paragraph for why a remap on 'odometry/filtered'
        # silently also hijacked this node's own odom0 subscription (same
        # resolved string, remapping can't tell the two apart), and why a
        # namespace is structurally immune to that: it only ever rewrites
        # RELATIVE names, never odom0's own absolute '/odometry/filtered'.
        # Requires ekf_global.yaml's own top-level key for this node to be
        # fully-qualified ('/ekf_global/ekf_global_filter_node') -- see that
        # file's own comment; a bare key silently stops matching a namespaced
        # node (confirmed empirically, not assumed).
        namespace='ekf_global',
        output='screen',
        parameters=[LaunchConfiguration('ekf_global_config')],
        prefix=['taskset -c ', LaunchConfiguration('ekf_global_cpu_affinity')],
    )

    slam_pose_relay_node = Node(
        package='f1tenth_localization',
        executable='slam_pose_relay_node',
        name='slam_pose_relay_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_global_config')],
    )

    return LaunchDescription([
        ekf_global_la, ekf_global_cpu_affinity_la,
        ekf_global_node, slam_pose_relay_node,
    ])
