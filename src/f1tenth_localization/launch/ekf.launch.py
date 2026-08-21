"""robot_localization EKF: fuses /odom (x, y, yaw, from vesc_to_odom_node)
with the VESC IMU (/sensors/imu/raw: yaw rate + linear acceleration) and
broadcasts the odom -> base_link transform. Replaces the old vesc_to_odom
in-node Kalman filter (vesc_to_odom_node's own publish_tf is left false).

cpu_affinity (added by the stack-wide CPU-budget investigation): ekf_node
is a vendored robot_localization binary, not this workspace's own source,
so it can't self-pin via a declared ROS param the way every other pinned
node in this stack does (see f1tenth_perception/cpu_affinity.py's own
docstring for that mechanism) -- `prefix=['taskset', '-c', ...]` is the
standard ROS 2 launch equivalent for a node whose source isn't ours to
modify.

CORRECTED (core-remap pass, following a live CPU-contention investigation --
see that pass's own report): the original ~15%-per-node estimate that
justified sharing this pair with behavior_executor_node/foxglove_bridge was
wrong, not just stale -- live `taskset -pc`/`ps -o %cpu` measurement (at
REST, no motion, no mission running) found this pair actually carrying
ekf_filter_node (38.0%) + ekf_global_filter_node (37.1%, added by the
later dual-EKF pass and never folded back into this estimate) +
foxglove_bridge (52.2%, itself >3x the assumed ~15%) +
behavior_executor_node (24.7% idle / ~45% documented under real BT
activity, see behavior_bringup.launch.py's own comment) = ~152% steady-
state demand on a 200% (2-core) budget -- confirmed saturated live
(cpu0/cpu1 both 99%+ busy via /proc/stat, at rest). This directly produced
`ekf_node`'s own "Failed to meet update rate!" warnings (up to ~59ms seen
against the 20ms/50Hz target -- see scripts/check_ekf_update_rate.py),
which under real motion is a plausible root cause of the large, rapid EKF
yaw oscillations that same investigation pass separately measured live.

foxglove_bridge and behavior_executor_node have since been moved OFF this
pair (see foxglove_bridge.launch.py/behavior_bringup.launch.py's own
matching comments) -- cores 0,1 are now reserved for JUST the two EKF
instances (ekf_filter_node here + ekf_global_filter_node, ekf_global.
launch.py, which intentionally keeps sharing this SAME pair with this one,
not a rigid one-core-each split -- letting the OS scheduler balance the
two instances across both cores tolerates either one's load spiking
without a hard per-core ceiling, matching the two-cores-for-a-cooperating-
pair reasoning this stack already uses for the local+global EKF's own
lockstep 50Hz design).

Still NOT live-verified under real motion (this correction pass had no
live hardware -- see its own report): at-rest measurement doesn't capture
real-motion CPU load (more odom/imu message traffic, more BT/mission
activity, MPC running). Re-check scripts/check_ekf_update_rate.py's own
output (or grep component_supervisor's own localization log directly)
after a real motion test once hardware is reconnected -- that is the
concrete, observable confirmation this fix actually worked, not just that
the reasoning was sound.
"""

from f1tenth_params.param_defaults import get_path_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config, ekf_config_desc = get_path_default('ekf_config')
    ekf_la = DeclareLaunchArgument(
        'ekf_config', default_value=ekf_config, description=ekf_config_desc)
    ekf_cpu_affinity_la = DeclareLaunchArgument(
        'ekf_cpu_affinity', default_value='0,1',
        description="Comma-separated core ids to pin ekf_node to via a "
                    "'taskset -c' launch prefix (vendored binary, can't "
                    "self-pin the way this workspace's own nodes do). "
                    "Unlike the self-pinning cpu_affinity params elsewhere "
                    "in this stack, an EMPTY value here is NOT a graceful "
                    "no-op -- 'taskset -c' with no core list is a shell-"
                    "level error, not an inherit-default-affinity fallback. "
                    "Must stay a valid, non-empty core list; to fully "
                    "disable pinning, remove this Node's prefix= argument "
                    "instead of emptying this value.")

    # enable_nav2 used to be redeclared here (never actually used by this file's own
    # Node) purely so it'd be registered before stack_bringup.launch.py's own
    # LaunchConfiguration('enable_nav2') resolved it -- an ordering trick. Now that
    # enable_nav2 is one of the 6 stack-wide branching args (see f1tenth_bringup/
    # config/stack_params.yaml), it's a plain yaml value read directly wherever
    # needed, so there's nothing left for this file to declare or forward.

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
        prefix=['taskset -c ', LaunchConfiguration('ekf_cpu_affinity')],
    )

    return LaunchDescription([ekf_la, ekf_cpu_affinity_la, ekf_node])
