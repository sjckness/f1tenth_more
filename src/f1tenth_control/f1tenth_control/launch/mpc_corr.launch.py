"""mpc_corr: mpc_controller's MPC_corr.py, the navigation.launch.py else-branch
(enable_nav2:=false) drive-command source -- see f1tenth_navigation/launch/
navigation.launch.py, which is what actually includes this file; not meant to be
included directly by stack_bringup.launch.py/component_supervisor_node.py.

odom_stale_timeout_sec is MPC_corr.py's only ROS param (self.declare_parameter,
in-code default 0.5 matching stack_params.yaml's own default) -- wired through here
the same way mpc.launch.py wires andre_mpc_node.py's gains, so the two copies of the
default (here and in MPC_corr.py itself) only matter if this file's value is
overridden.

use_rti_solver/cpu_affinity/nice added by the MPC optimization pass (frequency/
bottleneck audit follow-up): use_rti_solver picks mpc_solver.py's OSQP real-time-
iteration path (default, per stack_params.yaml) vs. the original from-scratch
SLSQP path every tick (opt-out, for rollback without a code change).
cpu_affinity defaults to '10,11' (Jetson Orin AGX, 12 homogeneous cores) --
see MPC_corr.py's own module-level comment for the reasoning. NOT in
stack_params.yaml, since the right value is machine-specific, unlike every
other arg here -- same precedent f1tenth_perception/launch/detection.launch.py's
own yolo/detection_3d/obstacle_projector_cpu_affinity args already follow (their
own comments assumed mpc_corr already reserved 10,11 this way, pinning themselves
away from it -- confirmed via git history that this file's own default had
actually been left at '' since the file was created, never wired to a real value
despite that assumption; see the "CPU pinning fix" pass's own investigation for
the full writeup). nice stays an empty/0-default no-op -- CAP_SYS_NICE isn't
available on this deployment (see MPC_corr.py's own comment), so a non-zero
default here couldn't do anything anyway.

CPU AFFINITY NOW A taskset -c LAUNCH PREFIX, NOT self-pinning (thread-pinning-
leak fix, Step 6 reintroduction investigation -- applied here for consistency
even without fresh live confirmation of a leak on THIS node specifically:
Step 6 never reintroduced behavior/control, so mpc_corr was never under
measured load, but it used the exact same in-process
os.sched_setaffinity(0, cores)-in-__init__ mechanism as behavior_executor_node
and the three perception nodes, ALL FOUR of which were confirmed live to leak
the vast majority of their threads -- the mechanism itself is what's broken,
not something specific to those four; there is no reason to expect mpc_corr's
own copy behaves differently). cpu_affinity_la below still exists unchanged --
it now feeds the launch-level prefix= argument instead of a ROS param this
node reads on itself.

car_radius/avoidance_margin (safety-margin unification pass, following the
safety_stop_controller retirement): wired here for the first time this pass --
previously only ever the in-code ROS-param defaults in MPC_corr.py itself, now
sourced from stack_params.yaml's car_radius/obstacle_safety_margin_m, the same
two keys the BT's IsObstacleDetected/IsProximityTooClose behaviours also derive
their own thresholds from (see f1tenth_behavior/behavior_bringup.launch.py and
stack_params.yaml's own car_radius comment for the full picture). Numeric
defaults are unchanged (0.20/0.12) -- this only promotes them to a shared,
single source of truth.
"""

from f1tenth_params.param_defaults import get_default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_stale_timeout_sec_default, odom_stale_timeout_sec_desc = get_default(
        'odom_stale_timeout_sec')
    odom_stale_timeout_sec_la = DeclareLaunchArgument(
        'odom_stale_timeout_sec', default_value=str(odom_stale_timeout_sec_default),
        description=odom_stale_timeout_sec_desc)

    use_rti_solver_default, use_rti_solver_desc = get_default('use_rti_solver')
    use_rti_solver_la = DeclareLaunchArgument(
        'use_rti_solver', default_value=str(use_rti_solver_default),
        description=use_rti_solver_desc)

    car_radius_default, car_radius_desc = get_default('car_radius')
    car_radius_la = DeclareLaunchArgument(
        'car_radius', default_value=str(car_radius_default),
        description=car_radius_desc)

    avoidance_margin_default, avoidance_margin_desc = get_default('obstacle_safety_margin_m')
    avoidance_margin_la = DeclareLaunchArgument(
        'avoidance_margin', default_value=str(avoidance_margin_default),
        description=avoidance_margin_desc)

    cpu_affinity_la = DeclareLaunchArgument(
        'cpu_affinity', default_value='10,11',
        description=(
            "Comma-separated core ids to pin mpc_corr to via a 'taskset -c' "
            "launch prefix. Default '10,11' (this deployment's Jetson Orin "
            "AGX, 12 cores) reserves mpc_corr its own pair, away from "
            "yolo_detector_node (8,9) and detection_3d_node/"
            "obstacle_projector_node (6,7) -- see f1tenth_perception/launch/"
            "detection.launch.py's own cpu_affinity args, which already "
            "assumed this pair was reserved. Must stay a valid, non-empty "
            "core list -- 'taskset -c' with no core list is a shell-level "
            "error, not a graceful no-op; remove this Node's prefix= "
            "argument instead to fully disable pinning -- e.g. on a "
            "different machine/core count where these specific ids don't "
            "make sense; see MPC_corr.py's own module-level comment for how "
            "to pick new ones."))
    nice_la = DeclareLaunchArgument(
        'nice', default_value='0',
        description=(
            "Process niceness for mpc_corr. 0 (default): no-op. Negative "
            "values need CAP_SYS_NICE/root and fail non-fatally otherwise."))

    mpc_corr_node = Node(
        package='mpc_controller',
        executable='mpc_corr',
        name='mpc_corr',
        output='screen',
        prefix=['taskset -c ', LaunchConfiguration('cpu_affinity')],
        parameters=[{
            'odom_stale_timeout_sec': LaunchConfiguration('odom_stale_timeout_sec'),
            'use_rti_solver': LaunchConfiguration('use_rti_solver'),
            'car_radius': LaunchConfiguration('car_radius'),
            'avoidance_margin': LaunchConfiguration('avoidance_margin'),
            'nice': LaunchConfiguration('nice'),
        }],
    )

    return LaunchDescription([
        odom_stale_timeout_sec_la, use_rti_solver_la, car_radius_la, avoidance_margin_la,
        cpu_affinity_la, nice_la, mpc_corr_node,
    ])
