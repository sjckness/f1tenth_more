#!/usr/bin/env python3
"""Shared Jetson process-tuning helper: CPU affinity + scheduling priority.

Same mechanism as mpc_controller/MPC_corr.py's own
_apply_cpu_affinity_and_priority() (itself ported from the deleted
andre_mpc_opt_node.py, git c36e19f) -- reused here rather than reimplemented,
since three perception nodes (yolo_detector_node, detection_3d_node,
obstacle_projector_node) all need the identical os.sched_setaffinity/os.nice
logic. Extracted to one shared function instead of copy-pasted three times,
since (unlike MPC_corr.py, the only consumer of its own copy) this package
has three consumers of the exact same code.

Added by the perception-optimization pass following the latency audit: the
audit found yolo_detector_node's own callback compute is fast (~22ms
inference) but /camera/detections lags /camera/image_raw by ~278ms measured
via ros2 topic delay -- root-caused to CPU/executor contention (system load
average 9-14.6 on 12 cores during the audit window, yolo_detector_node's own
nonvoluntary:voluntary context-switch ratio ~6:1), not transport/QoS. Pinning
the perception nodes away from mpc_corr's already-reserved cores (10,11) and
away from wherever the ZED container concentrates is the fix this pass
applies.
"""

import os


def declare_cpu_affinity_params(node, default_cpu_affinity='', default_nice=0):
    """Declare the two ROS params this module reads. Call once, early in
    __init__, before apply_cpu_affinity_and_priority(). Split out from apply()
    so a node can declare/read the values before deciding whether/when to
    apply them, if it ever needs that -- none of the three current callers do,
    but apply() itself assumes the params already exist rather than
    declaring them implicitly (matches MPC_corr.py's own declare-then-apply
    two-step, not a hidden side effect).
    """
    node.declare_parameter('cpu_affinity', default_cpu_affinity)
    node.declare_parameter('nice', default_nice)


def apply_cpu_affinity_and_priority(node):
    """Pin `node`'s process and raise its scheduling priority (best effort).

    Reads the 'cpu_affinity' (comma-separated core ids, e.g. '8,9') and
    'nice' (int) parameters -- declare_cpu_affinity_params() must have been
    called first. Both default to no-op (empty string / 0) so a node that
    never gets a deployment-specific value keeps the OS's default affinity,
    exactly like MPC_corr.py's own behavior. Which core ids to use is a
    deployment-time choice made in each node's own launch file
    (f1tenth_perception/launch/detection.launch.py /
    f1tenth_perception/launch/camera.launch.py), not hardcoded here -- pick
    ids that are free of whatever's already concentrated there (check
    /proc/<pid>/status's Cpus_allowed_list, or `taskset -pc <pid>`, first).
    """
    spec = str(node.get_parameter('cpu_affinity').value).strip()
    if spec and hasattr(os, 'sched_setaffinity'):
        try:
            ncpu = os.cpu_count() or 1
            cores = {int(c) for c in spec.split(',') if c.strip() != ''}
            cores = {c for c in cores if 0 <= c < ncpu}
            if cores:
                os.sched_setaffinity(0, cores)
                node.get_logger().info(f'CPU affinity pinned to {sorted(cores)}')
            else:
                node.get_logger().warn(
                    f'cpu_affinity="{spec}" has no valid core (cpu_count={ncpu})'
                )
        except Exception as exc:
            node.get_logger().warn(f'Could not set CPU affinity: {exc}')

    nice_val = int(node.get_parameter('nice').value)
    if nice_val != 0:
        # Negative niceness needs CAP_SYS_NICE (root) -- this user has no
        # passwordless sudo on this Jetson, so a PermissionError here is
        # expected and non-fatal, same as MPC_corr.py's own comment notes.
        try:
            os.nice(nice_val)
            node.get_logger().info(f'Process nice set to {nice_val:+d}')
        except Exception as exc:
            node.get_logger().warn(
                f'Could not set nice {nice_val:+d} (need CAP_SYS_NICE/root): {exc}'
            )
