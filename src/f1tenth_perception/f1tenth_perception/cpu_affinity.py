#!/usr/bin/env python3
"""Shared Jetson process-tuning helper: scheduling priority (nice) only.

Same mechanism as mpc_controller/MPC_corr.py's own _apply_cpu_affinity_and_
priority() (itself ported from the deleted andre_mpc_opt_node.py, git
c36e19f) -- reused here rather than reimplemented, since three perception
nodes (yolo_detector_node, detection_3d_node, obstacle_projector_node) all
need the identical os.nice logic. Extracted to one shared function instead
of copy-pasted three times, since (unlike MPC_corr.py, the only consumer of
its own copy) this package has three consumers of the exact same code.

Added by the perception-optimization pass following the latency audit: the
audit found yolo_detector_node's own callback compute is fast (~22ms
inference) but /camera/detections lags /camera/image_raw by ~278ms measured
via ros2 topic delay -- root-caused to CPU/executor contention (system load
average 9-14.6 on 12 cores during the audit window, yolo_detector_node's own
nonvoluntary:voluntary context-switch ratio ~6:1), not transport/QoS. Pinning
the perception nodes away from mpc_corr's already-reserved cores (10,11) and
away from wherever the ZED container concentrates was the fix that pass
applied.

CPU AFFINITY REMOVED FROM THIS MODULE (thread-pinning-leak fix, Step 6
reintroduction investigation): this used to also declare a cpu_affinity
param and call os.sched_setaffinity(0, cores) on it, applied once, in-
process, from each node's own __init__ -- confirmed live to only restrict
the ONE thread executing that call, not the process: under Stage 4 load,
yolo_detector_node showed 4 of 36 threads pinned (32 unpinned, full 0-11
affinity) and detection_3d_node/obstacle_projector_node showed 1 of
33/22 (the rest unpinned) -- with unpinned threads from multiple nodes
actually caught executing on cores 0-4 (the EKF pair, slam_toolbox, and
behavior_executor_node's own reserved cores) at the moment of checking, not
just theoretically able to. Affinity is now an external `taskset -c <cores>`
launch prefix instead (see f1tenth_perception/launch/detection.launch.py's
own matching comment) -- it sets the mask before each node's first
instruction runs, so every thread it or any library (CUDA/TensorRT
inference threads included) ever spawns inherits it, with no in-process
code needed at all. Confirmed empirically across every OTHER pinned node in
this stack (ekf_node x2, slam_toolbox, foxglove_bridge, all already
taskset-prefixed): 100% of every one of their threads stayed on their
assigned cores through Step 6's full reintroduction sequence, including
Stage 4's saturated load, while every self-pinning node leaked.

nice stays here (same self-applied, main-thread-only mechanism as before) --
it was never the leak; only affinity was. Renamed declare_cpu_affinity_
params/apply_cpu_affinity_and_priority -> declare_nice_param/apply_nice to
match what this module actually does now, rather than leaving misleadingly-
named functions that no longer touch affinity at all.
"""

import os


def declare_nice_param(node, default_nice=0):
    """Declare the one ROS param this module reads. Call once, early in
    __init__, before apply_nice(). Split out from apply() so a node can
    declare/read the value before deciding whether/when to apply it, if it
    ever needs that -- none of the three current callers do, but apply()
    itself assumes the param already exists rather than declaring it
    implicitly (matches MPC_corr.py's own declare-then-apply two-step, not
    a hidden side effect).
    """
    node.declare_parameter('nice', default_nice)


def apply_nice(node):
    """Raise `node`'s process scheduling priority (best effort).

    Reads the 'nice' (int) parameter -- declare_nice_param() must have been
    called first. Defaults to no-op (0) so a node that never gets a
    deployment-specific value keeps the OS's default priority, exactly like
    MPC_corr.py's own behavior.
    """
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
