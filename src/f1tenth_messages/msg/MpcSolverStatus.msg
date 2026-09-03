# Per-tick MPC solver outcome, published by mpc_controller's MPC_corr.py on
# /mpc/solver_status (one message per control_loop solve, i.e. at the control
# rate, 1/ts).
#
# WHY THIS EXISTS: these fields were previously computed in MPC_corr.py and
# written ONLY to the ROS logger ("SOLVE/out | dt=... success=..."). That made
# "did the MPC actually converge?" unanswerable from a bag -- the 2026-09-01
# mission analysis had to recover 1285 solves by parsing ~/.ros/log/python3_<pid>_<epoch>.log,
# which only worked because log rotation had not yet discarded them. Publishing
# the same values costs one small message per tick and makes the question
# answerable offline, from the bag alone.

std_msgs/Header header

# Solver's own convergence verdict. False means the returned control was NOT a
# converged optimum -- the single most direct explanation for a stop.
bool success

# Raw backend status code. NOTE for anyone thresholding on this: OSQP's
# "solved" code is 1, NOT 0 -- a `status != 0` check reads a perfectly healthy
# run as 100% failure (this exact mistake was made and corrected during the
# 2026-09-01 analysis). Prefer `success`, or compare status_message.
int32 status

# Human-readable form of `status` (e.g. 'solved', 'primal infeasible'), so a
# bag/log is self-describing without cross-referencing the backend's enum.
string status_message

# Wall-clock time spent inside solve_mpc_step(), seconds. Compare against
# control_period_sec below: solve_dt > control_period_sec means the control
# loop missed its real-time deadline, which presents as a stop/stale command
# and is easily mistaken for a convergence failure.
float32 solve_dt_sec

# The loop's own budget (MPC_corr.py's ts), carried alongside so the deadline
# comparison needs no external knowledge of how the node was configured.
float32 control_period_sec

# Objective value at the returned solution; NaN if the backend did not report
# one. Useful for spotting divergence that still reports success.
float32 cost

# Which backend produced this solve: 'rti' (OSQP/RTI, use_rti_solver true) or
# 'slsqp' (the from-scratch nonlinear fallback).
string solver

# Number of hard boundary constraints active in this solve, and the count of
# obstacle disks passed in. Constraint-set churn between consecutive solves
# degrades warm-starting; recording both makes that measurable rather than
# inferred.
int32 n_boundary_constraints
int32 n_obstacles

# ---------------------------------------------------------------------------
# PREDICTED HORIZON (added after the mission-replay video work found it was the
# one thing a bag could not answer: the geometry the solver optimized over was
# computed every tick and then thrown away).
#
# These are info["x_pred"] from mpc_solver.py -- the horizon states rolled
# forward through the TRUE nonlinear model (_rollout_true_dynamics), NOT the
# RTI path's internal linearization, so they are the same "what will actually
# happen" prediction MPC_corr.py's own compute_predicted_clearance() consumes.
# One entry per horizon step k = 1..N (the CURRENT state x0 is NOT included --
# a consumer that wants a path starting at the car prepends its own pose).
#
# All four arrays are the same length, or ALL EMPTY: a solve that produced no
# usable prediction publishes nothing here rather than a partial/padded
# trajectory. Consumers must handle the empty case -- it is the normal state on
# a failed solve, not an error.
#
# FRAME: prediction_frame_id, which is the MPC's own world frame -- 'odom', the
# frame get_odom_topic() selects (raw /odom or the LOCAL EKF's
# /odometry/filtered), NEVER the global map-frame EKF. Same frame, and the same
# reason, as /mpc/corridor_markers (see _publish_corridor_markers' docstring):
# these points come straight from x0/build_straight_corridor's own X0/Y0.
# Anything drawing them in map must transform by the live map->odom TF.
string prediction_frame_id
float32[] pred_x      # [m]
float32[] pred_y      # [m]
float32[] pred_yaw    # [rad], psi of the state vector [X, Y, psi, v]
float32[] pred_v      # [m/s]
