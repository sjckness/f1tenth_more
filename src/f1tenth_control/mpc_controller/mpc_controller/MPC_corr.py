import json
import math
import os
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_messages.msg import (
    BoundaryConstraintArray, MpcSolverStatus, Obstacle2DArray, TurnGoal)
from f1tenth_params.param_defaults import get_odom_topic, get_value
from mpc_controller.mpc_solver import solve_mpc_step
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def _resolve_debug_output_path(filename: str) -> Path:
    """Resolve <ws_root>/src/f1tenth_control/corridors_jsons/<filename>,
    reliably reaching the real source tree regardless of whether this
    workspace was built with --symlink-install.

    Generalized (old-workspace-name cleanup pass) from what was originally
    just _resolve_corridor_debug_path() (no filename argument, hardcoded to
    corridor_debug.jsonl) -- now also used for error_log_path/
    control_log_path below, which previously hardcoded Path.home() /
    'ros2_f110_ws' / ..., a stale reference to this project's old workspace
    name/layout (broken for anyone -- including the current setup -- not on
    that exact original machine, same class of bug as the one this function
    itself was already written to fix for the corridor debug path; see the
    history below).

    Same technique as f1tenth_diagnostics' sensor_covariance_calibration_node.
    resolve_source_vesc_yaml_path() and vesc_tuning's speed_sweep_diagnostic_
    node._default_results_dir() -- reimplemented standalone here rather than
    imported, to avoid an unwanted cross-package dependency between
    mpc_controller and either of those packages for what is otherwise a
    self-contained log-path computation.

    Previously (see git history) this assumed ament_python always
    editable-installs a package's own .py modules (egg-link back to source)
    "regardless of build/install layout" -- live-verified FALSE on this
    workspace (no --symlink-install): this file's own __file__ resolves to a
    PLAIN COPY under install/mpc_controller/lib/python3.10/site-packages/...,
    so a naive parents[2] anchor off __file__ only reached
    install/mpc_controller/lib/python3.10/, not
    src/f1tenth_control/corridors_jsons/ as intended -- same class of bug as
    the vesc.yaml path-resolution issue fixed in sensor_covariance_
    calibration_node.py. What IS reliable regardless of --symlink-install is
    colcon's own workspace layout convention: <ws_root>/install/ and
    <ws_root>/src/ are always siblings. So: walk up this file's resolved path
    looking for an 'install' directory; if found, its parent IS the
    workspace root, and 'src' sits right next to it. If no 'install' ancestor
    is found at all, this file must already be running from source (e.g.
    --symlink-install, or a non-colcon dev setup), so fall back to anchoring
    directly off it, as before.
    """
    this_file = Path(__file__).resolve()
    for parent in this_file.parents:
        if parent.name == 'install':
            ws_root = parent.parent
            return (ws_root / 'src' / 'f1tenth_control' / 'corridors_jsons'
                     / filename)

    # No 'install' ancestor -- this file's own resolved location IS inside
    # src/ already.
    # parents[0]=mpc_controller/mpc_controller (this file's dir),
    # parents[1]=mpc_controller (the package), parents[2]=f1tenth_control
    # (the top-level dir housing the mpc_controller and f1tenth_control
    # packages) -- corridors_jsons lives there, not inside any one package.
    return this_file.parents[2] / 'corridors_jsons' / filename


def _boundary_to_world(
        normal_x: float, normal_y: float, offset: float,
        robot_x: float, robot_y: float, robot_yaw: float) -> Tuple[float, float, float]:
    """Transform one (normal, offset) hard-boundary halfspace from
    base_link-relative (as published by costmap_boundary_node.py,
    f1tenth_costmap -- see f1tenth_messages/BoundaryConstraint.msg's own
    note) into this MPC's own world/odom frame (self.x/y/yaw, the SAME
    frame x0/x_ref live in -- see robot_to_global's own docstring for why
    that transform exists at all: MPC's internal state is NOT base_link-
    relative).

    Derivation: a base_link-frame point p_b satisfies normal_b . p_b <=
    offset_b. p_b relates to its world-frame equivalent p_w via
    p_b = R(-yaw) . (p_w - [x, y]) (the inverse of robot_to_global).
    Substituting: normal_b . R(-yaw) . (p_w - [x,y]) <= offset_b
    => (R(yaw) . normal_b) . p_w <= offset_b + (R(yaw) . normal_b) . [x, y]
    (using R(-yaw)^T = R(yaw) for a rotation matrix). So:
    normal_w = R(yaw) . normal_b, offset_w = offset_b + dot(normal_w, [x, y]).
    """
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    normal_world_x = cos_yaw * normal_x - sin_yaw * normal_y
    normal_world_y = sin_yaw * normal_x + cos_yaw * normal_y
    offset_world = offset + normal_world_x * robot_x + normal_world_y * robot_y
    return normal_world_x, normal_world_y, offset_world


def _pose_odom_to_map(
        x: float, y: float, psi: float,
        tf_x: float, tf_y: float, tf_yaw: float) -> Tuple[float, float, float]:
    """Express an odom-frame planar pose (x, y, psi) in the MAP frame, given
    the map -> odom transform (tf_x, tf_y, tf_yaw).

    (tf_x, tf_y, tf_yaw) is the edge EXACTLY as tf2 broadcasts and returns it
    for `lookup_transform('map', 'odom', ...)`: header.frame_id 'map',
    child_frame_id 'odom', i.e. the pose of the odom frame's origin expressed
    in map -- so applying it to something odom-frame yields the map-frame
    equivalent, which is this direction. Published by ekf_global_filter_node
    (robot_localization, world_frame: map, publish_tf: true -- see
    f1tenth_bringup/config/ekf_global.yaml) in 'ekf' mode.

    WHY THIS EXISTS (the bug it fixes): this stack's dual-EKF split puts the
    car's REAL heading drift into map -> odom, not into odom-frame yaw. The
    local EKF (world_frame: odom) is a smooth dead-reckoning integrator with
    no absolute yaw reference; slam_toolbox's absolute correction lands in the
    global EKF, which absorbs it into map -> odom. Measured over one post-fix
    mission: map-frame yaw drifted 13.1 deg while map -> odom's own yaw drifted
    12.8 deg in lockstep, leaving the odom-frame yaw the control loop reads
    (self.yaw) under 0.5 deg the whole time. So a straight move's reference
    line frozen in ODOM coordinates is frozen in a frame that is itself
    rotating with the car's real-world error -- it never presents a heading
    error for w_psi/w_term to correct. Freezing it in MAP instead, and
    reprojecting it back into the live odom frame every cycle
    (_pose_map_to_odom, the exact inverse of this), makes that real drift
    visible to the solver as an ordinary tracking error, without touching the
    odom-frame state x0 the dynamics/solve themselves run on.

    Pure and module-level for the same reason _boundary_to_world and
    _project_onto_line are: MPCController's constructor opens real log files
    and starts a real timer, so anything that needs direct unit coverage lives
    outside it.
    """
    cos_t, sin_t = math.cos(tf_yaw), math.sin(tf_yaw)
    x_map = tf_x + cos_t * x - sin_t * y
    y_map = tf_y + sin_t * x + cos_t * y
    psi_map = math.atan2(math.sin(psi + tf_yaw), math.cos(psi + tf_yaw))
    return x_map, y_map, psi_map


def _pose_map_to_odom(
        x: float, y: float, psi: float,
        tf_x: float, tf_y: float, tf_yaw: float) -> Tuple[float, float, float]:
    """Exact inverse of _pose_odom_to_map (see that function for the frame
    convention and for why either direction exists at all): express a
    map-frame planar pose in the CURRENT odom frame, given the SAME
    `lookup_transform('map', 'odom', ...)` edge -- not a separately looked-up
    'odom' -> 'map' one. Keeping both directions off one lookup is what makes
    capture-then-reproject an identity at move start by construction, rather
    than something that has to be argued about numerically.
    """
    cos_t, sin_t = math.cos(tf_yaw), math.sin(tf_yaw)
    dx = x - tf_x
    dy = y - tf_y
    x_odom = cos_t * dx + sin_t * dy
    y_odom = -sin_t * dx + cos_t * dy
    psi_odom = math.atan2(math.sin(psi - tf_yaw), math.cos(psi - tf_yaw))
    return x_odom, y_odom, psi_odom


def _project_onto_line(
        point: Tuple[float, float], line_origin: Tuple[float, float],
        line_yaw: float) -> float:
    """Signed along-line progress of `point` past `line_origin`, measured
    along the direction `line_yaw` -- i.e. the scalar s in
    line_origin + s * (cos(line_yaw), sin(line_yaw)) that is closest to
    `point`. Lateral offset is deliberately discarded: that is the whole
    point of the quantity.

    WHY THIS EXISTS: the goal_distance termination check needs "how far along
    the move's own direction has the car actually got", not "how far is the
    car from where it started". Those differ by exactly the lateral deviation
    an obstacle deflection (or a mechanical drift) leaves behind. The
    straight-line form (math.hypot from goal_start_xy) therefore counts a
    sideways detour as progress and ends the move early, by more the further
    the car was pushed. Nothing pulls that deviation back out any more either
    -- the corridor's origin tracks the car rather than homing onto the
    original line (see build_straight_corridor's goal_distance branch, and
    "THE PRICE" in its own comment) -- so a sideways offset persists for the
    rest of the move and this is the ONLY thing keeping "6 m" meaning 6 m
    along the intended direction.

    ONE CALLER, deliberately: control_loop's goal_distance termination check.
    build_straight_corridor used this too until the corridor stopped homing
    laterally; it does not any more, and must not start again -- the corridor's
    origin and the move's progress anchor are two separate quantities now (see
    the two-anchor note at that branch's own assignment).

    Negative means the car is BEHIND the origin along that direction (it
    happens: an avoidance manoeuvre can back the projection up), and is
    returned as-is rather than clamped, so a caller that cares can see it.

    Pure and module-level for the same reason _select_live_boundaries below
    is: MPCController's constructor opens real log files and starts a real
    timer, so anything that needs direct unit coverage lives outside it.
    """
    dx = float(point[0]) - float(line_origin[0])
    dy = float(point[1]) - float(line_origin[1])
    return dx * math.cos(line_yaw) + dy * math.sin(line_yaw)


def _select_live_boundaries(
        values: List[Tuple[float, float, float]], last_time: Optional[float],
        now_sec: float, timeout_sec: float) -> List[Tuple[float, float, float]]:
    """Pure staleness gate factored out of _get_live_boundaries so this
    pass's own single-source collapse (see that method's docstring -- was
    previously two independently-staled sources OR-combined, now one) has
    a directly pure-function-testable piece, matching this test suite's
    own established pure-function-level convention (mpc_controller's test
    suite deliberately never constructs a full MPCController Node --
    MPCController.__init__ opens real log files and starts a real control-
    loop timer as side effects, unlike the lightweight nodes elsewhere in
    this workspace that DO get constructed directly in their own tests).

    Returns `values` unchanged if `last_time` is not None and
    now_sec - last_time < timeout_sec, else an empty list -- the exact
    same now_sec - last_time < timeout pattern _update_active_odom already
    uses for hw/sim odom staleness."""
    if last_time is not None and (now_sec - last_time) < timeout_sec:
        return list(values)
    return []


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_corr')

        # =========================
        # Stato robot
        # =========================
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.v: Optional[float] = None

        # =========================
        # Odom sources (hardware + sim, hardware priority)
        # =========================
        # Each source keeps its own state and last-received clock time;
        # control_loop's _update_active_odom() picks which one feeds
        # self.x/self.y/self.yaw/self.v each tick.
        self.hw_x: Optional[float] = None
        self.hw_y: Optional[float] = None
        self.hw_yaw: Optional[float] = None
        self.hw_v: Optional[float] = None
        self.hw_odom_last_time: Optional[float] = None

        self.sim_x: Optional[float] = None
        self.sim_y: Optional[float] = None
        self.sim_yaw: Optional[float] = None
        self.sim_v: Optional[float] = None
        self.sim_odom_last_time: Optional[float] = None

        self.active_odom_source: Optional[str] = None
        self.odom_stale_timeout_sec = float(
            self.declare_parameter('odom_stale_timeout_sec', 0.5).value
        )

        # =========================
        # Goal distance (runtime command via /mpc/goal_distance)
        # =========================
        self.goal_distance: Optional[float] = None
        self.goal_start_xy: Optional[Tuple[float, float]] = None
        self.goal_reached = False
        self._no_goal_warned = False

        # ---- Frozen straight-move anchor, map-frame (drift-correction pass) --
        # goal_start_xy/psi_init_corridor above freeze the move's reference line
        # in ODOM coordinates, which is the frame this stack's dual-EKF split
        # deliberately keeps free of absolute-heading error -- see
        # _pose_odom_to_map's own docstring for the measurement and the full
        # reason that made the frozen line unable to present a heading error to
        # the solver at all.
        #
        # goal_anchor_map: (x, y, psi) of the move's start pose in the MAP frame,
        #   captured once in goal_distance_callback and never touched again for
        #   that move. None means "no map-frame anchor for this move" -- either
        #   no move is active, or the map -> odom lookup failed at move start
        #   (see that callback), in which case everything below degrades to the
        #   pre-existing odom-frame-only behaviour rather than blocking driving.
        # goal_anchor_odom: that same anchor reprojected into the CURRENT odom
        #   frame, refreshed once per control_loop tick by _refresh_goal_anchor().
        #   Read by two consumers that take DIFFERENT parts of it and must not be
        #   conflated: control_loop's progress check uses the whole pose (x, y AND
        #   psi) as the move's frozen origin and direction, while build_straight_
        #   corridor's goal_distance branch takes ONLY the psi -- its own origin
        #   tracks the car (see that branch's two-anchor note). Seeded at move
        #   start with the raw odom anchor, which is what the reprojection
        #   evaluates to at that instant anyway, so it is never None while a move
        #   is active and there is always a last-known-good value to hold.
        self.goal_anchor_map: Optional[Tuple[float, float, float]] = None
        self.goal_anchor_odom: Optional[Tuple[float, float, float]] = None

        # Frame names for the lookup above. Declared rather than hardcoded,
        # matching semantic_layer_node.py's own map_frame/base_frame params;
        # the defaults are the real, live names this stack broadcasts (ekf_
        # global.yaml's own map_frame/odom_frame keys, and raw_odom_map_tf_
        # node.py's hardcoded 'map'/'odom' header/child ids).
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)

        # Gate on the map -> odom edge being a genuinely INDEPENDENT absolute
        # correction, which is only true in localization_source == 'ekf'.
        # In 'raw_odom' mode raw_odom_map_tf_node.py mirrors /odom's own pose
        # verbatim into map -> odom, so that edge IS the car's pose: reprojecting
        # a captured anchor through it would pin the reference line to the
        # vehicle instead of the world -- strictly worse than the odom-frame
        # freeze it replaces, not merely no better. Default is therefore derived
        # from localization_source (the same single source of truth
        # get_odom_topic() above already follows) rather than a bare True, and
        # stays an explicit param so a future map -> odom source can opt in
        # without a code change.
        self.use_map_frame_goal_anchor = bool(self.declare_parameter(
            'use_map_frame_goal_anchor',
            get_value('localization_source') == 'ekf').value)

        # Staleness ceiling for the map -> odom edge used by the reprojection
        # (see _lookup_map_odom's own "AGE GUARD" note for the measurement and
        # for why a FRESH transform's age here is NEGATIVE, not zero).
        # 0.2s ~= 6 cycles of the global EKF's real ~32Hz output, so a healthy
        # stack never trips it while a genuine delivery stall does immediately.
        self.map_odom_max_age_sec = float(self.declare_parameter(
            'map_odom_max_age_sec', 0.2).value)

        # =========================
        # Goal pose (runtime command via /mpc/goal_pose) -- position-only
        # arrival this pass, no final-yaw alignment (see build_straight_corridor/
        # control_loop for the scope note). Mutually exclusive with goal_distance
        # mode: whichever topic was published to most recently wins -- each
        # callback clears the other mode's state (see goal_pose_callback/
        # goal_distance_callback).
        # =========================
        self.goal_pose_xy: Optional[Tuple[float, float]] = None
        self.goal_pose_yaw: Optional[float] = None  # stored but unused by corridor/termination logic this pass
        self.pose_goal_reached = False
        self.pose_goal_tolerance = float(
            self.declare_parameter('pose_goal_tolerance', 0.15).value
        )

        # External hold (see /mpc/hold subscription below): freezes control_loop's
        # output at zero without touching goal_start_xy/goal_reached/self.last_u, so
        # releasing it resumes exactly where the move left off -- deliberately
        # independent of goal_distance/goal_reached, which mission_manager's
        # HandleObjectAction must not repurpose for holding (see that behaviour's
        # docstring for why).
        self.hold = False

        # =========================
        # Ostacoli (live, no persistence -- overwritten each /perception/obstacles_2d
        # message; no matching against previous frames, no timeout/decay)
        # =========================
        self.obstacles_global_live: List[Tuple[float, float, float]] = []

        # =========================
        # Hard boundary constraints (see mpc_solver.py's own module
        # docstring, "Hard boundary constraints" section) -- up to 3
        # slots (front/left/right), ALL from costmap_boundary_node's own
        # single /costmap/boundaries topic (f1tenth_costmap) as of the
        # dual-EKF + costmap-derived-MPC-boundaries pass. Retires the
        # earlier two-topic/two-callback design (wall_detector_node's own
        # /perception/front_wall_boundary + lidar_boundary_node's own
        # /perception/lidar_boundaries, f1tenth_perception, both nodes now
        # deleted) -- one source now produces all three directions, so one
        # subscription/callback/staleness check replaces what used to be
        # two of each (_boundary_callback_common, the shared body between
        # the two old callbacks, is gone with them -- nothing left to
        # share once there's only one caller).
        #
        # Published base_link-relative (see f1tenth_messages/
        # BoundaryConstraint.msg's own note on frames); transformed to
        # WORLD/odom frame immediately in the callback (using self.x/y/yaw
        # AS OF message arrival as a proxy for "the pose at the source's
        # own measurement time") -- same approximation
        # obstacles_2d_callback's own robot_to_global call already makes
        # for obstacles, not a fresh independent judgment call. Staleness
        # handling reuses odom_stale_timeout_sec/the SAME now_sec -
        # last_time < timeout pattern _update_active_odom already uses for
        # hw/sim odom -- see _get_live_boundaries below, not a new timeout
        # mechanism.
        # =========================
        self.costmap_boundaries_world: List[Tuple[float, float, float]] = []
        self.costmap_boundaries_last_time: Optional[float] = None

        # =========================
        # MPC / modello
        # =========================
        self.last_u = np.array([0.0, 0.0], dtype=float)

        self.wheel_radius = 0.05
        self.ts = 0.1
        self.N = 20

        # UPGRADE: rough footprint radius used for the predicted-clearance check.
        # In-code default (0.20) matches stack_params.yaml's car_radius key, the
        # shared safety-margin unification pass's single source of truth for this
        # value -- see that key's own comment for why it's now also read (via the
        # same default) by the BT's IsObstacleDetected/IsProximityTooClose
        # behaviours, in a separate process, deriving their own thresholds off it.
        self.car_radius = float(self.declare_parameter('car_radius', 0.20).value)

        # Extra safety buffer beyond the car's physical footprint. Both active
        # obstacle-avoidance mechanisms now size their trigger/safety distance off
        # car_radius + avoidance_margin: this file's compute_local_target (R_safe,
        # below) directly, and mpc_solver.py's planner_cost_corridor indirectly via
        # corridor["car_radius"]/corridor["avoidance_margin"] (set in control_loop
        # below). Previously neither accounted for car_radius at all -- compute_local_
        # target used corridor["d_safe"] (self.dmin, 0.9m, the disabled hard
        # constraint's own value) and planner_cost_corridor used a hardcoded 0.3m.
        # In-code default (0.12) matches stack_params.yaml's obstacle_safety_margin_m
        # key -- see car_radius's own comment above.
        self.avoidance_margin = float(self.declare_parameter('avoidance_margin', 0.12).value)

        # Solver selection: real-time-iteration (linearize once + one warm-started
        # OSQP QP solve per tick) vs. the original from-scratch nonlinear SLSQP
        # solve every tick. Default true per the MPC optimization pass -- the
        # frequency/bottleneck audit measured SLSQP's solve_dt averaging 93ms of a
        # 112.6ms loop against the 100ms/10Hz budget; RTI is the fix. SLSQP is kept
        # as an explicit opt-out (mpc_solver.py still carries both paths behind one
        # solve_mpc_step() entry point) rather than removed, so a regression can be
        # rolled back with a launch arg, not a code change.
        self.use_rti_solver = bool(self.declare_parameter('use_rti_solver', True).value)

        # Hard boundary constraints (see mpc_solver.py's own module docstring
        # and _get_live_boundaries below). Default FALSE as of Andreas's
        # explicit request to run the MATLAB-comparison configuration by
        # default rather than as an opt-in launch arg -- MATLAB has no such
        # constraint, so this is the closer match while that comparison is
        # ongoing. Still a launch-arg opt-IN (mpc_corr.launch.py's own
        # use_hard_boundary_constraints) to turn them back on, same pattern
        # as use_rti_solver just above, not a code change. When false,
        # _get_live_boundaries always returns [] regardless of what
        # costmap_boundary_node is actually publishing -- solve_mpc_step
        # then sees boundaries=[], identical to no source ever having been
        # live. NOTE: this means the RTI solve currently runs with NO hard
        # wall constraint of any kind -- only the soft w_obs deflection cost
        # -- until this default is revisited.
        self.use_hard_boundary_constraints = bool(
            self.declare_parameter('use_hard_boundary_constraints', False).value)

        # Max tangential shove applied to the lookahead target when it lands
        # inside an obstacle's safety radius -- see compute_local_target,
        # which is the only reader. A DECLARED PARAMETER whose default comes
        # from stack_params.yaml via get_value(), same single-sourcing as
        # corridor_update_period below: the number lives in that file and
        # nowhere else, so a bare `ros2 run` that bypasses the launch file
        # still gets the deployed value, and the test stand-in in
        # test_corridor_lookahead_target.py reads the same key rather than
        # carrying a copy.
        self.obstacle_target_shift = float(
            self.declare_parameter(
                'obstacle_target_shift_m',
                get_value('obstacle_target_shift_m')).value)

        # nice: see _apply_nice() below, called near the end of __init__.
        # Defaults to no-op (0) so this node's priority is unchanged unless a
        # deployment explicitly opts in via mpc_corr.launch.py. CPU affinity
        # is now a `taskset -c` launch prefix instead -- see that method's
        # own docstring and mpc_corr.launch.py's matching comment.
        self.declare_parameter('nice', 0)

        self.params = {
            "L": 0.305,
            "lr": 0.17,
        }

        # Corridoio MATLAB-like
        #
        # Narrowed to 1/3 width on request (2026-09-07): corr_wmin 1.3 ->
        # 0.4333, corr_wmax 2.3 -> 0.7667. Those are kept.
        #
        # corr_L_base was shortened to 1/2 (3.0 -> 1.5) in the same request and
        # has been RESTORED to 3.0 (2026-09-08). It could not stay at 1.5: the
        # lookahead target is derived from it (see _corridor_lookahead below)
        # and must sit BEYOND the horizon's physical reach, N*ts*vdes = 20 *
        # 0.1 * 0.5 = 1.0 m, or the terminal cost stops acting as a direction
        # pull and becomes an arrival target. At L = 1.5 the derived lookahead
        # is 0.75 m -- inside the reach, and short enough that the arclength
        # advance clamped to the corridor's last index on every single cycle.
        # The WIDTH request is independent of the length and is untouched.
        # Every one of these is a bare literal with no ROS parameter behind it
        # (unlike corridor_update_period directly below, which IS a declared
        # param wired through stack_params.yaml), so there is no launch-time
        # override and no config file that records what they used to be.
        #
        # corr_wmin/corr_wmax are HALF-widths, not full widths: build_
        # straight_corridor places the walls at C +/- w*n (see its own
        # P_L0/P_R0/P_L1/P_R1 construction), so the corridor spans 2*w
        # across. Scaling the half-width by 1/3 therefore scales the full
        # corridor width by 1/3 as well -- the requested change, not a
        # sixth of it.
        #
        # NOTE these are not independent of build_straight_corridor's own
        # floor/clamp on length: its goal_distance branch uses
        # max(corr_L_base, 1.0) and its goal_pose branch clips the
        # car-to-goal distance into [1.0, corr_L_base].
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 0.4333
        self.corr_wmax = 0.7667

        # LOOKAHEAD IS DERIVED FROM CORRIDOR LENGTH, NOT A SECOND CONSTANT.
        # It used to be a bare `lookahead = 1.5` inside compute_local_target
        # while corr_L_base was 1.5 -- two independent literals describing one
        # geometric relationship, which silently collided: s_target =
        # s_cum[idx] + 1.5 on a corridor 1.5 m long is >= s_cum[-1] for every
        # idx, so searchsorted clamped to the last index on EVERY cycle and the
        # "advance along arclength" never operated at all. The target was
        # simply the corridor endpoint.
        #
        # The reference implementation runs lookahead 1.5 on L 3.0 -- the
        # target sits mid-corridor and advances -- so the ratio, not the
        # absolute value, is the tuned quantity.
        self.corr_lookahead_frac = 0.5
        # Floor, as a multiple of the horizon's physical reach (N*ts*vdes).
        # The reference's target sat beyond its horizon's 0.28 m reach, which
        # is what makes the terminal cost behave as a direction pull
        # (pure-pursuit-like "steer toward") rather than an arrival target
        # ("get to this point in N steps"). Keeping the lookahead outside the
        # reach preserves that character across horizon changes.
        self.corr_lookahead_reach_margin = 1.25
        self.corr_p = 1.8
        self.q = 1.2
        self.corr_epsiMax = math.radians(35.0)
        self.corr_tmax = math.tan(math.radians(35.0))

        # S-curve heading-blend shape (build_straight_corridor), ported from
        # f110_autonomy -- see that method's own comment for the full
        # rationale and the receding-horizon caveat at this pass's rebuild
        # rate.
        # DECLARED PARAMETERS, not bare literals, because the measurement
        # below says these two -- not which end of the blend is frozen -- are
        # what governs whether the heading return converges.
        #
        # HOW MUCH OF ITS HEADING ERROR THE CORRIDOR ASKS THE CAR TO CORRECT.
        # With psiStart live and psiEnd frozen, the bearing from the car to
        # the lookahead target is only PART of the way back to the reference
        # heading -- the S-curve deliberately spreads the return over the
        # corridor length. At u_start/u_end = 0.10/0.70 and lookahead 1.5 on
        # L 3.0, that bearing demands just 24.7% of the error. The corridor is
        # rebuilt from the LIVE pose every tick, so the other 75% is re-granted
        # every tick and the loop settles at a NON-ZERO heading error while
        # drifting laterally.
        #
        # Measured open-loop-to-closed-loop, car started 0.35 rad off the
        # reference heading, 8.0 s, N=20, ts=0.1, vdes=0.5, normalised
        # weights, corridor rebuilt every tick:
        #
        #   u_start/u_end   final psi     final lateral offset
        #   0.10 / 0.70      +0.1344 rad      +1.010 m   <- current default
        #   0.00 / 0.70      +0.0486 rad      +0.675 m
        #   0.00 / 0.40      +0.0011 rad      +0.265 m
        #   0.00 / 0.20      +0.0000 rad      +0.163 m
        #   0.00 / 0.05      +0.0000 rad      +0.123 m   <- ~= both-ends-frozen
        #
        # Monotone: the tighter the blend, the more of the error the corridor
        # demands per rebuild and the closer the return gets to zero. Left at
        # the reference implementation's own 0.10/0.70 by default, because
        # that is what the port is specified to reproduce -- but exposed so
        # the table above can be walked on the car instead of in a rebuild.
        self.corr_turn_u_start = float(
            self.declare_parameter('corr_turn_u_start', 0.10).value)
        self.corr_turn_u_end = float(
            self.declare_parameter('corr_turn_u_end', 0.70).value)

        # Which end of the heading blend is frozen (see build_straight_corridor).
        # True  -- psiStart = LIVE yaw, psiEnd = FROZEN anchor heading. The
        #          reference implementation's behaviour.
        # False -- both ends frozen at the anchor heading, i.e. a straight
        #          corridor through the live position along the reference
        #          direction. THE DEFAULT since 2026-09-08, and what the stack
        #          ran before the port.
        #
        # The default is stack_params.yaml's, not a literal here -- that key
        # carries the measurement this flip rests on. Summary: the two
        # geometries do NOT rank the way the port expected. Across both
        # rebuild periods (see corridor_update_period below), both-ends-frozen
        # returns the heading to zero and the reference blend does not, inside
        # the 8 s window; run long enough the blend does converge, but by then
        # it has traded away ~0.9 m of lateral offset that nothing restores.
        self.corridor_heading_return = bool(
            self.declare_parameter(
                'corridor_heading_return',
                get_value('corridor_heading_return')).value)

        # aggiornamento corridoio
        #
        # Was a flat 1.0s (10x slower than control_loop's own 10Hz tick,
        # self.ts below) -- found via Foxglove observation of ~1Hz corridor
        # updates against an ~80Hz-capable solver, confirmed live via
        # costmap_boundary_node's own front_clearance timer (20Hz, same
        # underlying pose source family) making a genuinely-1Hz corridor look
        # like a mismatch. Investigated rather than just raised: the ONLY
        # reason a value this conservative existed at all was the adjacent
        # comment at this rebuild's own call site ("corridor geometry stays on
        # the slow cadence -- it barely changes while driving straight") --
        # an argument for why leaving it slow was harmless, not that
        # rebuilding faster would be harmful. build_straight_corridor() itself
        # is a handful of vectorized numpy ops over corr_N=120-element arrays
        # -- sub-millisecond, nowhere near the ~10ms-class OSQP/RTI solve that
        # actually owns this tick's budget (self.ts=0.1s=100ms) -- so there
        # was no performance reason for 1.0s either.
        #
        # Set to HALF self.ts, not a bare copy of it: the gate below
        # (`now_sec - last_corridor_time >= corridor_update_period`) is
        # checked once per control_loop tick, so corridor rebuild can never
        # exceed control_loop's own 10Hz rate regardless of how low this is
        # set -- tying it to a bare `self.ts` risked the gate occasionally
        # missing a tick on ordinary timer-scheduling jitter (elapsed time
        # landing at e.g. 0.0999s instead of exactly 0.1s), silently falling
        # back toward a slower effective rate some ticks. Half of self.ts
        # guarantees the gate always passes every tick with margin, which in
        # practice means "rebuild every control_loop tick" -- the fastest
        # this can genuinely go without decoupling corridor rebuild onto its
        # own timer (a bigger change, not done here: this alone is already a
        # 10x improvement, 1Hz -> 10Hz, cutting the ~15cm stale-corridor gap
        # observed at test speed down to ~1.5cm).
        #
        # AND YET 1.0 IS WHAT SHIPS. Everything above this line is an argument
        # that nothing STOPS the rebuild going to 10Hz -- it is cheap, and the
        # gate can absorb it. That is not the same as an argument that faster
        # is BETTER, and when the question was finally measured rather than
        # reasoned about (2026-09-08, closed-loop, see stack_params.yaml's own
        # corridor_update_period comment for the four-cell table) the slower
        # rebuild won at both corridor geometries. The ~15cm stale-corridor
        # gap the 10Hz argument was built to close is real; it is simply
        # cheaper than what a fast rebuild costs the heading return, because
        # every rebuild re-grants the heading error the previous corridor was
        # asking the car to give up.
        #
        # THE PARAM IS A PERIOD IN SECONDS, NOT A FREQUENCY -- bigger is
        # slower. THE VALUE IS NOT WRITTEN HERE: the declare_parameter default
        # below reads stack_params.yaml through get_value(), so that file is
        # the single source of truth even for a bare `ros2 run` that bypasses
        # the launch file. It used to be a hand-mirrored literal (0.1) that
        # had drifted out of step with the yaml (1.0), with this file's own
        # comments (1.0), and with a test's comment (10.0).
        self.corridor_update_period = float(
            self.declare_parameter(
                'corridor_update_period',
                get_value('corridor_update_period')).value)
        self.last_corridor_time = None
        # Real computation time of the cached corridor (rclpy Time, not a
        # float) -- used ONLY to stamp /mpc/corridor_markers headers (see
        # _publish_corridor_markers below) with when the corridor was
        # actually computed, not whatever instant the marker happens to be
        # published at. Kept separate from last_corridor_time (a float
        # second-count used purely for the update_period gate above) since
        # the two need different representations.
        self.last_corridor_stamp = None
        self.cached_corridor: Optional[dict] = None
        self.cached_pref_nom: Optional[np.ndarray] = None

        self.prev_predicted_state = None
        # Bootstrap value only -- captured once from the first odom message this
        # node instance ever sees (control_loop() below), purely so
        # build_straight_corridor() has something non-None if a goal_distance
        # ever arrives before that first capture somehow lands. Every real
        # goal_distance move re-anchors this to the robot's heading AT THAT
        # MOVE'S START (goal_distance_callback below) -- see that callback's own
        # comment for why: reusing a single node-startup-time value here for
        # every move, forever, made every straight move drift back toward
        # whatever direction the car happened to be facing at process startup,
        # silently undoing any turn executed since (found via live testing:
        # straight-after-turn advanced along the pre-turn heading, not the
        # post-turn one -- build_straight_corridor's dpsi relinearization
        # actively steers toward psi_init_corridor, so this wasn't just a wrong
        # label on an already-straight path, it was steering error).
        self.psi_init_corridor = None

        # Old-workspace-name cleanup pass: previously hardcoded Path.home() /
        # 'ros2_f110_ws' / ... -- a stale reference to this project's old
        # workspace name/layout, broken for anyone not on that exact original
        # machine (same class of bug _resolve_debug_output_path() itself was
        # already written to fix for corridor_log_path below, just missed for
        # these two). Now resolved the same portable way, alongside
        # corridor_debug.jsonl in the same corridors_jsons/ directory.
        self.error_log_path = _resolve_debug_output_path('mpc_odom_error.csv')
        self.error_log_path.parent.mkdir(parents=True, exist_ok=True)
        self.error_log_file = open(self.error_log_path, 'w', encoding='utf-8')
        self.error_log_file.write('t,x_real,y_real,yaw_real,v_real,x_pred,y_pred,yaw_pred,v_pred,ex,ey,eyaw,ev\n')
        self.error_log_file.flush()

        self.prev_v_real = None
        self.v_real_log = None
        self.mpc_k = 0
        self.prev_v_cmd = None

        self.control_log_path = _resolve_debug_output_path('mpc_control_compare.csv')
        self.control_log_file = open(self.control_log_path, 'w', encoding='utf-8')
        self.control_log_file.write(
            't,t_mpc,delta_cmd,delta_real,a_cmd,v_cmd,wheel_speed_cmd,v_real,a_imu\n'
        )
        self.control_log_file.flush()

        self.delta_left_real = None
        self.delta_right_real = None
        self.delta_real = None
        self.ax_imu = None

        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # =========================
        # Limiti
        # =========================
        self.limits = {
            "delta_min": -1.05,
            "delta_max": 1.05,
            "a_min": -2.0,
            "a_max": 3.0,
            "dDeltaMin": -0.5,
            "dDeltaMax": 0.5,
            "dAMin": -2.0,
            "dAMax": 2.0,
            "vMin": -1.0,
            "vMax": 3.0,
        }

        # =========================
        # Pesi
        # =========================
        # w_psi / w_corr are now REALLY WIRED UP -- w_psi as a terminal-yaw
        # cost and w_corr as a stagewise squared-lateral-offset cost, in
        # mpc_solver.py's RTI QP as well as in planner_cost_corridor. Before
        # this pass both existed ONLY as commented-out lines in
        # planner_cost_corridor, which the default RTI backend never evaluates,
        # so any value here was inert in both backends -- the `8 * 0` /
        # `0 * 5.0` spellings hid that they were disabled twice over.
        #
        # w_psi = 1.0 is what makes the car recover the corridor's DIRECTION
        # after an avoidance manoeuvre. build_straight_corridor's goal_distance
        # branch already presents the right reference (psiEnd = the heading
        # captured at move start, blended in from the live yaw over the
        # corridor's length); nothing was pulling the solver onto it. Measured
        # closed-loop -- real corridor + compute_local_target + solver against
        # the real vehicle model, obstacle sitting on the line -- as distance
        # travelled past the obstacle before |yaw error| settles under 0.05 rad
        # and stays, versus the clearance actually achieved (metres of gap
        # beyond car_radius; avoidance_margin = 0.12 is the configured target):
        #
        #   w_psi   settle after obs   clearance r=0.15 / r=0.35   max|lat|
        #    0.0      3.10 m (bug)          0.201 / 0.210            1.03
        #    1.0      1.34 m                0.143 / 0.140            0.69
        #    1.5      1.14 m                0.128 / 0.124            0.63
        #    2.0      1.03 m                0.117 / 0.112            0.60
        #    4.0      0.82 m                0.093 / 0.086            0.54
        #
        # Strictly monotone: more w_psi buys faster direction recovery and pays
        # for it in obstacle clearance, because the same pull that straightens
        # the car also resists the deflection while the obstacle is still
        # there. 1.0 is the modest end of the useful range -- it cuts recovery
        # distance 2.3x versus today while keeping ~0.14 m of clearance, ~17%
        # above the configured margin, which leaves room for the model error,
        # solve latency and detection jitter this idealised loop does not have.
        # 1.5 is the largest value that still clears the configured 0.12 m in
        # both obstacle sizes, if a future tuning pass on real runs wants it.
        #
        # w_corr stays 0.0: measured, it degrades every metric at every value
        # tried, alone or alongside w_psi (at w_psi = 1.0, w_corr 0 -> 1.0
        # moves settle 1.34 -> 1.71 m and clearance 0.143 -> 0.137; w_corr
        # alone at 2.0 never settles at all). The reason was structural -- the
        # corridor was rebuilt from the LIVE pose every tick, so its centerline
        # passed through the car by construction and "lateral offset from the
        # centerline" measured departure from this tick's plan rather than from
        # the intended line; penalising it damped the very lateral motion an
        # avoidance-and-recovery manoeuvre is made of. That premise NO LONGER
        # HOLDS on straight moves: build_straight_corridor now anchors them to
        # a line frozen at move start, and the same note already recorded that
        # this term does work on a frozen corridor (w_corr 0 -> 10 cuts the
        # horizon's end lateral offset 0.324 -> 0.246 m). Left at 0.0 all the
        # same -- turning it on is a retune that needs its own measured pass
        # against the new geometry, not a side effect of the anchoring fix, and
        # the frozen line already restores laterally through w_term.
        self.weights = {
            "w_term": 3.0,
            "w_v": 8.0,
            "w_psi": 1.5,
            "w_u_a": 0.0,
            "w_du_delta": 15.0,
            "w_du_a": 0.0,
            "w_delta0": 0.0,
            # DECLARED, not effective. Every key here except w_term and w_psi
            # is a per-stage weight and is multiplied by
            # STAGE_WEIGHT_REF_HORIZON / N = 7/20 = 0.35 inside
            # solve_mpc_step (see mpc_solver.scale_stage_weights). So the
            # obstacle weight the solver actually sees is 8.0 * 0.35 = 2.8,
            # w_v is 2.8, w_du_delta is 5.25 -- while w_term 3.0 and w_psi 1.5
            # are terminal and pass through untouched. Quote the effective
            # number when comparing against a tuning note, not the literal.
            "w_obs": 8.0,
            "w_corr": 0.0,
        }

        # Disabled/warning-only clearance-log threshold (see
        # compute_predicted_clearance's comparison below) -- not a declared ROS
        # param, not a hard constraint. Previously a hardcoded, unrelated 0.9;
        # now derived from the same shared car_radius/avoidance_margin the two
        # active avoidance mechanisms use, so a log warning at least means the
        # same thing those mechanisms' own trigger radius does (safety-margin
        # unification pass).
        self.dmin = self.car_radius + self.avoidance_margin
        self.vdes = 0.5

        # =========================
        # Target smoothing / obstacle-deflection coast
        # =========================
        # Exponential smoothing on compute_local_target's (possibly obstacle-
        # deflected) output point -- raw per-frame obstacle detections can
        # otherwise make the target jump frame to frame even with
        # f1tenth_perception's own merge/plausibility filtering upstream.
        # new = alpha*raw + (1-alpha)*old; alpha=1.0 disables smoothing
        # (always the raw point); smaller alpha = smoother but slower to react.
        self.target_smoothing_alpha = float(
            self.declare_parameter('target_smoothing_alpha', 0.5).value)
        self.smoothed_target: Optional[np.ndarray] = None

        # When the obstacle(s) deflecting the target drop out of the live
        # per-frame list (one missed detection, one merge-away, object
        # actually cleared -- compute_local_target has no per-frame memory of
        # its own), don't snap the target straight back to the raw centerline
        # on the very next tick: decay the last-applied deflection linearly to
        # zero over this many ticks instead. 1 tick = one compute_local_target
        # call (~one control_loop tick, self.ts seconds apart).
        self.deflection_decay_ticks = int(
            self.declare_parameter('deflection_decay_ticks', 5).value)
        self.last_deflection_vec = np.zeros(2)
        self.deflection_decay_remaining = 0

        # =========================
        # Salvataggio debug corridoio
        # =========================
        self.save_corridor_debug = True
        # See _resolve_debug_output_path()'s docstring (top of this file) for
        # why this isn't a plain parents[2] anchor off __file__ -- that broke
        # under this workspace's actual (non --symlink-install) build, the
        # same class of bug fixed for vesc.yaml in
        # sensor_covariance_calibration_node.py's resolve_source_vesc_yaml_path().
        # This replaced a hardcoded Path.home()/'ros2_f110_ws'/... (a stale
        # reference to this project's old workspace name, broken for anyone
        # not on that exact original machine/setup).
        self.corridor_log_path = _resolve_debug_output_path('corridor_debug.jsonl')
        self.get_logger().info(f'corridor_log_path = "{self.corridor_log_path}"')

        if self.save_corridor_debug:
            self.corridor_log_path.parent.mkdir(parents=True, exist_ok=True)
            self.corridor_log_file = open(self.corridor_log_path, 'w', encoding='utf-8')

        # =========================
        # TF (map <-> odom ONLY)
        # =========================
        # Deliberately narrow scope: this buffer is used for the frozen
        # straight-move anchor's map <-> odom reprojection and NOTHING else.
        # self.x/self.y/self.yaw and the x0 handed to solve_mpc_step stay
        # exactly what _update_active_odom() puts there -- odom-frame, local
        # EKF -- because the vehicle model, the corridor geometry and the
        # markers (frame_id 'odom', see _publish_corridor_markers) are all
        # consistent in that frame and must remain so. Same Buffer +
        # TransformListener + lookup_transform(..., rclpy.time.Time())
        # ("latest available") shape as obstacle_projector_node.py,
        # detection_3d_node.py and semantic_layer_node.py already use.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =========================
        # Subscribers
        # =========================
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Follows localization_source (get_odom_topic(), see f1tenth_params'
        # param_defaults.py): '/odometry/filtered' (EKF-fused, gyro yaw rate included)
        # when localization_source is 'ekf', '/odom' (raw wheel/steering dead
        # reckoning) when 'raw_odom' -- previously hardcoded '/odom' regardless, so
        # switching localization_source to 'ekf' silently left this, the actual
        # driving controller when enable_nav2:=false, still reading the unfused topic.
        self.sub_odom_hw = self.create_subscription(
            Odometry,
            get_odom_topic(),
            self.hw_odom_callback,
            odom_qos
        )

        self.sub_odom_sim = self.create_subscription(
            Odometry,
            '/model/virtual_robot/odometry',
            self.sim_odom_callback,
            odom_qos
        )

        self.sub_goal_distance = self.create_subscription(
            Float32,
            '/mpc/goal_distance',
            self.goal_distance_callback,
            10
        )

        self.sub_goal_pose = self.create_subscription(
            PoseStamped,
            '/mpc/goal_pose',
            self.goal_pose_callback,
            10
        )

        # f1tenth_behavior's PublishMoveGoal, for a mission "turn" step (schema_
        # version 2.0). See goal_turn_callback's own docstring for how a signed
        # heading_delta_deg + speed + steering turn into an actual drive command.
        self.sub_goal_turn = self.create_subscription(
            TurnGoal,
            '/mpc/goal_turn',
            self.goal_turn_callback,
            10
        )

        self.sub_hold = self.create_subscription(
            Bool,
            '/mpc/hold',
            self.hold_callback,
            10
        )

        self.sub_obstacles_2d = self.create_subscription(
            Obstacle2DArray,
            '/perception/obstacles_2d',
            self.obstacles_2d_callback,
            10
        )

        self.sub_costmap_boundaries = self.create_subscription(
            BoundaryConstraintArray,
            '/costmap/boundaries',
            self.costmap_boundaries_callback,
            10
        )

        self.front_distance = 10.0
        self.sub_front_distance = self.create_subscription(
            Float32,
            '/perception/front_distance',
            self.front_distance_callback,
            10
        )

        self.sub_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # =========================
        # Publishers
        # =========================
        # Real-hardware drive command: ackermann_mux's "navigation" lane (priority 10,
        # topic "drive" -- f1tenth_bringup/config/mux.yaml) -> ackermann_to_vesc_node ->
        # vesc_driver_node. Same topic/QoS/message-construction pattern as
        # andre_mpc_node.py's self.pub, so the two are interchangeable backends for the
        # same lane. Replaces the old rear_pub/steer_pub pair, which targeted
        # f1tenth_sim's Gazebo ros2_control bridge (/rear_wheels_controller/commands,
        # /steering_controller/commands) -- nothing on the real stack subscribes to
        # those, which is why MPC mode previously ran with no actuation.
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.min_obstacle_distance_pub = self.create_publisher(
            Float32,
            '/mpc/min_obstacle_distance',
            10
        )

        # UPGRADE: real clearance verification over the *predicted* trajectory,
        # separate from min_obstacle_distance_pub (which only reflects "now").
        self.predicted_clearance_pub = self.create_publisher(
            Float32,
            '/mpc/predicted_min_clearance',
            10
        )

        # Per-tick solver outcome. These exact values were previously written
        # ONLY to the ROS logger (the SOLVE/out line below), which made "did
        # the MPC converge?" unanswerable from a bag -- the 2026-09-01 mission
        # analysis had to recover 1285 solves by parsing ~/.ros/log/*.log, and
        # only succeeded because rotation had not yet discarded them. Depth 10,
        # default reliable QoS: this is a diagnostic feed nothing controls off,
        # but it must not silently drop the one tick that explains a stop.
        self.solver_status_pub = self.create_publisher(
            MpcSolverStatus, '/mpc/solver_status', 10)

        self.goal_reached_pub = self.create_publisher(
            Bool,
            '/mpc/goal_reached',
            10
        )

        # Corridor visualization (Foxglove/RViz) -- the MPC's own soft
        # reference corridor (build_straight_corridor()'s xL/yL/xR/yR wall
        # polylines) had no ROS-visible representation at all before this;
        # only a debug JSONL log file (see save_corridor_snapshot()). NOT the
        # same thing as /costmap/boundaries (costmap_boundary_node's hard,
        # occupancy-grid-derived constraints, base_link frame, a non-
        # renderable custom BoundaryConstraintArray) -- see
        # _publish_corridor_markers()'s own docstring. visualization_msgs/
        # MarkerArray, not geometry_msgs/PolygonStamped or nav_msgs/Path --
        # matches this stack's own existing precedent (semantic_layer_node.py,
        # detection_3d_node.py both already publish MarkerArray for
        # Foxglove/RViz; neither PolygonStamped nor Path has any publisher
        # anywhere in this codebase), and a MarkerArray of two independent
        # LINE_STRIPs is a more natural fit for "two separate wall polylines"
        # than a single closed Polygon would be anyway. Published at whatever
        # rate the corridor is ACTUALLY recomputed at (see corridor_update_
        # period above and the need_update block in control_loop) -- not
        # throttled to a separate rate for Foxglove's sake; nothing so far
        # suggests that's needed (three thin LINE_STRIPs, corr_N=120 points
        # each, is a light payload compared to e.g. costmap_renderer_node's
        # own PNG stream). If it ever is needed, follow this stack's own
        # existing <topic>/viz convention (foxglove_bridge.launch.py's
        # /camera/image_raw -> /camera/image_raw/viz throttle nodes), not a
        # new topic-naming shape.
        self.corridor_markers_pub = self.create_publisher(
            MarkerArray,
            '/mpc/corridor_markers',
            10
        )

        # Jetson process tuning (priority); safe no-op if unset or denied by
        # the OS. CPU affinity is handled externally now -- see mpc_corr.
        # launch.py's own cpu_affinity comment.
        self._apply_nice()

        self.timer = self.create_timer(self.ts, self.control_loop)

        self.get_logger().info(
            'MPC Controller STARTED (pure MPC + obstacle avoidance '
            '[fixed deflection + predicted clearance], no plan executor)'
        )

        # ---- DEBUG: dump della configurazione all'avvio ----
        self.get_logger().info(
            f'CFG | ts={self.ts} N={self.N} vdes={self.vdes} dmin={self.dmin} '
            f'wheel_radius={self.wheel_radius} car_radius={self.car_radius} '
            f'avoidance_margin={self.avoidance_margin}'
        )
        self.get_logger().info(f'CFG | limits={self.limits}')
        self.get_logger().info(f'CFG | weights={self.weights}')
        self.get_logger().info(f'CFG | params={self.params}')

    def destroy_node(self):
        if hasattr(self, 'corridor_log_file'):
            try:
                self.corridor_log_file.close()
            except Exception:
                pass
        if hasattr(self, 'error_log_file'):
            try:
                self.error_log_file.close()
            except Exception:
                pass
        super().destroy_node()

    # ── Jetson process tuning ──────────────────────────────────────────────
    def _apply_nice(self):
        """Raise the process's scheduling priority (best effort).

        Ported from the deleted andre_mpc_opt_node.py (git c36e19f), which
        also set CPU affinity here -- see the "CPU AFFINITY REMOVED" note
        below for why that half moved out of this method (renamed from
        _apply_cpu_affinity_and_priority to match what it actually does now).

        CPU AFFINITY REMOVED FROM HERE (thread-pinning-leak fix, Step 6
        reintroduction investigation): this used to also declare a
        cpu_affinity param and call os.sched_setaffinity(0, cores) on it,
        applied once, in-process, from __init__ -- the same mechanism
        confirmed live to leak the vast majority of a process's threads in
        behavior_executor_node and all three f1tenth_perception detection
        nodes (e.g. yolo_detector_node: 32 of 36 threads fully unpinned,
        with threads from multiple nodes actually caught executing on
        reserved cores under real load, not just theoretically able to).
        mpc_corr was never under measured load in that investigation
        (behavior/control wasn't reintroduced there), but it shares the
        identical mechanism, so there's no reason to expect it behaved any
        differently. Affinity is now an external `taskset -c <cores>`
        launch prefix instead (see mpc_corr.launch.py's own matching
        comment) -- it sets the mask before this process's first
        instruction runs, so every thread this node or any library it uses
        ever spawns inherits it, with no in-process code needed at all.
        """
        nice_val = int(self.get_parameter('nice').value)
        if nice_val != 0:
            # Negative niceness lowers scheduling latency for the control loop
            # but needs CAP_SYS_NICE (root). Failure is non-fatal -- this user
            # does not have passwordless sudo on the Jetson this was ported to,
            # so expect (and don't treat as an error) a PermissionError here
            # unless the node is later run with elevated privileges.
            try:
                os.nice(nice_val)
                self.get_logger().info(f'Process nice set to {nice_val:+d}')
            except Exception as exc:
                self.get_logger().warn(
                    f'Could not set nice {nice_val:+d} (need CAP_SYS_NICE/root): {exc}'
                )

    # ==========================================
    # CALLBACKS
    # ==========================================
    def hw_odom_callback(self, msg: Odometry):
        self.hw_x = msg.pose.pose.position.x
        self.hw_y = msg.pose.pose.position.y
        self.hw_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.hw_v = msg.twist.twist.linear.x
        self.hw_odom_last_time = self.get_clock().now().nanoseconds * 1e-9

        # ---- DEBUG: conferma che /odom arriva davvero e cosa contiene ----
        self.get_logger().info(
            f'ODOM/hw | x={self.hw_x:+.4f} y={self.hw_y:+.4f} '
            f'yaw={self.hw_yaw:+.4f} v={self.hw_v:+.4f}',
            throttle_duration_sec=1.0
        )

    def sim_odom_callback(self, msg: Odometry):
        self.sim_x = msg.pose.pose.position.x
        self.sim_y = msg.pose.pose.position.y
        self.sim_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.sim_v = msg.twist.twist.linear.x
        self.sim_odom_last_time = self.get_clock().now().nanoseconds * 1e-9

        # ---- DEBUG ----
        self.get_logger().info(
            f'ODOM/sim | x={self.sim_x:+.4f} y={self.sim_y:+.4f} '
            f'yaw={self.sim_yaw:+.4f} v={self.sim_v:+.4f}',
            throttle_duration_sec=1.0
        )

    def hold_callback(self, msg: Bool):
        if msg.data != self.hold:
            self.get_logger().info(f'HOLD | {"engaged" if msg.data else "released"}')
        self.hold = msg.data

    def goal_distance_callback(self, msg: Float32):
        if self.x is None or self.y is None:
            self.get_logger().warn(
                'goal_distance ricevuto ma stato ancora None: comando ignorato.'
            )
            return

        self.goal_start_xy = (self.x, self.y)
        self.goal_distance = float(msg.data)
        # Re-anchor "straight" to THIS move's own heading, not whatever
        # psi_init_corridor last held (possibly node-startup, possibly several
        # moves and turns ago) -- see psi_init_corridor's own comment for the
        # bug this fixes. self.yaw is guaranteed non-None here: it's set
        # atomically alongside self.x/self.y in control_loop()'s odom-source
        # selection, and self.x is already known non-None from the guard above.
        self.psi_init_corridor = self.yaw
        self.goal_reached = False
        self._no_goal_warned = False

        # MAP-frame equivalent of the same anchor, captured in the same breath
        # -- this is the one that actually survives the car's real heading
        # drift (see _pose_odom_to_map's own docstring). goal_anchor_odom is
        # seeded with the raw odom anchor either way: with a map anchor it is
        # simply this instant's reprojection (an identity right now, diverging
        # as map -> odom is corrected); without one it stays that raw value for
        # the whole move, which is exactly the pre-existing behaviour. A failed
        # lookup is logged and driven through, never fatal and never blocking --
        # same fail-safe convention as obstacle_projector_node.py's own
        # lookup_transform guard, and as costmap_boundary_node's "degrade, keep
        # publishing, log at WARN" pass.
        self.goal_anchor_odom = (self.x, self.y, float(self.yaw))
        self.goal_anchor_map = None
        tf_map_odom = self._lookup_map_odom()
        if tf_map_odom is not None:
            self.goal_anchor_map = _pose_odom_to_map(
                self.x, self.y, float(self.yaw), *tf_map_odom)
            self.get_logger().info(
                f'ANCHOR/capture | odom=({self.x:+.3f},{self.y:+.3f},'
                f'{float(self.yaw):+.4f}) -> map=({self.goal_anchor_map[0]:+.3f},'
                f'{self.goal_anchor_map[1]:+.3f},{self.goal_anchor_map[2]:+.4f}) '
                f'tf_yaw={tf_map_odom[2]:+.4f}')
        elif self.use_map_frame_goal_anchor:
            self.get_logger().warn(
                f'ANCHOR/capture | no "{self.map_frame}" -> "{self.odom_frame}" '
                'transform at move start: this move keeps the odom-frame-only '
                'frozen anchor (heading drift absorbed by map -> odom will NOT '
                'be corrected for it).')

        # Switching to distance mode -- clear any pose-mode goal so the two
        # modes stay mutually exclusive (see goal_pose_callback).
        self.goal_pose_xy = None
        self.goal_pose_yaw = None
        self.pose_goal_reached = False
        self._invalidate_move_state()
        self.get_logger().info(
            f'Nuovo goal_distance={self.goal_distance:.3f} m da '
            f'({self.goal_start_xy[0]:.3f}, {self.goal_start_xy[1]:.3f})'
        )

    def _lookup_map_odom(
            self, max_age_sec: Optional[float] = None
    ) -> Optional[Tuple[float, float, float]]:
        """Latest map -> odom edge as (x, y, yaw), or None if it isn't
        available, is older than `max_age_sec`, or the feature is off (see
        use_map_frame_goal_anchor).

        `lookup_transform(map_frame, odom_frame, ...)` -- target 'map', source
        'odom' -- returns the edge in exactly the orientation _pose_odom_to_map/
        _pose_map_to_odom both expect; see the former's own docstring. Time() is
        "latest available", the same choice obstacle_projector_node.py and
        detection_3d_node.py make for their own fixed/slow-moving lookups: this
        anchor is a per-move reference line, not a per-message measurement that
        needs stamp-exact alignment, and the whole point of the reprojection is
        to track the LATEST correction rather than the one that was current when
        the move started.

        Failure is a normal, expected state early in a run (map -> odom does not
        exist until the global EKF has fused something, and a mission's first
        goal_distance can easily precede that), so it is a throttled WARN and a
        None return -- never an exception out of a callback or the control loop.
        """
        if not self.use_map_frame_goal_anchor:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'tf2 lookup "{self.map_frame}" -> "{self.odom_frame}" failed: '
                f'{exc}', throttle_duration_sec=5.0)
            return None

        # ---- AGE GUARD ----------------------------------------------------
        # WHY THIS IS NEEDED AT ALL, given the lookup above cannot fail during
        # a stall: rclpy.time.Time() means "latest available", and tf2 keeps a
        # 10s buffer, so a map -> odom delivery gap does NOT raise here -- the
        # buffer just keeps handing back the last transform it got, silently,
        # for as long as the gap lasts. That is the failure this guard exists
        # for, and it is not hypothetical: across every mpc_corr log in this
        # workspace the except branch above fired exactly ZERO times, so
        # nothing in this node has ever noticed a stale edge. Reprojecting the
        # anchor through one is strictly worse than not reprojecting: it
        # rotates the reference line by a correction that no longer describes
        # where the car is.
        #
        # SIGN CONVENTION -- read before changing the threshold. ekf_global.yaml
        # sets transform_time_offset: 0.05, which POST-dates every broadcast
        # map -> odom by 50ms. A perfectly fresh transform therefore has an age
        # of about -50ms here, not 0, and ages measured off the recorded stream
        # run to a median of -25ms. The comparison is deliberately one-sided
        # (age > max_age) so that post-dating can never trip the guard; only
        # genuinely OLD transforms do.
        if max_age_sec is not None:
            try:
                age = (self.get_clock().now()
                       - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            except (TypeError, ValueError) as exc:
                # Mismatched clock types (sim vs system) -- do not let a
                # diagnostic guard take out the control loop. Degrade to
                # today's unguarded behaviour and say so.
                self.get_logger().warn(
                    f'map -> odom age unavailable ({exc}): proceeding UNGUARDED',
                    throttle_duration_sec=5.0)
                age = None
            if age is not None and age > max_age_sec:
                self.get_logger().warn(
                    f'map -> odom STALE: age={age * 1e3:.0f}ms > '
                    f'{max_age_sec * 1e3:.0f}ms -- not reprojecting through it',
                    throttle_duration_sec=5.0)
                return None

        return (float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                float(self.quaternion_to_yaw(tf.transform.rotation)))

    def _refresh_goal_anchor(self):
        """Recompute self.goal_anchor_odom -- the frozen straight-move anchor
        expressed in THIS cycle's odom frame -- once per control_loop tick.

        Called from control_loop BEFORE both consumers (the goal_distance
        progress check and build_straight_corridor's own goal_distance branch)
        so that one lookup per tick serves both instead of each doing its own at
        its own instant -- the corridor is rebuilt on its own slower cadence
        (corridor_update_period, 1.0s by default) than the progress check runs
        (every tick), so two independent lookups would disagree by however much
        map -> odom moved in between. The two consumers take different parts of
        the result (see the attribute's own comment in __init__), but the
        CORRECTION they apply is the same one, which is the point.

        FLICKER NOTE: this runs at the control-loop rate, so the correction it
        tracks is continuous at 10 Hz; the corridor only samples it when it
        rebuilds, so a discrete SLAM correction landing between rebuilds still
        reaches the corridor as one step of up to corridor_update_period's worth
        of accumulated change. Since the corridor's origin stopped depending on
        this value that step is now a pure rotation about the car rather than a
        translation plus a rotation -- see build_straight_corridor's own comment.

        No map anchor (never captured, or the feature is off) -> nothing to do:
        goal_anchor_odom keeps the raw odom-frame value goal_distance_callback
        seeded it with, i.e. the pre-existing frozen-in-odom behaviour.

        Lookup failure OR a transform older than map_odom_max_age_sec (the
        case that actually happens -- see _lookup_map_odom's AGE GUARD: the
        lookup itself does not fail during a delivery stall, it silently
        returns the last transform tf2 holds) with a map anchor present ->
        HOLD THE LAST KNOWN GOOD
        reprojection, explicitly, rather than falling back to the raw odom
        anchor: that raw value is precisely the thing being corrected for, so
        reverting to it for one tick would silently reintroduce the bug (and,
        worse, jump the reference line mid-move). Holding the last good value
        instead is the same shape as costmap_boundary_node's own periodic-
        publish pass, which likewise keeps recomputing from its cached grid/pose
        of whatever age and only fails safe when the input has NEVER arrived --
        and that "never" case cannot occur here, since capture seeds a value.
        """
        if self.goal_anchor_map is None:
            return
        tf_map_odom = self._lookup_map_odom(max_age_sec=self.map_odom_max_age_sec)
        if tf_map_odom is None:
            self.get_logger().warn(
                'ANCHOR/hold | map -> odom unavailable OR STALE this tick: '
                'holding last reprojected anchor (NOT reverting to the raw '
                'odom anchor)', throttle_duration_sec=5.0)
            return
        self.goal_anchor_odom = _pose_map_to_odom(*self.goal_anchor_map, *tf_map_odom)

    def goal_pose_callback(self, msg: PoseStamped):
        if self.x is None or self.y is None:
            self.get_logger().warn(
                'goal_pose ricevuto ma stato ancora None: comando ignorato.'
            )
            return

        self.goal_pose_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_pose_yaw = self.quaternion_to_yaw(msg.pose.orientation)
        self.pose_goal_reached = False
        # Switching to pose mode -- clear any distance-mode goal so the two
        # modes stay mutually exclusive: whichever topic was published to most
        # recently wins (see goal_distance_callback).
        self.goal_distance = None
        self.goal_start_xy = None
        self.goal_anchor_map = None
        self.goal_anchor_odom = None
        self.goal_reached = False
        self._no_goal_warned = False
        self._invalidate_move_state()
        self.get_logger().info(
            f'Nuovo goal_pose=({self.goal_pose_xy[0]:.3f}, {self.goal_pose_xy[1]:.3f}) '
            f'yaw={self.goal_pose_yaw:+.3f} (yaw non ancora utilizzato, solo posizione)'
        )

    def _resolve_turn_reach(self, heading_delta_rad: float, steering: str) -> float:
        """How far ahead (meters) goal_turn_callback should place its synthetic
        goal_pose target, given the requested turn's heading change and
        steering aggressiveness.

        NOT a literal steering-angle command -- this vehicle only has a
        heading-change primitive at all because build_straight_corridor()
        already curves its corridor toward goal_pose_xy's bearing from the
        CURRENT position every rebuild (see that method: psiEnd =
        atan2(gy-Y0, gx-X0) when goal_pose_xy is set, vs. the fixed
        psi_init_corridor otherwise) -- confirmed by reading that method
        directly before relying on it, not assumed. goal_turn reuses that
        existing, already-working mechanism by computing a target point that
        SITS along the desired final heading, rather than adding a second,
        parallel drive mode with its own corridor/solver path.

        "steering" only shapes WHERE that target point is (via the standard
        Ackermann single-track turn-radius relationship, R = wheelbase /
        tan(steering_angle)): full_lock (corr_epsiMax, this vehicle's own
        assumed max steering angle) produces a short reach and therefore a
        tight curve; a shallow partial:<deg> produces a long reach and a
        gentle one. The solver still computes its own actual steering output
        every tick (bounded by self.limits) -- this heuristic never bypasses
        it, it only points the corridor.
        """
        max_steering_rad = self.corr_epsiMax
        if steering == 'full_lock':
            steering_rad = max_steering_rad
        else:
            # 'partial:<deg>' -- format already validated at mission load time
            # (mission_config.py's _is_valid_steering), so the split/float()
            # below is guaranteed to succeed for anything that reached here
            # via the mission pipeline. Still clamped defensively (a bare
            # `ros2 topic pub` onto /mpc/goal_turn bypasses that validation
            # entirely) rather than trusted blindly.
            try:
                requested_deg = abs(float(steering.split(':', 1)[1]))
            except (IndexError, ValueError):
                self.get_logger().warn(
                    f'goal_turn: steering={steering!r} non parseable, uso full_lock '
                    'come fallback.'
                )
                requested_deg = math.degrees(max_steering_rad)
            steering_rad = min(math.radians(requested_deg), max_steering_rad)
        # Avoid a near-zero (or exactly zero) steering angle producing a
        # near-infinite (or divide-by-zero) turn radius.
        steering_rad = max(steering_rad, math.radians(1.0))

        wheelbase = self.params['L']
        radius = wheelbase / math.tan(steering_rad)
        chord = 2.0 * radius * math.sin(abs(heading_delta_rad) / 2.0)
        return float(np.clip(chord, 0.3, self.corr_L_base))

    def goal_turn_callback(self, msg: TurnGoal):
        """f1tenth_behavior's PublishMoveGoal, once per mission "turn" step
        entry (schema_version 2.0). Resolves the signed heading_delta_deg
        into a synthetic goal_pose target and dispatches to the EXACT SAME
        corridor-following/arrival machinery goal_pose_callback already uses
        (self.goal_pose_xy/self.goal_pose_yaw/self.pose_goal_reached) --
        see _resolve_turn_reach's own docstring for why that's sufficient
        (build_straight_corridor already curves toward goal_pose_xy's live
        bearing every rebuild) rather than adding a fourth, parallel drive
        mode with its own path through control_loop.

        mpc_corr's own pose_goal_tolerance-based arrival here is NOT the
        authoritative "is the turn done" signal for the mission -- that's
        f1tenth_behavior's own orientation_delta stop_condition, tracked
        independently against live /odom yaw (see condition_eval.py). This
        node's arrival check only decides when IT stops actively driving
        toward the synthetic target; the mission may (and typically will)
        advance to the next move, which republishes a new goal here and
        supersedes this one, before or after this node's own tolerance is
        ever reached -- same relationship goal_pose already has with the
        mission layer today, not something new introduced for turn.
        """
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().warn(
                'goal_turn ricevuto ma stato ancora None: comando ignorato.'
            )
            return

        heading_delta_rad = math.radians(float(msg.heading_delta_deg))
        target_yaw = self.yaw + heading_delta_rad
        reach = self._resolve_turn_reach(heading_delta_rad, str(msg.steering))

        self.goal_pose_xy = (
            self.x + reach * math.cos(target_yaw),
            self.y + reach * math.sin(target_yaw),
        )
        self.goal_pose_yaw = target_yaw
        self.pose_goal_reached = False
        # Switching to turn (pose-mode-backed) -- clear any distance-mode
        # goal so the two stay mutually exclusive, same pattern goal_pose_
        # callback/goal_distance_callback already use for each other.
        self.goal_distance = None
        self.goal_start_xy = None
        self.goal_anchor_map = None
        self.goal_anchor_odom = None
        self.goal_reached = False
        self._no_goal_warned = False
        self._invalidate_move_state()

        # vdes override for the duration of the turn -- the one real
        # (non-stub) per-move vdes path in this file today. move.vdes at the
        # general mission level is still unimplemented elsewhere (see
        # f1tenth_behavior's check_stop_condition.py, which logs it as a
        # TODO for every OTHER move type) -- turn's own `speed` field is a
        # distinct, already-required field on the turn spec, not a reuse of
        # that stub, so wiring it here doesn't fix or touch that TODO.
        if msg.speed > 0.0:
            self.vdes = float(msg.speed)

        self.get_logger().info(
            f'Nuovo goal_turn: heading_delta={math.degrees(heading_delta_rad):+.1f} deg '
            f'target_yaw={target_yaw:+.3f} rad synthetic_target=('
            f'{self.goal_pose_xy[0]:.3f},{self.goal_pose_xy[1]:.3f}) reach={reach:.2f} m '
            f'steering={msg.steering!r} speed={msg.speed:.2f}'
        )

    def _invalidate_move_state(self):
        """Drop every piece of per-move cached geometry.

        Called by all three goal callbacks (goal_distance, goal_pose,
        goal_turn) the instant a new move is accepted.

        WHY THE CORRIDOR CACHE IS IN HERE, and what it cost when it was not:
        control_loop rebuilds the corridor only every corridor_update_period,
        so without this a brand-new move solved against the PREVIOUS move's
        corridor -- its length, its frozen heading, its endpoint -- for up to a
        full period. It reported "solved" the whole time, and because
        _publish_corridor_markers only fires on a rebuild, no marker was
        published to reveal that the geometry on screen belonged to the move
        before. A turn command following a straight one was the worst case: it
        steered against the straight move's frozen heading until the period
        expired. Clearing last_corridor_time forces need_update on the very
        next tick, which rebuilds AND publishes.

        WHAT IS DELIBERATELY NOT RESET: self.last_u. The RTI warm start is
        last_u tiled across the horizon (see mpc_solver._solve_rti), but last_u
        is also the input actually being held by the hardware right now, and it
        is what w_du_delta measures the next command against. Zeroing it on a
        goal boundary would command a steering snap to centre and charge the
        rate cost for a discontinuity the car never made. The linearization
        trajectory it seeds is re-rolled from the live x0 every tick anyway, so
        it carries no stale corridor information.
        """
        # Target smoothing / obstacle-deflection coast: previous move's state.
        self.smoothed_target = None
        self.last_deflection_vec = np.zeros(2)
        self.deflection_decay_remaining = 0
        # Corridor geometry: force a rebuild + marker publish on the next tick.
        self.cached_corridor = None
        self.last_corridor_time = None
        self.last_corridor_stamp = None
        self.cached_pref_nom = None

    def _update_active_odom(self):
        """Seleziona la sorgente odom attiva (hardware ha sempre priorita' se fresca)."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        hw_age = (now_sec - self.hw_odom_last_time) if self.hw_odom_last_time is not None else math.inf
        sim_age = (now_sec - self.sim_odom_last_time) if self.sim_odom_last_time is not None else math.inf

        if hw_age < self.odom_stale_timeout_sec:
            source = 'hardware'
            self.x, self.y, self.yaw, self.v = self.hw_x, self.hw_y, self.hw_yaw, self.hw_v
        elif sim_age < self.odom_stale_timeout_sec:
            source = 'sim'
            self.x, self.y, self.yaw, self.v = self.sim_x, self.sim_y, self.sim_yaw, self.sim_v
        else:
            source = None
            self.x = self.y = self.yaw = self.v = None

        # ---- DEBUG: eta' delle due sorgenti, per capire chi e' stale ----
        self.get_logger().info(
            f'ODOMSEL | src={source} hw_age={hw_age:.3f}s sim_age={sim_age:.3f}s '
            f'timeout={self.odom_stale_timeout_sec:.3f}s',
            throttle_duration_sec=2.0
        )

        if source != self.active_odom_source:
            self.get_logger().info(f'Odom source -> {source if source is not None else "NESSUNA (stale)"}')
            self.active_odom_source = source

        # Bootstrap-only capture (see self.psi_init_corridor's own comment) --
        # goal_distance_callback overwrites this for every real move; this just
        # covers the case nothing has done that yet.
        if source is not None and self.psi_init_corridor is None:
            self.psi_init_corridor = self.yaw
            self.get_logger().info(f'psi_init_corridor (bootstrap) = {self.psi_init_corridor:.3f}')

    def obstacles_2d_callback(self, msg: Obstacle2DArray):
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().warn(
                'obstacles_2d ricevuto ma stato ancora None: frame scartato.',
                throttle_duration_sec=5.0
            )
            return

        obstacles_global = []
        for obs in msg.obstacles:
            x_g, y_g = self.robot_to_global(obs.x, obs.y)
            obstacles_global.append((x_g, y_g, float(obs.r)))

        self.obstacles_global_live = obstacles_global

        # ---- DEBUG: primo ostacolo in frame robot vs frame globale ----
        if msg.obstacles:
            o0 = msg.obstacles[0]
            g0 = obstacles_global[0]
            self.get_logger().info(
                f'OBS/tf | n={len(msg.obstacles)} '
                f'robot=({o0.x:+.3f},{o0.y:+.3f},r={o0.r:.3f}) -> '
                f'world=({g0[0]:+.3f},{g0[1]:+.3f},r={g0[2]:.3f})',
                throttle_duration_sec=2.0
            )

    def costmap_boundaries_callback(self, msg: BoundaryConstraintArray):
        """base_link -> world transform (_boundary_to_world) for
        costmap_boundary_node's own single /costmap/boundaries source (0-3
        entries: front/left/right, each independently present or absent --
        see that node's own module docstring). See module-level
        _boundary_to_world's own docstring for the transform derivation,
        and the __init__ comment above self.costmap_boundaries_world for
        why this transforms immediately here rather than at solve time."""
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().warn(
                'BOUNDARY/costmap ricevuto ma stato ancora None: frame scartato.',
                throttle_duration_sec=5.0
            )
            return

        world = [
            _boundary_to_world(c.normal[0], c.normal[1], c.offset, self.x, self.y, self.yaw)
            for c in msg.constraints
        ]
        self.costmap_boundaries_world = world
        self.costmap_boundaries_last_time = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            f'BOUNDARY/costmap | n={len(world)}',
            throttle_duration_sec=2.0
        )

    def _get_live_boundaries(self) -> List[Tuple[float, float, float]]:
        """Freshness-gated list of (normal_x, normal_y, offset) hard
        boundary constraints, WORLD-frame (see costmap_boundaries_callback
        above), ready to pass straight into solve_mpc_step(boundaries=...).
        Staleness check itself is _select_live_boundaries (pure function,
        module level, directly unit-tested -- see that function's own
        docstring) -- mpc_solver.py's own pad_boundary_constraints is what
        turns "fewer than 3 live sources" into a structurally-fixed-size
        QP, not anything here. This is also exactly the state costmap_
        boundary_node's own staleness gating (map/pose too old or never
        received) forces continuously as of this pass -- see that node's
        own module docstring -- so an empty return here is real, live-
        relevant, expected behavior today, not a hypothetical edge case.

        Gated on self.use_hard_boundary_constraints (default False) -- see
        that attribute's own comment; false forces [] unconditionally,
        before even looking at staleness, so disabling the feature via
        launch arg can't be defeated by fresh data arriving."""
        if not self.use_hard_boundary_constraints:
            return []
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        return _select_live_boundaries(
            self.costmap_boundaries_world, self.costmap_boundaries_last_time,
            now_sec, self.odom_stale_timeout_sec)

    def joint_states_callback(self, msg: JointState):
        try:
            left_name = 'car_1_left_steering_hinge_joint'
            right_name = 'car_1_right_steering_hinge_joint'

            if left_name in msg.name and right_name in msg.name:
                iL = msg.name.index(left_name)
                iR = msg.name.index(right_name)

                omega_L = float(msg.velocity[iL])
                omega_R = float(msg.velocity[iR])

                self.delta_left_real = float(msg.position[iL])
                self.delta_right_real = float(msg.position[iR])
                self.delta_real = 0.5 * (self.delta_left_real + self.delta_right_real)
                self.v_real_log = self.wheel_radius * 0.5 * (omega_L + omega_R)

                # ---- DEBUG ----
                self.get_logger().info(
                    f'JOINT | dL={self.delta_left_real:+.4f} dR={self.delta_right_real:+.4f} '
                    f'delta_real={self.delta_real:+.4f} v_wheels={self.v_real_log:+.4f}',
                    throttle_duration_sec=2.0
                )
            else:
                # ---- DEBUG: i giunti attesi non ci sono, delta_real resta None ----
                self.get_logger().warn(
                    f'JOINT | giunti sterzo non trovati in /joint_states: {list(msg.name)}',
                    throttle_duration_sec=5.0
                )
        except Exception as e:
            self.get_logger().warn(f'joint_states parse failed: {e}')

    def imu_callback(self, msg):
        self.ax_imu = float(msg.linear_acceleration.x)

        # ---- DEBUG: se questa riga non compare mai, /imu non pubblica ----
        self.get_logger().info(
            f'IMU | ax={self.ax_imu:+.4f}',
            throttle_duration_sec=2.0
        )

    def front_distance_callback(self, msg):
        self.front_distance = float(msg.data)

        # ---- DEBUG ----
        self.get_logger().info(
            f'FRONT | d={self.front_distance:.3f}',
            throttle_duration_sec=2.0
        )

    def _publish_drive(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.pub.publish(msg)

        # ---- DEBUG: subs=0 significa che NESSUNO ascolta /drive
        # (ackermann_mux giu' o topic sbagliato) -> nessuna attuazione possibile.
        n_subs = self.pub.get_subscription_count()
        self.get_logger().info(
            f'PUB /drive | speed={speed:+.4f} steer={steering_angle:+.4f} subs={n_subs}',
            throttle_duration_sec=1.0
        )
        if n_subs == 0:
            self.get_logger().warn(
                'PUB /drive | nessun subscriber: il comando non raggiunge il VESC.',
                throttle_duration_sec=5.0
            )

    # ==========================================
    # LOOP CONTROLLO
    # ==========================================
    def control_loop(self):
        loop_t0 = self.get_clock().now().nanoseconds * 1e-9

        self._update_active_odom()

        # Diagnostic, independent of autonomy/goal state -- no consumer yet, but
        # cheap and useful to have live every tick regardless of what else is gating.
        d_robot_obs = self.compute_robot_obstacle_distance(self.obstacles_global_live)
        self.min_obstacle_distance_pub.publish(Float32(data=float(d_robot_obs)))

        if self.x is None or self.y is None or self.yaw is None or self.v is None:
            self.get_logger().warn('ODOM non disponibile: stato ancora None')
            self._publish_drive(0.0, 0.0)
            return

        if self.hold:
            # Deliberately touches nothing else -- goal_start_xy, goal_reached, and
            # self.last_u are all left exactly as they were, so releasing the hold
            # resumes the current move rather than restarting or skipping it.
            self._publish_drive(0.0, 0.0)
            return

        # ONE map -> odom reprojection per tick, shared by the goal_distance
        # progress check below and by build_straight_corridor's own
        # goal_distance branch further down -- see _refresh_goal_anchor's own
        # docstring for why they must not look this up independently. No-op in
        # pose/turn mode and whenever no map anchor was captured.
        self._refresh_goal_anchor()

        if self.goal_pose_xy is not None:
            # Pose mode -- position-only arrival, no final-yaw alignment this
            # pass (see build_straight_corridor's scope note). Mutually
            # exclusive with distance mode: goal_pose_callback/
            # goal_distance_callback each clear the other mode's state, so
            # goal_distance/goal_start_xy are guaranteed None here.
            gx, gy = self.goal_pose_xy

            if self.pose_goal_reached:
                self._publish_drive(0.0, 0.0)
                self.goal_reached_pub.publish(Bool(data=True))
                return

            dist_to_goal = math.hypot(self.x - gx, self.y - gy)

            # ---- DEBUG: avanzamento verso il goal pose ----
            self.get_logger().info(
                f'POSEGOAL | dist_to_goal={dist_to_goal:.4f} '
                f'tolerance={self.pose_goal_tolerance:.3f}'
            )

            if dist_to_goal <= self.pose_goal_tolerance:
                self.pose_goal_reached = True
                self._publish_drive(0.0, 0.0)
                self.goal_reached_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f'Goal pose raggiunto: dist_to_goal={dist_to_goal:.3f} m <= '
                    f'tolerance={self.pose_goal_tolerance:.3f} m'
                )
                return
            # altrimenti: prosegui verso la normale risoluzione MPC qui sotto
            # (build corridoio -> solver -> publish drive), come in modalita' distanza

        elif self.goal_distance is None or self.goal_start_xy is None:
            self._publish_drive(0.0, 0.0)
            if not self._no_goal_warned:
                self.get_logger().warn(
                    'Nessun comando su /mpc/goal_distance ancora ricevuto: robot fermo in attesa.'
                )
                self._no_goal_warned = True
            return

        else:
            if self.goal_reached:
                self._publish_drive(0.0, 0.0)
                self.goal_reached_pub.publish(Bool(data=True))
                return

            # ALONG-LINE progress, not straight-line displacement from
            # goal_start_xy (the old math.hypot form): "go 4m" means 4m made
            # good along psi_init_corridor, the direction the move started in
            # and the one the corridor references. The old form counted a
            # sideways detour as progress toward the goal, so a move that
            # dodged an obstacle terminated early by however much lateral
            # offset it had picked up, and it is still wrong for that reason.
            #
            # THE PROGRESS ANCHOR IS NOT THE CORRIDOR ORIGIN -- these are two
            # separate quantities and must stay that way. This one is the
            # move's ORIGINAL start point, frozen at move start and only ever
            # map-corrected since (goal_anchor_odom, refreshed by
            # _refresh_goal_anchor above), measured along that same corrected
            # heading: "goal_distance: 6.0" means 6 m made good from where the
            # move actually began, along the direction it began in. The
            # corridor's own origin deliberately tracks the CAR now instead
            # (see build_straight_corridor's goal_distance branch) -- if this
            # check were ever pointed at that moving origin, traveled would
            # collapse toward zero and the move would never terminate. The
            # fallback pair is the old raw odom anchor plus the live-yaw
            # default, matching build_straight_corridor's own bootstrap
            # fallback exactly.
            if self.goal_anchor_odom is not None:
                progress_anchor_x, progress_anchor_y, psi_line = self.goal_anchor_odom
            else:
                progress_anchor_x, progress_anchor_y = self.goal_start_xy
                psi_line = (self.psi_init_corridor
                            if self.psi_init_corridor is not None else self.yaw)
            traveled = _project_onto_line(
                (self.x, self.y), (progress_anchor_x, progress_anchor_y), psi_line)

            # ---- DEBUG: avanzamento verso il goal ----
            # displacement logged alongside so the two are directly comparable
            # in a bag/log: they are equal on a clean straight run and diverge
            # by exactly the lateral deviation once anything has deflected.
            # Measured from the SAME progress anchor as traveled, so the derived
            # lateral term below stays a real right-triangle leg rather than
            # mixing two different origins. Since the corridor stopped homing
            # laterally this lateral term is the one place a run's accumulated
            # sideways offset from the intended line is still visible from the
            # goal's own point of view (build_straight_corridor logs the same
            # quantity as lat_off, from the corridor's).
            displacement = math.hypot(self.x - progress_anchor_x,
                                      self.y - progress_anchor_y)
            self.get_logger().info(
                f'GOAL | traveled={traveled:.4f}/{self.goal_distance:.3f} m '
                f'residuo={self.goal_distance - traveled:+.4f} m '
                f'displacement={displacement:.4f} m '
                f'lateral={math.sqrt(max(displacement ** 2 - traveled ** 2, 0.0)):.4f} m'
            )

            if traveled >= self.goal_distance:
                self.goal_reached = True
                self._publish_drive(0.0, 0.0)
                self.goal_reached_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f'Goal raggiunto: traveled={traveled:.3f} m >= goal_distance={self.goal_distance:.3f} m'
                )
                return

        # confronto tra odometria attuale e primo stato predetto al ciclo precedente
        if self.prev_predicted_state is not None:
            x_pred = self.prev_predicted_state

            ex = float(self.x - x_pred[0])
            ey = float(self.y - x_pred[1])

            eyaw = float(self.yaw - x_pred[2])
            eyaw = math.atan2(math.sin(eyaw), math.cos(eyaw))

            ev = float(self.v - x_pred[3])

            self.get_logger().info(
                f'ERR | ex={ex:.3f} ey={ey:.3f} eyaw={eyaw:.3f} ev={ev:.3f}'
            )

            # ---- DEBUG: un ev che diverge monotonicamente = il modello accelera
            # mentre la realta' resta ferma (nessuna attuazione).
            if abs(ev) > 0.5:
                self.get_logger().warn(
                    f'ERR | divergenza velocita\' predetta/reale: ev={ev:+.3f} '
                    f'(v_odom={self.v:.3f} v_pred={x_pred[3]:.3f})',
                    throttle_duration_sec=3.0
                )

        obstacles_global = self.obstacles_global_live

        self.get_logger().info(f'OBSTACLES WORLD={obstacles_global}')
        self.get_logger().info(f'DIST ROBOT-OSTACOLO = {d_robot_obs:.3f}')

        x0 = np.array([self.x, self.y, self.yaw, self.v], dtype=float)

        vdes = self.vdes

        now_time = self.get_clock().now()
        now_sec = now_time.nanoseconds * 1e-9

        need_update = False
        if self.cached_corridor is None or self.last_corridor_time is None:
            need_update = True
        elif (now_sec - self.last_corridor_time) >= self.corridor_update_period:
            need_update = True

        # UPGRADE: corridor geometry (walls) stays on the slow cadence -- it
        # barely changes while driving straight. Obstacle-aware target
        # selection must NOT be gated by this, see below.
        if need_update:
            self.cached_corridor = self.build_straight_corridor(x0)
            self.last_corridor_time = now_sec
            # The actual computation instant, for /mpc/corridor_markers'
            # header.stamp below -- NOT re-derived from a fresh get_clock().
            # now() at publish time, which would be off by however long the
            # rest of this tick (obstacle attach, solve, publish_drive) takes.
            self.last_corridor_stamp = now_time

            # ---- DEBUG: il corridoio e' stato ricostruito ----
            self.get_logger().info(
                f'CORR | rebuilt: Pend=({self.cached_corridor["Pend"][0]:+.3f},'
                f'{self.cached_corridor["Pend"][1]:+.3f}) '
                f'psiRef={self.cached_corridor["psiRef"]:+.3f}'
            )
            self._publish_corridor_markers(self.cached_corridor, self.last_corridor_stamp)

        corridor = self.cached_corridor

        # UPGRADE: attach live obstacles/safety distance BEFORE computing the
        # target, and recompute the target every tick (not just on corridor
        # rebuild). Previously compute_local_target was only ever called right
        # after build_straight_corridor(), on a dict that did not have
        # "obstacles_world" set yet -- so its avoidance loop always saw an
        # empty obstacle list and never actually deflected the target. All
        # avoidance was coming from the solver's w_obs cost alone, fighting
        # every tick against a static straight-line target -- hence the
        # oversized, last-moment corrections.
        corridor["obstacles_world"] = obstacles_global
        corridor["d_safe"] = self.dmin
        # UPGRADE: car_radius/avoidance_margin, read by both
        # compute_local_target (below, this file) and mpc_solver.py's
        # planner_cost_corridor -- passed via the corridor dict rather than
        # adding new solve_mpc_step parameters. d_safe above is left
        # unchanged/still set (self.dmin) for the disabled hard constraint's
        # potential future use; it no longer drives either active mechanism
        # after this fix.
        corridor["car_radius"] = self.car_radius
        corridor["avoidance_margin"] = self.avoidance_margin

        pref_nom = self.compute_local_target(x0, corridor)
        self.cached_pref_nom = pref_nom

        # ---- DEBUG sintetico ----
        self.get_logger().info(
            f'DBG | front={self.front_distance:.2f} '
            f'psiRef={corridor["psiRef"]:.2f} yaw={self.yaw:.2f} '
            f'vdes={vdes:.2f} PEND={pref_nom} delta={float(self.last_u[0]):.3f} v={self.v:.2f}'
        )

        self.save_corridor_snapshot(corridor, obstacles_global)

        # ---- DEBUG: stato e ingressi passati al solver ----
        self.get_logger().info(
            f'SOLVE/in | x0=[{x0[0]:+.4f},{x0[1]:+.4f},{x0[2]:+.4f},{x0[3]:+.4f}] '
            f'last_u=[{self.last_u[0]:+.4f},{self.last_u[1]:+.4f}] '
            f'vdes={vdes:.3f} n_obs={len(obstacles_global)}'
        )

        live_boundaries = self._get_live_boundaries()
        self.get_logger().info(
            f'BOUNDARY/live | n={len(live_boundaries)}', throttle_duration_sec=2.0)

        solve_t0 = self.get_clock().now().nanoseconds * 1e-9

        u0, info = solve_mpc_step(
            x0=x0,
            last_u=self.last_u,
            pref_nom=pref_nom,
            corridor=corridor,
            horizon=self.N,
            ts=self.ts,
            params=self.params,
            limits=self.limits,
            weights=self.weights,
            obstacles=obstacles_global,
            dmin=self.dmin,
            vdes=vdes,
            solver='rti' if self.use_rti_solver else 'slsqp',
            boundaries=live_boundaries,
        )

        solve_dt = self.get_clock().now().nanoseconds * 1e-9 - solve_t0

        # ---- DEBUG: tempo di soluzione; se supera ts il loop va in ritardo ----
        # status_message added for this diagnostic run (hard boundary
        # constraints task) -- the bare int status code alone doesn't say
        # "primal infeasible" in a bag/log without cross-referencing OSQP's
        # own enum; the string does, for exactly the infeasibility-
        # detection case that fix's own regression test covers.
        # Published BEFORE the log line and before any of the early-outs below,
        # so a tick is recorded even on a solve that produces no usable x_pred.
        # cost is NaN when the backend reports none -- carried through as NaN
        # rather than coerced to 0.0, which would read as a converged zero-cost
        # solution.
        status_msg = MpcSolverStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.success = bool(info.get('success', False))
        try:
            status_msg.status = int(info.get('status', -1))
        except (TypeError, ValueError):
            # Some backends report a non-integer status; the string form below
            # still carries it, so this stays a diagnostic, not a crash.
            status_msg.status = -1
        status_msg.status_message = str(info.get('status_message', ''))
        status_msg.solve_dt_sec = float(solve_dt)
        status_msg.control_period_sec = float(self.ts)
        status_msg.cost = float(info.get('cost', float('nan')))
        status_msg.solver = 'rti' if self.use_rti_solver else 'slsqp'
        status_msg.n_boundary_constraints = int(len(live_boundaries))
        status_msg.n_obstacles = int(len(obstacles_global))
        # Predicted horizon (see MpcSolverStatus.msg's own section): the states
        # the solver just optimized over, published every tick so "what did the
        # MPC think would happen from here" is answerable from a bag instead of
        # being recomputed and discarded inside this loop. Frame is 'odom' --
        # the same frame, for the same reason, as _publish_corridor_markers'
        # own markers (x_pred is rolled forward from x0, which is odom-frame).
        # All-or-nothing: a solve with no usable x_pred publishes empty arrays
        # rather than a partial trajectory a consumer would have to guess at.
        x_pred_msg = info.get('x_pred')
        if x_pred_msg is not None and len(x_pred_msg) > 0:
            x_pred_arr = np.asarray(x_pred_msg, dtype=float)
            status_msg.prediction_frame_id = 'odom'
            status_msg.pred_x = x_pred_arr[:, 0].astype(np.float32).tolist()
            status_msg.pred_y = x_pred_arr[:, 1].astype(np.float32).tolist()
            status_msg.pred_yaw = x_pred_arr[:, 2].astype(np.float32).tolist()
            status_msg.pred_v = x_pred_arr[:, 3].astype(np.float32).tolist()
        self.solver_status_pub.publish(status_msg)

        self.get_logger().info(
            f'SOLVE/out | dt={solve_dt * 1e3:.1f} ms success={info.get("success")} '
            f'status={info.get("status")} status_message={info.get("status_message")} '
            f'cost={info.get("cost", float("nan")):.4f}',
            throttle_duration_sec=1.0
        )
        if solve_dt > self.ts:
            self.get_logger().warn(
                f'SOLVE/out | solver piu\' lento del periodo di controllo '
                f'({solve_dt * 1e3:.1f} ms > {self.ts * 1e3:.1f} ms)',
                throttle_duration_sec=3.0
            )
        if not info.get("success", False):
            self.get_logger().warn(
                f'SOLVE/out | ottimizzazione FALLITA status={info.get("status")}',
                throttle_duration_sec=2.0
            )

        if "x_pred" in info and len(info["x_pred"]) > 0:
            self.prev_predicted_state = info["x_pred"][0].copy()

            # ---- DEBUG: traiettoria di velocita' predetta sull'orizzonte ----
            v_traj = [f'{float(s[3]):.3f}' for s in info["x_pred"]]
            self.get_logger().info(f'PRED | v_horizon=[{", ".join(v_traj)}]')

            # UPGRADE: verify actual clearance along the *planned* trajectory,
            # not just the robot's current position -- min_obstacle_distance_pub
            # above only reflects "now", which says nothing about whether the
            # maneuver the solver just picked is actually going to clear the
            # obstacle by a safe margin.
            predicted_clearance = self.compute_predicted_clearance(
                info["x_pred"], obstacles_global
            )
            self.predicted_clearance_pub.publish(Float32(data=float(predicted_clearance)))
            self.get_logger().info(f'CLEARANCE | predicted_min={predicted_clearance:.3f} m')
            if predicted_clearance < self.dmin:
                self.get_logger().warn(
                    f'CLEARANCE | predicted min clearance {predicted_clearance:.3f} m '
                    f'< dmin {self.dmin:.3f} m over horizon',
                    throttle_duration_sec=1.0
                )

        delta_cmd = float(u0[0])
        a_cmd = float(u0[1])
        self.get_logger().info(f'INPUT MPC -> delta={delta_cmd:.4f}, a={a_cmd:.4f}')

        # ---- DEBUG: quali vincoli sono attivi su u0 ----
        at_delta_lim = (
            delta_cmd <= self.limits["delta_min"] + 1e-6 or
            delta_cmd >= self.limits["delta_max"] - 1e-6
        )
        at_a_lim = (
            a_cmd <= self.limits["a_min"] + 1e-6 or
            a_cmd >= self.limits["a_max"] - 1e-6
        )
        d_delta = delta_cmd - float(self.last_u[0])
        d_a = a_cmd - float(self.last_u[1])
        at_slew_lim = (
            d_a <= self.limits["dAMin"] * self.ts + 1e-9 or
            d_a >= self.limits["dAMax"] * self.ts - 1e-9
        )
        self.get_logger().info(
            f'LIM | delta_sat={at_delta_lim} a_sat={at_a_lim} slew_a_sat={at_slew_lim} '
            f'd_delta={d_delta:+.5f} d_a={d_a:+.5f}'
        )

        self.last_u = np.array([delta_cmd, a_cmd], dtype=float)

        v_cmd_now = self.v + a_cmd * self.ts

        v_cmd_log = self.prev_v_cmd if self.prev_v_cmd is not None else float("nan")
        self.prev_v_cmd = v_cmd_now

        wheel_speed = v_cmd_now / self.wheel_radius

        v_real = float(self.v)
        self.prev_v_real = v_real

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        t_mpc = self.mpc_k * self.ts
        self.mpc_k += 1

        a_imu = self.ax_imu if self.ax_imu is not None else float("nan")

        self.control_log_file.write(
            f'{now_sec},{t_mpc},{delta_cmd},{self.delta_real if self.delta_real is not None else float("nan")},{a_cmd},{v_cmd_log},{wheel_speed},{v_real},{a_imu}\n'
        )
        self.control_log_file.flush()

        self.get_logger().info(
            f'CTRL | t_mpc={t_mpc:.3f} delta_cmd={delta_cmd:.3f} a_cmd={a_cmd:.3f} '
            f'v_cmd={v_cmd_log:.3f} wheel_speed={wheel_speed:.3f} '
            f'v_real={v_real:.3f} a_imu={a_imu:.3f}'
        )

        self._publish_drive(v_cmd_now, delta_cmd)

        self.get_logger().info(
            f'MPC | x={self.x:.2f} y={self.y:.2f} yaw={self.yaw:.2f} v={self.v:.2f} '
            f'| delta={delta_cmd:.3f} a={a_cmd:.3f} v_cmd={v_cmd_log:.3f} '
            f'| obs_live={len(obstacles_global)} '
            f'| ok={info["success"]}'
        )

        if "zopt" in info:
            zopt = info["zopt"]
            self.get_logger().info(
                f'OPT | success={info["success"]} status={info["status"]} '
                f'cost={info["cost"]:.3f} preview={zopt[:min(len(zopt), 8)].tolist()}'
            )

        # ---- DEBUG: durata totale del tick ----
        loop_dt = self.get_clock().now().nanoseconds * 1e-9 - loop_t0
        self.get_logger().info(f'LOOP | dt={loop_dt * 1e3:.1f} ms (budget {self.ts * 1e3:.0f} ms)')

    # ==========================================
    # DEBUG DISTANZA
    # ==========================================
    def compute_robot_obstacle_distance(self, obstacles_global):
        if self.x is None or self.y is None:
            return 1e6

        dmin = 1e6
        for ox, oy, r in obstacles_global:
            d = math.hypot(self.x - ox, self.y - oy) - r
            dmin = min(dmin, d)
        return dmin

    # UPGRADE: clearance over the solver's *predicted* trajectory, not just the
    # current pose -- this is the actual verification that the chosen maneuver
    # keeps a safe margin, rather than just trusting the w_obs cost blindly.
    def compute_predicted_clearance(self, x_pred, obstacles_global):
        if not obstacles_global or x_pred is None or len(x_pred) == 0:
            return float('inf')

        dmin = float('inf')
        for state in x_pred:
            px, py = float(state[0]), float(state[1])
            for ox, oy, r in obstacles_global:
                d = math.hypot(px - ox, py - oy) - r - self.car_radius
                dmin = min(dmin, d)
        return dmin

    # ==========================================
    # CORRIDOIO — sempre dritto, l'evitamento ostacoli
    # e' interamente delegato a compute_local_target()
    # (deflessione tangenziale del target) e al peso w_obs
    # nel solver MPC.
    # ==========================================
    def build_straight_corridor(self, x):
        X0 = float(x[0])
        Y0 = float(x[1])
        psi0 = float(x[2])

        d_front = float(self.front_distance)

        psiStart = psi0

        if self.goal_pose_xy is not None:
            gx, gy = self.goal_pose_xy
            psiEnd = math.atan2(gy - Y0, gx - X0)
            L = float(np.clip(math.hypot(gx - X0, gy - Y0), 1.0, self.corr_L_base))
        else:
            # STRAIGHT (goal_distance) move: a corridor that ALWAYS PASSES
            # THROUGH THE CAR'S CURRENT POSITION, pointed along a FROZEN,
            # drift-corrected HEADING. Position tracks the car; direction does
            # not. That split is the whole design of this branch.
            #
            # WHAT THE HEADING IS, and why it is the half worth freezing: it is
            # goal_anchor_odom's psi -- the move's MAP-frame start heading,
            # reprojected into THIS tick's odom frame by _refresh_goal_anchor()
            # once per control_loop tick. It must NOT be a constant in odom
            # coordinates: this stack's dual-EKF split routes the car's real
            # heading drift into the map -> odom edge rather than into odom-frame
            # yaw, so a heading held constant in odom coordinates is a heading
            # rotating with the error, and presents the solver nothing to
            # correct. See _pose_odom_to_map's own docstring for the measurement
            # (13.1 deg of real yaw drift against under 0.5 deg of odom-frame
            # yaw). Reprojecting it turns that drift back into an ordinary
            # heading error the w_psi/w_term costs oppose -- confirmed live: the
            # accumulated drift this was built to kill is gone.
            #
            # WHAT THE ORIGIN IS, and why it is NOT the frozen line any more
            # (deliberate behaviour change, confirmed with Andreas -- NOT a
            # regression to be cautiously reverted): X0/Y0 stay the car's own
            # current position. The previous pass put the origin at the
            # perpendicular FOOT of the live pose on the frozen line, which
            # made the corridor home laterally back onto the exact original
            # line after any deflection. That worked, but it fed two moving
            # inputs into one geometry -- a live position and a static line
            # that itself steps whenever a SLAM correction lands in map -> odom
            # -- and the result was a visibly jittery corridor. Tracking the
            # car's position directly removes the lateral half of that motion
            # entirely and leaves a corridor that only ever pivots.
            #
            # THE PRICE, stated plainly so nobody has to rediscover it: there is
            # NO lateral homing any more (the HEADING return is back, see the
            # blend note below; lateral/cross-track return is deliberately out
            # of scope and is not in the reference implementation either).
            # After an obstacle deflection the car
            # keeps whatever sideways offset it picked up and simply carries on
            # parallel to the intended line, since the corridor -- and therefore
            # the lookahead target taken on its centerline (compute_local_target
            # -> pref_nom -> w_term) -- has moved sideways with it. Lateral
            # offset from the ORIGINAL line is once again zero by construction
            # here, so the corridor half-width bound and w_corr have nothing to
            # act on. That is the accepted trade for a simpler, steadier
            # corridor: the drift this all started with was a HEADING error, and
            # the frozen heading above is what actually fixes it.
            #
            # WHAT THIS DOES NOT TOUCH: the goal_distance progress/termination
            # check in control_loop still measures "distance made good" from the
            # ORIGINAL frozen start point (goal_anchor_odom's x/y) along this
            # same corrected heading -- see the two-anchor note at the
            # assignment below. The two must stay separate quantities, or
            # "goal_distance: 6.0" stops meaning 6 m along the intended line.
            #
            # WHICH END OF THE HEADING BLEND IS FROZEN -- corridor_heading_
            # return, and the default is BOTH (false). Read this whole note:
            # the argument below is the one the port was written on, and the
            # measurement contradicts its conclusion.
            #
            # THE ARGUMENT. With corridor_heading_return true, psiStart is the
            # car's LIVE yaw and psiEnd the move's FROZEN, map-corrected start
            # heading. The S-curve below therefore blends live -> frozen across
            # the corridor, and THAT blend is the heading return: the lookahead
            # target taken on the centerline (compute_local_target -> pref_nom
            # -> w_term) sits on the returning arc, so the solver steers back
            # onto the reference direction with no explicit heading cost
            # needed. It is the same thing the reference implementation does,
            # and the reason it runs with w_psi = 0 and w_corr = 0. Freezing
            # BOTH ends makes dpsi identically 0 and the S-curve inert here,
            # which reads like it must remove the RETURN along with the
            # drift-chasing: a corridor rigidly parallel to the reference line
            # and translated onto the car's current position appears to ask the
            # car only to hold the heading it already has.
            #
            # WHY IT IS WRONG. It is not the corridor alone that returns the
            # heading -- this stack runs w_psi = 1.5, a terminal yaw cost
            # pulling toward the corridor's psiRef, which is the FROZEN anchor
            # heading in both geometries. So both-ends-frozen still returns;
            # it just returns through w_psi instead of through the centerline
            # arc, and it does so at 100% of the error per rebuild instead of
            # the S-curve's 24.7%. Measured closed-loop (see stack_params.yaml
            # under corridor_update_period), both-ends-frozen is at zero
            # heading error in under 8 s having given up 0.089 m laterally;
            # the blend is still +0.0707 rad at 8 s and 0.750 m off, heading
            # for ~0.900 m of PERMANENT offset, because this corridor has no
            # lateral homing (see "THE PRICE" above). The blend does converge
            # eventually -- it is slow, not broken -- but it pays for the
            # slowness in offset that never comes back.
            #
            # true remains reachable as a launch arg: it is what f110_autonomy
            # and the MATLAB comparison do, and that comparison is still live.
            #
            # When no map anchor was captured (feature off, or no transform at
            # move start) goal_anchor_odom holds the raw odom-frame move-start
            # pose for the whole move, so this branch still runs -- with an
            # uncorrected frozen heading, which is the pre-drift-fix behaviour.
            psi_ref = self.psi_init_corridor
            anchor_pose = self.goal_anchor_odom
            if anchor_pose is not None:
                # ---- TWO SEPARATE ANCHORS. Do not collapse these into one. ----
                # progress_anchor_xy: the move's ORIGINAL start point (frozen,
                #   map-corrected). Owned by control_loop's termination check;
                #   read here ONLY for the lat_off diagnostic below. Nothing in
                #   this method's geometry is allowed to depend on it.
                # corridor_origin (X0, Y0): the car's CURRENT position, which is
                #   already expressed in the live odom frame the corridor, the
                #   markers and the solver's x0 all live in -- so it needs no
                #   reprojection of its own (odom -> map -> odom through the same
                #   latest transform is exactly the identity; see
                #   _pose_map_to_odom, the exact inverse of _pose_odom_to_map).
                #   X0/Y0 are therefore deliberately left as build_straight_
                #   corridor received them, NOT recomputed onto the frozen line.
                progress_anchor_xy = (float(anchor_pose[0]), float(anchor_pose[1]))
                # Frozen target heading; the ORIGIN end depends on the
                # parameter -- see the "WHICH END OF THE HEADING BLEND IS
                # FROZEN" note above.
                psiEnd = float(anchor_pose[2])
                # getattr, not a bare attribute, so the duck-typed stand-ins
                # the corridor tests build (which predate this parameter) still
                # select a defined shape. Its fallback tracks the SHIPPED
                # default from stack_params.yaml -- it is deliberately not a
                # second, independent opinion about which geometry is right.
                if getattr(self, 'corridor_heading_return',
                           get_value('corridor_heading_return')):
                    psiStart = psi0
                else:
                    # THE DEFAULT since 2026-09-08: both ends frozen, dpsi == 0,
                    # S-curve inert on this branch. It measures BETTER on the
                    # heading return at every rebuild period tested -- see the
                    # corridor_heading_return parameter's own note in __init__.
                    psiStart = psiEnd
                # Diagnostic only: how far the car now sits from the ORIGINAL
                # line. Deliberately not acted on any more (see "THE PRICE"
                # above) -- logged so a run can still be read back for how much
                # lateral offset a move actually accumulated.
                # psiEnd, not psiStart: this is the offset from the ORIGINAL
                # reference line, and that line's direction is the FROZEN
                # heading. psiStart is the live yaw now and would measure the
                # offset against a line that rotates with the car.
                lat_off = (-(X0 - progress_anchor_xy[0]) * math.sin(psiEnd)
                           + (Y0 - progress_anchor_xy[1]) * math.cos(psiEnd))
                self.get_logger().info(
                    f'CORR/tracking | origin=({X0:+.3f},{Y0:+.3f}) '
                    f'psi_live={psiStart:+.4f} psi_frozen={psiEnd:+.4f} '
                    f'progress_anchor=({progress_anchor_xy[0]:+.3f},'
                    f'{progress_anchor_xy[1]:+.3f}) lat_off={lat_off:+.3f}'
                )
            else:
                # Bootstrap fallback, unchanged from before this fix: no move
                # start has been recorded yet (no goal_distance received, or
                # only the odom bootstrap has set psi_init_corridor), so there
                # is no frozen line to reference. Blend from the live yaw toward
                # whatever heading reference does exist, off the live position.
                psiEnd = float(psi_ref) if psi_ref is not None else psi0
            L = max(self.corr_L_base, 1.0)

        u = np.linspace(0.0, 1.0, self.corr_N)
        s = L * u

        dpsi = math.atan2(math.sin(psiEnd - psiStart), math.cos(psiEnd - psiStart))
        # Heading blend shape: an S-curve (straight lead-in, sigmoid bend,
        # straight lead-out) rather than the previous flat linear taper
        # across the whole corridor length. Ported from f110_autonomy's
        # build_returning_corridor_explicit_t (the abandoned experimental
        # SLSQP-only branch reviewed 2026-08-31/2026-09-03) at Andreas's
        # explicit request -- this only ever changed the SHAPE of the blend
        # between psiStart and psiEnd, never which branch computes them.
        #
        # WHICH BRANCH THIS ACTUALLY AFFECTS: all of them, since the
        # heading-return fix. On the goal_pose branch (the one goal_turn
        # dispatches turns through) psiStart is the live yaw and psiEnd the
        # live bearing to the goal. On the goal_distance branch psiStart is
        # the live yaw and psiEnd the move's frozen, map-corrected start
        # heading, so dpsi is the car's accumulated heading error and this
        # shape is exactly what returns it. Same on that branch's bootstrap
        # fallback. dpsi == 0 (already aligned) still collapses every
        # expression below to a constant heading, which is correct: a car
        # already on the reference direction has nothing to return from.
        #
        # Scope note: at the shipping corridor_update_period of 1.0 s the
        # corridor this shape describes is a ~1s-lived reference, roughly
        # matching f110_autonomy's own slower-cadence design that the shape was
        # ported from. Set the period down to ~ts and it is rebuilt essentially
        # every control_loop tick, at which point the shape governs only the
        # near-term reference inside one replan window (N=20, ts=0.1, so 2.0 s
        # of horizon) rather than a standing scripted turn -- and the heading
        # return degrades accordingly, which is the measurement that fixed the
        # period at 1.0. On the goal_distance branch this shape is inert at the
        # default geometry anyway (corridor_heading_return false -> dpsi == 0);
        # it is live on every goal_pose turn regardless.
        tau = np.clip(
            (u - self.corr_turn_u_start) / max(self.corr_turn_u_end - self.corr_turn_u_start, 1e-6),
            0.0, 1.0)
        shape = 3.0 * tau ** 2 - 2.0 * tau ** 3
        theta = psiStart + dpsi * shape

        ds = np.zeros_like(s)
        ds[1:] = np.diff(s)

        xc = X0 + np.cumsum(np.cos(theta) * ds)
        yc = Y0 + np.cumsum(np.sin(theta) * ds)

        w0 = self.corr_wmin
        w1 = self.corr_wmax

        C0 = np.array([xc[0], yc[0]], dtype=float)
        C1 = np.array([xc[-1], yc[-1]], dtype=float)

        n0 = np.array([-math.sin(psiStart), math.cos(psiStart)], dtype=float)
        n1 = np.array([-math.sin(psiEnd), math.cos(psiEnd)], dtype=float)

        P_L0 = C0 + w0 * n0
        P_R0 = C0 - w0 * n0
        P_L1 = C1 + w1 * n1
        P_R1 = C1 - w1 * n1

        e0 = np.array([math.cos(psiStart), math.sin(psiStart)], dtype=float)
        e1 = np.array([math.cos(psiEnd), math.sin(psiEnd)], dtype=float)

        k0 = 0.55 * L
        k1 = 0.55 * L

        CL0 = P_L0 + k0 * e0
        CL1 = P_L1 - k1 * e1
        CR0 = P_R0 + k0 * e0
        CR1 = P_R1 - k1 * e1

        uu = u[:, None]

        left = (
            (1 - uu) ** 3 * P_L0 +
            3 * (1 - uu) ** 2 * uu * CL0 +
            3 * (1 - uu) * uu ** 2 * CL1 +
            uu ** 3 * P_L1
        )
        right = (
            (1 - uu) ** 3 * P_R0 +
            3 * (1 - uu) ** 2 * uu * CR0 +
            3 * (1 - uu) * uu ** 2 * CR1 +
            uu ** 3 * P_R1
        )

        xL = left[:, 0]
        yL = left[:, 1]
        xR = right[:, 0]
        yR = right[:, 1]

        dx = np.gradient(xc)
        dy = np.gradient(yc)
        dn = np.sqrt(dx ** 2 + dy ** 2)
        dn = np.maximum(dn, 1e-9)

        tx = dx / dn
        ty = dy / dn
        nx = -ty
        ny = tx

        halfWidth = 0.5 * np.sqrt((xL - xR) ** 2 + (yL - yR) ** 2)

        p_goal = np.array([xc[-1], yc[-1]], dtype=float)

        corridor = {
            "xc": xc,
            "yc": yc,
            "xL": xL,
            "yL": yL,
            "xR": xR,
            "yR": yR,
            "tx": tx,
            "ty": ty,
            "nx": nx,
            "ny": ny,
            "halfWidth": halfWidth,
            "psiRef": float(psiEnd),
            # The length THIS corridor was actually built with (the
            # goal_pose branch clips it into [1.0, corr_L_base], so it is not
            # always corr_L_base). _corridor_lookahead derives the lookahead
            # from it -- see that method.
            "L": float(L),
            "psiStart": float(psiStart),
            "t": float(dpsi),
            "Pend": p_goal,
            "dFront": float(d_front),
            "dpsi": float(dpsi),
        }

        # ---- DEBUG: geometria del corridoio appena costruito ----
        self.get_logger().info(
            f'CORR/build | L={L:.2f} psiStart(live)={psiStart:+.4f} '
            f'psiEnd(frozen)={psiEnd:+.4f} '
            f'dpsi={dpsi:+.4f} halfWidth=[{halfWidth[0]:.3f}..{halfWidth[-1]:.3f}] '
            f'dFront={d_front:.2f}'
        )

        return corridor

    def _corridor_line_marker(self, marker_id, ns, xs, ys, stamp, rgba):
        """One LINE_STRIP Marker from parallel x/y arrays (a corridor wall or
        the centerline) -- shared by _publish_corridor_markers() below, one
        call per polyline. Points are already absolute odom-frame coordinates
        (see that method's own frame comment), so pose stays identity -- no
        marker-local transform to apply."""
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = stamp.to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03  # line width, meters
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        # Slightly longer than corridor_update_period so a normal rebuild
        # refreshes each marker well before it would expire, but a stalled
        # mpc_corr (crashed, or stuck on a slow tick) makes the corridor
        # visibly disappear from Foxglove/RViz instead of silently showing a
        # stale, no-longer-true corridor forever -- same "don't let a viz
        # element imply liveness it doesn't have" reasoning as this repo's
        # own front_clearance/min_obstacle_distance staleness gaps (see
        # f1tenth_behavior/README.md's "Dependency failure behavior"
        # section) -- except here the marker's own lifetime enforces it
        # directly, rather than relying on a consumer to notice.
        # builtin_interfaces/Duration splits into sec + nanosec, and nanosec is
        # a uint32 the message class asserts on: anything >= 4294967296 (~4.295s)
        # raises "The 'nanosec' field must be an unsigned integer in
        # [0, 4294967295]". Packing the WHOLE lifetime into nanosec therefore
        # crashed mpc_corr outright for any corridor_update_period above about
        # 1.43s -- and stack_params.yaml's own default is 10.0, so this fired on
        # the very first _publish_corridor_markers() call, every run: the
        # AssertionError propagates out of control_loop() through rclpy's
        # executor and kills the process, navigation crash-loops until the
        # supervisor's restart budget is gone, and mpc_corr then stays absent
        # from the graph -- which /mission/start_mission's own preflight
        # (f1tenth_behavior/mission/preflight.py) correctly reports as "node
        # 'mpc_corr' not found in the ROS graph", i.e. start_mission stops
        # working with no obvious connection to a marker-lifetime line.
        lifetime_sec = 3.0 * self.corridor_update_period
        m.lifetime.sec = int(lifetime_sec)
        m.lifetime.nanosec = int(round((lifetime_sec - int(lifetime_sec)) * 1e9))
        m.points = [Point(x=float(px), y=float(py), z=0.0) for px, py in zip(xs, ys)]
        return m

    def _publish_corridor_markers(self, corridor, stamp):
        """Publishes the MPC's own soft reference corridor (build_straight_
        corridor()'s xL/yL/xR/yR wall polylines + xc/yc centerline) as a
        MarkerArray on /mpc/corridor_markers, for Foxglove/RViz -- see that
        publisher's own construction-site comment in __init__ for why
        MarkerArray (not PolygonStamped/Path) and why this is NOT the same
        thing as /costmap/boundaries.

        Frame: 'odom', not 'map' -- and this REMAINS correct after the
        map-frame anchor fix, which is worth spelling out because that fix
        makes the corridor's heading originate in map and so reads, at a
        glance, like it should have moved these markers to 'map' too. It
        should not. Every number in `corridor` is an odom-frame coordinate by
        the time it gets here: the origin X0/Y0 is the live pose from
        get_odom_topic()'s source (raw /odom or the LOCAL EKF's
        /odometry/filtered -- see _update_active_odom(), never the global
        map-frame EKF), and the heading is goal_anchor_odom's psi, which
        _refresh_goal_anchor() has already reprojected OUT of map and INTO
        this tick's odom frame via _pose_map_to_odom(). The anchor is STORED
        in map and CONSUMED in odom; only the stored form is map-frame, and it
        never reaches this method. Publishing under 'map' would therefore be
        wrong by the entire map -> odom offset -- which reached 1.88 m of
        translation and 32.3 deg of yaw by the end of the
        2026-09-07T12-48-40_mission-bottle_then_person run that motivated the
        fix, so it is not a rounding-level distinction.

        WHAT THIS MEANS FOR REPLAY, since that is the part that bit: these
        markers line up with map-frame data (the costmap, the occupancy grid,
        /ekf_global/odometry/filtered) only once the recorded map -> odom edge
        is applied, which RViz/Foxglove do automatically from /tf -- present
        in the bag at ~13.7 Hz. Comparing the raw marker coordinates against
        map-frame ones WITHOUT applying that edge is exactly the measurement
        that made the pre-fix corridor look 2.12 m adrift; the frame tag here
        is what tells a viewer (or an analyst) to apply it.
        """
        markers = MarkerArray()
        markers.markers.append(self._corridor_line_marker(
            0, 'corridor_left', corridor['xL'], corridor['yL'], stamp,
            (0.2, 0.6, 1.0, 0.9),
        ))
        markers.markers.append(self._corridor_line_marker(
            1, 'corridor_right', corridor['xR'], corridor['yR'], stamp,
            (1.0, 0.6, 0.2, 0.9),
        ))
        markers.markers.append(self._corridor_line_marker(
            2, 'corridor_centerline', corridor['xc'], corridor['yc'], stamp,
            (0.8, 0.8, 0.8, 0.6),
        ))
        self.corridor_markers_pub.publish(markers)

    def _corridor_lookahead(self, corridor):
        """Derive compute_local_target's lookahead from the corridor length.

        It is derived rather than carried as an independent constant because
        two requirements, both of which used to live only in prose:

        1. It must be a fraction of the corridor length (corr_lookahead_frac,
           0.5 -- the reference's 1.5 on L 3.0), so the target sits inside the
           corridor and the arclength advance actually operates instead of
           clamping to the last index every cycle.
        2. It must lie BEYOND the horizon's physical reach, N*ts*vdes, or the
           terminal cost changes character from "steer toward" to "arrive at".

        When the two disagree the reach floor wins and the mismatch is logged:
        that is a geometry that has been mis-configured, and it should say so
        rather than quietly degrade the way the 1.5/1.5 collision did.
        """
        L = float(corridor.get("L", self.corr_L_base))
        from_length = self.corr_lookahead_frac * L
        horizon_reach = float(self.N) * float(self.ts) * float(self.vdes)
        reach_floor = self.corr_lookahead_reach_margin * horizon_reach

        lookahead = from_length
        if from_length < reach_floor:
            lookahead = reach_floor
            self.get_logger().warn(
                f'TGT/geometry | corridor L={L:.2f} gives lookahead '
                f'{from_length:.2f} m, inside the horizon reach '
                f'{horizon_reach:.2f} m (floor {reach_floor:.2f}): the terminal '
                f'cost becomes an ARRIVAL target, not a direction pull. '
                f'Using the floor. Raise corr_L_base to at least '
                f'{reach_floor / max(self.corr_lookahead_frac, 1e-6):.2f}.',
                throttle_duration_sec=5.0)
            if lookahead >= L:
                self.get_logger().warn(
                    f'TGT/geometry | lookahead {lookahead:.2f} >= corridor '
                    f'length {L:.2f}: the target clamps to the corridor end on '
                    f'every cycle and never advances.',
                    throttle_duration_sec=5.0)
        return float(lookahead)

    def compute_local_target(self, x, corridor):
        p_robot = np.array([x[0], x[1]], dtype=float)

        xc = np.asarray(corridor["xc"], dtype=float)
        yc = np.asarray(corridor["yc"], dtype=float)

        d2 = (xc - p_robot[0]) ** 2 + (yc - p_robot[1]) ** 2
        idx = int(np.argmin(d2))
        lookahead = self._corridor_lookahead(corridor)

        ds = np.sqrt(np.diff(xc) ** 2 + np.diff(yc) ** 2)
        s_cum = np.concatenate(([0.0], np.cumsum(ds)))

        s_target = s_cum[idx] + lookahead
        idx_target = int(np.searchsorted(s_cum, s_target))
        idx_target = min(idx_target, len(xc) - 1)

        p_target = np.array(
            [xc[idx_target], yc[idx_target]],
            dtype=float
        )
        # Raw (pre-obstacle-deflection) target -- kept so the deflection-coast
        # logic below can tell whether THIS tick actually deflected anything,
        # and can decay a stale deflection back onto the (current) centerline
        # rather than some earlier tick's raw point.
        p_target_raw = p_target.copy()

        # ---- DEBUG: indice piu' vicino e indice di lookahead ----
        self.get_logger().info(
            f'TGT | idx={idx} idx_target={idx_target}/{len(xc) - 1} '
            f'lookahead={lookahead:.2f} nominale=({p_target[0]:+.3f},{p_target[1]:+.3f})'
        )

        # se il target lookahead cade dentro il margine di sicurezza di un
        # ostacolo, spostalo tangenzialmente fuori: e' qui che avviene
        # l'evitamento ostacoli / cambio di direzione locale, senza bisogno
        # di un piano.
        #
        # UPGRADE:
        #  - il raggio di innesco ora usa la stessa distanza di sicurezza del
        #    solver (R_safe = r + d_safe) invece del solo raggio ostacolo,
        #    cosi' il target inizia a muoversi PRIMA che il costo del solver
        #    debba intervenire con forza, non dopo.
        #  - lo spostamento tangenziale ora e' proporzionale a quanto il
        #    target ha "sconfinato" nel margine di sicurezza (0 a R_safe,
        #    massimo sulla superficie dell'ostacolo) invece di un salto fisso
        #    di 1.0 m.
        #
        # THE CEILING ON THAT DISPLACEMENT IS obstacle_target_shift_m, a
        # declared parameter read from stack_params.yaml (see __init__). It
        # used to be 0.6 * mean(halfWidth) -- corridor-derived, and so
        # self-limiting against leaving the corridor, but also unnamed,
        # untunable without a rebuild, and silently retuned by any change to
        # corr_wmin/corr_wmax. At the shipping geometry it evaluated to
        # exactly 0.36 m; the parameter now ships 0.30, deliberately smaller
        # so the car passes close to obstacles instead of swerving wide.
        # The PROPORTIONAL SHAPE below is unchanged -- this is only its
        # ceiling, still scaled by `penetration`.
        #
        # Because the value is no longer derived from the corridor, the
        # "never leaves the corridor" property is no longer structural: it
        # holds because 0.30 < the 0.4333 m narrow half-width, not because
        # the arithmetic forces it. The throttled warn below is what keeps
        # that from going quiet if either number is ever retuned past the
        # other.
        d_safe = corridor.get("d_safe", 0.0)
        max_defl = self.obstacle_target_shift

        mean_half_width = float(np.mean(corridor["halfWidth"]))
        if max_defl > mean_half_width:
            self.get_logger().warn(
                f'TGT/defl | obstacle_target_shift_m {max_defl:.3f} exceeds the '
                f'corridor mean half-width {mean_half_width:.3f}: a fully '
                f'penetrating obstacle deflects the target outside its own '
                f'corridor.',
                throttle_duration_sec=5.0)

        for ox, oy, r in corridor.get("obstacles_world", []):
            p_obs = np.array([ox, oy], dtype=float)
            R_safe = r + self.car_radius + self.avoidance_margin

            v = p_target - p_obs
            d = np.linalg.norm(v)

            if d < R_safe:
                if d < 1e-6:
                    psi = float(x[2])
                    v = np.array([np.cos(psi), np.sin(psi)], dtype=float)
                    d = 1e-6

                v_hat = v / d
                t = np.array([-v_hat[1], v_hat[0]], dtype=float)

                e = np.array([np.cos(float(x[2])), np.sin(float(x[2]))], dtype=float)
                if np.dot(t, e) < 0.0:
                    t = -t

                penetration = float(np.clip((R_safe - d) / max(R_safe - r, 1e-6), 0.0, 1.0))
                offset = max_defl * penetration

                p_target_old = p_target.copy()
                p_target = p_obs + R_safe * v_hat + offset * t

                # ---- DEBUG: deflessione tangenziale effettivamente applicata ----
                self.get_logger().info(
                    f'TGT/defl | ostacolo=({ox:+.3f},{oy:+.3f},r={r:.3f}) d={d:.3f}<R_safe={R_safe:.3f} '
                    f'pen={penetration:.2f} offset={offset:.3f} '
                    f'({p_target_old[0]:+.3f},{p_target_old[1]:+.3f}) -> '
                    f'({p_target[0]:+.3f},{p_target[1]:+.3f})'
                )

        # ---- Obstacle-deflection coast --------------------------------
        # deflected_this_tick is True iff the loop above actually moved
        # p_target away from the raw centerline point (i.e. some obstacle in
        # THIS frame's corridor["obstacles_world"] was inside R_safe of it).
        deflected_this_tick = not np.allclose(p_target, p_target_raw)

        if deflected_this_tick:
            self.last_deflection_vec = p_target - p_target_raw
            self.deflection_decay_remaining = self.deflection_decay_ticks
        elif self.deflection_decay_remaining > 0:
            # The obstacle that was deflecting the target is no longer in this
            # frame's list (missed detection, merged away, or genuinely
            # cleared) -- coast the deflection down to zero over
            # deflection_decay_ticks ticks instead of snapping back to the raw
            # centerline on this very first clear tick. Decrement BEFORE
            # computing the fraction so this first coast tick already shows
            # partial decay (not another full-strength tick) -- reaches
            # exactly zero after deflection_decay_ticks ticks, not ticks+1.
            self.deflection_decay_remaining -= 1
            decay_fraction = self.deflection_decay_remaining / self.deflection_decay_ticks
            p_target = p_target_raw + decay_fraction * self.last_deflection_vec
            self.get_logger().info(
                f'TGT/coast | no live deflection this tick, coasting '
                f'({self.deflection_decay_remaining}/{self.deflection_decay_ticks} '
                f'ticks left) -> ({p_target[0]:+.3f},{p_target[1]:+.3f})'
            )
        # else: no deflection this tick and none coasting -- p_target is
        # already the raw centerline point, nothing to do.

        # ---- Exponential smoothing on the final (possibly coasted) point ---
        if self.smoothed_target is None:
            self.smoothed_target = p_target.copy()
        else:
            alpha = self.target_smoothing_alpha
            self.smoothed_target = alpha * p_target + (1.0 - alpha) * self.smoothed_target

        return self.smoothed_target

    # ==========================================
    # TRASFORMAZIONE ROBOT -> GLOBALE
    # ==========================================
    def robot_to_global(self, x_r, y_r):
        x_g = self.x + math.cos(self.yaw) * x_r - math.sin(self.yaw) * y_r
        y_g = self.y + math.sin(self.yaw) * x_r + math.cos(self.yaw) * y_r
        return x_g, y_g

    # ==========================================
    # SAVE CORRIDOR DEBUG
    # ==========================================
    def save_corridor_snapshot(self, corridor, obstacles_global):
        if not hasattr(self, 'corridor_log_file'):
            return

        record = {
            "robot": {
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
                "v": self.v
            },
            "corridor": {
                "xc": corridor["xc"].tolist(),
                "yc": corridor["yc"].tolist(),
                "xL": corridor["xL"].tolist(),
                "yL": corridor["yL"].tolist(),
                "xR": corridor["xR"].tolist(),
                "yR": corridor["yR"].tolist(),
                "Pend": corridor["Pend"].tolist(),
            },
            "target": {
                "x": float(self.cached_pref_nom[0]),
                "y": float(self.cached_pref_nom[1])
            },
            "obstacles_world": [
                {"x": ox, "y": oy, "r": r}
                for (ox, oy, r) in obstacles_global
            ]
        }

        try:
            self.corridor_log_file.write(json.dumps(record) + "\n")
            self.corridor_log_file.flush()
        except Exception as e:
            self.get_logger().warn(f"Corridor log write failed: {e}")

    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = MPCController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()