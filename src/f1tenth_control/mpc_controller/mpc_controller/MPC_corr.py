import json
import collections
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data)

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Float32, Int32, String
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_messages.msg import (
    BoundaryConstraintArray, DriveClamp, DriveCommand, MissionStatus, MpcSolverStatus,
    MultiWallTrackArray, ObjectApproachStatus, ObjectGoal, Obstacle2DArray, ReverseStatus,
    SemanticObjectArray, SpeedChannelStatus, TurnGoal, WallTrack)
from f1tenth_messages.msg import MpcParamUpdate  # LLM2 (two-llm-arch), see _llm2_setup
from f1tenth_messages.srv import RequestReverse
from f1tenth_params.object_geometry import (
    ObstacleFilter, passes_obstacle_filter, SOLVER_FRONT_POINT_OFFSETS_M)
from f1tenth_params.param_defaults import get_odom_topic, get_value, get_wall_distance_topic
from f1tenth_params.semantic_objects import MEMORY as SEMANTIC_MEMORY
from f1tenth_params.semantic_objects import objects_to_odom_circles
from mpc_controller.campaign_status import corridor_payload, mpc_status_payload
from mpc_controller.cost_to_go import CostToGo, LatticeConfig
from mpc_controller.move_target import (
    MoveTarget, without_target_circles, without_target_objects)
from f1tenth_params.corridor_geometry import (
    CORRIDOR_HANDLE_FRAC, CORRIDOR_WIDTH_RATIO, POSE_HANDLE_FRAC,
    corridor_curves, corridor_curves_to_pose, wrap_pi)
from mpc_controller import wall_boundaries as wb
from mpc_controller.drive_limits import clamp_drive_speed, validate_speed_limits
from mpc_controller.llm2_params import (
    APPLIED_PARAMS_TOPIC, build_whitelist, LLM2_PARAMS_TOPIC, PendingUpdate, startup_warnings,
    tunable_weights, Update, validate_update)
from mpc_controller.model_log import ModelLogWriter
from mpc_controller.mpc_solver import STAGE_WEIGHT_REF_HORIZON, shift_warm_start, solve_mpc_step
from mpc_controller.stop_profile import (
    RAMP_UNKNOWN, StopRampLatch, StopRampProfile)
from mpc_controller import ramp_wall_source
from mpc_controller.object_approach import (
    TargetBehindPersistence, assess_object_approach, build_object_centreline,
    SPEED_BELOW_FLOOR, SPEED_DRIVE, ObjectStopLatch, floor_moving_speed,
    heading_margin_for, object_speed_decision, plan_object_heading)
from mpc_controller.object_guard import RefreshWatchdog, SteeringRamp
from mpc_controller.obstacle_select import apply_speed_cap, select_obstacles
from mpc_controller.reverse_manoeuvre import (
    Grid, ManoeuvreConfig, MODE_FORWARD, MODE_REVERSE, MODE_STOPPING, RearEnv, RearSolver,
    ReverseSupervisor, validate_reverse_mechanisms)
from mpc_controller.reverse_mpc import (
    extend_corridor_backwards, floor_moving_speed_signed, rear_probe_offsets, ReverseConfig,
    ReversePlanner, REVERSE as REVERSE_DIR)
from mpc_controller.speed_channel import (
    config_from as speed_config_from, FLOOR_ALL, FLOOR_OBJECT, SPEED_MODE_LEGACY,
    SPEED_MODE_OFFSET_FREE, SpeedChannel)
from mpc_controller.vehicle_model import (
    f1tenth_state_fcn_dt_beta, f1tenth_state_fcn_dt_beta_lag)
from mpc_controller.wall_tracker import PROVENANCE_NAMES, WallTracker, scan_to_odom_points
from mpc_controller.wall_turn import (
    SMOOTHSTEP_PEAK_SLOPE, min_turn_radius, plan_wall_turn_step)
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


#: The corridor's wall construction (corridor_geometry.CORRIDOR_WALL_SHAPES).
#: 'bezier': each wall a cubic Bezier from the start width tangent to psi
#: start to the end width tangent to psi end, i.e. the walls bend smoothly
#: with the corridor. 'trapezoid' (campaign-final Part D, a6dd23e) offset the
#: centreline by a linearly growing half-width and drew straight walls;
#: reverted 2026-10-13 on request. The width changes of Part D stay.
CORRIDOR_WALLS = 'bezier'


def lookahead_clamp_length(n_steps, ts, v_des, reach_margin):
    """Corridor length at and below which the lookahead pins to the corridor end.

    The lookahead is max(corr_lookahead_frac * L, this). The fractional term is
    0.5 * L, always inside the corridor, so it never pins; this floor is the
    only thing that can, and it does so exactly when it reaches L. At the
    shipping geometry (N 20, prediction step 0.2, vdes 0.5, margin 1.25) that
    is 2.5 m. ts here is the PREDICTION step (MPC_corr.ts_pred), not the
    control period: the floor is about where the horizon ends.

    A MODULE FUNCTION, not a method, because two callers need it and one of
    them is reached through the duck-typed corridor stand-ins: those bind
    _corridor_lookahead onto objects that carry the four values below but no
    methods, so a self.<method>() call there raises instead of computing. The
    other caller is the object branch's arc switch, which must stop using the
    arc before Pend starts feeding the terminal cost -- writing 1.25 there
    instead would leave the switch behind the moment any of these four moved.
    """
    return float(reach_margin) * float(n_steps) * float(ts) * float(v_des)


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


def _object_goal_debug_fields(msg):
    """Return an ObjectGoal's (track_id, gap_m), echoed on /mpc/object_status only.

    Neither steers nor stops anything. getattr: the callback-test stand-ins
    predate both fields, and a module function keeps them from needing a method.
    """
    return str(getattr(msg, 'track_id', '') or ''), float(getattr(msg, 'gap_m', math.nan))


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

        # =========================
        # Object mode (runtime command via /mpc/goal_object) -- the FIFTH goal
        # shape, and the first one that is REPUBLISHED throughout its move.
        #
        # WHAT MAKES IT DIFFERENT FROM goal_pose, which it superficially
        # resembles. goal_pose is a point, sent once; every message is a new
        # move, and goal_pose_callback re-anchors accordingly (clears the
        # corridor cache, drops the RTI warm start). An object goal is a point
        # that MOVES, refreshed at the tracker's rate, and re-anchoring per
        # refresh would drop the corridor cache and cold-start the solver on
        # every control tick -- at 20 Hz into a 10 Hz loop, permanently.
        #
        # So the callback branches on move_id BEFORE touching any state: a new
        # id is a new move and re-anchors; a repeat updates the target point
        # and the stamp and NOTHING else. See goal_object_callback.
        #
        # THE HELD HEADING. object_psi_c is the corridor's heading, carried
        # across rebuilds and rotated toward the target by object_approach.
        # plan_object_heading -- absolute, never an increment, for the reason
        # that module's docstring gives at length. It is seeded from the
        # bearing at the move's first message so a move starts pointed at its
        # target instead of sweeping onto it from the last move's heading.
        self.goal_object_move_id: Optional[str] = None
        self.goal_object_target_class: str = ''
        # The target as RECEIVED: map frame. Kept because the odom-frame copy
        # below has to be re-derived every tick from a transform that moves.
        self.goal_object_map_xy: Optional[Tuple[float, float]] = None
        # The same point in THIS tick's odom frame -- the one the corridor and
        # the solver actually use. Refreshed by _refresh_object_target(),
        # which holds its last good value when map -> odom is stale, exactly
        # as _refresh_goal_anchor does for the straight-move anchor.
        self.goal_object_odom_xy: Optional[Tuple[float, float]] = None
        self.goal_object_stamp: Optional[float] = None   # capture time, seconds
        self.goal_object_standoff: float = 0.0
        self.goal_object_speed: float = 0.0
        self.object_psi_c: Optional[float] = None
        # Odom-frame target at the last corridor build, for the early-rebuild
        # test (object_retarget_distance_m). None forces the first build.
        self.object_target_at_build: Optional[Tuple[float, float]] = None
        # True while _refresh_object_target is holding rather than tracking.
        self.object_target_held = False
        self.object_last_step = None    # ObjectHeadingStep, for the status topic
        # Echoed on /mpc/object_status for debugging; see _object_goal_debug_fields.
        self.goal_object_track_id = ''
        self.goal_object_gap_m = math.nan

        self.object_r_full = float(self.declare_parameter(
            'object_r_full_m', get_value('object_r_full_m')).value)
        self.object_r_freeze = float(self.declare_parameter(
            'object_r_freeze_m', get_value('object_r_freeze_m')).value)

        # ---- OBJECT CORRIDOR SHAPE -------------------------------------
        # WHAT THIS CHANGES. Today's object corridor sets psiStart == psiEnd
        # == psi_c, so dpsi is identically zero and the corridor is straight
        # by construction, whatever the car's heading error. Measured over the
        # 24 archived go_to_object rebuilds: dpsi used was 1e-5 rad while the
        # car's heading error toward the target ran 8-48 degrees. The corridor
        # pointed where the car was already going, never where it had to go.
        #
        #   'off'      today's geometry, unchanged. THE DEFAULT.
        #   'arc'      psiStart = live yaw, psiEnd = psi_c, origin = the car,
        #              at every range.
        #   'arc_far'  'arc' beyond the switch band, 'off' inside it.
        #
        # WHY 'arc' IS NOT THE DEFAULT AND 'arc_far' EXISTS. With the origin
        # on the car the corridor no longer ends at the goal: the arc bulges
        # off the straight line, measured mean 0.25 m and up to 0.96 m. That
        # is harmless while the lookahead still sits inside the corridor, and
        # NOT harmless once the lookahead clamps to the corridor end, because
        # from there Pend IS the terminal position cost and the car would be
        # sent to a point up to a metre off the goal -- the exact defect the
        # object branch's unclipped L was written to avoid. So the arc is used
        # only outside the clamp distance.
        self.object_corridor_mode = str(self.declare_parameter(
            'object_corridor_mode', get_value('object_corridor_mode')).value)
        if self.object_corridor_mode not in ('off', 'arc', 'arc_far'):
            self.get_logger().error(
                f"object_corridor_mode '{self.object_corridor_mode}' is not one "
                "of off|arc|arc_far: falling back to 'off'")
            self.object_corridor_mode = 'off'
        # WHICH GEOMETRY IS RUNNING, said out loud at startup and again at
        # every mission start (see hold_callback). A run was taken on 'off'
        # and read back from the figures as if it might have been the arc:
        # the mode is recorded in every corridor's v2 definition, but nobody
        # opens the jsonl mid-session. Default is INFO, anything else WARN --
        # a non-default geometry on the car is worth one loud line, and the
        # asymmetry is the point: silence in the log means the default.
        self._object_mode_default = str(get_value('object_corridor_mode'))
        self._log_object_corridor_mode('startup')
        # THE SWITCH BAND, as multiples of the lookahead clamp distance rather
        # than metres, so it follows that distance instead of restating it.
        # _lookahead_clamp_length() is the one definition; at the shipping
        # geometry it is 1.25 m, so these are 1.40 m and 1.10 m. Hysteresis
        # because the two geometries differ by the 0.5 m lead-in even when
        # dpsi is zero -- the step at the switch has a floor (measured 0.31 m
        # at dpsi = +1.0 deg), so a bare threshold would chatter across it.
        self.object_arc_switch_hi_frac = float(self.declare_parameter(
            'object_arc_switch_hi_frac',
            get_value('object_arc_switch_hi_frac')).value)
        self.object_arc_switch_lo_frac = float(self.declare_parameter(
            'object_arc_switch_lo_frac',
            get_value('object_arc_switch_lo_frac')).value)
        if self.object_arc_switch_lo_frac > self.object_arc_switch_hi_frac:
            raise ValueError(
                f'object_arc_switch_lo_frac {self.object_arc_switch_lo_frac} '
                f'exceeds _hi_frac {self.object_arc_switch_hi_frac}: the band '
                'would invert and the mode would latch on noise')
        # Which side of the band the last rebuild landed on. None until the
        # first object rebuild; reset with the rest of the object state.
        self._object_arc_active = False

        # ---- THE ARC'S CROSS-TRACK TOLERANCES --------------------------
        # ARC-SCOPED, and that scoping is the point. Off the arc, w_corr is
        # the corridor centreline weight the straight and turn branches are
        # tuned at (effective 1.25, rho 1.0 over sigma 0.20) and w_line is
        # inert because there is no target line. Making these two sigmas
        # global would retune the straight branch for a feature that is off
        # by default.
        #
        # ON an arc corridor the two terms own DIFFERENT curves, so sharing
        # 0.20 m would be an accident rather than a derivation:
        #
        #   corr_sigma  tolerance on distance to the CORRIDOR CENTRELINE, the
        #     arc the car is being asked to fly. 0.25 m is 58% of the
        #     narrowest half-width those corridors actually have (0.4327 m
        #     measured over the archived rebuilds), so the soft cost acts well
        #     before the hard half-width row does.
        #
        #   line_sigma  tolerance on distance to the TARGET LINE. This is a
        #     SLOW HOMING term, not a path-following one: the arc is
        #     re-anchored on the car at every rebuild and so has no memory of
        #     accumulated lateral offset -- the same "no lateral homing by
        #     design" property corridor_heading_return's own measurement is
        #     about. 0.35 m sits ABOVE the arc's own mean departure from the
        #     line (0.194 m) so the term does not fight the corridor's
        #     designed shape, and BELOW the maximum (0.466 m) so it still acts
        #     on a real excursion.
        #
        # Effective weights rho/(20 sigma^2): 0.800 + 0.408 = 1.208 total,
        # against the 1.25 the car is tuned at, and w_line < w_corr so the
        # untested term is the weaker one on the first runs. PROVISIONAL:
        # both are parameters precisely so they can be walked between runs.
        self.arc_corr_sigma_m = float(self.declare_parameter(
            'mpc_arc_corr_sigma_m', get_value('mpc_arc_corr_sigma_m')).value)
        self.arc_line_sigma_m = float(self.declare_parameter(
            'mpc_arc_line_sigma_m', get_value('mpc_arc_line_sigma_m')).value)
        for _name, _sigma in (('mpc_arc_corr_sigma_m', self.arc_corr_sigma_m),
                              ('mpc_arc_line_sigma_m', self.arc_line_sigma_m)):
            if not _sigma > 0.0:
                raise ValueError(
                    f'{_name} must be > 0: the weight is rho / (7 sigma^2)')

        # ---- REFERENCE STEP --------------------------------------------
        # The previous corridor, kept only to measure how far the reference
        # MOVES between rebuilds. At 1 Hz with the car at ~0.45 m/s the
        # centreline is re-laid from a pose half a metre further on every
        # second, so the solver is handed a visibly different reference each
        # time; that step is a wobble candidate in its own right and nothing
        # was measuring it. Three numbers, because they fail differently:
        # the centreline can shift while the tangent at the car holds (pure
        # lateral re-anchoring), the tangent can swing while the centreline
        # holds (a heading rethink), and Pend can jump on its own when the
        # target estimate moves.
        self._prev_corridor_ref = None
        self.object_c_safety = float(self.declare_parameter(
            'object_c_safety', get_value('object_c_safety')).value)
        self.object_retarget_distance = float(self.declare_parameter(
            'object_retarget_distance_m',
            get_value('object_retarget_distance_m')).value)
        # THE OPERATING FLOOR and the STOP ON ARRIVAL (object_approach's
        # ObjectStopLatch docstring). The approach drives at the move's speed,
        # never commanding less than min_moving_speed_mps while it moves, until
        # live r <= object_reach_tol_m + object_stop_distance_m; then zero,
        # latched for the rest of the move. object_reach_tol_m is the SAME key
        # the mission's object_reached reads, so the stop is placed where the
        # mission judges arrival.
        self.min_moving_speed = float(self.declare_parameter(
            'min_moving_speed_mps', get_value('min_moving_speed_mps')).value)
        self.object_stop_latch = ObjectStopLatch(
            float(self.declare_parameter(
                'object_reach_tol_m', get_value('object_reach_tol_m')).value),
            float(self.declare_parameter(
                'object_stop_distance_m', get_value('object_stop_distance_m')).value))
        self.object_speed_mode = None   # object_approach SPEED_*, this tick

        # THE DECELERATION RAMP (stop_profile.py, which has the derivation).
        # A cap on vdes -- the solver's speed reference along the whole horizon
        # -- computed from the distance still to give away to whichever
        # stopping point is nearest: the mission's wall target
        # (/mpc/stop_target), this node's own goal_distance, or its own
        # goal_pose. Not a drive lane, not a second controller: the solver
        # still produces the command and clamp_drive_speed still limits it.
        #
        # OBJECT MODE IS NOT ONE OF THE THREE. Its arrival already has a
        # measured, latched stop of exactly this shape (ObjectStopLatch above,
        # whose docstring is the reason this ramp hands over to a zero command
        # at all) and its speed reference is the one real per-move vdes path in
        # this file. Running both would be two authorities on one number.
        self.enable_stop_ramp = bool(self.declare_parameter(
            'enable_stop_ramp', get_value('enable_stop_ramp')).value)
        self.stop_ramp = None
        self.stop_ramp_latch = None
        if self.enable_stop_ramp:
            # Raises ValueError on an unusable tuning, deliberately at node
            # start rather than silently braking on invented numbers.
            self.stop_ramp = StopRampProfile(
                a_dec=float(self.declare_parameter(
                    'stop_ramp_a_dec_mps2',
                    get_value('stop_ramp_a_dec_mps2')).value),
                a_stop=float(self.declare_parameter(
                    'stop_ramp_a_stop_mps2',
                    get_value('stop_ramp_a_stop_mps2')).value),
                tau_latency_s=float(self.declare_parameter(
                    'stop_ramp_tau_latency_s',
                    get_value('stop_ramp_tau_latency_s')).value),
                v_min=self.min_moving_speed)
            # ONE latch, for the WALL stop only. The other two stopping points
            # already have theirs -- goal_reached and pose_goal_reached, each
            # of which makes its branch return a zero command for the rest of
            # the move -- and a second latch over the top of those would be a
            # duplicate piece of state to keep in sync for no gain.
            self.stop_ramp_latch = StopRampLatch(self.stop_ramp)
        # The mission's wall-distance target [m] for the current move, from
        # /mpc/stop_target; None when this move does not stop on a wall.
        self.stop_target_m: Optional[float] = None
        # Per-tick ramp bookkeeping, for the log line and /mpc/status.
        self.stop_ramp_mode = None
        self.stop_ramp_d_rem = None
        # Per-tick flags (see _assess_object_tick). The heading margin is not a
        # parameter: it is derived from r_freeze, the closest range the
        # inside_turn_radius test runs at (object_approach.heading_margin_for).
        self.object_heading_margin = heading_margin_for(self.object_r_freeze)
        self.object_behind = TargetBehindPersistence(float(self.declare_parameter(
            'object_behind_persist_sec',
            get_value('object_behind_persist_sec')).value))
        self.object_last_flags = None   # ObjectApproachFlags, this tick
        self.object_behind_terminal = False
        self.object_behind_for_s = 0.0
        # LEAVING object mode (see object_guard's module docstring). A move ends
        # on /mpc/goal_object_end or when /mpc/hold engages; its id is then
        # remembered, so a late queued ObjectGoal for it is ignored instead of
        # restarting the approach. Bounded: ids carry the run generation, so
        # only the recent past can ever be replayed.
        self.object_ended_move_ids = collections.deque(maxlen=64)
        self.object_exit_ramp = SteeringRamp(float(self.declare_parameter(
            'object_exit_steer_rate_rad_s',
            get_value('object_exit_steer_rate_rad_s')).value))
        self.object_exit_ramping = False
        self._object_exit_last_sec = None
        self.object_goal_watchdog = RefreshWatchdog(float(self.declare_parameter(
            'object_goal_timeout_sec', get_value('object_goal_timeout_sec')).value))
        self.object_goal_watchdog_tripped = False
        # What _publish_drive last actually sent, which is where the ramp-out
        # starts when no measured angle is available. NOT self.last_u[0]: the
        # hold and no-goal paths publish zero without touching last_u.
        self._last_published_steer = 0.0
        # Lead-in ahead of the car's projection onto the centreline. Not a
        # declared parameter: it exists only so compute_local_target's nearest
        # sample stays interior (see build_object_centreline) and there is
        # nothing to tune about it.
        self.object_lead_in_m = 0.5

        # =========================
        # Drive mode (runtime command via /mpc/goal_drive) -- the FOURTH goal
        # shape, and the only OPEN-ENDED one.
        # =========================
        # The other three all name a target this node drives toward and can
        # decide it has REACHED, at which point it latches zero and publishes
        # /mpc/goal_reached. A drive command names only a MODE: hold this
        # heading, or turn by this much and then hold the result. There is no
        # target, so there is nothing to arrive at, so this mode NEVER sets
        # goal_reached, NEVER publishes /mpc/goal_reached and NEVER
        # self-terminates. f1tenth_behavior's own stop_condition is the sole
        # authority on when the move ends -- see goal_drive_callback.
        #
        # Mutually exclusive with the other three in exactly the same way they
        # already are with each other: whichever topic was published to most
        # recently wins, and each callback clears the others' state.
        #
        # None means "no drive command active". A dict rather than five
        # parallel attributes so "is drive active" is one unambiguous check
        # (self.drive_cmd is not None) at all four of its read sites, instead
        # of a convention about which of five fields is authoritative.
        self.drive_cmd: Optional[dict] = None
        # Fallback for DriveCommand.msg's turn_mag_deg == 0 sentinel.
        # f110_autonomy hardcoded pi/2 here and never read a magnitude off the
        # phase at all, so a plan asking for 180 degrees silently got 90. This
        # port keeps 90 as the DEFAULT (matching that stack when a plan says
        # nothing) but reads the phase's own value whenever it has one -- that
        # is the fix for the bug, not a reproduction of it.
        self.drive_default_turn_mag_deg = float(
            self.declare_parameter('drive_default_turn_mag_deg', 90.0).value
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
        # OBSTACLES V2 (obstacle_select.py): ONE source for every obstacle,
        # the semantic layer's object list (recognised + 'unknown'), and the
        # list is aged and capped -- it used to latch forever when its
        # publisher went quiet (output/avoidance/log.md, A3).
        # obstacles_topic:=/perception/obstacles_2d restores the projector.
        self.obstacles_topic = str(self.declare_parameter(
            'obstacles_topic', get_value('mpc_obstacles_topic')).value)
        self.obstacles_max = int(self.declare_parameter(
            'obstacles_max', get_value('mpc_obstacles_max')).value)
        self.obstacles_max_age_s = float(self.declare_parameter(
            'obstacles_max_age_s', get_value('mpc_obstacles_max_age_s')).value)
        self.obstacles_live_time: Optional[float] = None
        # The semantic layer's second layer: an UNKNOWN object in the forward
        # strip close ahead caps the desired speed (-1.0 = no cap).
        self.unknown_speed_cap = None
        self.unknown_speed_cap_time: Optional[float] = None
        self.unknown_speed_cap_max_age_s = float(self.declare_parameter(
            'unknown_speed_cap_max_age_s', 0.5).value)

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
        # WALL ROWS (boundary_source 'walls' -- see wall_boundaries.py)
        # =========================
        # Nothing is transformed on receipt here, unlike the costmap path above:
        # a wall track is ALREADY in odom (MultiWallTrackArray.header.frame_id,
        # the same local-EKF odom self.x/y/yaw live in), and the one thing that
        # genuinely has to be re-derived every cycle -- which SIDE of each wall
        # the car is on -- depends on the car's CURRENT position, not on the
        # pose at the scan stamp. So the raw views are kept and the half-planes
        # are built in _get_live_boundaries, at solve time.
        self.wall_tracks: List[wb.WallTrackView] = []
        self.wall_tracks_frame: str = ''
        self.wall_tracks_last_time: Optional[float] = None
        self.wall_tracks_stamp: float = 0.0
        self.wall_coast_clock: Optional[wb.CoastClock] = None
        self.wall_row_identity: Optional[wb.RowIdentity] = None
        # Last cycle's rows/diagnostics, for /mpc/wall_constraints_diag and for
        # naming the rows involved when a solve comes back infeasible.
        self.wall_rows: List[wb.WallRow] = []
        self.wall_rows_diag: List[dict] = []
        # info['boundary_slack'] from the last solve, metres per QP slot. Empty
        # whenever the rows were hard or there was nothing live to soften.
        self.last_boundary_slack: List[float] = []

        # =========================
        # MPC / modello
        # =========================
        self.last_u = np.array([0.0, 0.0], dtype=float)

        # Previous tick's solved control sequence, shifted one step, handed
        # back to the RTI solver as its linearization reference AND OSQP's
        # initial iterate (mpc_solver.solve_mpc_step's warm_start_z). None
        # means "no usable previous solution" -- the solver then tiles
        # last_u, which is the pre-warm-start behaviour and the correct
        # prior on the first tick of a move.
        #
        # WHAT INVALIDATES IT, and why each one:
        #   - a failed solve            -- on failure the solver hands back
        #     the guess it was given, so keeping it would re-seed the next
        #     tick with the exact sequence that just failed to converge.
        #   - a new move (_invalidate_move_state) -- the corridor's psiRef
        #     changes there, so the stored plan was optimal for a DIFFERENT
        #     terminal heading. That is the "warm start from another
        #     corridor" bug and it is worse than no warm start at all.
        #   - a drive command ending (_clear_drive_state) -- same reason,
        #     plus vdes/avoidance_margin revert underneath it.
        # A PERIODIC corridor rebuild (corridor_update_period, mid-move)
        # deliberately does NOT invalidate it: that rebuild re-anchors the
        # corridor's position and psiStart at the live pose but leaves
        # psiRef -- the thing the stored plan was aiming at -- untouched.
        # Dropping the warm start once a second would put ~10% of ticks back
        # on the bad linearization this exists to remove.
        self.warm_start_z: Optional[np.ndarray] = None

        self.wheel_radius = 0.05
        self.ts = 0.1
        self.N = 20
        # PREDICTION step [s], separate from the control period self.ts. The
        # timer, the published command (v + a * ts), the first rate row and
        # the solve budget stay on self.ts; the model rollout, the later rate
        # rows and the horizon's physical reach use this. N * ts_pred = 4.0 s
        # at the shipped 0.2. Equal to self.ts reproduces the single-step
        # solver bit-for-bit, so mpc_prediction_dt_s:=0.1 is the rollback.
        self.ts_pred = float(self.declare_parameter(
            'mpc_prediction_dt_s', get_value('mpc_prediction_dt_s')).value)
        if not (math.isfinite(self.ts_pred) and self.ts_pred > 0.0):
            self.get_logger().error(
                f'mpc_prediction_dt_s={self.ts_pred} is not a positive time: '
                f'using the control period {self.ts}.')
            self.ts_pred = self.ts

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
        # and _get_live_boundaries below). When false, _get_live_boundaries
        # always returns [] regardless of what costmap_boundary_node is
        # publishing -- solve_mpc_step then sees boundaries=[], identical to
        # no source ever having been live.
        #
        # RESOLVED 2026-09-09 -- these four are now single-sourced from
        # stack_params.yaml through get_value(), like every other tuned
        # number in this file.
        #
        # THE DEFECT THIS REPLACES. The literal here was False and said so in
        # four places (this literal, its comment, mpc_solver.py's module
        # docstring, costmap_boundary_node.py's), while stack_params.yaml has
        # declared use_hard_boundary_constraints TRUE the whole time and
        # mpc_corr.launch.py has been passing it through. So the launched
        # stack -- the one on the vehicle -- has been solving WITH
        # costmap_boundary_node's wall rows since the key was added, and the
        # three "inert on the shipping configuration" boundary-shape
        # parameters below were live too. Same four-spellings-of-one-constant
        # failure CLAUDE.md records for corridor_update_period.
        #
        # Reviewed and TRUE IS KEPT -- see stack_params.yaml's own
        # "MPC BOUNDARY CONSTRAINTS" block for why. A bare `ros2 run` now
        # gets the deployed value too, which is the whole point of reading
        # get_value() rather than carrying a literal.
        self.use_hard_boundary_constraints = bool(
            self.declare_parameter(
                'use_hard_boundary_constraints',
                get_value('use_hard_boundary_constraints')).value)

        # Boundary-row shape. LIVE, not inert: use_hard_boundary_constraints
        # above is true on the launch path, so _get_live_boundaries returns
        # real rows and solve_mpc_step builds them with exactly these.
        #
        # boundary_max_sources: rows per stage. 3 matches costmap_boundary_
        # node's nearest-cell front/left/right output; a convex polytope
        # needs up to 8, and mpc_solver's pad_boundary_constraints would
        # otherwise TRUNCATE the extra faces silently.
        self.boundary_max_sources = int(
            self.declare_parameter(
                'boundary_max_sources', get_value('boundary_max_sources')).value)
        # boundary_hard: False (default) adds the rows with a slack variable
        # and a large penalty; True makes them strict. Soft is the default
        # deliberately -- see mpc_solver.py's own "SLACK AND THE SLOT COUNT"
        # docstring section for why a hard constraint derived from an
        # occupancy map is a hard failure mode.
        self.boundary_hard = bool(
            self.declare_parameter('boundary_hard', get_value('boundary_hard')).value)
        self.boundary_slack_weight = float(
            self.declare_parameter(
                'boundary_slack_weight', get_value('boundary_slack_weight')).value)
        # boundary_slack_max: the per-row slack cap, metres. A PARAMETER as of
        # this pass -- it was a solve_mpc_step keyword this node never passed,
        # so the shipped 0.5 m was untunable from config, and a wall the car
        # was further than that inside produced an infeasible QP that the logs
        # showed only as a discarded solve with no indication why. Default
        # unchanged.
        self.boundary_slack_max = float(
            self.declare_parameter(
                'boundary_slack_max', get_value('boundary_slack_max')).value)

        # ---- WALL ROWS (wall_boundaries.py) ---------------------------------
        # boundary_source picks where the rows come from, and 'costmap' -- the
        # default -- is byte-for-byte the behaviour that existed before this
        # pass: the /costmap/boundaries subscription, _select_live_boundaries,
        # boundary_slack_weight. 'walls' swaps in the tracker's half-planes and
        # wall_row_slack_weight; 'none' ships no rows at all, which is the A/B
        # baseline (it is NOT the same as use_hard_boundary_constraints false --
        # that also exists, is older, and is the MATLAB-comparison config).
        self.boundary_source = str(
            self.declare_parameter('boundary_source', get_value('boundary_source')).value)
        if self.boundary_source not in ('costmap', 'walls', 'none'):
            raise RuntimeError(
                f"boundary_source must be 'costmap', 'walls' or 'none', "
                f'got {self.boundary_source!r}')
        self.max_wall_constraints = int(
            self.declare_parameter(
                'max_wall_constraints', get_value('max_wall_constraints')).value)
        self.wall_row_max_coast_s = float(
            self.declare_parameter(
                'wall_row_max_coast_s', get_value('wall_row_max_coast_s')).value)
        self.wall_row_coast_margin_m = float(
            self.declare_parameter(
                'wall_row_coast_margin_m', get_value('wall_row_coast_margin_m')).value)
        self.wall_row_allow_stereo_only = bool(
            self.declare_parameter(
                'wall_row_allow_stereo_only',
                get_value('wall_row_allow_stereo_only')).value)
        self.other_walls_as_obstacles = bool(
            self.declare_parameter(
                'other_walls_as_obstacles', get_value('other_walls_as_obstacles')).value)
        self.wall_obstacle_spacing_m = float(
            self.declare_parameter(
                'wall_obstacle_spacing_m', get_value('wall_obstacle_spacing_m')).value)
        self.wall_obstacle_radius_m = float(
            self.declare_parameter(
                'wall_obstacle_radius_m', get_value('wall_obstacle_radius_m')).value)
        self.wall_row_end_margin_m = float(
            self.declare_parameter(
                'wall_row_end_margin_m', get_value('wall_row_end_margin_m')).value)
        self.wall_row_merge_angle_deg = float(
            self.declare_parameter(
                'wall_row_merge_angle_deg', get_value('wall_row_merge_angle_deg')).value)
        self.wall_row_merge_offset_m = float(
            self.declare_parameter(
                'wall_row_merge_offset_m', get_value('wall_row_merge_offset_m')).value)
        self.wall_row_slack_weight = float(
            self.declare_parameter(
                'wall_row_slack_weight', get_value('wall_row_slack_weight')).value)
        self.wall_row_max_tf_age_s = float(
            self.declare_parameter(
                'wall_row_max_tf_age_s', get_value('wall_row_max_tf_age_s')).value)
        # A MISMATCH IS A STARTUP ERROR, NEVER A SILENT TRUNCATION -- the
        # rule and its reasoning live in wb.check_row_capacity, which is pure
        # so it can be tested without constructing this node.
        capacity_error = wb.check_row_capacity(
            self.boundary_source, self.max_wall_constraints, self.boundary_max_sources)
        if capacity_error:
            raise RuntimeError(capacity_error)
        if self.boundary_source == 'walls':
            self.wall_coast_clock = wb.CoastClock()
            self.wall_row_identity = wb.RowIdentity(
                self.wall_row_merge_angle_deg, self.wall_row_merge_offset_m)

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

        # Wheelbase: a declared parameter (stack_params.yaml's mpc_wheelbase_m)
        # rather than the bare 0.305 literal it was, because the wall_turn
        # increment derives R_min from it -- see wall_turn.py. Same value.
        self.params = {
            "L": float(self.declare_parameter(
                'mpc_wheelbase_m', get_value('mpc_wheelbase_m')).value),
            "lr": 0.17,
        }

        # Corridoio MATLAB-like
        #
        # Narrowed to 1/3 width on request (2026-09-07): corr_wmin 1.3 ->
        # 0.4333, corr_wmax 2.3 -> 0.7667. WIDENED again (campaign-final,
        # 2026-10-12) with the 2.3 / 1.3 end-to-start ratio held fixed: the
        # start half-width is now the parameter corr_half_width_start_m and
        # the end is that times CORRIDOR_WIDTH_RATIO, so the shape cannot be
        # retuned by accident when the size is. See the yaml key.
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
        # corr_L_base and corr_N are bare literals with no ROS parameter
        # behind them (unlike corridor_update_period directly below, which IS
        # a declared param wired through stack_params.yaml), so there is no
        # launch-time override and no config file that records what they used
        # to be. The two half-widths were too, until campaign-final.
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
        self.corr_wmin = float(self.declare_parameter(
            'corr_half_width_start_m',
            get_value('corr_half_width_start_m')).value)
        if not self.corr_wmin > 0.0:
            raise ValueError(
                f'corr_half_width_start_m must be > 0, got {self.corr_wmin}')
        self.corr_wmax = self.corr_wmin * CORRIDOR_WIDTH_RATIO
        # The funnel's old per-metre opening (corridor_geometry's length_ref):
        # with it the end reaches corr_wmax only at corr_L_base and a 1 m
        # corridor ends barely wider than it starts. OFF by default, so the
        # end-to-start ratio is CORRIDOR_WIDTH_RATIO at every length.
        self.corr_width_scale_with_length = bool(self.declare_parameter(
            'corr_width_scale_with_length',
            get_value('corr_width_scale_with_length')).value)

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
            self.declare_parameter(
                'corr_turn_u_start', get_value('corr_turn_u_start')).value)
        self.corr_turn_u_end = float(
            self.declare_parameter(
                'corr_turn_u_end', get_value('corr_turn_u_end')).value)

        # wall_turn increment (build_straight_corridor's wall_turn branch, rule
        # in wall_turn.py). Each corridor rebuild asks only for the part of the
        # turn the distance to the wall and the MPC horizon can carry, instead
        # of the whole remaining angle.
        #
        # k_safety below smoothstep's 1.5 peak slope would ask for a curvature
        # above 1/R_min at the S-curve's steepest point, so it is raised to the
        # floor loudly rather than allowed to produce an untrackable reference.
        self.corr_wall_turn_k_safety = float(
            self.declare_parameter(
                'corr_wall_turn_k_safety', get_value('corr_wall_turn_k_safety')).value)
        if self.corr_wall_turn_k_safety < SMOOTHSTEP_PEAK_SLOPE:
            self.get_logger().error(
                f'corr_wall_turn_k_safety={self.corr_wall_turn_k_safety} is below '
                f'{SMOOTHSTEP_PEAK_SLOPE} (smoothstep peak slope): using '
                f'{SMOOTHSTEP_PEAK_SLOPE}, which has NO margin.')
            self.corr_wall_turn_k_safety = SMOOTHSTEP_PEAK_SLOPE
        self.corr_wall_turn_safety_margin_m = max(float(
            self.declare_parameter(
                'corr_wall_turn_safety_margin_m',
                get_value('corr_wall_turn_safety_margin_m')).value), 0.0)
        # /perception/front_distance older than this is treated as unknown, not
        # as a distance. Without it a dead camera path would leave the last
        # value (or the 10.0 bootstrap below) standing in for the wall forever.
        self.corr_wall_turn_front_distance_max_age_sec = float(
            self.declare_parameter(
                'corr_wall_turn_front_distance_max_age_sec',
                get_value('corr_wall_turn_front_distance_max_age_sec')).value)

        # =========================
        # d_wall corridor correction (the STRAIGHT drive branch only)
        # =========================
        # WHAT THIS IS AND WHERE IT LANDS. wall_distance_node tracks one wall
        # and publishes a heading correction on
        # /perception/d_wall/psi_correction (rad, left positive, same sense as
        # psi). It is added to psi_base in build_straight_corridor's
        # drive/"straight" branch and NOWHERE ELSE. Full analysis in
        # docs/wall_turn_investigation.md; the short version is three findings:
        #
        #  1. THE MPC DOES NOT OWN A TURN'S EXIT HEADING. A wall_turn move ends
        #     when f1tenth_behavior's orientation_delta stop_condition sees 90
        #     degrees of accumulated yaw (condition_eval.py), so the exit
        #     heading is move_start + 90 deg whatever the corridor asked for. An
        #     edit here can only change the PATH to those 90 degrees, and
        #     therefore where the car ends up.
        #  2. SO CORRECTING dpsi_this WAS REJECTED. It is bounded by
        #     min(|dpsi_rem|, ...) in wall_turn.py, so its authority goes to
        #     zero exactly as the turn completes -- which is where the
        #     correction is wanted -- and the ratchet suppresses the one sign
        #     that would help. It would also have to pass through the "ONE
        #     source of truth" invariant at the wall_turn branch below.
        #  3. THE POST-EXIT STRAIGHT CORRIDOR IS GENUINELY SEPARATE. Its psiEnd
        #     is psi_base, re-anchored per drive command; its psiRefTurn is
        #     None; it never calls plan_wall_turn_step, so neither the ratchet
        #     nor the horizon clip applies to it. That is the seam.
        #
        # IT DOES NOTHING TODAY UNLESS A MISSION HAS A MOVE AFTER THE TURN.
        # Every wall_turn mission in the repo sets terminal: true on the turn,
        # and on a terminal move AdvanceMove publishes /mpc/hold -- which
        # returns from control_loop BEFORE the corridor rebuild, so no post-exit
        # corridor is ever built. missions/wall_turn_then_straight.json is the
        # one that exercises this.
        self.corr_d_wall_correction_enable = bool(
            self.declare_parameter(
                'corr_d_wall_correction_enable',
                get_value('corr_d_wall_correction_enable')).value)
        # Older than this and the correction is treated as ABSENT, i.e. zero --
        # never as the last value still standing. A dead wall_distance_node must
        # decay to the geometry this branch had before the node existed, which
        # is the safe direction by construction. Same reasoning as
        # corr_wall_turn_front_distance_max_age_sec above.
        self.corr_d_wall_max_age_sec = float(
            self.declare_parameter(
                'corr_d_wall_max_age_sec', get_value('corr_d_wall_max_age_sec')).value)
        self.d_wall_correction = 0.0
        self.d_wall_correction_stamp_sec: Optional[float] = None

        # Wall tracker for the wall_turn increment (mpc_controller/
        # wall_tracker.py, design in its module docstring). Once a wall_turn
        # commits, the wall that triggered it is selected from /scan and held
        # as a line in the odom frame; from the next rebuild on the increment
        # runs on d_wall, the front bumper's perpendicular distance to that
        # line, instead of dFront -- which is measured along a heading that
        # rotates away from the wall mid-turn. dFront keeps the commit
        # decision, the safety layer and the fallback. None when disabled, and
        # every consumer below checks for None rather than a flag.
        #
        # The bumper offset is the URDF's base_front frame (base_front_x(),
        # no parameter: one base_link -> front offset for the whole stack) and
        # the inlier distance lidar_front_wall_inlier_distance_m: the same
        # physical quantity and the same LiDAR, so one source each rather
        # than a copy that could drift. See stack_params.yaml's WALL TRACKER
        # block.
        self.wall_track_enable = bool(
            self.declare_parameter('wall_track_enable', get_value('wall_track_enable')).value)
        # enable_scene_labels: the virtual wall is the single wall truth and is
        # already heading-independent (the wall normal's distance, dead-reckoned
        # on odometry through the turn, lidar_front_wall_node), so this private
        # /scan wall fit is bypassed rather than run as a second wall.
        if self.wall_track_enable and get_value('enable_scene_labels'):
            self.wall_track_enable = False
            self.get_logger().info(
                'enable_scene_labels: wall_turn runs on /perception/virtual_wall; '
                'the private wall tracker is bypassed')
        self.wall_tracker: Optional[WallTracker] = None
        if self.wall_track_enable:
            self.wall_tracker = WallTracker(
                normal_tol_rad=float(self.declare_parameter(
                    'wall_normal_tol_rad', get_value('wall_normal_tol_rad')).value),
                min_span_m=float(self.declare_parameter(
                    'wall_min_span_m', get_value('wall_min_span_m')).value),
                min_inliers=int(self.declare_parameter(
                    'wall_min_inliers', get_value('wall_min_inliers')).value),
                assoc_dist_m=float(self.declare_parameter(
                    'wall_assoc_dist_m', get_value('wall_assoc_dist_m')).value),
                dfront_slack_m=float(self.declare_parameter(
                    'wall_dfront_slack_m', get_value('wall_dfront_slack_m')).value),
                bumper_x_m=base_front_x(),
                inlier_distance_m=float(get_value('lidar_front_wall_inlier_distance_m')),
            )

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

        # UNWRAPPED rotation accumulated since the active wall_turn began, and
        # the yaw the last increment was measured from. Reset per turn by
        # goal_drive_callback.
        #
        # WHY A COUNTER AND NOT wrap(yaw - psi_init_corridor). A wrapped
        # difference lives in (-pi, pi], so it cannot tell 190 degrees of turn
        # from -170 degrees of turn -- they are the same number. Every turn
        # whose magnitude reaches 180 degrees therefore becomes ambiguous
        # exactly halfway through, and drive_turn_180.json asks for 180.0.
        # Summing per-tick increments, each of which is far below pi at any
        # reachable yaw rate, keeps the total unambiguous however far it goes.
        # See _accumulate_turn_progress and build_straight_corridor's wall_turn
        # branch for what consumes it.
        self.turn_progress_rad = 0.0
        self._turn_progress_last_yaw: Optional[float] = None
        # Per-turn memory of the wall_turn increment: whether the turn has
        # committed, and the end heading (as rotation from move start) the
        # last corridor asked for, which the ratchet will not let retreat.
        # Reset with the progress counter in _reset_turn_progress.
        self.wall_turn_committed = False
        self.wall_turn_commanded_rot: Optional[float] = None

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

        # ---- Model-validation log for tools/mpc_model_check.py (docs/DIAGNOSTICS.md).
        # Additive instrumentation: one row per SOLVED control step, written right
        # after the /drive publish, never read back by anything in this node.
        # One row per step of this timer (self.ts) -- not per odometry message, and
        # never at the ~2 Hz /slam/pose rate. Columns:
        #   t          header.stamp of the odometry message x0 came from, minus this
        #              node's clock at construction. The message stamp rather than
        #              receipt or solve time: it is closer to when the state was true.
        #   x, y, psi, v
        #              x0 exactly as handed to solve_mpc_step: ODOM-frame pose and
        #              forward speed from get_odom_topic() -- /odometry/filtered (local
        #              EKF, 50 Hz) with localization_source 'ekf', /odom with
        #              'raw_odom' -- or /model/virtual_robot/odometry only while the
        #              hardware source is stale. Not the map frame.
        #   steer_cmd  delta_cmd [rad] on the MODEL side of the servo mapping, ROS
        #              convention (+ = left, same sense as psi): exactly the /drive
        #              steering_angle, BEFORE ackermann_to_vesc_node applies the
        #              negative steering_angle_to_servo_gain_left/_right. Chosen
        #              because it is the quantity the solver's model uses, so the
        #              script's steering sign +1 is the one that should fit.
        #   accel_cmd  a_cmd [m/s^2], the model's acceleration input. What is actually
        #              published is the speed v + a_cmd * ts.
        # A relative model_log_path lands in this process's working directory; ''
        # disables the log. model_log.py explains why a step that reuses the previous
        # step's odometry message is skipped rather than written.
        self.hw_odom_stamp_sec = None
        self.sim_odom_stamp_sec = None
        self.state_stamp_sec = None
        self.model_log_t0 = self.get_clock().now().nanoseconds * 1e-9
        model_log_path = str(self.declare_parameter(
            'model_log_path', get_value('mpc_model_log_path')).value)
        self.model_log = ModelLogWriter(model_log_path)
        self._model_log_reported_skips = 0
        if self.model_log.enabled:
            self.get_logger().info(
                f'model log -> "{os.path.abspath(model_log_path)}" '
                '(t,x,y,psi,v,steer_cmd,accel_cmd; see docs/DIAGNOSTICS.md)')
        elif model_log_path:
            self.get_logger().warn(
                f'model log DISABLED: cannot open "{model_log_path}": {self.model_log.error}')
        else:
            self.get_logger().info('model log disabled (model_log_path is empty)')

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
        # delta_min/delta_max: DECLARED PARAMETERS, defaults read from
        # stack_params.yaml through get_value(), so a bare `ros2 run` that
        # bypasses the launch file gets the same envelope. See that file's
        # own "MPC ACTUATOR LIMITS" block for the derivation from
        # steering_calibration.yaml and for why the -0.264/+0.314 pair cited
        # in the work order was NOT used.
        #
        # These were -+1.05 rad (-+60 deg) bare literals. The servo clips at
        # about -+16 deg, so the QP spent every tick optimising over a
        # command range 3.8x wider than the actuator, and vesc_driver
        # silently clipped the excess. The silence is the damaging part: the
        # solver's own model integrates the command it CHOSE, so a saturated
        # tick predicted a trajectory the car was never going to fly, and
        # fed that prediction back in as the next tick's warm start.
        self.steering_angle_min = float(
            self.declare_parameter(
                'delta_min', get_value('mpc_steering_angle_min_rad')).value)
        self.steering_angle_max = float(
            self.declare_parameter(
                'delta_max', get_value('mpc_steering_angle_max_rad')).value)

        self.limits = {
            "delta_min": self.steering_angle_min,
            "delta_max": self.steering_angle_max,
            "a_min": -2.0,
            "a_max": 3.0,
            # dDeltaMin/dDeltaMax: LEFT AT -+0.5 rad/s (29 deg/s), and
            # DELIBERATELY NOT CHANGED, because it cannot be measured with
            # what this stack has. The work order asked for a measured slew
            # rate and said to leave it and flag it otherwise. Flagging it:
            #
            #   - The VESC returns no servo position. vesc_driver.cpp says so
            #     outright ("since vesc state does not include the servo
            #     position, publish the COMMANDED servo position as a
            #     'sensor'"), so /sensors/servo_position_command echoes the
            #     command, never the response.
            #   - /joint_states carries a static 0.0 for both steering
            #     hinges -- see description.launch.py, which is explicit that
            #     the hinges are not driven from real data.
            #   - None of the 39 archived mission bags records any servo or
            #     joint topic at all; they carry /drive and /ackermann_drive,
            #     which are commands.
            #   - No VESC is currently attached (/dev/ttyACM* absent).
            #
            # So there is no feedback channel to step-response. Measuring it
            # needs an instrument this stack does not have: a scope on the
            # servo line, or a constant-speed yaw-rate step inverted through
            # the bicycle model. Until then this bound stays a guess, and it
            # is a HARD one -- unlike w_du_delta it cannot be traded against
            # anything, so if it is too low it silently caps the steering
            # response no weight change can recover.
            "dDeltaMin": -0.5,
            "dDeltaMax": 0.5,
            "dAMin": -2.0,
            "dAMax": 2.0,
            # FORWARD ONLY, and 0.0 on purpose. The forward solver must never
            # plan its way backwards: reverse is a committed manoeuvre
            # (reverse_manoeuvre.py), not something braking falls into. With
            # max_reverse_speed_mps at 0.0 the /drive clamp hid this, but once
            # it was raised to 0.4 the forward solver's own braking undershoot
            # (v + a*ts dipping below zero near a stop) reached the wheels: on
            # 2026-10-20, 64 of 568 forward ticks commanded down to -0.153 m/s
            # in five separate windows, carrying FORWARD steering. The rear
            # solver is unaffected -- it overrides vMin/vMax with -v_rev on its
            # own limits dict (reverse_manoeuvre.py's RearSolver.solve).
            "vMin": 0.0,
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
        # w_corr WAS 0.0 AND IS NOW ON (effective 1.25). The note it used to
        # carry here is kept because its reasoning is still the reason to
        # watch this term, not because its conclusion still stands.
        #
        # What it recorded: w_corr degraded every metric at every value
        # tried (at w_psi 1.0, w_corr 0 -> 1.0 moved settle 1.34 -> 1.71 m
        # and clearance 0.143 -> 0.137; w_corr alone at 2.0 never settled).
        # The cause was structural -- the corridor was rebuilt from the LIVE
        # pose every tick, so its centreline passed through the car by
        # construction and "lateral offset from the centreline" measured
        # departure from THIS TICK'S plan rather than from the intended
        # line. Penalising that damped the very lateral motion an
        # avoidance-and-recovery manoeuvre is made of.
        #
        # That premise is gone: build_straight_corridor anchors straight
        # moves to a line frozen at move start, and the same note already
        # recorded that the term works on a frozen corridor (w_corr 0 -> 10
        # cut the horizon's end lateral offset 0.324 -> 0.246 m).
        #
        # WHAT IS STILL TRUE AND UNMEASURED. Those numbers were taken on the
        # OLD geometry and the OLD weight set; nothing here has been
        # re-measured closed-loop against the new one, and the stack has no
        # sim to re-measure it in. Two specific things to watch on the car:
        #
        #   - OVERSHOOT. w_corr is a stage cost, so it minimises the
        #     INTEGRATED cross-track error and will happily trade an
        #     end-of-horizon overshoot for it. Measured on the old set at
        #     w_corr 20: integrated error 3.39 -> 2.67 while the horizon
        #     ended at -0.149 m instead of +0.068 m, i.e. on the far side of
        #     the line. w_du_delta is what resists that. It went up
        #     (5.25 -> 8.33) partly for this reason and was then LOWERED to
        #     3.00 on 2026-09-09 -- see stack_params.yaml's mpc_w_du_delta,
        #     which explains why the derived 8.33 double-counted the hard
        #     rate bound. Overshoot about the line is therefore the FIRST
        #     thing to look for on the next run, and mpc_w_du_delta is the
        #     number to raise if it appears. The balance is reasoned, not
        #     measured, in both directions.
        #   - OBSTACLE CLEARANCE. The old note's mechanism -- a lateral
        #     penalty resists the deflection while an obstacle is still
        #     there -- has not gone away. w_obs was deliberately left at
        #     2.8 for less shyness, and w_corr now pulls the other way.
        # a bare `ros2 run` that bypasses the launch file gets the deployed
        # set too. Until this pass these were ten bare literals with no ROS
        # parameter behind them at all: no launch override, no config
        # record, and no way to change a weight without a rebuild. See that
        # file's own "MPC COST WEIGHTS" block for the rho/sigma^2 derivation
        # and the old -> new table.
        #
        # THE NUMBER IN THE YAML IS NOT THE NUMBER THE SOLVER APPLIES for
        # any key except w_term and w_psi. Those two are terminal and pass
        # through untouched; every other key is per-stage and is multiplied
        # by STAGE_WEIGHT_REF_HORIZON / N = 7/20 = 0.35 inside
        # solve_mpc_step (mpc_solver.scale_stage_weights). Quote the
        # EFFECTIVE number when comparing against a tuning note:
        #
        #   key            yaml literal   effective (x 0.35)
        #   w_corr              3.5714               1.2500
        #   w_psi_stage         4.7736               1.6708
        #   w_v                 4.2857               1.5000
        #   w_du_delta          8.5714               3.0000
        #   w_obs               8.0000               2.8000
        #   w_term              9.0                  9.0    (terminal)
        #   w_psi               4.5                  4.5    (terminal)
        #
        # The stage literals are rho / (7 * sigma^2), which is exactly what
        # makes the TOTAL stage cost over the horizon equal rho / sigma^2
        # for any N -- the set stays horizon-invariant through the existing
        # scale_stage_weights machinery rather than by dividing by N twice.
        # EXCEPT w_du_delta, which was deliberately moved off that derivation
        # (2026-09-09, 23.8095 -> 8.5714): its sigma was the servo's per-step
        # rate limit, so the derived value charged a soft cost for motion the
        # hard dDeltaMin/dDeltaMax bound already forbids. Horizon-invariance
        # is unaffected -- that property comes from scale_stage_weights, not
        # from the literal's provenance.
        def _w(key):
            return float(self.declare_parameter(key, get_value(key)).value)

        self.weights = {
            "w_term": _w('mpc_w_term'),
            "w_v": _w('mpc_w_v'),
            "w_psi": _w('mpc_w_psi'),
            "w_u_a": _w('mpc_w_u_a'),
            "w_du_delta": _w('mpc_w_du_delta'),
            "w_du_a": _w('mpc_w_du_a'),
            "w_delta0": _w('mpc_w_delta0'),
            "w_obs": _w('mpc_w_obs'),
            "w_corr": _w('mpc_w_corr'),
            "w_line": _w('mpc_w_line'),
            "w_psi_stage": _w('mpc_w_psi_stage'),
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
        # Baselines captured the instant the live values are first known, so a
        # per-move override (drive's speed / d_safe) can be UNDONE when that
        # move ends rather than leaking into every later move. Nothing else
        # writes these two.
        #
        # This matters most for the standoff: drive's approach_d_safe exists
        # to RELAX obstacle avoidance so the car can close on the object it
        # was told to stop in front of, and a relaxed standoff still in force
        # three moves later is a real safety regression, not a cosmetic one.
        self.vdes_default = self.vdes
        self.avoidance_margin_default = self.avoidance_margin

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
        #
        # ONE FILE PER NODE RUN, NEVER A TRUNCATION. This used to be a single
        # fixed corridor_debug.jsonl opened 'w', so every launch destroyed the
        # previous run's snapshots -- the file was found at 0 bytes, which is
        # what it is between a launch and the first solve, and every earlier
        # run was already gone. Nothing anywhere kept a copy, and these
        # snapshots carry the only record of the per-solve obstacle set and
        # local target; the corridor geometry itself is reproducible from
        # corridors.jsonl's v2 definition, but those two are not.
        #
        # The name now carries the node's start time and the file is opened
        # 'a', so a run can only ever add to its own file.
        self._corridor_log_stamp = datetime.now().strftime('%Y%m%dT%H%M%S')
        self.corridor_log_path = _resolve_debug_output_path(
            f'corridor_debug_{self._corridor_log_stamp}.jsonl')
        self.get_logger().info(f'corridor_log_path = "{self.corridor_log_path}"')

        # Set while a test-campaign test is open: snapshots go into that test's
        # own folder instead, so the run's evidence lands beside the streams it
        # belongs to rather than in a workspace-wide scratch file that the next
        # launch used to erase. See _on_campaign_status().
        self._corridor_test_file = None
        self._corridor_test_dir = None

        if self.save_corridor_debug:
            self.corridor_log_path.parent.mkdir(parents=True, exist_ok=True)
            self.corridor_log_file = open(self.corridor_log_path, 'a', encoding='utf-8')

        # The test-campaign logger names the open test (campaign_dir, mission,
        # test_id) on this topic every status tick. Subscribing is how this node
        # learns where to put a snapshot without either node importing the
        # other: mpc_controller must not depend on f1tenth_logger, and the
        # logger is often not running at all.
        self.create_subscription(
            String, '/test_campaign/logger_status', self._on_campaign_status, 10)

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

        # Drive-to-a-tracked-object. The ONE goal input that is republished
        # throughout its move rather than sent once, which is why it carries a
        # move_id and why its callback branches on that before touching state.
        self.sub_goal_object = self.create_subscription(
            ObjectGoal,
            '/mpc/goal_object',
            self.goal_object_callback,
            10
        )
        self.object_status_pub = self.create_publisher(
            ObjectApproachStatus, '/mpc/object_status', 10)
        # The mission's "this object move is over" (std_msgs/String move_id).
        # See goal_object_end_callback.
        self.sub_goal_object_end = self.create_subscription(
            String,
            '/mpc/goal_object_end',
            self.goal_object_end_callback,
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

        # f1tenth_behavior's PublishMoveGoal, for a mission "drive" step
        # (schema_version 3.0) -- the open-ended mode. See goal_drive_callback.
        self.sub_goal_drive = self.create_subscription(
            DriveCommand,
            '/mpc/goal_drive',
            self.goal_drive_callback,
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
            self.obstacles_topic,
            self.obstacles_2d_callback,
            10
        )
        self.sub_unknown_speed_cap = self.create_subscription(
            Float32, '/costmap/unknown_speed_cap', self.unknown_speed_cap_callback, 10)

        self.sub_costmap_boundaries = self.create_subscription(
            BoundaryConstraintArray,
            '/costmap/boundaries',
            self.costmap_boundaries_callback,
            10
        )

        # /perception/walls, ONLY in boundary_source 'walls'. Subscribing
        # unconditionally would deserialise up to four tracks at the tracker's
        # ~10 Hz on this node's executor for nothing in the default
        # configuration -- the same reason costmap_boundary_node's TF listener
        # now only exists with use_convex_polytope (cpu-cleanup, 2026-10-16).
        self.sub_wall_tracks = None
        self.wall_diag_pub = None
        if self.boundary_source == 'walls':
            self.sub_wall_tracks = self.create_subscription(
                MultiWallTrackArray,
                '/perception/walls',
                self.wall_tracks_callback,
                10
            )
            # A String of JSON, not a custom message, for exactly the reason
            # costmap_boundary_node's /costmap/safe_corridor_report is one: it
            # is a DIAGNOSTIC whose field set is still being shaped, nothing in
            # the stack parses it, and a .msg would freeze it prematurely --
            # and, worse here, every bag recorded against an older definition
            # would stop deserialising (CDR is positional).
            self.wall_diag_pub = self.create_publisher(
                String, '/mpc/wall_constraints_diag', 10)

        self.front_distance = 10.0
        # The wall contract's CLEAR value (stack_params wall_clear_distance_m):
        # at or above it the wall topics mean "nothing ahead", which must never
        # reach the geometry or the solver -- see _fresh_front_distance.
        self.wall_clear_distance_m = float(get_value('wall_clear_distance_m'))
        # Receipt time of the last front_distance message, for the wall_turn
        # increment's staleness check. None until one arrives, so the 10.0
        # bootstrap above is never mistaken for a measurement.
        self.front_distance_stamp_sec: Optional[float] = None
        # The one wall truth (get_wall_distance_topic): /perception/virtual_wall
        # with enable_scene_labels, else /perception/front_distance. Same
        # Float32 contract either way (from the bumper, <= 0 = no reading).
        self.sub_front_distance = self.create_subscription(
            Float32,
            get_wall_distance_topic(),
            self.front_distance_callback,
            10
        )

        # THE MISSION'S WALL TARGET for the deceleration ramp: the distance a
        # front_clearance move stops at, published once per move by
        # f1tenth_behavior's PublishMoveGoal (negative = this move does not
        # stop on a wall). This node reads the wall DISTANCE already, but the
        # TARGET lives in the mission JSON and there is no other way for it to
        # get here. Transient-local, depth 1: a value that changes once per
        # move must survive this node restarting mid-mission, and only the
        # latest one has any meaning.
        self.sub_stop_target = self.create_subscription(
            Float32,
            '/mpc/stop_target',
            self.stop_target_callback,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # THE FRONT-WALL LOCK, for the ramp (ramp_wall_source.py has the
        # cases). With wall_lock_enabled the ramp aims at the wall the mission
        # locked -- f1tenth_behavior publishes its track id on
        # /mpc/stop_wall_id -- reading its distance from the same /debug
        # payload the lock reads, instead of whichever wall the travel strip
        # meets on this scan. false: neither topic is subscribed and
        # _ramp_wall_distance is _fresh_front_distance, exactly as before.
        self.ramp_wall_source = None
        self.ramp_wall_pick = None      # this tick's RampWallPick, for the log
        if bool(self.declare_parameter(
                'wall_lock_enabled', get_value('wall_lock_enabled')).value):
            self.ramp_wall_source = ramp_wall_source.RampWallSource(
                max_age_s=float(get_value('wall_distance_max_age_s')),
                clear_distance_m=self.wall_clear_distance_m)
            self.sub_stop_wall_id = self.create_subscription(
                Int32, '/mpc/stop_wall_id', self.stop_wall_id_callback, 10)
            self.sub_virtual_wall_debug = self.create_subscription(
                String, '/perception/virtual_wall/debug',
                self.virtual_wall_debug_callback, 10)

        # The d_wall corridor correction. Subscribed even when
        # corr_d_wall_correction_enable is false, so the trace is on the wire
        # and in the bag either way and a run can be analysed for what the
        # correction WOULD have done -- which is exactly what Stage 4 of
        # docs/bringup_checklist.md does (max_psi_correction forced to 0.0,
        # node publishing, nothing applied). The flag gates the APPLICATION,
        # in build_straight_corridor, not the subscription.
        #
        # Reliable (plain depth-10), matching the publisher: this is a control
        # input to the corridor, not a sensor stream, and a dropped message is
        # a tick of stale geometry rather than a skipped sample.
        self.sub_d_wall_correction = self.create_subscription(
            Float32,
            str(get_value('wall_distance_output_topic')) + '/psi_correction',
            self.d_wall_correction_callback,
            10
        )

        # /scan for the wall tracker: the latest message only, with the odom
        # pose the car had when it arrived, so a rebuild can place its returns
        # in the odom frame. Nothing is fitted here -- that happens once per
        # control tick (_wall_track_tick), so at 40 Hz this callback is a
        # reference swap. Best-effort
        # sensor QoS: compatible with urg_node's reliable publisher, sends it
        # no acknowledgements, and cannot back-pressure it or the e-stop
        # (f1tenth_behavior's IsProximityTooClose) that reads the same topic.
        # base_link <- laser is a static edge, looked up once through the
        # buffer above (its map <-> odom-only scope note predates this; the
        # control state itself still never consults TF).
        self.scan_msg: Optional[LaserScan] = None
        self.scan_pose: Optional[Tuple[float, float, float]] = None
        self.scan_last_time: Optional[float] = None
        self._laser_pose: Optional[Tuple[str, Tuple[float, float, float]]] = None
        self.sub_scan = None
        if self.wall_tracker is not None:
            self.sub_scan = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

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
        # The single speed clamp on everything above -- see drive_limits.py.
        # Declared here, beside the publisher it guards, so nothing can publish
        # /drive before the limits exist.
        self.max_forward_speed = float(self.declare_parameter(
            'max_forward_speed_mps', get_value('max_forward_speed_mps')).value)
        self.max_reverse_speed = float(self.declare_parameter(
            'max_reverse_speed_mps', get_value('max_reverse_speed_mps')).value)
        validate_speed_limits(self.max_forward_speed, self.max_reverse_speed)
        self.drive_clamp_pub = self.create_publisher(DriveClamp, '/mpc/drive_clamp', 10)

        # REVERSE CHOSEN BY THE MPC (reverse_mpc.py). Off by default: with
        # allow_reverse false, _mpc_solve is solve_mpc_step and nothing below
        # is read on the control path (test_reverse_flag_off_identity.py).
        self.allow_reverse = bool(self.declare_parameter(
            'allow_reverse', get_value('allow_reverse')).value)
        # As set, before the speed-limit check below may switch it off: the
        # one-mechanism rule (_init_reverse_manoeuvre) is about the settings.
        self._allow_reverse_param = self.allow_reverse
        self.reverse_planner = None
        self.reverse_debug_pub = None
        reverse_cfg = self._reverse_config()   # declared either way, read only if on
        reverse_ctg = self._reverse_cost_to_go()
        if self.allow_reverse and reverse_cfg.allow_reverse:
            self.reverse_planner = ReversePlanner(reverse_cfg, ctg=reverse_ctg)
            self.reverse_debug_pub = self.create_publisher(
                String, '/mpc/reverse/debug', 10)
        else:
            self.allow_reverse = False

        # THE REVERSE MANOEUVRE (reverse_manoeuvre.py): a short reverse taken
        # only when the forward MPC is stuck, the rear is ready and the BT
        # allows it. Mutually exclusive with allow_reverse above, which lets
        # the forward MPC choose reverse itself.
        self.reverse_supervisor = None
        self.reverse_status_pub = None
        # Negative speeds refused by _publish_drive's final gate (diagnostics).
        self.reverse_suppressed = 0
        self._init_reverse_manoeuvre()

        # THE SPEED LAW (speed_channel.py): legacy v + a * ts, or offset-free.
        # After the reverse setup, which decides allow_reverse.
        self._init_speed_channel()

        # OBJECT MEMORY (f1tenth_costmap's semantic layer, Part B): objects
        # remembered out of view, with the radius inflated for people. The
        # MEMORY ones join the obstacle list (the TRACKED ones arrive live on
        # /perception/obstacles_2d already); all of them feed the reverse
        # manoeuvre's rear pre-check. Map frame, re-expressed every tick
        # through the current map -> odom. Each passes the projector's own
        # height/radius gate first, and the current object move's target is
        # left out of every list, live and remembered (move_target.py).
        self._init_semantic_memory()

        self.min_obstacle_distance_pub = self.create_publisher(
            Float32,
            '/mpc/min_obstacle_distance',
            10
        )

        # FORWARD-ONLY counterpart of the topic above -- a SECOND topic, not a
        # change to the first. Read this before merging them.
        #
        # /mpc/min_obstacle_distance is omnidirectional: compute_robot_obstacle_
        # distance takes the nearest obstacle in ANY direction, with no heading
        # term at all, so an object level with the rear axle counts exactly as
        # much as one dead ahead. That is the wrong signal for the mission
        # obstacle_distance_below stop_condition, whose whole meaning is
        # "something is in the way".
        #
        # It is published UNCHANGED anyway, and the new filtered value goes out
        # beside it, because every existing mission using obstacle_distance_below
        # was written against the omnidirectional behaviour and must keep it
        # exactly. condition_eval's own forward_only flag (default false)
        # selects which topic a given stop_condition reads -- see that module.
        #
        # f110_autonomy's distance_to_front_object() applied precisely this
        # filter (dot > 0 against the heading) and its guard "front_object" is
        # what maps onto obstacle_distance_below, so forward-only is the
        # faithful semantics for that path -- it just is not the safe DEFAULT
        # for missions that predate the distinction.
        self.min_obstacle_distance_forward_pub = self.create_publisher(
            Float32,
            '/mpc/min_obstacle_distance_forward',
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

        # The test-campaign logger's two feeds (f1tenth_logger test_campaign,
        # started by hand, never with the stack): std_msgs/String JSON, built
        # by campaign_status.py from values the tick has already computed.
        # /mpc/status after every solve, /corridor on every corridor rebuild,
        # with an id that counts rebuilds. Neither is read by anything that
        # controls the car, and both publishes are wrapped so a failure can
        # only lose a message -- see _publish_campaign_status.
        self.campaign_status_pub = self.create_publisher(String, '/mpc/status', 10)
        self.corridor_pub = self.create_publisher(String, '/corridor', 10)
        self._corridor_seq = 0

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

        # d_wall for the wall_turn increment, every control tick of a
        # wall_turn drive command, valid or not (see f1tenth_messages/
        # WallTrack.msg). A diagnostic feed like /mpc/solver_status: nothing
        # controls off the topic, but the trace is the evidence that the
        # tracker held the wall through a turn, so default reliable QoS.
        self.wall_track_pub = None
        if self.wall_tracker is not None:
            self.wall_track_pub = self.create_publisher(WallTrack, '/mpc/wall_track', 10)

        # LLM2's MPC-parameter updates (two-llm-arch). Off by default, and off
        # means not even subscribed -- see _llm2_setup. After self.weights and
        # the reverse setup, both of which it reads.
        self._llm2_setup()

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
            f'CFG | ts={self.ts} ts_pred={self.ts_pred} N={self.N} '
            f'vdes={self.vdes} dmin={self.dmin} '
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
        if hasattr(self, 'model_log'):
            self.model_log.close()
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
        # Model log only (see model_log_path in __init__).
        self.hw_odom_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

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
        # Model log only (see model_log_path in __init__).
        self.sim_odom_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ---- DEBUG ----
        self.get_logger().info(
            f'ODOM/sim | x={self.sim_x:+.4f} y={self.sim_y:+.4f} '
            f'yaw={self.sim_yaw:+.4f} v={self.sim_v:+.4f}',
            throttle_duration_sec=1.0
        )

    def _log_object_corridor_mode(self, when):
        """Say which object-corridor geometry is active, once, at `when`.

        WARN when it is not stack_params' default, INFO when it is: a run on
        a non-default geometry must be visible in a log read after the fact,
        and a run on the default must not add noise to every mission start.
        """
        mode = getattr(self, 'object_corridor_mode', 'off')
        default = getattr(self, '_object_mode_default', 'off')
        shape = {'off': 'straight corridor pinned to the target line',
                 'arc': 'arc from the car to the target, always',
                 'arc_far': 'arc beyond the switch band, straight inside it'}
        text = (f'OBJECT CORRIDOR | {when}: object_corridor_mode={mode!r} '
                f'({shape.get(mode, "unknown")})')
        if mode == default:
            self.get_logger().info(f'{text} -- the default')
        else:
            self.get_logger().warn(f'{text} -- NOT the default ({default!r})')

    def hold_callback(self, msg: Bool):
        if msg.data and getattr(self, 'reverse_supervisor', None) is not None:
            # A hold is a BT abort for a reverse manoeuvre too: it ends it.
            self.reverse_supervisor.abort('hold')
        if msg.data != self.hold:
            self.get_logger().info(f'HOLD | {"engaged" if msg.data else "released"}')
            # An abort holds; a mission start releases. Either way the speed
            # channel starts again from the zero the hold published.
            if getattr(self, 'speed_channel', None) is not None:
                self._speed_reset('hold' if msg.data else 'hold_release')
            # A hold RELEASE is this node's only view of "a mission just
            # started": the behaviour tree releases it as the mission goes
            # RUNNING. Logging the mode here puts it in the log next to the
            # run it governs, rather than only in a startup banner that may be
            # hours and several missions earlier.
            if not msg.data:
                self._log_object_corridor_mode('mission start')
        self.hold = msg.data
        # A hold ENDS an object approach; it does not pause it. Every mission
        # path that finishes a run holds (mission complete, abort, timeout), and
        # the object handler is not ticked afterwards to say so itself. Pausing
        # instead would resume driving at a stale target the moment the next
        # run releases the hold, before that run's first goal arrives.
        if msg.data and self.object_psi_c is not None:
            self._end_object_move('hold engaged')

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

        # Switching to distance mode -- clear any pose-mode goal so the modes
        # stay mutually exclusive (see goal_pose_callback), and any drive
        # command along with its per-move overrides (see _clear_drive_state).
        self.goal_pose_xy = None
        self.goal_pose_yaw = None
        self.pose_goal_reached = False
        self._clear_drive_state()
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
        self._clear_drive_state()
        self._invalidate_move_state()
        self.get_logger().info(
            f'Nuovo goal_pose=({self.goal_pose_xy[0]:.3f}, {self.goal_pose_xy[1]:.3f}) '
            f'yaw={self.goal_pose_yaw:+.3f} (yaw non ancora utilizzato, solo posizione)'
        )

    def goal_object_callback(self, msg: ObjectGoal):
        """Drive to a tracked object. Branches on move_id BEFORE touching state.

        THIS IS THE WHOLE POINT OF THE CALLBACK, so it is the first thing it
        does. Every other goal callback treats each message as a new move and
        re-anchors: clears the opposing modes, calls _invalidate_move_state()
        (corridor cache, target smoothing, RTI warm start), re-seeds the
        heading reference. That is right for a goal sent once.

        An object goal is REPUBLISHED at the tracker's rate for the whole move,
        because the target moves. Running the re-anchor path per message would
        drop the corridor cache and cold-start the solver on every single
        control tick -- corridor_update_period would stop meaning anything and
        the RTI warm start would never survive to be used. That is exactly
        what the retired object_goal_bridge did, driving /mpc/goal_pose at
        20 Hz, and it is the defect this message shape exists to remove.

        So:
          new move_id  -> a new move. Clear the other modes, invalidate, seed
                          psi_c from the CURRENT bearing to the target so the
                          corridor starts pointed at it rather than sweeping
                          onto it from whatever the last move left behind.
          same move_id -> the target moved. Store the point and the stamp.
                          Nothing else: no invalidation, no warm-start reset,
                          no cache null, no psi_c change (psi_c moves only at
                          a corridor rebuild, see build_straight_corridor's
                          object branch).

        The move_id must therefore be distinct between moves INCLUDING between
        two runs of the same single-move mission -- a mission's move ids repeat
        across runs. ObjectGoal.msg says so; the sender owns it.

        A standoff of zero or less is refused rather than clamped: it aims the
        corridor at the target itself, and the target may be a person.
        """
        move_id = str(msg.move_id)
        if move_id and move_id in self.object_ended_move_ids:
            # A queued refresh of a move that has already ended. Ignoring it is
            # the whole point of remembering ended ids: without this a late
            # message would re-seed psi_c and restart the approach.
            self.get_logger().warn(
                f'goal_object for ENDED move_id={move_id!r} ignored',
                throttle_duration_sec=2.0)
            return

        if self.x is None or self.y is None:
            self.get_logger().warn(
                'goal_object ricevuto ma stato ancora None: comando ignorato.')
            return

        standoff = float(msg.standoff)
        if not standoff > 0.0:
            self.get_logger().error(
                f'goal_object con standoff={standoff:.3f} <= 0 rifiutato: '
                'punterebbe il corridoio sull\'oggetto stesso.')
            return

        target_map = (float(msg.point.x), float(msg.point.y))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # ---- the same-move fast path. Deliberately first, deliberately tiny.
        if move_id and move_id == self.goal_object_move_id:
            self.goal_object_map_xy = target_map
            self.goal_object_stamp = stamp
            self.goal_object_standoff = standoff
            self.goal_object_speed = float(msg.speed)
            self.goal_object_track_id, self.goal_object_gap_m = (
                _object_goal_debug_fields(msg))
            self.object_goal_watchdog.note(now_sec)
            return

        # ---- a new move.
        # Mutually exclusive with the other four shapes, the same way they
        # already are with each other.
        self.goal_distance = None
        self.goal_start_xy = None
        self.goal_anchor_map = None
        self.goal_anchor_odom = None
        self.goal_reached = False
        self.goal_pose_xy = None
        self.goal_pose_yaw = None
        self.pose_goal_reached = False
        self._no_goal_warned = False
        self._clear_drive_state()
        # Clears object_psi_c / goal_object_odom_xy / object_target_at_build
        # along with the rest of the per-move geometry -- which is why the
        # object state is seeded AFTER this call, not before. It also marks the
        # PREVIOUS object move ended if one was active, which is why the new
        # move's id is assigned after it too.
        self._invalidate_move_state()

        self.goal_object_move_id = move_id
        self.goal_object_target_class = str(msg.target_class)
        self.goal_object_map_xy = target_map
        self.goal_object_stamp = stamp
        self.goal_object_standoff = standoff
        self.goal_object_speed = float(msg.speed)
        self.goal_object_track_id, self.goal_object_gap_m = (
            _object_goal_debug_fields(msg))
        self.object_goal_watchdog.reset()
        self.object_goal_watchdog.note(now_sec)
        self.object_goal_watchdog_tripped = False

        # Seed the held heading from the bearing NOW. The reprojection has not
        # run for this move yet, so the map point is used directly: at move
        # start the two frames differ only by the accumulated map -> odom
        # correction, which a bearing over several metres is insensitive to,
        # and the first rebuild replaces this with a properly reprojected one.
        dx = target_map[0] - self.x
        dy = target_map[1] - self.y
        self.object_psi_c = (math.atan2(dy, dx)
                             if math.hypot(dx, dy) > 1e-9 else self.yaw)
        self.object_target_held = False
        self.object_last_step = None
        # The only way into object mode, so the one place the persistence
        # clock needs resetting: flags are assessed only while psi_c is set.
        self.object_behind.reset()

        self.get_logger().info(
            f'Nuovo goal_object move_id={move_id!r} class={msg.target_class!r} '
            f'target_map=({target_map[0]:+.3f},{target_map[1]:+.3f}) '
            f'standoff={standoff:.2f} speed={msg.speed:.2f} '
            f'psi_c(seed)={self.object_psi_c:+.4f}')

    def goal_object_end_callback(self, msg: String):
        """Leave object mode and stop: the mission ended an object move.

        For the ACTIVE move this ends the approach (see _end_object_move). For
        any other id it only records the id as ended -- a move the mission
        ended before its first ObjectGoal ever arrived here must not start
        when that ObjectGoal finally does.
        """
        move_id = str(msg.data)
        if not move_id:
            return
        if move_id == self.goal_object_move_id and self.object_psi_c is not None:
            self._end_object_move('mission ended the move')
        else:
            self._mark_object_move_ended(move_id)

    def _mark_object_move_ended(self, move_id):
        if move_id and move_id not in self.object_ended_move_ids:
            self.object_ended_move_ids.append(move_id)

    def _end_object_move(self, reason):
        """Leave object mode: remember the id, drop the geometry, ramp the wheels out.

        Speed goes to zero on the next tick through the ordinary stop paths
        (hold, or no goal left); steering follows object_exit_ramp from the
        measured angle when /joint_states provides one, otherwise from what was
        last published. See object_guard.
        """
        move_id = self.goal_object_move_id
        self._mark_object_move_ended(move_id)
        self._invalidate_move_state()
        self.object_goal_watchdog.reset()
        self.object_goal_watchdog_tripped = False
        self.object_exit_ramp.seed(self._last_published_steer)
        self.object_exit_ramp.resync(getattr(self, 'delta_real', None))
        self.object_exit_ramping = True
        self._object_exit_last_sec = None
        self.get_logger().info(
            f'OBJECT/end | move_id={move_id!r}: {reason} -- object mode off, '
            f'steering ramps out from {self.object_exit_ramp.previous:+.3f} rad')

    def _publish_stop(self):
        """Publish zero speed; ramp the steering out if an object move just ended."""
        if not self.object_exit_ramping:
            self._publish_drive(0.0, 0.0)
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._object_exit_last_sec is None else now_sec - self._object_exit_last_sec
        self._object_exit_last_sec = now_sec
        steer = self.object_exit_ramp.apply(0.0, dt)
        if steer == 0.0:
            self.object_exit_ramping = False
        self._publish_drive(0.0, steer)

    def _refresh_object_target(self):
        """Reproject the object target into THIS tick's odom frame.

        The same job, the same transform and the same staleness policy as
        _refresh_goal_anchor -- read that docstring first; this is its
        counterpart for a target that also MOVES, and it reuses both
        _lookup_map_odom (with the same map_odom_max_age_sec guard) and
        _pose_map_to_odom rather than repeating either.

        Two differences worth stating, because neither is arbitrary:

        * A point, not a pose. _pose_map_to_odom's third return is a heading
          and there is none here -- the corridor's heading is object_psi_c,
          which lives in odom and is never reprojected (it is the held state;
          reprojecting it every tick would inject the map -> odom correction
          into it as a rotation and defeat the hold).
        * The held case is REPORTED, not just logged. A stale transform means
          the odom-frame target is drifting away from the real one at whatever
          rate the correction was moving, and a consumer deciding whether to
          keep driving needs to know -- so it goes out on
          /mpc/object_status.target_stale rather than only into a throttled
          warn nobody is reading at the time.
        """
        # object_psi_c, not goal_object_map_xy, is the "object mode is active"
        # test. _invalidate_move_state clears psi_c, so a goal of any other
        # shape switches this off; the map point and the move_id survive
        # (goal_object_callback owns those), and without this gate a refresh
        # arriving after a superseding goal_pose would repopulate the odom
        # target and the object branch would start driving again.
        if self.goal_object_map_xy is None or self.object_psi_c is None:
            return
        tf_map_odom = self._lookup_map_odom(max_age_sec=self.map_odom_max_age_sec)
        if tf_map_odom is None:
            # Hold the last good reprojection, exactly as _refresh_goal_anchor
            # does, and for the same reason: reverting to the raw map numbers
            # would jump the target by the whole accumulated correction.
            self.object_target_held = self.goal_object_odom_xy is not None
            if self.goal_object_odom_xy is None:
                # Nothing good to hold yet: the move began with no transform.
                # Using the map numbers raw is the pre-correction behaviour and
                # is better than having no target at all on the first tick.
                self.goal_object_odom_xy = self.goal_object_map_xy
            self.get_logger().warn(
                'OBJECT/hold | map -> odom unavailable OR STALE this tick: '
                'holding the last reprojected target',
                throttle_duration_sec=5.0)
            return
        x_odom, y_odom, _psi = _pose_map_to_odom(
            self.goal_object_map_xy[0], self.goal_object_map_xy[1], 0.0,
            *tf_map_odom)
        self.goal_object_odom_xy = (x_odom, y_odom)
        self.object_target_held = False

    def _object_range(self) -> float:
        """Range from the car to the STANDOFF point, in odom. Negative inside it.

        The same r object_approach computes; duplicated here only because the
        control loop needs it before the corridor is built (to set the speed
        reference) and the corridor build needs it after. Both read the same
        odom-frame target, refreshed once per tick.
        """
        if self.goal_object_odom_xy is None or self.x is None:
            return math.inf
        tx, ty = self.goal_object_odom_xy
        return math.hypot(tx - self.x, ty - self.y) - self.goal_object_standoff

    def _object_target_moved_since_build(self) -> bool:
        """Say whether the odom-frame target has moved far enough to rebuild early.

        A target refresh must NOT re-anchor the corridor -- that is what
        move_id is for, and re-anchoring per refresh is the defect this mode
        exists to remove. But a target that has genuinely walked away must not
        be chased at corridor_update_period's 1 Hz either, or the corridor
        points at where the person was a second ago.

        So the rebuild is triggered by DISTANCE MOVED, not by arrival of a
        message: object_retarget_distance_m (0.2 m) of real motion. Estimate
        jitter is well under that (5 cm is the figure the tests use), so noise
        alone does not trigger it and the period still governs the quiet case.
        """
        if self.object_target_at_build is None or self.goal_object_odom_xy is None:
            return True
        dx = self.goal_object_odom_xy[0] - self.object_target_at_build[0]
        dy = self.goal_object_odom_xy[1] - self.object_target_at_build[1]
        return math.hypot(dx, dy) >= self.object_retarget_distance

    def _assess_object_tick(self, x0, now_sec):
        """Assess this tick's approach geometry and flags from the live pose.

        EVERY TICK, not at rebuilds: a flag sampled at corridor rebuilds cannot
        bound its latency (object_approach's module docstring has the rig
        evidence). Runs after the rebuild check, so psi_c is the heading this
        tick's solve is actually flying. Returns None outside object mode.
        """
        if self.goal_object_odom_xy is None or self.object_psi_c is None:
            return None
        flags = assess_object_approach(
            self.object_psi_c, self.goal_object_odom_xy,
            (float(x0[0]), float(x0[1])), float(x0[2]),
            self.goal_object_standoff,
            r_freeze=self.object_r_freeze,
            heading_margin=self.object_heading_margin,
            wheelbase=self.params['L'],
            delta_min=self.limits['delta_min'],
            delta_max=self.limits['delta_max'])
        was_terminal = self.object_behind_terminal
        self.object_behind_terminal, self.object_behind_for_s = (
            self.object_behind.update(flags.target_behind, now_sec))
        if self.object_behind_terminal and not was_terminal:
            self.get_logger().warn(
                f'OBJECT/target_behind TERMINAL | held '
                f'{self.object_behind_for_s:.2f} s: alpha={flags.alpha:+.3f} '
                f'r={flags.r:+.3f} move_id={self.goal_object_move_id!r}')
        if flags.inside_turn_radius and not (
                self.object_last_flags is not None
                and self.object_last_flags.inside_turn_radius):
            self.get_logger().info(
                f'OBJECT/inside_turn_radius (advisory) | r={flags.r:+.3f} '
                f'alpha={flags.alpha:+.3f} psi_c={self.object_psi_c:+.4f}')
        self.object_last_flags = flags
        return flags

    def _publish_object_status(self, step, flags, speed_ref):
        """Everything the approach decided this tick, on /mpc/object_status.

        psi_c in particular is HELD state: a run cannot be read back from the
        pose and the target alone, so if it is not published it is not
        recoverable. target_age_s is the ObjectGoal stamp's whole purpose --
        the real age of the estimate being driven at, which a subscriber
        cannot reconstruct from its own arrival times.

        r, bearing, e and the flags come from this tick (`flags`); k and
        dpsi_max from the last rebuild (`step`). ObjectApproachStatus.msg
        labels each field accordingly.
        """
        message = ObjectApproachStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.move_id = self.goal_object_move_id or ''
        message.target_class = self.goal_object_target_class
        message.track_id = getattr(self, 'goal_object_track_id', '')
        message.gap = math.nan
        if self.object_psi_c is not None:
            message.psi_c = float(self.object_psi_c)
        if step is not None:
            message.k = float(step.k)
            message.dpsi_max = float(step.dpsi_max)
        if flags is not None:
            message.r = float(flags.r)
            message.bearing = float(flags.bearing)
            message.e = float(flags.e)
            message.alpha = float(flags.alpha)
            message.inside_turn_radius = bool(flags.inside_turn_radius)
            message.target_behind = bool(flags.target_behind)
            message.gap = float(flags.r) + float(getattr(self, 'goal_object_gap_m', math.nan))
        message.target_behind_for_s = float(self.object_behind_for_s)
        message.target_behind_terminal = bool(self.object_behind_terminal)
        message.goal_watchdog = bool(self.object_goal_watchdog_tripped)
        message.speed_ref = float(speed_ref)
        latch = getattr(self, 'object_stop_latch', None)
        message.stop_latched = bool(latch is not None and latch.latched)
        message.speed = float(self.v) if getattr(self, 'v', None) is not None else 0.0
        message.target_stale = bool(self.object_target_held)
        if self.goal_object_stamp is not None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            message.target_age_s = float(now_sec - self.goal_object_stamp)
        self.object_status_pub.publish(message)

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
        self._clear_drive_state()
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

    def _clear_drive_state(self):
        """Drop any active drive command AND undo its per-move overrides.

        Called by the other three goal callbacks (a new goal of any shape
        supersedes a drive command, exactly as the existing three already
        supersede each other) and by goal_drive_callback itself before
        installing a new one.

        THE OVERRIDES ARE THE POINT. Dropping self.drive_cmd alone would leave
        that move's relaxed obstacle standoff and its speed in force for every
        later move -- and approach_d_safe exists specifically to RELAX
        avoidance so the car can close on the object it was told to stop in
        front of. A relaxed standoff still active three moves later is a real
        safety regression. Restoring from the baselines captured in __init__
        (vdes_default/avoidance_margin_default), not from whatever the values
        happened to be a moment ago, so repeated overrides cannot ratchet.

        Deliberately does NOT touch goal_reached/pose_goal_reached/last_u or
        any corridor cache -- each caller owns those, and _invalidate_move_
        state() is what handles the geometry.
        """
        if self.drive_cmd is None:
            return
        self.drive_cmd = None
        self.vdes = self.vdes_default
        self.avoidance_margin = self.avoidance_margin_default
        # The tracked wall belongs to the wall_turn that just ended. getattr
        # for test_warm_start's duck-typed stand-in.
        tracker = getattr(self, 'wall_tracker', None)
        if tracker is not None:
            tracker.reset()
        # The stored plan was optimal for THIS drive command's corridor
        # (build_straight_corridor's drive branch: its own psiEnd, and for a
        # wall_turn its own signed turn) at THIS command's vdes and standoff.
        # All three change on the line above or on the next goal, so the plan
        # is no longer a guess at the next solve's answer -- it is a guess at
        # a different problem's answer. See self.warm_start_z's own comment.
        self.warm_start_z = None
        self.get_logger().info(
            f'DRIVE/clear | drive command dropped; vdes -> {self.vdes:.2f} '
            f'avoidance_margin -> {self.avoidance_margin:.3f} (baselines restored)'
        )

    def goal_drive_callback(self, msg: DriveCommand):
        """f1tenth_behavior's PublishMoveGoal, once per mission "drive" step
        entry (schema_version 3.0). The FOURTH goal shape, and the only
        open-ended one.

        WHAT MAKES IT DIFFERENT IN KIND, not just in fields: goal_distance,
        goal_pose and goal_turn each name a target this node drives toward,
        decides it has REACHED, and then latches zero on while publishing
        /mpc/goal_reached. A drive command names only a MODE -- "hold this
        heading" or "turn by this much off it, then hold the result" -- so
        there is nothing to arrive at. This mode therefore NEVER sets
        goal_reached, NEVER publishes /mpc/goal_reached and NEVER
        self-terminates. The behaviour tree's stop_condition is the sole
        authority on when the move ends, and /mpc/hold remains the sole
        authority on stopping the car (control_loop's hold branch is
        untouched by any of this and keeps absolute priority).

        That is the whole reason this shape exists. f110_autonomy's planner --
        still ours -- has always emitted mode + guard with no goal, and the
        only way to land one of its phases in this schema used to be
        fabricating a goal_distance of 50 m and hoping the guard fired first.
        Nothing is fabricated any more.

        WHAT THIS RE-ANCHORS, and the two f110_autonomy bugs it fixes:
        psi_init_corridor is set to self.yaw HERE, per move. That stack's
        psi_init_corridor was captured ONCE at the first odom message after
        node start and never re-anchored, so every phase after the first
        turned relative to a heading the car had long since left; and its
        turn magnitude was a hardcoded pi/2 rather than read off the phase.
        Both are deliberate divergences, not oversights -- see
        build_straight_corridor's own drive branch, where the re-anchored
        value is consumed.

        Mirrors the other three callbacks exactly otherwise: clear the other
        modes' state, invalidate the per-move cached geometry, log what was
        accepted.
        """
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().warn(
                'goal_drive ricevuto ma stato ancora None: comando ignorato.'
            )
            return

        mode = str(msg.mode)
        if mode not in ('straight', 'wall_turn'):
            # mission_config.py validates this at load time, so anything
            # reaching here with a bad mode came from a bare `ros2 topic pub`
            # that bypassed the mission pipeline. Degrade to the safe shape
            # (hold heading) and say so, rather than either crashing the
            # callback or driving an undefined geometry.
            self.get_logger().warn(
                f'goal_drive: mode={mode!r} sconosciuto, uso "straight" come '
                'fallback (i modi validi sono "straight" e "wall_turn").'
            )
            mode = 'straight'

        # Restore any PREVIOUS drive command's overrides before applying this
        # one -- see _clear_drive_state's own docstring on why the baselines,
        # not the live values, are what get restored from.
        self._clear_drive_state()

        # SENTINEL: 0 means "use the node default" (see DriveCommand.msg).
        turn_mag_deg = float(msg.turn_mag_deg)
        if turn_mag_deg == 0.0:
            turn_mag_deg = self.drive_default_turn_mag_deg
        # abs(): turn_mag_deg is a MAGNITUDE and turn_sign carries the
        # direction. A negative magnitude paired with a negative sign would
        # otherwise silently turn the wrong way. mission_config.py already
        # rejects that combination at load time; this is the on-the-wire
        # backstop for the same reason the mode check above exists.
        turn_mag_deg = abs(turn_mag_deg)

        turn_sign = float(msg.turn_sign)
        if mode == 'wall_turn' and turn_sign == 0.0:
            self.get_logger().warn(
                'goal_drive: mode "wall_turn" senza turn_sign; nessuna direzione '
                'in cui girare, degrado a "straight".'
            )
            mode = 'straight'

        self.drive_cmd = {
            'mode': mode,
            'turn_sign': turn_sign,
            'turn_mag_deg': turn_mag_deg,
        }

        # Re-anchor "this move's own start heading" -- see the docstring's
        # own paragraph on the two f110_autonomy bugs this fixes.
        self.psi_init_corridor = self.yaw
        # ...and start this turn's rotation counter from zero at that same
        # heading. Ordered after self.drive_cmd is installed above so the
        # accumulator's own mode check sees the NEW command.
        self._reset_turn_progress()

        # ---- per-move overrides, both duration-scoped by _clear_drive_state
        # SENTINEL: speed == 0 means "use the node default".
        if msg.speed > 0.0:
            self.vdes = float(msg.speed)
        # SENTINEL: d_safe < 0 means "use the node default". 0.0 is a REAL
        # value (no standoff at all), which is why the sentinel is negative
        # and not zero.
        #
        # HOW A TOTAL STANDOFF BECOMES A MARGIN: this node spends the standoff
        # as car_radius + avoidance_margin -- it is that SUM that sets
        # compute_local_target's R_safe and mpc_solver's own obstacle trigger
        # and boundary rows. d_safe is the total, so the margin that realises
        # it is d_safe - car_radius, floored at zero (a d_safe inside the car's
        # own radius cannot be honoured by a margin at all, and clamping is
        # more honest than a negative margin the solver would read as a bonus).
        # car_radius itself is deliberately left alone: it is a physical fact
        # about the vehicle, not a tuning knob, and compute_predicted_clearance
        # also reads it.
        if msg.d_safe >= 0.0:
            requested = float(msg.d_safe)
            # RELAXATION ONLY, never a tightening. This field exists to let the
            # car close on the object it was told to stop in front of, so a
            # request ABOVE the node's own baseline standoff is refused rather
            # than honoured -- a mission must not be able to make obstacle
            # avoidance more aggressive than the tuned configuration through a
            # per-move field. This is f110_autonomy's own min(self.dmin, ...)
            # clamp, kept here where the real numbers live rather than
            # duplicated into whatever produced the request.
            baseline = self.car_radius + self.avoidance_margin_default
            effective = min(requested, baseline)
            self.avoidance_margin = max(effective - self.car_radius, 0.0)
            realised = self.car_radius + self.avoidance_margin
            notes = ''
            if effective < requested:
                notes = (f' -- CLAMPED al baseline {baseline:.3f} m: '
                         'approach_d_safe puo\' solo RILASSARE la distanza di '
                         'sicurezza, mai stringerla')
            elif realised > effective + 1e-9:
                notes = (f' -- CLAMPED al raggio del veicolo {self.car_radius:.3f} m: '
                         'un d_safe interno al raggio non e\' realizzabile')
            self.get_logger().info(
                f'DRIVE/d_safe | richiesto={requested:.3f} m -> avoidance_margin='
                f'{self.avoidance_margin:.3f} (standoff effettivo {realised:.3f} m, '
                f'car_radius={self.car_radius:.3f} invariato)' + notes
            )

        # Switching to drive mode -- clear every other mode's state so the four
        # stay mutually exclusive, same pattern the existing three already use
        # for each other.
        self.goal_distance = None
        self.goal_start_xy = None
        self.goal_anchor_map = None
        self.goal_anchor_odom = None
        self.goal_pose_xy = None
        self.goal_pose_yaw = None
        self.pose_goal_reached = False
        self.goal_reached = False
        self._no_goal_warned = False
        self._invalidate_move_state()

        self.get_logger().info(
            f'Nuovo goal_drive: mode={mode!r} turn_sign={turn_sign:+.1f} '
            f'turn_mag_deg={turn_mag_deg:.1f} vdes={self.vdes:.2f} '
            f'psi_init_corridor(re-anchored)={self.psi_init_corridor:+.4f} rad '
            '-- OPEN-ENDED: nessun goal_reached, termina solo la stop_condition '
            'del behaviour tree.'
        )

    def _accumulate_turn_progress(self):
        """Add this tick's yaw increment to the active wall_turn's total.

        Called once per control_loop tick, before the corridor is built, so
        the turn's remaining rotation is measured against a total that is
        current rather than up to one corridor_update_period stale.

        Each increment is wrapped, and that wrap is safe where the total's
        would not be: it spans ONE control period, so at any yaw rate this
        car can reach it is orders of magnitude below pi. Summing them
        recovers the unwrapped rotation the wrapped total cannot express --
        see self.turn_progress_rad's own comment for why that matters at
        exactly 180 degrees.

        A no-op outside wall_turn: nothing else asks how far round the car
        has come, and leaving the counter alone means a straight move cannot
        silently accumulate a total that a later turn would inherit.
        """
        if self.yaw is None:
            return
        drive_cmd = getattr(self, 'drive_cmd', None)
        if drive_cmd is None or drive_cmd.get('mode') != 'wall_turn':
            return
        yaw = float(self.yaw)
        if self._turn_progress_last_yaw is None:
            self._turn_progress_last_yaw = yaw
            return
        step = math.atan2(math.sin(yaw - self._turn_progress_last_yaw),
                          math.cos(yaw - self._turn_progress_last_yaw))
        self.turn_progress_rad += step
        self._turn_progress_last_yaw = yaw

    def _reset_turn_progress(self):
        """Start a fresh turn from zero rotation.

        Separate from _invalidate_move_state so goal_drive_callback can order
        it AFTER installing the new drive_cmd -- the counter is per turn, and
        a turn that inherited the previous one's total would believe it was
        already part-way round.
        """
        self.turn_progress_rad = 0.0
        self._turn_progress_last_yaw = float(self.yaw) if self.yaw is not None else None
        # The increment's commit latch and ratchet are per turn for the same
        # reason: a turn that inherited them would start already committed,
        # or held to the previous turn's end heading.
        self.wall_turn_committed = False
        self.wall_turn_commanded_rot = None
        # And the tracked wall: it was THAT turn's wall, gated against THAT
        # turn's commit heading. getattr for the duck-typed test stand-ins.
        tracker = getattr(self, 'wall_tracker', None)
        if tracker is not None:
            tracker.reset()

    def _fresh_front_distance(self):
        """The wall distance (get_wall_distance_topic) as a distance, or None when it is not one.

        None when no message has arrived, when the last one is older than
        corr_wall_turn_front_distance_max_age_sec, or when it is not positive
        or non-finite (-1.0 is "no reading" on both wall topics:
        /perception/virtual_wall with no valid wall, front_clearance_node with
        none or inside the ZED's minimum range; never a distance). wall_turn.py
        treats None as unknown rather than as a wall at zero.

        ALSO None AT OR ABOVE wall_clear_distance_m (2026-10-15). That value
        is the wall contract's CLEAR state -- "the sensors work and there is
        nothing in the way" -- and it is NOT a wall 50 m away. Letting it
        through would put 50 m into the corridor length, the distance cap and
        the solver's geometry, all of which would silently accept it: the
        turn would commit against a wall that is not there and the corridor
        would stretch to the horizon. Returning None routes it to the SAME
        "unknown" handling an absent reading already gets, which is exactly
        right here: with no wall ahead there is no distance cap to apply.
        Capping to a finite number instead would keep a phantom wall at the
        cap, which is the error this contract exists to remove.
        """
        stamp = self.front_distance_stamp_sec
        if stamp is None:
            return None
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age > self.corr_wall_turn_front_distance_max_age_sec:
            return None
        d_front = float(self.front_distance)
        # <= 0, not < 0: the wire contract (get_wall_distance_topic) is "<= 0
        # = no reading", for /perception/virtual_wall's -1.0 as for this one's.
        if not math.isfinite(d_front) or d_front <= 0.0:
            return None
        if d_front >= self.wall_clear_distance_m:
            return None          # CLEAR: no wall ahead, not a wall at 50 m
        return d_front

    def _wall_stop_d_rem(self):
        """Distance [m] still to give away to the mission's wall target, or None.

        d_pred - target, with d_pred the latency-compensated wall distance
        (stop_profile.predicted_distance): where the bumper will be by the time
        a command issued on this tick reaches the wheels.

        None whenever there is nothing legitimate to ramp on, and EVERY route
        to None matters:

          * no target -- this move does not stop on a wall;
          * the ramp is disabled;
          * the mission's front-wall lock holds and the locked wall is not the
            FRONT one (_ramp_wall_distance): the wall in the strip is not the
            one this move stops at;
          * _fresh_front_distance() says the reading is absent, stale, not
            positive (the -1.0 NO_DATA sentinel) or at/above
            wall_clear_distance_m, which is the contract's CLEAR state and not
            a wall 50 m away.

        The last one is the one worth being explicit about: an absent or
        "clear" reading must NOT become a distance of zero, which would brake
        the car to a stop on a dead sensor. The caller's answer to None is to
        leave the speed alone -- and what a dead wall publisher should do to a
        MOVE is not this node's call at all: check_stop_condition.py holds the
        car on /mpc/hold for exactly that, which this node already obeys with
        absolute priority several branches above the ramp.
        """
        if self.stop_ramp is None or self.stop_target_m is None:
            return None
        d_est = self._ramp_wall_distance()
        if d_est is None:
            return None
        return self.stop_ramp.predict(d_est, self.v) - self.stop_target_m

    def _ramp_wall_distance(self):
        """Return the wall distance the ramp aims at, or None (ramp_wall_source.py).

        _fresh_front_distance() itself with wall_lock_enabled false, and while
        no lock is in force (not locked yet, or the locked wall lost). While
        the mission's wall is locked: that wall's own value while it is FRONT,
        and None -- no wall ramp -- while another wall, or none, is. Every
        _fresh_front_distance rule above still applies to both values.
        """
        raw = self._fresh_front_distance()
        # getattr for the duck-typed test stand-ins, which predate the lock.
        source = getattr(self, 'ramp_wall_source', None)
        if source is None:
            return raw
        pick = source.select(raw, self.get_clock().now().nanoseconds * 1e-9)
        previous = self.ramp_wall_pick
        self.ramp_wall_pick = pick
        if (previous is None or pick.source != previous.source
                or pick.track_id != previous.track_id):
            # NOT throttled: this is already edge-triggered, at most one line
            # per debug payload, and a throttled call drops the transition for
            # good -- ramp_wall_pick has moved on, so it is never re-logged.
            # That lost the swerve's locked -> not_front whenever it came
            # within 2 s of a far wall's CLEAR flicker.
            self.get_logger().info(
                f'RAMP | aiming at {ramp_wall_source.describe(pick)}')
        return pick.distance

    def _wall_stop_tick(self):
        """Hand the wall stop over to a zero command, and report whether it has.

        Returns True once the ramp has latched this move's wall stop, having
        published the zero command itself -- the caller then returns without
        solving, exactly as the object stop's own latched branch does. A HARD
        zero, not a ramp through the solver: by this point the car is inside
        its own stopping distance and the only thing left to do is stop.

        LATCHED, and nothing but a new move clears it (_invalidate_move_state,
        stop_target_callback). The wall reading keeps moving during the stop --
        the car's own last centimetres, estimate jitter, the contract flicking
        to CLEAR -- and re-arming on any of that would restart a car that is
        already braking at the operating floor. object_approach.ObjectStopLatch
        documents the same decision for the same reason.

        The steering is HELD at the last published value rather than zeroed:
        the move is not over (the mission's own guard ends it, see
        check_stop_condition's arrival arm) and straightening the wheel mid-stop
        is a jerk nobody asked for. Same choice the object stop-and-wait makes.
        """
        if self.stop_ramp is None:
            return False
        if not self.stop_ramp_latch.latched:
            d_rem_wall = self._wall_stop_d_rem()
            if d_rem_wall is None:
                return False
            if not self.stop_ramp_latch.update(
                    self.stop_ramp.decide(d_rem_wall, self.vdes)):
                return False
            self.get_logger().info(
                f'RAMP | wall stop: d_rem={d_rem_wall:+.3f} m <= d_final='
                f'{self.stop_ramp.d_final:.3f} m at v={self.v:.3f} -- zero '
                f'command, coasting onto the {self.stop_target_m:.2f} m target')
        self.stop_ramp_mode = 'stop_latched'
        self.stop_ramp_d_rem = self.stop_ramp_latch.latched_d_rem
        self._publish_drive(0.0, self._last_published_steer)
        return True

    def _stop_ramp_limits(self, v_ref):
        """`self.limits`, with vMax cut to what the ramp is asking for.

        THE REFERENCE ALONE DOES NOT SLOW THE CAR, which is the one thing the
        nominal-model rig found that no amount of reading would have
        (test_stop_ramp_closed_loop.py). vdes enters the QP as a soft stage
        cost at w_v 4.2857; w_term is 9.0 on a terminal point that sits at the
        horizon's own reach, and getting there sooner is worth more to the
        solver than tracking the reference. Traced tick by tick on a 1.0 m/s
        approach, the solver asked for +0.2 m/s^2 the whole way down a
        reference falling from 1.0 to 0.4 -- the cap was inert and the car
        arrived at full speed.

        So the profile also becomes a HARD row: the solver's own speed box on
        every predicted stage (mpc_solver's `row_v`), which no cost can
        outvote. Capping along the horizon is what the ramp was asked for; the
        soft reference stays as well, so the predicted trajectory the steering
        and corridor costs are built on is the one the car will actually fly.

        FEASIBILITY IS WHY IT IS NOT SIMPLY vMax = v_ref. The box binds from
        stage 1, which is at most one step of a_min away from the live speed:
        demanding v_ref outright while the car is still above it by more than
        a_min * ts makes the QP infeasible, and an infeasible QP here is a
        worse failure than an inert cap. The box is therefore never asked for
        more than one step of a_dec of braking -- max(v_ref, v - a_dec * ts) --
        which is feasible by construction (a_dec 0.6 against |a_min| 2.0) and
        which is itself exactly the profile: each tick's box is one a_dec step
        below the last, until it reaches v_ref and stays there.

        Returns self.limits unchanged when the ramp is not binding, so a tick
        with no stopping point in range solves against exactly the box it
        always did.
        """
        if self.stop_ramp is None or v_ref is None:
            return self.limits
        v_max = float(self.limits['vMax'])
        if not (v_ref < v_max):
            return self.limits
        ts_pred = float(getattr(self, 'ts_pred', self.ts))
        floor = float(self.v) - self.stop_ramp.a_dec * ts_pred
        box = max(float(v_ref), floor)
        if box >= v_max:
            return self.limits
        limits = dict(self.limits)
        limits['vMax'] = box
        return limits

    def _apply_stop_ramp(self, vdes, d_rem_goal=None):
        """Return `vdes` capped by the ramp for the NEAREST stopping point.

        Two stopping points can be live at once -- "drive 6 m" and "stop 2 m
        from the wall" is a perfectly ordinary move -- so the binding one is
        simply the smaller d_rem. Taking the minimum is also what makes a
        wall appearing mid-move do the right thing without any mode switching:
        the moment the wall is closer than the distance left to travel, it
        takes over the profile.

        THIS ONLY CAPS. The three hand-overs to a zero command live with the
        three arrivals instead -- _wall_stop_tick for the wall, and the
        goal_reached / pose_goal_reached branches for the other two -- because
        each of them also has to PUBLISH something and return, which a function
        that exists to return a number must not be doing.

        A HARD SPEED BOX GOES WITH THIS CAP, and the cap alone is not enough:
        see _stop_ramp_limits, which is where the measurement that proves it
        is written down.
        """
        self.stop_ramp_mode = None
        self.stop_ramp_d_rem = None
        if self.stop_ramp is None:
            return vdes

        d_rem_wall = self._wall_stop_d_rem()
        candidates = [d for d in (d_rem_wall, d_rem_goal) if d is not None]
        if not candidates:
            if self.stop_target_m is not None:
                # A wall stop was asked for and there is no usable distance to
                # ramp on. Logged, not acted on -- see _wall_stop_d_rem.
                self.stop_ramp_mode = RAMP_UNKNOWN
                # Not while the locked wall is simply not the one in the strip:
                # that is the swerve case, expected, and _ramp_wall_distance
                # has logged it.
                pick = getattr(self, 'ramp_wall_pick', None)
                if pick is None or pick.source != ramp_wall_source.SOURCE_NOT_FRONT:
                    self.get_logger().warn(
                        f'RAMP | wall target {self.stop_target_m:.2f} m but the wall '
                        'distance is absent, stale or CLEAR: no ramp this tick',
                        throttle_duration_sec=2.0)
            return vdes

        d_rem = min(candidates)
        decision = self.stop_ramp.decide(d_rem, vdes)
        self.stop_ramp_mode = decision.mode
        self.stop_ramp_d_rem = decision.d_rem
        self.get_logger().info(
            f'RAMP | {decision.mode} d_rem={d_rem:+.3f} m '
            f'(wall={d_rem_wall}, goal={d_rem_goal}) '
            f'vdes {vdes:.3f} -> {decision.v_ref:.3f}',
            throttle_duration_sec=1.0)
        return decision.v_ref

    def _invalidate_move_state(self):
        """Drop every piece of per-move cached geometry.

        Also starts the reverse manoeuvre's attempt count afresh (a new move
        lifts a "stuck" give-up).

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

        WHAT IS DELIBERATELY NOT RESET: self.last_u. It is the input actually
        being held by the hardware right now, and it is what w_du_delta
        measures the next command against. Zeroing it on a goal boundary would
        command a steering snap to centre and charge the rate cost for a
        discontinuity the car never made. It is also the fallback the RTI
        linearization tiles across the horizon once self.warm_start_z is
        cleared below -- and unlike that stored plan it carries no stale
        corridor information at all, being a single number about the present.
        """
        if getattr(self, 'reverse_supervisor', None) is not None:
            self.reverse_supervisor.new_move()
        # The speed channel's disturbance belongs to the move it was learnt on.
        # getattr for the duck-typed stand-ins, which carry no channel.
        if getattr(self, 'speed_channel', None) is not None:
            self._speed_reset('goal_change')
        # Target smoothing / obstacle-deflection coast: previous move's state.
        self.smoothed_target = None
        self.last_deflection_vec = np.zeros(2)
        self.deflection_decay_remaining = 0
        # Object mode's per-move geometry. Cleared HERE rather than in each of
        # the other four callbacks, so that any new goal of any shape drops an
        # active object approach without those callbacks having to grow a line
        # each -- they already all call this. goal_object_callback's own
        # new-move path therefore seeds these AFTER calling us.
        #
        # goal_object_map_xy/move_id are deliberately NOT cleared here: they
        # are the move's identity and its input, not its geometry, and
        # goal_object_callback owns both. Clearing the id here would make a
        # refresh arriving between a rebuild and the next message look like a
        # new move and re-anchor -- the exact failure this mode exists to
        # avoid. The control loop gates on goal_object_odom_xy, which IS
        # cleared, so an object approach superseded by another goal stops
        # driving immediately.
        # An object move superseded by ANY new goal is over: remember its id
        # so a late refresh cannot restart it. getattr, because several test
        # stand-ins for this method carry no object state at all.
        if (getattr(self, 'object_psi_c', None) is not None
                and getattr(self, 'goal_object_move_id', None)):
            self._mark_object_move_ended(self.goal_object_move_id)
        # A new goal owns the steering from here; an unfinished ramp-out from
        # the previous object move must not keep overriding the stop paths.
        self.object_exit_ramping = False
        self.goal_object_odom_xy = None
        self.object_psi_c = None
        self.object_target_at_build = None
        self.object_target_held = False
        self.object_last_step = None
        self.object_last_flags = None
        self.object_behind_terminal = False
        self.object_behind_for_s = 0.0
        # The stop-on-arrival latch belongs to the move it tripped in. getattr
        # for the duck-typed stand-ins that carry no object state.
        latch = getattr(self, 'object_stop_latch', None)
        if latch is not None:
            latch.reset()
        self.object_speed_mode = None
        # The ramp's wall stop, for the same reason as the object latch above:
        # a new move re-arms it. The TARGET itself is not cleared here -- see
        # stop_target_callback for why that would lose a race it must not lose.
        ramp_latch = getattr(self, 'stop_ramp_latch', None)
        if ramp_latch is not None:
            ramp_latch.reset()
        self.stop_ramp_mode = None
        self.stop_ramp_d_rem = None
        # Corridor geometry: force a rebuild + marker publish on the next tick.
        self.cached_corridor = None
        # A new move's first corridor has nothing meaningful to be a step FROM
        # -- differencing it against the last move's would report the whole
        # re-anchoring as reference movement. Same reason the arc's hysteresis
        # latch starts cold.
        self._prev_corridor_ref = None
        self._object_arc_active = False
        self.last_corridor_time = None
        self.last_corridor_stamp = None
        self.cached_pref_nom = None
        # The RTI warm start, for the same reason the corridor cache is here
        # and on the same boundary: it is the previous MOVE's control plan,
        # and the new move's corridor has a different psiRef. Seeding the
        # first solve of a turn with the straight move's plan would linearize
        # the whole QP around a trajectory aimed at the old heading -- the
        # exact failure the corridor cache above was added to prevent, one
        # layer down. Unlike last_u (see this docstring's own paragraph on
        # why THAT is deliberately kept) this is not a physical quantity the
        # hardware is holding; it is a stale opinion, and dropping it costs
        # only the first tick's linearization quality.
        self.warm_start_z = None

    def _update_active_odom(self):
        """Seleziona la sorgente odom attiva (hardware ha sempre priorita' se fresca)."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        hw_age = (now_sec - self.hw_odom_last_time) if self.hw_odom_last_time is not None else math.inf
        sim_age = (now_sec - self.sim_odom_last_time) if self.sim_odom_last_time is not None else math.inf

        if hw_age < self.odom_stale_timeout_sec:
            source = 'hardware'
            self.x, self.y, self.yaw, self.v = self.hw_x, self.hw_y, self.hw_yaw, self.hw_v
            self.state_stamp_sec = self.hw_odom_stamp_sec
        elif sim_age < self.odom_stale_timeout_sec:
            source = 'sim'
            self.x, self.y, self.yaw, self.v = self.sim_x, self.sim_y, self.sim_yaw, self.sim_v
            self.state_stamp_sec = self.sim_odom_stamp_sec
        else:
            source = None
            self.x = self.y = self.yaw = self.v = None
            self.state_stamp_sec = None

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
        self.obstacles_live_time = self.get_clock().now().nanoseconds * 1e-9

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

    def unknown_speed_cap_callback(self, msg: Float32):
        self.unknown_speed_cap = float(msg.data)
        self.unknown_speed_cap_time = self.get_clock().now().nanoseconds * 1e-9

    def _solve_obstacles(self, now_sec, obstacles=None):
        """Return the obstacles this tick may use: fresh, the obstacles_max nearest.

        obstacles: the list to select from, self.obstacles_global_live by
        default (_live_obstacles passes it without the move's target, so the
        target never takes one of the obstacles_max places).
        """
        obstacles = self.obstacles_global_live if obstacles is None else obstacles
        age = (None if self.obstacles_live_time is None
               else now_sec - self.obstacles_live_time)
        selected = select_obstacles(obstacles, self.x, self.y, age,
                                    self.obstacles_max_age_s, self.obstacles_max)
        if obstacles and not selected:
            self.get_logger().warn(
                f'obstacle list from {self.obstacles_topic} is {age:.2f} s old '
                f'(> {self.obstacles_max_age_s:.2f} s): solving without it',
                throttle_duration_sec=5.0)
        return selected

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

    def wall_tracks_callback(self, msg: MultiWallTrackArray):
        """Cache one /perception/walls message as plain odom-frame views.

        Deliberately does NO geometry. The half-planes are built at solve time
        (_wall_boundaries) because which side of each wall the car is on is a
        question about the car's CURRENT position, and because the activity
        test needs the corridor, which does not exist yet here. The costmap
        path transforms on receipt instead, and it has to: its rows arrive
        base_link-relative, so the pose they were computed against is the only
        one that can be used.

        The coast clock is updated here rather than at solve time, and that is
        load-bearing: it has to see EVERY cycle's states to know when each
        track was last CONFIRMED, including the cycles whose tracks are all
        rejected and the cycles the control loop does not run on.
        """
        self.wall_tracks = [wb.WallTrackView.from_msg(t) for t in msg.tracks]
        self.wall_tracks_frame = str(msg.header.frame_id)
        self.wall_tracks_last_time = self.get_clock().now().nanoseconds * 1e-9
        self.wall_tracks_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if self.wall_coast_clock is not None:
            self.wall_coast_clock.update(self.wall_tracks, self.wall_tracks_stamp)

        self.get_logger().info(
            f'WALLS/in | n={len(self.wall_tracks)} frame={self.wall_tracks_frame}',
            throttle_duration_sec=2.0)

    def _wall_path_points(self, corridor: Optional[Dict]):
        """The (x, y) odom points a wall row's extent is tested against.

        The corridor's own centreline plus the car's current position, which is
        what "the predicted horizon (or the corridor)" means in practice here:
        the corridor is the reference the horizon is solved to track and it
        exists BEFORE the solve, whereas the predicted states are last tick's.
        The car's own position is included so a wall right beside a stationary
        car still produces a row before any corridor has been built.
        """
        pts = []
        if self.x is not None and self.y is not None:
            pts.append((float(self.x), float(self.y)))
        if corridor is not None:
            xc = np.asarray(corridor['xc'], dtype=float).ravel()
            yc = np.asarray(corridor['yc'], dtype=float).ravel()
            pts.extend(zip(xc.tolist(), yc.tolist()))
        return pts

    def _wall_boundaries(self, corridor: Optional[Dict]):
        """Wall rows for this cycle, rebuilt from scratch against the live pose.

        Returns the (normal, offset) list for solve_mpc_step. Also refreshes
        self.wall_rows/self.wall_rows_diag and publishes
        /mpc/wall_constraints_diag, so a withheld cycle is visible as an empty
        row list with a reason rather than as silence.
        """
        reason = wb.stale_reason(
            self.wall_tracks_frame, self._odom_frame_name(),
            (None if self.wall_tracks_last_time is None
             else self.get_clock().now().nanoseconds * 1e-9 - self.wall_tracks_last_time),
            self.wall_row_max_tf_age_s)
        if reason:
            self.wall_rows = []
            self.wall_rows_diag = []
            self._publish_wall_diag(reason)
            self.get_logger().warn(
                f'WALL ROWS WITHHELD -- {reason}', throttle_duration_sec=5.0)
            return []
        if self.x is None or self.y is None:
            self.wall_rows, self.wall_rows_diag = [], []
            self._publish_wall_diag('no odom pose yet')
            return []

        rows, diag = wb.build_wall_rows(
            tracks=self.wall_tracks,
            car_xy=(float(self.x), float(self.y)),
            path_xy=self._wall_path_points(corridor),
            stamp=self.wall_tracks_stamp,
            coast_clock=self.wall_coast_clock,
            identity=self.wall_row_identity,
            max_rows=self.max_wall_constraints,
            max_coast_s=self.wall_row_max_coast_s,
            coast_margin_m=self.wall_row_coast_margin_m,
            allow_stereo_only=self.wall_row_allow_stereo_only,
            end_margin_m=self.wall_row_end_margin_m,
            merge_angle_deg=self.wall_row_merge_angle_deg,
            merge_offset_m=self.wall_row_merge_offset_m)
        self.wall_rows, self.wall_rows_diag = rows, diag
        self._publish_wall_diag('')
        self.get_logger().info(
            f'WALL ROWS | n={len(rows)} ids={[r.row_id for r in rows]}',
            throttle_duration_sec=2.0)
        return [r.constraint for r in rows]

    def _odom_frame_name(self) -> str:
        """The frame self.x/y/yaw are expressed in, for the wall frame check.

        'odom' unconditionally: this node's state comes from get_odom_topic()
        (the local EKF's /odometry/filtered, or raw /odom), both of which are
        odom-frame by construction -- see _update_active_odom. A separate
        parameter would only create a second place for the answer to be wrong.
        """
        return 'odom'

    def _publish_wall_diag(self, withheld_reason: str):
        """One JSON object per cycle on /mpc/wall_constraints_diag."""
        if self.wall_diag_pub is None:
            return
        self.wall_diag_pub.publish(String(data=json.dumps({
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'wall_stamp': self.wall_tracks_stamp,
            'boundary_source': self.boundary_source,
            'withheld': withheld_reason,
            'n_tracks': len(self.wall_tracks),
            'n_rows': len(self.wall_rows),
            'max_wall_constraints': self.max_wall_constraints,
            'boundary_max_sources': self.boundary_max_sources,
            'slack_weight': self.wall_row_slack_weight,
            'slack_max_m': self.boundary_slack_max,
            # Metres of violation per QP slot, from the PREVIOUS solve -- one
            # cycle behind the rows beside it, because the rows have to exist
            # before the solve that prices them. Row ids are stable across
            # cycles by construction (RowIdentity), so a reader can still line
            # the two up; the name says which solve it is rather than letting
            # the offset be discovered later. mpc_solver returns these in slot
            # order and pad_boundary_constraints fills the slots in row order,
            # so slot j belongs to rows[j] while j < n_rows.
            'slack_m_last_solve': [round(v, 4) for v in self.last_boundary_slack],
            'rows': self.wall_rows_diag,
        }, sort_keys=True)))

    def _get_live_boundaries(
            self, corridor: Optional[Dict] = None) -> List[Tuple[float, float, float]]:
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

        Gated on self.use_hard_boundary_constraints, which is TRUE on the
        launch path (stack_params.yaml) -- see that attribute's own comment.
        False forces [] unconditionally, before even looking at staleness, so
        disabling the feature via launch arg can't be defeated by fresh data
        arriving."""
        if not self.use_hard_boundary_constraints:
            return []
        if self.boundary_source == 'none':
            # No rows at all: the A/B baseline. Distinct from
            # use_hard_boundary_constraints false only in intent -- that flag
            # is the older MATLAB-comparison switch and is documented as such,
            # so a run that wants "costmap rows off, everything else as
            # shipped" says so here instead of borrowing that one.
            return []
        if self.boundary_source == 'walls':
            return self._wall_boundaries(corridor)
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
        self.front_distance_stamp_sec = self.get_clock().now().nanoseconds * 1e-9

        # ---- DEBUG ----
        self.get_logger().info(
            f'FRONT | d={self.front_distance:.3f}',
            throttle_duration_sec=2.0
        )

    def stop_target_callback(self, msg):
        """The current move's wall-distance target [m], or negative for none.

        One message per move, from f1tenth_behavior's PublishMoveGoal. NEGATIVE
        IS THE SENTINEL, exactly as DriveCommand.d_safe spends it: a mission
        with no front_clearance stop publishes -1.0 and the ramp then has no
        wall stopping point to aim at. Zero is NOT the sentinel, because a
        target of zero is a real (if unwise) request.

        DELIBERATELY NOT CLEARED BY _invalidate_move_state, unlike every other
        per-move input here. The goal shape and this value are published by the
        same behaviour on the same tick but on two different topics, so their
        arrival order is not guaranteed; clearing on the goal would drop the
        target whenever it lost the race. The BT publishes one of these for
        EVERY move instead -- the -1.0 for the moves that do not stop on a
        wall is what clears it -- so the last message is always the current
        move's answer. f1tenth_behavior's test_stop_target_published pins that
        it is published unconditionally, because this contract rests on it.
        """
        value = float(msg.data)
        self.stop_target_m = value if value >= 0.0 else None
        if self.stop_ramp_latch is not None:
            # A new target is a new move: re-arm the wall stop.
            self.stop_ramp_latch.reset()
        if getattr(self, 'ramp_wall_source', None) is not None:
            # ...and forget the last move's locked wall: until the BT's first
            # /mpc/stop_wall_id of this move the ramp reads the raw wall topic.
            self.ramp_wall_source.reset()
        self.get_logger().info(
            f'STOP TARGET | wall stop at {self.stop_target_m} m'
            if self.stop_target_m is not None else
            'STOP TARGET | cleared: this move does not stop on a wall')

    def stop_wall_id_callback(self, msg):
        """Keep the wall tracker id the mission's wall stop is locked on, or -1.

        f1tenth_behavior's CheckStopCondition, every tick it runs the
        front-wall lock (wall_lock.ramp_wall_id). Read by _ramp_wall_distance.
        """
        self.ramp_wall_source.on_lock_id(
            int(msg.data), self.get_clock().now().nanoseconds * 1e-9)

    def virtual_wall_debug_callback(self, msg):
        self.ramp_wall_source.on_debug(
            msg.data, self.get_clock().now().nanoseconds * 1e-9)

    def d_wall_correction_callback(self, msg):
        self.d_wall_correction = float(msg.data)
        self.d_wall_correction_stamp_sec = self.get_clock().now().nanoseconds * 1e-9

    def _fresh_d_wall_correction(self) -> float:
        """The d_wall heading correction [rad] to add to psi_base, or 0.0.

        ZERO, NEVER THE LAST KNOWN VALUE, in every degraded case: disabled, no
        message yet, stale, or non-finite. wall_distance_node already snaps its
        own output to zero when it loses the track; this is the second half of
        the same rule, for when the node itself goes away. The natural
        implementation holds the last value and that is the bug -- a held
        correction is a confident heading toward a position nothing can see any
        more. Zero returns this branch to exactly the geometry it had before
        wall_distance_node existed.
        """
        if not self.corr_d_wall_correction_enable:
            return 0.0
        stamp = self.d_wall_correction_stamp_sec
        if stamp is None:
            return 0.0
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age > self.corr_d_wall_max_age_sec:
            self.get_logger().warn(
                f'DWALL | correction {age:.2f} s old (> '
                f'{self.corr_d_wall_max_age_sec:.2f} s): treating it as absent. '
                'Is wall_distance_node running?',
                throttle_duration_sec=5.0)
            return 0.0
        correction = float(self.d_wall_correction)
        if not math.isfinite(correction):
            return 0.0
        return correction

    # =========================
    # Wall tracker glue (wall_tracker.py holds the logic)
    # =========================

    def _latest_odom_pose(self) -> Optional[Tuple[float, float, float]]:
        """The most recent odometry pose, hardware first, mirroring
        _update_active_odom's preference without its side effects."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (self.hw_odom_last_time is not None
                and now_sec - self.hw_odom_last_time < self.odom_stale_timeout_sec):
            return (self.hw_x, self.hw_y, self.hw_yaw)
        if (self.sim_odom_last_time is not None
                and now_sec - self.sim_odom_last_time < self.odom_stale_timeout_sec):
            return (self.sim_x, self.sim_y, self.sim_yaw)
        return None

    def scan_callback(self, msg: LaserScan):
        pose = self._latest_odom_pose()
        if pose is None:
            # A scan with no pose to place it cannot be used later either.
            return
        self.scan_msg = msg
        self.scan_pose = pose
        self.scan_last_time = self.get_clock().now().nanoseconds * 1e-9

    def _laser_pose_for(self, frame_id: str) -> Optional[Tuple[float, float, float]]:
        """(x, y, yaw) of the scan frame in base_link, from the static edge
        on /tf_static, cached on first success. None until it is known."""
        if self._laser_pose is not None and self._laser_pose[0] == frame_id:
            return self._laser_pose[1]
        try:
            stamped = self.tf_buffer.lookup_transform('base_link', frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'WALL/scan | no base_link <- {frame_id} transform yet ({exc}); '
                'the wall tracker cannot place returns until it arrives',
                throttle_duration_sec=5.0)
            return None
        t = stamped.transform
        pose = (t.translation.x, t.translation.y, self.quaternion_to_yaw(t.rotation))
        self._laser_pose = (frame_id, pose)
        self.get_logger().info(
            f'WALL/scan | base_link <- {frame_id}: x={pose[0]:.3f} y={pose[1]:.3f} '
            f'yaw={math.degrees(pose[2]):.2f} deg')
        return pose

    def _scan_points_odom(self, now_sec: float) -> Optional[np.ndarray]:
        """The latest scan's returns in the odom frame, or None when there is
        no scan, it is older than odom_stale_timeout_sec (the same staleness
        bound every other live input here uses), or the laser pose is
        unknown."""
        if self.scan_msg is None or self.scan_last_time is None:
            return None
        if now_sec - self.scan_last_time > self.odom_stale_timeout_sec:
            return None
        msg = self.scan_msg
        laser_pose = self._laser_pose_for(msg.header.frame_id)
        if laser_pose is None:
            return None
        return scan_to_odom_points(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            laser_pose, self.scan_pose)

    def _select_tracked_wall(self, car_pose, now_sec: float):
        """One selection attempt against the frozen commit references. Logs
        the accepted candidate, or every rejected one with its angle error --
        the record wall_normal_tol_rad gets tuned against."""
        tracker = self.wall_tracker
        points = self._scan_points_odom(now_sec)
        if points is None:
            self.get_logger().warn(
                'WALL/select | no fresh /scan (or no laser pose) on this rebuild; '
                'the increment stays on dFront and selection is retried next rebuild')
            return
        selection = tracker.select(points, car_pose, now_sec)
        window_txt = ('none (dFront unknown at commit)' if selection.window_m is None
                      else f'[{selection.window_m[0]:.2f}, {selection.window_m[1]:.2f}]')
        accepted = selection.accepted
        for cand in selection.candidates:
            if cand is accepted:
                verdict = 'ACCEPTED'
            elif cand.reason == 'accepted':
                verdict = 'passed, not best angle'
            else:
                verdict = f'rejected {cand.reason}'
            self.get_logger().info(
                f'WALL/select | {verdict}: d={cand.distance_m:.2f} '
                f'angle_err={math.degrees(cand.angle_err_rad):.1f} deg '
                f'inliers={cand.inlier_count} span={cand.span_m:.2f} '
                f'rms={cand.rms_m * 1e3:.1f} mm ahead={cand.ahead}')
        if accepted is None:
            self.get_logger().warn(
                f'WALL/select | no candidate passed the gates ({len(selection.candidates)} '
                f'extracted, {selection.n_points} returns ahead, window {window_txt}, '
                f'psi_commit={tracker.psi_commit:+.4f}); the increment stays on dFront')
            return
        self.get_logger().info(
            f'WALL/select | tracking d_wall={accepted.distance_m:.3f} '
            f'normal_yaw={accepted.line.normal_yaw:+.4f} psi_commit={tracker.psi_commit:+.4f} '
            f'window {window_txt} ({len(selection.candidates)} candidates)')

    def _wall_track_tick(self):
        """Every control tick of a wall_turn: refit the tracked wall from the
        latest scan (dead-reckon when there is none, or the refit degrades)
        and publish d_wall from the live pose; valid=false until a wall is
        selected. No-op outside a wall_turn.

        PER TICK, NOT PER REBUILD, ON PURPOSE. Replayed against the archive
        with a refit only on the 1 s rebuild cadence, d_wall ran a 0.15 m
        sawtooth: dead reckoning between refits inherits the odometry
        scale bias (~20% short, see lidar_front_wall_node.py), and each
        refit snapped it back. A scan arrives at 40 Hz and the refit is
        association plus three small SVDs, so every tick can afford one;
        the rebuild then reads a fit at most one tick old."""
        if self.wall_track_pub is None or self.x is None:
            return
        drive_cmd = getattr(self, 'drive_cmd', None)
        if drive_cmd is None or drive_cmd.get('mode') != 'wall_turn':
            return
        tracker = self.wall_tracker
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        car_pose = (self.x, self.y, self.yaw)
        if tracker.has_wall:
            step = tracker.update(self._scan_points_odom(now_sec), car_pose, now_sec)
            if step.provenance != WallTrack.PROVENANCE_MEASURED:
                self.get_logger().warn(
                    f'WALL/track | fallback: dead-reckoning d_wall={step.d_wall:.3f} '
                    f'({step.reason}, associated={step.inlier_count}, '
                    f'{step.since_last_fit_sec:.1f} s since the last fit)',
                    throttle_duration_sec=2.0)
        msg = WallTrack()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.valid = tracker.has_wall
        msg.provenance = tracker.provenance
        msg.psi_commit = float(tracker.psi_commit) if tracker.armed else math.nan
        msg.inlier_count = int(tracker.last_inlier_count)
        msg.span_m = float(tracker.last_span_m)
        if tracker.has_wall:
            msg.d_wall = float(tracker.d_wall(car_pose))
            msg.normal_yaw = float(tracker.line.normal_yaw)
            msg.since_last_fit_sec = float(now_sec - tracker.last_fit_time)
        else:
            msg.d_wall = math.nan
            msg.normal_yaw = math.nan
            msg.since_last_fit_sec = math.nan
        self.wall_track_pub.publish(msg)

    def _init_reverse_manoeuvre(self):
        """
        Declare the reverse manoeuvre's parameters and wire its topics.

        /mpc/reverse_status (ReverseStatus, every tick), /mpc/request_reverse
        (RequestReverse), and the inputs: /bt/allow_reverse (Bool, must be
        fresh), /mission/status (transient local), the rear costmap
        (rear_costmap_topic, transient local like costmap_boundary_node's).
        """
        def _p(key):
            return self.declare_parameter(key, get_value(key)).value

        hw = float(_p('rear_corridor_half_width_m'))
        cfg = ManoeuvreConfig(
            auto_enabled=bool(_p('reverse_auto_enabled')),
            solver_mode=str(_p('rear_solver_mode')),
            corridor_length_m=float(_p('rear_corridor_length_m')),
            corridor_half_width_m=(hw if hw > 0.0 else float(self.corr_wmin)),
            v_rev_mps=float(_p('v_rev_mps')),
            rear_dt_s=float(_p('mpc_rear.prediction_dt_s')),
            stuck_time_s=float(_p('stuck_time_s')),
            stuck_speed_mps=float(_p('stuck_speed_mps')),
            stuck_front_dist_m=float(_p('stuck_front_dist_m')),
            stuck_progress_m=float(_p('stuck_progress_m')),
            gear_change_dwell_s=float(_p('gear_change_dwell_s')),
            forward_min_time_s=float(_p('forward_min_time_s')),
            max_attempts=int(_p('reverse_max_attempts')),
            progress_reset_m=float(_p('reverse_progress_reset_m')),
            rear_stop_margin_m=float(_p('rear_stop_margin_m')),
            brake_delay_s=float(_p('rear_brake_delay_s')),
            brake_decel_mps2=float(_p('rear_brake_decel_mps2')),
            rear_check_margin_m=float(_p('rear_check_margin_m')),
            bt_allow_max_age_s=float(_p('bt_allow_reverse_max_age_s')),
            solver_budget_s=float(_p('rear_solver_budget_s')),
            timeout_factor=float(_p('reverse_timeout_factor')),
            car_rear_offset_m=float(_p('car_rear_offset_m')),
            car_half_width_m=float(get_value('swept_clearance_body_half_width_m')),
            costmap_free_max=int(_p('rear_costmap_free_max')),
            max_reverse_speed_mps=float(self.max_reverse_speed))
        rear_weights = {k: float(_p(f'mpc_rear.{k}')) for k in self.weights}
        costmap_topic = str(_p('rear_costmap_topic'))
        # One reverse mechanism at a time: both on refuses to start.
        try:
            validate_reverse_mechanisms(self._allow_reverse_param, cfg.auto_enabled)
        except ValueError as exc:
            self.get_logger().fatal(f'REVERSE | STARTUP REFUSED: {exc}')
            raise
        # /mpc/reverse_status is the BT's ONLY view of the MPC's direction: with
        # bt_reverse_integration, a status that is missing or stale counts as
        # forward and the front stops keep stopping in EVERY direction
        # (reverse_integration.py D1), which zeroes a reverse on the safety lane.
        # So the publisher is created whichever mechanism is enabled; the
        # manoeuvre fills it from its supervisor, allow_reverse from the
        # direction selector (_publish_reverse_status_from_planner).
        self.reverse_status_pub = self.create_publisher(ReverseStatus, '/mpc/reverse_status', 10)
        if self.allow_reverse:
            self.get_logger().info(
                'REVERSE/manoeuvre | not built: allow_reverse is on (the forward MPC '
                'chooses reverse itself) and reverse_auto_enabled is off; '
                '/mpc/reverse_status is published from the direction selector')
            return
        if self.max_reverse_speed < cfg.v_rev_mps:
            self.get_logger().warn(
                f'REVERSE/manoeuvre | max_reverse_speed_mps {self.max_reverse_speed:.2f} < '
                f'v_rev_mps {cfg.v_rev_mps:.2f}: the rear side will report '
                '"reverse_speed_limit" and no reverse will start')
        rear = RearSolver(
            cfg, rear_weights, self.limits, self.params,
            rear_probe_offsets(self.params['L'], base_front_x(), -cfg.car_rear_offset_m,
                               SOLVER_FRONT_POINT_OFFSETS_M),
            boundary_opts={'boundary_max_sources': self.boundary_max_sources,
                           'boundary_hard': self.boundary_hard,
                           'boundary_slack_weight': self.boundary_slack_weight})
        self.reverse_supervisor = ReverseSupervisor(
            cfg, rear, self.ts, car_radius=self.car_radius,
            avoidance_margin=self.avoidance_margin)
        self._bt_allow = None
        self._bt_allow_t = None
        self._mission_state = None
        self._emergency_stop = False
        self._rear_grid = None
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Bool, '/bt/allow_reverse', self._bt_allow_cb, 10)
        self.create_subscription(MissionStatus, '/mission/status', self._mission_status_cb,
                                 latched)
        self.create_subscription(OccupancyGrid, costmap_topic, self._rear_costmap_cb, latched)
        self.create_service(RequestReverse, '/mpc/request_reverse', self._request_reverse_cb)
        self.get_logger().info(
            f'REVERSE/manoeuvre | auto={cfg.auto_enabled} solver={cfg.solver_mode} '
            f'L={cfg.corridor_length_m:.2f} m v_rev={cfg.v_rev_mps:.2f} m/s '
            f'N_rear={rear.n} costmap={costmap_topic}')

    def _init_speed_channel(self):
        """
        Declare the speed channel's parameters and wire its two topics.

        /ackermann_drive (the mux output: what the VESC was actually sent) is
        read so the model starts from the command in force and the estimator
        freezes while another lane drives; /mpc/speed_channel
        (SpeedChannelStatus) goes out on every /drive publish.

        OFFSET-FREE NEEDS THE RTI SOLVER AND THE SINGLE-CANDIDATE PATH. The
        SLSQP rollback has no speed model, and allow_reverse's direction
        candidates build their sign schedules on the integrator model (their
        feasibility witness is a jerk-limited stop of v itself). Either one
        drops the channel to legacy, loudly: rolling back one of them must
        not need a second edit.
        """
        def _p(key):
            return self.declare_parameter(key, get_value(key)).value

        cfg = speed_config_from(_p)
        if cfg.mode == SPEED_MODE_OFFSET_FREE and (self.allow_reverse or not self.use_rti_solver):
            why = 'allow_reverse is on' if self.allow_reverse else 'use_rti_solver is false'
            self.get_logger().warn(
                f'SPEED | speed_command_mode offset_free needs the forward RTI solve and '
                f'{why}: running LEGACY (v_meas + a * ts)')
            cfg.mode = SPEED_MODE_LEGACY
        self.speed_channel = SpeedChannel(
            cfg, self.ts, self.ts_pred, self.N, self.max_forward_speed,
            self.max_reverse_speed, self.min_moving_speed)
        self._speed_prev_source = 'forward'
        self.create_subscription(
            AckermannDriveStamped, '/ackermann_drive', self._ackermann_drive_cb, 10)
        self.speed_channel_pub = self.create_publisher(
            SpeedChannelStatus, '/mpc/speed_channel', 10)
        ch = self.speed_channel
        self.get_logger().info(
            f'SPEED | mode={cfg.mode} tau={cfg.tau_s:.3f} s (alpha {ch.alpha:.3f}) '
            f'delay={cfg.delay_ticks} tick(s) observer L_v={ch.l_v:.3f} L_d={ch.l_d:.3f} '
            f'(bw {cfg.obs_bw_v_rad_s:.1f}/{cfg.obs_bw_d_rad_s:.1f} rad/s) '
            f'|d|<={cfg.d_max_mps:.2f} ceiling={ch.ceiling:.2f} m/s '
            f'breakaway={cfg.breakaway_mps:.2f} m/s floor={cfg.moving_floor}')

    def _ackermann_drive_cb(self, msg: AckermannDriveStamped):
        self.speed_channel.note_applied(
            float(msg.drive.speed), self.get_clock().now().nanoseconds * 1e-9)

    def _speed_offset_free(self) -> bool:
        """Whether this tick's forward solve uses the offset-free speed model."""
        ch = getattr(self, 'speed_channel', None)
        return ch is not None and ch.offset_free

    def _speed_floor_applies(self) -> bool:
        """
        Whether the operating floor bounds this solve's command (offset_free_moving_floor).

        'object' is the legacy rule exactly -- an object approach while it
        drives, the one place floor_moving_speed was ever applied -- moved from
        a post-solve max into the QP's command box. 'all' extends "never
        below min_moving_speed_mps while moving, zero only to stop" to every
        forward solve (the stop paths still publish zero).
        """
        mode = self.speed_channel.cfg.moving_floor
        if mode == FLOOR_ALL:
            return True
        if mode == FLOOR_OBJECT:
            return (self.goal_object_odom_xy is not None
                    and self.object_speed_mode == SPEED_DRIVE)
        return False

    def _speed_observe(self, now_sec):
        """Advance the speed estimator over the step that just ended (every tick, first)."""
        ch = getattr(self, 'speed_channel', None)
        if ch is None:
            return
        stamp = getattr(self, 'state_stamp_sec', None)
        recv = getattr(self, 'hw_odom_last_time', None)
        if getattr(self, 'active_odom_source', None) == 'sim':
            recv = getattr(self, 'sim_odom_last_time', None)
        fresh = (self.v is not None and stamp is not None
                 and stamp != getattr(self, '_speed_last_stamp', None)
                 and (recv is None or now_sec - recv <= ch.cfg.odom_max_age_s))
        self._speed_last_stamp = stamp
        state = getattr(self, '_mission_state', None)
        sup = getattr(self, 'reverse_supervisor', None)
        reverse = (self._reverse_manoeuvre_active()
                   or (sup is not None and sup.mode != MODE_FORWARD))
        stopping = bool(self.goal_reached or self.pose_goal_reached
                        or (self.stop_ramp_latch is not None and self.stop_ramp_latch.latched))
        ch.observe(now_sec, self.v, fresh, hold=bool(self.hold),
                   estop=bool(getattr(self, '_emergency_stop', False)),
                   mission_active=(state is None or state == 'RUNNING'),
                   stopping=stopping, reverse=reverse)

    def _speed_reset(self, reason):
        """Reset the speed channel (goal change, hold, e-stop, gear change)."""
        ch = getattr(self, 'speed_channel', None)
        if ch is None:
            return
        ch.reset(reason, self.v)
        self.get_logger().info(
            f'SPEED/reset | {reason}: d -> 0, command in force {ch.command_in_force:+.3f}')

    def _publish_speed_status(self, stamp):
        ch = getattr(self, 'speed_channel', None)
        pub = getattr(self, 'speed_channel_pub', None)
        if ch is None or pub is None:
            return
        st = ch.status()
        msg = SpeedChannelStatus()
        msg.header.stamp = stamp
        msg.mode = st['mode']
        for key in ('v_meas', 'v_hat', 'd_hat', 'innovation', 'v_cmd', 'v_cmd_in_force',
                    'v_cmd_applied', 'stall_cap', 'v_cmd_ceiling', 'speed_slack', 'tau_s',
                    'alpha', 'gain_l_v', 'gain_l_d'):
            setattr(msg, key, float(st[key]))
        for key in ('override', 'frozen', 'stalled', 'breakaway_active'):
            setattr(msg, key, bool(st[key]))
        msg.freeze_reasons = st['freeze_reasons']
        msg.last_reset = st['last_reset']
        msg.reset_count = int(st['reset_count'])
        pub.publish(msg)

    def _init_semantic_memory(self):
        """Declare the semantic-memory consumer's parameters and subscribe."""
        def _p(key):
            return self.declare_parameter(key, get_value(key)).value

        self.semantic_memory_enabled = bool(_p('mpc_semantic_memory_enabled'))
        self.semantic_memory_max_age_s = float(_p('mpc_semantic_memory_max_age_s'))
        self.semantic_memory_min_confidence = float(_p('mpc_semantic_memory_min_confidence'))
        self._semantic_class_margins = get_value('obstacle_class_margin_m')
        # obstacle_projector_node's gate, the same keys and the same function,
        # so a remembered object passes exactly the test a live one did.
        self._obstacle_filter = ObstacleFilter(
            float(_p('obstacle_z_min')), float(_p('obstacle_z_max')),
            float(_p('min_obstacle_radius')), float(_p('max_obstacle_radius')))
        self.target_exclusion_gate_m = float(_p('mpc_target_exclusion_gate_m'))
        self._semantic_msg = None
        self._semantic_t = None
        if self.semantic_memory_enabled:
            self.create_subscription(SemanticObjectArray, '/costmap/semantic_objects',
                                     self._semantic_objects_cb, 10)
            if not self.use_map_frame_goal_anchor:
                # _lookup_map_odom() returns None while the anchor is off, so
                # no map-frame object could ever be placed: say so once.
                self.get_logger().warn(
                    'SEMANTIC/memory | inactive: use_map_frame_goal_anchor is false '
                    f'(localization_source={get_value("localization_source")!r}), so '
                    'there is no map -> odom edge to place /costmap/semantic_objects; '
                    'no remembered object reaches the obstacle list or the rear check, '
                    'and the reverse manoeuvre cannot place its rear costmap either')

    def _semantic_objects_cb(self, msg: SemanticObjectArray):
        self._semantic_msg = msg
        self._semantic_t = self.get_clock().now().nanoseconds * 1e-9

    def _move_target(self) -> Optional[MoveTarget]:
        """Return the active object move's target, or None outside an object approach."""
        if (getattr(self, 'object_psi_c', None) is None
                or getattr(self, 'goal_object_map_xy', None) is None):
            return None
        return MoveTarget(
            track_id=str(getattr(self, 'goal_object_track_id', '') or ''),
            class_id=str(getattr(self, 'goal_object_target_class', '') or ''),
            map_xy=tuple(self.goal_object_map_xy),
            odom_xy=getattr(self, 'goal_object_odom_xy', None))

    def _live_obstacles(self) -> List[Tuple[float, float, float]]:
        """Return the live obstacle list's circles (odom): fresh, capped, without the target.

        The list is mpc_obstacles_topic's (the semantic layer's
        /costmap/semantic_obstacles since obstacles v2): the recognised
        objects the camera sees now and, in 'active' mode, the LiDAR's
        unknown ones. The remembered (MEMORY) ones are not on it: see
        _planning_obstacles.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        return self._solve_obstacles(now, without_target_circles(
            self.obstacles_global_live, self._move_target(), self.target_exclusion_gate_m))

    def _planning_obstacles(self) -> List[Tuple[float, float, float]]:
        """
        Return this tick's obstacle list: live circles plus remembered (MEMORY) objects.

        What the solver's w_obs, compute_local_target and the predicted
        clearance read. The move's target is in neither part.
        """
        live = self._live_obstacles()
        if not self.semantic_memory_enabled:
            return live
        memory = self._semantic_circles(memory_only=True)
        return list(live) + memory if memory else live

    def _semantic_circles(self, memory_only: bool) -> List[Tuple[float, float, float]]:
        """
        Return the semantic layer's objects as odom (x, y, r) circles, or [].

        memory_only keeps the MEMORY ones (the obstacle list's addition);
        False keeps every object (the rear pre-check). Every object first
        passes obstacle_projector_node's gate on its height and base radius
        (passes_obstacle_filter), and the move's target is never one of them.
        Nothing when the layer is off, its message older than
        mpc_semantic_memory_max_age_s, or no map -> odom edge exists to place
        map-frame objects.
        """
        if (not self.semantic_memory_enabled or self._semantic_msg is None
                or self._semantic_t is None):
            return []
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._semantic_t > self.semantic_memory_max_age_s:
            return []
        map_odom = self._lookup_map_odom()
        if map_odom is None:
            return []
        objs = [o for o in self._semantic_msg.objects
                if (not memory_only or int(o.state) == SEMANTIC_MEMORY)
                and passes_obstacle_filter(o.z_m, o.radius_m, self._obstacle_filter)]
        objs = without_target_objects(objs, self._move_target(), self.target_exclusion_gate_m)
        return objects_to_odom_circles(
            objs, map_odom, include_memory=True,
            min_confidence=self.semantic_memory_min_confidence,
            class_margins=self._semantic_class_margins)

    def _bt_allow_cb(self, msg: Bool):
        self._bt_allow = bool(msg.data)
        self._bt_allow_t = self.get_clock().now().nanoseconds * 1e-9

    def _mission_status_cb(self, msg: MissionStatus):
        self._mission_state = str(msg.state)
        estop = bool(msg.emergency_stop_active)
        if (estop != bool(getattr(self, '_emergency_stop', False))
                and getattr(self, 'speed_channel', None) is not None):
            self._speed_reset('estop' if estop else 'estop_release')
        self._emergency_stop = estop
        if self._emergency_stop and self.reverse_supervisor is not None:
            self.reverse_supervisor.abort('emergency_stop')

    def _rear_costmap_cb(self, msg: OccupancyGrid):
        info = msg.info
        q = info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._rear_grid = Grid(
            data=np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width),
            resolution=float(info.resolution), origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y), origin_yaw=float(yaw))

    def _reverse_env(self, live_boundaries):
        """
        Build this tick's RearEnv: the costmap only with a map -> odom edge to place it.

        objects: the live circles and EVERY semantic object (TRACKED and
        MEMORY), the move's target in neither.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        map_odom = self._lookup_map_odom() if self._rear_grid is not None else None
        return RearEnv(
            grid=self._rear_grid if map_odom is not None else None, map_to_odom=map_odom,
            objects=list(self._live_obstacles()) + self._semantic_circles(memory_only=False),
            boundaries=list(live_boundaries),
            bt_allow=self._bt_allow,
            bt_allow_age_s=(math.inf if self._bt_allow_t is None else now - self._bt_allow_t),
            abort=bool(self.hold or self._emergency_stop))

    def _reverse_front_distance(self, live_boundaries):
        """Return the free distance ahead of the front bumper (obstacles, wall topic, rows).

        The move's target is not an obstacle here either: a car stopped at
        its standoff from it is not stuck on it.
        """
        cands = []
        d_obs = self.compute_forward_obstacle_distance(self._live_obstacles())
        if d_obs < 1e5:
            cands.append(d_obs)
        d_wall = self._fresh_front_distance()
        if d_wall is not None:
            cands.append(d_wall)
        bumper = self._front_bumper_xy()
        if bumper is not None:
            hx, hy = math.cos(self.yaw), math.sin(self.yaw)
            for nx_, ny_, off in live_boundaries:
                if math.isfinite(off) and nx_ * hx + ny_ * hy > 0.7:
                    cands.append(max(off - (nx_ * bumper[0] + ny_ * bumper[1]), 0.0))
        return min(cands) if cands else None

    def _reverse_intentional_stop(self):
        """Tell whether this is a stop the mission asked for (stuck timer held at 0)."""
        if self._mission_state is not None and self._mission_state != 'RUNNING':
            return True
        return bool(self.hold or self.pose_goal_reached or self.goal_reached)

    def _request_reverse_cb(self, request, response):
        sup = self.reverse_supervisor
        if self.x is None or self.yaw is None or self.v is None:
            response.accepted, response.reason = False, 'no_state'
            return response
        env = self._reverse_env(self._get_live_boundaries())
        now = self.get_clock().now().nanoseconds * 1e-9
        response.accepted, response.reason = sup.request(
            now, np.array([self.x, self.y, self.yaw, self.v]), float(request.distance_m), env)
        self.get_logger().info(
            f'REVERSE/request | {request.distance_m:.2f} m -> '
            f'{"accepted" if response.accepted else "refused"}: {response.reason}')
        return response

    def _publish_reverse_status(self):
        st = self.reverse_supervisor.status()
        msg = ReverseStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = st['mode']
        msg.reverse_active = bool(st['reverse_active'])
        msg.stuck = bool(st['stuck'])
        msg.stuck_reason = st['stuck_reason']
        msg.stuck_timer_s = float(st['stuck_timer_s'])
        msg.intentional_stop = bool(st['intentional_stop'])
        msg.rear_ready = bool(st['rear_ready'])
        msg.rear_reason = st['rear_reason']
        msg.rear_solver_running = bool(st['rear_solver_running'])
        msg.rear_solve_ms = float(st['rear_solve_ms']) if math.isfinite(st['rear_solve_ms']) \
            else math.nan
        msg.rear_clearance_m = float(st['rear_clearance_m']) \
            if math.isfinite(st['rear_clearance_m']) else math.nan
        msg.bt_allow_reverse = bool(st['bt_allow_reverse'])
        msg.bt_allow_fresh = bool(st['bt_allow_fresh'])
        msg.attempts = int(min(st['attempts'], 255))
        msg.last_switch_time_s = float(st['last_switch_time_s'])
        msg.last_termination_reason = st['last_termination_reason']
        msg.requested_m = float(st['requested_m'])
        msg.travelled_m = float(st['travelled_m'])
        self.reverse_status_pub.publish(msg)

    def _publish_reverse_status_from_planner(self):
        """Publish /mpc/reverse_status for the allow_reverse path (no manoeuvre).

        The BT needs two fields (reverse_integration.py D1/D3): reverse_active,
        to stop the front cone from zeroing a reverse on the safety lane, and
        mode, for the blackboard mirror. Here they come from the direction
        selector, not the manoeuvre's supervisor:

          direction REVERSE, |v| still forward  -> "stopping", active
          direction REVERSE, otherwise          -> "reverse",  active
          direction FORWARD                     -> "forward",  not active

        "stuck" is never reported: that mode is the manoeuvre's attempt counter
        (D3's stuck_after_reverse outcome), which this path has no equivalent of.
        The manoeuvre-only fields are left at their neutral values.
        """
        if self.reverse_status_pub is None or self.reverse_planner is None:
            return
        direction = self.reverse_planner.selector.direction
        stop_speed = float(self.reverse_planner.cfg.stop_speed_mps)
        msg = ReverseStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        if direction == REVERSE_DIR:
            msg.mode = (MODE_STOPPING if float(self.v) > stop_speed else MODE_REVERSE)
            msg.reverse_active = True
        else:
            msg.mode = MODE_FORWARD
            msg.reverse_active = False
        msg.stuck = False
        msg.stuck_reason = ''
        msg.stuck_timer_s = 0.0
        msg.intentional_stop = False
        msg.rear_ready = False
        msg.rear_reason = 'allow_reverse'
        msg.rear_solver_running = False
        msg.rear_solve_ms = math.nan
        msg.rear_clearance_m = math.nan
        # Always False here: /bt/allow_reverse is subscribed on the manoeuvre
        # path only (that mechanism gates on it), so this path has no reading to
        # report and must not touch _bt_allow, which is not set on it either.
        msg.bt_allow_reverse = False
        msg.bt_allow_fresh = False
        msg.attempts = 0
        msg.last_switch_time_s = 0.0
        msg.last_termination_reason = ''
        msg.requested_m = 0.0
        msg.travelled_m = 0.0
        self.reverse_status_pub.publish(msg)

    def _reverse_config(self):
        """
        Build the reverse planner's config from the reverse_* parameters.

        v_rev_max is capped at max_reverse_speed_mps: the /drive clamp is the
        one authority on reverse speed, and a planner that may choose reverse
        the clamp then zeroes would sit still in a reverse plan forever. With
        the shipped clamp (0.0) that cap disables the feature, loudly.
        """
        def _p(key):
            return self.declare_parameter(key, get_value(key)).value

        v_rev_max = float(_p('reverse_v_rev_max_mps'))
        if self.allow_reverse and self.max_reverse_speed < v_rev_max:
            self.get_logger().error(
                f'REVERSE | allow_reverse is true but max_reverse_speed_mps '
                f'{self.max_reverse_speed:.2f} < reverse_v_rev_max_mps {v_rev_max:.2f}: '
                + ('reverse DISABLED (the /drive clamp would zero every reverse '
                   'command)' if self.max_reverse_speed <= 0.0 else
                   f'reverse capped at {self.max_reverse_speed:.2f} m/s'))
        v_rev_max = min(v_rev_max, self.max_reverse_speed)
        front_x = base_front_x()
        rear_x = -float(get_value('car_rear_offset_m'))
        return ReverseConfig(
            allow_reverse=v_rev_max > 0.0,
            switch_times_s=tuple(float(t) for t in _p('reverse_switch_times_s')),
            v_rev_max=v_rev_max,
            w_rev=float(_p('reverse_w_rev')),
            switch_threshold=float(_p('reverse_switch_threshold')),
            switch_threshold_rel=float(_p('reverse_switch_threshold_rel')),
            commit_break_margin=float(_p('reverse_commit_break_margin')),
            min_dwell_s=float(_p('reverse_min_dwell_s')),
            stop_speed_mps=float(_p('reverse_stop_speed_mps')),
            corr_back_length_m=float(_p('reverse_corr_back_length_m')),
            rear_offsets_m=rear_probe_offsets(
                self.params['L'], front_x, rear_x, SOLVER_FRONT_POINT_OFFSETS_M),
            w_ctg=float(_p('reverse_w_ctg')),
            ctg_follow_m=float(_p('reverse_ctg_follow_m')),
            ctg_h_xy=float(_p('reverse_ctg_h_xy')),
            ctg_h_psi=float(_p('reverse_ctg_h_psi')),
        )

    def _reverse_cost_to_go(self):
        """
        Build the terminal cost-to-go (cost_to_go.py) from the reverse_ctg_* parameters.

        Geometry from the same sources the rest of the node uses: wheelbase
        and steering limits from the MPC's own params, the front bumper from
        the URDF, the rear bumper and body half-width from stack_params.
        """
        def _p(key):
            return self.declare_parameter(key, get_value(key)).value

        lattice = LatticeConfig(
            window_m=float(_p('reverse_ctg_window_m')),
            resolution_m=float(_p('reverse_ctg_resolution_m')),
            n_theta=int(_p('reverse_ctg_n_theta')),
            primitive_length_m=float(_p('reverse_ctg_primitive_length_m')),
            k_rev=float(_p('reverse_ctg_k_rev')),
            gear_switch_cost=float(_p('reverse_ctg_gear_switch_cost')),
            steer_cost_per_m=float(_p('reverse_ctg_steer_cost_per_m')),
            unreachable_cost=float(_p('reverse_ctg_unreachable_cost')),
            goal_tolerance_m=float(_p('reverse_ctg_goal_tolerance_m')),
            footprint_margin_m=float(_p('reverse_ctg_footprint_margin_m')),
            front_x=base_front_x(),
            rear_x=-float(get_value('car_rear_offset_m')),
            half_width=float(get_value('swept_clearance_body_half_width_m')),
            wheelbase=self.params['L'], lr=self.params['lr'],
            steer_min=self.limits['delta_min'], steer_max=self.limits['delta_max'])
        self.reverse_ctg_use_corridor = bool(_p('reverse_ctg_use_corridor_bounds'))
        return CostToGo(
            lattice, wall_change_m=float(_p('reverse_ctg_wall_change_m')),
            min_rebuild_period_s=float(_p('reverse_ctg_min_rebuild_period_s')))

    def _update_cost_to_go(self, corridor, obstacles_global, live_boundaries, now_sec):
        """
        Keep the cost-to-go on this tick's goal and walls (allow_reverse only).

        A goal POSE exists on goal_pose / turn moves (position-only arrival)
        and on object moves (the standoff point); goal_distance and drive
        moves have none, and the candidates then run without it. The walls are
        the live boundary half-planes the QP's boundary rows use, the
        obstacles this tick's list.
        """
        ctg = self.reverse_planner.ctg
        if self.goal_object_odom_xy is not None and corridor.get('objectGoal') is not None:
            goal = (corridor['objectGoal'][0], corridor['objectGoal'][1], None)
        elif self.goal_distance is None and self.goal_pose_xy is not None:
            goal = (self.goal_pose_xy[0], self.goal_pose_xy[1], None)
        else:
            ctg.clear()
            return
        rebuilt = ctg.update(
            now_sec, (self.x, self.y), goal, halfplanes=live_boundaries,
            circles=obstacles_global,
            corridor=(extend_corridor_backwards(
                corridor, self.reverse_planner.cfg.corr_back_length_m)
                if self.reverse_ctg_use_corridor else None))
        if rebuilt:
            f = ctg.field
            self.get_logger().info(
                f'REVERSE/ctg | rebuilt ({ctg.last_rebuild_reason}) in {f.build_ms:.1f} ms: '
                f'goal=({goal[0]:+.2f},{goal[1]:+.2f}) edges={f.n_edges} '
                f'goal_states={f.n_goal_states}')

    def _mpc_solve(self, **solve_kwargs):
        """
        Solve the tick: today's single forward solve unless allow_reverse.

        With allow_reverse false this IS solve_mpc_step(**solve_kwargs) --
        same call, same arguments -- which is what keeps the flag-off
        behaviour bit-identical (pinned by test_reverse_flag_off_identity.py
        on the existing closed-loop scenarios).
        """
        if not getattr(self, 'allow_reverse', False) or self.reverse_planner is None:
            return solve_mpc_step(**solve_kwargs)
        return self.reverse_planner.step(
            now_sec=self.get_clock().now().nanoseconds * 1e-9, **solve_kwargs)

    def _publish_reverse_debug(self, info):
        """Per-candidate scores, winner and reason on /mpc/reverse/debug (JSON)."""
        dbg = info.get('reverse')
        if dbg is None or self.reverse_debug_pub is None:
            return
        self.reverse_debug_pub.publish(String(data=json.dumps(dbg)))
        self.get_logger().info(
            f'REVERSE | dir={dbg["direction"]} winner={dbg["winner"]} '
            f'reason={dbg["reason"]} committed={dbg["committed"]} '
            + ' '.join(f'{c["name"]}={c["score"] if c["score"] is not None else "x"}'
                       for c in dbg['candidates'])
            + f' tick={dbg["tick_ms"]:.1f}ms',
            throttle_duration_sec=0.5)

    def _reverse_manoeuvre_active(self) -> bool:
        """Report whether the committed reverse manoeuvre is driving right now.

        The one condition under which a negative speed may leave this node. It
        reads the supervisor's own reverse_active (true while stopping toward a
        reverse and while reversing), so the gate below and /mpc/reverse_status
        can never disagree about what the car is doing.
        """
        sup = getattr(self, 'reverse_supervisor', None)
        if sup is None:
            return False
        try:
            return bool(sup.status()['reverse_active'])
        except Exception:  # noqa: BLE001 -- a status we cannot read is not a reverse
            return False

    def _publish_drive(self, speed, steering_angle, source='forward'):
        requested = speed
        # FINAL GATE: a negative speed leaves this node only while the reverse
        # manoeuvre is driving. Anything else that asks to go backwards -- a
        # forward solve undershooting through zero, a stale command, a future
        # caller -- is published as a stop, never as a reverse, and counted.
        if (speed is not None and math.isfinite(speed) and speed < 0.0
                and not self._reverse_manoeuvre_active()):
            self.reverse_suppressed += 1
            self.get_logger().warn(
                f'DRIVE/no-reverse | {speed:+.3f} m/s requested outside a reverse '
                f'manoeuvre -> 0.000 (suppressed {self.reverse_suppressed})',
                throttle_duration_sec=1.0)
            speed = 0.0
        speed, clamped = clamp_drive_speed(
            speed, self.max_forward_speed, self.max_reverse_speed)
        stamp = self.get_clock().now().to_msg()
        # THE SPEED CHANNEL sees every command exactly as it left: it is the
        # command in force the next solve starts from and the input of the
        # estimator. A tick that published without planning ends the start-up
        # intent; a switch into or out of the reverse manoeuvre resets d.
        ch = getattr(self, 'speed_channel', None)
        if ch is not None:
            if not getattr(self, '_speed_tick_planned', False):
                ch.no_plan()
            prev = getattr(self, '_speed_prev_source', 'forward')
            if (source == 'manoeuvre') != (prev == 'manoeuvre'):
                self._speed_reset('reverse_switch')
            self._speed_prev_source = source
            ch.note_published(speed)
            self._speed_tick_planned = False
        if requested is not None and math.isfinite(requested) and requested < 0.0 \
                and speed == 0.0 and not clamped:
            # Report the suppression on the same topic the clamp uses: from a
            # consumer's point of view the command was limited, and applied 0.0
            # against a negative request says which limit did it.
            event = DriveClamp()
            event.header.stamp = stamp
            event.requested_speed = float(requested)
            event.applied_speed = 0.0
            event.max_forward_speed = float(self.max_forward_speed)
            event.max_reverse_speed = float(self.max_reverse_speed)
            self.drive_clamp_pub.publish(event)
        if clamped:
            event = DriveClamp()
            event.header.stamp = stamp
            event.requested_speed = float(requested) if math.isfinite(requested) else math.nan
            event.applied_speed = float(speed)
            event.max_forward_speed = float(self.max_forward_speed)
            event.max_reverse_speed = float(self.max_reverse_speed)
            self.drive_clamp_pub.publish(event)
            self.get_logger().warn(
                f'DRIVE/clamp | requested {requested:+.3f} m/s -> {speed:+.3f} '
                f'(limits +{self.max_forward_speed:.2f}/-{self.max_reverse_speed:.2f})',
                throttle_duration_sec=1.0)
        self._last_published_steer = float(steering_angle)
        msg = AckermannDriveStamped()
        msg.header.stamp = stamp
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.pub.publish(msg)
        if ch is not None:
            self._publish_speed_status(stamp)

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
        self._log_ctrl(source, requested, speed, steering_angle)

    def _log_ctrl(self, source, requested, applied, steer):
        """
        Log one CTRL line for the command that was ACTUALLY published.

        Emitted from _publish_drive, the single point every tick passes
        through, so the reverse ticks appear too. The old CTRL line sat in
        control_loop after the forward solve: it missed every before_forward
        manoeuvre tick (which returns before reaching it) and, on an
        after_forward tick, printed the forward solver's speed rather than the
        manoeuvre command that actually went out. That is why the 2026-10-20
        run could not say what speed the three reverses commanded.

        source is 'forward', 'manoeuvre', or 'clamp-0' when the final gate
        turned a negative request into a stop.
        """
        if requested is not None and math.isfinite(requested) and requested < 0.0 \
                and applied == 0.0:
            source = 'clamp-0'
        v_real = float(self.v) if self.v is not None else math.nan
        rev = 'mode=- reason=- travelled=-'
        sup = getattr(self, 'reverse_supervisor', None)
        if sup is not None:
            try:
                st = sup.status()
                rev = (f'mode={st["mode"]} reason={st["last_termination_reason"] or "-"} '
                       f'travelled={st["travelled_m"]:.3f}/{st["requested_m"]:.3f} '
                       f'rear={st["rear_reason"] or "ok"}')
            except Exception:  # noqa: BLE001 -- logging must never break the loop
                rev = 'mode=? reason=? travelled=?'
        self.get_logger().info(
            f'CTRL | src={source} v_pub={applied:+.3f} req={requested:+.3f} '
            f'steer={steer:+.4f} v_real={v_real:+.4f} {rev} '
            f'supp={getattr(self, "reverse_suppressed", 0)}')

    def _write_model_log(self, x0, delta_cmd, a_cmd):
        """Write one model-validation row. Never raises into the control loop."""
        log = self.model_log
        if not log.enabled or self.state_stamp_sec is None:
            return
        try:
            log.write(self.state_stamp_sec - self.model_log_t0,
                      x0[0], x0[1], x0[2], x0[3], delta_cmd, a_cmd)
        except Exception as exc:  # noqa: BLE001 -- logging must not stop control
            log.close()
            self.get_logger().error(f'model log DISABLED after an unexpected error: {exc}')
            return
        if not log.enabled:
            self.get_logger().warn(f'model log DISABLED after a failed write: {log.error}')
        elif log.skipped != self._model_log_reported_skips:
            self._model_log_reported_skips = log.skipped
            self.get_logger().warn(
                f'model log: {log.skipped} step(s) skipped so far because they reused the '
                'previous odometry message (dt = 0 in the analysis)',
                throttle_duration_sec=5.0)

    # ==========================================
    # LOOP CONTROLLO
    # ==========================================
    def control_loop(self):
        loop_t0 = self.get_clock().now().nanoseconds * 1e-9

        # THE TICK BOUNDARY for LLM2's parameter updates: a parked update is
        # applied here, before anything in this tick reads the weights, never
        # in the middle of a solve. No-op unless llm2_params_enabled.
        self._llm2_apply_pending()

        self._update_active_odom()
        # The speed estimator advances over the step that just ended BEFORE
        # anything can return early: a hold or a stop tick still has a
        # measurement and a command that acted, and must freeze or re-sync on
        # them rather than skip them.
        self._speed_tick_planned = False
        self._speed_observe(loop_t0)

        # Diagnostic, independent of autonomy/goal state -- no consumer yet, but
        # cheap and useful to have live every tick regardless of what else is gating.
        solve_obstacles = self._solve_obstacles(loop_t0)
        d_robot_obs = self.compute_robot_obstacle_distance(solve_obstacles)
        self.min_obstacle_distance_pub.publish(Float32(data=float(d_robot_obs)))
        # Forward-half-plane counterpart, published every tick beside the
        # omnidirectional one -- see min_obstacle_distance_forward_pub's own
        # comment for why this is a second topic and not a change to the
        # first. Published from the same obstacle list in the same breath, so
        # the two can never disagree about which frame they describe.
        d_front_obs = self.compute_forward_obstacle_distance(solve_obstacles)
        self.min_obstacle_distance_forward_pub.publish(Float32(data=float(d_front_obs)))

        if self.x is None or self.y is None or self.yaw is None or self.v is None:
            self.get_logger().warn('ODOM non disponibile: stato ancora None')
            # Hard zero, NOT the ramp: the ramp assumes a node with a live
            # state, which is exactly what this branch has lost.
            self.object_exit_ramping = False
            self.object_exit_ramp.seed(0.0)
            self._publish_drive(0.0, 0.0)
            return

        if self.hold:
            # Deliberately touches nothing else -- goal_start_xy, goal_reached, and
            # self.last_u are all left exactly as they were, so releasing the hold
            # resumes the current move rather than restarting or skipping it.
            # Object moves are the exception: hold_callback ENDS them.
            self._publish_stop()
            return

        # THE DECELERATION RAMP's hand-over to a zero command for the mission's
        # wall target. Before the mode chain, because a wall stop belongs to
        # the move whatever shape its goal has -- a drive, a goal_distance or a
        # goal_pose move can all carry a front_clearance stop. After the hold,
        # because a hold outranks everything and already publishes zero. No-op
        # in object mode: goal_object_odom_xy moves never publish a
        # /mpc/stop_target, so there is no target here to latch on.
        if self._wall_stop_tick():
            return

        # ONE map -> odom reprojection per tick, shared by the goal_distance
        # progress check below and by build_straight_corridor's own
        # goal_distance branch further down -- see _refresh_goal_anchor's own
        # docstring for why they must not look this up independently. No-op in
        # pose/turn mode and whenever no map anchor was captured.
        self._refresh_goal_anchor()

        # Same one-lookup-per-tick discipline for the object target: it is
        # read by the corridor build and by the branch below, and two
        # independent lookups would disagree by however much map -> odom moved
        # in between. No-op in every other mode.
        self._refresh_object_target()

        # How far round the active wall_turn has come, updated once per tick
        # so build_straight_corridor's wall_turn branch can subtract it from
        # the commanded total. No-op in every other mode.
        self._accumulate_turn_progress()
        # Refit the tracked wall from the latest scan and publish d_wall,
        # every tick of a wall_turn (no-op otherwise), BEFORE the corridor
        # rebuild below reads it.
        self._wall_track_tick()

        # The deceleration ramp's own-goal stopping point, filled in by
        # whichever of the two branches below owns one (pose: the distance to
        # the goal less its arrival tolerance; distance mode: the distance
        # still to travel). None in every other mode, which leaves the ramp
        # with only the mission's wall target to aim at. A LOCAL, not instance
        # state: it is derived fresh every tick from quantities those branches
        # compute anyway, and a stale copy of it would cap the speed for a
        # stopping point the move no longer has.
        d_rem_goal = None

        if self.drive_cmd is not None:
            # DRIVE MODE -- open-ended, and the ONLY branch here with no
            # termination check of its own. Read this before adding one.
            #
            # There is deliberately nothing to check: a drive command carries
            # a MODE, not a target, so there is no arrival condition, no
            # goal_reached to set, and no /mpc/goal_reached to publish. The
            # move ends when f1tenth_behavior's own stop_condition fires and
            # that tree publishes either the next move's goal or /mpc/hold.
            # Giving this node a second opinion about when a drive move is
            # done would put two authorities on "stop", which is precisely the
            # failure mode this whole goal shape exists to remove -- and is
            # what f110_autonomy's stop_at/stop_at_distance did (an if/elif/
            # ELSE that latched vdes = 0 and never evaluated its guard again).
            #
            # So this branch simply falls THROUGH to the corridor build and
            # the solve below, exactly as the goal_pose branch does when it
            # has not yet arrived. build_straight_corridor's own drive branch
            # is where the mode actually becomes geometry.
            #
            # /mpc/hold still has absolute priority: its branch returned
            # several lines above this one, untouched by any of this.
            self.get_logger().info(
                f'DRIVE | mode={self.drive_cmd["mode"]} '
                f'turn_sign={self.drive_cmd["turn_sign"]:+.1f} '
                f'turn_mag_deg={self.drive_cmd["turn_mag_deg"]:.1f} '
                '(open-ended, nessun goal_reached)',
                throttle_duration_sec=2.0
            )

        elif self.goal_object_odom_xy is not None:
            # OBJECT MODE -- drive at a tracked object, stopping a standoff
            # short of it. Falls through to the corridor build and the solve,
            # like the drive branch above.
            #
            # First, the refresh watchdog: a sender that stopped republishing
            # is a HARD stop, not a ramp -- see object_guard. Object mode is
            # kept, so the approach resumes if refreshes return.
            watchdog_now = self.get_clock().now().nanoseconds * 1e-9
            watchdog_reason = self.object_goal_watchdog.tripped(watchdog_now)
            if watchdog_reason is not None:
                if not self.object_goal_watchdog_tripped:
                    self.get_logger().error(
                        f'OBJECT/watchdog | {watchdog_reason}: hard stop, '
                        f'move_id={self.goal_object_move_id!r}')
                self.object_goal_watchdog_tripped = True
                self.object_exit_ramping = False
                self.object_exit_ramp.seed(0.0)
                self._publish_drive(0.0, 0.0)
                # Live geometry, not an empty status: r defaults to 0.0 on the
                # wire, which a reach check would read as arrival.
                flags = self._assess_object_tick(
                    (self.x, self.y, self.yaw), watchdog_now)
                self._publish_object_status(self.object_last_step, flags, 0.0)
                return
            if self.object_goal_watchdog_tripped:
                self.get_logger().info('OBJECT/watchdog | refreshes resumed')
                self.object_goal_watchdog_tripped = False

            #
            # SPEED: the move's speed until the stop latches, then zero. No ramp.
            # The car is not operated below min_moving_speed_mps while it moves
            # (the operating floor), so the braking parabola this branch used to
            # follow -- whose last 0.27 m sat between 0 and 0.4 m/s -- is gone.
            # See object_approach.ObjectStopLatch and object_speed_decision.
            #
            # Still NOT a pose_goal_tolerance-style arrival latch with
            # /mpc/goal_reached: the move ends when the mission's object_reached
            # fires (on live r, or on this latch once the car is at rest), which
            # keeps one authority on when the move is over.
            r_live = self._object_range()
            was_latched = self.object_stop_latch.latched
            self.object_stop_latch.update(r_live)
            if self.object_stop_latch.latched and not was_latched:
                self.get_logger().info(
                    f'OBJECT/stop | latched at r={self.object_stop_latch.latched_r:+.3f} '
                    f'(trigger {self.object_stop_latch.trigger_r:.3f} = reach tol + '
                    f'stop distance), v={self.v:.3f}: zero speed for the rest of '
                    f'move_id={self.goal_object_move_id!r}')
            decision = object_speed_decision(
                self.goal_object_speed, self.object_stop_latch.latched,
                self.min_moving_speed)
            self.object_speed_mode = decision.mode
            if decision.mode != SPEED_DRIVE:
                if decision.mode == SPEED_BELOW_FLOOR:
                    self.get_logger().error(
                        f'OBJECT/speed | ObjectGoal speed {self.goal_object_speed:.3f} '
                        f'is below the operating floor min_moving_speed_mps '
                        f'{self.min_moving_speed:.2f}: holding the car stopped rather '
                        'than driving it that slowly', throttle_duration_sec=2.0)
                # STOP-AND-WAIT: zero speed, steering HELD at what was last sent
                # (not ramped out -- the move is not over), no solve. Status is
                # still published every tick: object_reached needs the latch,
                # the measured speed and a fresh r to judge arrival.
                now_stop = self.get_clock().now().nanoseconds * 1e-9
                flags = self._assess_object_tick((self.x, self.y, self.yaw), now_stop)
                self._publish_drive(0.0, self._last_published_steer)
                self._publish_object_status(self.object_last_step, flags, 0.0)
                return
            self._object_vdes = decision.speed_ref
            self.get_logger().info(
                f'OBJECT | target_odom=('
                f'{self.goal_object_odom_xy[0]:+.3f},'
                f'{self.goal_object_odom_xy[1]:+.3f}) '
                f'r={r_live:+.3f} standoff='
                f'{self.goal_object_standoff:.2f} vdes={self._object_vdes:.3f}'
                + (' HELD' if self.object_target_held else ''),
                throttle_duration_sec=1.0)

        elif self.goal_pose_xy is not None:
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

            # THE RAMP's stopping point for this mode is the POSE ITSELF, not
            # the edge of the arrival tolerance. Aiming at the tolerance circle
            # would hand over to the zero command d_final OUTSIDE it and leave
            # the car resting short of the very circle the move is judged by;
            # aiming at the pose puts the hand-over inside it (0.133 m against
            # the 0.15 m tolerance) and the car comes to rest about 4 cm short
            # of the pose. The arrival check below therefore still fires, and
            # still owns the arrival -- the ramp only sets the speed it is
            # reached at. Its own stop is the backstop underneath, for a
            # tolerance TIGHTER than d_final, where the check below could
            # otherwise never fire at all.
            d_rem_goal = dist_to_goal

            if dist_to_goal <= self.pose_goal_tolerance:
                self.pose_goal_reached = True
                self._publish_drive(0.0, 0.0)
                self.goal_reached_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f'Goal pose raggiunto: dist_to_goal={dist_to_goal:.3f} m <= '
                    f'tolerance={self.pose_goal_tolerance:.3f} m'
                )
                return

            if (self.stop_ramp is not None
                    and self.stop_ramp.decide(d_rem_goal, self.vdes).stopping):
                # Inside the ramp's hand-over distance with a tolerance too
                # tight for the check above: the car cannot stop any closer, so
                # this IS the arrival. Reported the same way, on the same topic.
                self.pose_goal_reached = True
                self._publish_drive(0.0, 0.0)
                self.goal_reached_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f'Goal pose raggiunto (ramp): dist_to_goal={dist_to_goal:.3f} m '
                    f'<= d_final={self.stop_ramp.d_final:.3f} m, tolerance '
                    f'{self.pose_goal_tolerance:.3f} m is tighter than the stop'
                )
                return
            # altrimenti: prosegui verso la normale risoluzione MPC qui sotto
            # (build corridoio -> solver -> publish drive), come in modalita' distanza

        elif self.goal_distance is None or self.goal_start_xy is None:
            self._publish_stop()
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

            # THE RAMP's stopping point for this mode: the distance still to
            # make good along the move's own reference line. Checked BEFORE the
            # arrival below, because it fires d_final earlier -- that is the
            # whole point. The car is commanded to zero one stopping distance
            # short of the goal and coasts onto it, instead of being commanded
            # to zero AT the goal and coasting a whole stopping distance past
            # it (0.97 m at 1.5 m/s, and the error grew with the speed).
            d_rem_goal = self.goal_distance - traveled
            if (self.stop_ramp is not None
                    and self.stop_ramp.decide(d_rem_goal, self.vdes).stopping):
                self.goal_reached = True
                self._publish_drive(0.0, 0.0)
                self.goal_reached_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f'Goal raggiunto (ramp): traveled={traveled:.3f} m, residuo '
                    f'{d_rem_goal:+.3f} m <= d_final={self.stop_ramp.d_final:.3f} m '
                    f'-- comando a zero, la macchina rotola sul goal'
                )
                return

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

        # Live (the semantic layer's list, fresh and capped: obstacles v2) +
        # remembered (Part B), the object move's target in neither (see
        # _planning_obstacles). d_robot_obs / d_front_obs above stay on
        # solve_obstacles, target included: they are reported distances
        # (mission stop conditions read them), not avoidance.
        obstacles_global = self._planning_obstacles()
        # CONFIRMED 'other' wall segments, as soft w_obs disks, in
        # boundary_source 'walls' only. A NEW list -- never a mutation of
        # self.obstacles_global_live, which also feeds
        # compute_robot_obstacle_distance -> /mpc/min_obstacle_distance ->
        # the mission obstacle_distance_below stop condition. A wall must not
        # become a reason to stop a mission; it is a reason to steer.
        if self.boundary_source == 'walls' and self.other_walls_as_obstacles:
            wall_disks = wb.other_wall_obstacles(
                self.wall_tracks, self.wall_obstacle_spacing_m,
                self.wall_obstacle_radius_m)
            if wall_disks:
                obstacles_global = list(obstacles_global) + wall_disks
                self.get_logger().info(
                    f'WALL OBS | {len(wall_disks)} disks from other-role walls',
                    throttle_duration_sec=2.0)

        self.get_logger().info(f'OBSTACLES WORLD={obstacles_global}')
        self.get_logger().info(f'DIST ROBOT-OSTACOLO = {d_robot_obs:.3f}')

        x0 = np.array([self.x, self.y, self.yaw, self.v], dtype=float)

        vdes = self.vdes
        if self.goal_object_odom_xy is not None:
            # Object mode's ramped approach speed, computed in its branch
            # above. Applied here rather than there because `vdes` is
            # (re)assigned from self.vdes on the line above, after the branch
            # chain has run -- see that branch's own note.
            #
            # self.vdes itself is deliberately NOT written: it is the node's
            # standing speed and _clear_drive_state restores it from a
            # baseline, so ramping it would leave the last approach's speed in
            # force for the next move.
            vdes = self._object_vdes
        else:
            # THE DECELERATION RAMP's cap on the speed reference the solver
            # tracks across the whole horizon -- for the mission's wall target
            # and for this node's own two goal shapes, never for object mode
            # (which has its own, see __init__). Same reason the object branch
            # writes the local and not self.vdes: this is a per-tick cap, and
            # writing it back would leave the last approach's ramp in force.
            vdes = self._apply_stop_ramp(vdes, d_rem_goal)
        # The unknown-object speed cap (semantic layer, obstacles v2), last,
        # so it also bounds an object approach's ramped speed and the ramp.
        cap_age = (None if self.unknown_speed_cap_time is None
                   else loop_t0 - self.unknown_speed_cap_time)
        vdes = apply_speed_cap(vdes, self.unknown_speed_cap, cap_age,
                               self.unknown_speed_cap_max_age_s)

        now_time = self.get_clock().now()
        now_sec = now_time.nanoseconds * 1e-9

        need_update = False
        if self.cached_corridor is None or self.last_corridor_time is None:
            need_update = True
        elif (now_sec - self.last_corridor_time) >= self.corridor_update_period:
            need_update = True
        elif (self.goal_object_odom_xy is not None
                and self._object_target_moved_since_build()):
            # OBJECT MODE ONLY: the target itself moved far enough to be worth
            # a rebuild before the period is up. This is the one place the
            # period is legitimately pre-empted, and it is pre-empted by real
            # target MOTION rather than by message arrival -- see
            # _object_target_moved_since_build. Every other mode's corridor
            # describes geometry that does not move, so none of them needs it.
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
            self._publish_corridor_polygon(self.cached_corridor)

        if self.goal_object_odom_xy is not None:
            # EVERY TICK, not only on a rebuild. k and dpsi_max are the last
            # rebuild's (psi_c moves only there); r, bearing, e, the flags,
            # target_age_s and target_stale are this tick's -- the range and
            # the terminal flag are what the mission ends the move on.
            flags = self._assess_object_tick(x0, now_sec)
            self._publish_object_status(self.object_last_step, flags, vdes)

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
        # self.avoidance_margin is the ONE value a drive move's approach_d_safe
        # overrides, and it is read here, once per tick, straight off the live
        # attribute -- so the override needs no special case anywhere in this
        # function. goal_drive_callback sets it; _clear_drive_state restores
        # the baseline when the move ends. Both mechanisms that actually act on
        # the standoff (compute_local_target's R_safe just below, and
        # mpc_solver's obstacle trigger + boundary rows) read it from this
        # dict, so overriding it here covers both.
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

        live_boundaries = self._get_live_boundaries(corridor)
        # A WALL row is penalised differently from a costmap row, and much
        # harder: see wall_row_slack_weight's own derivation in
        # stack_params.yaml. One number per source rather than one shared one,
        # because the two carry different evidence -- a nearest-occupied-cell
        # tangent is a guess about an island, a tracked wall is a surface that
        # had to be confirmed, and treating a guess as near-inviolable is how
        # the costmap rows came to stop the car in the first place.
        boundary_slack_weight = (self.wall_row_slack_weight
                                 if self.boundary_source == 'walls'
                                 else self.boundary_slack_weight)
        self.get_logger().info(
            f'BOUNDARY/live | source={self.boundary_source} n={len(live_boundaries)}',
            throttle_duration_sec=2.0)
        if self.allow_reverse:
            self._update_cost_to_go(corridor, obstacles_global, live_boundaries, now_sec)

        # REVERSE MANOEUVRE: while one runs, IT drives and the forward solver
        # is not run at all (its warm start stays as it was).
        reverse_env = None
        if self.reverse_supervisor is not None:
            reverse_env = self._reverse_env(live_boundaries)
            manoeuvre_cmd = self.reverse_supervisor.before_forward(now_sec, x0, reverse_env)
            if manoeuvre_cmd is not None:
                self.last_u = np.array([manoeuvre_cmd.steer, 0.0], dtype=float)
                self._publish_drive(manoeuvre_cmd.speed, manoeuvre_cmd.steer,
                                    source='manoeuvre')
                self._publish_reverse_status()
                return

        # THE SPEED MODEL (speed_channel.py). Legacy: today's x0 and limits,
        # argument for argument. Offset-free: x0 gains the command in force,
        # the speed box moves onto the command (ceiling, ramp, stall cap, floor,
        # start-up bound) and the real speed gets a soft cap; the stop ramp's
        # box is handed over as a number instead of as limits['vMax'].
        x0_solve, last_u_solve, speed_kwargs = x0, self.last_u, {}
        solve_limits = self._stop_ramp_limits(vdes)
        if self._speed_offset_free():
            ch = self.speed_channel
            last_u_solve = np.array([self.last_u[0], ch.rate_in_force()], dtype=float)
            ramp_box = (float(solve_limits['vMax'])
                        if solve_limits is not self.limits else None)
            c0, speed_model = ch.plan(
                float(self.v), float(last_u_solve[1]), self.limits, vdes=vdes,
                ramp_box=ramp_box, floor=self._speed_floor_applies(), may_start=True)
            x0_solve = np.append(x0, c0)
            solve_limits = self.limits
            speed_kwargs = {'speed_model': speed_model}

        solve_t0 = self.get_clock().now().nanoseconds * 1e-9

        u0, info = self._mpc_solve(
            x0=x0_solve,
            last_u=last_u_solve,
            pref_nom=pref_nom,
            corridor=corridor,
            horizon=self.N,
            ts=self.ts_pred,
            params=self.params,
            limits=solve_limits,
            weights=self._weights_for(corridor),
            obstacles=obstacles_global,
            dmin=self.dmin,
            vdes=vdes,
            solver='rti' if self.use_rti_solver else 'slsqp',
            boundaries=live_boundaries,
            boundary_max_sources=self.boundary_max_sources,
            boundary_hard=self.boundary_hard,
            boundary_slack_weight=boundary_slack_weight,
            boundary_slack_max=self.boundary_slack_max,
            warm_start_z=self.warm_start_z,
            control_period=self.ts,
            **speed_kwargs,
        )

        solve_dt = self.get_clock().now().nanoseconds * 1e-9 - solve_t0

        # ---- per-row boundary violation, metres, for the next cycle's
        # /mpc/wall_constraints_diag. Slot order matches row order
        # (pad_boundary_constraints fills the slots in the order it is handed
        # the rows), so slot j belongs to wall_rows[j] while j < len(rows).
        self.last_boundary_slack = [float(v) for v in info.get('boundary_slack', [])]

        # ---- an infeasible solve NAMES THE ROWS. The existing fallback is
        # unchanged (warm start held, last_u tiled -- see the note below); what
        # this adds is the one thing it never told anyone: which constraint set
        # the solve was given. Without it an infeasible wall row is
        # indistinguishable from a bad linearisation in the log.
        if not info.get('success', False) and live_boundaries:
            self.get_logger().warn(
                f'SOLVE INFEASIBLE with {len(live_boundaries)} boundary row(s), '
                f'source={self.boundary_source}, '
                f'row_ids={[r.row_id for r in self.wall_rows]}, '
                f'status={info.get("status", "?")} -- falling back',
                throttle_duration_sec=2.0)

        # ---- carry this solve forward as the next tick's warm start.
        # Done HERE, immediately on return, rather than further down beside
        # self.last_u: several of the branches below return early (no usable
        # x_pred, hold, goal reached), and a warm start updated on only some
        # ticks is worse than one updated on none -- it would silently go
        # stale by an unpredictable number of periods.
        #
        # A SOLVE THAT FAILED IS NOT A SOLUTION. On failure _solve_rti hands
        # back the guess it was given (see its results.x check), so storing
        # info["zopt"] there would re-seed the next tick with the exact
        # sequence that just failed, and every tick after that. Drop to None
        # and let the solver tile last_u -- a different point, and a
        # dynamically consistent one.
        #
        # "solved inaccurate" IS KEPT, deliberately. OSQP reports it when it
        # hits max_iter having met the loose tolerance but not the tight one:
        # the iterate is a real, near-feasible solution, and it is precisely
        # the case where a good warm start next tick is what buys the
        # convergence. Discarding it would tile last_u instead, guaranteeing
        # the solver never climbs out of the inaccurate regime it is in.
        # info["success"] is already False for the genuinely unusable cases
        # (non-finite, infeasibility certificate), so that flag -- not the
        # status string -- is the right discriminator.
        if info.get("success", False):
            self.warm_start_z = shift_warm_start(
                info.get('zopt'), self.N, step_fraction=self.ts / self.ts_pred)
        else:
            self.warm_start_z = None

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
        self._publish_campaign_status(info, solve_dt, status_msg.solver)
        if self.allow_reverse:
            self._publish_reverse_debug(info)

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
            # One CONTROL period ahead, where the next tick's ERR line
            # compares it; x_pred[0] is one PREDICTION step ahead.
            if speed_kwargs:
                # The offset-free model's own one-tick prediction: the real
                # speed through the lag, not v + a * ts.
                self.prev_predicted_state = np.asarray(f1tenth_state_fcn_dt_beta_lag(
                    x0_solve, u0, self.ts, self.params['L'], self.params['lr'],
                    self.speed_channel.model_for(self.ts)), dtype=float)[:4]
            else:
                self.prev_predicted_state = np.asarray(f1tenth_state_fcn_dt_beta(
                    x0, u0, self.ts, self.params['L'], self.params['lr']), dtype=float)

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

        drive_source = 'forward'
        if speed_kwargs:
            # OFFSET-FREE: the command at stage 1, c0 + a0 * ts, inside the box
            # the plan was given -- the operating floor included, so the
            # legacy floor below has nothing left to do.
            v_cmd_now = self.speed_channel.command(a_cmd, info)
            self._speed_tick_planned = True
        else:
            v_cmd_now = self.v + a_cmd * self.ts
        if (not speed_kwargs and self.goal_object_odom_xy is not None
                and self.object_speed_mode == SPEED_DRIVE):
            # The operating floor, on the command that actually goes out. Every
            # non-driving object tick returned above, so this is only ever a
            # moving approach; the top of the range is _publish_drive's clamp.
            if self.allow_reverse:
                # Same floor in the selected direction, none while braking
                # for a direction change (reverse_mpc.floor_moving_speed_signed).
                v_cmd_now = floor_moving_speed_signed(
                    v_cmd_now, self.min_moving_speed,
                    self.reverse_planner.selector.direction, float(self.v),
                    self.reverse_planner.cfg.stop_speed_mps)
            else:
                v_cmd_now = floor_moving_speed(v_cmd_now, self.min_moving_speed)

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

        # The CTRL line moved to _publish_drive (_log_ctrl): it has to report
        # what actually went out, on every tick, and this point is skipped by
        # the before_forward manoeuvre path. The CSV above keeps its columns --
        # an analysis script parses them (test_model_log.py).
        self.get_logger().info(
            f'SOLVE/cmd | t_mpc={t_mpc:.3f} delta_cmd={delta_cmd:.3f} a_cmd={a_cmd:.3f} '
            f'v_cmd={v_cmd_log:.3f} wheel_speed={wheel_speed:.3f} '
            f'v_real={v_real:.3f} a_imu={a_imu:.3f}'
        )

        if self.reverse_supervisor is not None:
            self.reverse_supervisor.set_steer(delta_cmd)
            manoeuvre_cmd = self.reverse_supervisor.after_forward(
                now_sec, x0, bool(info.get('success', False)), info.get('x_pred'),
                self._mission_state == 'RUNNING' if self._mission_state is not None
                else not self.hold,
                self._reverse_intentional_stop(), self._reverse_front_distance(live_boundaries),
                reverse_env)
            if manoeuvre_cmd is not None:
                # The tick a manoeuvre starts: zero speed instead of the plan.
                v_cmd_now, delta_cmd = manoeuvre_cmd.speed, manoeuvre_cmd.steer
                drive_source = 'manoeuvre'
            self._publish_reverse_status()
        elif self.allow_reverse:
            self._publish_reverse_status_from_planner()

        self._publish_drive(v_cmd_now, delta_cmd, source=drive_source)

        # Model-validation row -- see model_log_path in __init__. AFTER the publish,
        # so its file I/O can never delay the command; x0/delta_cmd/a_cmd are the
        # same values the solve used and the publish sent.
        # accel_cmd keeps its documented meaning in both modes, "the car is sent
        # v + accel_cmd * ts" (docs/DIAGNOSTICS.md): offset-free sends a command
        # that is not v + a * ts, so the row carries the acceleration it implies.
        a_log = a_cmd if not speed_kwargs else (v_cmd_now - float(self.v)) / self.ts
        self._write_model_log(x0, delta_cmd, a_log)

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
    def _front_bumper_xy(self):
        """
        Return (x, y) of the front bumper (base_front) in odom, None without a pose.

        The MPC state is the rear axle (base_link); the bumper is the URDF's
        base_front, base_front_x() ahead of it along the heading. Both
        obstacle-distance topics are measured from here since 2026-10-12
        (they were from the rear axle).
        """
        if self.x is None or self.y is None or self.yaw is None:
            return None
        front = base_front_x()
        return (self.x + front * math.cos(self.yaw), self.y + front * math.sin(self.yaw))

    def compute_robot_obstacle_distance(self, obstacles_global):
        """Nearest obstacle in ANY direction, surface distance from the front bumper."""
        bumper = self._front_bumper_xy()
        if bumper is None:
            return 1e6

        dmin = 1e6
        for ox, oy, r in obstacles_global:
            d = math.hypot(bumper[0] - ox, bumper[1] - oy) - r
            dmin = min(dmin, d)
        return dmin

    # UPGRADE: clearance over the solver's *predicted* trajectory, not just the
    # current pose -- this is the actual verification that the chosen maneuver
    # keeps a safe margin, rather than just trusting the w_obs cost blindly.
    def compute_forward_obstacle_distance(self, obstacles_global):
        """Nearest obstacle in the FORWARD HALF-PLANE, surface distance
        (centre distance minus radius) from the front bumper (base_front),
        or 1e6 when there is nothing ahead.

        Identical to compute_robot_obstacle_distance above except for the
        half-plane test: an obstacle counts only if it lies in front of the
        car, i.e. the vector from the car to it has a positive component along
        the heading. Ported from f110_autonomy's distance_to_front_object(),
        which used the same dot > 0 test, because that is the sensor its guard
        "front_object" actually meant -- and "front_object" is what
        f1tenth_behavior's obstacle_distance_below stop_condition stands in
        for on the LLM path.

        THE BOUNDARY IS THE FULL HALF-PLANE, not a narrow cone, and that is
        deliberate: dot > 0 admits an obstacle at 89 degrees off the nose.
        A cone would be a different, tighter signal, and the one that already
        exists in this stack for that job is /costmap/front_clearance
        (symmetric +-front_facing_max_rad, map-derived) behind the separate
        front_clearance stop_condition. Reproducing the reference behaviour
        exactly beats inventing a third geometry here.

        1e6, not inf or None, matching compute_robot_obstacle_distance's own
        no-obstacles return so both topics carry the same "nothing to report"
        value and a consumer cannot tell them apart by sentinel alone.
        """
        bumper = self._front_bumper_xy()
        if bumper is None:
            return 1e6

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        # FROM THE FRONT BUMPER (2026-10-12), half-plane included: an obstacle
        # beside the body, ahead of the rear axle but behind the bumper, is
        # not in front of the car any more.
        dmin = 1e6
        for ox, oy, r in obstacles_global:
            dx = ox - bumper[0]
            dy = oy - bumper[1]
            if dx * cos_yaw + dy * sin_yaw <= 0.0:
                continue  # behind the bumper (or exactly abeam) -- not in the way
            d = math.hypot(dx, dy) - r
            dmin = min(dmin, d)
        return dmin

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

        # Signed rotation THIS corridor asks for, measured from psiStart, for a
        # wall_turn (the increment, not the whole remaining turn -- see that
        # branch). None on every other branch, and the solver's terminal yaw cost falls
        # straight back to its original shortest-branch unwrap when it is
        # absent -- so every non-wall_turn corridor is byte-identical to
        # before. Set only inside the wall_turn branch below.
        turn_remaining = None

        # True only on the object branch. Carried into the corridor dict so
        # _corridor_lookahead can tell "the lookahead reaches past the end
        # because the geometry is mis-configured" (every other branch, worth a
        # warning) from "because the vehicle has arrived" (this branch, the
        # expected end of every successful approach).
        object_mode = False

        # Which shape the object branch chose, for the v2 record and the
        # campaign columns. 'none' on every other branch: the field always
        # exists, so a reader never has to infer the mode from its absence.
        object_corridor_shape = 'none'

        # The POSE the centreline must END on, when it must end on one.
        # Set only by the object branch, and only where ending on the target
        # is both wanted and drivable (see there). None everywhere else means
        # the ordinary length-driven geometry: the corridor ends where a
        # heading ramp of that length happens to put it.
        pose_target = None

        # The standoff point this corridor was built to reach, and the tracked
        # target it was derived from -- both odom, both None off the object
        # branch. Logged per rebuild because "where was the car being sent"
        # was not recoverable from a finished run: corridors.jsonl carried the
        # geometry but never the target behind it, so a corridor built on a
        # stale or jumped target looked exactly like one built on a good one.
        object_goal_xy = None
        # The straight line the target sits on, for mpc_solver's w_line term.
        # None on every branch but the object one, and None there too whenever
        # the line is meaningless -- see where it is set. None means the term
        # adds no QP rows at all, not that it adds zero-weighted ones.
        target_line = None

        # getattr, not a bare attribute, for the same reason corridor_heading_
        # return below uses one: the corridor tests build duck-typed stand-ins
        # that predate this mode and carry only the fields the geometry under
        # test needs. A bare self.drive_cmd would make every one of them raise
        # AttributeError here instead of selecting a shape. The real node
        # always has the attribute (set in __init__).
        drive_cmd = getattr(self, 'drive_cmd', None)
        # Same reason, same convention, for the object branch's two: the
        # stand-ins predate this mode too, and a bare attribute read below
        # makes all 35 of them raise AttributeError instead of selecting a
        # shape. The real node always has both (set in __init__).
        object_target = getattr(self, 'goal_object_odom_xy', None)
        object_psi_c = getattr(self, 'object_psi_c', None)

        if drive_cmd is not None:
            # DRIVE (open-ended) move -- a corridor built from a MODE, not
            # from a target. Same shape as the goal_distance branch below (it
            # passes through the car's current position along a frozen
            # heading), differing only in where that frozen heading comes
            # from and, for wall_turn, in which end of the blend is live.
            #
            # THE REFERENCE IS THIS MOVE'S OWN START HEADING, NOT THE RUN'S.
            # psi_init_corridor is re-anchored by goal_drive_callback on every
            # drive command. f110_autonomy computed the identical expression
            # (psiEnd = psi_base + turn_sign * turn_mag for wall_turn,
            # psi_base for straight) against a psi_base captured ONCE at the
            # first odom message after node start and never re-anchored --
            # so its second turn was measured from a heading the car had left
            # long ago, and its third from one further still. That is a bug,
            # and this is a deliberate divergence from it, not an oversight.
            #
            # The second divergence is turn_mag_deg itself: that stack never
            # read a magnitude off the phase (hardcoded pi/2), so a plan
            # asking for 180 degrees got 90. Here the phase's own value is
            # what lands, with the node default reserved for the sentinel.
            psi_base = (self.psi_init_corridor
                        if self.psi_init_corridor is not None else psi0)
            if drive_cmd['mode'] == 'wall_turn':
                # THE COMMANDED TURN IS SIGNED, AND ITS SIGN IS NOT
                # RECOVERABLE FROM psiEnd ALONE. turn_sign says which way
                # round to go; a heading does not. "Turn 180 degrees right"
                # and "turn 180 degrees left" name the SAME psiEnd, and any
                # consumer that later re-derives the rotation from
                # (psiEnd - yaw) has to guess -- the shortest branch is the
                # only thing it can guess, and at 180 that is a coin flip
                # decided by floating-point noise in sin(). Worse, it locks
                # in: a solve whose reference rollout drifts a hair the wrong
                # way pushes the error past pi, the shortest branch flips to
                # the other side, the car steers further that way, and the
                # next tick is deeper in. drive_turn_180.json asks for
                # exactly 180.0, so this is not a corner case.
                #
                # So the rotation is computed here, where the sign is still
                # known, and carried to the solver as psiRefTurn (see the
                # corridor dict below and mpc_solver's terminal yaw cost).
                # turn_progress_rad is the unwrapped rotation already made --
                # unwrapped precisely so that subtracting it stays correct
                # past 180 degrees, which a wrapped difference cannot be.
                signed_total = math.radians(
                    drive_cmd['turn_sign'] * drive_cmd['turn_mag_deg'])
                # ONE CORRIDOR CARRIES ONE INCREMENT, NOT THE WHOLE TURN. This
                # used to be psiEnd = psi_base + signed_total on every rebuild:
                # a target the 1.0 m horizon (N * ts * vdes at 0.5 m/s) cannot
                # reach for any turn much past 54 degrees, so the QP sat on the
                # steering bound. wall_turn.py decides how much of what is
                # still owed THIS corridor asks for -- nothing until the wall
                # distance forces the turn to commit, then no more than the
                # distance and the horizon can carry -- and the next rebuild
                # asks for the next piece. Read its module docstring for the
                # rule, the commit gate and the anti-chatter ratchet.
                #
                # dpsi_this is the ONE source of truth for this corridor: it is
                # psiEnd - psiStart (terminal heading cost), psiRefTurn (the
                # solver's signed branch) AND the centreline's S-curve dpsi
                # below. Feeding the increment to one of them while another
                # still swung the full angle would set them against each other.
                # WHICH DISTANCE THE RULE RUNS ON. dFront until the turn
                # commits and on the rebuild it commits; from then on d_wall,
                # the tracked wall's bumper distance (wall_tracker.py), for
                # as long as the tracker holds a wall. dFront is measured
                # along the car's heading, which rotates away from the wall
                # mid-turn, so it goes optimistic exactly when the rule
                # needs it honest; d_wall is heading-independent. When no
                # candidate passed the selection gates the rule stays on
                # dFront, exactly as before the tracker existed. getattr for
                # the duck-typed corridor test stand-ins, as with drive_cmd.
                d_front = self._fresh_front_distance()
                # None (no usable distance) for the rule below; the corridor
                # debug and the CORR/build log further down take -1.0, the
                # wire's own "no reading" (float(None) used to raise out of the
                # control loop on every wall_turn rebuild without a wall).
                tracker = getattr(self, 'wall_tracker', None)
                d_wall = None
                if tracker is not None and tracker.has_wall:
                    # Refitted by _wall_track_tick earlier this tick.
                    d_wall = tracker.d_wall((X0, Y0, psi0))
                d_rule = d_wall if d_wall is not None else d_front
                was_committed = self.wall_turn_committed
                wall_step = plan_wall_turn_step(
                    signed_total, self.turn_progress_rad,
                    d_rule,
                    wheelbase=self.params['L'],
                    delta_min=self.limits['delta_min'],
                    delta_max=self.limits['delta_max'],
                    k_safety=self.corr_wall_turn_k_safety,
                    safety_margin=self.corr_wall_turn_safety_margin_m,
                    # self.ts (control period), NOT ts_pred, deliberately: the
                    # cap was tuned at N * 0.1 * vdes = 1.0 m of reach, and at
                    # the 2.0 m of the 4 s horizon it would let one rebuild ask
                    # for a whole 90 deg on the 1.2 m S-curve ramp (peak
                    # curvature 1.96 1/m against 1/R_min = 1.08).
                    n_steps=self.N, ts=self.ts, v_ref=self.vdes,
                    committed=self.wall_turn_committed,
                    prev_commanded_rot=self.wall_turn_commanded_rot)
                self.wall_turn_committed = wall_step.committed
                self.wall_turn_commanded_rot = wall_step.commanded_rot
                turn_remaining = wall_step.dpsi_this
                psiEnd = psi0 + wall_step.dpsi_this
                # SELECT THE WALL ON THE REBUILD THE TURN COMMITS. The gating
                # references (this heading, this dFront) are frozen then and
                # never move; a rebuild with no usable scan retries the
                # selection against those same references, which is not
                # re-gating a selected wall -- there is none yet.
                if tracker is not None and wall_step.committed and not tracker.has_wall:
                    if not was_committed:
                        tracker.commit(psi0, d_front)
                        self.get_logger().info(
                            f'WALL/commit | psi_commit={psi0:+.4f} '
                            f'dFront={"unknown" if d_front is None else f"{d_front:.2f}"}: '
                            'selecting the front wall from /scan')
                    self._select_tracked_wall(
                        (X0, Y0, psi0), self.get_clock().now().nanoseconds * 1e-9)
                d_avail_txt = ('unknown' if wall_step.d_avail is None
                               else f'{wall_step.d_avail:.2f}')
                if d_wall is not None:
                    d_rule_txt = (f'd_wall={d_wall:.2f} '
                                  f'({PROVENANCE_NAMES[tracker.provenance]}, '
                                  f'{tracker.last_inlier_count} returns)')
                else:
                    d_rule_txt = 'd_wall=none'
                self.get_logger().info(
                    f'CORR/wall_turn | dFront={self.front_distance:.2f} '
                    f'{d_rule_txt} '
                    f'd_avail={d_avail_txt} '
                    f'rem={wall_step.dpsi_rem:+.4f}/{signed_total:+.4f} '
                    f'R_min={wall_step.r_min:.3f} '
                    f'by_dist={wall_step.dpsi_by_dist:.4f} '
                    f'by_horizon={wall_step.dpsi_by_horizon:.4f} '
                    f'committed={wall_step.committed} held={wall_step.held} '
                    f'dpsi_this={wall_step.dpsi_this:+.4f} '
                    f'cmd_rot={wall_step.commanded_rot:+.4f}'
                )
                # psiStart stays the LIVE yaw for a turn, so the S-curve below
                # describes a real arc from where the car is pointing now to
                # where the move wants it pointing. Freezing both ends here
                # (as the goal_distance branch does by default) would make
                # dpsi identically zero and leave a corridor running
                # perpendicular to the car through its own position -- the
                # solver's w_psi would still eventually rotate the car, but
                # through a geometry the half-width bound is violated by from
                # the first tick. f110_autonomy set psiStart = psi0 here too.
                psiStart = psi0
            else:
                # "straight": hold the move's own start heading. Same
                # geometry, and the same corridor_heading_return choice, as
                # the goal_distance branch -- see its own long note below for
                # why both-ends-frozen is the shipping default and what the
                # measurement was.
                #
                # PLUS the d_wall correction, and THIS IS THE ONE PLACE IT IS
                # APPLIED. See the corr_d_wall_correction_enable block in
                # __init__ for why here and not at dpsi_this, and
                # docs/wall_turn_investigation.md for the full seam analysis.
                #
                # WHY BIASING psi_base IS A LATERAL CONTROLLER AT ALL. This
                # branch's corridor passes through the car's current position
                # and points along a FROZEN heading; the S-curve below blends
                # the live yaw onto it over the corridor's length, and the
                # solver's w_psi/w_term costs pull the car onto that heading.
                # Rotating the frozen heading by dpsi therefore commands a
                # sustained heading offset, which integrates into lateral
                # motion at edot = v*sin(dpsi) -- the first-order law
                # wall_distance.py derives. It does NOT displace the corridor
                # sideways, so corr_wmin/corr_wmax and the hard boundary rows
                # are untouched and the car never starts outside its own
                # corridor. The deliberate removal of lateral homing recorded
                # in the goal_distance branch below is likewise untouched: the
                # centreline still passes through the car by construction.
                #
                # ZERO unless a fresh, enabled correction exists, so with
                # wall_distance_node absent, stale or disabled this line is
                # byte-identical to psiEnd = psi_base.
                #
                # getattr, not a bare method call, for exactly the reason the
                # drive_cmd lookup above uses one: the corridor tests build
                # duck-typed stand-ins carrying only the fields the geometry
                # under test needs, and a bare self._fresh_d_wall_correction()
                # makes every one of them raise AttributeError here instead of
                # selecting a shape. test_wall_turn_increment.py's
                # test_the_straight_drive_branch_is_unchanged is the one that
                # catches it. The real node always has the method.
                _correction = getattr(self, '_fresh_d_wall_correction', None)
                dpsi_d_wall = _correction() if _correction is not None else 0.0
                psiEnd = psi_base + dpsi_d_wall
                if dpsi_d_wall != 0.0:
                    self.get_logger().info(
                        f'CORR/d_wall | psi_base={psi_base:+.4f} '
                        f'dpsi_d_wall={dpsi_d_wall:+.4f} -> psiEnd={psiEnd:+.4f}')
                if getattr(self, 'corridor_heading_return',
                           get_value('corridor_heading_return')):
                    psiStart = psi0
                else:
                    psiStart = psiEnd
            # No goal to clip the length against (that is what open-ended
            # means), so the full nominal corridor length, exactly as the
            # goal_distance branch uses.
            L = max(self.corr_L_base, 1.0)
            self.get_logger().info(
                f'CORR/drive | mode={drive_cmd["mode"]} '
                f'psi_base={psi_base:+.4f} psiStart={psiStart:+.4f} '
                f'psiEnd={psiEnd:+.4f} L={L:.2f}'
            )

        elif object_target is not None and object_psi_c is not None:
            # OBJECT MODE -- a straight corridor PINNED AT THE TARGET, along a
            # HELD heading. Two deliberate departures from every other branch,
            # both of which the goal_pose branch below gets wrong for this job.
            #
            # 1. THE LINE IS PINNED TO THE TARGET, NOT TO THE CAR. Every other
            #    corridor here passes through the car's current position; the
            #    goal_distance branch says so outright, and accepts in exchange
            #    that it has no lateral homing ("THE PRICE", below). That trade
            #    is right for a move defined by a direction and a distance and
            #    wrong for one defined by a PLACE: with the line translated onto
            #    the car there is no cross-track error, so nothing pulls the car
            #    back onto the line to the object and the approach converges in
            #    heading only. Pinning the line through the target gives w_corr
            #    and the half-width bound a real error to act on.
            #
            # 2. NO LENGTH FLOOR. The goal_pose branch clips its length into
            #    [1.0, corr_L_base], so a goal 0.3 m away still gets a 1.0 m
            #    corridor whose far end is PAST the goal -- and since the
            #    lookahead clamps to the corridor end inside about 1.25 m, the
            #    terminal cost then pulls the car THROUGH the target for the
            #    whole final approach. Here the corridor ends exactly at the
            #    goal; a short corridor is the correct description of a nearly
            #    finished approach.
            #
            # psiStart == psiEnd == psi_c, so dpsi is 0 and the S-curve below
            # is inert -- the centreline is straight by construction rather
            # than by a special case. The heading is not computed from the live
            # bearing here: it is the HELD state, rotated a scheduled fraction
            # of the way toward the bearing by plan_object_heading, which is
            # called exactly once per rebuild. That is what keeps a jittering
            # target estimate out of the corridor. See object_approach's module
            # docstring for why the state is absolute and what goes wrong if it
            # is turned into an increment.
            step = plan_object_heading(
                object_psi_c, object_target, (X0, Y0), psi0,
                self.goal_object_standoff,
                r_full=self.object_r_full,
                r_freeze=self.object_r_freeze,
                c_safety=self.object_c_safety,
                wheelbase=self.params['L'],
                delta_min=self.limits['delta_min'],
                delta_max=self.limits['delta_max'],
                # Control period, not ts_pred: same reason as the wall_turn
                # cap -- psi_c's per-rebuild swing stays as tuned.
                n_steps=self.N, ts=self.ts, v_ref=self.vdes)
            self.object_psi_c = step.psi_c_new
            self.object_last_step = step
            self.object_target_at_build = object_target

            origin, psi_c, L = build_object_centreline(
                (X0, Y0), object_target, step.psi_c_new,
                self.goal_object_standoff, self.corr_N,
                behind=self.object_lead_in_m)
            # Reassigning the corridor ORIGIN, which no other branch does. Safe
            # because everything downstream builds from xc/yc, and the only
            # other reader of X0/Y0 is the goal_distance branch's own lat_off
            # diagnostic.
            # THE SHAPE DECISION. `origin`/`psi_c` above are today's geometry:
            # the centreline pinned to the target line, starting behind the
            # car, straight because both ends are psi_c. object_corridor_mode
            # can replace it with the arc that actually gets the car there.
            #
            # r_goal is the distance from the CAR to the corridor's own end
            # (the standoff point), which is the length an arc from here would
            # have, and so the quantity the switch band is about: the arc is
            # only safe while the lookahead still sits inside the corridor.
            r_goal = math.hypot(origin[0] + L * math.cos(psi_c) - X0,
                                origin[1] + L * math.sin(psi_c) - Y0)
            # The corridor's own end under today's geometry: the standoff
            # point, i.e. the place the car is actually being sent to. Kept
            # because the pose geometry below needs it as a POINT, while the
            # rest of this branch only ever needs its distance.
            object_goal_xy = (origin[0] + L * math.cos(psi_c),
                              origin[1] + L * math.sin(psi_c))
            # getattr for the same reason as drive_cmd/object_target above:
            # the corridor stand-ins predate this mode and carry none of it.
            # Absent -> 'off' -> today's geometry, which is what a stand-in
            # written before the mode existed is asserting against.
            _mode = getattr(self, 'object_corridor_mode', 'off')
            if _mode == 'arc':
                use_arc = True
                clamp_len = float('nan')
            elif _mode == 'arc_far':
                clamp_len = lookahead_clamp_length(
                    self.N, getattr(self, 'ts_pred', self.ts), self.vdes,
                    self.corr_lookahead_reach_margin)
                # Hysteresis: enter the arc above hi, leave it below lo, hold
                # in between. See the parameter block for why the band cannot
                # be a single threshold.
                if r_goal >= getattr(self, 'object_arc_switch_hi_frac',
                                     1.12) * clamp_len:
                    use_arc = True
                elif r_goal <= getattr(self, 'object_arc_switch_lo_frac',
                                       0.88) * clamp_len:
                    use_arc = False
                else:
                    use_arc = getattr(self, '_object_arc_active', False)
            else:
                use_arc = False
                clamp_len = float('nan')
            self._object_arc_active = use_arc

            if use_arc:
                # The SAME family as every other branch, differing only in
                # what the two headings are: origin on the car (as the
                # straight and turn branches already do), psiStart the live
                # yaw, psiEnd the bearing psi_c that plan_object_heading
                # already computes and rate-limits. dpsi is then the car's
                # heading error and the existing S-curve draws the arc.
                psiStart = psi0
                psiEnd = psi_c
                # A target on top of the car has no arc; the guard keeps
                # corridor_curves' ds from collapsing to zero.
                L = max(r_goal, 1e-3)
                object_corridor_shape = 'arc'
                # ENDING ON THE TARGET, and the one condition under which that
                # is even a question. The heading-ramp arc is built from a
                # LENGTH, so where it ends is an output -- measured over the 24
                # archived rebuilds it lands mean 0.25 m, max 0.96 m from the
                # goal, and Pend is what w_term pulls toward once the lookahead
                # clamps. corridor_curves_to_pose ends ON the goal by
                # construction, tangent to psi_c, with the cap perpendicular
                # to it.
                #
                # ONLY WHEN THE TARGET IS WITHIN THE MAXIMUM LENGTH, and the
                # switch is explicit rather than a side effect of the cut
                # below: a corridor cut at corr_L_base does not reach the
                # target at all, so "ends on the target" is not a property it
                # can have. Beyond the cap the ramp arc is the geometry, cut,
                # with psiEnd still the bearing -- see the cut block.
                #
                # Feasibility is decided at the build site, not here: the
                # curvature it needs is a property of the curve, so it is
                # measured on the curve rather than predicted from a rule.
                if r_goal <= max(float(self.corr_L_base), 1e-3):
                    pose_target = object_goal_xy
            else:
                X0, Y0 = origin
                psiStart = psi_c
                psiEnd = psi_c
                object_corridor_shape = 'straight'
            # THE TARGET LINE, which is what today's corridor centreline IS:
            # the line through the target at psi_c. It is handed to the solver
            # separately from here on, because under the arc shape the corridor
            # no longer lies along it and the two errors stop being the same
            # number. Set from `origin` and psi_c, so it is the same line in
            # both shapes and the switch does not move it.
            #
            # CLEARED, not down-weighted, when the target is BEHIND the car:
            # past pi/2 the bearing flips and the "line to the target" runs
            # backwards through the car, so a cost pulling onto it would drive
            # the car away from the approach. assess_object_approach's
            # persistent flag is the gate. getattr for the duck-typed corridor
            # test stand-ins, which carry no object flags.
            if not use_arc:
                # ON A STRAIGHT OBJECT CORRIDOR THE CENTRELINE *IS* THE TARGET
                # LINE. w_corr already owns that error; adding w_line on top
                # would charge the same deviation twice and double the lateral
                # stiffness (1.25 + 1.25) on the DEFAULT path, where nothing
                # about the geometry has changed at all. The split exists only
                # because the arc makes them two different curves.
                target_line = None
            elif getattr(self, 'object_behind_terminal', False):
                target_line = None
                self.get_logger().warn(
                    'CORR/object | target behind the car: the target-line cost '
                    'is OFF for this corridor', throttle_duration_sec=5.0)
            else:
                target_line = {'p': [float(origin[0]), float(origin[1])],
                               'psi': float(psi_c)}
            self.get_logger().info(
                f'CORR/object_shape | mode={_mode} '
                f'shape={object_corridor_shape} r_goal={r_goal:.2f} '
                f'clamp={clamp_len:.2f} '
                f'band=[{getattr(self, "object_arc_switch_lo_frac", 0.88) * clamp_len:.2f},'
                f'{getattr(self, "object_arc_switch_hi_frac", 1.12) * clamp_len:.2f}] '
                f'dpsi={wrap_pi(psiEnd - psiStart):+.4f}')
            object_mode = True
            self.get_logger().info(
                f'CORR/object | psi_c={step.psi_c_new:+.4f} e={step.e:+.4f} '
                f'k={step.k:.3f} dpsi_max={step.dpsi_max:.4f} r={step.r:+.3f} '
                f'L={L:.3f}')

        elif self.goal_pose_xy is not None:
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

        # THE GEOMETRY ITSELF LIVES IN f1tenth_params.corridor_geometry.
        # It used to be ~110 lines inline here, which meant the test-campaign
        # plotter (and anything else re-evaluating a logged corridor) had to
        # reimplement the shape and could silently drift from it. Read that
        # module's docstring for what the three curves are and why the
        # centreline's cumsum makes `corr_N` part of the definition rather
        # than a rendering detail.
        #
        # turn_remaining is passed straight through as dpsi: SIGNED and
        # UNWRAPPED for a wall_turn, because the geometry has to bend the way
        # the mission asked, not the short way. wrap() cannot express a
        # rotation of 180 degrees or more -- wrap(+270 deg) is -90 deg -- so a
        # corridor built from it would curl RIGHT for a commanded 270 degree
        # LEFT turn, and then every position term in the QP (w_term to Pend,
        # w_corr onto the centreline, w_psi_stage onto the local tangent)
        # would pull against the terminal heading cost instead of with it.
        # Same defect as the terminal unwrap, one layer out: see the wall_turn
        # branch above. None on every other branch, where corridor_curves
        # falls back to the shortest-branch wrap, exactly as before.
        #
        # The S-curve heading blend (corr_turn_u_start/u_end) and the widening
        # funnel (corr_wmin/corr_wmax) are unchanged; see corridor_geometry
        # for the shape and stack_params.yaml for the measured table behind
        # the 0.00/0.40 ramp.
        # R_min for the direction this corridor turns, so the ramp-span clamp
        # knows which steering bound applies: the two are not equal (0.923 m
        # left, 0.955 m right at the shipping calibration). Same rule as
        # plan_object_heading's and wall_turn's owed_sign.
        #
        # getattr for both, for exactly the reason drive_cmd and object_target
        # above use one: the corridor tests build duck-typed stand-ins carrying
        # only the fields the geometry under test needs, and a bare self.limits
        # makes every one of them raise AttributeError here instead of
        # selecting a shape. Without the limits there is no R_min to clamp
        # against, so the clamp is simply off and the geometry is what it was
        # before it existed -- which is the right answer for a stand-in.
        # ---- THE MAXIMUM CORRIDOR LENGTH, one cap for every branch ------
        # Every branch but the object one already produces L <= corr_L_base
        # (goal_distance uses it outright, goal_pose clips into
        # [1.0, corr_L_base]), so this line bites only the object branch --
        # which is the point: the cap belongs to the corridor, not to the
        # branch that asked for one, and a branch added later inherits it.
        #
        # A CUT IS NOT THE CLIP THE OBJECT BRANCH REFUSES. That refusal is
        # about the LOWER bound: a 1.0 m floor on a goal 0.3 m away puts the
        # corridor END PAST the target, and inside the lookahead clamp
        # (~1.25 m) Pend IS the terminal position cost, so the car is sent
        # through the thing it was approaching. A maximum-length cut can only
        # move the end CLOSER than the target, and it can only engage when the
        # target is farther than corr_L_base = 3.0 m -- i.e. when Pend sits at
        # 3.0 m, more than twice the clamp distance, where w_term acts on the
        # lookahead as a direction pull and not as an arrival target. The two
        # cases are opposites, and the lower bound stays absent here.
        #
        # SAME ARC, ENDING EARLY. The heading ramp is a fraction of L, so
        # simply shortening L would cram the same turn into a shorter corridor
        # and bend it harder. Rescaling the ramp by L_full / L_cut keeps it at
        # the same METRES from the car, which makes the cut corridor the exact
        # prefix of the uncut one: tau is then a function of distance alone.
        #
        # psiEnd IS UNCHANGED, and that is a consequence rather than a choice.
        # The ramp finishes at u_end * L_full (1.85 m into the 4.63 m corridor
        # P004-R003 actually logged), so with the cut at 3.0 m the heading has
        # already stopped changing there and the direction AT the cut is
        # psiEnd. The end cap, built from psiEnd, is therefore perpendicular to
        # the real end direction. The only way that stops holding is a target
        # beyond corr_L_base / u_end = 7.5 m, where min(1.0, ...) lands the
        # ramp's end exactly on the cut instead: the turn is then planned over
        # the corridor that exists rather than over a target too far to see,
        # the cap is still perpendicular, and it is said out loud below.
        L_full = float(L)
        # A pose corridor is selected only inside the cap, so there is nothing
        # to cut -- and its length is an OUTPUT of the fit, not an input, so
        # capping it here would cap a number that has not been computed yet.
        L = (L_full if pose_target is not None
             else min(L_full, max(float(self.corr_L_base), 1e-3)))
        corridor_cut = L < L_full - 1e-9
        _scale = (L_full / L) if corridor_cut and L > 0.0 else 1.0
        u_start_use = min(1.0, self.corr_turn_u_start * _scale)
        u_end_use = min(1.0, self.corr_turn_u_end * _scale)
        if corridor_cut:
            ramp_truncated = self.corr_turn_u_end * L_full > L + 1e-9
            self.get_logger().info(
                f'CORR/cut | target at {L_full:.2f} m is beyond corr_L_base='
                f'{self.corr_L_base:.2f} m: corridor cut to {L:.2f} m, ramp '
                f'u=[{u_start_use:.3f}, {u_end_use:.3f}]'
                + (' -- RAMP TRUNCATED: the turn is planned over the cut '
                   'corridor, not over the full approach' if ramp_truncated
                   else ''),
                throttle_duration_sec=5.0)

        _dpsi_for_bound = (turn_remaining if turn_remaining is not None
                           else wrap_pi(psiEnd - psiStart))
        _limits = getattr(self, 'limits', None)
        _params = getattr(self, 'params', None)
        _r_min = None
        if _limits is not None and _params is not None:
            _bound = (_limits['delta_max'] if _dpsi_for_bound >= 0.0
                      else _limits['delta_min'])
            _r_min = min_turn_radius(_params['L'], _bound)
        # THE POSE GEOMETRY, and its fallback. The curve is built first and
        # judged afterwards because its peak curvature is a property of the
        # curve: kappa grows as the heading error over the distance left, so
        # the same 10 degrees is free at 3 m and past full lock at 0.5 m, and
        # no distance threshold expresses that as well as the number itself.
        # Over the 24 archived rebuilds the fit is drivable on every rebuild
        # beyond 2.0 m, 8 of 9 beyond 1.55 m and 9 of 12 beyond 1.25 m -- so a
        # fallback is not an edge case, it is the final approach, and it is the
        # straight geometry the branch has always used there.
        if pose_target is not None:
            geom = corridor_curves_to_pose(
                X0, Y0, psiStart, pose_target[0], pose_target[1], psiEnd,
                self.corr_N,
                w0=self.corr_wmin,
                w1=self.corr_wmax,
                r_min=_r_min,
                length_ref=(getattr(self, 'corr_L_base', None)
                            if getattr(self, 'corr_width_scale_with_length',
                                       False) else None),
                walls=CORRIDOR_WALLS,
            )
            # THE CAP APPLIES TO THIS SHAPE TOO, and it has to be checked
            # after the fit rather than before it: the pose corridor's length
            # is an OUTPUT (an arc between two poses is longer than the chord
            # between them -- measured 1.008x median, 1.14x worst over the
            # archived rebuilds), so a target inside corr_L_base can still
            # produce a corridor past it. Rejected rather than trimmed,
            # because a trimmed pose corridor no longer ends on the pose that
            # is its whole reason for existing.
            _over_cap = float(geom['length']) > max(float(self.corr_L_base),
                                                    1e-3) + 1e-9
            if geom['feasible'] and not _over_cap:
                object_corridor_shape = 'pose_arc'
                L = float(geom['length'])
                L_full = L
            elif _over_cap:
                self.get_logger().info(
                    f'CORR/pose | the fit is {geom["length"]:.2f} m long for a '
                    f'{L_full:.2f} m chord, past corr_L_base='
                    f'{self.corr_L_base:.2f} m: building the cut arc instead',
                    throttle_duration_sec=5.0)
                pose_target = None
                object_corridor_shape = 'arc'
                L = min(L_full, max(float(self.corr_L_base), 1e-3))
                corridor_cut = L < L_full - 1e-9
                _scale = (L_full / L) if corridor_cut and L > 0.0 else 1.0
                u_start_use = min(1.0, self.corr_turn_u_start * _scale)
                u_end_use = min(1.0, self.corr_turn_u_end * _scale)
            else:
                self.get_logger().warn(
                    f'CORR/pose | ending on the target needs kappa='
                    f'{geom["kappa_max"]:.3f} 1/m (R='
                    f'{1.0 / max(geom["kappa_max"], 1e-9):.2f} m) against '
                    f'R_min={_r_min:.3f} m at {L_full:.2f} m to go: falling '
                    'back to the straight corridor pinned to the target line',
                    throttle_duration_sec=5.0)
                pose_target = None
                # Back to the geometry that branch would have built without
                # the arc at all -- the straight corridor on the target line,
                # not the ramp arc, because the ramp arc's own endpoint drift
                # is worst exactly here.
                X0, Y0 = origin
                psiStart = psi_c
                psiEnd = psi_c
                L = min(L_full, max(float(self.corr_L_base), 1e-3))
                corridor_cut = L < L_full - 1e-9
                object_corridor_shape = 'straight'
                self._object_arc_active = False
                # AND THE TARGET LINE GOES WITH IT. It was set above because
                # the arc was chosen, and on the straight corridor the
                # centreline IS the target line -- leaving it would charge the
                # same lateral error through w_corr and w_line at once, which
                # is the doubling the split exists to avoid.
                target_line = None

        if pose_target is None:
            geom = corridor_curves(
                X0, Y0, psiStart, psiEnd, L, self.corr_N,
                dpsi=turn_remaining,
                u_start=u_start_use,
                u_end=u_end_use,
                w0=self.corr_wmin,
                w1=self.corr_wmax,
                # BOTH CLAMPS APPLY TO EVERY BRANCH, not only the object one.
                # The ramp span is a fraction of L, so peak curvature runs away
                # as the corridor shortens; the straight branch is unprotected
                # without this and goes past full lock beyond ~50 degrees of
                # heading error at L 3.0. wall_turn caps its own dpsi
                # separately, so there this is belt-and-braces.
                r_min=_r_min,
                # The per-metre funnel opening, only when
                # corr_width_scale_with_length asks for it: 10 of the 24
                # archived object rebuilds were WIDER THAN LONG without it,
                # but with it the end/start ratio fell to 1.2-1.6 on the
                # corridors the car actually built and the walls read as
                # parallel. getattr: the stand-ins predate the parameter.
                length_ref=(getattr(self, 'corr_L_base', None)
                            if getattr(self, 'corr_width_scale_with_length',
                                       False) else None),
                walls=CORRIDOR_WALLS,
            )
        if not geom['feasible'] and pose_target is None:
            self.get_logger().warn(
                f'CORR/curvature | dpsi={geom["dpsi"]:+.4f} over L={L:.2f} m '
                f'needs a ramp longer than the corridor: kappa_max='
                f'{geom["kappa_max"]:.3f} (R={1.0 / max(geom["kappa_max"], 1e-9):.2f} m) '
                f'against R_min={_r_min:.3f} m. '
                'The reference asks for more than full lock; the solver will '
                'saturate steering.', throttle_duration_sec=5.0)
        xc, yc = geom["xc"], geom["yc"]
        xL, yL = geom["xL"], geom["yL"]
        xR, yR = geom["xR"], geom["yR"]
        halfWidth = geom["halfWidth"]
        dpsi = geom["dpsi"]
        p_goal = geom["Pend"]
        if d_front is None:
            d_front = -1.0      # see the wall_turn branch: debug and log only

        corridor = {
            "xc": xc,
            "yc": yc,
            "xL": xL,
            "yL": yL,
            "xR": xR,
            "yR": yR,
            "tx": geom["tx"],
            "ty": geom["ty"],
            "nx": geom["nx"],
            "ny": geom["ny"],
            "halfWidth": halfWidth,
            # THE FUNCTION DEFINITION, for corridor_payload's v2 record: every
            # number needed to re-evaluate these exact arrays offline. Kept
            # next to the arrays rather than recomputed at publish time so the
            # two can never describe different corridors.
            "defn": {
                "type": "mpc_corr/v2",
                "C0": [float(X0), float(Y0)],
                "psiStart": float(psiStart),
                "psiEnd": float(psiEnd),
                "dpsi": float(dpsi),
                "psiRefTurn": (None if turn_remaining is None
                               else float(turn_remaining)),
                "L": float(L),
                "corr_N": int(self.corr_N),
                # The ramp ACTUALLY USED, which is the asked-for one except
                # on a cut corridor -- a re-evaluator reproduces the arrays
                # from these, so they must be what corridor_curves was given.
                "u_start": float(u_start_use),
                "u_end": float(u_end_use),
                # What the cut did, for the campaign rather than for the
                # geometry: L above is already the cut length, so a reader
                # needs no arithmetic, but "3.00 m because the target was
                # 4.63 m away" and "3.00 m because that is the nominal length"
                # are different runs and only these two tell them apart.
                "L_full": float(L_full),
                "cut": bool(corridor_cut),
                "w0": float(self.corr_wmin),
                "w1": float(self.corr_wmax),
                "handle_frac": float(CORRIDOR_HANDLE_FRAC),
                # The wall construction; its absence means the Bezier walls
                # of every older record.
                'walls': CORRIDOR_WALLS,
                # WHICH CENTRELINE, because from here there are two kinds and
                # they are not distinguishable from the other fields: 'ramp'
                # is the integrated heading ramp of the given L, 'bezier' the
                # cubic through the two END POSES. A reader that does not know
                # the key is looking at a record written before the pose
                # geometry existed, where 'ramp' is the only possibility --
                # which is exactly what corridor_def.evaluate assumes.
                "centreline": ('bezier' if pose_target is not None
                               else 'ramp'),
                # The end POSE, which for a bezier centreline is an input and
                # not a consequence: C0 + these + the handles reproduce it.
                "C1": ([float(pose_target[0]), float(pose_target[1])]
                       if pose_target is not None else None),
                "handle_a": (float(POSE_HANDLE_FRAC)
                             if pose_target is not None else None),
                "handle_b": (float(POSE_HANDLE_FRAC)
                             if pose_target is not None else None),
                # what the clamps actually produced, not what was asked for
                "u_end_eff": float(geom["u_end_eff"]),
                "w1_eff": float(geom["w1_eff"]),
                "ramp_clamped": bool(geom["ramp_clamped"]),
                "feasible": bool(geom["feasible"]),
                "kappa_max": float(geom["kappa_max"]),
                # 'none' off the object branch; 'straight', 'arc' (the ramp
                # arc, ending where its length puts it) or 'pose_arc' (the
                # bezier, ending ON the target) on it.
                "object_shape": object_corridor_shape,
                "object_corridor_mode": str(
                    getattr(self, 'object_corridor_mode', 'off')),
                "ctrl_left": geom["ctrl_left"],
                "ctrl_right": geom["ctrl_right"],
                "Pend": [float(p_goal[0]), float(p_goal[1])],
            },
            "psiRef": float(psiEnd),
            # WHICH WAY ROUND, which psiRef alone cannot say. None for every
            # corridor that is not an active wall_turn; the signed rotation in
            # radians this corridor asks for, measured from psiStart, when it
            # is (the same dpsi_this as psiEnd and the S-curve). mpc_solver's
            # terminal yaw cost uses it INSTEAD OF wrapping psiRef onto the
            # shortest branch -- see the wall_turn branch above for why the
            # shortest branch is wrong at and beyond 180 degrees, and
            # test_turn_branch.py for what is pinned.
            "psiRefTurn": (None if turn_remaining is None
                           else float(turn_remaining)),
            # The length THIS corridor was actually built with (the
            # goal_pose branch clips it into [1.0, corr_L_base], so it is not
            # always corr_L_base). _corridor_lookahead derives the lookahead
            # from it -- see that method.
            "L": float(L),
            # See object_mode's own declaration at the top of this method.
            "objectMode": bool(object_mode),
            "psiStart": float(psiStart),
            "objectShape": object_corridor_shape,
            "objectTarget": (None if object_target is None
                             else [float(object_target[0]),
                                   float(object_target[1])]),
            "objectGoal": (None if object_goal_xy is None
                           else [float(object_goal_xy[0]),
                                 float(object_goal_xy[1])]),
            # None off the object branch and whenever the line is meaningless;
            # mpc_solver's w_line term is structurally absent when it is None.
            "targetLine": target_line,
            "t": float(dpsi),
            "Pend": p_goal,
            "dFront": float(d_front),
            "dpsi": float(dpsi),
        }

        # getattr, like every other new read in this method: the stand-ins
        # bind build_straight_corridor onto objects that carry only the fields
        # the geometry under test needs, and a bare self._reference_step makes
        # all of them raise instead of building a corridor. No previous
        # corridor to difference against is the same answer as no method.
        _ref_step = getattr(self, '_reference_step', None)
        corridor["refStep"] = (
            _ref_step(corridor) if _ref_step is not None
            else {'centreline_m': None, 'tangent_rad': None, 'pend_m': None})

        # ---- DEBUG: geometria del corridoio appena costruito ----
        self.get_logger().info(
            f'CORR/build | L={L:.2f} psiStart(live)={psiStart:+.4f} '
            f'psiEnd(frozen)={psiEnd:+.4f} '
            f'dpsi={dpsi:+.4f} halfWidth=[{halfWidth[0]:.3f}..{halfWidth[-1]:.3f}] '
            f'dFront={d_front:.2f}'
        )

        return corridor

    def _weights_for(self, corridor):
        """self.weights, with the arc's own cross-track split when it applies.

        Returns self.weights UNCHANGED for every corridor that is not an arc,
        so the straight and turn branches keep exactly the weight set they are
        tuned at and w_line stays inert (no target line, no rows).

        On an arc corridor the two cross-track terms own different curves, so
        each gets the weight its OWN tolerance derives:

            literal = rho / (STAGE_WEIGHT_REF_HORIZON * sigma^2)

        the same identity the rest of the stage set obeys -- see
        test_weight_set.py. rho is 1.0 for both, as it is for the w_corr these
        replace: one sigma of sustained offset is worth the same as it was,
        what changes is how much offset a sigma is.
        """
        # BOTH ARC SHAPES, for the same reason: 'arc' and 'pose_arc' are
        # anchored on the car, so the car sits exactly on its own centreline
        # and w_corr sees no error at the moment of the rebuild. That is what
        # makes w_line load-bearing rather than optional, and it is a property
        # of where the corridor STARTS -- which the two share -- not of how it
        # ends. 'straight' keeps the single-weight set, where the centreline
        # IS the target line and splitting would charge the same error twice.
        if corridor.get('objectShape') not in ('arc', 'pose_arc'):
            return self.weights
        weights = dict(self.weights)
        weights['w_corr'] = 1.0 / (
            STAGE_WEIGHT_REF_HORIZON * self.arc_corr_sigma_m ** 2)
        weights['w_line'] = 1.0 / (
            STAGE_WEIGHT_REF_HORIZON * self.arc_line_sigma_m ** 2)
        return weights

    # ── LLM2 MPC-parameter updates (two-llm-arch) ─────────────────────────
    def _llm2_setup(self):
        """
        Declare llm2_params_enabled; only when true, subscribe to /llm2/mpc_params.

        False (the default): nothing else is declared, no subscription and no
        publisher exist, and _llm2_apply_pending returns at once -- the
        controller is the one it was without this block. True: the whitelist
        (llm2_param_whitelist, llm2_param_min, llm2_param_max) is checked by
        llm2_params.build_whitelist, and a bad one refuses the start, as the
        reverse mechanisms do. Updates are then parked by
        _llm2_params_callback and applied by _llm2_apply_pending at the top of
        control_loop. The rules are in mpc_controller/llm2_params.py.
        """
        self._llm2_pending = None
        if not bool(self.declare_parameter(
                'llm2_params_enabled', get_value('llm2_params_enabled')).value):
            return
        names = self.declare_parameter(
            'llm2_param_whitelist', get_value('llm2_param_whitelist')).value
        mins = self.declare_parameter('llm2_param_min', get_value('llm2_param_min')).value
        maxs = self.declare_parameter('llm2_param_max', get_value('llm2_param_max')).value
        self._llm2_tunable = tunable_weights(self.weights)
        try:
            self._llm2_whitelist = build_whitelist(names, mins, maxs, self._llm2_tunable)
        except ValueError as exc:
            self.get_logger().fatal(f'LLM2 | STARTUP REFUSED: {exc}')
            raise
        current = {name: self.weights[self._llm2_tunable[name]]
                   for name in self._llm2_whitelist}
        planner = self.reverse_planner
        for warning in startup_warnings(
                self._llm2_whitelist, current,
                w_rev=planner.cfg.w_rev if planner is not None else None,
                vdes=self.vdes_default):
            self.get_logger().warn(f'LLM2 | {warning}')
        self._llm2_pending = PendingUpdate()
        self.llm2_applied_pub = self.create_publisher(MpcParamUpdate, APPLIED_PARAMS_TOPIC, 10)
        self.sub_llm2_params = self.create_subscription(
            MpcParamUpdate, LLM2_PARAMS_TOPIC, self._llm2_params_callback, 10)
        self.get_logger().info(
            f'LLM2 | parameter updates ON from {LLM2_PARAMS_TOPIC}, applied at the tick '
            f'boundary, echoed on {APPLIED_PARAMS_TOPIC}. Whitelist (declared units): '
            + ', '.join(f'{n} [{lo:g}, {hi:g}] now {current[n]:g}'
                        for n, (lo, hi) in self._llm2_whitelist.items()))

    def _llm2_params_callback(self, msg):
        """
        Validate an LLM2 update and park it for the next tick.

        Touches neither self.weights nor a parameter: applying is
        _llm2_apply_pending's job, at the tick boundary. An invalid update is
        rejected whole and logged, and does not displace a valid one already
        parked.
        """
        try:
            values = validate_update(msg.names, msg.values, self._llm2_whitelist)
        except ValueError as exc:
            self.get_logger().warn(
                f'LLM2 | REJECTED seq={msg.seq} source={msg.source!r}, whole update, '
                f'nothing applied: {exc}')
            return
        merged_over = self._llm2_pending.put(Update(int(msg.seq), str(msg.source), values))
        if merged_over is not None:
            self.get_logger().warn(
                f'LLM2 | seq={msg.seq} merged over seq={merged_over}, which no tick '
                f'had applied yet')

    def _llm2_apply_pending(self):
        """
        Apply the parked LLM2 update, if any. Called first thing in control_loop.

        Through the declared parameters (one set_parameters_atomically, so
        `ros2 param get /mpc_corr mpc_w_v` shows the live value) AND into
        self.weights, which is what every solve reads: MPC_corr reads the
        mpc_w_* parameters once, in __init__, and has no parameter callback,
        so the parameter write alone would change nothing. A refused parameter
        write applies nothing. Each application is echoed on
        /mpc/applied_params with the update's seq and logged old -> new.
        """
        pending = getattr(self, '_llm2_pending', None)
        if pending is None:
            return
        update = pending.take()
        if update is None:
            return
        result = self.set_parameters_atomically([
            Parameter(name, Parameter.Type.DOUBLE, value)
            for name, value in update.values.items()])
        if not result.successful:
            self.get_logger().error(
                f'LLM2 | seq={update.seq} NOT applied, parameter write refused: '
                f'{result.reason}')
            return
        old = {name: self.weights[self._llm2_tunable[name]] for name in update.values}
        weights = dict(self.weights)
        for name, value in update.values.items():
            weights[self._llm2_tunable[name]] = value
        self.weights = weights

        msg = MpcParamUpdate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.seq = update.seq
        msg.source = update.source
        msg.names = list(update.values)
        msg.values = list(update.values.values())
        self.llm2_applied_pub.publish(msg)

        changed = ', '.join(f'{name} {old[name]:g} -> {value:g}'
                            for name, value in update.values.items() if value != old[name])
        if changed:
            self.get_logger().info(
                f'LLM2 | applied seq={update.seq} ({update.source}): {changed}')
        else:
            self.get_logger().info(
                f'LLM2 | applied seq={update.seq} ({update.source}): no value changed '
                f'({len(update.values)} parameters)', throttle_duration_sec=30.0)

    def _reference_step(self, corridor):
        """How far this corridor's reference moved from the previous one.

        Three separable quantities, all in metres/radians, None on the first
        corridor of a move (there is nothing to difference against):

          centreline_m  max over this centreline of the distance to the
                        NEAREST point of the previous one. A curve-to-curve
                        separation, not a sample-to-sample one, so simply
                        re-laying the same geometry from a pose further along
                        it reads ~0 -- which is the point: sliding along the
                        reference is not a step, changing it is.
          tangent_rad   change in the corridor's tangent AT THE CAR, i.e. what
                        the stage heading cost starts asking for differently.
          pend_m        how far the terminal point moved.

        Never raises: this is diagnostics on the control path.
        """
        try:
            xc = np.asarray(corridor['xc'], dtype=float)
            yc = np.asarray(corridor['yc'], dtype=float)
            pend = np.asarray(corridor['Pend'], dtype=float)
            tan_now = math.atan2(float(corridor['ty'][0]), float(corridor['tx'][0]))
            prev = getattr(self, '_prev_corridor_ref', None)
            self._prev_corridor_ref = {
                'xc': xc, 'yc': yc, 'Pend': pend, 'tan': tan_now}
            if prev is None:
                return {'centreline_m': None, 'tangent_rad': None, 'pend_m': None}
            d = np.hypot(xc[:, None] - prev['xc'][None, :],
                         yc[:, None] - prev['yc'][None, :])
            return {
                'centreline_m': float(d.min(axis=1).max()),
                'tangent_rad': float(wrap_pi(tan_now - prev['tan'])),
                'pend_m': float(np.hypot(*(pend - prev['Pend']))),
            }
        except Exception as exc:  # noqa: BLE001 - diagnostics must not stop control
            self.get_logger().warn(
                f'reference step not measured: {type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0)
            return {'centreline_m': None, 'tangent_rad': None, 'pend_m': None}

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

    def _publish_campaign_status(self, info, solve_dt, solver):
        """/mpc/status for the test-campaign logger, once per solve.

        Carries this solve's predicted horizon (campaign_status.horizon_payload)
        alongside the convergence fields, which is how the horizon reaches a
        test folder: /mpc/solver_status has published the same prediction since
        96c6bbc, but the campaign logger subscribes here, not there.

        Never raises: this runs inside control_loop between the solve and the
        /drive publish, and a malformed info dict or a JSON error must cost a
        status message, not the command that follows it.
        """
        try:
            # ts and the frame ride along so the payload's horizon is readable
            # on its own: without the prediction step a reader cannot place
            # step k in time, and 'odom' is asserted here rather than assumed
            # downstream because it is the same frame, for the same reason, as
            # the /corridor polygon this logger draws the horizon against.
            payload = mpc_status_payload(info, solve_dt, solver,
                                         ts=self.ts_pred, frame_id='odom')
            self.campaign_status_pub.publish(String(data=json.dumps(payload)))
        except Exception as exc:  # noqa: BLE001 - diagnostics must not stop control
            self.get_logger().warn(
                f'/mpc/status not published: {type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0)

    def _publish_corridor_polygon(self, corridor):
        """/corridor for the test-campaign logger, once per corridor rebuild.

        Same frame as /mpc/corridor_markers ('odom', see
        _publish_corridor_markers for why) and the same walls, as one closed
        polygon. odom_topic names the pose estimate the walls were built
        from, so a consumer measuring clearance against another estimate can
        tell. Never raises, for the same reason as _publish_campaign_status.
        """
        try:
            self._corridor_seq += 1
            if getattr(self, 'active_odom_source', None) == 'sim':
                odom_topic = self.sub_odom_sim.topic_name
            else:
                odom_topic = self.sub_odom_hw.topic_name
            payload = corridor_payload(
                corridor, self._corridor_seq, frame_id='odom', source='mpc_corr',
                odom_topic=odom_topic)
            self.corridor_pub.publish(String(data=json.dumps(payload)))
        except Exception as exc:  # noqa: BLE001 - diagnostics must not stop control
            self.get_logger().warn(
                f'/corridor not published: {type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0)

    def _lookahead_clamp_length(self):
        """Corridor length at and below which the lookahead pins to the end.

        The lookahead is max(corr_lookahead_frac * L, this). The fractional
        term is 0.5 * L, always inside the corridor, so it never pins; this
        floor is the only thing that can, and it does so exactly when it
        reaches L. At the shipping geometry (N 20, ts_pred 0.2, vdes 0.5,
        margin 1.25) that is 2.5 m. getattr: the duck-typed stand-ins that
        bind this carry ts but no ts_pred, and for them the two are equal.

        WHY IT IS A METHOD AND NOT A CONSTANT. Two callers now need it:
        _corridor_lookahead, which applies it, and the object branch's arc
        switch, which must stop using the arc before Pend starts feeding the
        terminal cost. Writing 1.25 in the second place would leave the switch
        behind the moment N, ts, vdes or the margin moved.
        """
        return lookahead_clamp_length(
            self.N, getattr(self, 'ts_pred', self.ts), self.vdes,
            self.corr_lookahead_reach_margin)

    def _corridor_lookahead(self, corridor):
        """Derive compute_local_target's lookahead from the corridor length.

        It is derived rather than carried as an independent constant because
        two requirements, both of which used to live only in prose:

        1. It must be a fraction of the corridor length (corr_lookahead_frac,
           0.5 -- the reference's 1.5 on L 3.0), so the target sits inside the
           corridor and the arclength advance actually operates instead of
           clamping to the last index every cycle.
        2. It must lie BEYOND the horizon's physical reach, N*ts_pred*vdes,
           or the terminal cost changes character from "steer toward" to
           "arrive at".

        When the two disagree the reach floor wins and the mismatch is logged:
        that is a geometry that has been mis-configured, and it should say so
        rather than quietly degrade the way the 1.5/1.5 collision did.
        """
        L = float(corridor.get("L", self.corr_L_base))
        from_length = self.corr_lookahead_frac * L
        # The floor and the raw reach it is derived from: the warning below
        # quotes both, because "inside the horizon reach" and "below the floor"
        # are different statements and the fix differs.
        ts_pred = getattr(self, 'ts_pred', self.ts)
        horizon_reach = float(self.N) * float(ts_pred) * float(self.vdes)
        reach_floor = lookahead_clamp_length(
            self.N, ts_pred, self.vdes, self.corr_lookahead_reach_margin)

        lookahead = from_length
        if from_length < reach_floor:
            lookahead = reach_floor
            if corridor.get("objectMode", False):
                # SUPPRESSED FOR THE OBJECT MODE ONLY, and not because the
                # warnings are noisy -- because here they are WRONG.
                #
                # Both warnings below say the same thing in different words:
                # the lookahead reaches past the corridor, so the terminal cost
                # stops being a direction pull and becomes an arrival target,
                # and pref_nom pins to the corridor end every cycle. On every
                # other branch that means the geometry was mis-configured,
                # because those corridors END NOWHERE IN PARTICULAR -- the
                # goal_distance branch's end is an arbitrary corr_L_base along
                # a heading, and the goal_pose branch's end is floored at 1.0 m
                # and can sit past the goal. Pinning the terminal cost there is
                # a real defect and deserves to be shouted about.
                #
                # An object corridor ends AT THE GOAL. So "the lookahead
                # reaches past the end" means "the goal is within a lookahead",
                # pref_nom clamps to the goal and the terminal cost becomes
                # "arrive at the goal" -- which is precisely what is wanted for
                # the last stretch of an approach, and is the state EVERY
                # successful approach passes through. Warning here would fire
                # once per rebuild on every correct run, and a warning that
                # fires on success is one nobody reads when it matters.
                #
                # The condition is not lost: it goes out on
                # /mpc/object_status as the range r, from which it is exactly
                # recoverable, instead of into throttled prose.
                self.get_logger().debug(
                    f'TGT/object | lookahead {lookahead:.2f} >= corridor '
                    f'{L:.2f}: pref_nom clamps to the goal (expected on the '
                    'final approach)')
                return float(lookahead)
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
        # holds because 0.30 < the 0.64 m narrow half-width, not because
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
    def _on_campaign_status(self, msg):
        """Follow the test-campaign logger's open test, so snapshots land in it.

        The status carries campaign_dir / mission / test_id, which is the test
        folder. On "open" for a test this node is not already writing to, the
        previous per-test handle is closed and a new one opened in append mode;
        on "closed" (or a malformed status) it falls back to this node's own
        per-run file, so free driving outside a campaign is still recorded and
        still never overwrites anything.

        Never raises: this runs on the control node and a bad status message
        must cost a destination, not the node.
        """
        if not self.save_corridor_debug:
            return
        try:
            data = json.loads(msg.data)
            state = data.get('state')
            campaign_dir = data.get('campaign_dir')
            mission = data.get('mission')
            test_id = data.get('test_id')
            target = None
            if (state == 'open' and campaign_dir and mission and test_id):
                target = Path(campaign_dir) / str(mission) / str(test_id)
            if target == self._corridor_test_dir:
                return
            if self._corridor_test_file is not None:
                self._corridor_test_file.close()
                self._corridor_test_file = None
            self._corridor_test_dir = target
            if target is not None:
                target.mkdir(parents=True, exist_ok=True)
                self._corridor_test_file = open(
                    target / 'corridor_debug.jsonl', 'a', encoding='utf-8')
                self.get_logger().info(
                    f'corridor snapshots -> {target / "corridor_debug.jsonl"}')
            else:
                self.get_logger().info(
                    f'corridor snapshots -> {self.corridor_log_path} (no test open)')
        except Exception as exc:  # noqa: BLE001 - diagnostics must not stop control
            self.get_logger().warn(
                f'campaign status ignored: {type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0)

    def save_corridor_snapshot(self, corridor, obstacles_global):
        if not hasattr(self, 'corridor_log_file'):
            return

        record = {
            # Seconds since the epoch, so a snapshot can be lined up with the
            # test folder's own streams. There was no timestamp at all before,
            # which made these records impossible to align with anything.
            "t": self.get_clock().now().nanoseconds * 1e-9,
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

        # The open test's folder when there is one, this node's own per-run
        # file otherwise. Never both: a snapshot belongs to exactly one run.
        handle = self._corridor_test_file or self.corridor_log_file
        try:
            handle.write(json.dumps(record) + "\n")
            handle.flush()
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