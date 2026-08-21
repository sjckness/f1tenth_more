"""Shared stop_condition evaluator.

Used by both CheckStopCondition (the current move's own stop_condition) and
HandleObjectAction (a hold's resume_condition, or the implicit object_cleared
fallback when resume_condition is omitted -- see runtime.HoldContext). One
evaluator, one place the config table's stop_condition.type semantics are
actually implemented, so the two callers can't drift apart.
"""

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from f1tenth_behavior.mission.mission_config import STUB_STOP_CONDITION_TYPES, StopCondition
from f1tenth_behavior.mission.runtime import DetectionInfo

# object_seen has no per-condition debounce/freshness field of its own in the
# schema (only object_cleared's debounce_sec is configurable) -- this is the
# assumption flagged back to the user: how long a sighting counts as "current"
# for object_seen (stop_condition) and ObjectSeen (BT condition) alike.
DEFAULT_SEEN_FRESHNESS_SEC = 1.0


@dataclass
class EvalContext:
    now: float  # time.monotonic()
    move_start_time: float
    move_start_xy: Optional[Tuple[float, float]]
    current_xy: Optional[Tuple[float, float]]
    detected_classes: Dict[str, DetectionInfo]
    min_obstacle_distance: Optional[float]
    # Live value of /costmap/front_clearance (f1tenth_costmap's
    # costmap_boundary_node -- nearest-occupied-cell extraction from slam_
    # toolbox's own /slam/map; retired f1tenth_perception's wall_detector_
    # node, see the dual-EKF + costmap-derived-MPC-boundaries pass), meters,
    # or None if no message has arrived yet. Distinct from
    # min_obstacle_distance: that's YOLO/obstacle_projector_node's discrete-
    # object distance, this is the nearest occupied map cell roughly ahead --
    # separate sensing paths, separate stop_condition types, deliberately
    # not merged.
    front_clearance: Optional[float]
    # The move's own top-level goal_distance, used as distance_reached's implicit
    # target when the stop_condition itself doesn't override it with its own
    # `distance` field.
    default_distance: Optional[float]
    # Per-current-move latch of mpc_corr's own /mpc/goal_reached (shared by its
    # distance-mode and pose-mode arrival, mode-agnostic on the wire) -- see
    # CheckStopCondition's own docstring for how this gets reset per move and why
    # a bare "last received value" isn't safe to use directly. None means the
    # caller doesn't track this (HandleObjectAction's resume_condition doesn't
    # -- see evaluate()'s goal_reached branch below), not "not yet received".
    goal_reached: Optional[bool] = None
    # Both new for orientation_delta (schema_version 2.0's "turn" step type),
    # both defaulted (must come after every non-defaulted field above, same
    # dataclass-ordering constraint that already put goal_reached last) --
    # HandleObjectAction's own EvalContext(...) call site (resume_condition)
    # does not supply either, same as goal_reached; remembering the break
    # that omission caused for front_clearance (a non-defaulted field there
    # broke that other construction site) is exactly why these two get
    # defaults from the start.
    #
    # current_yaw: live odometry yaw (radians, /odom's own convention),
    # captured by CheckStopCondition alongside current_xy. None until the
    # first odometry message arrives -- same "no message yet" meaning as
    # front_clearance's own None, not "yaw is exactly zero".
    current_yaw: Optional[float] = None
    # turn_start_yaw: this move's yaw at the moment it was first observed
    # (MissionRuntimeState.move_start_yaw, lazily captured the same way
    # move_start_xy already is). Kept for reference/logging, but NOT what
    # orientation_delta itself checks against any more -- see turn_accum_deg
    # below for why (naive current-minus-start, even wrapped, cannot represent
    # a turn of >=180 deg).
    turn_start_yaw: Optional[float] = None
    # turn_accum_deg: signed cumulative rotation (degrees) since this move
    # started, from CheckStopCondition's own per-odom-message unwrap-and-
    # accumulate (see that module's docstring) -- NOT a wrapped instantaneous
    # current-minus-start delta. That distinction is exactly the fix for the
    # "180 deg turn spins forever" bug: a wrapped delta (atan2(sin(x), cos(x)))
    # is mathematically bounded to (-180, 180] deg, so it can never reach, and
    # can only ever brush, a target of exactly 180 -- and for any target above
    # that (e.g. a 270 deg turn) it could NEVER be satisfied at all, since
    # continuing to rotate past the wrapped representation's own maximum
    # makes the wrapped magnitude fall back toward 0 rather than keep growing.
    # Accumulating each tick's own small (always well under 180 deg) wrapped
    # step, rather than re-deriving from absolute start/current yaw, has no
    # such ceiling. None until CheckStopCondition has seen at least one odom
    # message for this move -- same "no message yet" meaning as the other
    # Optional fields here.
    turn_accum_deg: Optional[float] = None


def evaluate(condition: StopCondition, ctx: EvalContext) -> Optional[bool]:
    """Returns True (satisfied), False (not yet), or None.

    None means either `condition.type` is one of STUB_STOP_CONDITION_TYPES
    (manual -- explicitly out of scope, no mechanism planned), or it's
    'goal_reached' from a caller that doesn't supply ctx.goal_reached
    (HandleObjectAction's resume_condition -- see its own branch below). Every
    caller must treat None as "does not satisfy on its own, and never will
    without external input" -- i.e. behave like an always-RUNNING condition,
    not crash and not silently succeed.
    """
    t = condition.type
    p = condition.params

    if t in STUB_STOP_CONDITION_TYPES:
        return None

    if t == 'goal_reached':
        # Real signal from mpc_corr (shared /mpc/goal_reached, distance-mode
        # and pose-mode both use it) via CheckStopCondition's per-move latch --
        # see EvalContext.goal_reached and CheckStopCondition's docstring. Not
        # per-move tolerance-aware: mpc_corr's own pose_goal_tolerance ROS
        # parameter governs arrival globally, not a per-move `tolerance` field
        # in the mission JSON (that field, if present, is currently unread --
        # see the README's config table note).
        if ctx.goal_reached is None:
            return None
        return bool(ctx.goal_reached)

    if t == 'distance_reached':
        if ctx.current_xy is None or ctx.move_start_xy is None:
            return False
        target = p.get('distance', ctx.default_distance)
        if target is None:
            # mission_config.py's validator rejects this combination at load
            # time, so reaching this at runtime would mean a bug upstream, not a
            # user config error -- fail closed (never satisfied) rather than
            # raise out of a BT tick.
            return False
        traveled = math.hypot(
            ctx.current_xy[0] - ctx.move_start_xy[0],
            ctx.current_xy[1] - ctx.move_start_xy[1],
        )
        return traveled >= float(target)

    if t == 'time_elapsed':
        return (ctx.now - ctx.move_start_time) >= float(p['duration_sec'])

    if t == 'object_seen':
        info = ctx.detected_classes.get(p['class'])
        if info is None or (ctx.now - info.last_seen) > DEFAULT_SEEN_FRESHNESS_SEC:
            return False
        min_conf = p.get('min_confidence')
        if min_conf is not None and info.score < float(min_conf):
            return False
        return True

    if t == 'object_cleared':
        info = ctx.detected_classes.get(p['class'])
        debounce = float(p.get('debounce_sec', 1.0))
        if info is None:
            return True  # never seen at all counts as "cleared"
        return (ctx.now - info.last_seen) >= debounce

    if t == 'obstacle_distance_below':
        if ctx.min_obstacle_distance is None:
            return False
        return ctx.min_obstacle_distance < float(p['distance'])

    if t == 'front_clearance':
        # Same shape as obstacle_distance_below above, different sensor
        # source -- see EvalContext.front_clearance's own comment.
        # ctx.front_clearance is None until the first /costmap/front_
        # clearance message arrives (costmap_boundary_node also publishes a
        # finite "clear at least this far" value, not silence, whenever
        # nothing occupied is found within range -- see that node's own
        # module docstring / costmap_boundary.front_clearance_from_
        # extraction -- so None here means specifically "no message
        # received yet", not "clear ahead").
        if ctx.front_clearance is None:
            return False
        return ctx.front_clearance < float(p['distance'])

    if t == 'orientation_delta':
        # No-odometry-yet case mirrors front_clearance's own None handling:
        # never satisfied, never crashes, never false-triggers.
        #
        # Deliberately NOT a wrapped current-minus-start delta (that was the
        # original implementation, and the bug: atan2(sin(x), cos(x)) bounds
        # its result to (-180, 180] deg, so a target of exactly 180 could only
        # ever be brushed, never reliably landed on tick-to-tick, and a target
        # above 180 could never be satisfied at all -- see turn_accum_deg's
        # own comment on EvalContext for the full explanation). Using the
        # accumulated, unwrapped rotation instead has no such ceiling.
        if ctx.turn_accum_deg is None:
            return False
        # >=, not ==: ticks land on whatever the odometry rate happens to
        # produce, essentially never exactly on the target -- same reasoning
        # obstacle_distance_below/front_clearance's own threshold checks use.
        return abs(ctx.turn_accum_deg) >= abs(float(p['value']))

    # Unreachable if the condition came from mission_config.parse_mission() (it
    # validates `type` against this same set) -- fail safe rather than crash the
    # BT tick if one somehow got here unvalidated.
    return None
