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

    # Unreachable if the condition came from mission_config.parse_mission() (it
    # validates `type` against this same set) -- fail safe rather than crash the
    # BT tick if one somehow got here unvalidated.
    return None
