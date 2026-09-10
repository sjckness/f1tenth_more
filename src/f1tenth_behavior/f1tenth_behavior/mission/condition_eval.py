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

# How many CONSECUTIVE satisfied ticks a condition needs before evaluate_
# debounced() reports it satisfied, when its stop_condition does not say.
#
# 1 -- i.e. no debounce, fire on the first satisfied tick -- and that default
# is load-bearing: it is exactly what every condition did before debouncing
# existed, so every mission already under missions/ behaves bit-identically
# without being touched. Only a stop_condition that explicitly asks for more
# gets more.
#
# f110_autonomy debounced its "wall" guard at 3 consecutive ticks
# (wall_count_needed = 3) against a noisy raw depth signal. The LLM path asks
# for the same 3 when it maps that guard (see llm/plan_translate.py); nothing
# else does.
DEFAULT_DEBOUNCE_TICKS = 1

# stop_condition types that honour a `debounce_ticks` param. Deliberately a
# set of one rather than "any type": debouncing a condition that is already
# an accumulation (orientation_delta) or an edge (goal_reached) either does
# nothing or actively delays it, and quietly accepting the field on those
# would advertise a behaviour they do not have.
DEBOUNCEABLE_TYPES = {'front_clearance'}


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
    # Live value of /mpc/min_obstacle_distance_forward -- the FORWARD-HALF-PLANE
    # counterpart of min_obstacle_distance above, from the same mpc_corr tick
    # and the same obstacle list (see MPC_corr.py's compute_forward_obstacle_
    # distance). Read only by an obstacle_distance_below whose own params say
    # `"forward_only": true`; that flag defaults to FALSE, so every existing
    # mission keeps reading the omnidirectional value and behaves exactly as
    # before.
    #
    # WHY BOTH EXIST rather than one corrected signal: min_obstacle_distance
    # has no heading term at all, so an object level with the rear axle counts
    # as much as one dead ahead -- wrong for "something is in the way", but it
    # is what every mission already under missions/ was written and tuned
    # against. Changing it in place would have altered those missions silently,
    # which the compatibility rule forbids. New callers (the LLM path's
    # front_object guard) opt in.
    #
    # None means "no message received yet", same as every other Optional
    # sensor field here -- not "nothing ahead", which is a large finite value.
    min_obstacle_distance_forward: Optional[float] = None


def debounce_ticks_for(condition: StopCondition) -> int:
    """How many CONSECUTIVE satisfied ticks `condition` needs before it counts
    as fired. DEFAULT_DEBOUNCE_TICKS (1, i.e. no debounce) unless the
    stop_condition explicitly asks for more AND its type is debounceable.

    Silently returns the default for a non-debounceable type carrying the
    field: mission_config.py is where a nonsensical combination should be
    rejected by name, not here in a per-tick hot path that has no way to
    report anything. Values below 1 are clamped up rather than treated as
    "fire before the condition is ever true".
    """
    if condition.type not in DEBOUNCEABLE_TYPES:
        return DEFAULT_DEBOUNCE_TICKS
    raw = condition.params.get('debounce_ticks', DEFAULT_DEBOUNCE_TICKS)
    try:
        ticks = int(raw)
    except (TypeError, ValueError):
        return DEFAULT_DEBOUNCE_TICKS
    return max(ticks, 1)


class ConditionDebouncer:
    """Consecutive-satisfied-tick latch for one caller's current condition.

    WHY THE STATE LIVES HERE AND NOT IN evaluate(): evaluate() is a pure
    function of (condition, ctx) and every one of its callers depends on that
    -- it is called from two behaviours and directly from tests, and a hidden
    per-condition counter inside it would make the same inputs return
    different answers depending on call history. So the counting is an
    explicit object the caller owns, resets and passes ticks to, while the
    POLICY (which types debounce, and what the default is) stays here beside
    the evaluator it belongs to.

    Owned per-caller, reset on every move change -- see CheckStopCondition,
    the only user today. HandleObjectAction deliberately has none: a
    resume_condition with no debouncer is evaluated exactly as it always was
    (DEFAULT_DEBOUNCE_TICKS == 1 means the two paths agree anyway).

    Ported from f110_autonomy's wall_count_needed, which existed because its
    "wall" guard read a raw, unfiltered depth value that could dip below the
    threshold for a single frame on noise alone.
    """

    def __init__(self):
        self._streak = 0

    def reset(self) -> None:
        """Drop any accumulated streak -- call on every move change, so a
        partial streak from the previous move cannot count toward this one."""
        self._streak = 0

    @property
    def streak(self) -> int:
        """Consecutive raw-satisfied ticks seen so far. Diagnostic only."""
        return self._streak

    def update(self, condition: StopCondition, raw: Optional[bool]) -> Optional[bool]:
        """Fold this tick's raw evaluate() result into the streak and return
        the DEBOUNCED answer.

        None passes straight through untouched and does NOT disturb the streak
        -- it means "this condition cannot be judged from here at all" (a stub
        type, or a signal this caller doesn't track), which is categorically
        different from "not satisfied this tick" and must not be counted as
        either a hit or a miss.
        """
        if raw is None:
            return None
        if not raw:
            self._streak = 0
            return False
        self._streak += 1
        return self._streak >= debounce_ticks_for(condition)


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
        # TWO SOURCES, selected by this condition's own `forward_only` flag:
        #
        #   false (DEFAULT) -- /mpc/min_obstacle_distance, omnidirectional.
        #     No heading term whatsoever: an obstacle level with the rear axle
        #     reads exactly like one dead ahead. This is what this condition
        #     has always done, so it stays the default and every mission
        #     already under missions/ keeps behaving identically.
        #
        #   true -- /mpc/min_obstacle_distance_forward, nearest obstacle in
        #     the FORWARD HALF-PLANE (MPC_corr.compute_forward_obstacle_
        #     distance's dot > 0 test). This is the honest reading of
        #     "something is in the way", and it is exactly what
        #     f110_autonomy's distance_to_front_object() computed for the
        #     guard "front_object" that maps onto this condition. The LLM
        #     path sets it (see llm/plan_translate.py); nothing else does.
        #
        # Explicit `is True` rather than truthiness, matching how
        # mission_config.py parses booleans: a stray `"forward_only": "no"`
        # should not quietly select the filtered source.
        if p.get('forward_only') is True:
            value = ctx.min_obstacle_distance_forward
        else:
            value = ctx.min_obstacle_distance
        if value is None:
            return False
        return value < float(p['distance'])

    if t == 'front_clearance':
        # Same shape as obstacle_distance_below above, different sensor
        # source -- see EvalContext.front_clearance's own comment.
        #
        # This type is also the only one that honours `debounce_ticks`, but
        # NOT here: debouncing is a property of a SEQUENCE of ticks, and this
        # function is deliberately pure. ConditionDebouncer (above) wraps this
        # result for callers that track ticks; the answer below is the raw,
        # single-tick one, which is exactly what that wrapper needs.
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
