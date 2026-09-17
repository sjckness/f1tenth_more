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
from f1tenth_behavior.mission.runtime import DetectionInfo, ObjectStatusSample

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
    # Live value of /perception/front_distance (f1tenth_perception's
    # front_clearance_node -- EMA-smoothed BACKGROUND distance from the ZED
    # depth ROI, detected objects removed by construction), meters, or None if
    # no message has arrived yet.
    #
    # DESPITE THE FIELD NAME THIS IS A WALL DISTANCE, NOT A CLEARANCE, and the
    # distinction is the whole point. front_clearance_node publishes both:
    # front_distance excludes detected objects (a person 40 cm ahead does not
    # lower it -- it reports the wall behind them), while its
    # /perception/front_clearance is min(background, nearest in-corridor
    # obstacle). A mission saying "stop 2 m from the wall" wants the former;
    # the latter would stop it 2 m from whatever object wandered into the
    # corridor. The name here (and the stop_condition type's own name) still
    # says "clearance" only because renaming the type is a schema change --
    # see check_stop_condition.py's module docstring for the full three-topic
    # list, including /costmap/front_clearance, which this used to read.
    #
    # Distinct from min_obstacle_distance: that's YOLO/obstacle_projector_
    # node's discrete-object distance -- the OPPOSITE quantity, kept as its
    # own stop_condition type (obstacle_distance_below) precisely so a mission
    # picks one deliberately.
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
    # object_reached (schema_version 4.0). object_status is the latest
    # /mpc/object_status; object_move_id is the wire id THIS move's handler
    # publishes, so a status belonging to any other move -- including the same
    # move of a previous run -- never satisfies it. The three limits come from
    # stack_params.yaml via CheckStopCondition. None/defaults for every caller
    # that does not track object moves, which then never satisfies it.
    object_status: Optional[ObjectStatusSample] = None
    object_move_id: Optional[str] = None
    object_reach_tol_m: float = 0.10
    object_reach_max_target_age_sec: float = 1.0
    object_status_max_gap_sec: float = 0.5
    # A latched stop counts as arrival once the measured speed is at or below
    # this [m/s]. See the object_reached branch of evaluate().
    object_rest_speed_mps: float = 0.05


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
        # ctx.front_clearance is None until the first /perception/front_
        # distance message arrives. front_clearance_node publishes one message
        # per depth frame unconditionally (never gated on a synchronizer, and
        # it holds its EMA rather than dropping the publish when a frame
        # yields no reading -- see that node's own module docstring), so None
        # here means specifically "no message received yet", not "clear
        # ahead", and continued silence means the node or the ZED is down
        # rather than "nothing found in range".
        #
        # A NON-POSITIVE VALUE IS THE PUBLISHER'S "NO READING" SENTINEL, NOT A
        # DISTANCE, and treating it as one is a stop at every threshold. front_
        # clearance_node's _publishable() emits -1.0 whenever its EMA has never
        # held a value, and _publish_too_close() emits -1.0 for front_distance
        # deliberately (something inside minimum stereo range leaves the
        # background genuinely unobserved). Its docstring states the contract:
        # "everything downstream should treat a negative value as absent rather
        # than as a distance." A bare `<` does the opposite -- -1.0 < any
        # threshold is True -- so the guard fired the instant the reading went
        # absent. That is exactly what a person standing close enough to fill
        # the ROI produces: their pixels are excluded as object, the surviving
        # background count falls under min_bg_pixels_for_reading, the EMA is
        # never seeded, and -1.0 goes on the wire while the actual wall sits
        # metres further back. The car stopped at the person and the reason was
        # here, not in the perception node.
        #
        # Rejecting it (rather than holding the last good value, or treating it
        # as 0) is the fail-safe direction FOR A GUARD: this stop_condition
        # decides when a move has ARRIVED, so an absent measurement must mean
        # "not yet", and the move stays bounded by its own timeout_sec. Genuine
        # proximity is not this type's job -- the emergency lane's LiDAR
        # IsProximityTooClose owns that, on a separate sensor, and is unaffected
        # by any of this.
        #
        # This did not bite while the type read /costmap/front_clearance:
        # costmap_boundary_node WITHHOLDS the publish when it has nothing to
        # report, so "absent" arrived as silence (the None branch above) and
        # never as a negative number on the wire.
        if ctx.front_clearance is None or ctx.front_clearance <= 0.0:
            return False
        return ctx.front_clearance < float(p['distance'])

    if t == 'object_reached':
        # ONE range source: mpc_corr's live r (distance to the target minus the
        # standoff, computed every control tick from the pose and the point it
        # is actually driving at). Arrival additionally requires that the
        # status is this move's, still arriving, not from a tripped refresh
        # watchdog, and about a target estimate no older than the limit --
        # a car at the right distance from where a person WAS is not there.
        s = ctx.object_status
        if s is None or ctx.object_move_id is None or s.move_id != ctx.object_move_id:
            return False
        if ctx.now - s.received_sec > ctx.object_status_max_gap_sec:
            return False
        if s.goal_watchdog:
            return False
        if s.target_age_s > ctx.object_reach_max_target_age_sec:
            return False
        if s.r <= ctx.object_reach_tol_m:
            return True
        # ...OR mpc_corr stopped on arrival and the car is at rest. mpc_corr
        # latches its stop when live r reaches object_reach_tol_m +
        # object_stop_distance_m, so the car comes to rest one braking
        # distance later -- at r near object_reach_tol_m, but on either side
        # of it, by the measured spread of the stopping distance plus up to one
        # control tick of travel past the trigger. Requiring r <= tol alone
        # would miss every stop that came to rest a few centimetres short, and
        # the move would then time out with the car parked at the gap. The
        # stop decision was made on this move's own live r, under the same
        # move-id, freshness, watchdog and target-age gates above; "at rest"
        # makes the status the mission records the resting one, not a braking
        # one.
        return s.stop_latched and abs(s.speed) <= ctx.object_rest_speed_mps

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
