"""Mission config schema, parser, and validator.

See f1tenth_behavior/README.md's "Mission config" section for the authoritative
field-by-field table this mirrors -- keep the two in sync if either changes.

Deliberately has no rclpy/ROS dependency at all: this is pure JSON-in,
dataclasses-out, so it's usable/testable standalone. mission/loader.py is the
ROS-facing wrapper (parameter + topic) that calls load_mission_file() below.
"""

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from f1tenth_behavior.mission.object_classes import OBJECT_CLASSES

STOP_CONDITION_TYPES = {
    'distance_reached', 'goal_reached', 'time_elapsed', 'object_seen',
    'object_cleared', 'obstacle_distance_below', 'front_clearance',
    'orientation_delta', 'manual', 'object_reached',
}

# schema_version "4.0" adds the go_to_object step (ObjectSpec) and its
# object_reached stop_condition. The terminal-last-move rule that "3.0"
# introduced applies to every version from "3.0" on.
TERMINAL_REQUIRED_VERSIONS = {'3.0', '4.0'}
ON_OBJECT_ACTIONS = {
    'stop_and_hold', 'reduce_speed', 'reduce_speed_for', 'abort_mission',
    'skip_to_move', 'log_only',
}
# 'stop' added alongside the "turn" step type / schema_version 2.0 -- a third
# on_timeout behavior distinct from the two that already existed: neither
# aborts the whole mission (like 'abort') nor jumps to the next move (like
# 'skip'), it just halts the car in place and leaves the mission sitting on
# the current (now-timed-out) move indefinitely. See CheckStopCondition's own
# timeout-handling branch for the implementation.
ON_TIMEOUT_VALUES = {'abort', 'skip', 'stop'}
# Only one reference frame for a turn step's heading tracking is supported
# today (see TurnSpec.reference) -- kept as a real field, not hardcoded,
# specifically so a future reference (e.g. a fused/filtered heading source
# distinct from whichever /odom-equivalent topic localization_source
# currently selects) can be added without a schema change.
VALID_TURN_REFERENCES = {'odometry_orientation'}

# schema_version "3.0"'s "drive" step: the driving MODE a drive move commands.
# Ported 1:1 from f110_autonomy's own phase vocabulary (the LLM planner has
# always emitted these two names -- see llm_planner_node.SYSTEM_PROMPT's
# VALID_MODES) so the planner's JSON maps onto this schema with nothing
# invented in between. "straight" holds the move's own start heading;
# "wall_turn" turns by turn_sign * turn_mag_deg off that same start heading
# and then holds the result. Neither carries a target -- see DriveSpec.
DRIVE_MODES = {'straight', 'wall_turn'}

# stop_condition.type values that are explicitly out of scope for this pass
# (manual advance -- no mechanism planned). condition_eval.evaluate() returns
# None for these; every consumer of that function has to treat None as "never
# satisfies on its own", not crash. goal_reached is NOT a stub anymore -- since
# mpc_corr gained a /mpc/goal_pose input (and publishes /mpc/goal_reached on
# arrival for both its distance- and pose-mode goals), CheckStopCondition
# tracks that signal for real; see condition_eval.evaluate()'s own goal_reached
# branch and CheckStopCondition's docstring. It still evaluates to None when
# used as HandleObjectAction's resume_condition, though (that caller doesn't
# track the live signal) -- flagged there, not silent.
#
# front_clearance is likewise NOT a stub -- added alongside f1tenth_perception's
# wall_detector_node (RANSAC plane segmentation), which was the FIRST real
# backing sensor source for it: this type did not exist in this schema at all
# before that integration (there was no prior stub, real or otherwise, for the
# name "front_clearance" specifically -- confirmed by reading this set before
# adding it). The backing source has been swapped twice since, each time a
# source swap rather than a second integration -- wall_detector_node ->
# f1tenth_costmap's costmap_boundary_node (/costmap/front_clearance, the
# dual-EKF + costmap-derived-MPC-boundaries pass) -> f1tenth_perception's
# front_clearance_node, whose /perception/front_DISTANCE it reads today: a
# mission asking to stop N metres from a wall needs the wall measured on this
# frame rather than read out of an already-converged SLAM map, AND needs
# detected objects excluded from that measurement, which is what
# front_distance does and what that node's similarly-named
# /perception/front_clearance (min(background, nearest obstacle)) explicitly
# does not. THE TYPE NAME IS NOW A MISNOMER and is kept only because renaming
# it would invalidate every mission JSON using it. front_clearance itself has
# been a real (non-stub) type continuously since the original addition. See
# CheckStopCondition's own docstring for the full three-topic disambiguation
# and how the live value reaches condition_eval.evaluate().
#
# orientation_delta (schema_version 2.0, alongside the "turn" step type) is
# also real, not a stub -- backed by CheckStopCondition's own /odom
# subscription (already owned there for distance_reached; extended to also
# extract yaw) rather than a new sensor. Turn-exclusive by construction (see
# _parse_move's own validation below): using it as any other step's
# stop_condition, or as a resume_condition, is rejected/never-satisfies
# respectively -- see condition_eval.py's own orientation_delta branch for why
# the resume_condition case specifically returns None rather than raising.
STUB_STOP_CONDITION_TYPES = {'manual'}
# on_object actions whose control effect depends on a vdes-override mechanism
# that doesn't exist yet.
STUB_ON_OBJECT_ACTIONS = {'reduce_speed', 'reduce_speed_for'}


class MissionConfigError(ValueError):
    """Raised on any schema violation. Callers must treat this (and, from
    load_mission_file(), OSError/json.JSONDecodeError too) as "reject the whole
    mission" -- never install a partially-parsed MissionConfig over whatever
    mission was already loaded and running.
    """


@dataclass(frozen=True)
class GoalPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StopCondition:
    type: str
    # Every extra field from the JSON verbatim, rather than one dataclass per
    # type -- the field set genuinely differs per type (see the config table),
    # and a 7-way tagged union of near-empty dataclasses wouldn't be any clearer
    # than each evaluator reading params[...] for the fields its own type needs.
    params: Dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class OnObjectAction:
    # JSON key is "class" -- "class" itself is a reserved word, hence "cls".
    cls: str
    action: str
    params: Dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class TurnSpec:
    """A "turn" step's own fields -- see Move.turn. Sibling to (not nesting)
    Move's own stop_condition, same shape convention goal_distance/goal_pose
    already use (stop_condition always lives at the Move level, not inside a
    per-type sub-object) -- the task description that introduced this showed
    stop_condition nested under "turn" in its own field list; kept it a
    sibling instead for schema consistency with every other step type."""
    heading_delta_deg: float  # signed: + = left/CCW, - = right/CW (right-hand rule)
    speed: float               # linear speed while turning [m/s]
    steering: str               # "full_lock" or "partial:<deg>" -- format validated
                                 # at load time, see _parse_turn_spec below
    reference: str = 'odometry_orientation'  # only supported value today, see
                                              # VALID_TURN_REFERENCES


@dataclass(frozen=True)
class DriveSpec:
    """A "drive" step's own fields -- see Move.drive, schema_version 3.0.

    THE POINT OF THIS TYPE, stated once so nobody re-derives it: a drive step
    carries NO GOAL. goal_distance/goal_pose/turn each name a target the
    controller drives toward and can decide it has REACHED; drive names only
    a MODE, and the move ends when and only when the sibling stop_condition
    fires. That makes it the first goal shape whose duration is not knowable
    at load time, and the reason mpc_corr's own goal_drive path never sets
    goal_reached, never publishes /mpc/goal_reached and never self-terminates
    (see MPC_corr.py's goal_drive_callback).

    This is f110_autonomy's phase model made first-class. That stack's LLM
    planner -- still ours -- has always emitted mode + guard with no goal;
    until this version the only way to land one of its phases in this schema
    was to fabricate a goal_distance of 50.0 m and hope the guard fired first
    (see llm/plan_translate.py's own history). Nothing is invented here any
    more: mode maps to mode, and the guard maps to the stop_condition.

    Sibling to (not nesting) Move's own stop_condition -- same convention
    goal_distance/goal_pose/turn already follow.
    """
    mode: str                  # one of DRIVE_MODES
    # Turn direction, matching TurnSpec.heading_delta_deg's sign convention
    # and the LLM planner's own turn_sign exactly: + = left/CCW, - = right/CW.
    # Meaningless (and ignored, not rejected) when mode == "straight", which
    # is why it has a default rather than being required alongside mode.
    turn_sign: float = 0.0
    # Unsigned turn magnitude in DEGREES -- turn_sign carries the direction.
    # 0.0 means "use mpc_corr's own drive_default_turn_mag_deg" (90.0), the
    # sentinel DriveCommand.msg documents.
    #
    # DEGREES, not radians, deliberately: every other angle in this schema is
    # in degrees (TurnSpec.heading_delta_deg, orientation_delta's own value),
    # and the LLM planner's radians are converted exactly once, at the
    # plan_translate boundary. One unit per layer, converted at one seam.
    #
    # f110_autonomy never read a magnitude off the phase at all -- its
    # turn_mag was a hardcoded pi/2 -- so a plan asking for 180 degrees got
    # 90. That is a bug this port fixes rather than reproduces.
    turn_mag_deg: float = 0.0
    # Linear speed [m/s] for the duration of the move. 0.0 means "use
    # mpc_corr's own self.vdes".
    speed: float = 0.0
    # Total obstacle standoff [m] for the duration of the move -- the quantity
    # mpc_corr spends as car_radius + avoidance_margin. Reproduces
    # f110_autonomy's terminal-stop d_safe relaxation
    # (d_safe = max(0, min(dmin, stop_at - obs_margin - 0.1))), whose purpose
    # is to stop the obstacle penalty pushing the car AROUND the object it was
    # commanded to stop in front of.
    #
    # None (the default), not a negative number, means "leave mpc_corr's own
    # standoff alone" -- 0.0 is a real, meaningful value (no standoff at all),
    # so "unset" needs a representation distinct from any legal value. The
    # negative sentinel DriveCommand.msg uses exists only because a ROS msg
    # float32 field cannot be null; the conversion happens in
    # publish_move_goal.py, not here.
    # tuned against legacy height radii, re-validate on floor
    approach_d_safe: Optional[float] = None


@dataclass(frozen=True)
class ObjectSpec:
    """A "go_to_object" step's own fields -- see Move.go_to_object, schema 4.0.

    Drive to the nearest confirmed semantic track of `target_class` and stop
    `standoff_m` short of it. The move's life is owned by the mission's object
    handler (behaviours/go_to_object.py): it acquires a track, follows it,
    keeps the last point through a short loss, and ends the move with one of
    the outcomes reached / target_not_found / target_lost /
    target_unreachable / timeout. mpc_corr only drives at the point it is
    given (/mpc/goal_object) and reports the approach geometry back
    (/mpc/object_status).

    Constraints enforced at load time, by name (see _parse_move): the
    stop_condition must be object_reached, timeout_sec is required, and
    on_object is not allowed -- the thing being approached is itself a
    detected object, and an on_object hold would end the approach (a hold ends
    an object move in mpc_corr; it does not pause it).
    """

    target_class: str            # one of object_classes.OBJECT_CLASSES
    speed: float                 # approach speed ceiling [m/s], > 0
    acquire_timeout_sec: float   # no track of the class within this -> target_not_found
    standoff_m: float = 1.0      # metres short of the target, > 0
    lost_grace_sec: float = 1.5  # keep the last point this long after losing the track


@dataclass(frozen=True)
class Move:
    id: str
    stop_condition: StopCondition
    goal_distance: Optional[float] = None
    goal_pose: Optional[GoalPose] = None
    turn: Optional[TurnSpec] = None
    # The fourth, OPEN-ENDED goal shape (schema_version 3.0) -- see DriveSpec
    # for what makes it different in kind from the three above, not just in
    # fields. Still exactly one of the four per move (_parse_move's own
    # sum(...) == 1).
    drive: Optional[DriveSpec] = None
    # The fifth goal shape (schema_version 4.0) -- see ObjectSpec. Exactly one
    # of the five per move.
    go_to_object: Optional[ObjectSpec] = None
    vdes: Optional[float] = None
    on_object: List[OnObjectAction] = field(default_factory=list)
    # Optional as of schema_version 2.0 (previously always required) --
    # None means no per-move timeout is enforced (CheckStopCondition's own
    # timeout branch skips the check entirely rather than treating None as
    # "already expired"). Existing 1.x missions always set this explicitly,
    # so relaxing the requirement doesn't change how any of them parse or
    # behave -- see parse_mission()'s own backward-compat note.
    timeout_sec: Optional[float] = None
    on_timeout: str = 'abort'
    # schema_version 3.0. True means: when this move's stop_condition fires,
    # the mission COMPLETES rather than advancing, and the car latches at
    # vdes = 0.
    #
    # THIS FLAG DECLARES BEHAVIOUR, IT DOES NOT IMPLEMENT IT -- read this
    # before adding a latch anywhere. AdvanceMove's existing last-move branch
    # already does exactly this and has since schema 1.0: it calls
    # MissionRuntimeState.complete() and publishes /mpc/hold(True), and
    # mpc_corr's hold path publishes zero drive while touching nothing else
    # (MPC_corr.py's control_loop, first branch). So `terminal` is a
    # LOAD-TIME ASSERTION that the mission author knew the last move ends the
    # run, not a second mechanism. Building a parallel latch here would give
    # "stop" two authorities, which is precisely the failure this schema
    # version exists to remove.
    #
    # It is the schema word for f110_autonomy's stop_at/stop_at_distance,
    # whose implementation was an if/elif/ELSE against the guard: a phase
    # carrying either NEVER evaluated its guard at all and simply latched
    # vdes = 0 where it stood. Ours evaluates the stop_condition normally and
    # then completes -- same outcome for a well-formed last move, but the
    # guard still runs, still scores, and still logs.
    #
    # Validation (_parse_move + parse_mission): legal ONLY on the last move,
    # and REQUIRED on the last move -- but the requirement applies to
    # schema_version "3.0" missions ONLY. Pre-3.0 missions neither gain the
    # requirement nor change behaviour; see parse_mission()'s own
    # compatibility note.
    terminal: bool = False


@dataclass(frozen=True)
class MissionConfig:
    mission_id: str
    moves: List[Move]
    # "1.0" (the implicit, pre-existing behavior) if the JSON omits this field
    # entirely -- see parse_mission(). "2.0" is the first version that knows
    # about "turn" steps / orientation_delta / optional timeout_sec; "3.0"
    # adds the "drive" step (DriveSpec) and Move.terminal; "4.0" adds the
    # "go_to_object" step (ObjectSpec) and the object_reached stop_condition.
    #
    # MOSTLY informational bookkeeping rather than a feature flag: every new
    # FIELD stays available regardless of the version declared (a
    # schema_version "1.0" mission that happened to use a "turn" or a "drive"
    # step parses and runs exactly like a "2.0"/"3.0" one), because none of
    # them is structurally incompatible with older missions that simply don't
    # use them.
    #
    # THE ONE EXCEPTION, and the only thing anywhere gated on this value:
    # parse_mission() requires the last move to set terminal: true when this
    # is exactly "3.0". That rule cannot be universal without invalidating
    # every mission written before the field existed -- see its own comment
    # in parse_mission().
    schema_version: str = '1.0'

    def index_of(self, move_id: str) -> Optional[int]:
        for i, m in enumerate(self.moves):
            if m.id == move_id:
                return i
        return None


def _require(cond: bool, message: str) -> None:
    if not cond:
        raise MissionConfigError(message)


def _parse_stop_condition(raw: object, where: str) -> StopCondition:
    _require(isinstance(raw, dict), f'{where}: stop_condition must be an object')
    t = raw.get('type')
    _require(
        t in STOP_CONDITION_TYPES,
        f'{where}: stop_condition.type={t!r} not in {sorted(STOP_CONDITION_TYPES)}',
    )
    params = {k: v for k, v in raw.items() if k != 'type'}

    if t == 'time_elapsed':
        _require('duration_sec' in params, f'{where}: time_elapsed requires duration_sec')
    elif t == 'object_seen':
        _require('class' in params, f'{where}: object_seen requires class')
    elif t == 'object_cleared':
        _require('class' in params, f'{where}: object_cleared requires class')
    elif t == 'obstacle_distance_below':
        _require('distance' in params, f'{where}: obstacle_distance_below requires distance')
        # Optional, defaults to false = the omnidirectional
        # /mpc/min_obstacle_distance this condition has always read. true
        # selects mpc_corr's forward-half-plane counterpart instead -- see
        # condition_eval.py's own branch for both sources. Explicit bool only:
        # `"forward_only": "yes"` is a mistake worth naming rather than
        # silently reading as false.
        if 'forward_only' in params:
            _require(
                params['forward_only'] is True or params['forward_only'] is False,
                f'{where}: obstacle_distance_below forward_only must be a JSON boolean '
                f'(got {params["forward_only"]!r})',
            )
    elif t == 'front_clearance':
        _require('distance' in params, f'{where}: front_clearance requires distance')
        # Optional, defaults to condition_eval.DEFAULT_DEBOUNCE_TICKS (1 --
        # fire on the first satisfied tick, i.e. exactly today's behaviour).
        # Only front_clearance honours it; see condition_eval.
        # DEBOUNCEABLE_TYPES for why the set is deliberately that narrow.
        if 'debounce_ticks' in params:
            dt = params['debounce_ticks']
            _require(
                isinstance(dt, int) and not isinstance(dt, bool) and dt >= 1,
                f'{where}: front_clearance debounce_ticks must be an integer >= 1 '
                f'(got {dt!r})',
            )
    elif t == 'object_reached':
        # No fields: the reach tolerance and the maximum target age are
        # stack-wide (object_reach_tol_m / object_reach_max_target_age_sec in
        # stack_params.yaml), and go_to_object-exclusivity needs the sibling
        # step, so it is checked in _parse_move.
        _require(
            not params,
            f'{where}: object_reached takes no fields (got {sorted(params)}); the '
            'tolerance is object_reach_tol_m in stack_params.yaml',
        )
    elif t == 'orientation_delta':
        _require('value' in params, f'{where}: orientation_delta requires value')
        _require(
            isinstance(params['value'], (int, float)) and not isinstance(params['value'], bool),
            f'{where}: orientation_delta value must be numeric',
        )
        # The turn-exclusivity check (this type may only appear on a "turn"
        # step) and the heading_delta_deg/value consistency check both need
        # the sibling `turn` field, which isn't visible from here -- see
        # _parse_move's own validation, after both this and the turn spec
        # have been parsed.

    # Type-scoped optional fields, rejected by name on any OTHER type rather
    # than silently ignored. Both were added late enough that a mission
    # carrying one on the wrong type is far more likely a copy-paste from a
    # neighbouring condition than a deliberate no-op, and a field that
    # advertises behaviour it does not have is worse than an error.
    _require(
        'debounce_ticks' not in params or t == 'front_clearance',
        f'{where}: debounce_ticks is only meaningful on a front_clearance '
        f'stop_condition (got type={t!r})',
    )
    _require(
        'forward_only' not in params or t == 'obstacle_distance_below',
        f'{where}: forward_only is only meaningful on an obstacle_distance_below '
        f'stop_condition (got type={t!r})',
    )

    return StopCondition(type=t, params=params)


def _parse_turn_spec(raw: object, where: str) -> TurnSpec:
    _require(isinstance(raw, dict), f'{where}: turn must be an object')

    heading_delta_deg = raw.get('heading_delta_deg')
    _require(
        isinstance(heading_delta_deg, (int, float)) and not isinstance(heading_delta_deg, bool),
        f'{where}: turn.heading_delta_deg is required and must be numeric',
    )

    speed = raw.get('speed')
    _require(
        isinstance(speed, (int, float)) and not isinstance(speed, bool) and speed > 0.0,
        f'{where}: turn.speed is required and must be a positive number',
    )

    steering = raw.get('steering')
    _require(
        isinstance(steering, str) and _is_valid_steering(steering),
        f'{where}: turn.steering must be "full_lock" or "partial:<deg>" (got {steering!r})',
    )

    reference = raw.get('reference', 'odometry_orientation')
    _require(
        reference in VALID_TURN_REFERENCES,
        f'{where}: turn.reference={reference!r} not in {sorted(VALID_TURN_REFERENCES)}',
    )

    return TurnSpec(
        heading_delta_deg=float(heading_delta_deg), speed=float(speed),
        steering=steering, reference=reference,
    )


def _parse_drive_spec(raw: object, where: str) -> DriveSpec:
    """Parse+validate a "drive" step's own sub-object (schema_version 3.0).

    Same explicit isinstance discipline _parse_turn_spec above already uses,
    and for the same reason mission/loader.py records: JSON is not typed, and
    `"speed": "0.5"` or `"turn_mag_deg": true` must be a load-time rejection
    with a specific message rather than a TypeError somewhere downstream in a
    BT tick. bool is excluded explicitly everywhere a number is wanted --
    bool IS an int in Python, so `isinstance(True, (int, float))` is True and
    a bare check would silently accept it.
    """
    _require(isinstance(raw, dict), f'{where}: drive must be an object')

    mode = raw.get('mode')
    _require(
        mode in DRIVE_MODES,
        f'{where}: drive.mode={mode!r} not in {sorted(DRIVE_MODES)}',
    )

    # turn_sign/turn_mag_deg are REQUIRED for wall_turn and meaningless for
    # straight. A straight step that carries them anyway is accepted (the
    # values are simply unused, exactly as mpc_corr ignores them on the wire)
    # rather than rejected -- the LLM planner emits mode/turn_sign as a pair
    # habitually, and rejecting a harmless extra field would fail plans that
    # are otherwise perfectly executable.
    turn_sign = raw.get('turn_sign', 0.0)
    _require(
        isinstance(turn_sign, (int, float)) and not isinstance(turn_sign, bool),
        f'{where}: drive.turn_sign must be numeric (got {type(turn_sign).__name__})',
    )
    turn_sign = float(turn_sign)

    turn_mag_deg = raw.get('turn_mag_deg', 0.0)
    _require(
        isinstance(turn_mag_deg, (int, float)) and not isinstance(turn_mag_deg, bool),
        f'{where}: drive.turn_mag_deg must be numeric '
        f'(got {type(turn_mag_deg).__name__})',
    )
    turn_mag_deg = float(turn_mag_deg)
    _require(
        turn_mag_deg >= 0.0,
        f'{where}: drive.turn_mag_deg must be >= 0 (it is a MAGNITUDE -- put the '
        'direction in drive.turn_sign, not in its sign)',
    )

    if mode == 'wall_turn':
        _require(
            turn_sign in (-1.0, 1.0),
            f'{where}: drive.mode "wall_turn" requires drive.turn_sign of exactly '
            f'-1.0 (right/CW) or +1.0 (left/CCW), got {turn_sign!r}; there is no '
            'direction to turn in otherwise',
        )

    speed = raw.get('speed', 0.0)
    _require(
        isinstance(speed, (int, float)) and not isinstance(speed, bool),
        f'{where}: drive.speed must be numeric (got {type(speed).__name__})',
    )
    speed = float(speed)
    _require(
        speed >= 0.0,
        f'{where}: drive.speed must be >= 0 (0 means "use the controller default", '
        'see DriveSpec.speed)',
    )

    # Absent -> None ("leave mpc_corr's standoff alone"), which is NOT the
    # same as 0.0 (a real request for no standoff at all). See
    # DriveSpec.approach_d_safe.
    approach_d_safe = None
    if raw.get('approach_d_safe') is not None:
        ads = raw['approach_d_safe']
        _require(
            isinstance(ads, (int, float)) and not isinstance(ads, bool),
            f'{where}: drive.approach_d_safe must be numeric '
            f'(got {type(ads).__name__})',
        )
        approach_d_safe = float(ads)
        _require(
            approach_d_safe >= 0.0,
            f'{where}: drive.approach_d_safe must be >= 0 (it is a standoff '
            'distance; omit the field entirely to keep the controller default)',
        )

    return DriveSpec(
        mode=mode, turn_sign=turn_sign, turn_mag_deg=turn_mag_deg,
        speed=speed, approach_d_safe=approach_d_safe,
    )


def _require_positive_number(raw: dict, key: str, where: str, default=None) -> float:
    value = raw.get(key, default)
    _require(
        isinstance(value, (int, float)) and not isinstance(value, bool),
        f'{where}: go_to_object.{key} is required and must be numeric'
        if default is None else f'{where}: go_to_object.{key} must be numeric',
    )
    _require(value > 0.0, f'{where}: go_to_object.{key} must be > 0 (got {value!r})')
    return float(value)


def _parse_object_spec(raw: object, where: str) -> ObjectSpec:
    """Parse+validate a "go_to_object" step's own sub-object (schema 4.0)."""
    _require(isinstance(raw, dict), f'{where}: go_to_object must be an object')
    known = {'target_class', 'standoff_m', 'speed', 'acquire_timeout_sec', 'lost_grace_sec'}
    unknown = sorted(set(raw) - known)
    _require(not unknown, f'{where}: go_to_object has unknown field(s) {unknown}')

    target_class = raw.get('target_class')
    _require(
        isinstance(target_class, str) and target_class in OBJECT_CLASSES,
        f'{where}: go_to_object.target_class={target_class!r} is not a class the '
        'configured detector produces (object_classes.OBJECT_CLASSES, regenerated by '
        'tools/gen_intent_target_enum.py)',
    )
    speed = _require_positive_number(raw, 'speed', where)
    acquire_timeout_sec = _require_positive_number(raw, 'acquire_timeout_sec', where)
    standoff_m = _require_positive_number(raw, 'standoff_m', where, default=1.0)

    lost_grace_sec = raw.get('lost_grace_sec', 1.5)
    _require(
        isinstance(lost_grace_sec, (int, float)) and not isinstance(lost_grace_sec, bool)
        and lost_grace_sec >= 0.0,
        f'{where}: go_to_object.lost_grace_sec must be a number >= 0 (got {lost_grace_sec!r})',
    )
    return ObjectSpec(
        target_class=target_class, speed=speed, acquire_timeout_sec=acquire_timeout_sec,
        standoff_m=standoff_m, lost_grace_sec=float(lost_grace_sec),
    )


def _is_valid_steering(value: str) -> bool:
    """"full_lock", or "partial:<deg>" where <deg> parses as a real number
    (sign allowed, e.g. "partial:15" or "partial:15.5") -- mirrors the exact
    format mpc_corr.py's own goal_turn_callback expects to parse back out on
    the wire (see that method's docstring), so a mission that passes this
    check is guaranteed not to hit its "unparseable steering" fallback later.
    """
    if value == 'full_lock':
        return True
    if not value.startswith('partial:'):
        return False
    try:
        float(value.split(':', 1)[1])
    except ValueError:
        return False
    return True


def _parse_on_object(raw: object, where: str) -> OnObjectAction:
    _require(isinstance(raw, dict), f'{where}: on_object entry must be an object')
    cls = raw.get('class')
    _require(isinstance(cls, str) and cls, f'{where}: on_object entry requires a non-empty class')
    action = raw.get('action')
    _require(
        action in ON_OBJECT_ACTIONS,
        f'{where}: on_object action={action!r} not in {sorted(ON_OBJECT_ACTIONS)}',
    )
    params = {k: v for k, v in raw.items() if k not in ('class', 'action')}

    if action == 'reduce_speed':
        _require('factor' in params, f'{where}: reduce_speed requires factor')
    elif action == 'reduce_speed_for':
        _require(
            'factor' in params and 'duration_sec' in params,
            f'{where}: reduce_speed_for requires factor and duration_sec',
        )
    elif action == 'abort_mission':
        _require('reason' in params, f'{where}: abort_mission requires reason')
    elif action == 'skip_to_move':
        _require('move_id' in params, f'{where}: skip_to_move requires move_id')
    elif action == 'log_only':
        _require('message' in params, f'{where}: log_only requires message')

    if 'resume_condition' in params and params['resume_condition'] is not None:
        params = dict(params)
        params['resume_condition'] = _parse_stop_condition(
            params['resume_condition'], f'{where} (on_object {cls}/{action}).resume_condition')
        _require(
            params['resume_condition'].type != 'object_reached',
            f'{where}: object_reached is not a valid resume_condition -- it is the '
            'stop_condition of a go_to_object step only',
        )

    return OnObjectAction(cls=cls, action=action, params=params)


def _parse_move(raw: object, where: str) -> Move:
    _require(isinstance(raw, dict), f'{where}: move must be an object')
    move_id = raw.get('id')
    _require(isinstance(move_id, str) and move_id, f'{where}: move requires a non-empty id')
    where = f'{where} (id={move_id})'

    has_distance = raw.get('goal_distance') is not None
    has_pose = raw.get('goal_pose') is not None
    has_turn = raw.get('turn') is not None
    has_drive = raw.get('drive') is not None
    has_object = raw.get('go_to_object') is not None
    _require(
        sum((has_distance, has_pose, has_turn, has_drive, has_object)) == 1,
        f'{where}: exactly one of goal_distance/goal_pose/turn/drive/go_to_object is required',
    )

    goal_distance = None
    if has_distance:
        gd = raw['goal_distance']
        _require(
            isinstance(gd, (int, float)) and not isinstance(gd, bool),
            f'{where}: goal_distance must be numeric (got {type(gd).__name__})',
        )
        goal_distance = float(gd)
    goal_pose = None
    if has_pose:
        gp = raw['goal_pose']
        _require(
            isinstance(gp, dict) and {'x', 'y', 'yaw'} <= gp.keys(),
            f'{where}: goal_pose requires x, y, yaw',
        )
        goal_pose = GoalPose(x=float(gp['x']), y=float(gp['y']), yaw=float(gp['yaw']))
    turn = _parse_turn_spec(raw['turn'], where) if has_turn else None
    drive = _parse_drive_spec(raw['drive'], where) if has_drive else None
    go_to_object = _parse_object_spec(raw['go_to_object'], where) if has_object else None

    _require('stop_condition' in raw, f'{where}: stop_condition is required')
    stop_condition = _parse_stop_condition(raw['stop_condition'], where)

    # object_reached and go_to_object are each other's only partner: the
    # condition reads the approach geometry only an object step produces, and
    # an object step ending on anything else would bypass its outcomes.
    if has_object:
        _require(
            stop_condition.type == 'object_reached',
            f'{where}: a go_to_object step requires stop_condition.type '
            f'"object_reached" (got {stop_condition.type!r})',
        )
    elif stop_condition.type == 'object_reached':
        _require(False, f'{where}: object_reached is only valid on a go_to_object step')

    if stop_condition.type == 'distance_reached' and 'distance' not in stop_condition.params:
        _require(
            has_distance,
            f'{where}: distance_reached with no explicit distance needs goal_distance set',
        )

    # orientation_delta is heading-step-exclusive: valid on a "turn" step
    # (schema 2.0) or a "drive" step (schema 3.0), rejected on anything else.
    #
    # WHY DRIVE WAS ADDED HERE, since this used to read `_require(has_turn)`:
    # the LLM planner's guard "turned" is a GUARD, not a goal -- it says when
    # to stop turning, on a phase whose mode already says how to turn. In this
    # schema that is a drive step with an orientation_delta stop_condition,
    # and nothing else expresses it. Keeping the check turn-exclusive would
    # have made every "gira a destra" plan unloadable, so this relaxation is a
    # prerequisite for the drive step, not an extra.
    if stop_condition.type == 'orientation_delta':
        _require(
            has_turn or has_drive,
            f'{where}: orientation_delta stop_condition is only valid on a "turn" '
            'or "drive" step',
        )
        # ...and its magnitude must agree with whichever of the two actually
        # commands the heading change -- validated here (load time) rather
        # than trusted separately at evaluation time, so a mismatched pair can
        # never silently mean two different things (e.g. the step drives
        # 90 deg but the mission advances at 45 deg because someone edited one
        # field and not the other).
        if has_turn:
            commanded_deg = abs(turn.heading_delta_deg)
            commanded_field = 'abs(turn.heading_delta_deg)'
        else:
            # A drive step's magnitude is turn_mag_deg, which is ALREADY
            # unsigned (direction lives in turn_sign) -- abs() anyway so the
            # two branches compare like for like and a future sign convention
            # change here can't silently invert the check.
            commanded_deg = abs(drive.turn_mag_deg)
            commanded_field = 'abs(drive.turn_mag_deg)'
            # A "straight" drive step turns by definition zero, so pairing it
            # with orientation_delta asks the mission to advance on a rotation
            # nothing is commanding. Caught here rather than left to time out
            # live.
            _require(
                drive.mode == 'wall_turn',
                f'{where}: orientation_delta stop_condition on a drive step '
                f'requires drive.mode "wall_turn" (got {drive.mode!r}) -- a '
                '"straight" step commands no rotation for it to measure',
            )
            _require(
                commanded_deg > 0.0,
                f'{where}: orientation_delta stop_condition requires an explicit '
                'non-zero drive.turn_mag_deg -- 0 means "use the controller '
                'default", which this check cannot compare against',
            )
        _require(
            math.isclose(
                abs(float(stop_condition.params['value'])), commanded_deg,
                abs_tol=1e-6,
            ),
            f'{where}: orientation_delta value ({stop_condition.params["value"]}) must equal '
            f'{commanded_field} ({commanded_deg})',
        )

    timeout_sec = None
    if raw.get('timeout_sec') is not None:
        ts = raw['timeout_sec']
        _require(
            isinstance(ts, (int, float)) and not isinstance(ts, bool),
            f'{where}: timeout_sec must be numeric (got {type(ts).__name__})',
        )
        timeout_sec = float(ts)
        _require(timeout_sec > 0.0, f'{where}: timeout_sec must be > 0')

    on_timeout_explicit = 'on_timeout' in raw
    on_timeout = raw.get('on_timeout', 'abort')
    _require(
        on_timeout in ON_TIMEOUT_VALUES,
        f'{where}: on_timeout={on_timeout!r} not in {sorted(ON_TIMEOUT_VALUES)}',
    )
    # timeout_sec is optional (see Move.timeout_sec's own comment) precisely
    # so a move can opt out of the safety cap entirely -- an explicit
    # on_timeout with nothing for it to ever apply to is almost always a
    # mistake (a field the author meant to pair with a timeout_sec that got
    # dropped/renamed), so this is rejected at load time rather than silently
    # ignored. The reverse (timeout_sec with no explicit on_timeout) is NOT
    # an error -- on_timeout's own default ('abort') already covers that
    # case exactly as it did before schema_version 2.0.
    _require(
        not (on_timeout_explicit and timeout_sec is None),
        f'{where}: on_timeout is set but timeout_sec is not -- on_timeout has nothing to '
        'apply to (either add timeout_sec or remove on_timeout)',
    )

    vdes = None
    if raw.get('vdes') is not None:
        vd = raw['vdes']
        _require(
            isinstance(vd, (int, float)) and not isinstance(vd, bool),
            f'{where}: vdes must be numeric (got {type(vd).__name__})',
        )
        vdes = float(vd)

    on_object_raw = raw.get('on_object', [])
    _require(isinstance(on_object_raw, list), f'{where}: on_object must be a list')
    on_object = [_parse_on_object(o, where) for o in on_object_raw]

    if has_object:
        _require(
            timeout_sec is not None,
            f'{where}: a go_to_object step requires timeout_sec -- an approach to a '
            'moving target must have a bound',
        )
        _require(
            not on_object,
            f'{where}: on_object is not allowed on a go_to_object step -- the target is '
            'itself a detected object, and a stop_and_hold would END the approach '
            '(mpc_corr ends an object move on /mpc/hold)',
        )

    # Explicit `is True`/`is False` rather than truthiness: `"terminal": 1`
    # and `"terminal": "yes"` are mistakes worth naming, not values to
    # silently coerce. Absent -> False, which is what every pre-3.0 mission
    # gets and exactly today's behaviour. The POSITIONAL checks (last move
    # only; required on the last move of a 3.0 mission) need the whole move
    # list and live in parse_mission() instead.
    terminal = raw.get('terminal', False)
    _require(
        terminal is True or terminal is False,
        f'{where}: terminal must be a JSON boolean (got {terminal!r})',
    )

    return Move(
        id=move_id, goal_distance=goal_distance, goal_pose=goal_pose, turn=turn,
        drive=drive, go_to_object=go_to_object, vdes=vdes,
        stop_condition=stop_condition, on_object=on_object,
        timeout_sec=timeout_sec, on_timeout=on_timeout, terminal=terminal,
    )


def parse_mission(raw: object) -> MissionConfig:
    """Parse+validate an already-json.loads()'d mission dict into a MissionConfig.
    Raises MissionConfigError with a specific, actionable message on the first
    violation found. Never returns a partially-built MissionConfig -- either the
    whole thing validates or nothing is returned at all.

    BACKWARD COMPATIBILITY IS A HARD REQUIREMENT of this function, restated
    here because schema_version 3.0 is the first version to gate anything on
    the version string at all: every mission under missions/ that does not
    declare schema_version "3.0" must keep parsing AND behaving exactly as it
    did before 3.0 existed. Concretely -- a pre-3.0 mission never has to set
    Move.terminal, never has to use a "drive" step, and its last move ends the
    mission through AdvanceMove's own last-move branch exactly as it always
    has. The existing test suite is the proof: it is expected to pass
    unmodified.
    """
    _require(isinstance(raw, dict), 'mission root must be a JSON object')
    mission_id = raw.get('mission_id')
    _require(isinstance(mission_id, str) and mission_id, 'mission_id is required')

    # Optional, defaults to "1.0" -- every mission written before this field
    # existed omits it entirely, and that must keep parsing exactly as it did
    # before (see MissionConfig.schema_version's own comment: this is
    # informational, not an enforced gate on which fields below are legal).
    schema_version = raw.get('schema_version', '1.0')
    _require(isinstance(schema_version, str) and schema_version, 'schema_version must be a non-empty string if present')

    moves_raw = raw.get('moves')
    _require(isinstance(moves_raw, list) and len(moves_raw) > 0, 'moves must be a non-empty list')

    moves = [_parse_move(m, f'moves[{i}]') for i, m in enumerate(moves_raw)]

    ids = [m.id for m in moves]
    dupes = sorted({i for i in ids if ids.count(i) > 1})
    _require(not dupes, f'duplicate move id(s): {dupes}')

    # ---- terminal (schema_version 3.0) ------------------------------------
    # Two rules, with deliberately different compatibility treatment:
    #
    #   LEGAL ONLY ON THE LAST MOVE -- enforced for EVERY schema version. A
    #   terminal flag on a non-last move is a contradiction in any version
    #   (the mission both ends there and has moves after it), and no mission
    #   written before 3.0 can be carrying the field at all, so enforcing it
    #   universally cannot break one. This is also the exact mistake
    #   f110_autonomy's if/elif/ELSE made unnoticeable: a stop_at on an
    #   intermediate phase silently latched the car there forever instead of
    #   failing. It fails here, at load time, by name.
    #
    #   REQUIRED ON THE LAST MOVE -- enforced for schema_version "3.0" ONLY.
    #   Every mission under missions/ predates this field; making it a
    #   universal requirement would make all of them unloadable, and
    #   COMPATIBILITY IS NON-NEGOTIABLE (see this function's own docstring).
    #   A pre-3.0 mission's last move still ends the mission exactly as it
    #   always has -- AdvanceMove's last-move branch does not consult this
    #   flag (see Move.terminal on why the flag declares rather than
    #   implements) -- so nothing about those missions changes, at load time
    #   or at run time.
    for m in moves[:-1]:
        _require(
            not m.terminal,
            f'move {m.id!r}: terminal is only legal on the LAST move -- a terminal '
            'move ends the mission, so the moves after it could never run',
        )
    if schema_version in TERMINAL_REQUIRED_VERSIONS:
        _require(
            moves[-1].terminal,
            f'move {moves[-1].id!r}: schema_version {schema_version!r} requires the '
            'last move to set terminal: true -- an open-ended mission whose final move '
            'does not declare itself terminal has nothing that stops the car',
        )

    id_set = set(ids)
    for m in moves:
        for oo in m.on_object:
            if oo.action == 'skip_to_move':
                target = oo.params['move_id']
                _require(
                    target in id_set,
                    f'move {m.id!r}: skip_to_move target {target!r} is not a known move id',
                )

    return MissionConfig(mission_id=mission_id, moves=moves, schema_version=schema_version)


def load_mission_file(path: str) -> MissionConfig:
    """Read and parse a mission JSON file from disk.

    Raises MissionConfigError (schema violation), OSError (file not found/
    unreadable), or json.JSONDecodeError (malformed JSON) -- callers must catch
    all three the same way: log clearly, leave whatever mission was already
    loaded untouched, do not install a partial result.
    """
    text = Path(path).read_text(encoding='utf-8')
    raw = json.loads(text)
    return parse_mission(raw)
