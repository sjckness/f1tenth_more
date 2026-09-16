"""Translates an LLM phase list (llm_planner_node.py's own vocabulary --
mode/guard/thresh/turn_sign/stop_at/stop_at_distance) into a mission dict
matching f1tenth_behavior's REAL mission schema (mission/mission_config.py's
parse_mission()), ready to json.dump() to a file and hand to
/mission/load_mission.

WHY THIS MODULE STILL EXISTS after schema_version 3.0 made the guard model
first-class, since "the schema now speaks the LLM's language" sounds like it
should have deleted it: the two vocabularies are structurally different
shapes of the same information, not different information. The LLM emits FLAT
phases --

    {"mode": "wall_turn", "turn_sign": -1.0, "guard": "turned", "thresh": 1.3}

-- and the schema is NESTED, with the driving command and the thing that ends
it in two sibling objects:

    {"id": ..., "drive": {"mode": "wall_turn", "turn_sign": -1.0,
                          "turn_mag_deg": 74.48},
     "stop_condition": {"type": "orientation_delta", "value": 74.48}}

So there is still a flat -> nested reshape, a guard-name -> condition-type
lookup, and a radians -> degrees conversion to do. What there is no longer is
any INVENTION: every field below comes from a field the LLM actually sent.

WHAT WENT AWAY WITH 3.0, and it was the whole problem:

  - goal_distance: 50.0. A "wall"/"front_object"-guarded phase has no distance
    to travel -- it stops on CLEARANCE -- but the pre-3.0 schema had no goal
    shape that could say so, so this module fabricated a 50 m target and hoped
    the guard fired first. The MPC was genuinely driving toward a point 50 m
    away, and /mpc/goal_reached was meaningless for those moves.
    STRAIGHT_GOAL_DISTANCE_CAP_M is deleted; drive moves are open-ended.

  - A turn speed this module had to make up (DEFAULT_TURN_SPEED_MPS) because
    TurnSpec.speed was required and the LLM has no speed field. DriveSpec.speed
    has a "use the controller default" sentinel of 0, which is the honest
    encoding of "the plan did not say", so that constant is deleted too.

  - front_clearance for guard "front_object". That guard means "the nearest
    OBJECT ahead", and f110_autonomy backed it with distance_to_front_object()
    -- nearest obstacle in the forward half-plane. The front_clearance
    stop_condition in this stack is object-BLIND, which is the opposite of
    what that guard needs: it read /costmap/front_clearance (nearest occupied
    cell of slam_toolbox's map, which a chair or a person is not on at all)
    and now reads /perception/front_distance, whose whole design removes
    detected objects from the measurement. Either way the value never drops
    for a person and the move never ends. It maps instead to
    obstacle_distance_below with forward_only: true, which is that same
    forward-half-plane obstacle distance. See the GUARD_MAP table below.

TWO ENTRY POINTS, and new work wants the second one:

  phases_to_mission(phases)   the LEGACY flat phase vocabulary described
                              above, in which the model authors its own turn
                              magnitude in radians. Kept working, unchanged.
  translate(intent)           the INTENT vocabulary (schemas/intent_v1.json),
                              in which it does not. See the "INTENT v1 ->
                              mission v3.0" section at the foot of this file
                              for the provenance rule that governs it, and
                              llm/README.md for the D1-D4 decisions.

Deliberately pure Python, no rclpy/ROS import of any kind -- fully
unit-testable in isolation (see test/test_plan_translate.py and
test/test_intent_translate.py) the same way mission_config.py itself is.
translate() does import f1tenth_behavior's mission_config, lazily and only to
validate what it is about to emit; that module is pure stdlib too, so the
property holds. llm_planner_node.py (this package's ROS-facing node) is the
only caller.

THE ONE PLACE THIS IS NOT A LITERAL FIELD COPY, and why -- read this before
"simplifying" it into `drive['mode'] = ph['mode']`:

SYSTEM_PROMPT's REGOLA 1 tells the model that to keep going after a turn it
must reuse "wall_turn" with the same turn_sign, never "straight". Its own
example 1 does exactly that:

    [{"mode":"straight",  "guard":"wall",     "thresh":3.0},
     {"mode":"wall_turn", "guard":"turned",   "thresh":1.3, "turn_sign":-1.0},
     {"mode":"wall_turn", "guard":"distance", "thresh":2.0, "turn_sign":-1.0,
      "stop_at_distance":2.0}]

That rule is an artefact of f110_autonomy's psi_init_corridor being captured
ONCE at node start and never re-anchored. There, "straight" meant "realign to
the heading the RUN began with", so reusing it after a turn would have steered
the car back toward its pre-turn direction; "wall_turn" with the same sign
recomputed the SAME absolute target (psi_base + sign * turn_mag) and therefore
HELD the post-turn heading.

MPC_corr re-anchors psi_init_corridor per move (goal_drive_callback), which is
a deliberate divergence and a fix for that stack's second-turn bug. In the
re-anchored model "straight" already means "hold the heading THIS move started
with" -- which is precisely what phase 3 above wants -- while "wall_turn" would
re-anchor to the post-turn heading and turn a SECOND time. Copying the mode
across literally would make example 1 turn 180 degrees instead of 90 and drive
back the way it came.

So: the phase that actually commands a rotation is the one whose guard is
"turned", and only that one becomes drive.mode "wall_turn". Every other phase
becomes "straight" regardless of what its own `mode` says. That preserves the
plan's BEHAVIOUR exactly, which is what a lossless translation means; copying
the field verbatim would preserve its spelling and break its meaning.
`mode` is otherwise unused here, exactly as it was before 3.0.

Schema facts this module depends on -- verified by reading mission/
mission_config.py and mission/condition_eval.py, not assumed:

  - Top-level key is "moves", not "plan".
  - A move is discriminated by exactly one of goal_distance/goal_pose/turn/
    drive being set (mission_config._parse_move: `sum(...) == 1`). Every move
    this module emits is a `drive` move.
  - stop_condition is always a SIBLING field on the move, never nested inside
    the goal shape.
  - stop_condition.type spellings actually in STOP_CONDITION_TYPES:
    "distance_reached", "front_clearance" and "obstacle_distance_below" (param
    key "distance"), "orientation_delta" (param key "value", DEGREES).
  - "orientation_delta" on a drive move requires drive.mode "wall_turn", a
    non-zero drive.turn_mag_deg, and `value` == abs(drive.turn_mag_deg) to
    within math.isclose(abs_tol=1e-6). That equality is why `value` below is
    always the SAME Python float object's value as turn_mag_deg and is never
    recomputed from thresh a second time -- two independent conversions of the
    "same" number risk a last-bit mismatch the loader would then reject.
  - schema_version "3.0" REQUIRES terminal: true on the last move, which is
    why the last move here always gets it, whether or not the phase carried a
    stop_at.
"""

import dataclasses
import functools
import hashlib
import json
import math
import pathlib
import time
from typing import Optional

import jsonschema

# Mission-level schema_version. "3.0" is the version that introduced the
# "drive" step and Move.terminal -- i.e. the first version able to express an
# LLM phase at all without fabricating a goal. Not optional bookkeeping here:
# mission_config.parse_mission() gates the "last move must be terminal"
# requirement on exactly this string.
MISSION_SCHEMA_VERSION = '3.0'

# Consecutive satisfied ticks a "wall"-guarded phase needs before it advances.
# f110_autonomy used wall_count_needed = 3 against a raw, unfiltered depth
# value that could dip below the threshold for a single frame on noise alone.
# condition_eval's own default is 1 (no debounce), deliberately, so that
# missions written before debouncing existed are untouched -- this is the one
# path that asks for the reference stack's value.
WALL_DEBOUNCE_TICKS = 3

# Subtracted from a terminal stop_at to get drive.approach_d_safe -- the
# standoff the car is allowed to close to while approaching the object it was
# told to stop in front of.
#
# f110_autonomy computed d_safe = max(0, min(dmin, stop_at - obs_margin - 0.1))
# for exactly this purpose: without it the obstacle penalty pushes the car
# AROUND the object it is meant to stop in front of. Only the literal 0.1
# lives here. The obs_margin term and the min(dmin, ...) clamp are both
# mpc_corr's own arithmetic -- it holds the real car_radius/avoidance_margin
# and clamps the request so it can only ever RELAX the standoff, never tighten
# it (see MPC_corr.goal_drive_callback). Duplicating those values in this file
# would be a second, drifting spelling of numbers that live in
# stack_params.yaml.
APPROACH_D_SAFE_BACKOFF_M = 0.1

# guard -> (stop_condition type, threshold param name, extra fixed params).
#
# THE ENTIRE GUARD MAPPING, in one table, because it is the part most likely
# to be wrong and hardest to notice being wrong -- a guard silently pointed at
# the wrong sensor produces a mission that loads, runs, and simply never
# advances.
#
#   wall         -> front_clearance. Backed by /perception/front_distance,
#                   front_clearance_node's object-EXCLUDED background distance
#                   from the ZED depth ROI (was /costmap/front_clearance, the
#                   nearest occupied cell of slam_toolbox's map within a
#                   symmetric forward cone). Correct for "a wall", and
#                   strictly more correct after the swap on both counts: the
#                   camera sees a wall the car is meeting for the first time,
#                   which the map-derived value could not until SLAM had
#                   mapped it, and objects are removed from the measurement so
#                   a person walking through does not fire a wall guard.
#                   Debounced at 3, matching the reference stack.
#   front_object -> obstacle_distance_below, forward_only. The nearest YOLO/
#                   depth-projected obstacle in the forward half-plane
#                   (MPC_corr.compute_forward_obstacle_distance's dot > 0
#                   test), which is what f110_autonomy's own
#                   distance_to_front_object() computed. Correct for "a chair",
#                   "a person" -- things front_clearance's backing topic
#                   deliberately subtracts out, and which the map-derived
#                   source before it never had on the map in the first place.
#   turned       -> orientation_delta. Handled separately below, not from this
#                   table: it is the only guard whose threshold needs a unit
#                   conversion AND whose value has to stay bit-identical to a
#                   field in the drive spec.
#   distance     -> distance_reached.
GUARD_MAP = {
    'wall': ('front_clearance', 'distance', {'debounce_ticks': WALL_DEBOUNCE_TICKS}),
    'front_object': ('obstacle_distance_below', 'distance', {'forward_only': True}),
    'distance': ('distance_reached', 'distance', {}),
}


class PlanTranslationError(ValueError):
    """Raised when `phases` reaches phases_to_mission() malformed/unvalidated
    (see _sanity_check), or contains a (mode, guard) pairing outside the
    mapping this module implements (see module docstring)."""


def _sanity_check(phases) -> None:
    """Defensive re-check of the invariants this module's own mapping logic
    depends on -- NOT a reimplementation of llm_planner_node.validate_plan()'s
    full contract (its THRESH_RANGE plausibility warnings, for instance, don't
    matter for translation correctness). Deliberately does not import
    validate_plan() from llm_planner_node.py: that module imports rclpy at
    load time, and pulling it in here (even lazily) would break this module's
    own "no ROS imports, fully unit-testable in isolation" property.

    The REAL first line of defense is and stays LLMPlannerNode.
    process_command() calling validate_plan() before this function is ever
    reached -- this is only the second line, so a phase list that skips that
    step fails loudly and specifically here instead of crashing on a bare
    KeyError/TypeError somewhere in the mapping loop below, or silently
    producing a bogus mission.
    """
    if not isinstance(phases, list) or not phases:
        raise PlanTranslationError('phases must be a non-empty list (did validate_plan() run?)')
    for i, ph in enumerate(phases):
        if not isinstance(ph, dict):
            raise PlanTranslationError(f'phase {i} is not an object (did validate_plan() run?)')
        if 'mode' not in ph or 'guard' not in ph or 'thresh' not in ph:
            raise PlanTranslationError(
                f'phase {i} is missing mode/guard/thresh (did validate_plan() run?)')
        if not isinstance(ph['thresh'], (int, float)) or isinstance(ph['thresh'], bool):
            raise PlanTranslationError(
                f'phase {i}: thresh must be numeric, got {ph["thresh"]!r} '
                '(did normalize_plan()+validate_plan() run?)')
        if ph['mode'] == 'wall_turn' and 'turn_sign' not in ph:
            raise PlanTranslationError(
                f'phase {i}: wall_turn with no turn_sign (did validate_plan() run?)')


def _phase_to_move(i: int, ph: dict) -> dict:
    """One flat phase -> one nested drive move. Structural only: every value
    below comes from a field the phase actually carried."""
    mode = ph['mode']
    guard = ph['guard']
    thresh = float(ph['thresh'])
    move_id = f'move_{i}'

    if guard == 'turned':
        if mode != 'wall_turn' or 'turn_sign' not in ph:
            raise PlanTranslationError(
                f"phase {i}: guard 'turned' requires mode 'wall_turn' with a turn_sign "
                f'(got mode={mode!r}); no direction to derive the rotation from.'
            )
        # THE ONE UNIT CONVERSION IN THE WHOLE PIPELINE. The LLM's "turned"
        # threshold is in radians (SYSTEM_PROMPT says so, and THRESH_RANGE
        # bounds it as radians); every angle in the mission schema is in
        # degrees. Converted exactly once, here, at the boundary between the
        # two vocabularies.
        turn_mag_deg = math.degrees(thresh)
        return {
            'id': move_id,
            'drive': {
                'mode': 'wall_turn',
                'turn_sign': float(ph['turn_sign']),
                'turn_mag_deg': turn_mag_deg,
            },
            # Same value, not a second conversion of thresh -- mission_config
            # requires these to match to 1e-6 and re-deriving invites a
            # last-bit mismatch it would reject at load time.
            'stop_condition': {'type': 'orientation_delta', 'value': turn_mag_deg},
        }

    try:
        cond_type, thresh_key, extra = GUARD_MAP[guard]
    except KeyError:
        raise PlanTranslationError(
            f'phase {i}: unsupported guard {guard!r} -- outside the mapping SYSTEM_PROMPT '
            'teaches the LLM to produce; refusing to guess.'
        ) from None

    stop_condition = {'type': cond_type, thresh_key: thresh}
    stop_condition.update(extra)
    return {
        'id': move_id,
        # "straight", NOT ph['mode'] -- see the module docstring's own section
        # on this. A wall_turn phase whose guard is not "turned" is a
        # post-turn CONTINUATION in the reference stack's non-re-anchored
        # geometry; with per-move re-anchoring, "hold the heading this move
        # started with" is what reproduces it, and copying "wall_turn" across
        # would turn a second time.
        'drive': {'mode': 'straight'},
        'stop_condition': stop_condition,
    }


def phases_to_mission(phases: list, mission_id: Optional[str] = None) -> dict:
    """Translate a phase list (already normalize_plan()+validate_plan()'d by
    the caller -- see _sanity_check's own docstring for why that's still not
    fully trusted here) into a mission dict matching mission_config.
    parse_mission()'s schema.

    Raises PlanTranslationError -- never returns a partial/best-effort
    result, same "all or nothing" discipline mission_config.parse_mission()
    itself uses.
    """
    _sanity_check(phases)

    if mission_id is None:
        mission_id = f'llm_plan_{int(time.time())}'

    moves = [_phase_to_move(i, ph) for i, ph in enumerate(phases)]

    # ---- the last move is always terminal ------------------------------
    # schema_version "3.0" requires it (mission_config.parse_mission), and it
    # is true by construction anyway: this is the end of the plan, so the
    # mission completes here rather than advancing. Set unconditionally, not
    # only when the phase carried a stop_at, so a phase list that somehow
    # reached here without one still produces a LOADABLE mission -- one that
    # stops on its own guard -- instead of a mission_config rejection from
    # inside the planner node. validate_plan() is what refuses a plan whose
    # last phase has no stop_at at all; that is its job, not this function's.
    moves[-1]['terminal'] = True

    # ---- stop_at / stop_at_distance -------------------------------------
    # OVERRIDE the last move's guard-derived stop_condition rather than
    # joining it: in f110_autonomy these were an if/elif checked BEFORE the
    # guard, so a phase carrying either never evaluated its guard at all.
    # Overriding reproduces that precedence exactly, while the surrounding
    # machinery (scoring, logging, timeout) keeps running -- which theirs did
    # not, because it simply latched vdes = 0 where it stood.
    #
    # Both branches write an EXPLICIT threshold rather than relying on any
    # other field of the move, so the override stands alone.
    # validate_plan() already guarantees at most one of the two is present,
    # and only on the last phase.
    last = phases[-1]
    if 'stop_at' in last:
        stop_at = float(last['stop_at'])
        # "Stop quando oggetto davanti <= valore" (SYSTEM_PROMPT) -- an
        # OBJECT, so the forward-half-plane obstacle distance, not the
        # map-derived front_clearance. This is the correction that makes a
        # "fermati alla sedia" plan actually stop: a chair is not on the map.
        moves[-1]['stop_condition'] = {
            'type': 'obstacle_distance_below',
            'distance': stop_at,
            'forward_only': True,
        }
        # Relax the obstacle standoff for the approach, so avoidance does not
        # push the car around the very object it was told to stop in front of
        # -- see APPROACH_D_SAFE_BACKOFF_M. mpc_corr clamps this so it can
        # only ever relax, never tighten.
        moves[-1]['drive']['approach_d_safe'] = max(
            0.0, stop_at - APPROACH_D_SAFE_BACKOFF_M)
    elif 'stop_at_distance' in last:
        # A distance, not an object: no obstacle standoff to relax, so no
        # approach_d_safe. Deliberate -- f110_autonomy's relaxation was tied
        # to stop_at alone for the same reason.
        moves[-1]['stop_condition'] = {
            'type': 'distance_reached',
            'distance': float(last['stop_at_distance']),
        }

    return {
        'mission_id': mission_id,
        'schema_version': MISSION_SCHEMA_VERSION,
        'moves': moves,
    }


# =====================================================================
# INTENT v1 -> mission v3.0
# =====================================================================
#
# The second, narrower entry point into this module, and the one new work
# should use. phases_to_mission() above translates the LEGACY flat phase
# vocabulary, in which the model authored its own turn magnitude in radians;
# translate() below takes the minimal INTENT vocabulary, in which it does not.
#
# WHY THE INTENT FORMAT HAS NO TURN MAGNITUDE. The mission
# "llm_plan_1789054103" shipped a turn of 74.48451336700703 degrees. That was
# not a hallucination: llm_planner_node.SYSTEM_PROMPT tells the model outright
# to use `guard "turned" thresh 1.3` for a turn, math.degrees(1.3) is
# 74.48451336700703, and phases_to_mission() rendered it at full float
# precision. Deleting turn magnitude from what the model may author deletes
# that whole class of defect: the magnitude becomes a named config value the
# translator injects.
#
# THE PROVENANCE RULE, which is what keeps invention out. Every field of every
# emitted mission carries exactly one of four sources:
#
#   INTENT   copied from the intent JSON
#   CONFIG   a named key of TranslatorConfig
#   DERIVED  a documented formula over INTENT and CONFIG
#   CONST    a literal fixed by the mission schema
#
# There is no fifth category, and in particular there is no "sensible
# default": a value that cannot be traced to one of the four is not emitted at
# all, and translate() raises instead. test_intent_translate.py asserts that
# every leaf of every translated mission has a provenance entry, which is the
# test that stops invention creeping back in later.

INTENT = 'INTENT'
CONFIG = 'CONFIG'
DERIVED = 'DERIVED'
CONST = 'CONST'

# Per-guard thresh ranges, enforced HERE and only here.
#
# intent_v1.json carries a blanket 0.2-20.0 on thresh because JSON Schema
# cannot condition a range on a sibling property without another oneOf layer,
# and the prompt states these tighter numbers to the model. That makes three
# places the range could live and exactly one place it is CHECKED. See
# intent_v1.json's own description for why multipleOf is not used for the
# 2-decimal rule either.
GUARD_THRESH_RANGE = {
    'wall': (0.5, 4.0),
    'front_object': (0.5, 5.0),
    'distance': (0.2, 20.0),
}

# left/right -> drive.turn_sign.
#
# NOT A GUESS: mission_config._parse_drive_spec states the convention
# outright -- "drive.turn_sign of exactly -1.0 (right/CW) or +1.0 (left/CCW)"
# -- and it is the value the stack validates against. The 2026-09-14 hardware
# run of wall_turn.json corroborates the sign: turn_sign -1.0 took /odom yaw
# from -0.014 rad to -1.62 rad, i.e. decreasing yaw, i.e. clockwise.
#
# STILL UNCONFIRMED PHYSICALLY. Nobody has stood in the room and checked that
# clockwise-in-odom is the direction a person calls "right"; an inverted yaw
# convention anywhere upstream would satisfy everything above and still turn
# the wrong way. That is Stage 2 of docs/bringup_checklist.md. Until it is
# signed off, a mission whose direction matters should be run with a hand on
# the e-stop.
TURN_SIGN = {'left': 1.0, 'right': -1.0}

# Clamp on the emitted timeout_sec, in seconds. The lower bound keeps a
# degenerate move from being born already timed out; the upper bound is the
# actual point of the exercise -- the generated missions that motivated this
# work carried no timeout at all, so an open guard that never fired ran until
# someone reached the car.
TIMEOUT_MIN_SEC = 1
TIMEOUT_MAX_SEC = 300


class IntentSchemaError(PlanTranslationError):
    """The intent document does not validate against schemas/intent_v1.json."""


class UnsupportedIntentModeError(IntentSchemaError):
    """The model authored a mode the schema allows and the runtime cannot run.

    A subclass of IntentSchemaError so _plan_v2 still feeds it back as retry
    text, but a distinct type so the node can log it separately: reaching this
    means the model ignored the system prompt, which is a prompt-quality
    signal worth counting. A request the model correctly routed to
    "unsupported" never arrives here.
    """


class IntentRangeError(PlanTranslationError):
    """A thresh is outside the per-guard range in GUARD_THRESH_RANGE.

    Carries the guard, the offending value and the range, so the caller can
    say which of the three it was without re-deriving it.
    """

    def __init__(self, guard, value, low, high):
        """Record the guard, the rejected value and the range it missed."""
        super().__init__(
            f'guard {guard!r}: thresh {value!r} outside the permitted range '
            f'{low} - {high} (intent_v1.json carries only the blanket range; '
            'the per-guard one is enforced here)')
        self.guard = guard
        self.value = value
        self.range = (low, high)


class EmptyPlanError(PlanTranslationError):
    """The intent carried no executable phase, so no mission is emitted.

    Not an error in the model: it is the correct answer to "gira e vai avanti
    un po'", and to a command whose every phase was unsupported. The caller
    should surface `unsupported` to the operator rather than retry.
    """


class TranslatorOutputError(PlanTranslationError):
    """translate() built a mission the stack's own loader rejects.

    This is a TRANSLATOR BUG, never user error, and it is raised rather than
    returned so the invalid document cannot reach the mission system. The
    offending mission is attached as `.mission` for logging in full.
    """

    def __init__(self, message, mission):
        """Record the rejection message and the document that caused it."""
        super().__init__(message)
        self.mission = mission


# Prefix of this module's keys in stack_params.yaml. Flat, like every other
# key in that file (front_clearance_*, swept_clearance_*, ...).
_STACK_PARAM_PREFIX = 'mission_translator_'


@dataclasses.dataclass(frozen=True)
class TranslatorConfig:
    """Every number translate() is allowed to inject, and nothing else.

    The defaults below are the SAME values as the mission_translator_* block
    of stack_params.yaml, and test_intent_translate.py asserts that they still
    match, so the two spellings cannot drift. They are duplicated rather than
    read from stack_params on import because this module keeps its "imports in
    isolation, no ament index required" property -- get_value() needs a
    resolvable package share directory, which a bare unit test does not have.
    Use from_stack_params() in a running node; construct one literally in a
    test, which also makes the test deterministic.
    """

    # Reference mission wall_turn.json runs its straight run-up at 0.4 and its
    # turn at 0.5 -- these are those two values, not an independent guess.
    speed_straight: float = 0.4
    speed_turn: float = 0.5

    # The magnitude every emitted turn carries. See translate()'s own note on
    # D2: the behaviour tree honours whatever value it is given, so this is a
    # real choice rather than the only reachable number, but 90 is the only
    # magnitude any of this has been run at.
    turn_magnitude_deg: float = 90.0

    # rad/s, used ONLY to size a turn's timeout, never to command anything,
    # so being wrong makes the timeout loose or tight and cannot steer the
    # car. MEASURED: the 2026-09-14 hardware run of missions/wall_turn.json
    # turned 1.6091 rad in 7.3747 s = 0.2182 rad/s. The synthetic 0.6 this
    # started at is 2.8x too fast and would have sized a 90 deg turn at 13 s
    # against an observed 7.4 s turn.
    nominal_yaw_rate: float = 0.22

    # Metres. "wall" and "front_object" have no bounded travel -- if the wall
    # is never seen the move never ends on its own -- so the timeout has to be
    # sized against an assumed worst-case distance instead of a real one.
    open_guard_max_distance: float = 10.0

    timeout_factor: float = 3.0
    timeout_floor: float = 5.0

    mission_id_prefix: str = 'llm'

    # D1. True: a straight phase after a turn is emitted as drive.mode
    # "straight", which is what phases_to_mission() has always done and what
    # missions/wall_turn_then_straight.json exercises. False restores the old
    # workaround -- re-emit it as a wall_turn carrying the previous turn's
    # sign and a zero magnitude -- for the case where a straight move after a
    # turn turns out not to hold its heading on hardware.
    post_turn_uses_straight: bool = True

    @classmethod
    def from_stack_params(cls):
        """Build a config from the mission_translator_* keys of stack_params.

        Imported lazily: f1tenth_params resolves its yaml through the ament
        index, which a unit test running outside a sourced workspace does not
        have.
        """
        from f1tenth_params.param_defaults import get_value
        fields = dataclasses.fields(cls)
        return cls(**{f.name: get_value(_STACK_PARAM_PREFIX + f.name) for f in fields})


def _intent_schema_path() -> pathlib.Path:
    """Locate schemas/intent_v1.json, in the source tree or the install share.

    The source-tree path is tried first and is what a unit test hits; the
    ament share directory is the installed layout. symlink-install makes the
    two the same file, but a non-symlink install does not.
    """
    here = pathlib.Path(__file__).resolve().parent.parent
    local = here / 'schemas' / 'intent_v1.json'
    if local.is_file():
        return local
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory('llm')) / 'schemas' / 'intent_v1.json'


@functools.lru_cache(maxsize=1)
def load_intent_schema() -> dict:
    """Return the parsed intent_v1 schema. Cached -- it never changes at runtime."""
    with open(_intent_schema_path(), encoding='utf-8') as fh:
        return json.load(fh)


def _canonical_json(obj) -> str:
    """Byte-stable JSON for hashing: sorted keys, no incidental whitespace."""
    return json.dumps(obj, sort_keys=True, separators=(',', ':'), ensure_ascii=True)


@dataclasses.dataclass(frozen=True)
class TranslationResult:
    """What translate() returns. The mission alone is never the whole answer.

    requires_confirmation is the field that matters operationally: it is True
    exactly when the intent carried a non-empty `unsupported`, meaning the
    operator asked for something the primitives cannot do and the mission
    below is only the part that survived. THAT MISSION MUST NOT AUTO-RUN --
    it is precisely the case where the person believes they asked for
    something else, and the car would execute the remainder without them.
    """

    mission: dict
    requires_confirmation: bool
    unsupported: tuple
    provenance: Optional[dict] = None


def _round_m(value: float) -> float:
    """Round a distance to 2 decimal places (centimetres)."""
    return round(float(value), 2)


def _round_deg(value: float) -> float:
    """Round an angle to 1 decimal place."""
    return round(float(value), 1)


def _expected_seconds(phase: dict, cfg: TranslatorConfig) -> float:
    """How long this phase should take if nothing goes wrong.

    Documented, not guessed, and computed per guard:

        distance      thresh / speed_straight
        wall          open_guard_max_distance / speed_straight
        front_object  open_guard_max_distance / speed_straight
        turn          radians(turn_magnitude_deg) / nominal_yaw_rate

    The two open guards have no bounded travel, which is the whole reason
    open_guard_max_distance exists: a wall that is never seen must still end
    the move.
    """
    if phase['mode'] == 'turn':
        return math.radians(cfg.turn_magnitude_deg) / cfg.nominal_yaw_rate
    guard = phase['guard']
    if guard == 'distance':
        return float(phase['thresh']) / cfg.speed_straight
    return cfg.open_guard_max_distance / cfg.speed_straight


def _timeout_sec(phase: dict, cfg: TranslatorConfig) -> int:
    """clamp(ceil(expected * factor + floor), TIMEOUT_MIN_SEC, TIMEOUT_MAX_SEC)."""
    raw = math.ceil(_expected_seconds(phase, cfg) * cfg.timeout_factor + cfg.timeout_floor)
    return int(max(TIMEOUT_MIN_SEC, min(TIMEOUT_MAX_SEC, raw)))


def _check_ranges(plan) -> None:
    """Enforce the per-guard thresh ranges intent_v1.json cannot express."""
    for phase in plan:
        if phase['mode'] != 'straight':
            continue
        guard = phase['guard']
        low, high = GUARD_THRESH_RANGE[guard]
        value = float(phase['thresh'])
        if not (low <= value <= high):
            raise IntentRangeError(guard, value, low, high)


def _intent_move(i, phase, cfg, prev_turn_sign, prov):
    """One intent phase -> one v3.0 move, recording provenance for every leaf.

    prev_turn_sign is the turn_sign of the most recent turn phase, or None,
    and is used only by the post_turn_uses_straight=False branch.
    """
    at = f'moves[{i}]'
    mode = phase['mode']

    # go_to validates against intent_v1.json but CANNOT BE EXECUTED by this
    # runtime, and the gap is not in this translator -- see the schema branch's
    # own description. mission_config.DRIVE_MODES is {'straight', 'wall_turn'}
    # and its comment says outright that "neither carries a target"; nothing
    # bridges a detected class to a pose either, because the two halves never
    # meet (runtime.DetectionInfo is (last_seen, score) with no position, and
    # Obstacle2D.msg is x/y/r with no class).
    #
    # Refused HERE, as an IntentSchemaError, for two reasons. Without this the
    # phase falls through to the straight branch below and dies on
    # phase['guard'] with a bare KeyError -- uncaught by _plan_v2, which
    # catches only PlanTranslationError subclasses, so it would surface as a
    # traceback in the planner rather than as a handled failure. And
    # IntentSchemaError is the one _plan_v2 feeds back to the model as retry
    # text, so the model is told to re-plan with the request in `unsupported`,
    # where the operator actually sees it.
    if mode == 'go_to':
        # Written for the MODEL, not for a log reader: _plan_v2 appends this
        # verbatim to the next prompt, where it competes with the system
        # prompt for attention. Italian to match the prompt and the wrapper
        # _plan_v2 puts around it; short, and only the three things the model
        # has to act on.
        raise UnsupportedIntentModeError(
            f'fase {i}: "go_to" non e\' eseguibile da questo robot. '
            'Non sostituirlo con "front_object": quello si ferma alla cosa '
            'piu\' vicina, non a quella nominata. '
            'Rimetti la richiesta in "unsupported".')

    move = {'id': f'move_{i}_{mode}'}
    prov[f'{at}.id'] = (DERIVED, 'f"move_{index}_{intent mode}"')

    if mode == 'turn':
        turn_sign = TURN_SIGN[phase['dir']]
        mag = _round_deg(cfg.turn_magnitude_deg)
        move['drive'] = {
            'mode': 'wall_turn',
            'turn_sign': turn_sign,
            'turn_mag_deg': mag,
            'speed': cfg.speed_turn,
        }
        # ONE variable, written into two fields. mission_config requires
        # stop_condition.value to equal abs(drive.turn_mag_deg) to 1e-6, and
        # re-deriving the second from the first is how a last-bit mismatch
        # gets in. This is the same discipline phases_to_mission() uses.
        move['stop_condition'] = {'type': 'orientation_delta', 'value': mag}
        prov[f'{at}.drive.mode'] = (INTENT, f'plan[{i}].mode "turn" -> "wall_turn"')
        prov[f'{at}.drive.turn_sign'] = (INTENT, f'plan[{i}].dir via TURN_SIGN')
        prov[f'{at}.drive.turn_mag_deg'] = (CONFIG, 'turn_magnitude_deg')
        prov[f'{at}.drive.speed'] = (CONFIG, 'speed_turn')
        prov[f'{at}.stop_condition.type'] = (CONST, 'orientation_delta')
        prov[f'{at}.stop_condition.value'] = (CONFIG, 'turn_magnitude_deg')
    else:
        guard = phase['guard']
        cond_type, thresh_key, extra = GUARD_MAP[guard]
        if cfg.post_turn_uses_straight or prev_turn_sign is None:
            move['drive'] = {'mode': 'straight', 'speed': cfg.speed_straight}
            prov[f'{at}.drive.mode'] = (INTENT, f'plan[{i}].mode "straight"')
        else:
            # D1 fallback. A zero magnitude is legal on a wall_turn whose stop
            # condition is not orientation_delta, and it is what "keep the
            # heading, do not turn again" spells in the old vocabulary.
            move['drive'] = {
                'mode': 'wall_turn',
                'turn_sign': prev_turn_sign,
                'turn_mag_deg': 0.0,
                'speed': cfg.speed_straight,
            }
            prov[f'{at}.drive.mode'] = (CONFIG, 'post_turn_uses_straight=False')
            prov[f'{at}.drive.turn_sign'] = (DERIVED, 'turn_sign of the preceding turn phase')
            prov[f'{at}.drive.turn_mag_deg'] = (CONST, '0.0 -- hold heading, do not turn again')
        prov[f'{at}.drive.speed'] = (CONFIG, 'speed_straight')

        stop = {'type': cond_type, thresh_key: _round_m(phase['thresh'])}
        prov[f'{at}.stop_condition.type'] = (INTENT, f'plan[{i}].guard {guard!r} via GUARD_MAP')
        prov[f'{at}.stop_condition.{thresh_key}'] = (
            INTENT, f'plan[{i}].thresh, rounded to 2 dp')
        for key, value in extra.items():
            stop[key] = value
            prov[f'{at}.stop_condition.{key}'] = (CONST, f'GUARD_MAP[{guard!r}] fixed param')
        move['stop_condition'] = stop

    move['timeout_sec'] = _timeout_sec(phase, cfg)
    prov[f'{at}.timeout_sec'] = (
        DERIVED,
        'clamp(ceil(expected_s * timeout_factor + timeout_floor), '
        f'{TIMEOUT_MIN_SEC}, {TIMEOUT_MAX_SEC}) -- see _expected_seconds')
    move['on_timeout'] = 'abort'
    prov[f'{at}.on_timeout'] = (CONST, 'abort')
    return move


def _validate_output(mission: dict) -> None:
    """Reject a mission the stack's own loader would reject, before it ships.

    Deliberately calls mission_config.parse_mission() rather than checking a
    separate JSON Schema copy of v3.0. There IS no such schema file in this
    repo, and writing one would create a second spelling of the rules that
    drifts from the loader the missions actually go through -- the loader is
    the contract. mission_config is pure stdlib (no rclpy), so importing it
    costs this module nothing.

    Then the three things a schema could not express anyway.
    """
    try:
        from f1tenth_behavior.mission.mission_config import MissionConfigError, parse_mission
    except ImportError as exc:  # pragma: no cover - packaging failure, not logic
        raise TranslatorOutputError(
            f'cannot validate output: f1tenth_behavior is not importable ({exc}). '
            'Refusing to emit an unvalidated mission.', mission) from exc

    try:
        parse_mission(mission)
    except MissionConfigError as exc:
        raise TranslatorOutputError(
            f'translator produced a mission the loader rejects: {exc}', mission) from exc

    moves = mission['moves']
    ids = [m['id'] for m in moves]
    if len(set(ids)) != len(ids):
        raise TranslatorOutputError(f'duplicate move ids: {ids}', mission)
    terminals = [i for i, m in enumerate(moves) if m.get('terminal')]
    if terminals != [len(moves) - 1]:
        raise TranslatorOutputError(
            f'terminal must be set on the last move and only there, got indices {terminals}',
            mission)


def translate(intent, *, config=None, explain=False) -> TranslationResult:
    """Translate an intent_v1 document into a validated mission v3.0.

    Emits nothing that is not INTENT, CONFIG, DERIVED or CONST -- see this
    section's header comment. Never returns a partial or patched mission: on
    any failure it raises one of IntentSchemaError, IntentRangeError,
    EmptyPlanError or TranslatorOutputError, so the caller surfaces the
    specific reason instead of running something approximate.

    With explain=True the result carries a `provenance` dict mapping every
    emitted path to its source, and for DERIVED fields the formula.

    ON TURN MAGNITUDE (decision D2, ANSWERED -- the work order's premise was
    wrong). The behaviour tree does NOT force 90 degrees: condition_eval's
    orientation_delta branch ends the move on
    `abs(turn_accum_deg) >= abs(value)`, whatever `value` is, and
    mission_config requires that value to equal abs(drive.turn_mag_deg). So
    the magnitude is honoured end to end and turn_magnitude_deg is a genuine
    config choice, not the only reachable number. It stays at 90.0 because 90
    is the only magnitude any of this has been run at on hardware, and because
    the MPC side of a >90 turn is unverified -- not because anything clamps it.
    """
    cfg = config if config is not None else TranslatorConfig()

    try:
        jsonschema.validate(intent, load_intent_schema())
    except jsonschema.ValidationError as exc:
        raise IntentSchemaError(f'intent does not validate: {exc.message}') from exc

    plan = intent['plan']
    unsupported = tuple(intent['unsupported'])
    _check_ranges(plan)

    if not plan:
        raise EmptyPlanError(
            'intent carries no executable phase; nothing to translate. '
            f'unsupported={list(unsupported)}')

    prov = {}
    moves = []
    prev_turn_sign = None
    for i, phase in enumerate(plan):
        moves.append(_intent_move(i, phase, cfg, prev_turn_sign, prov))
        if phase['mode'] == 'turn':
            prev_turn_sign = TURN_SIGN[phase['dir']]

    moves[-1]['terminal'] = True
    prov[f'moves[{len(moves) - 1}].terminal'] = (DERIVED, 'True on the last move only')

    mission_id = '{}_{}'.format(
        cfg.mission_id_prefix,
        hashlib.sha1(_canonical_json(intent).encode('utf-8')).hexdigest()[:12])
    mission = {
        'mission_id': mission_id,
        'schema_version': MISSION_SCHEMA_VERSION,
        'moves': moves,
    }
    # Deterministic BY CONSTRUCTION: sha1 over the canonical intent, never a
    # timestamp. The same intent yields the same mission byte for byte, which
    # is what makes the golden fixtures stable and lets a re-issued command be
    # recognised as the same mission.
    prov['mission_id'] = (DERIVED, 'f"{mission_id_prefix}_{sha1(canonical_json(intent))[:12]}"')
    prov['schema_version'] = (CONST, MISSION_SCHEMA_VERSION)

    _validate_output(mission)

    return TranslationResult(
        mission=mission,
        requires_confirmation=bool(unsupported),
        unsupported=unsupported,
        provenance=prov if explain else None,
    )


# Name of the versioned prompt that teaches a model the intent vocabulary.
# A FILE, not a string literal in a node, so it can be diffed, reviewed and
# pinned independently of the code that sends it -- and so the test below can
# parse its worked examples and check them against the schema.
INTENT_PROMPT_FILENAME = 'planner_system_prompt.v2.it.txt'


def _prompt_path(filename: str) -> pathlib.Path:
    """Locate a prompt file, in the source tree or the install share."""
    here = pathlib.Path(__file__).resolve().parent.parent
    local = here / 'prompts' / filename
    if local.is_file():
        return local
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory('llm')) / 'prompts' / filename


def load_intent_prompt(filename: str = INTENT_PROMPT_FILENAME) -> str:
    """Return the system prompt that teaches the intent vocabulary.

    NOT yet wired into llm_planner_node: that node still runs the legacy
    normalize_plan/validate_plan/phases_to_mission path against
    SYSTEM_PROMPT. Switching it over is a separate change -- see
    llm/README.md, "Wiring this in".
    """
    with open(_prompt_path(filename), encoding='utf-8') as fh:
        return fh.read()


def intent_prompt_examples(filename: str = INTENT_PROMPT_FILENAME):
    """Extract the worked examples from the prompt, as (command, intent) pairs.

    The prompt's examples are the model's strongest signal, and a wrong one
    teaches the wrong output directly. Parsing them here lets the test suite
    hold them to the same schema everything else is held to.
    """
    pairs = []
    pending = None
    for line in load_intent_prompt(filename).splitlines():
        stripped = line.strip()
        if stripped.startswith('"') and stripped.endswith('"') and len(stripped) > 1:
            pending = stripped[1:-1]
        elif stripped.startswith('{') and pending is not None:
            pairs.append((pending, json.loads(stripped)))
            pending = None
    return pairs
