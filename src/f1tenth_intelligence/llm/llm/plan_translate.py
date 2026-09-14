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

Deliberately pure Python, no rclpy/ROS import of any kind -- fully
unit-testable in isolation (see test/test_plan_translate.py) the same way
mission_config.py itself is. llm_planner_node.py (this package's ROS-facing
node) is the only caller.

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

import math
import time
from typing import Optional

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
