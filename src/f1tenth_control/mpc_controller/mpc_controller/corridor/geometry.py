"""
Corridor geometry for mpc_corr: pure functions, no rclpy, no node state.

Everything here takes plain values and returns plain values. What used to be
written onto the node during a corridor build comes back in the result
(CorridorBuild.updates) for the caller to apply, and what used to be logged
comes back as LogLine data for the caller to emit: the caller owns the logger
because rclpy keys its throttling by call site, so each kind of line must keep
a call site of its own (see MPC_corr's _CORRIDOR_LOG_SITES).

Inputs the node has to measure first -- the fresh front distance, the wall
tracker's d_wall, the d_wall heading correction, the reprojected anchor --
arrive already measured in CorridorState. Side effects the build used to
trigger (committing the wall tracker and selecting the wall) are left to the
caller too: CorridorBuild.wall says what the wall_turn branch decided.

The shape of the curves themselves -- the S-curve heading ramp, the widening
funnel, the pose-to-pose Bezier -- is f1tenth_params.corridor_geometry's, and
is called, not copied: the test-campaign plotter re-evaluates logged
corridors through that same module.
"""

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, NamedTuple, Optional, Tuple

from f1tenth_params.corridor_geometry import (
    CORRIDOR_HANDLE_FRAC, POSE_HANDLE_FRAC, corridor_curves,
    corridor_curves_to_pose, wrap_pi)
from mpc_controller.object_approach import (
    build_object_centreline, plan_object_heading)
from mpc_controller.wall_tracker import PROVENANCE_NAMES
from mpc_controller.wall_turn import min_turn_radius, plan_wall_turn_step
import numpy as np


class LogLine(NamedTuple):
    """One log line as data: the call site that must emit it, and its text."""

    site: str
    text: str


# ----------------------------------------------------------------------------
# Frame helpers
# ----------------------------------------------------------------------------
def pose_odom_to_map(
        x: float, y: float, psi: float,
        tf_x: float, tf_y: float, tf_yaw: float) -> Tuple[float, float, float]:
    """
    Express an odom-frame planar pose (x, y, psi) in the MAP frame.

    Given the map -> odom transform (tf_x, tf_y, tf_yaw).

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
    (pose_map_to_odom, the exact inverse of this), makes that real drift
    visible to the solver as an ordinary tracking error, without touching the
    odom-frame state x0 the dynamics/solve themselves run on.
    """
    cos_t, sin_t = math.cos(tf_yaw), math.sin(tf_yaw)
    x_map = tf_x + cos_t * x - sin_t * y
    y_map = tf_y + sin_t * x + cos_t * y
    psi_map = math.atan2(math.sin(psi + tf_yaw), math.cos(psi + tf_yaw))
    return x_map, y_map, psi_map


def pose_map_to_odom(
        x: float, y: float, psi: float,
        tf_x: float, tf_y: float, tf_yaw: float) -> Tuple[float, float, float]:
    """
    Express a map-frame planar pose in the CURRENT odom frame.

    Exact inverse of pose_odom_to_map (see that function for the frame
    convention and for why either direction exists at all), given the SAME
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


def project_onto_line(
        point: Tuple[float, float], line_origin: Tuple[float, float],
        line_yaw: float) -> float:
    """
    Return the signed along-line progress of `point` past `line_origin`.

    Measured along the direction `line_yaw` -- i.e. the scalar s in
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
    original line (see build_corridor's goal_distance branch, and "THE PRICE"
    in its own comment) -- so a sideways offset persists for the rest of the
    move and this is the ONLY thing keeping "6 m" meaning 6 m along the
    intended direction.

    ONE CALLER, deliberately: control_loop's goal_distance termination check.
    The corridor build used this too until the corridor stopped homing
    laterally; it does not any more, and must not start again -- the
    corridor's origin and the move's progress anchor are two separate
    quantities now (see the two-anchor note at that branch's own assignment).

    Negative means the car is BEHIND the origin along that direction (it
    happens: an avoidance manoeuvre can back the projection up), and is
    returned as-is rather than clamped, so a caller that cares can see it.
    """
    dx = float(point[0]) - float(line_origin[0])
    dy = float(point[1]) - float(line_origin[1])
    return dx * math.cos(line_yaw) + dy * math.sin(line_yaw)


# ----------------------------------------------------------------------------
# Corridor build
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class CorridorConfig:
    """
    The node's corridor knobs, as read for one build.

    Field names are the node attribute names they are read from, so the
    mapping in MPC_corr's build_straight_corridor stays a list of identities.
    Fields only one branch reads may be None for a caller that never takes
    that branch (the duck-typed test stand-ins carry only what their branch
    needs); the defaults are the ones the build has always used for an
    absent attribute.
    """

    corr_L_base: float
    corr_N: int
    corr_wmin: float
    corr_wmax: float
    corr_turn_u_start: float
    corr_turn_u_end: float
    corridor_heading_return: bool
    object_corridor_mode: str = 'off'
    object_arc_switch_hi_frac: float = 1.12
    object_arc_switch_lo_frac: float = 0.88
    corr_lookahead_reach_margin: Optional[float] = None
    corr_wall_turn_k_safety: Optional[float] = None
    corr_wall_turn_safety_margin_m: Optional[float] = None
    object_r_full: Optional[float] = None
    object_r_freeze: Optional[float] = None
    object_c_safety: Optional[float] = None
    object_lead_in_m: Optional[float] = None
    N: Optional[int] = None
    ts: Optional[float] = None
    params: Optional[Dict[str, float]] = None
    limits: Optional[Dict[str, float]] = None


@dataclass(frozen=True)
class CorridorState:
    """
    Everything live one build reads, already measured by the caller.

    x is the solver state x0 (only x, y, psi are read). front_distance is the
    raw last /perception/front_distance value; d_front_fresh is the same
    reading run through the node's staleness check (wall_turn only). d_wall,
    wall_provenance and wall_inlier_count are the wall tracker's, taken before
    the build when it already holds a wall. d_wall_correction is the d_wall
    heading correction for a straight drive, 0.0 when absent or stale.
    goal_anchor_odom is the anchor ALREADY reprojected into this tick's odom
    frame.
    """

    x: Any
    front_distance: float
    vdes: Optional[float] = None
    drive_cmd: Optional[Dict[str, Any]] = None
    psi_init_corridor: Optional[float] = None
    goal_anchor_odom: Optional[Tuple[float, float, float]] = None
    goal_pose_xy: Optional[Tuple[float, float]] = None
    object_target: Optional[Tuple[float, float]] = None
    object_psi_c: Optional[float] = None
    object_standoff: Optional[float] = None
    object_arc_active: bool = False
    object_behind_terminal: bool = False
    turn_progress_rad: Optional[float] = None
    wall_turn_committed: Optional[bool] = None
    wall_turn_commanded_rot: Optional[float] = None
    d_front_fresh: Optional[float] = None
    d_wall: Optional[float] = None
    wall_provenance: Optional[int] = None
    wall_inlier_count: Optional[int] = None
    d_wall_correction: float = 0.0


class WallTurnDecision(NamedTuple):
    """What the wall_turn branch decided, for the caller's wall tracker."""

    committed: bool
    was_committed: bool
    psi_commit: float
    d_front: Optional[float]
    pose: Tuple[float, float, float]


@dataclass
class CorridorBuild:
    """
    One corridor build's result.

    corridor   the corridor dict, without refStep (the caller measures that
               against its own previous corridor).
    updates    node attributes the build decided, by attribute name, in the
               order they were decided; only the ones this branch writes.
    logs       lines to emit after any wall-tracker side effects.
    build_log  the closing CORR/build line, emitted after refStep.
    wall       the wall_turn branch's decision, None on every other branch.
    """

    corridor: Dict[str, Any]
    updates: Dict[str, Any] = field(default_factory=dict)
    logs: List[LogLine] = field(default_factory=list)
    build_log: Optional[LogLine] = None
    wall: Optional[WallTurnDecision] = None


def branch_of(drive_cmd, object_target, object_psi_c) -> str:
    """
    Name the build branch a goal selects.

    'wall_turn' / 'drive' for a drive command, 'object' for an object move
    with a held heading, 'goal' for everything else (goal_pose / goal_turn
    and goal_distance, which build_corridor tells apart itself). The caller
    uses it to measure only the inputs that branch reads.
    """
    if drive_cmd is not None:
        return 'wall_turn' if drive_cmd['mode'] == 'wall_turn' else 'drive'
    if object_target is not None and object_psi_c is not None:
        return 'object'
    return 'goal'


def build_corridor(state: CorridorState, cfg: CorridorConfig) -> CorridorBuild:
    """
    Build the reference corridor for the active goal.

    CORRIDOIO -- sempre dritto, l'evitamento ostacoli e' interamente delegato
    a local_target / deflect_target (deflessione tangenziale del target) e al
    peso w_obs nel solver MPC.
    """
    x = state.x
    X0 = float(x[0])
    Y0 = float(x[1])
    psi0 = float(x[2])

    d_front = float(state.front_distance)

    psiStart = psi0

    logs: List[LogLine] = []
    updates: Dict[str, Any] = {}
    wall = None

    # Signed rotation THIS corridor asks for, measured from psiStart, for a
    # wall_turn (the increment, not the whole remaining turn -- see that
    # branch). None on every other branch, and the solver's terminal yaw cost falls
    # straight back to its original shortest-branch unwrap when it is
    # absent -- so every non-wall_turn corridor is byte-identical to
    # before. Set only inside the wall_turn branch below.
    turn_remaining = None

    # True only on the object branch. Carried into the corridor dict so
    # corridor_lookahead can tell "the lookahead reaches past the end
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

    drive_cmd = state.drive_cmd
    object_target = state.object_target
    object_psi_c = state.object_psi_c
    branch = branch_of(drive_cmd, object_target, object_psi_c)

    if branch in ('wall_turn', 'drive'):
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
        psi_base = (state.psi_init_corridor
                    if state.psi_init_corridor is not None else psi0)
        if branch == 'wall_turn':
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
            # dFront, exactly as before the tracker existed.
            #
            # NOTE d_front is REBOUND here to the freshness-checked reading,
            # which is None when stale or absent, and it is that value the
            # corridor dict's dFront and the CORR/build line read below.
            d_front = state.d_front_fresh
            d_wall = state.d_wall
            d_rule = d_wall if d_wall is not None else d_front
            was_committed = state.wall_turn_committed
            wall_step = plan_wall_turn_step(
                signed_total, state.turn_progress_rad,
                d_rule,
                wheelbase=cfg.params['L'],
                delta_min=cfg.limits['delta_min'],
                delta_max=cfg.limits['delta_max'],
                k_safety=cfg.corr_wall_turn_k_safety,
                safety_margin=cfg.corr_wall_turn_safety_margin_m,
                n_steps=cfg.N, ts=cfg.ts, v_ref=state.vdes,
                committed=state.wall_turn_committed,
                prev_commanded_rot=state.wall_turn_commanded_rot)
            updates['wall_turn_committed'] = wall_step.committed
            updates['wall_turn_commanded_rot'] = wall_step.commanded_rot
            turn_remaining = wall_step.dpsi_this
            psiEnd = psi0 + wall_step.dpsi_this
            # SELECT THE WALL ON THE REBUILD THE TURN COMMITS. The gating
            # references (this heading, this dFront) are frozen then and
            # never move; a rebuild with no usable scan retries the
            # selection against those same references, which is not
            # re-gating a selected wall -- there is none yet. The caller
            # owns the tracker, so the decision goes back to it.
            wall = WallTurnDecision(
                committed=wall_step.committed, was_committed=was_committed,
                psi_commit=psi0, d_front=d_front, pose=(X0, Y0, psi0))
            d_avail_txt = ('unknown' if wall_step.d_avail is None
                           else f'{wall_step.d_avail:.2f}')
            if d_wall is not None:
                d_rule_txt = (f'd_wall={d_wall:.2f} '
                              f'({PROVENANCE_NAMES[state.wall_provenance]}, '
                              f'{state.wall_inlier_count} returns)')
            else:
                d_rule_txt = 'd_wall=none'
            logs.append(LogLine(
                'wall_turn',
                f'CORR/wall_turn | dFront={state.front_distance:.2f} '
                f'{d_rule_txt} '
                f'd_avail={d_avail_txt} '
                f'rem={wall_step.dpsi_rem:+.4f}/{signed_total:+.4f} '
                f'R_min={wall_step.r_min:.3f} '
                f'by_dist={wall_step.dpsi_by_dist:.4f} '
                f'by_horizon={wall_step.dpsi_by_horizon:.4f} '
                f'committed={wall_step.committed} held={wall_step.held} '
                f'dpsi_this={wall_step.dpsi_this:+.4f} '
                f'cmd_rot={wall_step.commanded_rot:+.4f}'))
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
            # MPC_corr's __init__ for why here and not at dpsi_this, and
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
            dpsi_d_wall = state.d_wall_correction
            psiEnd = psi_base + dpsi_d_wall
            if dpsi_d_wall != 0.0:
                logs.append(LogLine(
                    'd_wall',
                    f'CORR/d_wall | psi_base={psi_base:+.4f} '
                    f'dpsi_d_wall={dpsi_d_wall:+.4f} -> psiEnd={psiEnd:+.4f}'))
            if cfg.corridor_heading_return:
                psiStart = psi0
            else:
                psiStart = psiEnd
        # No goal to clip the length against (that is what open-ended
        # means), so the full nominal corridor length, exactly as the
        # goal_distance branch uses.
        L = max(cfg.corr_L_base, 1.0)
        logs.append(LogLine(
            'drive',
            f'CORR/drive | mode={drive_cmd["mode"]} '
            f'psi_base={psi_base:+.4f} psiStart={psiStart:+.4f} '
            f'psiEnd={psiEnd:+.4f} L={L:.2f}'))

    elif branch == 'object':
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
            state.object_standoff,
            r_full=cfg.object_r_full,
            r_freeze=cfg.object_r_freeze,
            c_safety=cfg.object_c_safety,
            wheelbase=cfg.params['L'],
            delta_min=cfg.limits['delta_min'],
            delta_max=cfg.limits['delta_max'],
            n_steps=cfg.N, ts=cfg.ts, v_ref=state.vdes)
        updates['object_psi_c'] = step.psi_c_new
        updates['object_last_step'] = step
        updates['object_target_at_build'] = object_target

        origin, psi_c, L = build_object_centreline(
            (X0, Y0), object_target, step.psi_c_new,
            state.object_standoff, cfg.corr_N,
            behind=cfg.object_lead_in_m)
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
        _mode = cfg.object_corridor_mode
        if _mode == 'arc':
            use_arc = True
            clamp_len = float('nan')
        elif _mode == 'arc_far':
            clamp_len = lookahead_clamp_length(
                cfg.N, cfg.ts, state.vdes,
                cfg.corr_lookahead_reach_margin)
            # Hysteresis: enter the arc above hi, leave it below lo, hold
            # in between. See the parameter block for why the band cannot
            # be a single threshold.
            if r_goal >= cfg.object_arc_switch_hi_frac * clamp_len:
                use_arc = True
            elif r_goal <= cfg.object_arc_switch_lo_frac * clamp_len:
                use_arc = False
            else:
                use_arc = state.object_arc_active
        else:
            use_arc = False
            clamp_len = float('nan')
        updates['_object_arc_active'] = use_arc

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
            if r_goal <= max(float(cfg.corr_L_base), 1e-3):
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
        # persistent flag is the gate.
        if not use_arc:
            # ON A STRAIGHT OBJECT CORRIDOR THE CENTRELINE *IS* THE TARGET
            # LINE. w_corr already owns that error; adding w_line on top
            # would charge the same deviation twice and double the lateral
            # stiffness (1.25 + 1.25) on the DEFAULT path, where nothing
            # about the geometry has changed at all. The split exists only
            # because the arc makes them two different curves.
            target_line = None
        elif state.object_behind_terminal:
            target_line = None
            logs.append(LogLine(
                'object_behind',
                'CORR/object | target behind the car: the target-line cost '
                'is OFF for this corridor'))
        else:
            target_line = {'p': [float(origin[0]), float(origin[1])],
                           'psi': float(psi_c)}
        logs.append(LogLine(
            'object_shape',
            f'CORR/object_shape | mode={_mode} '
            f'shape={object_corridor_shape} r_goal={r_goal:.2f} '
            f'clamp={clamp_len:.2f} '
            f'band=[{cfg.object_arc_switch_lo_frac * clamp_len:.2f},'
            f'{cfg.object_arc_switch_hi_frac * clamp_len:.2f}] '
            f'dpsi={wrap_pi(psiEnd - psiStart):+.4f}'))
        object_mode = True
        logs.append(LogLine(
            'object',
            f'CORR/object | psi_c={step.psi_c_new:+.4f} e={step.e:+.4f} '
            f'k={step.k:.3f} dpsi_max={step.dpsi_max:.4f} r={step.r:+.3f} '
            f'L={L:.3f}'))

    elif state.goal_pose_xy is not None:
        gx, gy = state.goal_pose_xy
        psiEnd = math.atan2(gy - Y0, gx - X0)
        L = float(np.clip(math.hypot(gx - X0, gy - Y0), 1.0, cfg.corr_L_base))
    else:
        # STRAIGHT (goal_distance) move: a corridor that ALWAYS PASSES
        # THROUGH THE CAR'S CURRENT POSITION, pointed along a FROZEN,
        # drift-corrected HEADING. Position tracks the car; direction does
        # not. That split is the whole design of this branch.
        #
        # WHAT THE HEADING IS, and why it is the half worth freezing: it is
        # goal_anchor_odom's psi -- the move's MAP-frame start heading,
        # reprojected into THIS tick's odom frame by
        # CorridorManager.refresh_anchor() once per control_loop tick. It
        # must NOT be a constant in odom coordinates: this stack's dual-EKF
        # split routes the car's real heading drift into the map -> odom
        # edge rather than into odom-frame yaw, so a heading held constant
        # in odom coordinates is a heading rotating with the error, and
        # presents the solver nothing to correct. See pose_odom_to_map's own
        # docstring for the measurement (13.1 deg of real yaw drift against
        # under 0.5 deg of odom-frame yaw). Reprojecting it turns that drift
        # back into an ordinary heading error the w_psi/w_term costs oppose
        # -- confirmed live: the accumulated drift this was built to kill is
        # gone.
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
        # the lookahead target taken on its centerline (local_target
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
        # target taken on the centerline (local_target -> pref_nom
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
        psi_ref = state.psi_init_corridor
        anchor_pose = state.goal_anchor_odom
        if anchor_pose is not None:
            # ---- TWO SEPARATE ANCHORS. Do not collapse these into one. ----
            # progress_anchor_xy: the move's ORIGINAL start point (frozen,
            #   map-corrected). Owned by control_loop's termination check;
            #   read here ONLY for the lat_off diagnostic below. Nothing in
            #   this function's geometry is allowed to depend on it.
            # corridor_origin (X0, Y0): the car's CURRENT position, which is
            #   already expressed in the live odom frame the corridor, the
            #   markers and the solver's x0 all live in -- so it needs no
            #   reprojection of its own (odom -> map -> odom through the same
            #   latest transform is exactly the identity; see
            #   pose_map_to_odom, the exact inverse of pose_odom_to_map).
            #   X0/Y0 are therefore deliberately left as received, NOT
            #   recomputed onto the frozen line.
            progress_anchor_xy = (float(anchor_pose[0]), float(anchor_pose[1]))
            # Frozen target heading; the ORIGIN end depends on the
            # parameter -- see the "WHICH END OF THE HEADING BLEND IS
            # FROZEN" note above.
            psiEnd = float(anchor_pose[2])
            if cfg.corridor_heading_return:
                psiStart = psi0
            else:
                # THE DEFAULT since 2026-09-08: both ends frozen, dpsi == 0,
                # S-curve inert on this branch. It measures BETTER on the
                # heading return at every rebuild period tested -- see the
                # corridor_heading_return parameter's own note in MPC_corr.
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
            logs.append(LogLine(
                'tracking',
                f'CORR/tracking | origin=({X0:+.3f},{Y0:+.3f}) '
                f'psi_live={psiStart:+.4f} psi_frozen={psiEnd:+.4f} '
                f'progress_anchor=({progress_anchor_xy[0]:+.3f},'
                f'{progress_anchor_xy[1]:+.3f}) lat_off={lat_off:+.3f}'))
        else:
            # Bootstrap fallback, unchanged from before this fix: no move
            # start has been recorded yet (no goal_distance received, or
            # only the odom bootstrap has set psi_init_corridor), so there
            # is no frozen line to reference. Blend from the live yaw toward
            # whatever heading reference does exist, off the live position.
            psiEnd = float(psi_ref) if psi_ref is not None else psi0
        L = max(cfg.corr_L_base, 1.0)

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
    # Without limits and params (a caller that has none, e.g. a duck-typed
    # stand-in) there is no R_min to clamp against, so the clamp is simply
    # off and the geometry is what it was before it existed.
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
         else min(L_full, max(float(cfg.corr_L_base), 1e-3)))
    corridor_cut = L < L_full - 1e-9
    _scale = (L_full / L) if corridor_cut and L > 0.0 else 1.0
    u_start_use = min(1.0, cfg.corr_turn_u_start * _scale)
    u_end_use = min(1.0, cfg.corr_turn_u_end * _scale)
    if corridor_cut:
        ramp_truncated = cfg.corr_turn_u_end * L_full > L + 1e-9
        logs.append(LogLine(
            'cut',
            f'CORR/cut | target at {L_full:.2f} m is beyond corr_L_base='
            f'{cfg.corr_L_base:.2f} m: corridor cut to {L:.2f} m, ramp '
            f'u=[{u_start_use:.3f}, {u_end_use:.3f}]'
            + (' -- RAMP TRUNCATED: the turn is planned over the cut '
               'corridor, not over the full approach' if ramp_truncated
               else '')))

    _dpsi_for_bound = (turn_remaining if turn_remaining is not None
                       else wrap_pi(psiEnd - psiStart))
    _r_min = None
    if cfg.limits is not None and cfg.params is not None:
        _bound = (cfg.limits['delta_max'] if _dpsi_for_bound >= 0.0
                  else cfg.limits['delta_min'])
        _r_min = min_turn_radius(cfg.params['L'], _bound)
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
            cfg.corr_N,
            w0=cfg.corr_wmin,
            w1=cfg.corr_wmax,
            r_min=_r_min,
            length_ref=cfg.corr_L_base,
        )
        # THE CAP APPLIES TO THIS SHAPE TOO, and it has to be checked
        # after the fit rather than before it: the pose corridor's length
        # is an OUTPUT (an arc between two poses is longer than the chord
        # between them -- measured 1.008x median, 1.14x worst over the
        # archived rebuilds), so a target inside corr_L_base can still
        # produce a corridor past it. Rejected rather than trimmed,
        # because a trimmed pose corridor no longer ends on the pose that
        # is its whole reason for existing.
        _over_cap = float(geom['length']) > max(float(cfg.corr_L_base),
                                                1e-3) + 1e-9
        if geom['feasible'] and not _over_cap:
            object_corridor_shape = 'pose_arc'
            L = float(geom['length'])
            L_full = L
        elif _over_cap:
            logs.append(LogLine(
                'pose_over_cap',
                f'CORR/pose | the fit is {geom["length"]:.2f} m long for a '
                f'{L_full:.2f} m chord, past corr_L_base='
                f'{cfg.corr_L_base:.2f} m: building the cut arc instead'))
            pose_target = None
            object_corridor_shape = 'arc'
            L = min(L_full, max(float(cfg.corr_L_base), 1e-3))
            corridor_cut = L < L_full - 1e-9
            _scale = (L_full / L) if corridor_cut and L > 0.0 else 1.0
            u_start_use = min(1.0, cfg.corr_turn_u_start * _scale)
            u_end_use = min(1.0, cfg.corr_turn_u_end * _scale)
        else:
            logs.append(LogLine(
                'pose_infeasible',
                f'CORR/pose | ending on the target needs kappa='
                f'{geom["kappa_max"]:.3f} 1/m (R='
                f'{1.0 / max(geom["kappa_max"], 1e-9):.2f} m) against '
                f'R_min={_r_min:.3f} m at {L_full:.2f} m to go: falling '
                'back to the straight corridor pinned to the target line'))
            pose_target = None
            # Back to the geometry that branch would have built without
            # the arc at all -- the straight corridor on the target line,
            # not the ramp arc, because the ramp arc's own endpoint drift
            # is worst exactly here.
            X0, Y0 = origin
            psiStart = psi_c
            psiEnd = psi_c
            L = min(L_full, max(float(cfg.corr_L_base), 1e-3))
            corridor_cut = L < L_full - 1e-9
            object_corridor_shape = 'straight'
            updates['_object_arc_active'] = False
            # AND THE TARGET LINE GOES WITH IT. It was set above because
            # the arc was chosen, and on the straight corridor the
            # centreline IS the target line -- leaving it would charge the
            # same lateral error through w_corr and w_line at once, which
            # is the doubling the split exists to avoid.
            target_line = None

    if pose_target is None:
        geom = corridor_curves(
            X0, Y0, psiStart, psiEnd, L, cfg.corr_N,
            dpsi=turn_remaining,
            u_start=u_start_use,
            u_end=u_end_use,
            w0=cfg.corr_wmin,
            w1=cfg.corr_wmax,
            # BOTH CLAMPS APPLY TO EVERY BRANCH, not only the object one.
            # The ramp span is a fraction of L, so peak curvature runs away
            # as the corridor shortens; the straight branch is unprotected
            # without this and goes past full lock beyond ~50 degrees of
            # heading error at L 3.0. wall_turn caps its own dpsi
            # separately, so there this is belt-and-braces.
            r_min=_r_min,
            # And the funnel opens at a fixed rate per metre rather than
            # reaching corr_wmax whatever the length: 10 of the 24 archived
            # object rebuilds were WIDER THAN LONG without this.
            length_ref=cfg.corr_L_base,
        )
    if not geom['feasible'] and pose_target is None:
        logs.append(LogLine(
            'curvature',
            f'CORR/curvature | dpsi={geom["dpsi"]:+.4f} over L={L:.2f} m '
            f'needs a ramp longer than the corridor: kappa_max='
            f'{geom["kappa_max"]:.3f} (R={1.0 / max(geom["kappa_max"], 1e-9):.2f} m) '
            f'against R_min={_r_min:.3f} m. '
            'The reference asks for more than full lock; the solver will '
            'saturate steering.'))
    xc, yc = geom["xc"], geom["yc"]
    xL, yL = geom["xL"], geom["yL"]
    xR, yR = geom["xR"], geom["yR"]
    halfWidth = geom["halfWidth"]
    dpsi = geom["dpsi"]
    p_goal = geom["Pend"]

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
            "corr_N": int(cfg.corr_N),
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
            "w0": float(cfg.corr_wmin),
            "w1": float(cfg.corr_wmax),
            "handle_frac": float(CORRIDOR_HANDLE_FRAC),
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
            "object_corridor_mode": str(cfg.object_corridor_mode),
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
        # always corr_L_base). corridor_lookahead derives the lookahead
        # from it -- see that function.
        "L": float(L),
        # See object_mode's own declaration at the top of this function.
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

    # ---- DEBUG: geometria del corridoio appena costruito ----
    build_log = LogLine(
        'build',
        f'CORR/build | L={L:.2f} psiStart(live)={psiStart:+.4f} '
        f'psiEnd(frozen)={psiEnd:+.4f} '
        f'dpsi={dpsi:+.4f} halfWidth=[{halfWidth[0]:.3f}..{halfWidth[-1]:.3f}] '
        f'dFront={d_front:.2f}')

    return CorridorBuild(corridor=corridor, updates=updates, logs=logs,
                         build_log=build_log, wall=wall)


def corridor_reference(corridor) -> Dict[str, Any]:
    """Return what reference_step compares between rebuilds, for one corridor."""
    xc = np.asarray(corridor['xc'], dtype=float)
    yc = np.asarray(corridor['yc'], dtype=float)
    pend = np.asarray(corridor['Pend'], dtype=float)
    tan_now = math.atan2(float(corridor['ty'][0]), float(corridor['tx'][0]))
    return {'xc': xc, 'yc': yc, 'Pend': pend, 'tan': tan_now}


def reference_step(ref, prev) -> Dict[str, Optional[float]]:
    """
    Measure how far a corridor's reference moved from the previous one.

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

    ref and prev are corridor_reference() results; prev may be None.
    """
    if prev is None:
        return {'centreline_m': None, 'tangent_rad': None, 'pend_m': None}
    d = np.hypot(ref['xc'][:, None] - prev['xc'][None, :],
                 ref['yc'][:, None] - prev['yc'][None, :])
    return {
        'centreline_m': float(d.min(axis=1).max()),
        'tangent_rad': float(wrap_pi(ref['tan'] - prev['tan'])),
        'pend_m': float(np.hypot(*(ref['Pend'] - prev['Pend']))),
    }


# ----------------------------------------------------------------------------
# Lookahead target
# ----------------------------------------------------------------------------
def lookahead_clamp_length(n_steps, ts, v_des, reach_margin):
    """
    Return the corridor length at and below which the lookahead pins to the end.

    The lookahead is max(corr_lookahead_frac * L, this). The fractional term is
    0.5 * L, always inside the corridor, so it never pins; this floor is the
    only thing that can, and it does so exactly when it reaches L. At the
    shipping geometry (N 20, ts 0.1, vdes 0.5, margin 1.25) that is 1.25 m.

    A MODULE FUNCTION, not a method, because two callers need it and one of
    them is reached through the duck-typed corridor stand-ins: those bind
    _corridor_lookahead onto objects that carry the four values below but no
    methods, so a self.<method>() call there raises instead of computing. The
    other caller is the object branch's arc switch, which must stop using the
    arc before Pend starts feeding the terminal cost -- writing 1.25 there
    instead would leave the switch behind the moment any of these four moved.
    """
    return float(reach_margin) * float(n_steps) * float(ts) * float(v_des)


def corridor_lookahead(L, lookahead_frac, n_steps, ts, vdes, reach_margin,
                       object_mode) -> Tuple[float, List[LogLine]]:
    """
    Derive the target lookahead from the corridor length L.

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
    logs = []
    from_length = lookahead_frac * L
    # The floor and the raw reach it is derived from: the warning below
    # quotes both, because "inside the horizon reach" and "below the floor"
    # are different statements and the fix differs.
    horizon_reach = float(n_steps) * float(ts) * float(vdes)
    reach_floor = lookahead_clamp_length(n_steps, ts, vdes, reach_margin)

    lookahead = from_length
    if from_length < reach_floor:
        lookahead = reach_floor
        if object_mode:
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
            logs.append(LogLine(
                'lookahead_object',
                f'TGT/object | lookahead {lookahead:.2f} >= corridor '
                f'{L:.2f}: pref_nom clamps to the goal (expected on the '
                'final approach)'))
            return float(lookahead), logs
        logs.append(LogLine(
            'lookahead_floor',
            f'TGT/geometry | corridor L={L:.2f} gives lookahead '
            f'{from_length:.2f} m, inside the horizon reach '
            f'{horizon_reach:.2f} m (floor {reach_floor:.2f}): the terminal '
            f'cost becomes an ARRIVAL target, not a direction pull. '
            f'Using the floor. Raise corr_L_base to at least '
            f'{reach_floor / max(lookahead_frac, 1e-6):.2f}.'))
        if lookahead >= L:
            logs.append(LogLine(
                'lookahead_clamped',
                f'TGT/geometry | lookahead {lookahead:.2f} >= corridor '
                f'length {L:.2f}: the target clamps to the corridor end on '
                f'every cycle and never advances.'))
    return float(lookahead), logs


def local_target(x, corridor, lookahead) -> Tuple[np.ndarray, List[LogLine]]:
    """
    Return the nominal lookahead target on the centreline, before any obstacle.

    The point `lookahead` metres of arclength past the centreline sample
    nearest the car, clamped to the corridor's last sample.
    """
    p_robot = np.array([x[0], x[1]], dtype=float)

    xc = np.asarray(corridor["xc"], dtype=float)
    yc = np.asarray(corridor["yc"], dtype=float)

    d2 = (xc - p_robot[0]) ** 2 + (yc - p_robot[1]) ** 2
    idx = int(np.argmin(d2))

    ds = np.sqrt(np.diff(xc) ** 2 + np.diff(yc) ** 2)
    s_cum = np.concatenate(([0.0], np.cumsum(ds)))

    s_target = s_cum[idx] + lookahead
    idx_target = int(np.searchsorted(s_cum, s_target))
    idx_target = min(idx_target, len(xc) - 1)

    p_target = np.array(
        [xc[idx_target], yc[idx_target]],
        dtype=float
    )
    # ---- DEBUG: indice piu' vicino e indice di lookahead ----
    logs = [LogLine(
        'target',
        f'TGT | idx={idx} idx_target={idx_target}/{len(xc) - 1} '
        f'lookahead={lookahead:.2f} nominale=({p_target[0]:+.3f},{p_target[1]:+.3f})')]
    return p_target, logs


def deflect_target(p_target, x, obstacles, car_radius, avoidance_margin,
                   max_defl, half_width) -> Tuple[np.ndarray, List[LogLine]]:
    """
    Displace the target tangentially out of any obstacle's safety radius.

    Se il target lookahead cade dentro il margine di sicurezza di un
    ostacolo, spostalo tangenzialmente fuori: e' qui che avviene
    l'evitamento ostacoli / cambio di direzione locale, senza bisogno
    di un piano.

    UPGRADE:
     - il raggio di innesco ora usa la stessa distanza di sicurezza del
       solver (R_safe = r + d_safe) invece del solo raggio ostacolo,
       cosi' il target inizia a muoversi PRIMA che il costo del solver
       debba intervenire con forza, non dopo.
     - lo spostamento tangenziale ora e' proporzionale a quanto il
       target ha "sconfinato" nel margine di sicurezza (0 a R_safe,
       massimo sulla superficie dell'ostacolo) invece di un salto fisso
       di 1.0 m.

    THE CEILING ON THAT DISPLACEMENT IS max_defl, the node's
    obstacle_target_shift_m (a declared parameter read from
    stack_params.yaml). It used to be 0.6 * mean(halfWidth) --
    corridor-derived, and so self-limiting against leaving the corridor,
    but also unnamed, untunable without a rebuild, and silently retuned by
    any change to corr_wmin/corr_wmax. At the shipping geometry it evaluated
    to exactly 0.36 m; the parameter now ships 0.30, deliberately smaller so
    the car passes close to obstacles instead of swerving wide. The
    PROPORTIONAL SHAPE below is unchanged -- this is only its ceiling, still
    scaled by `penetration`.

    Because the value is no longer derived from the corridor, the "never
    leaves the corridor" property is no longer structural: it holds because
    0.30 < the 0.4333 m narrow half-width, not because the arithmetic forces
    it. The throttled warn below is what keeps that from going quiet if
    either number is ever retuned past the other.
    """
    logs = []
    mean_half_width = float(np.mean(half_width))
    if max_defl > mean_half_width:
        logs.append(LogLine(
            'deflection_ceiling',
            f'TGT/defl | obstacle_target_shift_m {max_defl:.3f} exceeds the '
            f'corridor mean half-width {mean_half_width:.3f}: a fully '
            f'penetrating obstacle deflects the target outside its own '
            f'corridor.'))

    for ox, oy, r in obstacles:
        p_obs = np.array([ox, oy], dtype=float)
        R_safe = r + car_radius + avoidance_margin

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
            logs.append(LogLine(
                'deflection',
                f'TGT/defl | ostacolo=({ox:+.3f},{oy:+.3f},r={r:.3f}) '
                f'd={d:.3f}<R_safe={R_safe:.3f} '
                f'pen={penetration:.2f} offset={offset:.3f} '
                f'({p_target_old[0]:+.3f},{p_target_old[1]:+.3f}) -> '
                f'({p_target[0]:+.3f},{p_target[1]:+.3f})'))
    return p_target, logs


class TargetFilter(NamedTuple):
    """The target's smoothing and deflection-coast state, carried between ticks."""

    smoothed: Optional[np.ndarray]
    last_deflection_vec: np.ndarray
    decay_remaining: int


def coast_and_smooth(p_target, p_target_raw, state: TargetFilter, decay_ticks,
                     alpha) -> Tuple[TargetFilter, List[LogLine]]:
    """
    Coast a vanished deflection down and exponentially smooth the target.

    p_target is this tick's (possibly deflected) point, p_target_raw the
    centreline point before deflection. Returns the new state; its .smoothed
    is the target to steer at.

    Obstacle-deflection coast: deflected_this_tick is True iff the
    deflection actually moved p_target away from the raw centerline point
    (i.e. some obstacle in THIS frame's list was inside R_safe of it). When
    the obstacle that was deflecting the target is no longer in this frame's
    list (missed detection, merged away, or genuinely cleared) the
    deflection coasts down to zero over decay_ticks ticks instead of
    snapping back to the raw centerline on the very first clear tick.
    """
    logs = []
    last_deflection_vec = state.last_deflection_vec
    decay_remaining = state.decay_remaining
    deflected_this_tick = not np.allclose(p_target, p_target_raw)

    if deflected_this_tick:
        last_deflection_vec = p_target - p_target_raw
        decay_remaining = decay_ticks
    elif decay_remaining > 0:
        # Decrement BEFORE computing the fraction so this first coast tick
        # already shows partial decay (not another full-strength tick) --
        # reaches exactly zero after decay_ticks ticks, not ticks+1.
        decay_remaining -= 1
        decay_fraction = decay_remaining / decay_ticks
        p_target = p_target_raw + decay_fraction * last_deflection_vec
        logs.append(LogLine(
            'coast',
            f'TGT/coast | no live deflection this tick, coasting '
            f'({decay_remaining}/{decay_ticks} '
            f'ticks left) -> ({p_target[0]:+.3f},{p_target[1]:+.3f})'))
    # else: no deflection this tick and none coasting -- p_target is
    # already the raw centerline point, nothing to do.

    # ---- Exponential smoothing on the final (possibly coasted) point ---
    if state.smoothed is None:
        smoothed = p_target.copy()
    else:
        smoothed = alpha * p_target + (1.0 - alpha) * state.smoothed

    return TargetFilter(smoothed, last_deflection_vec, decay_remaining), logs
