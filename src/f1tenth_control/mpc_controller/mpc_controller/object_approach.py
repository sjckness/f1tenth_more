"""How a corridor aimed at a tracked object turns toward it, per rebuild.

Pure: no rclpy, no numpy, no ROS types. Modelled on wall_turn.py, which
solves the neighbouring problem (how much of a commanded rotation one
corridor asks for) and whose helpers -- wrap_to_pi, min_turn_radius,
horizon_heading_reach -- are imported rather than re-derived here.

THE STATE IS AN ABSOLUTE HEADING, AND THAT IS THE DIFFERENCE FROM wall_turn
----------------------------------------------------------------------------
psi_c is the corridor's heading in the odom frame, carried across rebuilds by
the caller. Each rebuild rotates it a fraction of the way toward the bearing
to the target and returns the new value; the corridor is then built along it.

wall_turn.py:56-64 documents the opposite choice and the trap behind it. Its
ratchet holds `commanded_rot = turn_progress + dpsi_this` -- the rotation from
the move's start -- and explicitly NOT dpsi_this, because dpsi_this is
measured from the LIVE heading and therefore has to shrink as the car turns:
hold the increment and the end heading marches past the target.

That trap cannot arise here, because the quantity held is not an increment
measured from a moving reference. It is the reference. The error this module
drives to zero is

    e = wrap(bearing - psi_c)

and the second term is the held state itself, not the car's yaw. So:

  * A car that has not yet turned onto psi_c does not make e smaller by
    existing. Under a live-yaw error term it would, and the corridor would
    stop asking for the rotation it had already been granted.
  * A car that overshoots psi_c does not make e larger in the opposite
    direction and start fighting its own corridor.
  * With a stationary target and a converged psi_c, e is exactly zero and the
    corridor heading is bit-stable across rebuilds no matter what the car
    does. Under a live-yaw term it would jitter with every yaw wobble, at the
    rebuild rate, which is precisely the chatter wall_turn's ratchet exists
    to suppress -- suppressed here by construction instead.

Do not "simplify" e to wrap(bearing - car_yaw). That is the increment trap,
wearing different clothes.

WHAT EACH TERM IS FOR
---------------------
    r        = d - standoff, the range still to close. Negative inside it.
    bearing  = absolute heading from car to target.
    e        = wrap(bearing - psi_c), what the corridor still owes.
    k        = the DISTANCE SCHEDULE, 1 far away and 0 close in.
    dpsi_max = the KINEMATIC CAP, what one rebuild can physically deliver.

k and dpsi_max do different jobs and neither replaces the other.

k answers "how eagerly should the corridor chase the target right now". Far
out, a moving target or a jittering estimate should be tracked fully (k = 1):
there is distance in hand to absorb the turn, and refusing to track it just
accumulates error. Close in, chasing is actively harmful -- the same 5 cm of
estimate jitter subtends a bearing change that grows as 1/r, so at 0.4 m a
jitter that was negligible at 3 m swings the bearing by tens of degrees and
would whip the corridor across the target on every rebuild. k ramps linearly
to zero at r_freeze and the corridor goes rigid for the final approach, which
is the one phase where the car must simply fly what it was given.

dpsi_max answers "how much rotation can this rebuild actually be flown". It
is the smaller of two caps, both borrowed from wall_turn's reasoning:

    r / (c_safety * R_min)          -- the distance cap. Turning by dpsi at
                                       the tightest radius costs R_min * dpsi
                                       of path; c_safety is the margin on
                                       that. Asking for more than the
                                       remaining range can carry aims the
                                       corridor at a heading the car reaches
                                       only after passing the target.
    horizon_heading_reach(...)      -- the horizon cap, identical in kind and
                                       in code to wall_turn's own: the MPC
                                       cannot fly more rotation than
                                       N * ts * v_ref of path is worth, and a
                                       terminal heading beyond that is answered
                                       by sitting on the steering bound.

c_safety's FLOOR IS 1.0 HERE, NOT wall_turn's 1.5. wall_turn needs 1.5
because its corridor bends along a smoothstep whose peak slope is 1.5, so the
tightest point of the blend is 1.5x the average. This corridor is STRAIGHT --
build_object_centreline lays a straight centreline along psi_c -- so the car
flies a plain arc onto it and the honest factor is 1.0. 1.5 is still the
shipped value, as margin, but a value between 1.0 and 1.5 is a tuning choice
rather than a geometry error and is not rejected.

FLAGS: ONE ADVISORY, ONE TERMINAL, BOTH PER TICK
-----------------------------------------------
Two geometric conditions are reported, and they mean different things to the
caller. Both are evaluated EVERY CONTROL TICK by assess_object_approach,
against the live pose and the held psi_c -- not at corridor rebuilds. A flag
sampled once per rebuild cannot bound its own latency: in the closed-loop rig
a target that walks past the car was astern for six ticks that fell entirely
between two rebuilds and was never reported, and another rebuild caught a
two-tick crossing and reported it twenty ticks early.

`target_behind` (TERMINAL once persistent): |wrap(bearing - car_yaw)| > pi/2.
The target is in the rear half-plane. This vehicle has no reverse primitive
and the corridor is a forward object, so no corridor built along any heading
reaches it. A car circling a person who walks past it grazes 90 degrees and
turns back, so the raw condition alone is not an outcome:
TargetBehindPersistence makes it terminal only once it has held continuously
for object_behind_persist_sec.

`inside_turn_radius` (ADVISORY only, never ends a move): the goal lies inside
one of the two circles the car would trace at full lock, so no forward path
of curvature at most 1/R_min reaches it from the current heading. Derivation,
because the obvious test (r < R_min) is wrong in both directions:

    Put the car at the origin heading +x. Its two tightest circles have
    centres at (0, +R) and (0, -R), R = R_min. A goal at range r and bearing
    alpha (from the car's heading) sits at (r cos a, r sin a). Its squared
    distance to the left centre is

        r^2 cos^2 a + (r sin a - R)^2  =  r^2 - 2 r R sin a + R^2

    which is inside that circle when that is < R^2, i.e. when

        r^2 < 2 r R sin a    <=>    r < 2 R sin a        (r > 0)

    and symmetrically r < -2 R sin a for the right circle. Together:

        UNREACHABLE  <=>  r < 2 * R_min * |sin(alpha)|

    So the threshold is not R_min. It is ZERO straight ahead -- a target dead
    ahead is reachable at any range, which r < R_min would wrongly reject --
    and 2 * R_min abeam, twice what r < R_min would allow through.

It is advisory because the approach does not need to hit the goal POINT:
the goal jitters with the estimate, and arriving within the reach tolerance is
arriving. Two changes make it a signal rather than noise.

  1. It is evaluated only while r > r_freeze. Inside r_freeze the corridor is
     frozen and the car flies what it was given; there is nothing left to
     re-plan, and the 1/r bearing sensitivity that motivates r_freeze makes
     the test fire on estimate jitter alone (15-37 tick episodes in the
     jitter run before this rule). This also subsumes the old r <= 0 special
     case, since r_freeze >= 0 is required.

  2. It carries an explicit HEADING MARGIN delta:

        UNREACHABLE  <=>  r_goal < 2 * R_min * sin(max(|alpha_goal| - delta, 0))

     delta is the goal-bearing uncertainty the test cannot resolve. The goal
     point sits `standoff` back from the target estimate along psi_c, so it
     inherits the estimate's lateral error sigma_p unchanged, and at goal
     range r_goal that is a bearing error of atan(sigma_p / r_goal). The test
     runs only for r > r_freeze, so the largest such error it can see is at
     r_freeze. Taking two sigma, so that a single noisy sample does not flag:

        delta = atan(2 * sigma_p / r_freeze)
              = atan(2 * 0.05 / 0.4) = 0.2450 rad = 14.0 deg

     sigma_p = 0.05 m is detection_3d_node's position_sigma_base_m, the
     detector's own floor, and the figure object_r_freeze_m's description
     already quotes. The margin is derived from r_freeze at run time
     (heading_margin_for), so retuning r_freeze moves it with it. In the rig
     the one spurious tick outside r_freeze exceeded the unmargined threshold
     by 4.2 deg; the 70-degrees-off case, which really is inside the circle,
     exceeds it by 30 deg or more.

The circle whose radius applies is chosen by which side the GOAL is on
(alpha_goal's sign), since the two steering bounds differ. The test runs on
the goal point (target minus standoff along the current psi_c), not on the
target: the goal is where the car is actually being sent.

REFERENCE FRAME. Everything here is one planar frame and the caller picks it;
mpc_corr uses odom, because that is where the corridor, the solver and the
vehicle state already live. Angles are radians, CCW positive, and every angle
that crosses a boundary has been through wrap_to_pi.
"""

import math
from typing import NamedTuple, Tuple

from mpc_controller.wall_turn import (
    horizon_heading_reach,
    min_turn_radius,
    wrap_to_pi,
)

__all__ = [
    'DEFAULT_HEADING_MARGIN_RAD',
    'ObjectApproachFlags',
    'ObjectHeadingStep',
    'TARGET_POSITION_SIGMA_M',
    'TargetBehindPersistence',
    'approach_lookahead',
    'assess_object_approach',
    'build_object_centreline',
    'goal_point',
    'heading_margin_for',
    'min_standoff_clear_of',
    'object_speed_ref',
    'plan_object_heading',
]

# detection_3d_node's position_sigma_base_m: the detector's floor 1-sigma
# position error. See heading_margin_for.
TARGET_POSITION_SIGMA_M = 0.05
HEADING_MARGIN_SIGMAS = 2.0

# Below this range the bearing to the target is not a meaningful quantity and
# the 1/r sensitivity is unbounded. Same role as wall_turn's own guards.
_COINCIDENT_EPS = 1e-9


def heading_margin_for(r_freeze: float,
                       sigma_p: float = TARGET_POSITION_SIGMA_M,
                       n_sigma: float = HEADING_MARGIN_SIGMAS) -> float:
    """Heading margin [rad] for the inside_turn_radius test.

    atan(n_sigma * sigma_p / r_freeze): the largest goal-bearing error target
    estimate noise can cause at a range the test still runs at. See the
    module docstring for the derivation.
    """
    if not r_freeze > 0.0:
        raise ValueError(
            f'r_freeze {r_freeze} must be > 0: the heading margin is the '
            'bearing error at r_freeze, which is unbounded at zero range')
    return math.atan(float(n_sigma) * float(sigma_p) / float(r_freeze))


DEFAULT_HEADING_MARGIN_RAD = heading_margin_for(0.4)


class ObjectHeadingStep(NamedTuple):
    """One rebuild's decision. The caller carries psi_c_new forward."""

    # The new held corridor heading [rad, absolute]. Carry this to the next
    # rebuild; it is the module's only state.
    psi_c_new: float
    # Range still to close: distance to target minus standoff. Negative when
    # the vehicle is already inside the standoff.
    r: float
    # Absolute heading from the vehicle to the target [rad].
    bearing: float
    # wrap(bearing - psi_c): what the held heading still owes. Measured
    # against psi_c, never against live yaw -- see the module docstring.
    e: float
    # Distance schedule in [0, 1]: the fraction of e this rebuild asked for.
    k: float
    # The kinematic cap actually in force this rebuild [rad].
    dpsi_max: float


class ObjectApproachFlags(NamedTuple):
    """One control tick's geometry and flags, from the live pose."""

    # Range still to close: distance to target minus standoff.
    r: float
    # Absolute heading from the vehicle to the target [rad].
    bearing: float
    # wrap(bearing - psi_c), against the HELD heading.
    e: float
    # wrap(bearing - car_yaw): where the target is relative to the car.
    alpha: float
    # Raw, this tick: the target is in the rear half-plane. Terminal only
    # through TargetBehindPersistence.
    target_behind: bool
    # ADVISORY: the goal is inside a full-lock circle by more than the
    # heading margin. Always False at r <= r_freeze.
    inside_turn_radius: bool


def goal_point(target_xy: Tuple[float, float], psi_c: float,
               standoff: float) -> Tuple[float, float]:
    """Return the point `standoff` metres short of the target, along `psi_c`.

    Back along the CORRIDOR heading, not along the line from the car. The
    corridor is a straight line through the target at heading psi_c, and the
    goal has to sit on it or the terminal cost pulls off the centreline the
    rest of the geometry is built around. While psi_c is still converging the
    two differ; once it has converged they coincide.
    """
    tx, ty = target_xy
    return (tx - standoff * math.cos(psi_c), ty - standoff * math.sin(psi_c))


def object_speed_ref(r: float, v_move: float, a_dec: float) -> float:
    """Speed command at range `r`, reaching zero exactly at r = 0.

    The braking parabola v = sqrt(2 a d), capped at the move's own speed: the
    fastest approach that can still stop in the distance left at a_dec. It is
    a REFERENCE, not a stop -- the solver still owns the actual command, and
    this mode deliberately has no hard tolerance stop of the kind pose mode
    uses (see MPC_corr's goal_object branch).

    Zero at and inside r = 0, rather than negative or clamped to some floor:
    r = 0 is the standoff, which is where the vehicle is meant to come to
    rest. A floor here would make it creep into the target forever.

    a_dec <= 0 is treated as "no ramp" and returns v_move, so a mis-set
    parameter fails visibly as an unramped approach rather than as a vehicle
    that never moves.
    """
    v_move = float(v_move)
    if a_dec <= 0.0:
        return v_move
    if r <= 0.0:
        return 0.0
    return min(v_move, math.sqrt(2.0 * float(a_dec) * float(r)))


def _reachable_within_min_radius(r_goal: float, alpha_goal: float,
                                 r_min: float, heading_margin: float) -> bool:
    """Report whether a forward arc at R_min can still reach the goal.

    Unreachable when r_goal < 2 * R_min * sin(max(|alpha_goal| - margin, 0))
    -- see the module docstring for the derivation and the margin. A goal at
    the car (r_goal ~ 0) is not judged here.
    """
    if r_goal <= _COINCIDENT_EPS:
        return True
    alpha_eff = max(abs(alpha_goal) - float(heading_margin), 0.0)
    return r_goal >= 2.0 * r_min * math.sin(min(alpha_eff, math.pi / 2.0))


def plan_object_heading(psi_c: float,
                        target_xy: Tuple[float, float],
                        car_xy: Tuple[float, float],
                        car_yaw: float,
                        standoff: float,
                        *,
                        r_full: float,
                        r_freeze: float,
                        c_safety: float,
                        wheelbase: float,
                        delta_min: float,
                        delta_max: float,
                        n_steps: int,
                        ts: float,
                        v_ref: float) -> ObjectHeadingStep:
    """Rotate the held corridor heading toward the target, once.

    psi_c is the previous rebuild's psi_c_new (or, on a move's first rebuild,
    the bearing from the car to the target -- the caller seeds it, so that a
    move starts pointed at its target rather than sweeping onto it from
    whatever the last move left behind).

    Returns a step whose psi_c_new the caller carries forward. Every other
    field is diagnostic. Flags are not judged here: they belong to every
    tick, not to rebuilds -- see assess_object_approach.
    """
    if not r_full > r_freeze:
        raise ValueError(
            f'r_full {r_full} must exceed r_freeze {r_freeze}: the schedule '
            'divides by their difference')
    if not c_safety >= 1.0:
        raise ValueError(
            f'c_safety {c_safety} is below 1.0: turning by dpsi costs at '
            'least R_min * dpsi of path, so a factor under 1 asks for a '
            'rotation the remaining range cannot carry')
    if not standoff > 0.0:
        raise ValueError(
            f'standoff {standoff} must be > 0: a zero standoff aims the '
            'corridor at the target itself')

    cx, cy = float(car_xy[0]), float(car_xy[1])
    tx, ty = float(target_xy[0]), float(target_xy[1])
    psi_c = wrap_to_pi(float(psi_c))

    dx, dy = tx - cx, ty - cy
    d = math.hypot(dx, dy)
    # A target on top of the car has no bearing. Hold the heading rather than
    # inventing one from atan2(0, 0); r is deeply negative there anyway, so
    # k is 0 and the corridor would not have moved.
    bearing = math.atan2(dy, dx) if d > _COINCIDENT_EPS else psi_c
    r = d - float(standoff)

    e = wrap_to_pi(bearing - psi_c)

    # Which steering bound applies is decided by which way the corridor still
    # has to rotate -- the same rule as wall_turn's owed_sign, and for the
    # same reason: the two bounds are not equal (-0.283 / +0.278), so a left
    # turn and a right turn have different R_min.
    bound = delta_max if e >= 0.0 else delta_min
    r_min = min_turn_radius(wheelbase, bound)

    # THE SCHEDULE. Linear in r between the two radii, flat outside them.
    k = (r - float(r_freeze)) / (float(r_full) - float(r_freeze))
    k = min(1.0, max(0.0, k))

    # THE CAP. max(r, 0) because r goes negative inside the standoff, and a
    # negative cap would invert the clamp below into "rotate at least this
    # much" -- the corridor would snap to the bearing exactly when k has
    # already decided it must not move.
    dpsi_by_dist = max(r, 0.0) / (float(c_safety) * r_min)
    dpsi_by_horizon = horizon_heading_reach(n_steps, ts, v_ref, r_min)
    dpsi_max = min(dpsi_by_dist, dpsi_by_horizon)

    requested = k * e
    dpsi = min(dpsi_max, max(-dpsi_max, requested))
    psi_c_new = wrap_to_pi(psi_c + dpsi)

    return ObjectHeadingStep(
        psi_c_new=psi_c_new,
        r=r,
        bearing=bearing,
        e=e,
        k=k,
        dpsi_max=dpsi_max,
    )


def assess_object_approach(psi_c: float,
                           target_xy: Tuple[float, float],
                           car_xy: Tuple[float, float],
                           car_yaw: float,
                           standoff: float,
                           *,
                           r_freeze: float,
                           heading_margin: float,
                           wheelbase: float,
                           delta_min: float,
                           delta_max: float) -> ObjectApproachFlags:
    """Evaluate this tick's approach geometry and flags against the live pose.

    Called every control tick with the HELD psi_c. Nothing here is state: the
    terminal form of target_behind needs time, and that lives in
    TargetBehindPersistence. See the module docstring for both flags.
    """
    if not r_freeze >= 0.0:
        raise ValueError(
            f'r_freeze {r_freeze} must be >= 0: inside_turn_radius is skipped '
            'at r <= r_freeze, and a negative value would evaluate it past the '
            'standoff, where it fires on the final tick of every approach')

    cx, cy = float(car_xy[0]), float(car_xy[1])
    tx, ty = float(target_xy[0]), float(target_xy[1])
    psi_c = wrap_to_pi(float(psi_c))
    d = math.hypot(tx - cx, ty - cy)
    bearing = math.atan2(ty - cy, tx - cx) if d > _COINCIDENT_EPS else psi_c
    r = d - float(standoff)
    e = wrap_to_pi(bearing - psi_c)
    alpha = wrap_to_pi(bearing - float(car_yaw))

    target_behind = abs(alpha) > math.pi / 2.0

    inside_turn_radius = False
    # Behind first: a target astern also fails the circle test, and naming it
    # inside_turn_radius would point at steering when the problem is that the
    # thing is behind the car.
    if not target_behind and r > float(r_freeze):
        gx, gy = goal_point((tx, ty), psi_c, standoff)
        r_goal = math.hypot(gx - cx, gy - cy)
        alpha_goal = wrap_to_pi(math.atan2(gy - cy, gx - cx) - float(car_yaw))
        bound = delta_max if alpha_goal >= 0.0 else delta_min
        r_min = min_turn_radius(wheelbase, bound)
        inside_turn_radius = not _reachable_within_min_radius(
            r_goal, alpha_goal, r_min, heading_margin)

    return ObjectApproachFlags(
        r=r, bearing=bearing, e=e, alpha=alpha,
        target_behind=target_behind, inside_turn_radius=inside_turn_radius)


class TargetBehindPersistence:
    """Make target_behind terminal only once it has held for persist_sec.

    Not a latch: it reports the current persisted state, and a target that
    comes back in front clears it. The CONSUMER latches the outcome (the
    mission ends the move on the first terminal report). reset() between
    moves. Time comes from the caller, so a node's clock (including sim time)
    is the one used.
    """

    def __init__(self, persist_sec: float):
        """Hold time [s] before target_behind is terminal; 0 is immediate."""
        if not persist_sec >= 0.0:
            raise ValueError(f'persist_sec {persist_sec} must be >= 0')
        self.persist_sec = float(persist_sec)
        self._since = None

    def reset(self):
        """Forget any running hold, for a new move."""
        self._since = None

    def update(self, target_behind: bool, now_sec: float) -> Tuple[bool, float]:
        """Return (terminal, seconds target_behind has held continuously)."""
        if not target_behind:
            self._since = None
            return False, 0.0
        if self._since is None:
            self._since = float(now_sec)
        held = float(now_sec) - self._since
        # 1e-9: tick times built as n * ts do not subtract to exact multiples.
        return held >= self.persist_sec - 1e-9, held


def approach_lookahead(r: float, corridor_length: float,
                       lookahead_frac: float, reach_floor: float,
                       arrival_band: float) -> Tuple[float, bool]:
    """Lookahead for an object corridor, and whether it was clamped to the end.

    Separate from MPC_corr._corridor_lookahead, and deliberately NOT sharing
    its warnings. That function warns when the lookahead exceeds the corridor
    length, because on a straight or a distance-mode corridor that means the
    target pins to the far end and the geometry has been mis-configured.

    Here it means the vehicle has arrived. An object corridor ENDS at the
    goal, so a lookahead that reaches past the end is the correct and expected
    state for the last `reach_floor` metres of every approach -- pref_nom
    clamps to the goal and the terminal cost becomes "arrive at", which is
    exactly what is wanted. Emitting the mis-configuration warning there would
    fire it once per rebuild on every successful approach, and a warning that
    fires on success is one nobody reads.

    Returns (lookahead, at_end). `at_end` is the honest signal the suppressed
    warning used to carry: the caller can log it once per approach, or put it
    on the status topic, instead of throttled prose.
    """
    from_length = float(lookahead_frac) * float(corridor_length)
    lookahead = max(from_length, float(reach_floor))
    at_end = lookahead >= float(corridor_length) or r <= float(arrival_band)
    return lookahead, at_end


def build_object_centreline(car_xy: Tuple[float, float],
                            target_xy: Tuple[float, float],
                            psi_c: float,
                            standoff: float,
                            n_points: int,
                            behind: float = 0.5
                            ) -> Tuple[Tuple[float, float], float, float]:
    """Origin, heading and length of the object corridor's straight centreline.

    The centreline is the line through the TARGET at heading psi_c. It starts
    `behind` metres back from the car's own projection onto that line and ends
    at the goal.

    WHY IT IS PINNED TO THE TARGET AND NOT TO THE CAR. Every other corridor in
    this stack passes through the car's current position (see
    build_straight_corridor's goal_distance branch, which says so explicitly).
    That is right for a move defined by a direction and a distance, and wrong
    for one defined by a place: a corridor translated onto the car has no
    lateral error to correct, so nothing pulls the car back onto the line to
    the object. Pinning the line to the target gives the w_corr/half-width
    machinery a real cross-track error to act on, which is what makes the
    approach converge in position rather than only in heading.

    WHY IT STARTS BEHIND THE CAR. compute_local_target picks the nearest
    centreline sample and advances a lookahead from there. If the line started
    at the car's projection exactly, that nearest sample would be index 0 on
    every rebuild and any backward excursion would fall off the end. Half a
    metre of lead-in costs nothing and keeps the projection interior.

    NO LENGTH FLOOR. The goal_pose branch clips its corridor length into
    [1.0, corr_L_base], so a goal 0.3 m away still produces a 1.0 m corridor
    whose far end is PAST the goal, and the terminal cost then pulls the car
    through the target. Here the corridor ends at the goal, full stop: a short
    corridor is the correct description of a nearly-finished approach.

    Returns ((x0, y0), psi_c, length). `n_points` is accepted so the caller's
    sample count travels with the geometry; the sampling itself belongs to the
    caller, which already has numpy.
    """
    gx, gy = goal_point(target_xy, psi_c, standoff)
    cos_c, sin_c = math.cos(psi_c), math.sin(psi_c)
    # Car's signed position along the line, measured from the goal.
    s_car = (car_xy[0] - gx) * cos_c + (car_xy[1] - gy) * sin_c
    s_start = min(s_car - float(behind), -float(behind))
    origin = (gx + s_start * cos_c, gy + s_start * sin_c)
    length = -s_start
    return origin, psi_c, max(length, _COINCIDENT_EPS)


def min_standoff_clear_of(target_radius: float, car_radius: float,
                          avoidance_margin: float,
                          clearance: float = 0.0) -> float:
    """Smallest standoff that keeps the goal outside the target's own R_safe.

    compute_local_target deflects its lookahead target tangentially whenever
    that target falls within R_safe = obstacle_radius + car_radius +
    avoidance_margin of an obstacle. The tracked object IS an obstacle -- it is
    in the same detection stream that feeds the obstacle list, and it is not
    excluded -- so a goal placed inside its R_safe is pushed sideways by the
    avoidance machinery, and the car arrives beside the thing it was sent to.

    Returns R_safe plus `clearance`. See MPC_corr's goal_object branch for the
    shipped numbers.
    """
    return (float(target_radius) + float(car_radius)
            + float(avoidance_margin) + float(clearance))
