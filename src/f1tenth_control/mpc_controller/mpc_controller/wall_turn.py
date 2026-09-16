"""
How much of a wall_turn ONE corridor rebuild asks for.

THE DEFECT THIS REPLACES. build_straight_corridor used to hand every
wall_turn corridor the WHOLE remaining turn: psiEnd = psi_base + the full
commanded angle, rebuilt about once a second with the same target. The MPC
horizon covers N * ts * v_ref = 20 * 0.1 * 0.5 = 1.0 m, and a 74.5 degree turn
at full lock needs 1.36 m of arc, so the terminal heading cost asked for
something no control sequence inside the horizon can reach.

THE RULE, per rebuild (all angles signed, radians):

    R_min           = wheelbase / tan(|steering bound for this direction|)
    d_avail         = max(dFront - safety_margin, 0)
    dpsi_rem        = turn_total - turn_progress
    dpsi_by_dist    = d_avail / (k_safety * R_min)
    dpsi_by_horizon = n_steps * ts * v_ref / R_min
    dpsi_this       = sign(dpsi_rem) * min(|dpsi_rem|, dpsi_by_dist, dpsi_by_horizon)

once the turn has COMMITTED, and 0 before. psiEnd = psi_current + dpsi_this.

THE COMMIT GATE. The min() alone does not keep the corridor straight far from
the wall: there dpsi_by_dist is the LARGEST term, so the min picks dpsi_rem and
the corridor would bend by the whole turn at once. So the turn commits when
the distance can no longer carry what is left,

    d_avail <= k_safety * R_min * |dpsi_rem|    (i.e. dpsi_by_dist <= |dpsi_rem|)

and stays committed for the rest of the move. Without the latch the gate
re-closes as soon as the first increment lands (the remainder is then small
next to the distance left), which splits one turn into two with a straight
in between.

WHY k_safety >= 1.5. Smoothstep's peak slope is 1.5, so a turn of dpsi spread
over a distance d peaks at a curvature of 1.5 * dpsi / d. Holding that at or
under 1 / R_min needs d >= 1.5 * R_min * dpsi; k_safety = 1.5 is that limit
with no margin at all.

WHICH STEERING BOUND. The two are not equal (delta_min -0.283, delta_max
+0.278), so R_min is 1.069 m turning left and 1.049 m turning right. The
direction is the sign of what is still owed, so a car that overshoots and has
to come back uses the other side's bound.

dpsi_rem IS NOT wrap_to_pi(psi_target - psi_current). Below 180 degrees the
two are the same number. At 180 they are not: wrap_to_pi(pi) is +pi or -pi
depending on rounding, which is the exact bug 32587b0 fixed for
drive_turn_180.json. turn_progress is MPC_corr's unwrapped per-tick yaw sum,
so turn_total - turn_progress has no seam at +-pi and never forgets which way
round the mission asked for.

CHATTER. dFront is a live camera estimate and it jitters. Two things keep
that out of the corridor:

  * THE RATCHET. Once committed, the corridor's END HEADING -- as rotation
    from the move's start, commanded_rot = turn_progress + dpsi_this -- never
    retreats toward the start heading within the move. A dip in dFront
    therefore cannot bend a corridor back toward the wall. It is not
    dpsi_this that is held: dpsi_this is measured from the LIVE heading and
    has to shrink as the car turns, and holding it would push psiEnd past the
    target.
  * THE JUMP BOUND is the horizon cap, and nothing overrides it:
    |dpsi_this| <= dpsi_by_horizon on every rebuild. No rebuild can put the
    end heading further ahead of the car than the horizon can reach, so
    between two rebuilds the end heading moves by at most dpsi_by_horizon
    plus whatever the car itself rotated in between.

The one case where the end heading does retreat is the horizon clip winning
over the ratchet, which needs the car to rotate AWAY from the turn.

WHAT THE DISTANCE FEASIBILITY DOES AND DOES NOT GUARANTEE. Every rebuild that
is not held obeys 1.5 * |dpsi_this| / d_avail <= 1 / R_min. A held rebuild can
exceed it: the car fell behind an end heading that was feasible when it was
issued, or dFront dipped. Holding is deliberate -- shrinking the ask near a
wall points the corridor back at the wall. WallTurnStep.held flags it.

UNKNOWN dFront. None, NaN, or a negative value (front_clearance_node publishes
-1.0 for "no reading yet" and for "something inside the ZED's minimum range")
is not a distance, and it is not a wall at zero either.

  * Committed: the rebuild drops the distance cap and the turn carries on
    within the horizon's reach.
  * Not committed, on a later rebuild of the move: nothing changes -- the
    corridor stays straight and the turn does not commit. A missing reading
    is no evidence the wall got closer, and committing on one would start the
    turn early, into whatever runs alongside the car.
  * Not committed, on the move's FIRST rebuild: the turn commits. There is no
    earlier reading it could have been waiting on, and a wall_turn issued
    with the camera path down would otherwise drive on and never turn.

WHAT THIS DOES NOT FIX. dFront is a straight-ahead measurement along the
camera's optical axis, not along the arc. Once the car is bending, that ray
meets the wall further away than the car's own path will, so d_avail is
optimistic mid-turn.
"""

import math
from typing import NamedTuple, Optional

# Peak slope of smoothstep 3t^2 - 2t^3 (at t = 0.5). See the module docstring.
SMOOTHSTEP_PEAK_SLOPE = 1.5


def wrap_to_pi(angle: float) -> float:
    """Map `angle` into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def min_turn_radius(wheelbase: float, steering_bound: float) -> float:
    """Tightest single-track turn radius at a steering bound: L / tan(|delta|)."""
    return float(wheelbase) / math.tan(abs(float(steering_bound)))


def horizon_heading_reach(n_steps: int, ts: float, v_ref: float,
                          r_min: float) -> float:
    """Heading change the MPC horizon can actually fly, in radians.

    n_steps * ts * v_ref is how far the horizon reaches along the path; at the
    tightest radius that arc is worth arc/R_min of rotation. Asking a corridor
    for more than this puts the terminal heading somewhere no control sequence
    inside the horizon can reach, and the QP answers by sitting on the steering
    bound -- the defect this module's docstring opens with.

    Named and shared rather than inlined, because object_approach.py needs the
    identical quantity for the identical reason, and two copies of a cap this
    load-bearing would drift.

    Clamped at zero: a v_ref of zero (or a nonsense negative one) reaches
    nothing, and a negative reach would invert every min() it feeds.
    """
    return max(float(n_steps) * float(ts) * float(v_ref), 0.0) / float(r_min)


def wall_turn_trigger_distance(turn: float, wheelbase: float, steering_bound: float,
                               k_safety: float, safety_margin: float) -> float:
    """Return the dFront at which a still-straight car commits to `turn` radians."""
    r_min = min_turn_radius(wheelbase, steering_bound)
    return float(safety_margin) + float(k_safety) * r_min * abs(float(turn))


class WallTurnStep(NamedTuple):
    """One rebuild's decision. The caller carries committed/commanded_rot forward."""

    # Signed rotation this corridor asks for, measured from the LIVE heading.
    dpsi_this: float
    # turn_progress + dpsi_this: the end heading as rotation from move start.
    commanded_rot: float
    # Signed rotation the move still owes (turn_total - turn_progress).
    dpsi_rem: float
    # max(dFront - margin, 0), or None when dFront was unknown.
    d_avail: Optional[float]
    # Tightest radius for the direction dpsi_rem turns.
    r_min: float
    # Distance cap; math.inf when dFront was unknown.
    dpsi_by_dist: float
    # Horizon cap from this rebuild's n_steps * ts * v_ref.
    dpsi_by_horizon: float
    # Latched commit state for the next rebuild of the same move.
    committed: bool
    # The ratchet stopped the end heading retreating on this rebuild.
    held: bool


def plan_wall_turn_step(turn_total: float, turn_progress: float,
                        d_front: Optional[float], *,
                        wheelbase: float, delta_min: float, delta_max: float,
                        k_safety: float, safety_margin: float,
                        n_steps: int, ts: float, v_ref: float,
                        committed: bool = False,
                        prev_commanded_rot: Optional[float] = None) -> WallTurnStep:
    """
    Decide how much of the turn this corridor rebuild asks for.

    turn_total and turn_progress are signed and unwrapped (MPC_corr's
    turn_sign * turn_mag and turn_progress_rad). committed and
    prev_commanded_rot come from the previous rebuild of the SAME move:
    pass False/None on a move's first rebuild. See the module docstring for
    the rule, the gate and the ratchet.
    """
    if k_safety < SMOOTHSTEP_PEAK_SLOPE:
        raise ValueError(
            f'k_safety {k_safety} is below the smoothstep peak slope '
            f'{SMOOTHSTEP_PEAK_SLOPE}: the turn could not be tracked at R_min')

    turn_total = float(turn_total)
    turn_progress = float(turn_progress)
    dpsi_rem = turn_total - turn_progress

    # Which way the car still has to rotate picks the steering bound. At
    # exactly zero nothing is owed and the choice only affects the report.
    owed_sign = dpsi_rem if dpsi_rem != 0.0 else turn_total
    bound = delta_max if owed_sign >= 0.0 else delta_min
    r_min = min_turn_radius(wheelbase, bound)

    dpsi_by_horizon = horizon_heading_reach(n_steps, ts, v_ref, r_min)

    if d_front is not None and math.isfinite(d_front) and d_front >= 0.0:
        d_avail = max(float(d_front) - float(safety_margin), 0.0)
        dpsi_by_dist = d_avail / (k_safety * r_min)
        committed = committed or d_avail <= k_safety * r_min * abs(dpsi_rem)
    else:
        # Unknown distance: see the module docstring's three cases.
        d_avail = None
        dpsi_by_dist = math.inf
        committed = committed or prev_commanded_rot is None

    if committed:
        dpsi_this = math.copysign(
            min(abs(dpsi_rem), dpsi_by_dist, dpsi_by_horizon), owed_sign)
    else:
        dpsi_this = 0.0

    # Compared in the commanded turn's own sense, so "retreat" means the same
    # thing for a left and a right turn. Only reassigned when the ratchet or
    # the clip actually changes it, so the last rebuild's dpsi_this is dpsi_rem
    # bit-for-bit rather than (a + b) - a.
    held = False
    if committed and prev_commanded_rot is not None:
        sense = 1.0 if turn_total >= 0.0 else -1.0
        # A previous end heading past the target (after an overshoot) must not
        # pin the corridor beyond it.
        floor_s = min(sense * float(prev_commanded_rot), abs(turn_total))
        if sense * (turn_progress + dpsi_this) < floor_s:
            dpsi_this = sense * floor_s - turn_progress
            held = True
    if abs(dpsi_this) > dpsi_by_horizon:
        dpsi_this = math.copysign(dpsi_by_horizon, dpsi_this)

    return WallTurnStep(
        dpsi_this=dpsi_this,
        commanded_rot=turn_progress + dpsi_this,
        dpsi_rem=dpsi_rem,
        d_avail=d_avail,
        r_min=r_min,
        dpsi_by_dist=dpsi_by_dist,
        dpsi_by_horizon=dpsi_by_horizon,
        committed=committed,
        held=held,
    )
