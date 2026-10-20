"""The deceleration ramp: how a move with a known stopping point slows into it.

Pure: no rclpy, no numpy, no ROS types. MPC_corr.control_loop builds one
StopRampProfile from the parameters and calls decide() every tick; the answer
is a CAP ON vdes, the solver's speed reference along the whole horizon. It is
not a drive lane, not a separate controller, and it never publishes anything:
the solver still owns the command, the /drive clamp still owns the limit, and
the move's own stop_condition still owns when the move ends.

WHAT IT REPLACES. Every stopping point in this stack used to be a THRESHOLD
crossed at cruise speed: the mission's front_clearance guard fires at
distance < target, mpc_corr's distance mode at traveled >= goal_distance, its
pose mode at dist_to_goal <= pose_goal_tolerance. In all three the command
steps from the move's speed to zero in one tick and the car then covers its
whole stopping distance PAST the point it was asked to stop at. With the
measured braking plant (below) that is 0.111 m at 0.4 m/s, 0.55 m at 1.0 m/s
and 0.97 m at 1.5 m/s -- a mission that says "stop 2 m from the wall" stopped
at 1.03 m from it at 1.5 m/s, and the error GREW with the approach speed,
which is the opposite of what a tolerance should do.

THE PROFILE

    d_pred = d_est - v * tau_latency          what the distance will be when
                                              this tick's command acts
    d_rem  = d_pred - target                  distance still to give away
    v_ref  = min(v_cruise, sqrt(2 a_dec (d_rem - d_final)))
    v_ref  = max(v_ref, v_min)  while moving  -- the operating floor
    v_ref  = 0                  once d_rem <= d_final

sqrt(2 a d) is the speed from which a constant deceleration a comes to rest
in exactly d. Capping the reference by it means the car is never travelling
faster than it can still shed over the distance it has left, so the ramp is
feasible at every point of the approach by construction rather than by tuning.
Monotone non-decreasing in d_rem, so a distance estimate that walks inward
cannot ask for more speed than the previous one did.

THE FLOOR IS WHY THERE IS A d_final AT ALL. The car is not operated below
0.4 m/s (min_moving_speed_mps): zero is used only to stop, and the VESC tracks
a slower command badly (vesc_ackermann's velocity_correction.hpp: 0.15 m/s
commanded measured 0.083). sqrt(2 a_dec d) passes under the floor for the last
v_min^2 / (2 a_dec) metres -- 0.133 m at a_dec 0.6 -- which is precisely the
region the floor forbids. object_approach.py's ObjectStopLatch documents the
same wall and answers it the same way: drive the floor to a computed trigger,
then command zero and let the car coast the rest. So the ramp hands over to a
zero command at

    d_final = v_min * tau_latency + v_min^2 / (2 a_stop)

the distance the car covers after the command goes to zero AT THE FLOOR, and
comes to rest at the target. d_final is COMPUTED from the braking plant, not
hand-tuned: at the shipped values (v_min 0.4, tau_latency 0.2, a_stop 1.5) it
is 0.133 m, and object_stop_distance_m -- measured independently from six
archived stops, tools/measure_stop_distance.py -- ships 0.14 m for the same
quantity. The two agree to a centimetre, which is the only cross-check this
module gets without a car.

a_stop IS NOT a_dec, AND THEY ARE NOT INTERCHANGEABLE. a_dec is a REQUEST:
how hard the ramp chooses to slow down, and it is the one free tuning knob
here. a_stop is a MEASUREMENT: how hard the car actually stops when the
command goes to zero, fitted as d = v tau + v^2 / (2 a) over the archived
stops at tau 0.144 s, a 1.50 m/s^2. Setting a_dec anywhere near a_stop asks
the ramp to brake as hard as a dead stop and there is nothing gentle left in
it; the shipped a_dec 0.6 is 2.5x gentler, which is the whole point of the
exercise.

ONE tau_latency FOR TWO DELAYS, DELIBERATELY CONSERVATIVE. The tau in d_pred
is SENSING delay: how stale d_est is by the time a command issued now reaches
the wheels (the wall estimate's own age, its publish interval, and the control
period -- about 0.19 s measured on the 10-16 bag: /perception/virtual_wall at
17 Hz with a 28.6 ms median publish latency, consumed by a 10 Hz control
loop). The tau in d_final is ACTUATION delay: the 0.144 s dead time in the
braking fit. They are different quantities that happen to be the same size,
and the parameter is single because every way of being wrong about it is then
wrong in the same direction: a tau that is too large predicts the car closer
than it is and hands over to the zero command early, so the car stops SHORT of
the target -- further from the wall. At the shipped 0.2 s against the measured
0.144 s that bias is about 0.02 m. Splitting them is a refinement for the day
a run measures them separately, and the module docstring will say so.

WHAT THE RAMP DOES NOT FIX, and cannot:

  * A BIASED DISTANCE ESTIMATE. The ramp lands the car where d_est says the
    target is. If the wall estimate reads 0.1 m long, the car stops 0.1 m
    from where the mission meant, as abruptly or as smoothly as you like.
    /perception/virtual_wall measured 0.996 / 1.990 m against a bumper at
    1.00 / 2.00 m on the 10-14 calibration clips, so that bias is about
    -1 cm today, and it is the floor on this feature's accuracy.
  * ODOMETRY SCALE. Distance mode's d_rem is odometry-derived, and the wheels
    read 1.2-1.45x short while braking (object_stop_distance_m's own note).
    The ramp inherits that error; it does not add to it.
  * A MISSING ESTIMATE. No reading, a stale one or "clear ahead" is not a
    distance and the ramp refuses to act on any of them (decide() returns
    RAMP_UNKNOWN and no cap). What a dead wall publisher should do to a move
    is the mission guard's business, and check_stop_condition.py already holds
    the car for it -- it is explicitly NOT this module's job to invent a
    distance, least of all zero.
  * STEERING. This is a longitudinal profile. A corner taken too fast is the
    solver's problem and the corridor's, not the ramp's.
"""

import math
from typing import NamedTuple, Optional

__all__ = [
    'RAMP_CRUISE', 'RAMP_FLOOR', 'RAMP_RAMP', 'RAMP_STOP', 'RAMP_UNKNOWN',
    'StopRampDecision', 'StopRampLatch', 'StopRampProfile',
    'final_stop_distance', 'predicted_distance',
]


# The five ways the ramp can be driving this tick; MPC_corr logs them and the
# rigs count them. There is no 'disabled' state: a disabled ramp is not
# consulted at all, so it has no answer to report.
RAMP_CRUISE = 'cruise'      # far enough out that v_cruise is the binding cap
RAMP_RAMP = 'ramp'          # on the braking parabola, above the floor
RAMP_FLOOR = 'floor'        # the parabola has passed under v_min; hold v_min
RAMP_STOP = 'stop'          # d_rem <= d_final: zero, and coast to the target
RAMP_UNKNOWN = 'unknown'    # no usable distance to the stopping point: no cap


def final_stop_distance(v_min: float, tau_latency_s: float,
                        a_stop: float) -> float:
    """The distance [m] the car covers after the command goes to zero at v_min.

    v_min * tau + v_min^2 / (2 a_stop): the latency dead time at speed, then a
    constant deceleration to rest. The hand-over point of the ramp, and the
    module docstring derives why it has to exist.
    """
    return float(v_min) * float(tau_latency_s) + float(v_min) ** 2 / (2.0 * float(a_stop))


def predicted_distance(d_est: float, v: float, tau_latency_s: float) -> float:
    """`d_est` advanced by the distance the car covers during the latency.

    Never clamped at zero: a negative prediction means the stopping point is
    already behind the bumper, which decide() must see as "stop", not as
    "zero distance left, keep ramping".
    """
    return float(d_est) - float(v) * float(tau_latency_s)


class StopRampDecision(NamedTuple):
    """How the ramp drives this tick."""

    # RAMP_CRUISE, RAMP_RAMP, RAMP_FLOOR, RAMP_STOP or RAMP_UNKNOWN.
    mode: str
    # The speed reference [m/s] to pass the solver as vdes. Never above
    # v_cruise -- this is a cap, and raising a speed the mission chose would
    # be a different feature with a different review.
    v_ref: float
    # The remaining distance [m] the decision was made on, or None for
    # RAMP_UNKNOWN. Logged, and the rigs assert on it.
    d_rem: Optional[float]

    @property
    def stopping(self) -> bool:
        """Whether this decision is the hand-over to the zero command."""
        return self.mode == RAMP_STOP


class StopRampProfile:
    """The ramp's tuning, validated once, with d_final derived from it.

    Immutable and cheap to hold: MPC_corr builds one at startup and calls
    decide() at the control rate. Deliberately NOT a free function with six
    arguments -- d_final is a derived constant of the tuning, and computing it
    per tick inside the caller is how the two halves of the same formula drift
    apart.
    """

    def __init__(self, a_dec: float, a_stop: float, tau_latency_s: float,
                 v_min: float):
        """Raise ValueError unless every value is a usable physical quantity.

        Raising rather than falling back to a default: these four come from
        stack_params.yaml and a nonsensical one is a configuration error that
        must surface at node start, not a reason to brake on invented numbers.
        """
        if not (math.isfinite(a_dec) and a_dec > 0.0):
            raise ValueError(
                f'a_dec must be finite and > 0 m/s^2, got {a_dec!r}: a ramp that '
                'does not decelerate is not a ramp')
        if not (math.isfinite(a_stop) and a_stop > 0.0):
            raise ValueError(
                f'a_stop must be finite and > 0 m/s^2, got {a_stop!r}: it is the '
                'measured braking of the car, and a zero makes d_final infinite')
        if not (math.isfinite(tau_latency_s) and tau_latency_s >= 0.0):
            raise ValueError(
                f'tau_latency_s must be finite and >= 0 s, got {tau_latency_s!r}')
        if not (math.isfinite(v_min) and v_min > 0.0):
            raise ValueError(
                f'v_min must be finite and > 0 m/s, got {v_min!r}: the operating '
                'floor is what the ramp hands over to the zero command at')
        self.a_dec = float(a_dec)
        self.a_stop = float(a_stop)
        self.tau_latency_s = float(tau_latency_s)
        self.v_min = float(v_min)
        self.d_final = final_stop_distance(self.v_min, self.tau_latency_s, self.a_stop)

    def __repr__(self):
        return (f'StopRampProfile(a_dec={self.a_dec:g}, a_stop={self.a_stop:g}, '
                f'tau_latency_s={self.tau_latency_s:g}, v_min={self.v_min:g}, '
                f'd_final={self.d_final:.4f})')

    def predict(self, d_est: float, v: float) -> float:
        """predicted_distance with this profile's tau. See that function."""
        return predicted_distance(d_est, v, self.tau_latency_s)

    def ramp_speed(self, d_rem: float) -> float:
        """sqrt(2 a_dec (d_rem - d_final)), floored at 0 inside d_final.

        The raw parabola, with no cruise cap and no operating floor applied --
        decide() is what a caller wants. Exposed because the rigs and the unit
        tests check the parabola itself, and because it reads better in a log
        line than the expression does.
        """
        s = float(d_rem) - self.d_final
        if not (s > 0.0):
            return 0.0
        return math.sqrt(2.0 * self.a_dec * s)

    def decide(self, d_rem: Optional[float], v_cruise: float) -> StopRampDecision:
        """The cap on vdes for a stopping point `d_rem` metres away.

        d_rem is the distance still to give away -- already target-relative and
        already latency-corrected (predict()). None or non-finite means there
        is no usable distance to the stopping point, which is RAMP_UNKNOWN and
        NO cap: the caller drives exactly as it would without the ramp. That is
        the one safe answer, because the alternatives are both wrong -- a cap
        of zero stops the car on a dead sensor, and a cap built on the last
        good reading drives on a number nothing is measuring any more.

        A v_cruise at or below zero is passed straight through (the caller is
        waiting or stopping for its own reasons, and the ramp has no business
        turning that into motion). A v_cruise below v_min is NOT raised to the
        floor either: whoever asked for it wanted to go slower, and faster is
        the wrong way to be wrong -- object_speed_decision's BELOW_FLOOR note
        makes the same call for the same reason.
        """
        if d_rem is None or not math.isfinite(d_rem):
            return StopRampDecision(RAMP_UNKNOWN, float(v_cruise), None)

        d_rem = float(d_rem)
        v_cruise = float(v_cruise)
        if d_rem <= self.d_final:
            return StopRampDecision(RAMP_STOP, 0.0, d_rem)
        if not v_cruise > 0.0:
            return StopRampDecision(RAMP_CRUISE, v_cruise, d_rem)

        v_ramp = self.ramp_speed(d_rem)
        if v_ramp >= v_cruise:
            return StopRampDecision(RAMP_CRUISE, v_cruise, d_rem)
        if v_ramp >= self.v_min:
            return StopRampDecision(RAMP_RAMP, min(v_ramp, v_cruise), d_rem)
        # Under the floor but outside d_final: hold the floor, capped by
        # v_cruise so a mission asking for less than the floor still gets less.
        return StopRampDecision(RAMP_FLOOR, min(self.v_min, v_cruise), d_rem)


class StopRampLatch:
    """Zero on arrival: trip once d_rem reaches d_final, then hold for the move.

    LATCHED, not re-evaluated, and for the same reason ObjectStopLatch is: once
    the command is zero the car is still braking, and d_rem keeps moving --
    estimate jitter, the wall contract flicking to CLEAR, the car's own last
    centimetres. Re-arming on d_rem > d_final would restart a car that is
    already inside its own stopping distance, at the very speed the floor sets.
    A new move ends the latch (reset()); nothing else does.

    A RAMP_UNKNOWN tick does not trip it and does not clear it. Both halves
    matter: an absent distance must not be read as arrival, and a latch that
    has already tripped must not be undone by the sensor going quiet during the
    very stop it commanded.
    """

    def __init__(self, profile: StopRampProfile):
        self.profile = profile
        self.latched = False
        # d_rem on the tick that tripped it, for the log line and the rigs.
        self.latched_d_rem: Optional[float] = None

    def update(self, decision: StopRampDecision) -> bool:
        """Fold this tick's decision in; return whether the stop is latched."""
        if not self.latched and decision.stopping:
            self.latched = True
            self.latched_d_rem = decision.d_rem
        return self.latched

    def reset(self) -> None:
        """Clear the latch: a new move starts driving."""
        self.latched = False
        self.latched_d_rem = None
