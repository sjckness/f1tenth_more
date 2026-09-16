"""Pure-pursuit geometry: vehicle pose + object point -> curvature command.

Stateless. No ROS, no numpy. See the package docstring for frame conventions.

The object is treated as a *point*. No arrival orientation is required and no
surface normal is estimated, tracked or consumed anywhere.

Ambiguities resolved here, stated rather than picked silently:

* **Object exactly astern** (``alpha == +/-pi``) has no well-defined turn
  direction. ``copysign`` breaks the tie toward the sign of ``alpha``, so a
  wrapped ``+pi`` steers left. Any deterministic choice is acceptable; the
  point is that it does not flicker between frames.
* **``reachable``** is specifically the single-forward-arc criterion. An
  object astern reports ``reachable=True`` whenever ``sin(alpha)`` is small
  even though no one forward arc reaches it; ``behind`` is the flag that
  matters in that case, and the caller is expected to read both.
* **``arrived``** is reported independently of the other flags. A curvature
  is still returned when ``arrived`` is set; stopping is the caller's job,
  not this module's.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, copysign, hypot, pi, sin

__all__ = ['wrap_pi', 'PursuitParams', 'PursuitSolution', 'solve_pursuit',
           'CurvatureLimiter', 'curvature_from_steering']

# Below this separation the bearing to the object is meaningless (and the
# 1/d in the curvature formula is a division by zero).
_COINCIDENT_EPS = 1e-9


def wrap_pi(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``.

    The single angle-wrapping helper for the whole package: raw angles are
    never compared or interpolated anywhere else.
    """
    wrapped = (angle + pi) % (2.0 * pi) - pi
    # The modulo above yields [-pi, pi); the interval we advertise is
    # (-pi, pi], so the open end is folded over.
    if wrapped <= -pi:
        wrapped += 2.0 * pi
    return wrapped


@dataclass(frozen=True)
class PursuitParams:
    """Tuning for :func:`solve_pursuit`."""

    r_min: float = 1.5
    """Minimum turning radius [m]. Curvature is clamped to ``+/-1/r_min``."""

    d_lookahead_min: float = 1.0
    """Floor on the distance used in the curvature formula [m]."""

    d_stop: float = 0.5
    """Separation at or below which ``arrived`` is reported [m]."""

    straight_eps: float = 1e-3
    """Bearing deadband [rad] below which curvature is exactly ``0.0``."""

    def __post_init__(self) -> None:
        if not self.r_min > 0.0:
            raise ValueError(f'r_min must be > 0, got {self.r_min}')
        if not self.d_lookahead_min > 0.0:
            raise ValueError(
                f'd_lookahead_min must be > 0, got {self.d_lookahead_min}')
        if self.d_stop < 0.0:
            raise ValueError(f'd_stop must be >= 0, got {self.d_stop}')
        if self.straight_eps < 0.0:
            raise ValueError(
                f'straight_eps must be >= 0, got {self.straight_eps}')


@dataclass(frozen=True)
class PursuitSolution:
    """Result of one pursuit solve."""

    curvature: float
    """Commanded curvature [1/m]; positive turns left."""

    radius: float | None
    """Signed arc radius [m], or ``None`` when the command is straight."""

    alpha: float
    """Bearing to the object in the vehicle frame [rad], positive to the left."""

    psi_end: float
    """Heading at the far end of the arc [rad]. Derived from the *raw*
    ``alpha``, so it stays the true corridor end-heading even while the
    commanded curvature saturates."""

    distance: float
    """Planar vehicle-to-object distance [m]. The true distance, not floored."""

    straight: bool
    reachable: bool
    behind: bool
    arrived: bool


def solve_pursuit(
    vehicle_xy: tuple[float, float],
    vehicle_psi: float,
    object_xy: tuple[float, float],
    params: PursuitParams | None = None,
) -> PursuitSolution:
    """Solve the unique forward arc from the vehicle pose through the object.

    With ``alpha`` the bearing to the object in the vehicle frame and ``d``
    the planar distance, the circular arc that starts tangent to the current
    heading and passes through the object has curvature::

        kappa = 2 * sin(alpha) / d

    and leaves the vehicle at the far end heading ``psi + 2 * alpha``.

    That one expression already delivers the three behaviours the mission
    needs, which is why none of them is special-cased below: ``alpha == 0``
    falls out as ``kappa == 0`` (straight ahead, no branch); ``kappa`` scales
    as ``1/d``, so the turn tightens as the object nears; and ``kappa`` is
    continuous in both ``alpha`` and ``d``, so the steering command has no
    steps as the object crosses the centreline.
    """
    par = params if params is not None else PursuitParams()

    vx, vy = vehicle_xy
    ox, oy = object_xy
    dx = ox - vx
    dy = oy - vy
    distance = hypot(dx, dy)

    if distance < _COINCIDENT_EPS:
        # Coincident pose: atan2(0, 0) is defined but meaningless, and the
        # formula would divide by zero. Sitting on the target is arrival.
        return PursuitSolution(
            curvature=0.0,
            radius=None,
            alpha=0.0,
            psi_end=wrap_pi(vehicle_psi),
            distance=distance,
            straight=True,
            reachable=True,
            behind=False,
            arrived=True,
        )

    alpha = wrap_pi(atan2(dy, dx) - vehicle_psi)

    # From the raw alpha, deliberately: psi_end describes the corridor, and
    # the corridor does not change just because the steering is saturated.
    psi_end = wrap_pi(vehicle_psi + 2.0 * alpha)

    arrived = distance <= par.d_stop
    behind = abs(alpha) > pi / 2.0

    # A target inside either minimum-turning circle cannot be reached by a
    # single forward arc. Evaluated on the true distance, not the floored
    # one -- it is a statement about the target, not about the command.
    reachable = distance >= 2.0 * par.r_min * abs(sin(alpha))

    kappa_max = 1.0 / par.r_min

    if behind:
        # 2*sin(alpha)/d decays back toward zero as alpha -> pi, so an object
        # directly astern would command almost no steering and the vehicle
        # would drive straight past it. Turn as hard as the vehicle can,
        # toward the side the object is on, and let the geometry come back
        # into the forward sector.
        curvature = copysign(kappa_max, alpha)
        straight = False
    elif abs(alpha) < par.straight_eps:
        # Exactly zero, not a residue that changes sign on sensor noise.
        curvature = 0.0
        straight = True
    else:
        # Flooring d bounds the command as the vehicle arrives: kappa goes as
        # 1/d, so without this it diverges at the target. The trade is real
        # -- terminal tracking accuracy is given up for a bounded, followable
        # command inside d_lookahead_min.
        d_effective = max(distance, par.d_lookahead_min)
        curvature = 2.0 * sin(alpha) / d_effective
        if curvature > kappa_max:
            curvature = kappa_max
        elif curvature < -kappa_max:
            curvature = -kappa_max
        straight = False

    radius = None if straight else 1.0 / curvature

    return PursuitSolution(
        curvature=curvature,
        radius=radius,
        alpha=alpha,
        psi_end=psi_end,
        distance=distance,
        straight=straight,
        reachable=reachable,
        behind=behind,
        arrived=arrived,
    )


def curvature_from_steering(steering_angle: float, wheelbase: float) -> float:
    """Ackermann steering angle [rad] -> path curvature [1/m]."""
    if not wheelbase > 0.0:
        raise ValueError(f'wheelbase must be > 0, got {wheelbase}')
    from math import tan
    return tan(steering_angle) / wheelbase


class CurvatureLimiter:
    """Slew-rate limit on the commanded curvature itself.

    Stateful, and deliberately kept out of :func:`solve_pursuit`, which stays
    a pure function of pose and target.

    **Why the limit belongs on curvature and not on the position estimate.**
    The requirement bounds the per-cycle *curvature* step. An earlier design
    rate-limited the object position estimate instead, but the transfer from
    a lateral position correction to a curvature change is
    ``2 * cos(alpha) / d**2`` -- distance dependent -- so a constant position
    rate cannot satisfy a distance-independent curvature bound. It fails at
    small enough ``d`` for *any* constant: at dt = 0.01 s, a 1.5 m/s position
    limit yields 0.0075 of curvature step at d = 2 m and 0.030 at d = 1 m.

    **There is no "unseeded" state, and that is the point.** The limiter is
    always synchronised to a curvature, because the steered wheels are always
    at *some* angle. An earlier version cleared to unseeded on mission
    changes and then passed the next command through unlimited -- which
    bypassed the limiter at precisely the moments a step occurs, handing the
    servo a step command on every re-acquisition. Hence:

    * :meth:`seed` -- cold start, before anything has been commanded. The
      constructor does this with ``initial_kappa=0.0``, which **assumes the
      vehicle starts with its wheels centred**. If that is not true of your
      platform, seed it explicitly at startup.
    * :meth:`resync` -- mission changes (LOST, re-acquisition). Synchronises
      to the measured steering angle when the vehicle publishes one, and
      otherwise keeps the last commanded value. It never clears.

    **Choosing max_kappa_rate from the actuator.** For an Ackermann vehicle
    of wheelbase ``L`` the steering angle is ``delta = atan(L * kappa)``, so
    ``ddelta/dt = L / (1 + (L*kappa)**2) * dkappa/dt``, largest at
    ``kappa = 0`` where it is simply ``L * dkappa/dt``.

    The default here is *not* actuator-derived, and the provenance matters:
    0.5 (1/m)/s x a 10 ms cycle gives exactly 0.005, the old cruise-smoothness
    bound. For a typical F1TENTH platform (``L`` ~ 0.33 m) that is about
    0.165 rad/s = 9.5 deg/s of steering, against roughly 400 deg/s for a hobby
    servo, i.e. ``dkappa/dt`` of about 21 (1/m)/s. The default is some 40x
    below the actuator limit: a smoothness choice, not a physical one.
    """

    def __init__(self, max_kappa_rate: float = 0.5,
                 initial_kappa: float = 0.0) -> None:
        if not max_kappa_rate > 0.0:
            raise ValueError(
                f'max_kappa_rate must be > 0, got {max_kappa_rate}')
        self.max_kappa_rate = float(max_kappa_rate)
        self._previous = float(initial_kappa)
        self.saturated = False
        """Whether the most recent :meth:`apply` hit the rate limit."""
        self.saturations = 0
        self.applications = 0

    @property
    def previous(self) -> float:
        """The last curvature this limiter emitted. Never ``None``."""
        return self._previous

    def seed(self, kappa: float) -> None:
        """Set the synchronisation point outright. Cold start only."""
        self._previous = float(kappa)

    def resync(self, measured_kappa: float | None = None) -> float:
        """Resynchronise to physical state at a mission change.

        With a measured steering angle this snaps to where the wheels *are*;
        without one it keeps the last commanded value, which is the best
        available estimate of the same thing. Either way the next
        :meth:`apply` is rate limited away from a real starting point rather
        than passed through from nowhere.
        """
        if measured_kappa is not None:
            self._previous = float(measured_kappa)
        return self._previous

    def apply(self, kappa: float, dt: float) -> float:
        """Rate-limit ``kappa`` against the previous command.

        ``dt <= 0`` **holds** the previous value rather than passing the
        demand through. Zero elapsed time permits zero slew: the wheels
        cannot have moved, so the command must not either. An earlier version
        passed through here, which meant the very first control tick after
        startup -- where ``dt`` is 0 because there is no previous tick to
        measure from -- handed the servo the full demanded curvature as a
        step. It also meant a stopped or backward clock disabled the limiter
        entirely, which is the worst possible moment to lose it; holding
        instead degrades to a constant command, which the watchdog catches.
        """
        kappa = float(kappa)
        self.applications += 1

        if dt <= 0.0:
            self.saturated = kappa != self._previous
            if self.saturated:
                self.saturations += 1
            return self._previous

        max_step = self.max_kappa_rate * float(dt)
        delta = kappa - self._previous
        if delta > max_step:
            limited = self._previous + max_step
            self.saturated = True
        elif delta < -max_step:
            limited = self._previous - max_step
            self.saturated = True
        else:
            limited = kappa
            self.saturated = False

        if self.saturated:
            self.saturations += 1
        self._previous = limited
        return limited
