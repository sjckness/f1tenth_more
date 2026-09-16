"""The go_to_object mission state machine, with no ROS in it.

Split out of :mod:`mission_node` so it can be tested without an rclpy
runtime: mission-level failures live in state machines, and the first run of
those tests found one (arrival had to dominate reachability, or the vehicle
repositioned away from an object it had already reached).

The node owns messages, TF, parameters and the clock. This owns the
transitions and the command, and it is the only place the curvature limiter
is applied.

States
------
``SEARCH``      nothing tracked yet.
``ACQUIRE``     a track exists but has not earned the approach.
``APPROACH``    driving the pursuit arc.
``REPOSITION``  the target is inside the minimum turning circle; backing off.
``ARRIVED``     within ``d_stop``. Latched.
``LOST``        the estimate went away. Drive gate dropped, steering ramped out.

Every command leaves through the limiter, including the zero commanded in
the halt states. Stepping the steering to zero is as impossible for the servo
as stepping it to anything else, so ``LOST`` *ramps* the wheels out rather
than demanding an instantaneous return to centre; the drive gate drops on the
same cycle, so the vehicle is already being brought to a stop while the
steering unwinds. The one exception is the watchdog, below.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import copysign

from .pursuit_geometry import (
    CurvatureLimiter,
    PursuitParams,
    PursuitSolution,
    solve_pursuit,
)

__all__ = ['MissionState', 'MissionParams', 'MissionCommand', 'GoToObjectMission']


class MissionState(Enum):
    SEARCH = 'SEARCH'
    ACQUIRE = 'ACQUIRE'
    APPROACH = 'APPROACH'
    REPOSITION = 'REPOSITION'
    ARRIVED = 'ARRIVED'
    LOST = 'LOST'


@dataclass(frozen=True)
class MissionParams:
    confidence_threshold: float = 0.5
    """Track confidence required, *in addition to* ``converged``, to approach."""

    reposition_timeout: float = 2.0
    """Give up repositioning after this long [s].

    Turning away resolves every unreachable start pose measured, but the
    margin depends on ``max_kappa_rate``, because the opposite-lock demand is
    ramped like every other command rather than stepped. Worst case over the
    seven measured poses at r_min = 1.5 m, v = 2 m/s::

        max_kappa_rate      worst recovery      margin to 2 s
             0.05                1.20 s              1.7x
             0.50 (default)      0.93 s              2.2x
             2.00                0.71 s              2.8x
            21.00                0.63 s              3.2x

    The unlimited-slew figure is 0.61 s; the rest is ramp. If you lower
    ``max_kappa_rate`` much below the default, raise this with it -- the two
    are coupled, and a REPOSITION that times out aborts to LOST.

    The timeout exists so an unmodelled geometry aborts instead of orbiting
    forever, not to catch these.
    """

    odom_timeout: float = 0.2
    """Watchdog: no odometry for this long [s] is a hard stop."""

    state_timeout: float = 1.0
    """Watchdog: no usable track for this long [s] is a hard stop.

    Longer than ``max_age``, deliberately: a track ageing out is an ordinary
    LOST, which ramps the steering out and drops the drive gate. This is the
    escalation for when that condition persists.
    """

    def __post_init__(self) -> None:
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be in [0, 1]')
        if not self.reposition_timeout > 0.0:
            raise ValueError('reposition_timeout must be > 0')
        if not self.odom_timeout > 0.0:
            raise ValueError('odom_timeout must be > 0')
        if not self.state_timeout > 0.0:
            raise ValueError('state_timeout must be > 0')


@dataclass(frozen=True)
class MissionCommand:
    curvature: float
    """The command to publish: rate limited."""

    drive_enable: bool
    state: MissionState
    solution: PursuitSolution | None = None

    curvature_raw: float = 0.0
    """What pursuit asked for, before the limiter. Diagnostics."""

    limiter_saturated: bool = False
    watchdog: bool = False


class GoToObjectMission:
    """Pure state machine. ``update`` is the whole interface."""

    def __init__(
        self,
        pursuit: PursuitParams | None = None,
        mission: MissionParams | None = None,
        limiter: CurvatureLimiter | None = None,
    ) -> None:
        self.pursuit = pursuit if pursuit is not None else PursuitParams()
        self.mission = mission if mission is not None else MissionParams()
        self.limiter = limiter if limiter is not None else CurvatureLimiter()
        self._state = MissionState.SEARCH
        self._last_now: float | None = None
        self._reposition_since: float = 0.0
        self._track_missing_since: float | None = None
        self._measured_kappa: float | None = None
        self.transitions: list[tuple[float, MissionState, MissionState]] = []

    @property
    def state(self) -> MissionState:
        return self._state

    def reset(self) -> None:
        """Return to SEARCH. Does not clear the limiter's physical sync."""
        self._state = MissionState.SEARCH
        self._last_now = None
        self._reposition_since = 0.0
        self._track_missing_since = None
        self.limiter.resync(self._measured_kappa)

    def update(self, now: float, vehicle_xy, vehicle_psi: float, track,
               *, measured_kappa: float | None = None,
               last_odom_stamp: float | None = None) -> MissionCommand:
        """Advance one control tick.

        ``track`` is a ``TrackedObject`` or ``None``; only ``position``,
        ``converged`` and ``confidence`` are read, so a stub suffices in
        tests. ``measured_kappa`` is the curvature the wheels are actually
        at, when the vehicle reports its steering angle.
        """
        now = float(now)
        dt = 0.0 if self._last_now is None else max(now - self._last_now, 0.0)
        self._last_now = now
        self._measured_kappa = measured_kappa

        if track is None:
            if self._track_missing_since is None:
                self._track_missing_since = now
        else:
            self._track_missing_since = None

        stale = self._watchdog_reason(now, last_odom_stamp)
        if stale is not None:
            return self._hard_stop(stale, now)

        if self._state is MissionState.ARRIVED:
            return self._halt(MissionState.ARRIVED, dt)

        if track is None:
            if self._state in (MissionState.ACQUIRE, MissionState.APPROACH,
                               MissionState.REPOSITION):
                self._go(MissionState.LOST, now)
            return self._halt(self._state, dt)

        if self._state in (MissionState.SEARCH, MissionState.LOST):
            self._go(MissionState.ACQUIRE, now)

        if self._state is MissionState.ACQUIRE:
            ready = (track.converged
                     and track.confidence >= self.mission.confidence_threshold)
            if not ready:
                return self._halt(MissionState.ACQUIRE, dt)
            self._go(MissionState.APPROACH, now)

        solution = solve_pursuit(
            vehicle_xy, vehicle_psi,
            (float(track.position[0]), float(track.position[1])), self.pursuit)

        # Arrival dominates reachability, and must be checked in REPOSITION
        # too. A target within d_stop but inside the minimum turning circle
        # is *reached*, not unreachable: checking arrival only in APPROACH
        # let the vehicle pass within 0.2 m of the object and then reposition
        # away from it, APPROACH <-> REPOSITION, forever.
        if solution.arrived:
            self._go(MissionState.ARRIVED, now)
            return self._halt(MissionState.ARRIVED, dt)

        if self._state is MissionState.APPROACH and not solution.reachable:
            self._go(MissionState.REPOSITION, now)

        if self._state is MissionState.REPOSITION:
            if solution.reachable:
                self._go(MissionState.APPROACH, now)
            elif now - self._reposition_since > self.mission.reposition_timeout:
                self._go(MissionState.LOST, now)
                return self._halt(MissionState.LOST, dt)

        if self._state is MissionState.REPOSITION:
            # Away from the object, not toward it. Turning toward an
            # unreachable target is the livelock: the vehicle orbits it at
            # r_min and the geometry never resolves.
            raw = copysign(1.0 / self.pursuit.r_min, -solution.alpha)
        else:
            raw = solution.curvature

        limited = self.limiter.apply(raw, dt)
        return MissionCommand(limited, True, self._state, solution,
                              curvature_raw=raw,
                              limiter_saturated=self.limiter.saturated)

    # -- internals --------------------------------------------------------

    def _watchdog_reason(self, now: float, last_odom_stamp) -> str | None:
        if (last_odom_stamp is not None
                and now - float(last_odom_stamp) > self.mission.odom_timeout):
            return f'no odometry for {now - float(last_odom_stamp):.2f} s'
        if (self._track_missing_since is not None
                and now - self._track_missing_since > self.mission.state_timeout):
            return f'no track for {now - self._track_missing_since:.2f} s'
        return None

    def _hard_stop(self, reason: str, now: float) -> MissionCommand:
        """Failsafe: zero immediately, not a ramp.

        The ramp in :meth:`_halt` assumes a node that is still ticking. The
        watchdog fires precisely when that assumption is in doubt, and a
        stalled node must not leave the last command latched -- that is a car
        driving in a circle with nobody home. So this publishes hard zero and
        drags the limiter's synchronisation point to zero with it, rather
        than emitting a ramp whose next step may never arrive.
        """
        self.watchdog_reason = reason
        if self._state is not MissionState.LOST:
            self._go(MissionState.LOST, now)
        self.limiter.seed(0.0)
        return MissionCommand(0.0, False, MissionState.LOST, None,
                              curvature_raw=0.0, watchdog=True)

    def _halt(self, state: MissionState, dt: float) -> MissionCommand:
        # Drive gate down immediately; steering ramped out under the same
        # rate limit as everything else, because the servo cannot step to
        # centre any more than it can step anywhere else.
        limited = self.limiter.apply(0.0, dt)
        return MissionCommand(limited, False, state, None, curvature_raw=0.0,
                              limiter_saturated=self.limiter.saturated)

    def _go(self, new_state: MissionState, now: float) -> None:
        if new_state is self._state:
            return
        self.transitions.append((now, self._state, new_state))
        if new_state is MissionState.REPOSITION:
            self._reposition_since = now
        if new_state in (MissionState.LOST, MissionState.ACQUIRE):
            # Resynchronise to physical state at exactly the moments a step
            # would otherwise be handed to the servo.
            self.limiter.resync(self._measured_kappa)
        self._state = new_state
