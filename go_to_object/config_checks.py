"""Startup validation: refuse an unsafe configuration rather than clamp it.

Pure, so it can be exercised without a ROS runtime, and so the node can call
it before it publishes anything.

The distinction between an error and a warning here is whether this node can
*know* the configuration is wrong. A curvature rate above what the steering
servo can physically slew is wrong on its own terms and refuses to start. A
blind-travel budget that looks generous is only wrong relative to an
operating speed this node does not control -- it publishes curvature, not
speed -- so it is reported with the arithmetic and left to the operator.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import degrees

from .mission_state import MissionParams
from .object_tracker import TrackerParams
from .pursuit_geometry import PursuitParams

__all__ = ['VehicleLimits', 'Finding', 'ConfigError', 'validate',
           'raise_on_errors', 'describe']

ERROR = 'error'
WARNING = 'warning'


class ConfigError(RuntimeError):
    """Raised at startup for a configuration that must not be driven."""


@dataclass(frozen=True)
class VehicleLimits:
    """What the physical platform can actually do.

    Defaults are an F1TENTH-class car. These are *not* tuning: they describe
    hardware, and getting them wrong disables the checks that depend on them.
    """

    wheelbase: float = 0.33
    """[m]"""

    steering_slew_rate: float = 6.98
    """Steering servo slew capability [rad/s]. 6.98 rad/s ~ 400 deg/s."""

    min_turn_radius: float = 1.0
    """The tightest circle the vehicle can physically drive [m]."""

    cruise_speed: float = 2.0
    """Speed this mission is expected to run at [m/s].

    Advisory only: this node does not command speed. It is here so the
    speed-dependent checks have a number to work from instead of silently
    assuming one.
    """

    def max_kappa_rate(self) -> float:
        """Curvature slew the actuator can deliver, worst case.

        ``delta = atan(L * kappa)`` gives ``ddelta/dt = L * dkappa/dt`` at
        ``kappa = 0``, which is where the mapping is steepest, so this is the
        binding bound across the whole curvature range.
        """
        return self.steering_slew_rate / self.wheelbase


@dataclass(frozen=True)
class Finding:
    severity: str
    check: str
    message: str

    def __str__(self) -> str:
        return f'[{self.severity}] {self.check}: {self.message}'


def validate(pursuit: PursuitParams, tracker: TrackerParams,
             mission: MissionParams, max_kappa_rate: float,
             limits: VehicleLimits | None = None) -> list[Finding]:
    """Return every problem found. Empty means the configuration is sane."""
    limits = limits if limits is not None else VehicleLimits()
    found: list[Finding] = []

    actuator = limits.max_kappa_rate()
    if max_kappa_rate > actuator:
        found.append(Finding(
            ERROR, 'max_kappa_rate',
            f'{max_kappa_rate:.3f} (1/m)/s exceeds what the steering can slew: '
            f'{limits.steering_slew_rate:.2f} rad/s '
            f'({degrees(limits.steering_slew_rate):.0f} deg/s) over a '
            f'{limits.wheelbase:.3f} m wheelbase allows {actuator:.3f} (1/m)/s. '
            'The limiter would be a no-op and the servo would define the real '
            'slew rate, unmeasured.'))

    if pursuit.r_min < limits.min_turn_radius:
        found.append(Finding(
            ERROR, 'r_min',
            f'{pursuit.r_min:.2f} m is tighter than the vehicle can turn '
            f'({limits.min_turn_radius:.2f} m). Commanded curvature would '
            'saturate against the steering stops, and every arc the planner '
            'believes in would be unfollowable.'))

    if mission.state_timeout <= tracker.max_age:
        found.append(Finding(
            ERROR, 'state_timeout',
            f'{mission.state_timeout:.2f} s is not longer than max_age '
            f'({tracker.max_age:.2f} s), so the watchdog would fire on every '
            'ordinary track timeout and pre-empt the LOST ramp.'))

    blind = tracker.max_age * limits.cruise_speed
    if blind > pursuit.d_stop:
        found.append(Finding(
            WARNING, 'max_age',
            f'{tracker.max_age:.2f} s at {limits.cruise_speed:.1f} m/s is '
            f'{blind:.2f} m of travel on odometry alone, more than d_stop '
            f'({pursuit.d_stop:.2f} m): the vehicle can coast past the object '
            f'while blind. To bound blind travel by d_stop, max_age must be '
            f'below {pursuit.d_stop / limits.cruise_speed:.2f} s at this '
            'speed. This node does not command speed, so it cannot decide '
            'this for you.'))

    if pursuit.d_stop < pursuit.d_lookahead_min:
        found.append(Finding(
            WARNING, 'd_stop',
            f'd_stop ({pursuit.d_stop:.2f} m) is inside d_lookahead_min '
            f'({pursuit.d_lookahead_min:.2f} m), so between those radii the '
            'curvature command is computed against the floored distance and '
            'under-steers. That is the documented lookahead trade, and it is '
            'what REPOSITION exists to recover from -- but it does mean the '
            'final approach is open-loop in curvature authority.'))

    if tracker.nis_alarm_ratio * 2.0 <= 2.0 + 3.0 * (2.0 / max(tracker.nis_window, 1) ** 0.5):
        found.append(Finding(
            WARNING, 'nis_alarm_ratio',
            f'the alarm at {tracker.nis_alarm_ratio * 2.0:.2f} is within 3 '
            f'sampling sigma of the expectation for a {tracker.nis_window}-sample '
            f'window (sigma = {2.0 / tracker.nis_window ** 0.5:.2f}); it will '
            'fire on noise.'))

    return found


def raise_on_errors(findings: list[Finding]) -> None:
    errors = [f for f in findings if f.severity == ERROR]
    if errors:
        raise ConfigError(
            'refusing to run on an unsafe configuration:\n  '
            + '\n  '.join(str(f) for f in errors))


def describe(pursuit: PursuitParams, tracker: TrackerParams,
             mission: MissionParams, max_kappa_rate: float,
             limits: VehicleLimits) -> list[str]:
    """Every loaded parameter, one per line, for the startup log."""
    lines = [f'max_kappa_rate={max_kappa_rate}']
    for label, obj in (('pursuit', pursuit), ('tracker', tracker),
                       ('mission', mission), ('vehicle', limits)):
        for field in obj.__dataclass_fields__:
            lines.append(f'{label}.{field}={getattr(obj, field)}')
    return lines
