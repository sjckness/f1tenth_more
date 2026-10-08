"""Camera-pan aim law: where to point the camera so it looks where the car is GOING.

Pure geometry + signal conditioning, no rclpy, no ROS -- independently unit
tested (test/test_aim_law.py), the same split swept_corridor.py uses. The node
(camera_pan_controller_node.py) feeds live v / omega_z / steering in and
publishes the result as the pan command.

The behaviour (mode 'track_heading'): aim at a point an arc-length ahead on the
path the car will actually travel -- the bicycle-model arc for the current
curvature, NOT the instantaneous steering ray. Curvature is taken from the
measured yaw rate when the car is moving fast enough to trust omega_z / v, and
from the steering angle (tan(delta)/L) when it is slow or stopped, blended
smoothly so there is no switching glitch.

Frame: base_link = rear axle on the ground, +x forward, +z up, yaw about +z
(left positive). The pan angle is a yaw about +z of the camera pivot, so a
left turn (positive curvature) gives a positive pan.

Arc point (rear-axle bicycle model), curvature kappa, arc length s:
    theta = kappa * s
    P_x = sin(theta) / kappa          -> s           as kappa -> 0
    P_y = (1 - cos(theta)) / kappa     -> 0           as kappa -> 0
Both are evaluated through their kappa->0 series so there is never a divide by
zero (a straight line is just the limit, continuous in kappa).

Pan = atan2(P_y - pivot_y, P_x - pivot_x), i.e. the bearing of the look-ahead
point FROM the camera pivot (not from base_link: the pivot sits ~0.36 m ahead),
then clamped to +-max_pan.

Reversing or stopped -> return to 0 (looking straight ahead): a look-ahead arc
behind the car is not a thing we point the camera at.

PanSmoother adds the temporal conditioning the raw geometry must not do itself
(so it stays a pure function of the instantaneous state): a deadband (ignore
sub-threshold changes), a first-order low-pass, and a slew-rate limit, in that
order, applied per control tick.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AimParams:
    """All tuning for the aim law.

    The node fills these from stack_params.yaml; the defaults here are only for
    the unit tests and must match the yaml.
    """

    wheelbase_m: float = 0.305          # L, tan(delta)/L fallback (= swept_clearance_wheelbase_m)
    max_pan_rad: float = 0.2618         # +-15 deg hard clamp (also the joint limit)
    t_lookahead_s: float = 0.6          # look-ahead time; s = |v| * t_lookahead
    s_min_m: float = 0.5                # look-ahead arc length clamp
    s_max_m: float = 4.0
    pivot_x_m: float = 0.36             # camera pivot in base_link (sim default; car is a param)
    pivot_y_m: float = 0.0
    v_curv_min_mps: float = 0.5         # below |v| this, omega_z/v untrusted -> steering only
    v_curv_full_mps: float = 1.2        # at/above |v| this, trust measured omega_z/v fully
    v_stop_mps: float = 0.1             # |v| below this = stopped -> aim 0
    reverse_aims_zero: bool = True      # v < -v_stop (reversing) -> aim 0


_EPS = 1e-6


def _smoothstep(x: float, lo: float, hi: float) -> float:
    """0 below lo, 1 above hi, smooth (C1) cubic in between. lo < hi required."""
    if hi <= lo:
        return 1.0 if x >= hi else 0.0
    t = (x - lo) / (hi - lo)
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


def curvature(v: float, omega_z: float, delta: float, p: AimParams) -> float:
    """Return the blended path curvature kappa [1/m].

    kappa_steer = tan(delta)/L is always valid. kappa_meas = omega_z/v is only
    trustworthy once |v| is well above noise, so it is blended in by a smoothstep
    of |v| from v_curv_min (weight 0) to v_curv_full (weight 1). No hard switch,
    and at low speed the (possibly huge) omega_z/v never dominates.
    """
    kappa_steer = math.tan(delta) / p.wheelbase_m
    speed = abs(v)
    w = _smoothstep(speed, p.v_curv_min_mps, p.v_curv_full_mps)
    if w <= 0.0 or speed < _EPS:
        return kappa_steer
    kappa_meas = omega_z / v
    return (1.0 - w) * kappa_steer + w * kappa_meas


def lookahead_point(kappa: float, s: float) -> tuple:
    """Return the point (x, y) an arc length s along the constant-curvature arc.

    Measured from the rear axle, in base_link. kappa->0 is handled by a series
    expansion (the straight-line limit), so there is never a divide by zero.
    """
    theta = kappa * s
    if abs(kappa) < _EPS:
        # sin(theta)/kappa = s*sinc(theta); (1-cos theta)/kappa = s*(theta/2 - ...)
        px = s * (1.0 - theta * theta / 6.0)
        py = s * (theta / 2.0)
        return px, py
    px = math.sin(theta) / kappa
    py = (1.0 - math.cos(theta)) / kappa
    return px, py


def aim_pan(v: float, omega_z: float, delta: float, p: AimParams) -> float:
    """Return the raw desired pan angle [rad], clamped to +-max_pan.

    Pure function of the instantaneous state (no history -- PanSmoother owns the
    temporal part). Stopped or reversing -> 0 (look straight ahead).
    """
    if abs(v) < p.v_stop_mps:
        return 0.0
    if p.reverse_aims_zero and v < -p.v_stop_mps:
        return 0.0
    s = min(p.s_max_m, max(p.s_min_m, abs(v) * p.t_lookahead_s))
    kappa = curvature(v, omega_z, delta, p)
    px, py = lookahead_point(kappa, s)
    pan = math.atan2(py - p.pivot_y_m, px - p.pivot_x_m)
    return min(p.max_pan_rad, max(-p.max_pan_rad, pan))


class PanSmoother:
    """Condition the command in time so the image does not jitter.

    Deadband, then a first-order low-pass, then a slew-rate limit. Stateful; one
    per node.

    deadband_rad: ignore a new target within this of the held one (kills
        dithering around a near-constant aim).
    lp_tau_s: low-pass time constant (0 disables). alpha = dt/(tau+dt).
    rate_max_radps: max |d(output)/dt|. Caps how fast the COMMAND moves; the
        servo's own speed limit caps the measured angle (that is what the
        TF-from-measured path exercises).
    """

    def __init__(self, deadband_rad: float, lp_tau_s: float, rate_max_radps: float,
                 initial: float = 0.0):
        self.deadband_rad = max(0.0, deadband_rad)
        self.lp_tau_s = max(0.0, lp_tau_s)
        self.rate_max_radps = max(0.0, rate_max_radps)
        self._out = initial
        self._target_held = initial

    def reset(self, value: float = 0.0) -> None:
        self._out = value
        self._target_held = value

    def update(self, target: float, dt: float) -> float:
        if dt <= 0.0:
            return self._out
        # deadband: only move the held target once the change is worth it
        if abs(target - self._target_held) >= self.deadband_rad:
            self._target_held = target
        # first-order low-pass toward the held target
        if self.lp_tau_s > 0.0:
            alpha = dt / (self.lp_tau_s + dt)
            desired = self._out + alpha * (self._target_held - self._out)
        else:
            desired = self._target_held
        # slew-rate limit
        if self.rate_max_radps > 0.0:
            max_step = self.rate_max_radps * dt
            step = desired - self._out
            step = min(max_step, max(-max_step, step))
            self._out = self._out + step
        else:
            self._out = desired
        return self._out

    @property
    def value(self) -> float:
        return self._out
