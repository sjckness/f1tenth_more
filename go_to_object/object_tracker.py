"""Fuse object detections with odometry into a smooth relative estimate.

No ROS. numpy only. See the package docstring for frame conventions.

The central design decision
---------------------------
The object is held as a **point in the world/odom frame**, not relative to
the vehicle:

* A static object does not move in the world frame, so between detections
  there is nothing to propagate. Distance and bearing are recomputed
  geometrically from the current odom pose, so they update at odom rate
  (100 Hz+) rather than at camera rate (10-30 Hz).
* A vehicle closing on the object therefore produces a smoothly shrinking
  distance with *zero* detections arriving.
* Odom drift is slow, and it perturbs relative geometry far less than it
  perturbs absolute position. Relative geometry is all the controller
  consumes.

Do not restructure this to track the object in the body frame.

The structural weakness of that choice is an odom *discontinuity* -- an AMCL
relocalization or a pose-graph optimisation moves the vehicle pose
instantaneously, while the object estimate stays where it was. That is
handled explicitly in :meth:`ObjectTracker.push_odom`.

Known limitation: no data association
-------------------------------------
There is one track and one gate. The gate is the only thing deciding which
detections belong to the object, and it cannot tell "the object, measured
noisily" from "a different object of the same class". With two similar
objects in view, detections alternate between them; each is gated against an
estimate sitting near the other, both are rejected, and after
``max_rejections`` consecutive misses the track resets -- and can then
re-acquire on either one.

This module assumes **a single instance of the target class in view**.
Delivering more than that needs real data association (nearest-neighbour or
JPDA over multiple hypotheses), which is deliberately not implemented here:
a half-measure that silently picks the nearer blob is worse than a
documented single-target assumption, because it fails without saying so.
Upstream filtering -- pick one instance and publish only that -- satisfies
the assumption.

Ambiguities resolved here, stated rather than picked silently:

* ``state(now)`` computes the relative geometry against the *latest* odom
  sample rather than against ``odom.at(now)``. ``now`` drives ageing. In the
  intended wiring the caller ticks on a timer just after odom arrives, so
  the two agree to well under a control period.
* A detection whose capture stamp falls outside the odom buffer is rejected
  without incrementing the consecutive-rejection counter. It is a timing or
  plumbing failure, not evidence that the track is wrong.
* On the ``max_rejections``-th consecutive rejection the track is reset and
  that detection is *discarded*; re-acquisition happens on the next one, so
  ``initialised`` is observably ``False`` in between.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
from math import atan2, cos, hypot, sin
from typing import Deque

import numpy as np

from .pursuit_geometry import wrap_pi

__all__ = [
    'OdomSample',
    'OdomBuffer',
    'TrackerParams',
    'TrackedObject',
    'ObjectTracker',
]

_LOG = logging.getLogger(__name__)

# Degrees of freedom of the innovation, and hence the expectation of the
# normalised innovation squared for a correctly modelled filter.
_NIS_DOF = 2.0


class _Throttle:
    """Emit at most once per ``period`` of the caller's own clock."""

    def __init__(self, period: float = 2.0) -> None:
        self.period = float(period)
        self._last: dict[str, float] = {}

    def ready(self, key: str, now: float) -> bool:
        last = self._last.get(key)
        if last is not None and now - last < self.period:
            return False
        self._last[key] = now
        return True


@dataclass(frozen=True)
class OdomSample:
    """A timestamped planar vehicle pose in the world frame."""

    stamp: float
    x: float
    y: float
    psi: float


class OdomBuffer:
    """Ring buffer of timestamped poses with wrap-aware interpolated lookup."""

    def __init__(self, horizon: float = 2.0, capacity: int = 1024,
                 nominal_period: float = 0.01) -> None:
        if not horizon > 0.0:
            raise ValueError(f'horizon must be > 0, got {horizon}')
        if capacity < 2:
            raise ValueError(f'capacity must be >= 2, got {capacity}')
        if not nominal_period > 0.0:
            raise ValueError('nominal_period must be > 0')
        self.horizon = float(horizon)
        self.nominal_period = float(nominal_period)
        self._buf: Deque[OdomSample] = deque(maxlen=capacity)
        self._throttle = _Throttle()

    def __len__(self) -> int:
        return len(self._buf)

    def clear(self) -> None:
        self._buf.clear()

    def push(self, stamp: float, x: float, y: float, psi: float) -> bool:
        """Append a sample. Returns ``False`` if it was dropped.

        Out-of-order samples are dropped rather than sorted in: re-sorting
        would let a stale pose silently overwrite the interpolation bracket
        that a detection is about to be fused against.
        """
        stamp = float(stamp)
        if self._buf and stamp <= self._buf[-1].stamp:
            return False
        self._buf.append(OdomSample(stamp, float(x), float(y), wrap_pi(psi)))
        return True

    def latest(self) -> OdomSample | None:
        return self._buf[-1] if self._buf else None

    def _period(self) -> float:
        if len(self._buf) >= 2:
            return max(self._buf[-1].stamp - self._buf[-2].stamp, 1e-9)
        return self.nominal_period

    def at(self, stamp: float) -> OdomSample | None:
        """Interpolate the pose at ``stamp``, or ``None`` if unavailable.

        Never extrapolates. A stamp marginally ahead of the newest sample is
        ordinary scheduling jitter and is served with the newest pose, but
        only within one odom period: beyond that the caller's clock and ours
        disagree (a sim-time / wall-clock mismatch is the usual cause), and
        silently serving the newest pose would degrade to *no* latency
        compensation with nothing said. So it is rejected and warned about.
        """
        if not self._buf:
            return None

        stamp = float(stamp)
        newest = self._buf[-1]
        oldest = self._buf[0]

        if stamp > newest.stamp:
            if stamp - newest.stamp <= self._period():
                return newest
            if self._throttle.ready('future', stamp):
                _LOG.warning(
                    'odom lookup %.3f s ahead of the newest sample (period '
                    '%.4f s): detection stamps and odom stamps are not on the '
                    'same clock -- check use_sim_time on both publishers',
                    stamp - newest.stamp, self._period())
            return None
        if stamp < newest.stamp - self.horizon:
            return None
        if stamp < oldest.stamp:
            return None

        if stamp == newest.stamp:
            return newest

        hi = len(self._buf) - 1
        while hi > 0 and self._buf[hi - 1].stamp > stamp:
            hi -= 1
        if hi == 0:
            return oldest

        a = self._buf[hi - 1]
        b = self._buf[hi]
        span = b.stamp - a.stamp
        if span <= 0.0:
            return a
        t = (stamp - a.stamp) / span

        return OdomSample(
            stamp=stamp,
            x=a.x + (b.x - a.x) * t,
            y=a.y + (b.y - a.y) * t,
            # Interpolating 170 deg -> -170 deg naively sweeps the long way
            # round through zero. Step along the wrapped difference instead.
            psi=wrap_pi(a.psi + wrap_pi(b.psi - a.psi) * t),
        )


@dataclass(frozen=True)
class TrackerParams:
    """Tuning for :class:`ObjectTracker`."""

    sigma_range_base: float = 0.05
    """Range noise floor [m], independent of distance."""

    sigma_range_slope: float = 0.002
    """Quadratic range-noise growth [m per m^2]: the along-sight standard
    deviation is ``sigma_range_base + sigma_range_slope * d**2``.

    **Inflating range noise is not the safe direction.** An over-large ``R``
    makes the filter distrust every measurement equally, which widens the
    Mahalanobis gate until gross outliers pass through unchallenged. This is
    a real bug that occurred with a slope of ``0.012``: it produced a 20 m
    range standard deviation at 41 m, and a 40 m outlier sailed through the
    gate.

    Under-stating it is not safe either, and fails less visibly: the gate
    then rejects honest measurements, the track ages out, and the mission
    reports a lost object rather than a mis-tuned filter. The NIS monitor
    (``nis_mean``) exists to tell those two apart at runtime.
    """

    sigma_bearing: float = 0.015
    """Angular noise [rad]; the across-sight standard deviation is
    ``d * sigma_bearing``."""

    q_static: float = 0.01
    """Process noise for a nominally static object [m/sqrt(s)]."""

    p0: float = 4.0
    """Initial per-axis position variance [m^2]."""

    gate_chi2: float = 9.21
    """Mahalanobis gate, 2 DoF, 99%."""

    max_rejections: int = 12
    """Consecutive gated rejections before the track resets."""

    max_age: float = 0.8
    """Staleness limit since the last accepted detection [s].

    Chosen, not inherited: at 2 m/s this is 1.6 m of blind travel on
    odometry alone before the track is dropped.
    """

    n_converged: int = 4
    """Accepted detections at which the count term of ``confidence``
    saturates and ``converged`` is reported."""

    odom_horizon: float = 2.0
    """Odometry lookback available for latency compensation [s]."""

    max_odom_jump: float = 0.5
    """Single-step translation above which odom is treated as discontinuous [m]."""

    max_odom_jump_psi: float = 0.3
    """Single-step rotation above which odom is treated as discontinuous [rad]."""

    odom_jump_var_inflation: float = 0.25
    """Per-axis variance added to ``P`` after a relocalization [m^2]."""

    nis_window: int = 50
    """Accepted updates in the NIS consistency window."""

    nis_alarm_ratio: float = 2.0
    """Windowed-mean NIS, as a multiple of the 2-DoF expectation of 2.0,
    above which the measurement model is reported as inconsistent."""

    def __post_init__(self) -> None:
        if self.sigma_range_base <= 0.0:
            raise ValueError('sigma_range_base must be > 0')
        if self.sigma_range_slope < 0.0:
            raise ValueError('sigma_range_slope must be >= 0')
        if self.sigma_bearing <= 0.0:
            raise ValueError('sigma_bearing must be > 0')
        if self.q_static < 0.0:
            raise ValueError('q_static must be >= 0')
        if self.p0 <= 0.0:
            raise ValueError('p0 must be > 0')
        if self.gate_chi2 <= 0.0:
            raise ValueError('gate_chi2 must be > 0')
        if self.max_rejections < 1:
            raise ValueError('max_rejections must be >= 1')
        if self.max_age <= 0.0:
            raise ValueError('max_age must be > 0')
        if self.n_converged < 1:
            raise ValueError('n_converged must be >= 1')
        if self.odom_horizon <= 0.0:
            raise ValueError('odom_horizon must be > 0')
        if self.max_odom_jump <= 0.0:
            raise ValueError('max_odom_jump must be > 0')
        if self.max_odom_jump_psi <= 0.0:
            raise ValueError('max_odom_jump_psi must be > 0')
        if self.odom_jump_var_inflation < 0.0:
            raise ValueError('odom_jump_var_inflation must be >= 0')
        if self.nis_window < 1:
            raise ValueError('nis_window must be >= 1')
        if self.nis_alarm_ratio <= 0.0:
            raise ValueError('nis_alarm_ratio must be > 0')


@dataclass(frozen=True)
class TrackedObject:
    """Relative estimate handed to the controller."""

    position: np.ndarray
    """World-frame object position [m], the Kalman estimate."""

    covariance: np.ndarray
    """2x2 world-frame position covariance [m^2]."""

    distance: float
    """Planar vehicle-to-object distance [m]."""

    bearing: float
    """Bearing in the vehicle frame [rad], positive to the left. Feeds
    straight into pursuit as its ``alpha``."""

    stamp: float
    age: float
    confidence: float
    converged: bool

    nis_mean: float
    """Windowed mean normalised innovation squared. Expectation is 2.0 for a
    correctly modelled filter; persistently higher means the modelled ``R``
    disagrees with the real sensor. ``0.0`` until the first update."""

    nis_samples: int
    """Accepted updates currently in the NIS window.

    The windowed mean of a chi-squared(2) statistic is itself noisy: its
    standard error is ``2 / sqrt(n)``, so at the default window of 50 a single
    reading carries about +/-0.28 of 1-sigma spread. Readings between roughly
    1.4 and 2.6 are ordinary. Do not read one sample as a trend.
    """

    rejection_rate: float
    """Rolling fraction of detections the gate rejected, in ``[0, 1]``.

    The complement to ``nis_mean``, and necessary because the gate truncates
    it: a rejected update contributes no NIS sample, so an ``R`` that is too
    *tight* hides from NIS exactly when it matters -- the accepted population
    looks consistent while most of the data is being thrown away. That failure
    shows up here instead, as a rejection rate climbing toward 1.
    """


class ObjectTracker:
    """Kalman filter on a *static* 2-D object position in the world frame.

    The state is position only. The prediction step therefore leaves the mean
    untouched and merely grows ``P``; all of the apparent motion in the
    controller's inputs comes from the vehicle moving, not from the object.
    """

    def __init__(self, params: TrackerParams | None = None) -> None:
        self.params = params if params is not None else TrackerParams()
        self._odom = OdomBuffer(horizon=self.params.odom_horizon)
        self._p_filt: np.ndarray | None = None
        self._P: np.ndarray | None = None
        self._t_last_accept: float = 0.0
        self._t_pred: float = 0.0
        self._n_accepted: int = 0
        self._n_rejected: int = 0
        self._nis: Deque[float] = deque(maxlen=self.params.nis_window)
        self._outcomes: Deque[bool] = deque(maxlen=self.params.nis_window)
        self._nis_range: Deque[float] = deque(maxlen=self.params.nis_window)
        self._nis_cross: Deque[float] = deque(maxlen=self.params.nis_window)
        self._throttle = _Throttle()
        self.odom_jumps: int = 0

    # -- odometry ---------------------------------------------------------

    def push_odom(self, stamp: float, x: float, y: float, psi: float) -> bool:
        """Append a pose, handling relocalization discontinuities.

        A jump in the odom pose is new information about the *vehicle*, not
        about the object, so the track is kept and rigidly carried across the
        jump: the object is re-placed so that its position **relative to the
        vehicle** is exactly preserved. Resetting instead would throw away a
        good estimate; doing nothing would leave the relative geometry wrong
        by the whole jump, which either gates every subsequent detection out
        or steers the vehicle at a point the object is not at.
        """
        stamp = float(stamp)
        psi = wrap_pi(psi)
        previous = self._odom.latest()

        if previous is None or stamp <= previous.stamp:
            return self._odom.push(stamp, x, y, psi)

        d_xy = hypot(x - previous.x, y - previous.y)
        d_psi = abs(wrap_pi(psi - previous.psi))
        if (d_xy <= self.params.max_odom_jump
                and d_psi <= self.params.max_odom_jump_psi):
            return self._odom.push(stamp, x, y, psi)

        self.odom_jumps += 1
        if self._p_filt is not None and self._P is not None:
            self._p_filt = self._rigid_carry(previous, (x, y, psi), self._p_filt)
            self._P = self._P + np.eye(2) * self.params.odom_jump_var_inflation

        # Purge rather than keep: interpolating across the discontinuity
        # would synthesise poses the vehicle never occupied, and those would
        # then corrupt the latency compensation of any detection whose
        # capture stamp spans the jump.
        self._odom.clear()
        if self._throttle.ready('odom_jump', stamp):
            _LOG.warning(
                'odom discontinuity at t=%.3f (%.2f m, %.3f rad in one step): '
                'object estimate carried rigidly, P inflated, pose buffer '
                'purged', stamp, d_xy, d_psi)
        return self._odom.push(stamp, x, y, psi)

    @staticmethod
    def _rigid_carry(before: OdomSample, after, point: np.ndarray) -> np.ndarray:
        """Re-express ``point`` so its pose-relative geometry is unchanged."""
        ax, ay, apsi = after
        rel_x = point[0] - before.x
        rel_y = point[1] - before.y
        c0, s0 = cos(before.psi), sin(before.psi)
        body_x = c0 * rel_x + s0 * rel_y
        body_y = -s0 * rel_x + c0 * rel_y
        c1, s1 = cos(apsi), sin(apsi)
        return np.array([ax + c1 * body_x - s1 * body_y,
                         ay + s1 * body_x + c1 * body_y], dtype=float)

    @property
    def odom(self) -> OdomBuffer:
        return self._odom

    @property
    def initialised(self) -> bool:
        return self._p_filt is not None

    @property
    def nis_mean(self) -> float:
        return float(np.mean(self._nis)) if self._nis else 0.0

    @property
    def rejection_rate(self) -> float:
        if not self._outcomes:
            return 0.0
        return float(sum(1 for ok in self._outcomes if not ok)
                     / len(self._outcomes))

    def reset(self) -> None:
        """Drop the track. Odometry history is kept -- it is still valid."""
        self._p_filt = None
        self._P = None
        self._t_last_accept = 0.0
        self._t_pred = 0.0
        self._n_accepted = 0
        self._n_rejected = 0
        self._nis.clear()
        self._nis_range.clear()
        self._nis_cross.clear()
        self._outcomes.clear()

    # -- detections -------------------------------------------------------

    def push_detection(self, stamp: float, body_xy) -> bool:
        """Fuse one detection. Returns ``True`` if it was accepted.

        ``stamp`` must be the *frame capture* time and ``body_xy`` the object
        centroid in the vehicle body frame at that instant.
        """
        stamp = float(stamp)
        bx, by = float(body_xy[0]), float(body_xy[1])

        # Latency compensation. A detection is stamped at frame capture,
        # typically 50-150 ms before it reaches us. Transforming it with the
        # *current* pose instead injects an error of speed x latency along the
        # direction of travel -- at 2 m/s that is 20 cm, arriving as a jump on
        # every single camera frame.
        pose = self._odom.at(stamp)
        if pose is None:
            # Timing/plumbing failure, not evidence about the track: it must
            # not be able to march the track toward a reset.
            return False

        c, s = cos(pose.psi), sin(pose.psi)
        z = np.array([pose.x + c * bx - s * by,
                      pose.y + s * bx + c * by], dtype=float)
        R, rot = self._measurement_covariance(bx, by, pose.psi)

        if self._p_filt is None:
            self._initialise(stamp, z)
            return True

        self._predict_to(stamp)
        assert self._P is not None

        innovation = z - self._p_filt
        S = self._P + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False

        nis = float(innovation @ S_inv @ innovation)
        if nis > self.params.gate_chi2:
            self._outcomes.append(False)
            self._n_rejected += 1
            if self._n_rejected >= self.params.max_rejections:
                # Without this the filter can gate forever against an
                # estimate that has drifted away from the truth.
                self.reset()
            return False

        K = self._P @ S_inv
        self._p_filt = self._p_filt + K @ innovation

        # Joseph form: algebraically equal to (I - K) P for the optimal gain,
        # but it stays symmetric positive-definite under repeated
        # near-singular updates, where the short form drifts and can go
        # indefinite.
        I_KH = np.eye(2) - K
        P = I_KH @ self._P @ I_KH.T + K @ R @ K.T
        self._P = 0.5 * (P + P.T)

        self._outcomes.append(True)
        self._record_nis(nis, innovation, S, rot, stamp)

        self._t_last_accept = stamp
        self._n_accepted += 1
        self._n_rejected = 0
        return True

    def _initialise(self, stamp: float, z: np.ndarray) -> None:
        self._p_filt = z.copy()
        self._P = np.eye(2) * self.params.p0
        self._t_last_accept = stamp
        self._t_pred = stamp
        self._n_accepted = 1
        self._n_rejected = 0

    def _measurement_covariance(self, bx: float, by: float, psi_pose: float):
        """Anisotropic ``R``, built in the line-of-sight frame.

        Depth error is far larger along the line of sight than across it and
        grows with range. A circular ``R`` would make the filter over-trust
        range, pulling the estimate along the sight line on every frame.

        Returns ``(R_world, rotation)``; the rotation is kept so the NIS
        monitor can split the residual back into range and cross components
        and name the parameter responsible.
        """
        d = hypot(bx, by)
        sigma_range = (self.params.sigma_range_base
                       + self.params.sigma_range_slope * d * d)
        sigma_cross = max(d * self.params.sigma_bearing, 1e-6)
        R_los = np.diag([sigma_range ** 2, sigma_cross ** 2])

        theta = psi_pose + atan2(by, bx)
        c, s = cos(theta), sin(theta)
        rot = np.array([[c, -s], [s, c]])
        return rot @ R_los @ rot.T, rot

    def _record_nis(self, nis: float, innovation: np.ndarray, S: np.ndarray,
                    rot: np.ndarray, stamp: float) -> None:
        """Accumulate the consistency statistic and warn when it drifts.

        NIS is the runtime check that the modelled ``R`` matches the sensor
        actually connected. A filter whose ``R`` is too tight gates out good
        detections and presents as a *perception* fault -- the track ages
        out, the mission reports a lost object, and debugging goes to the
        wrong module. This is the cheapest thing that distinguishes the two.
        """
        self._nis.append(nis)

        residual_los = rot.T @ innovation
        S_los = rot.T @ S @ rot
        self._nis_range.append(float(residual_los[0] ** 2 / max(S_los[0, 0], 1e-12)))
        self._nis_cross.append(float(residual_los[1] ** 2 / max(S_los[1, 1], 1e-12)))

        if len(self._nis) < self._nis.maxlen:
            return
        mean = float(np.mean(self._nis))
        if mean <= self.params.nis_alarm_ratio * _NIS_DOF:
            return
        if not self._throttle.ready('nis', stamp):
            return

        range_term = float(np.mean(self._nis_range))
        cross_term = float(np.mean(self._nis_cross))
        culprit = ('sigma_range_base / sigma_range_slope'
                   if range_term >= cross_term else 'sigma_bearing')
        _LOG.warning(
            'NIS mean %.2f over %d updates (expected %.1f): the modelled '
            'measurement noise disagrees with the sensor. Dominant axis is '
            '%s (range %.2f vs cross %.2f) -- check %s',
            mean, len(self._nis), _NIS_DOF,
            'range' if range_term >= cross_term else 'cross',
            range_term, cross_term, culprit)

    def _predict_to(self, stamp: float) -> None:
        """Static object: the mean is unchanged, only ``P`` grows."""
        if self._P is None:
            return
        dt = stamp - self._t_pred
        if dt <= 0.0:
            return
        self._P = self._P + np.eye(2) * (self.params.q_static ** 2) * dt
        self._t_pred = stamp

    # -- output -----------------------------------------------------------

    def state(self, now: float) -> TrackedObject | None:
        """Relative estimate at ``now``, or ``None`` if unusable.

        Returns the Kalman estimate directly. An earlier version rate-limited
        a second, smoothed copy of the position; that is gone. Smoothing the
        *position* cannot bound a *curvature* step -- the transfer between
        them scales as ``1/d**2`` -- so the limit now lives on curvature, in
        :class:`~go_to_object.pursuit_geometry.CurvatureLimiter`, and this
        filter is left to be the only source of lag it ever was.
        """
        now = float(now)
        if self._p_filt is None or self._P is None:
            return None

        age = now - self._t_last_accept
        if age > self.params.max_age:
            return None

        pose = self._odom.latest()
        if pose is None:
            return None

        # Non-mutating forward prediction, so repeated state() calls at the
        # same instant do not compound.
        dt_pred = max(now - self._t_pred, 0.0)
        covariance = self._P + np.eye(2) * (self.params.q_static ** 2) * dt_pred

        dx = float(self._p_filt[0]) - pose.x
        dy = float(self._p_filt[1]) - pose.y
        distance = hypot(dx, dy)
        bearing = wrap_pi(atan2(dy, dx) - pose.psi)

        return TrackedObject(
            position=self._p_filt.copy(),
            covariance=covariance,
            distance=distance,
            bearing=bearing,
            stamp=now,
            age=age,
            confidence=self._confidence(age, covariance),
            converged=self._n_accepted >= self.params.n_converged,
            nis_mean=self.nis_mean,
            nis_samples=len(self._nis),
            rejection_rate=self.rejection_rate,
        )

    def _confidence(self, age: float, covariance: np.ndarray) -> float:
        """Confidence in ``[0, 1]``: detection count, age, and spread.

        Zero on a fresh track -- one detection is not yet evidence -- and
        monotonic in each term.
        """
        p = self.params
        denom = max(p.n_converged - 1, 1)
        count_term = min(max((self._n_accepted - 1) / denom, 0.0), 1.0)
        age_term = min(max(1.0 - age / p.max_age, 0.0), 1.0)

        # RMS per-axis standard deviation, measured against the prior spread:
        # a track no tighter than its initialisation has learned nothing.
        spread = float(np.sqrt(max(np.trace(covariance), 0.0) / 2.0))
        spread_term = min(max(1.0 - spread / np.sqrt(p.p0), 0.0), 1.0)

        return float(count_term * age_term * spread_term)
