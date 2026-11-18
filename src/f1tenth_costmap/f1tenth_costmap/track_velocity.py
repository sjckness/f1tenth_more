"""
track_velocity.py: per-track velocity in the LOCAL odom frame. Pure Python + numpy, no rclpy.

WHY A SECOND ESTIMATE. The semantic layer's tracks (semantic_layer.SemanticObject) live in the MAP
frame and carry an EMA-blended vx_map/vy_map that (a) only camera detections ever write -- a
LiDAR-only "unknown" track always has v = 0 -- (b) is measured against the EMA-smoothed position,
so a constant velocity settles at v / alpha, and (c) inherits every map -> odom jump a SLAM
correction folds into the global EKF. It is only an association aid and is never published.

This module estimates what a controller can use: a constant-velocity Kalman filter per track id,
fed with the RAW measurement of each update (the LiDAR cluster centroid, the camera detection)
brought into the local odom frame AT THE MEASUREMENT'S OWN TIMESTAMP:

    p_odom(t) = T_odom<-base_link(t) . T_base_link<-map(t) . p_map(t)

where T_base_link<-map(t) is the exact inverse of the pose the node used to place the measurement
in map, so the map pose cancels and no SLAM correction reaches the estimate. Differencing positions
already in odom compensates the car's own motion; differencing base_link positions would put the
car's velocity into every object's.

OUTPUT per id (estimate()): vx, vy [m/s, odom], speed, heading [rad, odom], the 2x2 velocity
covariance, sigma_v = sqrt(largest eigenvalue), and v_valid, true when the filter has run for at
least min_age_s, |v| > min_speed and sigma_v < max_sigma_v. A new id starts a new filter (reset on
id change); a track not measured for more than max_gap_s restarts its filter.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional

import numpy as np


@dataclass
class VelocityEstimate:
    """One track's velocity estimate (odom frame)."""

    vx: float
    vy: float
    speed: float
    heading: float
    cov: np.ndarray
    sigma_v: float
    valid: bool
    age_s: float
    x: float
    y: float


class _CvFilter:
    """Constant-velocity Kalman filter, state (x, y, vx, vy), white-acceleration process noise."""

    def __init__(self, t: float, x: float, y: float, meas_sigma: float, init_sigma_v: float):
        self.t0 = self.t = float(t)
        self.s = np.array([x, y, 0.0, 0.0], dtype=float)
        r2 = meas_sigma * meas_sigma
        self.P = np.diag([r2, r2, init_sigma_v ** 2, init_sigma_v ** 2])
        self.n = 1

    def predict(self, dt: float, q_acc: float):
        if dt <= 0.0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = q_acc * q_acc
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = q * np.array([[dt4 / 4, 0, dt3 / 2, 0], [0, dt4 / 4, 0, dt3 / 2],
                          [dt3 / 2, 0, dt2, 0], [0, dt3 / 2, 0, dt2]])
        self.s = F @ self.s
        self.P = F @ self.P @ F.T + Q

    def update(self, t: float, x: float, y: float, q_acc: float, meas_sigma: float):
        self.predict(t - self.t, q_acc)
        self.t = max(self.t, float(t))
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        R = np.eye(2) * meas_sigma * meas_sigma
        innov = np.array([x, y]) - H @ self.s
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ innov
        self.P = (np.eye(4) - K @ H) @ self.P
        self.n += 1


class TrackVelocityEstimator:
    """Per-id constant-velocity filters on odom-frame measurements (see the module docstring)."""

    def __init__(self, q_acc: float = 1.5, meas_sigma: float = 0.05, min_age_s: float = 0.5,
                 min_speed: float = 0.1, max_sigma_v: float = 0.25, max_gap_s: float = 1.0,
                 init_sigma_v: float = 1.0):
        self.q_acc = float(q_acc)
        self.meas_sigma = float(meas_sigma)
        self.min_age_s = float(min_age_s)
        self.min_speed = float(min_speed)
        self.max_sigma_v = float(max_sigma_v)
        self.max_gap_s = float(max_gap_s)
        self.init_sigma_v = float(init_sigma_v)
        self._f: Dict[int, _CvFilter] = {}
        self._last_meas_t: Dict[int, float] = {}

    def update(self, track_id: int, t: float, x: float, y: float) -> bool:
        """Feed one odom-frame measurement of `track_id` at time t; False if it was not new."""
        last = self._last_meas_t.get(track_id)
        if last is not None and t <= last:
            return False
        f = self._f.get(track_id)
        if f is None or t - f.t > self.max_gap_s:
            self._f[track_id] = _CvFilter(t, x, y, self.meas_sigma, self.init_sigma_v)
        else:
            f.update(t, x, y, self.q_acc, self.meas_sigma)
        self._last_meas_t[track_id] = t
        return True

    def estimate(self, track_id: int, t_now: Optional[float] = None) -> Optional[VelocityEstimate]:
        """
        Return the id's velocity (and position predicted to t_now), None if never measured.

        The id's velocity (and position predicted to t_now, if given); None if never measured.
        """
        f = self._f.get(track_id)
        if f is None:
            return None
        vx, vy = float(f.s[2]), float(f.s[3])
        cov = f.P[2:4, 2:4].copy()
        sigma_v = float(math.sqrt(max(np.linalg.eigvalsh(cov).max(), 0.0)))
        speed = math.hypot(vx, vy)
        age = f.t - f.t0
        dt = 0.0 if t_now is None else max(0.0, min(float(t_now) - f.t, self.max_gap_s))
        valid = age >= self.min_age_s and speed > self.min_speed and sigma_v < self.max_sigma_v
        return VelocityEstimate(vx=vx, vy=vy, speed=speed, heading=math.atan2(vy, vx), cov=cov,
                                sigma_v=sigma_v, valid=bool(valid), age_s=age,
                                x=float(f.s[0]) + vx * dt, y=float(f.s[1]) + vy * dt)

    def retain(self, track_ids: Iterable[int]) -> None:
        """Forget every filter whose id is not in track_ids (dead tracks)."""
        keep = set(track_ids)
        for tid in [k for k in self._f if k not in keep]:
            del self._f[tid]
            self._last_meas_t.pop(tid, None)


def interpolate_pose(history, t: float, max_extrapolate_s: float = 0.2):
    """
    Return (x, y, yaw) at time t from a time-sorted [(stamp, (x, y, yaw))], None if out of range.

    (x, y, yaw) at time t from a time-sorted [(stamp, (x, y, yaw))]; None if out of range.

    Linear in x, y and in the unwrapped yaw between the two samples around t; up to
    max_extrapolate_s past the newest sample it holds the newest (a measurement stamped slightly
    after the last odometry message).
    """
    if not history:
        return None
    if t >= history[-1][0]:
        return history[-1][1] if t - history[-1][0] <= max_extrapolate_s else None
    if t < history[0][0]:
        return None
    lo, hi = 0, len(history) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if history[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (ta, (xa, ya, aa)), (tb, (xb, yb, ab)) = history[lo], history[hi]
    w = 0.0 if tb <= ta else (t - ta) / (tb - ta)
    da = math.atan2(math.sin(ab - aa), math.cos(ab - aa))
    return xa + w * (xb - xa), ya + w * (yb - ya), aa + w * da


def map_to_odom_at(p_map, pose_map, pose_odom):
    """
    Return a map point in odom, through base_link (pose_map inverted, then pose_odom).

    A map point -> odom through base_link: exact inverse of pose_map, then pose_odom.
    """
    xm, ym, am = pose_map
    dx, dy = p_map[0] - xm, p_map[1] - ym
    c, s = math.cos(am), math.sin(am)
    bx, by = c * dx + s * dy, -s * dx + c * dy
    xo, yo, ao = pose_odom
    c, s = math.cos(ao), math.sin(ao)
    return xo + c * bx - s * by, yo + s * bx + c * by


def arc_centre_correction(p_bl, radius: float, sensor_xy=(0.0, 0.0)):
    """
    Return a LiDAR cluster's centroid moved away from the sensor by 2 r / pi (base_link).

    A LiDAR cluster's centroid moved away from the sensor by 2 r / pi (base_link).

    A round object is seen as its near half: the centroid of a half-circle arc sits 2 r / pi from
    the centre, towards the sensor, and turns with the bearing as the car drives past -- a static
    object would show a velocity. Moving the centroid back along the bearing removes that.
    """
    dx, dy = p_bl[0] - sensor_xy[0], p_bl[1] - sensor_xy[1]
    d = math.hypot(dx, dy)
    if d < 1e-6 or radius <= 0.0:
        return float(p_bl[0]), float(p_bl[1])
    k = 2.0 * radius / math.pi / d
    return float(p_bl[0] + k * dx), float(p_bl[1] + k * dy)
