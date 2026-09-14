"""Sensor -> base_link points, the steering estimate, and fusion for
swept_clearance_node. Pure logic, no rclpy (the lidar_front_wall.py /
lidar_front_wall_node.py split).

Nothing in this file decides clearance: both sensor adapters only produce
points, and swept_corridor.clearance() is the one caller-agnostic judge.

WHICH STEERING ANGLE
--------------------
The corridor should follow the wheels, not the latest command, and on this car
they differ: the MPC model-check latency sweep (tools/mpc_model_check.py,
out3_turn_left.png) bottoms out at 230 ms in a flat 200-300 ms basin. That
sweep lumps EKF lag in with the servo, and its minimum is shallow, so it is an
order of magnitude, not a calibration.

The MPC has no lagged steering estimate to reuse. Its state is (x, y, psi, v);
steering is an input only, and last_u[0] -- the newest command -- is what it
treats as "held by the hardware right now". /joint_states carries a static 0.0
for the hinges and the VESC reports no servo position. So the estimate is
built here from the commands themselves, read off the ackermann_mux OUTPUT
(/ackermann_drive) so a teleop or safety-lane command counts as much as the
MPC's.

STEERING_LAGGED (the default): the command that was in effect `lag_sec` ago --
the newest one received at or before now - lag. Until a command that old
exists, the OLDEST held one. Later rather than earlier.

That is not conservative in every case, and the node's steering_estimate
parameter exists because of the exception: if the newer command steers INTO
something the older arc misses, the lagged corridor reports clear until the
lag has elapsed. STEERING_ENVELOPE covers both directions by evaluating the
lagged angle AND every command still in transit (received within the last
`lag_sec`) and taking the smallest clearance. STEERING_LATEST is the newest
command alone.
"""

import math
from collections import deque
from typing import NamedTuple, Optional

import numpy as np

STEERING_LAGGED = 'lagged_command'
STEERING_LATEST = 'latest_command'
STEERING_ENVELOPE = 'envelope'
STEERING_MODES = (STEERING_LAGGED, STEERING_LATEST, STEERING_ENVELOPE)


def quaternion_to_rotation(x, y, z, w):
    """3x3 rotation matrix of a (not necessarily normalised) quaternion."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError('zero quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                   rotation, translation):
    """LaserScan returns -> (N, 2) base_link XY.

    Drops inf, nan, returns below range_min, and returns above range_max
    (not a measurement). Keeps the whole scan, every angle: pre-cropping to a
    forward window is exactly the fixed-cone bug, so what matters is left to
    the corridor. rotation/translation: base_link <- scan frame, applied in 3D
    before the height is dropped, so a mount yaw (or a rear-facing laser) is
    handled rather than assumed away.
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(r.size)
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r = r[valid]
    a = angles[valid]
    local = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
    base = local @ np.asarray(rotation, dtype=float).T + np.asarray(translation, dtype=float)
    return base[:, :2]


def depth_to_points(depth, fx, fy, cx, cy, rotation, translation, *, stride, min_depth,
                    max_depth, z_min, z_max, max_points):
    """Depth image [m] -> (N, 2) base_link XY of the pixels in the obstacle band.

    Pinhole unprojection of every `stride`-th pixel in both directions with
    the camera_info intrinsics, then base_link <- optical frame. Kept: points
    whose base_link height is in [z_min, z_max] -- above the floor, below the
    car's top -- so the floor and overhead structure do not register. Then
    decimated evenly to at most `max_points`.

    ZED depth has two kinds of invalid pixel and they are not alike (see
    front_clearance_node.py's "-Inf AND NaN/+Inf" section):
      * -inf: closer than the camera can measure. Placed at `min_depth` along
        its ray, an upper bound on the true range, rather than dropped --
        dropping it would read an object against the lens as clear.
      * nan / +inf: no stereo match, no information. Dropped.
    Finite depths <= 0 or above `max_depth` are dropped.
    """
    d = np.asarray(depth, dtype=float)[::stride, ::stride]
    rows, cols = d.shape
    v = (np.arange(rows) * stride)[:, None]
    u = (np.arange(cols) * stride)[None, :]
    too_close = np.isneginf(d)
    measured = np.isfinite(d) & (d > 0.0) & (d <= max_depth)
    keep = too_close | measured
    z = np.where(too_close, min_depth, d)[keep]
    uu = np.broadcast_to(u, d.shape)[keep]
    vv = np.broadcast_to(v, d.shape)[keep]
    optical = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], axis=1)
    base = optical @ np.asarray(rotation, dtype=float).T + np.asarray(translation, dtype=float)
    band = (base[:, 2] >= z_min) & (base[:, 2] <= z_max)
    xy = base[band, :2]
    if max_points > 0 and xy.shape[0] > max_points:
        xy = xy[::int(math.ceil(xy.shape[0] / max_points))]
    return xy


class SteeringHistory:
    """Recent steering commands [rad] with receipt times [s] -- see the module
    docstring for what each mode means and why.
    """

    def __init__(self, lag_sec):
        if lag_sec < 0.0:
            raise ValueError(f'lag_sec must be >= 0, got {lag_sec!r}')
        self.lag_sec = float(lag_sec)
        self._times = deque()
        self._values = deque()

    def __len__(self):
        return len(self._times)

    def add(self, t, steering):
        """Record a command. Non-finite commands are ignored (returns False).
        A timestamp older than the newest held one (a clock reset, a bag
        loop) starts the history over rather than reordering it.
        """
        if not math.isfinite(steering):
            return False
        if self._times and t < self._times[-1]:
            self._times.clear()
            self._values.clear()
        self._times.append(float(t))
        self._values.append(float(steering))
        # Any query comes at now >= t, so its cutoff is >= t - lag and nothing
        # older than the newest command at or before t - lag can be selected.
        cutoff = t - self.lag_sec
        while len(self._times) >= 2 and self._times[1] <= cutoff:
            self._times.popleft()
            self._values.popleft()
        return True

    def latest(self) -> Optional[float]:
        return self._values[-1] if self._values else None

    def lagged(self, now) -> Optional[float]:
        if not self._values:
            return None
        cutoff = now - self.lag_sec
        selected = self._values[0]
        for t, value in zip(self._times, self._values):
            if t > cutoff:
                break
            selected = value
        return selected

    def in_transit(self, now):
        """Commands received within the last lag_sec: angles the wheels are
        still on their way to."""
        cutoff = now - self.lag_sec
        return [v for t, v in zip(self._times, self._values) if t > cutoff]

    def angles(self, now, mode):
        """The steering angles to evaluate the corridor at for `mode`; empty
        when no command has ever been received."""
        if not self._values:
            return []
        if mode == STEERING_LAGGED:
            return [self.lagged(now)]
        if mode == STEERING_LATEST:
            return [self.latest()]
        if mode == STEERING_ENVELOPE:
            return [self.lagged(now), *self.in_transit(now)]
        raise ValueError(f'unknown steering mode {mode!r}; expected one of {STEERING_MODES}')


class SensorReading(NamedTuple):
    name: str
    stamp: Optional[float]   # receipt time [s] of the value, None if never
    value: Optional[float]   # clearance [m]
    timeout: float           # older than this [s] and it is stale


def fuse(now, readings):
    """(fused clearance [m], names of stale sensors).

    The minimum over sensors whose reading is no older than its timeout. A
    stale sensor is left out, never taken as clear. With nothing fresh --
    both stale, or no sensor enabled -- 0.0, so a consumer stops the car.
    """
    fresh = []
    stale = []
    for reading in readings:
        if (reading.stamp is not None and reading.value is not None
                and now - reading.stamp <= reading.timeout):
            fresh.append(reading.value)
        else:
            stale.append(reading.name)
    return (min(fresh) if fresh else 0.0), stale
