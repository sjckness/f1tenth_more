"""Pure-pursuit path tracking for an Ackermann car: the controller of both new modes.

Pure: numpy and math only. The car's reference point is base_link, the rear
axle, which is where the bicycle-model pure-pursuit law is exact:

    delta = atan(2 L sin(alpha) / l_d)

with alpha the bearing of the lookahead point in the car frame and l_d its
distance. The lookahead point is the path point ``l_d`` of ARC LENGTH past the
car's foot point on the path; past the path's end it is extrapolated along the
last segment, so the steering stays calm while the car closes on the goal
instead of swinging toward a point that sits beside it.

The commanded curvature 2 sin(alpha) / l_d also caps the speed (lateral
acceleration), so a tight bend is taken slower. The speed is otherwise the
caller's: arrival, ramps and obstacle slow-downs live in the nodes.
"""

from dataclasses import dataclass
import math
from typing import Optional

from f1tenth_planners.geometry import (
    nearest_on_path, path_arclength, point_at_arclength, world_to_robot)
import numpy as np


@dataclass
class PurePursuitConfig:
    """The tracker's geometry and limits; every value has a stack_params.yaml key."""

    wheelbase: float = 0.305
    delta_min: float = -0.309
    delta_max: float = 0.319
    lookahead_min: float = 0.6
    lookahead_gain: float = 0.5
    lookahead_max: float = 1.5
    lateral_accel_max: float = 1.0


@dataclass
class TrackingCommand:
    """One tick of the tracker's output, and the geometry that produced it."""

    steering: float
    speed_cap: float
    lookahead_xy: tuple
    curvature: float
    cross_track: float
    s_car: float
    remaining: float


class PurePursuit:
    """Track an Nx2 path; set_path() once per plan, step() every tick."""

    def __init__(self, config: PurePursuitConfig):
        self.config = config
        self.path: Optional[np.ndarray] = None
        self._s: Optional[np.ndarray] = None

    def set_path(self, path) -> None:
        """Take a new path (Nx2, odom); None clears it."""
        if path is None:
            self.path, self._s = None, None
            return
        path = np.asarray(path, dtype=float).reshape(-1, 2)
        if len(path) < 2:
            raise ValueError('pure pursuit needs a path of at least two points')
        self.path = path
        self._s = path_arclength(path)

    @property
    def length(self) -> float:
        """Arc length of the current path (0 without one)."""
        return 0.0 if self._s is None else float(self._s[-1])

    def lookahead_distance(self, v: float) -> float:
        """Speed-scheduled lookahead [m]."""
        c = self.config
        return float(np.clip(c.lookahead_min + c.lookahead_gain * abs(v),
                             c.lookahead_min, c.lookahead_max))

    def steering_for(self, x_r: float, y_r: float) -> float:
        """Pure-pursuit steering toward a car-frame point, clamped to the steering bounds."""
        c = self.config
        dist = math.hypot(x_r, y_r)
        if dist < 1e-6:
            return 0.0
        alpha = math.atan2(y_r, x_r)
        if x_r <= 0.0:
            # Behind the axle: the law has no meaning there. Full lock toward it.
            return c.delta_max if y_r >= 0.0 else c.delta_min
        delta = math.atan2(2.0 * c.wheelbase * math.sin(alpha), dist)
        return float(np.clip(delta, c.delta_min, c.delta_max))

    def step(self, pose, v: float) -> TrackingCommand:
        """Steering and curvature speed cap for the car at ``pose`` (x, y, yaw) moving at v."""
        if self.path is None:
            raise RuntimeError('pure pursuit has no path')
        c = self.config
        x, y, _ = pose
        _, s_car, cross = nearest_on_path(self.path, x, y)
        l_d = self.lookahead_distance(v)
        lx, ly, _ = point_at_arclength(self.path, s_car + l_d)
        x_r, y_r = world_to_robot(pose, lx, ly)
        steering = self.steering_for(x_r, y_r)
        dist = max(math.hypot(x_r, y_r), 1e-6)
        curvature = 2.0 * y_r / (dist * dist)
        if abs(curvature) > 1e-6:
            speed_cap = math.sqrt(c.lateral_accel_max / abs(curvature))
        else:
            speed_cap = math.inf
        return TrackingCommand(
            steering=steering, speed_cap=speed_cap, lookahead_xy=(lx, ly),
            curvature=curvature, cross_track=cross, s_car=s_car,
            remaining=max(self.length - s_car, 0.0))


def min_turning_radius(wheelbase: float, delta_min: float, delta_max: float) -> float:
    """Tightest turn radius [m] the steering bounds allow (the weaker side)."""
    bound = min(abs(delta_min), abs(delta_max))
    if bound <= 1e-6:
        return math.inf
    return wheelbase / math.tan(bound)


def rate_limit(previous: float, target: float, max_rate: float, dt: float) -> float:
    """Move from ``previous`` toward ``target`` by at most max_rate * dt."""
    if max_rate <= 0.0 or dt <= 0.0:
        return target
    step = max_rate * dt
    return previous + float(np.clip(target - previous, -step, step))
