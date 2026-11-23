"""Noise / error models. All parameters from sensors.yaml; everything except odometry is off by default.

* lidar: Gaussian range noise.
* depth (ZED-like, for later): NaN on far floor pixels with probability p, Gaussian noise sigma = k*z^2,
  synthetic confidence map consistent with the NaN mask.
* imu: white Gaussian noise on gyro and accelerometer.
* odometry: speed scale error + yaw-rate bias (applied in the wheel-odometry integrator).
"""
from __future__ import annotations

import numpy as np


class NoiseModels:
    def __init__(self, sensors_cfg: dict, floor_class_id: int | None) -> None:
        self.cfg = sensors_cfg
        dn = sensors_cfg["stereo"]["depth_noise"]
        self.rng = np.random.default_rng(int(dn["seed"]))
        self.floor_id = floor_class_id

    def lidar(self, ranges: np.ndarray) -> np.ndarray:
        n = self.cfg["lidar"]["noise"]
        if not n["enabled"]:
            return ranges
        out = ranges.copy()
        finite = np.isfinite(out)
        out[finite] += self.rng.normal(0.0, float(n["stddev"]), int(finite.sum())).astype(out.dtype)
        return out

    def depth(self, depth: np.ndarray, semantic_ids: np.ndarray | None) -> tuple[np.ndarray, np.ndarray | None]:
        """Return (noisy depth, confidence map or None). ``semantic_ids`` holds class ids per pixel."""
        dn = self.cfg["stereo"]["depth_noise"]
        if not dn["enabled"]:
            return depth, None
        out = depth.astype(np.float32).copy()
        finite = np.isfinite(out)
        k = float(dn["k_quadratic"])
        out[finite] += (self.rng.normal(0.0, 1.0, int(finite.sum())) * k * out[finite] ** 2).astype(np.float32)
        nan_mask = np.zeros(out.shape, dtype=bool)
        if semantic_ids is not None and self.floor_id is not None:
            floor = (semantic_ids == self.floor_id) & finite & (depth > float(dn["floor_nan_beyond"]))
            nan_mask = floor & (self.rng.random(out.shape) < float(dn["floor_nan_probability"]))
            out[nan_mask] = np.nan
        conf = None
        if dn["publish_confidence"]:
            # ZED convention: 0..100, lower = more confident. NaN pixels -> 100.
            conf = np.clip(100.0 * k * np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0) ** 2 / 0.5, 0, 99)
            conf = conf.astype(np.float32)
            conf[nan_mask | ~finite] = 100.0
        return out, conf

    def imu(self, gyro: np.ndarray, accel: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        n = self.cfg["imu"]["noise"]
        if not n["enabled"]:
            return gyro, accel
        return (gyro + self.rng.normal(0.0, float(n["gyro_stddev"]), 3),
                accel + self.rng.normal(0.0, float(n["accel_stddev"]), 3))


class WheelOdometry:
    """Bicycle-model dead reckoning like vesc_to_odom: speed from the rear wheels, steering from the command."""

    def __init__(self, wheelbase: float, odom_cfg: dict) -> None:
        self.L = float(wheelbase)
        n = odom_cfg["noise"]
        self.scale = 1.0 + (float(n["speed_scale_error"]) if n["enabled"] else 0.0)
        self.bias = float(n["yaw_rate_bias"]) if n["enabled"] else 0.0
        self.reset()

    def reset(self) -> None:
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0

    def update(self, wheel_speed: float, steering: float, dt: float) -> None:
        v = wheel_speed * self.scale
        w = v * np.tan(steering) / self.L + self.bias
        # midpoint integration
        yaw_mid = self.yaw + 0.5 * w * dt
        self.x += v * np.cos(yaw_mid) * dt
        self.y += v * np.sin(yaw_mid) * dt
        self.yaw = float(np.arctan2(np.sin(self.yaw + w * dt), np.cos(self.yaw + w * dt)))
        self.v, self.w = v, w
