"""Does the LiDAR see a WALL where the camera says the wall is? (campaign fix A4)

Pure: no rclpy. front_clearance_node calls confirm_wall() once per depth
frame with the latest /scan and publishes /perception/front_wall_confirmed:
the camera's front_distance when the LiDAR agrees, -1.0 when it does not.
The mission's wall stop reads that topic (wall_stop_require_lidar), so a
person or a plant that leaks into front_distance (R009: the stop fired on a
person ~1.5 m ahead) cannot end a wall move on its own.

WHAT COUNTS AS A WALL. In the forward sector (+-half_angle_deg of the car's
heading, base_link), take the returns whose forward coordinate is within
search_window_m of the distance the camera reports, and fit the dominant
line (the same MSAC + TLS refit lidar_front_wall uses). Confirmed when:
  - the line has >= min_inliers returns on it;
  - the inliers span >= min_extent_m along the line -- a person (legs), a
    plant or a chair leg spans well under a metre; a wall does not;
  - the line faces the car (normal within max_normal_angle_deg of the
    heading) and actually crosses the car's forward axis inside its own
    extent (+- crossing_margin_m), i.e. it is IN FRONT, not off to a side;
  - its forward intercept agrees with the camera's distance (converted to
    base_link) within tolerance_m + tolerance_frac x distance.
Anything else -> not confirmed, with the reason.

WHY NOT /perception/lidar_front_wall. That node fits a +-10 deg sector and
accepts any line there (its own docstring lists "an object face within the
0.3 m gate" as a known blind spot); at 1.5 m the whole sector is 0.53 m wide,
so a person filling it reads as a valid wall fit. The extent test above is
the part that tells a wall from an object, and it needs a wider sector.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from f1tenth_perception.lidar_front_wall import _robust_line

REASON_OK = 'ok'
REASON_NO_CAMERA = 'no_camera_distance'
REASON_NO_SCAN = 'no_scan'
REASON_FEW_RETURNS = 'too_few_returns'
REASON_NO_LINE = 'no_line'
REASON_SHORT = 'segment_too_short'
REASON_OBLIQUE = 'oblique'
REASON_NOT_AHEAD = 'not_in_front'
REASON_DISAGREE = 'distance_disagrees'


@dataclass(frozen=True)
class WallConfirmConfig:
    half_angle_deg: float = 30.0
    search_window_m: float = 0.6
    inlier_distance_m: float = 0.04
    max_pairs: int = 800
    min_inliers: int = 10
    min_extent_m: float = 0.8
    max_normal_angle_deg: float = 45.0
    crossing_margin_m: float = 0.15
    tolerance_m: float = 0.30
    tolerance_frac: float = 0.10


@dataclass
class WallConfirmation:
    confirmed: bool
    reason: str
    lidar_distance_m: float = math.nan     # forward intercept, base_link
    camera_distance_m: float = math.nan    # camera reading, base_link
    extent_m: float = 0.0
    inliers: int = 0
    normal_angle_deg: float = math.nan
    detail: Dict = field(default_factory=dict)

    def as_dict(self) -> Dict:
        def r(v):
            return None if not math.isfinite(v) else round(float(v), 3)
        return {'confirmed': self.confirmed, 'reason': self.reason,
                'lidar_distance_m': r(self.lidar_distance_m),
                'camera_distance_m': r(self.camera_distance_m),
                'extent_m': r(self.extent_m), 'inliers': int(self.inliers),
                'normal_angle_deg': r(self.normal_angle_deg)}


def scan_points_base(ranges: Sequence[float], angle_min: float, angle_increment: float,
                     range_min: float, range_max: float,
                     laser_pose: Tuple[float, float, float],
                     half_angle_rad: float) -> np.ndarray:
    """Valid returns within +-half_angle_rad of the car's heading, as base_link (x, y)."""
    ranges = np.asarray(ranges, dtype=np.float64)
    lx, ly, lyaw = laser_pose
    angles = angle_min + np.arange(ranges.size) * angle_increment + lyaw
    bearings = np.arctan2(np.sin(angles), np.cos(angles))
    with np.errstate(invalid='ignore'):
        valid = ((np.abs(bearings) <= half_angle_rad + 0.5 * abs(angle_increment))
                 & np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max))
    r = ranges[valid]
    b = bearings[valid]
    return np.column_stack([lx + r * np.cos(b), ly + r * np.sin(b)])


def confirm_wall(points_base: Optional[np.ndarray], camera_distance_m: float,
                 camera_x_m: float, cfg: WallConfirmConfig = WallConfirmConfig()
                 ) -> WallConfirmation:
    """Confirm the camera's front distance against the forward LiDAR returns.

    points_base: scan_points_base() output (None = no scan). camera_distance_m:
    front_distance (along the optical axis, from the lens); camera_x_m: the
    lens' x in base_link, so the camera's wall sits at camera_x + distance.
    """
    if not (camera_distance_m is not None and math.isfinite(camera_distance_m)
            and camera_distance_m > 0.0):
        return WallConfirmation(False, REASON_NO_CAMERA)
    d_cam = float(camera_x_m) + float(camera_distance_m)
    if points_base is None:
        return WallConfirmation(False, REASON_NO_SCAN, camera_distance_m=d_cam)
    pts = np.asarray(points_base, dtype=np.float64).reshape(-1, 2)
    near = pts[np.abs(pts[:, 0] - d_cam) <= cfg.search_window_m]
    if near.shape[0] < cfg.min_inliers:
        return WallConfirmation(False, REASON_FEW_RETURNS, camera_distance_m=d_cam,
                                detail={'returns_near': int(near.shape[0])})
    line = _robust_line(near, cfg.inlier_distance_m, cfg.max_pairs)
    if line is None:
        return WallConfirmation(False, REASON_NO_LINE, camera_distance_m=d_cam)
    nx, ny, offset, count, _rms = line
    normal_angle = math.degrees(math.atan2(ny, nx))
    out = WallConfirmation(False, '', camera_distance_m=d_cam, inliers=int(count),
                           normal_angle_deg=normal_angle)
    if count < cfg.min_inliers:
        out.reason = REASON_FEW_RETURNS
        return out
    if abs(normal_angle) > cfg.max_normal_angle_deg or nx <= 1e-6:
        out.reason = REASON_OBLIQUE
        return out
    inlier_pts = near[np.abs(near @ np.array([nx, ny]) - offset) <= cfg.inlier_distance_m]
    tangent = np.array([-ny, nx])
    s = inlier_pts @ tangent
    out.extent_m = float(s.max() - s.min())
    # Where the line crosses the car's forward axis (y = 0).
    x_int = offset / nx
    out.lidar_distance_m = float(x_int)
    s_cross = float(np.array([x_int, 0.0]) @ tangent)
    if out.extent_m < cfg.min_extent_m:
        out.reason = REASON_SHORT
        return out
    if not (s.min() - cfg.crossing_margin_m <= s_cross <= s.max() + cfg.crossing_margin_m):
        out.reason = REASON_NOT_AHEAD
        return out
    if abs(x_int - d_cam) > cfg.tolerance_m + cfg.tolerance_frac * d_cam:
        out.reason = REASON_DISAGREE
        return out
    out.confirmed = True
    out.reason = REASON_OK
    return out
