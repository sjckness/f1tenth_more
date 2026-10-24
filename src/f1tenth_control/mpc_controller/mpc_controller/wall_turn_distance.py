"""
Where the wall_turn increment's dFront comes from, in 'semantic' mode.

With corr_wall_turn_distance_source 'semantic', wall_turn.py decides what to
do with a distance; this module only decides which distance it is. Per
corridor rebuild, in this order:

  1. SEMANTIC. A ray from the front bumper along the car's heading, against
     the segments (p0 -> p1) of the usable tracks on /perception/walls. The
     nearest intersection wins. The segments, not the semantic layer's 5 cm
     raster cells: the raster is drawn FROM these segments (f1tenth_costmap's
     wall_raster.py), so it could only add quantisation and a 5 Hz throttle.
  2. LIDAR. When the ray meets no usable segment: the forward-sector line fit
     (/perception/lidar_front_wall_virtual, WallEstimate), turned into the
     same quantity -- the distance along the heading from the bumper -- so
     the value does not step when the source switches.
  3. NONE. Unknown. wall_turn.py keeps the corridor straight until a
     distance appears.

FRAME. The segments are tested in the odom frame, where /perception/walls
publishes them and where MPC_corr's own pose lives. The semantic layer moves
them into the map frame only to draw them; going there and back would add TF
error and nothing else.

WHICH TRACKS COUNT -- the wall rows' caps (wall_boundaries.py), not new ones.
The message as a whole is refused by wall_boundaries.stale_reason: older than
wall_row_max_tf_age_s by receipt, or not in the odom frame. Within it a track
counts when it is CONFIRMED, or COASTING for no longer than
wall_row_max_coast_s (CoastClock's derived coast age); a TENTATIVE track is
not yet a wall. A stereo-only track (SOURCE_STEREO) never counts, whatever
wall_row_allow_stereo_only says: this path takes no camera input. Any role
counts -- a side wall the heading points into is the thing ahead. A refused
track is simply not hit, so the LiDAR fallback applies.

The LiDAR estimate is refused past its own max age by receipt (MPC_corr
passes corr_wall_turn_front_distance_max_age_sec, the cap the legacy source
already runs under).

THE RAY HAS NO WIDTH. A gap between two segments ahead is a miss, and the
LiDAR fit decides. A wall the ray meets behind the bumper, or runs along
(parallel), is not ahead.

Pure logic, no rclpy: MPC_corr.py hands over the cached WallTrackViews and
the WallEstimate's (distance, normal_angle) and calls wall_turn_distance().
"""

import math
from typing import Dict, NamedTuple, Optional, Sequence, Tuple

from mpc_controller import wall_boundaries as wb

SOURCE_SEMANTIC = 'semantic'
SOURCE_LIDAR = 'lidar'
SOURCE_NONE = 'none'

# |cross(heading, segment)| below this is a ray running along the segment.
_PARALLEL_EPS = 1e-9


class WallTurnDistance(NamedTuple):
    """This rebuild's distance and where it came from."""

    # Along the heading from the front bumper [m]; None when unknown.
    distance: Optional[float]
    # SOURCE_SEMANTIC, SOURCE_LIDAR or SOURCE_NONE.
    source: str
    # The track the ray hit (semantic only), else None.
    track_id: Optional[int]


def ray_segment_distance(ox: float, oy: float, heading: float,
                         p0: Tuple[float, float],
                         p1: Tuple[float, float]) -> Optional[float]:
    """
    Return the distance along the ray (ox, oy, heading) to segment p0-p1, or None.

    None when the ray misses, runs parallel to the segment, or meets its
    line behind the origin. An endpoint counts as on the segment.
    """
    ux, uy = math.cos(heading), math.sin(heading)
    ex, ey = p1[0] - p0[0], p1[1] - p0[1]
    denom = ux * ey - uy * ex
    if abs(denom) < _PARALLEL_EPS:
        return None
    wx, wy = p0[0] - ox, p0[1] - oy
    t = (wx * ey - wy * ex) / denom     # along the ray
    s = (wx * uy - wy * ux) / denom     # along the segment, 0 at p0, 1 at p1
    if t < 0.0 or s < 0.0 or s > 1.0:
        return None
    return t


def track_counts(track: wb.WallTrackView, coast_age_s: float, max_coast_s: float) -> bool:
    """Whether a ray hit on this track is a wall (module docstring, WHICH TRACKS COUNT)."""
    if track.source == wb.SOURCE_STEREO:
        return False
    if track.state == wb.STATE_CONFIRMED:
        return True
    return track.state == wb.STATE_COASTING and coast_age_s <= max_coast_s


def nearest_wall_hit(x: float, y: float, heading: float,
                     tracks: Sequence[wb.WallTrackView],
                     coast_ages: Dict[int, float], *,
                     bumper_x_m: float, max_coast_s: float) -> Optional[Tuple[float, int]]:
    """
    Return (distance from the bumper, track id) of the nearest counting hit, or None.

    (x, y) is base_link in odom; the ray starts bumper_x_m ahead of it.
    coast_ages maps track id -> coast age [s]; a missing id is 0.0, as
    CoastClock.age reports for a track it has no stamp for.
    """
    bx = x + bumper_x_m * math.cos(heading)
    by = y + bumper_x_m * math.sin(heading)
    best = None
    for trk in tracks:
        if not track_counts(trk, coast_ages.get(trk.id, 0.0), max_coast_s):
            continue
        t = ray_segment_distance(bx, by, heading, trk.p0, trk.p1)
        if t is not None and (best is None or t < best[0]):
            best = (t, trk.id)
    return best


def lidar_forward_distance(distance: float, normal_angle: float,
                           bumper_x_m: float) -> Optional[float]:
    """
    Return the forward-sector fit as a distance along the heading from the bumper.

    The fit is the line n . p = distance in base_link, n = (cos a, sin a) the
    normal toward the wall (WallEstimate). The heading ray from base_link
    meets it at distance / cos(a); the bumper is bumper_x_m along the same
    ray. None (no distance) for a non-finite fit, a line not ahead, or one met behind the
    bumper -- the same answers ray_segment_distance gives.
    """
    distance = float(distance)
    normal_angle = float(normal_angle)
    if not (math.isfinite(distance) and math.isfinite(normal_angle)):
        return None
    c = math.cos(normal_angle)
    if c <= _PARALLEL_EPS:
        return None
    t = distance / c - float(bumper_x_m)
    return t if t >= 0.0 else None


def wall_turn_distance(x: float, y: float, heading: float,
                       tracks: Sequence[wb.WallTrackView],
                       coast_ages: Dict[int, float],
                       walls_frame: str, walls_age_s: Optional[float],
                       lidar_wall: Optional[Tuple[float, float]],
                       lidar_age_s: Optional[float], *,
                       bumper_x_m: float, odom_frame: str,
                       max_walls_age_s: float, max_coast_s: float,
                       max_lidar_age_s: float) -> WallTurnDistance:
    """
    Pick this rebuild's distance: semantic hit, else LiDAR fit, else unknown.

    walls_age_s / lidar_age_s are receipt ages [s], None when nothing has
    arrived. lidar_wall is WallEstimate's (distance, normal_angle) from the
    base_link origin, None when the estimate is not one.
    """
    if not wb.stale_reason(walls_frame, odom_frame, walls_age_s, max_walls_age_s):
        hit = nearest_wall_hit(x, y, heading, tracks, coast_ages,
                               bumper_x_m=bumper_x_m, max_coast_s=max_coast_s)
        if hit is not None:
            return WallTurnDistance(hit[0], SOURCE_SEMANTIC, hit[1])
    if (lidar_wall is not None and lidar_age_s is not None
            and lidar_age_s <= max_lidar_age_s):
        d = lidar_forward_distance(lidar_wall[0], lidar_wall[1], bumper_x_m)
        if d is not None:
            return WallTurnDistance(d, SOURCE_LIDAR, None)
    return WallTurnDistance(None, SOURCE_NONE, None)
