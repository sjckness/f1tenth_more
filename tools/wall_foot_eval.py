#!/usr/bin/env python3
"""P1: is the WALL FOOT a better camera wall source than the stereo columns?

Read-only feasibility study. Nothing here is wired into the stack: it builds
three candidate camera sources over a recorded bag and scores each against the
LiDAR walls of the same cycle.

    python3 tools/wall_foot_eval.py [--out DIR] [--t0 S] [--t1 S]
        [--max-range M] [--frames N] BAG

THE FOOT, AND WHY IT MIGHT BE BETTER. The stereo columns the tracker consumes
today summarise a column's WALL depth, so a wall is only as good as the depth
ON the wall -- which on glass, on dark surfaces and past ~4 m is exactly where
this camera is worst. The foot asks a different question: where does the FLOOR
stop? The floor is textured, near, and the one surface the depth is reliable
on, and the pixel where floor becomes wall is a ray that meets the floor plane
at the wall's base. One ray-plane intersection, no depth on the wall at all.

WHAT IS COMPARED, per depth frame:

  foot_0.5s / foot_1.0s   the foot points of the last 0.5 s / 1.0 s, carried
                          to the current pose on /odometry/filtered, turned
                          into a virtual scan from the camera origin and run
                          through the SAME C extraction the LiDAR walls use.
  stereo_old              StereoColumns -> per-column points -> the same
                          extraction. This is today's camera wall source.
  merged                  foot points plus the stereo_old segments that sit
                          within 10 deg and 0.2 m of a foot or LiDAR segment
                          of the same cycle.

THE RADIAL FILTER, on every camera source. A surface lying ALONG the viewing
ray is not observable: one pixel of lateral error moves it metres. The LiDAR
has the same problem and gates it by incidence angle; the camera sources never
did, and the 45 deg "fans" that come out of a column profile are exactly this.
A segment is dropped when the angle between its direction and the camera ->
midpoint ray is under --ray-angle, or when the bearing span it subtends from
the camera is under --span-deg. Every metric is reported with and without it.

THE REFERENCE is the LiDAR's own walls: mwt.extract_lines on the scan nearest
the frame stamp, same ExtractionParams as the node. A camera segment is scored
against the nearest LiDAR line within --match-m / --match-deg; one with no
such line is SPURIOUS.

TIME BASE: header stamps, t = 0 at the first /scan stamp, as in
corridor_wall_replay.py.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import corridor_wall_replay as cwr                                    # noqa: E402
import scene_labels_replay as slr                                     # noqa: E402

import rclpy                                                          # noqa: E402
from corridor_perception.extraction import extract_segments           # noqa: E402
from corridor_perception.geometry import Pose2D                       # noqa: E402
from corridor_perception.scan import clean_ranges                     # noqa: E402
from f1tenth_perception import multi_wall_tracker as mwt              # noqa: E402
from f1tenth_perception import scene_mask as sm                       # noqa: E402

FLOORS = (sm.CLASS_FLOOR, sm.CLASS_FLOOR_INFERRED)
MAX_RANGE_M = 5.0
WALL_RUN_PX = 3          # consecutive wall pixels above the transition
REOPEN_PX = 10           # floor again within this many px above -> opening
HORIZON_PAD_PX = 3
NO_RETURN = 65.533
VIRTUAL_BINS = 720       # uniform bearing grid, so the C backend takes it
VIRTUAL_FOV_DEG = 100.0
RAY_ANGLE_DEG = 25.0
SPAN_DEG = 3.0
MATCH_M = 0.5
MATCH_DEG = 15.0
GATE_M = 0.2             # merged: stereo_old accepted within this of a foot/LiDAR line
GATE_DEG = 10.0
BUCKETS = ((0.0, 1.5), (1.5, 3.0), (3.0, 5.0))
SIDE_MIN_M, SIDE_MAX_M = 1.5, 5.0


# ===========================================================================
# Geometry
# ===========================================================================

def camera_pose(R, t):
    """(x, y, yaw) of the camera's optical origin in base_link, as a Pose2D.

    The optical frame's +Z is the viewing direction; its heading in base_link
    is where that axis points once rotated.
    """
    fwd = R @ np.array([0.0, 0.0, 1.0])
    return Pose2D(float(t[0]), float(t[1]), float(math.atan2(fwd[1], fwd[0])))


def horizon_row(R, t, fy, cy):
    """Image row the floor plane recedes to, or None when the camera looks up.

    A ray through row v meets z = 0 only while its base-frame z component is
    negative; the row where it turns is the horizon.
    """
    rows = np.arange(0, 4096, dtype=float)
    d = np.stack([np.zeros_like(rows), (rows - cy) / fy, np.ones_like(rows)], axis=1)
    dz = (R @ d.T)[2]
    below = np.flatnonzero(dz < 0.0)
    return float(rows[below[0]]) if below.size else None


def foot_points(classes, depth, fx, fy, cx, cy, R, t, max_range_m=MAX_RANGE_M):
    """Wall-foot points (x, y) in base_link, one per accepted column.

    Vectorised over the whole image: no Python loop over pixels or columns.
    Returns (xy (N, 2), sigma (N,), cols (N,), rows (N,), reasons dict).
    """
    c = np.asarray(classes)
    h, w = c.shape
    is_floor = (c == FLOORS[0]) | (c == FLOORS[1])
    is_wall = c == sm.CLASS_WALL
    is_object = c == sm.CLASS_OBJECT
    is_self = c == sm.CLASS_SELF

    # A run of WALL_RUN_PX wall pixels, anchored at its BOTTOM row.
    run = is_wall.copy()
    for k in range(1, WALL_RUN_PX):
        run[k:] &= is_wall[:-k]
    # The transition: floor at row r, a wall run starting at r - 1.
    trans = np.zeros_like(is_floor)
    trans[1:] = is_floor[1:] & run[:-1]

    # Scanning UP from the bottom is the LAST row index that qualifies.
    order = np.arange(h)[:, None]
    rows = np.where(trans.any(axis=0), (trans * order).max(axis=0), -1)
    cols = np.arange(w)
    ok = rows >= 0
    reasons = {'no_transition': int((~ok).sum())}

    r = np.clip(rows, 0, h - 1)
    # floor -> object is an occluder standing on the floor, not a wall base.
    occluded = ok & is_object[np.clip(r - 1, 0, h - 1), cols]
    # floor again just above the wall: a doorway or an opening, not a base.
    above = np.zeros(w, dtype=bool)
    for k in range(WALL_RUN_PX + 1, WALL_RUN_PX + 1 + REOPEN_PX):
        above |= is_floor[np.clip(r - k, 0, h - 1), cols]
    reopened = ok & above
    # the self mask between the bumper and the foot
    below_self = np.zeros(w, dtype=bool)
    for k in range(1, 6):
        below_self |= is_self[np.clip(r + k, 0, h - 1), cols]
    selfed = ok & (is_self[r, cols] | below_self)

    hz = horizon_row(R, t, fy, cy)
    at_horizon = ok & (np.abs(r - (hz if hz is not None else -1e9)) < HORIZON_PAD_PX)

    keep = ok & ~occluded & ~reopened & ~selfed & ~at_horizon
    reasons.update(occluded=int(occluded.sum()), reopened=int(reopened.sum()),
                   self_mask=int(selfed.sum()), horizon=int(at_horizon.sum()))

    u = cols[keep].astype(float)
    v = r[keep].astype(float)
    d_opt = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=1)
    d_b = d_opt @ R.T
    with np.errstate(divide='ignore', invalid='ignore'):
        s = -float(t[2]) / d_b[:, 2]
    good = np.isfinite(s) & (s > 0)
    xy = t[None, :2] + s[:, None] * d_b[:, :2]
    xy, good_u = xy[good], u[good]
    rng = np.hypot(xy[:, 0] - t[0], xy[:, 1] - t[1])
    near = rng <= max_range_m
    reasons['too_far'] = int((~near).sum())
    xy, rng, good_u = xy[near], rng[near], good_u[near]
    # sigma_r ~ d^2 / (h fy): one pixel of row error, propagated to range.
    sigma = rng ** 2 / max(float(t[2]) * fy, 1e-6)
    _ = depth                      # the foot needs no depth ON the wall
    return xy, sigma, good_u, v, reasons


def virtual_scan_lines(xy, cam, params, bins=VIRTUAL_BINS, fov_deg=VIRTUAL_FOV_DEG):
    """Points -> a uniform virtual scan from `cam` -> C extraction -> base_link lines."""
    if xy.shape[0] < 3:
        return []
    dx, dy = xy[:, 0] - cam.x, xy[:, 1] - cam.y
    rng = np.hypot(dx, dy)
    bearing = np.arctan2(dy, dx) - cam.theta
    bearing = np.arctan2(np.sin(bearing), np.cos(bearing))
    half = math.radians(fov_deg) / 2.0
    inc = 2 * half / bins
    idx = np.floor((bearing + half) / inc).astype(int)
    sel = (idx >= 0) & (idx < bins) & np.isfinite(rng)
    ranges = np.full(bins, NO_RETURN)
    # Nearest return wins the bin, like a laser.
    np.minimum.at(ranges, idx[sel], rng[sel])
    angles = -half + (np.arange(bins) + 0.5) * inc
    # Every range array enters the extraction through clean_ranges: the
    # no-return sentinel must become inf or it is fitted as a surface.
    lines = extract_segments(clean_ranges(ranges, 0.05, NO_RETURN), angles, params)
    return [ln.transform(cam, 'base_link') for ln in lines]


def stereo_old_lines(columns, R, t, params, max_range_m=MAX_RANGE_M):
    """Today's camera wall source: StereoColumns -> points -> the same extraction."""
    d = np.asarray(columns.distance_m, dtype=float)
    u = np.arange(d.size, dtype=float)
    ok = np.isfinite(d) & (d > 0)
    if ok.sum() < 3:
        return [], np.zeros((0, 2))
    z = d[ok]
    x_opt = (u[ok] - columns.cx) / columns.fx * z
    pts = np.stack([x_opt, np.zeros_like(z), z], axis=1) @ R.T + t[None, :]
    xy = pts[:, :2]
    cam = camera_pose(R, t)
    rng = np.hypot(xy[:, 0] - cam.x, xy[:, 1] - cam.y)
    xy = xy[rng <= max_range_m]
    return virtual_scan_lines(xy, cam, params), xy


# ===========================================================================
# The radial filter, and segment comparison
# ===========================================================================

def line_endpoints(line):
    return np.asarray(line.p_start, dtype=float), np.asarray(line.p_end, dtype=float)


def radial_verdict(line, cam, min_ray_deg=RAY_ANGLE_DEG, min_span_deg=SPAN_DEG):
    """(keep, ray_angle_deg, bearing_span_deg) for one camera segment."""
    a, b = line_endpoints(line)
    mid = 0.5 * (a + b)
    ray = mid - np.array([cam.x, cam.y])
    direction = b - a
    nr, nd = np.linalg.norm(ray), np.linalg.norm(direction)
    if nr < 1e-6 or nd < 1e-6:
        return False, 0.0, 0.0
    cosang = abs(float(ray @ direction) / (nr * nd))
    ray_angle = math.degrees(math.acos(min(1.0, cosang)))
    ray_angle = 90.0 - abs(90.0 - ray_angle)          # fold to [0, 90]
    ba = math.atan2(a[1] - cam.y, a[0] - cam.x)
    bb = math.atan2(b[1] - cam.y, b[0] - cam.x)
    span = abs(math.degrees(math.atan2(math.sin(bb - ba), math.cos(bb - ba))))
    keep = (ray_angle >= min_ray_deg) and (span >= min_span_deg)
    return keep, ray_angle, span


def seg_angle(line):
    a, b = line_endpoints(line)
    return math.atan2(b[1] - a[1], b[0] - a[0])


def angle_gap_deg(t1, t2):
    """Undirected angle between two line directions, in [0, 90]."""
    d = math.degrees(abs(math.atan2(math.sin(t1 - t2), math.cos(t1 - t2))))
    return min(d, 180.0 - d)


def point_seg_distance(p, a, b):
    ab = b - a
    n = float(ab @ ab)
    s = 0.0 if n < 1e-12 else float(np.clip((p - a) @ ab / n, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + s * ab)))


def match(line, refs, match_m=MATCH_M, match_deg=MATCH_DEG):
    """The nearest reference line within the gates, with its residual and angle error."""
    a, b = line_endpoints(line)
    mid = 0.5 * (a + b)
    best = None
    for ref in refs:
        ra, rb = line_endpoints(ref)
        ang = angle_gap_deg(seg_angle(line), seg_angle(ref))
        if ang > match_deg:
            continue
        res = point_seg_distance(mid, ra, rb)
        if res > match_m:
            continue
        if best is None or res < best[1]:
            best = (ref, res, ang)
    return best


def near_any(line, refs, gate_m=GATE_M, gate_deg=GATE_DEG):
    return match(line, refs, gate_m, gate_deg) is not None


def side_role(line, cam_range_min=SIDE_MIN_M, cam_range_max=SIDE_MAX_M):
    """'left' / 'right' / None for a segment that could be a side wall."""
    a, b = line_endpoints(line)
    mid = 0.5 * (a + b)
    d = math.hypot(mid[0], mid[1])
    if not (cam_range_min <= d <= cam_range_max):
        return None
    if abs(math.degrees(seg_angle(line))) % 180.0 < 30.0:
        return None                                    # across the path, not a side
    return 'left' if mid[1] > 0 else 'right'


# ===========================================================================
# Metrics
# ===========================================================================

def point_residuals(xy, scan_xy, cam):
    """Distance from each camera point to the nearest LiDAR return, bucketed.

    The segment-level residual needs a segment to exist; this one does not,
    which is what makes it the honest figure when a source barely forms lines.
    """
    out = {b: [] for b in BUCKETS}
    if xy.shape[0] == 0 or scan_xy.shape[0] == 0:
        return out
    d = np.hypot(scan_xy[None, :, 0] - xy[:, 0:1],
                 scan_xy[None, :, 1] - xy[:, 1:2]).min(axis=1)
    rng = np.hypot(xy[:, 0] - cam.x, xy[:, 1] - cam.y)
    for lo, hi in BUCKETS:
        sel = (rng >= lo) & (rng < hi)
        out[(lo, hi)] = d[sel].tolist()
    return out


def range_sensitivity(h_m, fy):
    """d(range)/d(row) [m/px] = r^2 / (h fy): P1's own sigma_r, as a table."""
    return {f'{r:.1f}m': {'rows_below_horizon': round(h_m * fy / r, 1),
                          'm_per_pixel': round(r * r / (h_m * fy), 4)}
            for r in (1.0, 1.5, 2.0, 3.0, 4.0, 5.0)}


def stats(values):
    if not values:
        return {'n': 0, 'median': None, 'p90': None}
    a = np.asarray(values, dtype=float)
    return {'n': int(a.size), 'median': round(float(np.median(a)), 4),
            'p90': round(float(np.percentile(a, 90)), 4)}


def score(per_frame, label):
    """Bucketed residuals, angle error, spurious rate, coverage, drops."""
    res = {b: [] for b in BUCKETS}
    ang, spurious, frames = [], 0, 0
    seen_left = seen_right = 0
    dropped = kept = 0
    for f in per_frame:
        frames += 1
        sides = set()
        for item in f[label]:
            line, keep, _ra, _sp, m = item
            if keep:
                kept += 1
            else:
                dropped += 1
                continue
            a, b = line_endpoints(line)
            d = math.hypot(*(0.5 * (a + b)))
            if m is None:
                spurious += 1
            else:
                for lo, hi in BUCKETS:
                    if lo <= d < hi:
                        res[(lo, hi)].append(m[1])
                ang.append(m[2])
            r = side_role(line)
            if r:
                sides.add(r)
        seen_left += 'left' in sides
        seen_right += 'right' in sides
    out = {'frames': frames, 'segments_kept': kept, 'segments_dropped_radial': dropped,
           'angle_err_deg': stats(ang),
           'pct_angle_over_20': round(100.0 * sum(a > 20.0 for a in ang) / max(len(ang), 1), 1),
           'spurious_per_frame': round(spurious / max(frames, 1), 3),
           'coverage_left_pct': round(100.0 * seen_left / max(frames, 1), 1),
           'coverage_right_pct': round(100.0 * seen_right / max(frames, 1), 1),
           'coverage_both_pct': None}
    for (lo, hi), v in res.items():
        out[f'residual_{lo}_{hi}m'] = stats(v)
    return out


# ===========================================================================
# Main
# ===========================================================================

def open_bag(bag, attempts=6):
    """Src(bag), skipping topics whose recorded type no longer deserializes.

    An archive bag can carry an older f1tenth_messages definition; nothing the
    foot needs is in those topics, and refusing the whole bag over one of them
    would be the wrong trade.
    """
    topics, dropped = list(cwr.TOPICS), set()
    for _ in range(attempts):
        try:
            return cwr.Src(bag, topics=topics), dropped
        except Exception as exc:
            name = None
            text = str(exc)
            for t in topics:
                ty = None
                try:
                    ty = cwr.Src.__dict__ and None
                except Exception:
                    pass
            # the message type is in the exception text; map it back to topics
            for t in list(topics):
                short = t.strip('/').split('/')[-1]
                if short and short in text:
                    name = t
                    break
            if name is None:
                # fall back on the type name, e.g. "MultiWallTrackArray"
                probe = cwr.Src(bag, topics=[])
                for t, ty in probe.types.items():
                    if ty.split('/')[-1] in text and t in topics:
                        name = t
                        break
            if name is None:
                raise
            topics.remove(name)
            dropped.add(name)
    raise SystemExit('could not open the bag after skipping ' + ', '.join(sorted(dropped)))


def run(bag, out_dir, t0=None, t1=None, max_range=MAX_RANGE_M, max_frames=None,
        ray_angle=RAY_ANGLE_DEG, span_deg=SPAN_DEG):
    out_dir = Path(out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    src, dropped = open_bag(bag)
    if dropped:
        print('   topics skipped (recorded with an older message definition): '
              + ', '.join(sorted(dropped)))
    depth_topic = src.pick(cwr.DEPTH_TOPICS)
    if depth_topic is None:
        raise SystemExit('this bag has no depth: the wall foot cannot be built')
    to_base = slr.static_tf([m for _s, _r, m in src.get('/tf_static')])
    poses = cwr.Poses(src.get('/odometry/filtered'))
    params = mwt.extraction_params(backend='c')
    laser = to_base('laser')
    laser_pose = (float(laser[1][0]), float(laser[1][1]), 0.0)

    print(f'== {src.name}: depth on {depth_topic}, {len(src.get(depth_topic))} frames')
    rep = cwr.MaskReplay(src, depth_topic)
    rows = rep.run()
    rep.close()
    cam_frame = rows[0]['columns'].camera_frame if rows else None
    R, t = to_base(cam_frame)
    cam = camera_pose(R, t)
    print(f'   camera {cam_frame} at ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m, '
          f'yaw {math.degrees(cam.theta):.2f} deg')

    scans = src.get('/scan')
    scan_t = np.array([s[0] for s in scans])
    per_frame, timing, foot_hist = [], {'foot_ms': [], 'scan_ms': [], 'total_ms': []}, []
    point_res = {'foot': {b: [] for b in BUCKETS}, 'stereo': {b: [] for b in BUCKETS}}
    accum = []                                 # (stamp, xy) for the time windows
    reasons_tot = {}

    for row in rows:
        ts = row['stamp']
        rel = src.rel(ts)
        if (t0 is not None and rel < t0) or (t1 is not None and rel > t1):
            continue
        if max_frames and len(per_frame) >= max_frames:
            break
        cols = row['columns']
        classes = cwr.image_array(row['classes'].data, row['classes'].height,
                                  row['classes'].width, row['classes'].step,
                                  row['classes'].encoding)
        classes = np.asarray(classes).reshape(row['classes'].height, row['classes'].width)
        w0 = time.perf_counter()
        xy, sigma, _u, _v, reasons = foot_points(
            classes, row['depth'], cols.fx, cols.fy, cols.cx, cols.cy, R, t, max_range)
        t_foot = (time.perf_counter() - w0) * 1e3
        for k, v in reasons.items():
            reasons_tot[k] = reasons_tot.get(k, 0) + v
        foot_hist.append(int(xy.shape[0]))

        here = poses.at(ts)
        accum.append((ts, xy, here))
        accum[:] = [a for a in accum if ts - a[0] <= 1.0]

        def carried(window_s):
            """Foot points of the last window_s, in TODAY's base_link."""
            parts = []
            for (st, pxy, pose) in accum:
                if ts - st > window_s or pxy.shape[0] == 0:
                    continue
                if pose is None or here is None:
                    parts.append(pxy)
                    continue
                # old base_link -> odom -> today's base_link
                co, so = math.cos(pose[2]), math.sin(pose[2])
                world = np.stack([pose[0] + co * pxy[:, 0] - so * pxy[:, 1],
                                  pose[1] + so * pxy[:, 0] + co * pxy[:, 1]], axis=1)
                ch, sh = math.cos(-here[2]), math.sin(-here[2])
                d = world - np.array([here[0], here[1]])
                parts.append(np.stack([ch * d[:, 0] - sh * d[:, 1],
                                       sh * d[:, 0] + ch * d[:, 1]], axis=1))
            return np.vstack(parts) if parts else np.zeros((0, 2))

        w1 = time.perf_counter()
        foot05 = virtual_scan_lines(carried(0.5), cam, params)
        foot10 = virtual_scan_lines(carried(1.0), cam, params)
        t_scan = (time.perf_counter() - w1) * 1e3

        stereo, stereo_xy = stereo_old_lines(cols, R, t, params, max_range)

        i = int(np.clip(np.searchsorted(scan_t, ts), 0, len(scans) - 1))
        scan = cwr.materialize(scans[i][2])
        lidar = mwt.extract_lines(scan.ranges, scan.angle_min, scan.angle_increment,
                                  scan.range_min, scan.range_max, laser_pose,
                                  params=params)

        merged = list(foot05) + [s for s in stereo if near_any(s, foot05 + lidar)]

        r = np.asarray(scan.ranges, dtype=float)
        ang = scan.angle_min + np.arange(r.size) * scan.angle_increment
        good = np.isfinite(r) & (r > scan.range_min) & (r < 20.0)
        scan_xy = np.stack([laser_pose[0] + r[good] * np.cos(ang[good]),
                            laser_pose[1] + r[good] * np.sin(ang[good])], axis=1)
        for key, pts in (('foot', xy), ('stereo', stereo_xy)):
            for b, vals in point_residuals(pts, scan_xy, cam).items():
                point_res[key][b].extend(vals)

        frame = {'t': round(rel, 3), 'n_foot': int(xy.shape[0]),
                 'n_lidar': len(lidar), 'lidar': lidar,
                 'foot_xy': xy, 'stereo_xy': stereo_xy}
        for name, lines in (('foot_0.5s', foot05), ('foot_1.0s', foot10),
                            ('stereo_old', stereo), ('merged', merged)):
            scored = []
            for ln in lines:
                keep, ra, sp = radial_verdict(ln, cam, ray_angle, span_deg)
                scored.append((ln, keep, ra, sp, match(ln, lidar)))
            frame[name] = scored
        per_frame.append(frame)
        timing['foot_ms'].append(t_foot)
        timing['scan_ms'].append(t_scan)
        timing['total_ms'].append(t_foot + t_scan)

    if not per_frame:
        raise SystemExit('no frames in the window')

    report = {'bag': src.name, 'frames': len(per_frame),
              'camera': {'frame': cam_frame, 'xyz': [round(float(q), 4) for q in t],
                         'yaw_deg': round(math.degrees(cam.theta), 3)},
              'foot_points_per_frame': stats(foot_hist),
              'column_rejections': reasons_tot,
              'timing_ms': {k: {'p50': round(float(np.percentile(v, 50)), 2),
                                'p95': round(float(np.percentile(v, 95)), 2)}
                            for k, v in timing.items() if v},
              'range_sensitivity': range_sensitivity(float(t[2]), float(rows[0]['columns'].fy)),
              'point_residual_m': {k: {f'{lo}-{hi}m': stats(v) for (lo, hi), v in d.items()}
                                   for k, d in point_res.items()},
              'sources': {}}
    for name in ('foot_0.5s', 'foot_1.0s', 'stereo_old', 'merged'):
        with_radial = score(per_frame, name)
        forced = [{name: [(ln, True, ra, sp, m) for (ln, _k, ra, sp, m) in f[name]]}
                  for f in per_frame]
        raw = score(forced, name)
        report['sources'][name] = {'radial_on': with_radial, 'radial_off': raw}

    (out_dir / 'wall_foot_eval.json').write_text(json.dumps(report, indent=2))
    write_report(out_dir / 'wall_foot_eval.md', report)
    plots(out_dir, per_frame, cam)
    camera_frames(out_dir, rows, src, R, t, max_range, t0, t1)
    print(f'   wrote {out_dir}/wall_foot_eval.{{json,md}} and the frames')
    return report


def write_report(path, r):
    L = [f"# Wall-foot feasibility — `{r['bag']}`", '',
         f"{r['frames']} depth frames. Camera {r['camera']['frame']} at "
         f"{r['camera']['xyz']} m, yaw {r['camera']['yaw_deg']} deg.", '',
         f"Foot points per frame: median {r['foot_points_per_frame']['median']}, "
         f"p90 {r['foot_points_per_frame']['p90']}.", '',
         '## Column rejections (totals over the window)', '',
         '| reason | columns |', '|---|---|']
    for k, v in r['column_rejections'].items():
        L.append(f'| {k} | {v} |')
    L += ['', '## Timing', '', '| stage | p50 [ms] | p95 [ms] |', '|---|---|---|']
    for k, v in r['timing_ms'].items():
        L.append(f"| {k} | {v['p50']} | {v['p95']} |")
    L += ['', '## Range sensitivity of the foot — d(range)/d(row)', '',
          'One pixel of foot-row error, in metres of range. This is P1\'s own',
          'sigma_r = d^2 / (h fy), with h the camera height.', '',
          '| range | rows below the horizon | m per pixel |', '|---|---|---|']
    for k, v in r['range_sensitivity'].items():
        L.append(f"| {k} | {v['rows_below_horizon']} | {v['m_per_pixel']} |")
    L += ['', '## Point residual — nearest LiDAR return to each camera point', '',
          '| source | 0-1.5 m | 1.5-3 m | 3-5 m |', '|---|---|---|---|']
    for k, d in r['point_residual_m'].items():
        cells = []
        for b in ('0.0-1.5m', '1.5-3.0m', '3.0-5.0m'):
            q = d.get(b, {'n': 0})
            cells.append(f"{q['median']}/{q['p90']} (n={q['n']})" if q['n'] else '--')
        L.append(f"| {k} | " + ' | '.join(cells) + ' |')
    L += ['', '## Per source', '']
    head = ('| source | radial | residual 0-1.5 med/p90 | 1.5-3 med/p90 | 3-5 med/p90 | '
            'angle med | >20 deg | spurious/frame | left % | right % | kept | dropped |')
    L += [head, '|' + '---|' * 12]
    for name, both in r['sources'].items():
        for mode in ('radial_on', 'radial_off'):
            s = both[mode]
            def cell(key):
                d = s[key]
                return f"{d['median']}/{d['p90']} (n={d['n']})" if d['n'] else '--'
            L.append(f"| {name} | {'on' if mode == 'radial_on' else 'off'} | "
                     f"{cell('residual_0.0_1.5m')} | {cell('residual_1.5_3.0m')} | "
                     f"{cell('residual_3.0_5.0m')} | {s['angle_err_deg']['median']} | "
                     f"{s['pct_angle_over_20']} % | {s['spurious_per_frame']} | "
                     f"{s['coverage_left_pct']} | {s['coverage_right_pct']} | "
                     f"{s['segments_kept']} | {s['segments_dropped_radial']} |")
    Path(path).write_text('\n'.join(L) + '\n')


def plots(out_dir, per_frame, cam):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as exc:                                   # pragma: no cover
        print(f'   no matplotlib ({exc}); skipping the frames')
        return
    idx = np.linspace(0, len(per_frame) - 1, min(8, len(per_frame))).astype(int)
    for n, i in enumerate(idx):
        f = per_frame[i]
        fig, ax = plt.subplots(figsize=(7, 7))
        for ln in f['lidar']:
            a, b = line_endpoints(ln)
            ax.plot([a[0], b[0]], [a[1], b[1]], color='0.55', lw=3, zorder=1)
        if f['foot_xy'].shape[0]:
            d = np.hypot(f['foot_xy'][:, 0], f['foot_xy'][:, 1])
            ax.scatter(f['foot_xy'][:, 0], f['foot_xy'][:, 1], c=d, s=5,
                       cmap='viridis', zorder=2)
        for name, col in (('foot_0.5s', 'tab:green'), ('stereo_old', 'tab:red'),
                          ('merged', 'tab:blue')):
            for (ln, keep, _ra, _sp, _m) in f[name]:
                a, b = line_endpoints(ln)
                ax.plot([a[0], b[0]], [a[1], b[1]], color=col, lw=1.6,
                        ls='-' if keep else '--', zorder=3)
        ax.plot([cam.x], [cam.y], 'k^', ms=8)
        ax.set_aspect('equal'); ax.grid(alpha=0.3)
        ax.set_xlim(-1, 6); ax.set_ylim(-3.5, 3.5)
        ax.set_title(f"t = {f['t']:.2f} s   grey LiDAR, green foot, red stereo_old, "
                     f"blue merged, dashed = radial-rejected", fontsize=8)
        fig.savefig(out_dir / f'topdown_{n:02d}_t{f["t"]:.1f}.png', dpi=110,
                    bbox_inches='tight')
        plt.close(fig)


def camera_frames(out_dir, rows, src, R, t, max_range, t0, t1, n=2):
    """Two mask frames with the detected foot row drawn on them."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:                                          # pragma: no cover
        return
    usable = [r for r in rows
              if (t0 is None or src.rel(r['stamp']) >= t0)
              and (t1 is None or src.rel(r['stamp']) <= t1)]
    if not usable:
        return
    for k, i in enumerate(np.linspace(0, len(usable) - 1, n).astype(int)):
        row = usable[i]
        c = row['columns']
        im = row['classes']
        classes = np.asarray(cwr.image_array(im.data, im.height, im.width, im.step,
                                             im.encoding)).reshape(im.height, im.width)
        xy, _sig, u, _v, _rs = foot_points(classes, row['depth'], c.fx, c.fy, c.cx, c.cy,
                                           R, t, max_range)
        # redo the row search so the drawn line is the one the metrics used
        is_floor = (classes == FLOORS[0]) | (classes == FLOORS[1])
        is_wall = classes == sm.CLASS_WALL
        run = is_wall.copy()
        for j in range(1, WALL_RUN_PX):
            run[j:] &= is_wall[:-j]
        trans = np.zeros_like(is_floor)
        trans[1:] = is_floor[1:] & run[:-1]
        order = np.arange(classes.shape[0])[:, None]
        rr = np.where(trans.any(axis=0), (trans * order).max(axis=0), -1).astype(float)
        rr[rr < 0] = np.nan
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.imshow(classes, cmap='tab10', vmin=0, vmax=sm.CLASS_MAX, interpolation='nearest')
        ax.plot(np.arange(classes.shape[1]), rr, color='lime', lw=1.4)
        ax.set_title(f"t = {src.rel(row['stamp']):.2f} s  class mask, foot row in green "
                     f"({xy.shape[0]} accepted columns)", fontsize=9)
        ax.set_axis_off()
        fig.savefig(out_dir / f'camera_{k:02d}.png', dpi=110, bbox_inches='tight')
        plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--out', default='output/wall_v2/p1')
    ap.add_argument('--t0', type=float, default=None)
    ap.add_argument('--t1', type=float, default=None)
    ap.add_argument('--max-range', type=float, default=MAX_RANGE_M)
    ap.add_argument('--frames', type=int, default=None)
    ap.add_argument('--ray-angle', type=float, default=RAY_ANGLE_DEG)
    ap.add_argument('--span-deg', type=float, default=SPAN_DEG)
    a = ap.parse_args(argv)
    rclpy.init()
    try:
        run(a.bag, a.out, a.t0, a.t1, a.max_range, a.frames, a.ray_angle, a.span_deg)
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
