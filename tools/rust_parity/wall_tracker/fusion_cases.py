#!/usr/bin/env python3
"""The Rust per-cycle fusion against wall_fusion's Python, on synthetic scenes, bit for bit.

Usage (after `source install/setup.bash`; F1TENTH_WALL_TRACKER_CORE_LIB may name
a fresh cargo build):
    python3 tools/rust_parity/wall_tracker/fusion_cases.py [--n 3000] [--json OUT]

The bag replays (parity_wall_tracker.py) compare the fusion end to end on what
the car saw; this sweeps the branches a bag reaches rarely, with the Python
chain lidar_front_wall_node._publish_observations runs (compensate, project,
cross_check_profile, observations_from_lines, stereo_only_observations over the
WALL cloud) written out with the module functions themselves:

  robust_line   lidar_front_wall._robust_line on walls with noise and
                outliers, tiny sets (one kept hypothesis: numpy's pairwise
                column sum), duplicates (no hypothesis), exact lines
  fusion        a camera looking down the corridor with a random mount
                error; LiDAR walls (some seen by the stereo too: AGREE, some
                with a wrong stereo depth: DISAGREE and the veto), a glass
                stretch only the stereo sees (STEREO_ONLY: the cloud fit), the
                profile with gaps and heights, the odometry compensation on
                or off, the stereo weight anywhere in [0, 1]
  rectangles    verify_rectangles on random class/depth images (uint8 /
                uint16 mm, zeros = no depth) and random segments, in and out
                of the frame and of the stereo range
"""

import argparse
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

from corridor_perception.geometry import Pose2D
from f1tenth_perception import scene_mask, wall_fusion
from f1tenth_perception import wall_tracker_backend as wtb
from f1tenth_perception import wall_tracker_native as wtn
from f1tenth_perception.lidar_front_wall import _robust_line
from f1tenth_perception.scene_labels import WallLine

#: base_link from the optical frame: optical z forward, x right, y down.
R_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def bits_equal(a, b):
    if isinstance(a, float) or isinstance(b, float):
        if isinstance(a, bool) or isinstance(b, bool):
            return a == b
        a, b = float(a), float(b)
        if math.isnan(a) and math.isnan(b):
            return True
        return struct.pack('<d', a) == struct.pack('<d', b)
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        a, b = np.asarray(a), np.asarray(b)
        if a.shape != b.shape:
            return False
        if a.dtype.kind == 'f':
            both = np.isnan(a) & np.isnan(b)
            return bool(np.all(both | (a.view(np.int64) == b.view(np.int64))))
        return bool(np.array_equal(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(bits_equal(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(bits_equal(x, y) for x, y in zip(a, b))
    return a == b


def obs_dict(o):
    wl = o.line
    return {'line': (wl.nx, wl.ny, wl.d, wl.t0, wl.t1, int(wl.inliers), wl.rms, wl.source),
            'z': (o.z_min, o.z_max, o.height_measured), 'source': o.source,
            'confidence': o.confidence, 'cols': (o.col_begin, o.col_end),
            'counts': dict(o.counts), 'stereo_weight': o.stereo_weight}


class Report:
    def __init__(self):
        self.out = {}

    def add(self, name, ok, case=None):
        r = self.out.setdefault(name, {'cases': 0, 'mismatches': 0, 'first': None})
        r['cases'] += 1
        if not ok:
            r['mismatches'] += 1
            if r['first'] is None:
                r['first'] = repr(case)[:600]


# ---- robust line ---------------------------------------------------------

def robust_cases(rng, n):
    out = []
    for k in range(n):
        kind = k % 6
        if kind == 0:     # a wall with noise
            m = int(rng.integers(40, 401))
            ang = rng.uniform(-math.pi, math.pi)
            t = rng.uniform(-2, 2, m)
            p = np.column_stack([t * math.cos(ang), t * math.sin(ang)]) + rng.uniform(-6, 6, 2)
            p += rng.normal(scale=10.0 ** rng.uniform(-3.5, -1.5), size=(m, 2))
        elif kind == 1:   # a wall and outliers (a person in front)
            m = int(rng.integers(40, 300))
            t = rng.uniform(-1.5, 1.5, m)
            p = np.column_stack([np.full(m, 3.0), t]) + rng.normal(scale=0.01, size=(m, 2))
            o = int(m * rng.uniform(0.1, 0.45))
            p[:o] = rng.uniform([1.0, -1.0], [2.5, 1.0], size=(o, 2))
        elif kind == 2:   # tiny sets: few hypotheses, perhaps one
            m = int(rng.integers(3, 6))
            p = rng.uniform(-0.05, 0.05, size=(m, 2)) + rng.uniform(-3, 3, 2)
        elif kind == 3:   # duplicates: no hypothesis longer than the baseline
            p = np.repeat(rng.uniform(-3, 3, (1, 2)), int(rng.integers(3, 50)), axis=0)
        elif kind == 4:   # two walls (a corner)
            m = int(rng.integers(40, 300))
            a = np.column_stack([np.full(m // 2, 2.0), rng.uniform(-1, 1, m // 2)])
            b = np.column_stack([rng.uniform(0.5, 2, m - m // 2), np.full(m - m // 2, 1.0)])
            p = np.vstack([a, b]) + rng.normal(scale=0.005, size=(m, 2))
        else:             # an exact line, float32-rounded like a PointCloud2
            m = int(rng.integers(40, 400))
            t = np.linspace(-1, 1, m)
            p = np.column_stack([1.0 + 0.3 * t, 2.0 - 0.7 * t]).astype(np.float32).astype(float)
        out.append((p, float(rng.choice([0.03, 0.06])), int(rng.choice([300, 800]))))
    return out


def check_robust(rng, n, rep):
    for p, inl, mp in robust_cases(rng, n):
        a = _robust_line(p, inl, mp)
        b = wtn.debug_robust_line(p, inl, mp)
        rep.add('robust_line', bits_equal(a, b) if (a is None) == (b is None) else False,
                (p.shape, inl, mp, a, b))


# ---- the fused cycle -----------------------------------------------------

def py_chain(lines, pts, laser_z, cam, width, distance, z_lo, z_hi, comp, cloud, prm):
    """lidar_front_wall_node._publish_observations' fused branch, module functions only."""
    fx, fy, cx, cy, rot, trans = cam

    def compensate(points):
        if comp is None or len(points) == 0:
            return points
        ps, pim = comp
        xy = pim.inverse().transform_points(ps.transform_points(np.asarray(points)[:, :2]))
        out = np.array(points, dtype=float, copy=True)
        out[:, :2] = xy
        return out

    def project(points):
        return scene_mask.project_to_image(points, fx, fy, cx, cy, rot, trans)

    def project_from_scan(points):
        return project(compensate(points))

    scan_xyz = np.column_stack([pts, np.full(len(pts), laser_z)]) if len(pts) \
        else np.empty((0, 3))
    scan_xyz = compensate(scan_xyz)
    scan_uv = project(scan_xyz)
    scan_depth = ((scan_xyz - trans) @ rot)[:, 2] if len(scan_xyz) else np.empty(0)
    check = wall_fusion.cross_check_profile(
        scan_uv[:, 0], scan_depth, distance, width, tol_base_m=prm['tol_base_m'],
        tol_quad_per_m2=prm['tol_quad_per_m2'])
    obs, rejected = wall_fusion.observations_from_lines(
        lines, check, project_from_scan, width, z_lo, z_hi,
        default_height=prm['default_height'],
        disagree_reject_frac=prm['disagree_reject_frac'],
        stereo_weight=prm['stereo_weight'], conf_range_ref_m=prm['conf_range_ref_m'],
        conf_min_inlier_ref=prm['conf_min_inlier_ref'],
        conf_max_rms_scale=prm['conf_max_rms_scale'])
    if cloud is None:
        def column_points(a, b):
            return None
    else:
        us = project(cloud)[:, 0]
        finite = np.isfinite(us)

        def column_points(a, b):
            sel = finite & (us >= a) & (us < b)
            return cloud[sel] if np.any(sel) else None
    obs += wall_fusion.stereo_only_observations(
        check, column_points, width, min_span_columns=prm['stereo_min_columns'],
        inlier_m=prm['inlier_m'], min_span_m=prm['min_span_m'],
        default_height=prm['default_height'], z_min=z_lo, z_max=z_hi)
    return obs, rejected, check


def small_rotation(rng, s):
    ax = rng.normal(size=3)
    ax /= np.linalg.norm(ax)
    ang = rng.normal(scale=s)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


def scene(rng):
    width = int(rng.choice([320, 640]))
    s = width / 640.0
    fx = fy = 262.4 * s * rng.uniform(0.98, 1.02)
    cx, cy = 320.0 * s + rng.normal(scale=2), 180.0 * s + rng.normal(scale=2)
    rot = small_rotation(rng, 0.03) @ R_OPT
    trans = np.array([0.2, rng.normal(scale=0.02), 0.5])
    laser_z = 0.19
    # The walls: a front wall at x = d, the corridor's two sides.
    d = rng.uniform(2.0, 9.0)
    half = rng.uniform(0.8, 2.0)
    walls = [WallLine(1.0, 0.0, d, -half, half, int(rng.integers(8, 80)),
                      float(rng.uniform(0.0, 0.03)), 'lidar'),
             WallLine(0.0, 1.0, half, -d, -0.2, int(rng.integers(8, 80)),
                      float(rng.uniform(0.0, 0.03)), 'lidar'),
             WallLine(0.0, -1.0, half, 0.2, d, int(rng.integers(8, 80)),
                      float(rng.uniform(0.0, 0.03)), 'lidar')]
    rng.shuffle(walls)
    lines = walls[:int(rng.integers(0, 4))]
    # LiDAR returns on the front wall, except a glass stretch.
    g0, g1 = sorted(rng.uniform(-half, half, 2))
    ys = rng.uniform(-half, half, int(rng.integers(0, 300)))
    ys = ys[(ys < g0) | (ys > g1)]
    pts = np.column_stack([np.full(ys.size, d), ys]) + rng.normal(scale=0.01, size=(ys.size, 2))
    # The stereo profile: the wall's optical depth per column, a bias, gaps.
    u = np.arange(width) + 0.5
    xo = (u - cx) / fx
    # base direction of each column's ray at the image centre row
    dirs = rot @ np.vstack([xo, np.zeros(width), np.ones(width)])
    with np.errstate(divide='ignore', invalid='ignore'):
        lam = (d - trans[0]) / dirs[0]
    hit_y = trans[1] + lam * dirs[1]
    on = (lam > 0) & (np.abs(hit_y) <= half)
    bias = rng.choice([0.0, 0.0, rng.uniform(0.5, 2.0)])
    distance = np.where(on, lam * (1.0 + rng.normal(scale=0.01, size=width)) + bias, np.nan)
    distance[rng.uniform(size=width) < rng.uniform(0, 0.3)] = np.nan
    z_lo = np.where(on, rng.uniform(-0.2, 0.3, width), np.nan)
    z_hi = np.where(on, rng.uniform(1.0, 2.5, width), np.nan)
    if rng.uniform() < 0.3:
        z_lo[:] = np.nan
    # The WALL cloud on the glass stretch (and some clutter), float32-rounded.
    m = int(rng.choice([0, 50, 600, 3000]))
    gy = rng.uniform(g0, g1, m)
    cloud = np.column_stack([np.full(m, d) + rng.normal(scale=0.02, size=m), gy,
                             rng.uniform(0.0, 2.0, m)])
    if m and rng.uniform() < 0.5:
        k = m // 5
        cloud[:k] = rng.uniform([1, -2, 0], [8, 2, 2], size=(k, 3))
    cloud = cloud.astype(np.float32).astype(float) if m else None
    comp = None
    if rng.uniform() < 0.6:
        comp = (Pose2D(rng.normal(), rng.normal(), rng.uniform(-3, 3)),
                Pose2D(rng.normal(), rng.normal(), rng.uniform(-3, 3)))
        # A plausible motion: the image pose near the scan pose.
        ps = comp[0]
        comp = (ps, Pose2D(ps.x + rng.normal(scale=0.1), ps.y + rng.normal(scale=0.1),
                           ps.theta + rng.normal(scale=0.05)))
    prm = {'tol_base_m': 0.05, 'tol_quad_per_m2': 0.012,
           'disagree_reject_frac': float(rng.choice([0.5, 0.3, 0.8])),
           'stereo_weight': float(rng.choice([1.0, rng.uniform(0, 1), 0.0, 1.3, -0.2])),
           'default_height': 2.5, 'conf_range_ref_m': 5.0, 'conf_min_inlier_ref': 8.0,
           'conf_max_rms_scale': 6.0, 'stereo_min_columns': 8,
           'inlier_m': float(rng.choice([0.03, 0.06])), 'min_span_m': 0.5}
    cam = (float(fx), float(fy), float(cx), float(cy), rot, trans)
    return lines, pts, laser_z, cam, width, distance, z_lo, z_hi, comp, cloud, prm


def check_fusion(rng, n, rep, stats):
    for _ in range(n):
        args = scene(rng)
        lines, pts, laser_z, cam, width, distance, z_lo, z_hi, comp, cloud, prm = args
        a_obs, a_rej, a_chk = py_chain(*args)
        b_obs, b_rej, b_chk = wtb.fusion_cycle(
            lines, pts, laser_z, cam, width, distance, z_lo, z_hi, comp, cloud,
            tol_base_m=prm['tol_base_m'], tol_quad_per_m2=prm['tol_quad_per_m2'],
            disagree_reject_frac=prm['disagree_reject_frac'],
            stereo_weight=prm['stereo_weight'], default_height=prm['default_height'],
            conf_range_ref_m=prm['conf_range_ref_m'],
            conf_min_inlier_ref=prm['conf_min_inlier_ref'],
            conf_max_rms_scale=prm['conf_max_rms_scale'],
            stereo_min_columns=prm['stereo_min_columns'], inlier_m=prm['inlier_m'],
            min_span_m=prm['min_span_m'])
        ok = (a_rej == b_rej and len(a_obs) == len(b_obs)
              and all(bits_equal(obs_dict(x), obs_dict(y)) for x, y in zip(a_obs, b_obs))
              and all(bits_equal(a_chk[k], b_chk[k])
                      for k in ('status', 'lidar_m', 'stereo_m', 'tol_m', 'n_px')))
        # The LiDAR observations must carry the very WallLine objects.
        ok = ok and all(x.line is y.line for x, y in zip(a_obs, b_obs) if x.line.source != 'stereo')
        rep.add('fusion', ok, (len(a_obs), len(b_obs), a_rej, b_rej))
        st = np.asarray(a_chk['status'])
        for name, code in (('agree', 1), ('lidar_only', 2), ('stereo_only', 3), ('disagree', 4)):
            stats[f'columns_{name}'] = stats.get(f'columns_{name}', 0) + int(np.sum(st == code))
        stats['observations'] = stats.get('observations', 0) + len(a_obs)
        stats['stereo_only_obs'] = stats.get('stereo_only_obs', 0) + sum(
            1 for o in a_obs if o.source == wall_fusion.SOURCE_STEREO)
        stats['both_obs'] = stats.get('both_obs', 0) + sum(
            1 for o in a_obs if o.source == wall_fusion.SOURCE_BOTH)
        stats['rejected'] = stats.get('rejected', 0) + a_rej


# ---- the rectangle ---------------------------------------------------------

def check_rectangles(rng, n, rep, stats):
    p = wall_fusion.RectParams()
    grid = wall_fusion.rect_grid(p)
    for _ in range(n):
        h, w = (180, 320) if rng.uniform() < 0.5 else (360, 640)
        labels = rng.choice([0, 1, 2, 3, 4, 5, 6, 7], size=(h, w),
                            p=[0.05, 0.05, 0.1, 0.2, 0.05, 0.05, 0.45, 0.05]).astype(np.uint8)
        # blocks of one class, like a real mask
        for _b in range(int(rng.integers(0, 8))):
            r0, c0 = int(rng.integers(0, h)), int(rng.integers(0, w))
            labels[r0:r0 + int(rng.integers(5, 80)), c0:c0 + int(rng.integers(5, 120))] = \
                int(rng.choice([2, 6, 3]))
        depth = rng.uniform(300, 12000, size=(h, w)).astype(np.uint16)
        depth[rng.uniform(size=(h, w)) < 0.15] = 0
        s = w / 640.0
        fx = fy = 262.4 * s
        cx, cy = 320.0 * s, 180.0 * s
        rot = small_rotation(rng, 0.05) @ R_OPT
        trans = np.array([0.2, 0.0, 0.5])
        segs = []
        for _k in range(int(rng.integers(0, 12))):
            x0, y0 = rng.uniform(-2, 10), rng.uniform(-5, 5)
            ang, ln = rng.uniform(-math.pi, math.pi), rng.uniform(0.0, 20.0)
            segs.append(((x0, y0), (x0 + ln * math.cos(ang), y0 + ln * math.sin(ang))))
        laser_z = float(rng.choice([0.19, 0.0]))
        a = wall_fusion.verify_rectangles(segs, labels, depth, fx, fy, cx, cy, rot, trans,
                                          laser_z=laser_z, p=p, grid=grid, depth_scale=0.001)
        b = wtb.verify_rectangles(segs, labels, depth, fx, fy, cx, cy, rot, trans,
                                  laser_z=laser_z, p=p, grid=grid, depth_scale=0.001)
        rep.add('rectangles', bits_equal(a, b), (len(segs), a[:1], b[:1]))
        stats['rect_segments'] = stats.get('rect_segments', 0) + len(segs)
        stats['rect_informative'] = stats.get('rect_informative', 0) + sum(
            1 for v in a if v['coverage'] >= p.coverage_min)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=3000)
    ap.add_argument('--json', type=Path, default=None)
    cfg = ap.parse_args()
    if not wtn.available():
        sys.exit(f'library not loadable: {wtn.load_error()}')
    print(f'library {wtn.library_path()} ({wtn.build_info()})')
    rng = np.random.default_rng(20261024)
    rep, stats = Report(), {}
    check_robust(rng, cfg.n, rep)
    check_fusion(rng, cfg.n, rep, stats)
    check_rectangles(rng, max(200, cfg.n // 5), rep, stats)
    for name, r in rep.out.items():
        print(f'{name:12s} {r["cases"]:7d} cases  {r["mismatches"]} mismatches'
              + (f'  first {r["first"]}' if r['first'] else ''))
    print('coverage', stats)
    total = sum(r['mismatches'] for r in rep.out.values())
    print(f'TOTAL mismatches: {total}')
    if cfg.json:
        cfg.json.parent.mkdir(parents=True, exist_ok=True)
        cfg.json.write_text(json.dumps({'library': wtn.library_path(), 'build': wtn.build_info(),
                                        'n': cfg.n, 'results': rep.out, 'coverage': stats,
                                        'total_mismatches': total}, indent=1))
    return 0 if total == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
