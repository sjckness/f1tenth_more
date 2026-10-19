#!/usr/bin/env python3
"""Can the LiDAR's own intensity tell a pane of glass from a wall? (G, feasibility)

Usage (after `source install/setup.bash`):
    python3 tools/glass_intensity_probe.py BAG [--out DIR] [--window N]

NOTHING IN THE STACK READS THIS. It is a measurement tool: it drives the same
in-process replay corridor_wall_replay.py uses (front_clearance_node's mask
tick, then lidar_front_wall_node's tracker), takes the GROUND TRUTH from the
rectangle verification's own flags -- a track is glass when the node says
VERIFIED and glass, normal when it says VERIFIED and not glass -- and then
looks at what the LiDAR reported along those same walls.

WHY THE FLAGS AND NOT TRACK IDS. Track ids are not stable: the replay starts
cold, so the ids differ from the recorded run and from each other between
passes. The flags are the thing under test's own output, which is also the
only labelling that stays correct when the pipeline changes.

PER BEAM, inside a track's own angular span as seen from the laser:
  range, intensity, incidence (the angle between the beam and the segment's
  NORMAL: 0 is square-on, 90 is grazing), and the verdict the negative
  evidence uses -- hit |r - s| <= 0.15, pass r > s + 0.15, occluded
  r < s - 0.15, where s is where the beam would meet the track's line.

INTENSITY IS NORMALISED BY RANGE SQUARED, i * (r / 1 m)^2, because a return's
energy falls as 1/r^2 and every comparison here is between surfaces at
different distances. The raw value is kept alongside it.

Outputs in --out (default output/glass_lidar/): beams.csv (a decimated sample
of the per-beam rows), windows.csv (the per-track features), features.json
(AUC and thresholds) and three plots.
"""

import argparse
import collections
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib            # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt   # noqa: E402

import rclpy                 # noqa: E402
import corridor_wall_replay as cwr   # noqa: E402
from corridor_perception.geometry import wrap_pi   # noqa: E402
from f1tenth_perception import multi_wall_tracker as mwt   # noqa: E402
from f1tenth_perception import wall_fusion   # noqa: E402

TOL_M = 0.15          # the negative evidence's own hit/pass tolerance
NEAR_NORMAL_DEG = 5.0  # "seen square-on"
BEAM_SAMPLE = 7       # every Nth beam row into beams.csv


def beam_features(trk, car_pose, scan_msg, laser_pose):
    """Per-beam rows inside this track's span. None when the span holds none."""
    ranges, bearings, _angles = mwt.scan_in_base_link(
        scan_msg.ranges, scan_msg.angle_min, scan_msg.angle_increment,
        scan_msg.range_min, scan_msg.range_max, laser_pose)
    inten = np.asarray(scan_msg.intensities, dtype=float)
    if inten.size != ranges.size:
        return None
    r = np.asarray(ranges, dtype=float)
    lxy = np.asarray(laser_pose[:2], dtype=float)
    (ax, ay), (bx, by) = mwt.segment_in_base_link(trk, car_pose)
    b0 = math.atan2(ay - lxy[1], ax - lxy[0])
    b1 = math.atan2(by - lxy[1], bx - lxy[0])
    half = 0.5 * abs(wrap_pi(b1 - b0))
    centre = wrap_pi(b0 + 0.5 * wrap_pi(b1 - b0))
    sel = np.flatnonzero(np.abs(wrap_pi(bearings - centre)) <= half)
    if sel.size < 3:
        return None
    bv = bearings[sel]
    theta_bl = trk.theta - car_pose.theta
    n = np.array([math.cos(theta_bl), math.sin(theta_bl)])
    d = float(n @ np.array([ax, ay]))
    cos_inc = n[0] * np.cos(bv) + n[1] * np.sin(bv)
    usable = np.abs(cos_inc) > 1e-3
    if not usable.any():
        return None
    sel, bv, cos_inc = sel[usable], bv[usable], cos_inc[usable]
    s = (d - float(n @ lxy)) / cos_inc
    ahead = s > 0.05
    sel, bv, cos_inc, s = sel[ahead], bv[ahead], cos_inc[ahead], s[ahead]
    if sel.size == 0:
        return None
    rv, iv = r[sel], inten[sel]
    # 0 deg is square-on to the surface, 90 deg is grazing.
    incidence = np.degrees(np.arccos(np.clip(np.abs(cos_inc), 0.0, 1.0)))
    finite = np.isfinite(rv)
    with np.errstate(invalid='ignore'):
        hit = finite & (np.abs(rv - s) <= TOL_M)
        passed = (~finite) | (finite & (rv > s + TOL_M))
        occluded = finite & (rv < s - TOL_M)
    i_norm = np.where(finite, iv * np.square(rv), np.nan)
    return {'beam': sel, 'range': np.where(finite, rv, np.nan), 'intensity':
            np.where(finite, iv, np.nan), 'i_norm': i_norm, 'incidence': incidence,
            'expected': s, 'hit': hit, 'pass': passed, 'occluded': occluded,
            'no_return': ~finite}


def window_features(rows):
    """The four candidate features over one window of beam rows."""
    inc = np.concatenate([r['incidence'] for r in rows])
    i_norm = np.concatenate([r['i_norm'] for r in rows])
    rng = np.concatenate([r['range'] for r in rows])
    hit = np.concatenate([r['hit'] for r in rows])
    passed = np.concatenate([r['pass'] for r in rows])
    nore = np.concatenate([r['no_return'] for r in rows])
    opinionated = int(np.count_nonzero(hit) + np.count_nonzero(passed))
    near = (inc < NEAR_NORMAL_DEG) & np.isfinite(i_norm)
    good = np.isfinite(i_norm) & np.isfinite(inc)
    slope = math.nan
    if np.count_nonzero(good) >= 10 and np.ptp(inc[good]) > 2.0:
        slope = float(np.polyfit(inc[good], i_norm[good], 1)[0])
    return {
        'n_beams': int(inc.size),
        'n_near_normal': int(np.count_nonzero(near)),
        'i_norm_max_near_normal': (float(np.nanmax(i_norm[near]))
                                   if np.any(near) else math.nan),
        'i_norm_median': float(np.nanmedian(i_norm[good])) if good.any() else math.nan,
        'slope_per_deg': slope,
        'pass_ratio': (float(np.count_nonzero(passed) / opinionated)
                       if opinionated else math.nan),
        'no_return_frac': float(np.count_nonzero(nore) / inc.size) if inc.size else math.nan,
        'range_median': float(np.nanmedian(rng)) if np.isfinite(rng).any() else math.nan,
        'incidence_median': float(np.median(inc)),
    }


def auc(pos, neg):
    """Mann-Whitney AUC of pos against neg, NaNs dropped. 0.5 is chance."""
    pos = np.asarray([v for v in pos if v is not None and np.isfinite(v)], dtype=float)
    neg = np.asarray([v for v in neg if v is not None and np.isfinite(v)], dtype=float)
    if pos.size == 0 or neg.size == 0:
        return math.nan, 0, 0
    allv = np.concatenate([pos, neg])
    order = np.argsort(allv, kind='mergesort')
    ranks = np.empty(allv.size, dtype=float)
    ranks[order] = np.arange(1, allv.size + 1)
    # average ranks over ties
    _vals, inv, counts = np.unique(allv, return_inverse=True, return_counts=True)
    sums = np.bincount(inv, weights=ranks)
    ranks = (sums / counts)[inv]
    u = ranks[:pos.size].sum() - pos.size * (pos.size + 1) / 2.0
    return float(u / (pos.size * neg.size)), int(pos.size), int(neg.size)


def best_threshold(pos, neg):
    """(threshold, direction, balanced accuracy) of the best single cut."""
    pos = np.asarray([v for v in pos if np.isfinite(v)], dtype=float)
    neg = np.asarray([v for v in neg if np.isfinite(v)], dtype=float)
    if pos.size == 0 or neg.size == 0:
        return math.nan, '', math.nan
    best = (math.nan, '', 0.0)
    for thr in np.unique(np.concatenate([pos, neg])):
        for direction in ('>=', '<='):
            tp = np.count_nonzero(pos >= thr) if direction == '>=' else np.count_nonzero(pos <= thr)
            tn = np.count_nonzero(neg < thr) if direction == '>=' else np.count_nonzero(neg > thr)
            bal = 0.5 * (tp / pos.size + tn / neg.size)
            if bal > best[2]:
                best = (float(thr), direction, float(bal))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default='output/glass_lidar')
    ap.add_argument('--window', type=int, default=10,
                    help='scans per feature window (default 10)')
    cfg = ap.parse_args()
    out = Path(cfg.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    src = cwr.Src(Path(cfg.bag))
    depth_topic = next((t for t in cwr.DEPTH_TOPICS if src.get(t)), None)
    mask = cwr.MaskReplay(src, depth_topic)
    mask_rows = mask.run()
    cycles = cwr.recorded_cycles(src)
    wall = cwr.WallReplay(src, 'full', mask_rows)
    node = wall.node

    # Per track: the beam rows of every cycle, and what the node called it.
    per_track = collections.defaultdict(list)
    labels = collections.defaultdict(collections.Counter)
    scans = {round(t, 6): m for t, _r, m in src.get('/scan')}
    original = node._track_walls

    def track_walls(msg, laser_pose, sectors, scan_stamp, stamp, pts):
        result = original(msg, laser_pose, sectors, scan_stamp, stamp, pts)
        car = node._poses.at(scan_stamp)
        if car is not None and node.wall_tracker is not None:
            for trk in node.wall_tracker.tracks:
                if trk.verify != wall_fusion.VERIFY_VERIFIED:
                    continue
                rows = beam_features(trk, car, msg, laser_pose)
                if rows is None:
                    continue
                rows['stamp'] = scan_stamp
                per_track[trk.id].append(rows)
                labels[trk.id]['glass' if trk.glass else 'normal'] += 1
        return result

    node._track_walls = track_walls
    wall.run(cycles)

    # ---- labels -----------------------------------------------------------
    label_of, mixed = {}, []
    for tid, counter in labels.items():
        glass, normal = counter['glass'], counter['normal']
        label_of[tid] = 'glass' if glass > normal else 'normal'
        if glass and normal:
            mixed.append((tid, glass, normal))
    n_glass = sum(1 for v in label_of.values() if v == 'glass')
    print(f'verified tracks: {len(label_of)} ({n_glass} glass, '
          f'{len(label_of) - n_glass} normal); mixed-flag tracks: {len(mixed)}')

    # ---- beams.csv --------------------------------------------------------
    with open(out / 'beams.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['stamp', 'track', 'label', 'beam', 'range_m', 'intensity',
                    'i_norm', 'incidence_deg', 'expected_m', 'verdict'])
        for tid, rows in per_track.items():
            for row in rows:
                for k in range(0, row['beam'].size, BEAM_SAMPLE):
                    verdict = ('no_return' if row['no_return'][k] else
                               'hit' if row['hit'][k] else
                               'pass' if row['pass'][k] else
                               'occluded' if row['occluded'][k] else 'other')
                    w.writerow([round(row['stamp'], 6), tid, label_of[tid],
                                int(row['beam'][k]),
                                cwr.num(row['range'][k]), cwr.num(row['intensity'][k], 1),
                                cwr.num(row['i_norm'][k], 1),
                                cwr.num(row['incidence'][k], 2),
                                cwr.num(row['expected'][k]), verdict])

    # ---- windows.csv ------------------------------------------------------
    windows = []
    for tid, rows in per_track.items():
        for i in range(0, len(rows), cfg.window):
            chunk = rows[i:i + cfg.window]
            if len(chunk) < max(2, cfg.window // 2):
                continue
            feat = window_features(chunk)
            feat.update({'track': tid, 'label': label_of[tid], 'n_scans': len(chunk),
                         'stamp': round(chunk[0]['stamp'], 3)})
            windows.append(feat)
    keys = ['track', 'label', 'stamp', 'n_scans', 'n_beams', 'n_near_normal',
            'range_median', 'incidence_median', 'i_norm_max_near_normal',
            'i_norm_median', 'slope_per_deg', 'pass_ratio', 'no_return_frac']
    with open(out / 'windows.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        for row in windows:
            w.writerow({k: (cwr.num(row[k], 4) if isinstance(row[k], float) else row[k])
                        for k in keys})
    print(f'windows: {len(windows)} '
          f'({sum(1 for x in windows if x["label"] == "glass")} glass)')

    # ---- separability -----------------------------------------------------
    report = {'window_scans': cfg.window, 'tracks': len(label_of), 'glass_tracks': n_glass,
              'mixed_flag_tracks': mixed, 'windows': len(windows), 'features': {}}
    for feat in ('i_norm_max_near_normal', 'i_norm_median', 'slope_per_deg',
                 'pass_ratio', 'no_return_frac'):
        pos = [x[feat] for x in windows if x['label'] == 'glass']
        neg = [x[feat] for x in windows if x['label'] == 'normal']
        a, npos, nneg = auc(pos, neg)
        thr, direction, bal = best_threshold(pos, neg)
        report['features'][feat] = {
            'auc': None if math.isnan(a) else round(a, 4),
            'n_glass': npos, 'n_normal': nneg,
            'glass_median': cwr.num(float(np.nanmedian(pos)), 3) if npos else None,
            'normal_median': cwr.num(float(np.nanmedian(neg)), 3) if nneg else None,
            'threshold': cwr.num(thr, 4), 'direction': direction,
            'balanced_accuracy': cwr.num(bal, 4)}
        print(f'  {feat:26s} AUC {a:6.3f}  glass {npos:3d} / normal {nneg:3d}  '
              f'best: {direction} {thr:.4g} -> balanced {bal:.3f}')

    # ---- plots ------------------------------------------------------------
    def scatter(xkey, ykey, xlabel, ylabel, path, ylog=False):
        fig, ax = plt.subplots(figsize=(7.2, 4.4))
        for label, colour in (('normal', '#3b76af'), ('glass', '#d1495b')):
            x, y = [], []
            for tid, rows in per_track.items():
                if label_of[tid] != label:
                    continue
                for row in rows:
                    x.append(row[xkey])
                    y.append(row[ykey])
            if not x:
                continue
            x = np.concatenate(x)
            y = np.concatenate(y)
            ok = np.isfinite(x) & np.isfinite(y)
            ax.scatter(x[ok], y[ok], s=2, alpha=0.12, color=colour, linewidths=0,
                       label=f'{label} ({int(ok.sum())} beams)')
            if ok.sum() > 20:
                edges = np.linspace(np.nanpercentile(x[ok], 1),
                                    np.nanpercentile(x[ok], 99), 14)
                idx = np.clip(np.digitize(x[ok], edges) - 1, 0, edges.size - 2)
                mid = 0.5 * (edges[:-1] + edges[1:])
                med = [np.median(y[ok][idx == k]) if np.any(idx == k) else np.nan
                       for k in range(edges.size - 1)]
                ax.plot(mid, med, color=colour, lw=2.0)
        if ylog:
            ax.set_yscale('log')
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.25)
        ax.legend(loc='best', fontsize=8)
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)

    scatter('incidence', 'i_norm', 'incidence from the surface normal [deg]',
            'intensity x range^2', out / 'intensity_vs_incidence.png', ylog=True)
    scatter('range', 'i_norm', 'range [m]', 'intensity x range^2',
            out / 'intensity_vs_range.png', ylog=True)

    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    edges = np.arange(0.0, 91.0, 7.5)
    for label, colour in (('normal', '#3b76af'), ('glass', '#d1495b')):
        inc, passed, hit = [], [], []
        for tid, rows in per_track.items():
            if label_of[tid] != label:
                continue
            for row in rows:
                inc.append(row['incidence'])
                passed.append(row['pass'])
                hit.append(row['hit'])
        if not inc:
            continue
        inc = np.concatenate(inc)
        passed = np.concatenate(passed)
        hit = np.concatenate(hit)
        idx = np.clip(np.digitize(inc, edges) - 1, 0, edges.size - 2)
        mid, ratio = [], []
        for k in range(edges.size - 1):
            m = idx == k
            n = np.count_nonzero(passed[m]) + np.count_nonzero(hit[m])
            if n >= 20:
                mid.append(0.5 * (edges[k] + edges[k + 1]))
                ratio.append(np.count_nonzero(passed[m]) / n)
        ax.plot(mid, ratio, 'o-', color=colour, label=label)
    ax.set_xlabel('incidence from the surface normal [deg]')
    ax.set_ylabel('pass-through ratio  pass / (pass + hit)')
    ax.set_ylim(-0.02, 1.02)
    ax.grid(alpha=0.25)
    ax.legend(loc='best', fontsize=9)
    fig.tight_layout()
    fig.savefig(out / 'passthrough_vs_incidence.png', dpi=130)
    plt.close(fig)

    (out / 'features.json').write_text(json.dumps(report, indent=1))
    print(f'wrote {out}/beams.csv, windows.csv, features.json and three plots')
    mask.close()
    wall.close()


if __name__ == '__main__':
    sys.exit(main())
