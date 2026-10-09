#!/usr/bin/env python3
"""Recompute /perception/front_distance from a bag (campaign fix A5).

Usage (after `source install/setup.bash`):
    python3 tools/replay_front_distance.py <run folder | bag folder> [--csv out.csv]
        [--estimator band|median|p25] [--mode auto|crop|full]

Two ways to recompute, picked by what the bag holds (--mode auto):

  crop  /perception/debug/front_roi_depth + front_roi_class (recorded since
        the A5 commit, front_clearance_publish_debug_raw true): the ROI depth
        and what front_clearance_node made of every pixel (0 no depth,
        1 object-excluded, 2 background, 3 LiDAR housing). The estimator and
        the EMA are re-run on the background pixels -- exact for the shipped
        estimator, and --estimator tries alternatives on the SAME pixels.
  full  the full depth image (/zed2/zed_node/depth/depth_registered) plus
        /camera/detections (+ /camera/detection_masks): the real
        FrontClearanceNode is instantiated and fed the bag in receive order,
        its clock replaced by the bag clock. Only possible on a bag that
        recorded the depth image (the mission logger does not by default).

Compared against the recorded /perception/debug/front_distance_raw and
/perception/front_distance when present. Prints a summary; --csv writes one
row per depth frame.

NOTE: the R009-R011 bags predate A5. Unless depth was recorded by hand they
hold neither mode's input, and the tool says so (tools/m02_orin_report.py
reports what CAN be recovered from them).
"""

import argparse
import csv
import sys
import pathlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from campaign_bag import latest_before, nearest, read_topics, resolve_bag, stamp_s  # noqa: E402

CROP_DEPTH = '/perception/debug/front_roi_depth'
CROP_CLASS = '/perception/debug/front_roi_class'
REC_RAW = '/perception/debug/front_distance_raw'
REC_FD = '/perception/front_distance'
REC_DEBUG = '/perception/debug/front_clearance'
DEPTH = '/zed2/zed_node/depth/depth_registered'
DETS = '/camera/detections'
MASKS = '/camera/detection_masks'
OBST = '/perception/obstacles_2d'
SCAN = '/scan'
TF_STATIC = '/tf_static'


def _params():
    """front_clearance_* from stack_params when the workspace is sourced."""
    names = {'background_percentile_low': 5.0, 'background_percentile_high': 50.0,
             'min_bg_pixels_for_reading': 60, 'distance_ema_alpha': 0.25}
    try:
        from f1tenth_params.param_defaults import get_value
        return {k: get_value('front_clearance_' + k) for k in names}
    except Exception:  # noqa: BLE001 - offline fallback, said out loud
        print('[replay] stack_params not reachable: using the 2026-10-09 values',
              file=sys.stderr)
        return names


def estimate(depths, how, p):
    if depths.size == 0:
        return None
    if how == 'median':
        return float(np.median(depths))
    if how == 'p25':
        return float(np.percentile(depths, 25.0))
    lo = np.percentile(depths, p['background_percentile_low'])
    hi = np.percentile(depths, p['background_percentile_high'])
    band = depths[(depths >= lo) & (depths <= hi)]
    return float(np.mean(band if band.size else depths))


def _image(msg):
    from cv_bridge import CvBridge
    return CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')


def replay_crops(data, how, p):
    by_stamp = {}
    for t, msg in data[CROP_CLASS]:
        by_stamp[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
    rows = []
    ema = None
    for t, dmsg in data[CROP_DEPTH]:
        key = (dmsg.header.stamp.sec, dmsg.header.stamp.nanosec)
        cmsg = by_stamp.get(key)
        if cmsg is None:
            continue
        depth = np.asarray(_image(dmsg), dtype=np.float32)
        cls = np.asarray(_image(cmsg), dtype=np.uint8)
        bg = depth[cls == 2]
        raw = (estimate(bg, how, p) if bg.size >= int(p['min_bg_pixels_for_reading'])
               else None)
        if raw is not None:
            a = float(p['distance_ema_alpha'])
            ema = raw if ema is None else a * raw + (1.0 - a) * ema
        rec_raw = nearest(data[REC_RAW], t)
        rec_fd = nearest(data[REC_FD], t)
        rows.append({
            't': t, 'stamp': stamp_s(dmsg.header), 'bg_px': int(bg.size),
            'excluded_px': int(np.count_nonzero(cls == 1)),
            'no_depth_px': int(np.count_nonzero(cls == 0)),
            'recomputed_raw': raw, 'recomputed_fd': ema,
            'recorded_raw': None if rec_raw is None else float(rec_raw[1].data),
            'recorded_fd': None if rec_fd is None else float(rec_fd[1].data),
        })
    return rows


def replay_full(data, how):
    """Feed the bag through the real FrontClearanceNode (shipped estimator only)."""
    import rclpy
    from f1tenth_perception.front_clearance_node import FrontClearanceNode
    if how != 'band':
        print('[replay] --estimator is ignored in full mode (the node runs its own)',
              file=sys.stderr)
    rclpy.init()
    node = FrontClearanceNode(parameter_overrides=[
        rclpy.parameter.Parameter('publish_debug_raw', value=True)])
    clock = {'t': 0.0}
    node._now_seconds = lambda: clock['t']
    out = {'raw': [], 'fd': []}
    node.front_distance_raw_pub.publish = lambda m: out['raw'].append(m.data)
    node.front_distance_pub.publish = lambda m: out['fd'].append(m.data)
    for name in ('front_wall_pub', 'front_clearance_pub', 'front_blocked_pub',
                 'bg_pixel_count_pub', 'front_wall_confirmed_pub', 'front_debug_pub',
                 'front_roi_depth_pub', 'front_roi_class_pub'):
        pub = getattr(node, name, None)
        if pub is not None:
            pub.publish = lambda m: None
    for _, msg in data[TF_STATIC]:
        node._tf_static_callback(msg)
    events = []
    for topic, cb in ((DETS, node._detections_callback), (MASKS, node._masks_callback),
                      (OBST, node._obstacles_callback), (SCAN, node._scan_callback),
                      (DEPTH, node.depth_callback)):
        events += [(t, i, topic, cb, m) for i, (t, m) in enumerate(data[topic])]
    # Same receive time: the cached inputs first, the depth frame last (the
    # node does its work on the depth frame).
    events.sort(key=lambda e: (e[0], e[2] == DEPTH))
    rows = []
    for t, _, topic, cb, msg in events:
        clock['t'] = t
        n_fd = len(out['fd'])
        cb(msg)
        if topic == DEPTH and len(out['fd']) > n_fd:
            rec_raw = nearest(data[REC_RAW], t)
            rec_fd = nearest(data[REC_FD], t)
            mask = latest_before(data[MASKS], t)
            rows.append({
                't': t, 'stamp': stamp_s(msg.header),
                'mask_age_s': (None if mask is None
                               else stamp_s(msg.header) - stamp_s(mask[1].header)),
                'recomputed_raw': out['raw'][-1] if out['raw'] else None,
                'recomputed_fd': out['fd'][-1],
                'recorded_raw': None if rec_raw is None else float(rec_raw[1].data),
                'recorded_fd': None if rec_fd is None else float(rec_fd[1].data),
            })
    node.destroy_node()
    rclpy.shutdown()
    return rows


def summarize(rows):
    def diffs(a, b):
        out = []
        for r in rows:
            x, y = r.get(a), r.get(b)
            if x is None or y is None or x < 0 or y < 0:
                continue
            out.append(abs(x - y))
        return out
    print(f'frames: {len(rows)}')
    for a, b in (('recomputed_raw', 'recorded_raw'), ('recomputed_fd', 'recorded_fd')):
        d = diffs(a, b)
        if d:
            print(f'|{a} - {b}|: n={len(d)} mean={np.mean(d):.4f} '
                  f'max={np.max(d):.4f} m')
        else:
            print(f'{b}: not in the bag (nothing to compare {a} against)')
    fd = [r['recomputed_fd'] for r in rows if r.get('recomputed_fd') is not None]
    if fd:
        print(f'recomputed front_distance: min={min(fd):.3f} max={max(fd):.3f} m')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--csv', default=None)
    ap.add_argument('--estimator', choices=('band', 'median', 'p25'), default='band')
    ap.add_argument('--mode', choices=('auto', 'crop', 'full'), default='auto')
    args = ap.parse_args()
    bag = resolve_bag(args.bag)
    data = read_topics(bag, [CROP_DEPTH, CROP_CLASS, REC_RAW, REC_FD, REC_DEBUG,
                             DEPTH, DETS, MASKS, OBST, SCAN, TF_STATIC])
    have_crop = bool(data[CROP_DEPTH]) and bool(data[CROP_CLASS])
    have_full = bool(data[DEPTH]) and bool(data[DETS])
    mode = args.mode
    if mode == 'auto':
        mode = 'crop' if have_crop else ('full' if have_full else None)
    print(f'bag: {bag}')
    print('present: ' + ', '.join(f'{k} ({len(v)})' for k, v in data.items() if v))
    if mode is None or (mode == 'crop' and not have_crop) or (mode == 'full' and not have_full):
        print('CANNOT RECOMPUTE: the bag has neither the A5 ROI crops '
              f'({CROP_DEPTH} + {CROP_CLASS}) nor the depth image ({DEPTH}) with '
              f'detections. Recorded front_distance messages: {len(data[REC_FD])}.')
        return 2
    rows = (replay_crops(data, args.estimator, _params()) if mode == 'crop'
            else replay_full(data, args.estimator))
    print(f'mode: {mode}, estimator: {args.estimator}')
    summarize(rows)
    if args.csv:
        keys = sorted({k for r in rows for k in r})
        with open(args.csv, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: ('' if v is None else v) for k, v in r.items()})
        print(f'wrote {args.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
