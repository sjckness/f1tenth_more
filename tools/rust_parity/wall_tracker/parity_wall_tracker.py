#!/usr/bin/env python3
"""R6 parity: lidar_front_wall_node with wall_tracker_impl py and rust, in lockstep, on bags.

Usage (after `source install/setup.bash`):
    python3 tools/rust_parity/wall_tracker/parity_wall_tracker.py \
        [--cycles car|every] [--stereo rec|full] [--doors] [--json OUT] BAG [BAG ...]

BAG is a path or a replay_common.BAGS key. Two LidarFrontWallNode instances are
built in-process (corridor_wall_replay's WallReplay, isolated domain): one with
wall_tracker_impl py, one with rust. Every cycle feeds both the same inputs in
the same order, then compares, BIT FOR BIT:

  messages   every message the node published this cycle, field by field
             (message_to_ordereddict; NOT the CDR bytes: rmw leaves the
             alignment padding uninitialised, so one message serializes to
             different bytes twice in a row): /perception/walls, /perception/walls/markers,
             /perception/wall_observations, /perception/lidar_front_wall,
             /perception/lidar_front_wall_virtual, /perception/virtual_wall,
             /perception/wall_planes, /perception/doors (--doors);
  debug      /perception/virtual_wall/debug as JSON, timing keys removed
             (tracker_ms, rect_ms, door_ms), with wall_debug_tracks ON so the
             per-track payload (roles, verdicts, front reasons) is in it;
  state      the WHOLE tracker state: every field of every track (x, P,
             extent, deques, obs_ext, roles, verify, strip, ...) and the
             tracker attributes (ids, stamps, odometer, counters,
             rejections, strip reading, front_status).

A float is "identical" when its bits are (NaN == NaN counts); the report also
gives the largest absolute difference, which is the 1e-9 gate's number. The
rust node runs with F1TENTH_WALL_TRACKER_VERIFY_SYNC=1: every library call also
checks its Python mirror against the library state.
"""

import argparse
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

os.environ.setdefault('WALL_DEBUG_TRACKS', '1')
os.environ['F1TENTH_WALL_TRACKER_VERIFY_SYNC'] = '1'
sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_common as rc  # noqa: E402

import numpy as np  # noqa: E402
from rosidl_runtime_py import message_to_ordereddict  # noqa: E402

TIMING_KEYS = ('tracker_ms', 'rect_ms', 'door_ms')
MSG_CAPS = ('walls_pub', 'walls_markers_pub', 'observations_pub', 'fit_pub', 'estimate_pub',
            'virtual_pub', 'planes_pub', 'doors_pub')
TRACK_FIELDS = ('id', 'hits', 't0', 't1', 'born', 'last_seen', 'ever_confirmed', 'state',
                'source', 'has_lidar', 'has_both', 'z_min', 'z_max', 'height_measured',
                'confidence', 'role', 'verify', 'glass', 'rect_coverage', 'rect_support',
                'rect_beyond_share', 'rect_informative', 'rect_reject_streak', 'miss',
                'odo_at_last_seen', 'occluded', 'strip_verdict', 'strip_pred_m', 'strip_miss',
                'contradicted')
TRACKER_FIELDS = ('_next_id', '_last_stamp', '_last_yaw', '_odometer', 'n_short_dropped',
                  'n_negative_killed', 'n_gap_rejected', 'n_merge_gap_refused',
                  'n_stereo_near_rejected', 'n_camera_rejected', 'camera_fresh')


def _f(v):
    return float(v)


def track_state(t):
    d = {k: getattr(t, k) for k in TRACK_FIELDS}
    for k in ('t0', 't1', 'born', 'last_seen', 'z_min', 'z_max', 'confidence', 'rect_coverage',
              'rect_support', 'rect_beyond_share', 'odo_at_last_seen', 'strip_pred_m'):
        d[k] = _f(d[k])
    d['x'] = [_f(v) for v in np.asarray(t.x).ravel()]
    d['P'] = [_f(v) for v in np.asarray(t.P).ravel()]
    d['confirm_obs'] = ([None if c is None else [_f(c[0]), _f(c[1])] for c in t.confirm_obs],
                        t.confirm_obs.maxlen)
    d['lidar_hist'] = ([bool(v) for v in t.lidar_hist], t.lidar_hist.maxlen)
    d['conf_hist'] = ([_f(v) for v in t.conf_hist], t.conf_hist.maxlen)
    d['obs_ext'] = ([[_f(s), [_f(v) for v in np.asarray(p).ravel()]] for s, p in t.obs_ext],
                    t.obs_ext.maxlen)
    return d


def tracker_state(trk):
    if trk is None:
        return None
    d = {k: getattr(trk, k) for k in TRACKER_FIELDS}
    d['_last_car_t'] = None if trk._last_car_t is None else [_f(v) for v in trk._last_car_t]
    d['laser_xy'] = None if trk.laser_xy is None else [_f(v) for v in trk.laser_xy]
    d['camera_xy'] = None if trk.camera_xy is None else [_f(v) for v in trk.camera_xy]
    d['rejections'] = [list(r) for r in trk.rejections]
    s = trk.strip
    d['strip'] = [s.range_m, s.n_returns, s.n_cut, s.n_no_return]
    fs = trk.front_status
    d['front_status'] = {'id': fs['id'], 'reason': fs['reason'],
                         'distance_m': _f(fs['distance_m']),
                         'rejects': sorted((int(k), v) for k, v in fs['rejects'].items())}
    d['tracks'] = [track_state(t) for t in trk.tracks]
    return d


class Diff:
    """Recursive comparison: bit-identical floats, else the largest |a - b|."""

    def __init__(self):
        self.paths = []
        self.max_abs = 0.0
        self.bit_only = 0    # equal values with different bits (signed zeros)

    def cmp(self, a, b, path='$'):
        if isinstance(a, float) or isinstance(b, float):
            if not (isinstance(a, (int, float)) and isinstance(b, (int, float))) \
                    or isinstance(a, bool) or isinstance(b, bool):
                self.paths.append((path, repr(a), repr(b)))
                return
            a, b = float(a), float(b)
            if math.isnan(a) and math.isnan(b):
                return
            if struct.pack('<d', a) == struct.pack('<d', b):
                return
            if a == b:
                self.bit_only += 1
                return
            if math.isfinite(a) and math.isfinite(b):
                self.max_abs = max(self.max_abs, abs(a - b))
            else:
                self.max_abs = math.inf
            self.paths.append((path, repr(a), repr(b)))
            return
        if isinstance(a, dict) and isinstance(b, dict):
            if set(a) != set(b):
                self.paths.append((path + '{keys}', sorted(a), sorted(b)))
            for k in a:
                if k in b:
                    self.cmp(a[k], b[k], f'{path}.{k}')
            return
        if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
            if len(a) != len(b):
                self.paths.append((path + '[len]', len(a), len(b)))
            for k, (x, y) in enumerate(zip(a, b)):
                self.cmp(x, y, f'{path}[{k}]')
            return
        if a != b:
            self.paths.append((path, repr(a)[:200], repr(b)[:200]))


def debug_dict(caps):
    items = caps.get('virtual_debug_pub') or []
    out = []
    for m in items:
        d = json.loads(m.data)
        for k in TIMING_KEYS:
            d.pop(k, None)
        out.append(d)
    return out


def run(name, cycles_mode, stereo, doors, limit=None):
    src = rc.load(name, full=(stereo == 'full'))
    rows = rc.mask_rows(src) if stereo == 'full' else None
    cycles = rc.cycles_of(src, cycles_mode)
    if limit:
        cycles = cycles[:limit]
    base = {'door_detection_enabled': True} if doors else {}
    exact = cycles_mode == 'every'
    py = rc.Replay(src, stereo, rows, dict(base, wall_tracker_impl='py'), exact_odom=exact)
    rs = rc.Replay(src, stereo, rows, dict(base, wall_tracker_impl='rust'), exact_odom=exact)
    assert type(py.node.wall_tracker).__name__ == 'MultiWallTracker', type(py.node.wall_tracker)
    assert type(rs.node.wall_tracker).__name__ == 'NativeMultiWallTracker', \
        type(rs.node.wall_tracker)
    res = {'bag': str(rc.bag_path(name)), 'cycles_mode': cycles_mode, 'stereo': stereo,
           'doors': doors, 'cycles': len(cycles), 'stepped': 0, 'cycles_with_diff': 0,
           'state_diffs': 0, 'message_diffs': 0, 'debug_diffs': 0, 'max_abs_diff': 0.0,
           'bit_only_equal': 0, 'first_diffs': [], 'messages_compared': 0,
           'tracks_compared': 0, 'max_tracks': 0, 'confirmed_track_steps': 0,
           'front_steps': 0, 'rect_cycles': 0, 'door_cycles': 0, 'doors_seen': 0,
           'roles': {}, 'tracker_ms': {'py': [], 'rust': []}}
    t_wall = time.time()
    for k, cycle in enumerate(cycles):
        _ms_py, cp = py.step(cycle)
        _ms_rs, cr = rs.step(cycle)
        diff = Diff()
        # messages, serialized
        for cap in MSG_CAPS:
            a, b = cp.get(cap) or [], cr.get(cap) or []
            if len(a) != len(b):
                diff.paths.append((f'msg.{cap}[len]', len(a), len(b)))
                res['message_diffs'] += 1
                continue
            for ma, mb in zip(a, b):
                res['messages_compared'] += 1
                n0 = len(diff.paths)
                diff.cmp(message_to_ordereddict(ma), message_to_ordereddict(mb), f'msg.{cap}')
                if len(diff.paths) > n0:
                    res['message_diffs'] += 1
        da, db = debug_dict(cp), debug_dict(cr)
        nd0 = len(diff.paths)
        diff.cmp(da, db, 'debug')
        if len(diff.paths) > nd0:
            res['debug_diffs'] += 1
        sa, sb = tracker_state(py.node.wall_tracker), tracker_state(rs.node.wall_tracker)
        ns0 = len(diff.paths)
        diff.cmp(sa, sb, 'state')
        if len(diff.paths) > ns0:
            res['state_diffs'] += 1
        res['max_abs_diff'] = max(res['max_abs_diff'], diff.max_abs)
        res['bit_only_equal'] += diff.bit_only
        if diff.paths:
            res['cycles_with_diff'] += 1
            if len(res['first_diffs']) < 12:
                res['first_diffs'].append({'cycle': k, 'stamp': cycle['stamp'],
                                           'paths': [list(map(str, p)) for p in diff.paths[:8]]})
        if da:
            d = da[-1]
            if d.get('front_reason') != 'no_pose':
                res['stepped'] += 1
                for impl, dd in (('py', cp), ('rust', cr)):
                    items = dd.get('virtual_debug_pub') or []
                    if items:
                        tms = json.loads(items[-1].data).get('tracker_ms')
                        if tms is not None:
                            res['tracker_ms'][impl].append(float(tms))
            if d.get('front_id') is not None:
                res['front_steps'] += 1
            if d.get('rect_informative'):
                res['rect_cycles'] += 1
            if d.get('doors') is not None:
                res['door_cycles'] += 1
                res['doors_seen'] += int(d.get('doors') or 0)
        if sa is not None:
            n = len(sa['tracks'])
            res['tracks_compared'] += n
            res['max_tracks'] = max(res['max_tracks'], n)
            res['confirmed_track_steps'] += sum(1 for t in sa['tracks'] if t['ever_confirmed'])
            for t in sa['tracks']:
                res['roles'][t['role']] = res['roles'].get(t['role'], 0) + 1
    py.close()
    rs.close()
    rc.cwr.close_ro_connections()
    for impl in ('py', 'rust'):
        v = res['tracker_ms'][impl]
        res['tracker_ms'][impl] = {'n': len(v), 'p50': round(rc.percentile(v, 50), 4),
                                   'p95': round(rc.percentile(v, 95), 4)} if v else {'n': 0}
    res['wall_s'] = round(time.time() - t_wall, 1)
    res['identical'] = res['cycles_with_diff'] == 0
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--cycles', choices=('car', 'every'), default='car')
    ap.add_argument('--stereo', choices=('rec', 'full'), default='rec')
    ap.add_argument('--doors', action='store_true', help='door_detection_enabled true')
    ap.add_argument('--limit', type=int, default=None)
    ap.add_argument('--json', type=Path, default=None)
    cfg = ap.parse_args()
    rc.rclpy_init()
    out = []
    for bag in cfg.bags:
        print(f'== {bag} ({cfg.cycles}, {cfg.stereo}, doors {cfg.doors})', flush=True)
        r = run(bag, cfg.cycles, cfg.stereo, cfg.doors, cfg.limit)
        print(f'   {r["cycles"]} cycles, {r["stepped"]} tracker steps, {r["tracks_compared"]} '
              f'track states, {r["messages_compared"]} messages: '
              f'{"IDENTICAL" if r["identical"] else "DIFFERENT"} ({r["cycles_with_diff"]} cycles '
              f'differ; state {r["state_diffs"]}, messages {r["message_diffs"]}, debug '
              f'{r["debug_diffs"]}; max |diff| {r["max_abs_diff"]:.3g}; bit-only {r["bit_only_equal"]})'
              f'; tracker_ms py {r["tracker_ms"]["py"]} rust {r["tracker_ms"]["rust"]}; '
              f'{r["wall_s"]} s', flush=True)
        for fd in r['first_diffs'][:3]:
            print(f'   first diff at cycle {fd["cycle"]}: {fd["paths"][:4]}', flush=True)
        out.append(r)
    if cfg.json:
        cfg.json.parent.mkdir(parents=True, exist_ok=True)
        cfg.json.write_text(json.dumps({'runs': out}, indent=1, default=str))
        print(f'wrote {cfg.json}')
    return 0 if all(r['identical'] for r in out) else 1


if __name__ == '__main__':
    sys.exit(main())
