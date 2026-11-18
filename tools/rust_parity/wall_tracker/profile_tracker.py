#!/usr/bin/env python3
"""R6.0: where lidar_front_wall_node's wall tracker cycle spends its time, on car bags.

Usage (after `source install/setup.bash`):
    python3 tools/rust_parity/wall_tracker/profile_tracker.py \
        [--cycles car|every] [--stereo rec|full] [--repeat 3] [--impl py|rust] \
        [--json OUT.json] BAG [BAG ...]

BAG is a path or a replay_common.BAGS key (c1015, c1016, c1019, obst1019,
potted1435, person1455, goto1550, mission1021).

TWO MEASUREMENTS PER BAG.

  totals      the node's own tracker_ms (from /perception/virtual_wall/debug,
              the same perf_counter span the car logs: extraction to
              front_track, publishing excluded) and the whole /scan callback,
              uninstrumented. The replay runs --repeat times and each cycle
              keeps its MINIMUM over the repeats (every repeat is the same
              deterministic sequence of states, so the minimum is the cost
              with the least interference).
  stages      one more replay with a timer around every stage below; per
              cycle the EXCLUSIVE time of each stage (its own time, children
              subtracted), then p50 / p95 / mean over the cycles. The timers
              cost ~1 us per call, which shows in the many-call stages
              (_gate, _open_length); `overhead_ms` is the instrumented
              tracker_ms minus the uninstrumented one.

THE CYCLE AS THE CAR RUNS IT: wall_debug_tracks off (WALL_DEBUG_TRACKS=0, the
car's default), door detection at its stack default (off), the rectangle
verification on whenever the cycle has a class mask + depth pair (recorded, or
recomputed with --stereo full).
"""

import argparse
import collections
import json
import math
import os
import platform
import sys
import time
from pathlib import Path

os.environ.setdefault('WALL_DEBUG_TRACKS', '0')
sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_common as rc  # noqa: E402

from f1tenth_perception import multi_wall_tracker as mwt  # noqa: E402
from f1tenth_perception import wall_fusion  # noqa: E402

#: Node methods (instance-patched), node-level stages.
NODE_STAGES = ('_publish_observations', '_cloud_columns', '_verify_tracks', '_publish_walls',
               '_count_objects_behind_walls', '_detect_doors', '_track_payload',
               '_publish_virtual', '_publish', '_sectors', '_track_walls')
#: Module functions looked up at call time by the node or the tracker.
MWT_FUNCS = ('extract_lines', 'scan_in_base_link', 'observation_line', 'linear_sum_assignment',
             'segment_in_base_link', 'strip_returns', 'role_with_hysteresis', 'ray_blocked_by',
             'to_wall_line', 'line_to_wall_line', 'track_in_base_link', 'front_distances',
             'propagate_objects')
FUSION_FUNCS = ('cross_check_profile', 'observations_from_lines', 'stereo_only_observations',
                'verify_rectangles', 'confidence', 'verify_step')
#: Tracker methods (instance-patched).
TRACKER_METHODS = ('step', 'update_roles', 'published', 'front_track', 'apply_rect_verdicts',
                   '_refusal', '_wall_spans', '_chi2_between', '_predict', '_gate', '_update',
                   '_absorb', '_spawn', '_promote', '_refresh_extent', '_merge',
                   '_apply_negative_evidence', 'negative_evidence', '_expired',
                   '_apply_strip_evidence', 'update_occlusion', '_open_length', 'front_reason',
                   'confirm_reason')


class Prof:
    """Inclusive / exclusive timers with a stack, per cycle."""

    def __init__(self):
        self.stack = []
        self.excl = collections.defaultdict(float)
        self.incl = collections.defaultdict(float)
        self.calls = collections.Counter()

    def reset(self):
        self.excl.clear()
        self.incl.clear()
        self.calls.clear()

    def wrap(self, name, fn):
        prof = self

        def timed(*a, **kw):
            t0 = time.perf_counter()
            prof.stack.append(0.0)
            try:
                return fn(*a, **kw)
            finally:
                dt = time.perf_counter() - t0
                child = prof.stack.pop()
                prof.incl[name] += dt
                prof.excl[name] += dt - child
                prof.calls[name] += 1
                if prof.stack:
                    prof.stack[-1] += dt
        timed.__wrapped__ = fn
        return timed


def instrument(node, prof):
    """Patch timers in; returns an undo callable."""
    undo = []
    for name in NODE_STAGES:
        if hasattr(node, name):
            setattr(node, name, prof.wrap('node.' + name, getattr(node, name)))
    for mod, names, tag in ((mwt, MWT_FUNCS, 'mwt.'), (wall_fusion, FUSION_FUNCS, 'fusion.')):
        for name in names:
            original = getattr(mod, name)
            setattr(mod, name, prof.wrap(tag + name, original))
            undo.append((mod, name, original))
    tracker = node.wall_tracker
    if tracker is not None:
        for name in TRACKER_METHODS:
            if hasattr(tracker, name):
                setattr(tracker, name, prof.wrap('tracker.' + name, getattr(tracker, name)))

    def restore():
        for mod, name, original in undo:
            setattr(mod, name, original)
    return restore


#: --cycles every feeds the odometry exactly (replay_common.Replay).
EXACT_ODOM = [False]


def debug_payload(caps):
    items = caps.get('virtual_debug_pub') or []
    return json.loads(items[-1].data) if items else {}


def stepped(d):
    """Did the tracker really step on this cycle? tracker_ms is the LAST step's
    value, so a no-pose cycle (front_reason 'no_pose') would repeat it."""
    return d.get('front_reason') != 'no_pose'


def run_totals(src, cycles, stereo, rows, overrides, repeat):
    """Per cycle min over `repeat` replays of (tracker_ms, scan_ms)."""
    best_tracker = [math.inf] * len(cycles)
    best_scan = [math.inf] * len(cycles)
    ran = [False] * len(cycles)
    for _ in range(repeat):
        rp = rc.Replay(src, stereo=stereo, replayed=rows, overrides=overrides,
                       exact_odom=EXACT_ODOM[0])
        for k, cycle in enumerate(cycles):
            ms, caps = rp.step(cycle)
            d = debug_payload(caps)
            tms = d.get('tracker_ms')
            if tms is None or not stepped(d):
                continue
            ran[k] = True
            best_tracker[k] = min(best_tracker[k], float(tms))
            best_scan[k] = min(best_scan[k], ms)
        rp.close()
    return ([v for v, r in zip(best_tracker, ran) if r],
            [v for v, r in zip(best_scan, ran) if r])


def run_stages(src, cycles, stereo, rows, overrides):
    """Per cycle {stage: exclusive ms} and the instrumented tracker_ms."""
    rp = rc.Replay(src, stereo=stereo, replayed=rows, overrides=overrides,
                       exact_odom=EXACT_ODOM[0])
    prof = Prof()
    restore = instrument(rp.node, prof)
    per_cycle, tracker_ms, calls = [], [], collections.Counter()
    try:
        for cycle in cycles:
            prof.reset()
            _ms, caps = rp.step(cycle)
            d = debug_payload(caps)
            if d.get('tracker_ms') is None or not stepped(d):
                continue
            tracker_ms.append(float(d['tracker_ms']))
            per_cycle.append({k: v * 1e3 for k, v in prof.excl.items()})
            calls.update(prof.calls)
    finally:
        restore()
        rp.close()
    return per_cycle, tracker_ms, calls


def summarize(values):
    v = [x for x in values if x is not None and math.isfinite(x)]
    if not v:
        return {'n': 0}
    return {'n': len(v), 'p50': round(rc.percentile(v, 50), 4),
            'p95': round(rc.percentile(v, 95), 4), 'mean': round(sum(v) / len(v), 4),
            'max': round(max(v), 4)}


def profile_bag(name, cycles_mode, stereo, repeat, overrides):
    src = rc.load(name, full=(stereo == 'full'))
    rows = rc.mask_rows(src) if stereo == 'full' else None
    cycles = rc.cycles_of(src, cycles_mode)
    t0 = time.time()
    tracker, scan = run_totals(src, cycles, stereo, rows, overrides, repeat)
    per_cycle, tracker_instr, calls = run_stages(src, cycles, stereo, rows, overrides)
    stages = sorted({k for c in per_cycle for k in c})
    stage_stats = {}
    for k in stages:
        vals = [c.get(k, 0.0) for c in per_cycle]
        s = summarize(vals)
        s['calls_per_cycle'] = round(calls[k] / max(1, len(per_cycle)), 2)
        stage_stats[k] = s
    out = {
        'bag': str(rc.bag_path(name)), 'cycles_mode': cycles_mode, 'stereo': stereo,
        'cycles': len(cycles), 'cycles_with_tracker': len(tracker), 'repeat': repeat,
        'overrides': overrides or {},
        'tracker_ms': summarize(tracker), 'scan_callback_ms': summarize(scan),
        'tracker_ms_instrumented': summarize(tracker_instr),
        'overhead_ms_p50': round(rc.percentile(tracker_instr, 50)
                                 - rc.percentile(tracker, 50), 4) if tracker else None,
        'stages_exclusive_ms': stage_stats,
        'rect_cycles': sum(1 for c in per_cycle if c.get('fusion.verify_rectangles')),
        'wall_s': round(time.time() - t0, 1),
    }
    rc.cwr.close_ro_connections()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--cycles', choices=('car', 'every'), default='car')
    ap.add_argument('--stereo', choices=('rec', 'full'), default='rec')
    ap.add_argument('--repeat', type=int, default=3)
    ap.add_argument('--impl', choices=('py', 'rust'), default=None,
                    help='wall_tracker_impl override (default: the stack default)')
    ap.add_argument('--json', type=Path, default=None)
    cfg = ap.parse_args()
    overrides = {'wall_tracker_impl': cfg.impl} if cfg.impl else None
    results = []
    EXACT_ODOM[0] = cfg.cycles == 'every'
    rc.rclpy_init()
    for bag in cfg.bags:
        print(f'== {bag} ({cfg.cycles}, {cfg.stereo}, impl {cfg.impl or "default"})', flush=True)
        r = profile_bag(bag, cfg.cycles, cfg.stereo, cfg.repeat, overrides)
        print(f'   {r["cycles_with_tracker"]} tracker cycles; tracker_ms {r["tracker_ms"]}; '
              f'instrumented {r["tracker_ms_instrumented"]}; {r["wall_s"]} s', flush=True)
        top = sorted(r['stages_exclusive_ms'].items(), key=lambda kv: -kv[1].get('mean', 0))
        for k, s in top[:14]:
            print(f'     {k:42s} p50 {s["p50"]:8.4f} p95 {s["p95"]:8.4f} mean {s["mean"]:8.4f}'
                  f'  calls/cycle {s["calls_per_cycle"]}')
        results.append(r)
    doc = {'host': platform.node(), 'cpu': platform.processor(), 'python': sys.version.split()[0],
           'loadavg': os.getloadavg(), 'results': results}
    if cfg.json:
        cfg.json.parent.mkdir(parents=True, exist_ok=True)
        cfg.json.write_text(json.dumps(doc, indent=1))
        print(f'wrote {cfg.json}')


if __name__ == '__main__':
    main()
