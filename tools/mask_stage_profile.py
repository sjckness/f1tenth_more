#!/usr/bin/env python3
"""Per-stage timing of front_clearance_node's depth path, on a bag's own frames.

Usage (after `source install/setup.bash`):
    python3 tools/mask_stage_profile.py [--frames N] [--overlay] BAG

BAG is a bag directory or an archive run folder. Opened read-only (sqlite
`mode=ro`); nothing is published to a graph, and the node is constructed
in-process with its publishers replaced by sinks, on the isolated domain
scene_labels_replay sets up.

WHY. The node's own `ms` field says how long the whole scene-mask tick took --
82.6 ms median and 149 ms p95 on the car (corridor10m_20261015T171753) against
22 ms for the same frames replayed on an idle Orin -- but not WHERE it went.
This times each stage separately on the real frames so the bottleneck is a
measurement rather than a guess.

WHAT IT CANNOT TELL YOU. It runs on an idle machine against a node that ran
under full mission load, so every number here is a LOWER bound on the live
cost and the ratios between stages are what to read, not the absolutes. It
also cannot reproduce the input rates: the bag carries the throttled 5 Hz
depth copy, and only the frames the recorder kept.
"""
import argparse
import sys
import time
from pathlib import Path


WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / 'tools'))

import scene_labels_replay as slr  # noqa: E402
import front_phantom_replay as fpr  # noqa: E402
import corridor_wall_replay as cwr  # noqa: E402

import rclpy  # noqa: E402
from f1tenth_perception import inferred_floor, scene_mask, wall_fusion  # noqa: E402


def stats(values, scale=1.0):
    v = sorted(x * scale for x in values if x is not None)
    if not v:
        return {'n': 0}
    return {'n': len(v), 'median': round(v[len(v) // 2], 2),
            'p95': round(v[min(len(v) - 1, int(len(v) * 0.95))], 2),
            'max': round(v[-1], 2)}


class Timer:
    """Wall-clock stage accumulator. One name, one list of milliseconds."""

    def __init__(self):
        self.stages = {}

    def add(self, name, ms):
        self.stages.setdefault(name, []).append(ms)

    def wrap(self, name, fn):
        """Return fn with its wall time recorded under `name` on every call."""
        def timed(*a, **kw):
            t = time.perf_counter()
            try:
                return fn(*a, **kw)
            finally:
                self.add(name, (time.perf_counter() - t) * 1e3)
        return timed

    def report(self):
        return {k: stats(v) for k, v in self.stages.items()}


def profile(bag, frames=None, overlay=False):
    from f1tenth_perception.front_clearance_node import FrontClearanceNode
    # The same header-stamp-ordered reader the corridor replay uses, so "this
    # frame" means the same thing in both reports.
    src = cwr.Src(bag)
    timer = Timer()
    clock = [0.0]
    node = FrontClearanceNode(parameter_overrides=fpr.front_clearance_params())
    node._now_seconds = lambda: clock[0]
    caps = {}
    for attr in ('front_distance_pub', 'front_wall_pub', 'front_clearance_pub',
                 'front_blocked_pub', 'wall_debug_pub', 'scene_labels_pub',
                 'scene_debug_pub', 'stereo_wall_pub', 'unclassified_pub',
                 'front_distance_raw_pub', 'bg_pixel_count_pub',
                 'scene_mask_image_pub', 'wall_points_pub', 'scene_mask_stats_pub',
                 'stereo_columns_pub'):
        if getattr(node, attr, None) is not None:
            # cwr.Capture answers get_subscription_count 1, which is what turns
            # the overlay path on: whether it belongs on the critical path is
            # one of the questions, so it is a flag rather than an assumption.
            caps[attr] = cwr.Capture() if overlay else slr.Capture()
            setattr(node, attr, caps[attr])
    if not overlay:
        caps['scene_mask_image_pub'].get_subscription_count = lambda: 0

    # Stage boundaries. Each is wrapped where it is CALLED FROM, so the time is
    # attributed to the tick that called it and not to the module.
    scene_mask.classify = timer.wrap('classify', scene_mask.classify)
    scene_mask.wall_points = timer.wrap('wall_points', scene_mask.wall_points)
    scene_mask.overlay = timer.wrap('overlay_draw', scene_mask.overlay)
    scene_mask.project_to_image = timer.wrap('overlay_project',
                                             scene_mask.project_to_image)
    scene_mask.class_stats = timer.wrap('class_stats', scene_mask.class_stats)
    wall_fusion.column_ranges = timer.wrap('columns_percentile',
                                           wall_fusion.column_ranges)
    wall_fusion.column_extent = timer.wrap('columns_extent',
                                           wall_fusion.column_extent)
    inferred_floor.apply = timer.wrap('inferred_floor_apply', inferred_floor.apply)
    inferred_floor.effective_ranges = timer.wrap('inferred_floor_ranges',
                                                 inferred_floor.effective_ranges)
    node._scene_mask_tick = timer.wrap('TICK_total', node._scene_mask_tick)
    node._publish_stereo_columns = timer.wrap('publish_stereo_columns',
                                              node._publish_stereo_columns)
    node._publish_scene_mask_overlay = timer.wrap('publish_overlay',
                                                  node._publish_scene_mask_overlay)
    node._infer_floor = timer.wrap('inferred_floor_total', node._infer_floor)
    node._label_scene = timer.wrap('scene_labels_tick', node._label_scene)
    node._raw_object_mask = timer.wrap('object_mask', node._raw_object_mask)
    node._update_object_confidence = timer.wrap('object_confidence',
                                                node._update_object_confidence)

    for t, _r, m in src.get('/tf_static'):
        clock[0] = t
        node._tf_static_callback(m)
    for name in ('/zed2/zed_node/depth/camera_info', '/camera/camera_info'):
        if src.get(name):
            node._camera_info_callback(src.get(name)[0][2])
            break
    depth_topic = next((n for n in ('/zed2/zed_node/depth/depth_registered_5hz',
                                    '/zed2/zed_node/depth/depth_registered',
                                    '/camera/depth/image_raw') if src.get(n)), None)
    if depth_topic is None:
        raise SystemExit('no depth topic in this bag')

    feed = {'/camera/detections': node._detections_callback,
            '/camera/detection_masks': node._masks_callback,
            '/perception/obstacles_2d': node._obstacles_callback,
            '/camera/image_raw': node._image_callback,
            '/scan': node._scan_callback}
    events = [(t, 0, name, m) for name in feed for t, _r, m in src.get(name)]
    events += [(t, 1, depth_topic, m) for t, _r, m in src.get(depth_topic)]
    events.sort(key=lambda e: (e[0], e[1]))

    n = 0
    sizes = set()
    for t, _k, name, m in events:
        clock[0] = t
        if name != depth_topic:
            feed[name](m)
            continue
        node._next_scene_mask = -1e18
        t0 = time.perf_counter()
        node.depth_callback(m)
        timer.add('DEPTH_CALLBACK_total', (time.perf_counter() - t0) * 1e3)
        sizes.add((m.width, m.height))
        for cap in caps.values():
            cap.pop()
        n += 1
        if frames and n >= frames:
            break
    node.destroy_node()
    return n, sizes, timer


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', type=Path)
    ap.add_argument('--frames', type=int, default=0)
    ap.add_argument('--overlay', action='store_true',
                    help='render the debug overlay too (off by default, as on the car)')
    cfg = ap.parse_args()
    rclpy.init()
    try:
        n, sizes, timer = profile(cfg.bag, cfg.frames or None, cfg.overlay)
    finally:
        rclpy.shutdown()
    print(f'{n} depth frames at {sorted(sizes)}, overlay '
          f'{"ON" if cfg.overlay else "off"}')
    rows = timer.report()
    width = max(len(k) for k in rows)
    print(f'{"stage":{width}s} {"n":>5s} {"median":>8s} {"p95":>8s} {"max":>8s}')
    order = ['DEPTH_CALLBACK_total', 'TICK_total', 'classify', 'inferred_floor_total',
             'inferred_floor_ranges', 'inferred_floor_apply', 'wall_points',
             'publish_stereo_columns', 'columns_percentile', 'columns_extent',
             'class_stats', 'publish_overlay', 'overlay_project', 'overlay_draw',
             'object_mask', 'object_confidence', 'scene_labels_tick']
    for k in order + [k for k in rows if k not in order]:
        s = rows.get(k)
        if not s or not s['n']:
            continue
        print(f'{k:{width}s} {s["n"]:5d} {s["median"]:8.2f} {s["p95"]:8.2f} '
              f'{s["max"]:8.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
