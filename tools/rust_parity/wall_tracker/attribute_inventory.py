#!/usr/bin/env python3
"""R6.0: every tracker attribute lidar_front_wall_node.py (and what it calls) reads.

Usage (after `source install/setup.bash`):
    python3 tools/rust_parity/wall_tracker/attribute_inventory.py [--json OUT] BAG [BAG ...]

Two views, because each misses what the other sees:

  runtime   a replay (wall_debug_tracks ON, door detection ON, so every reader
            runs) with Track.__getattribute__ and
            MultiWallTracker.__getattribute__ instrumented: every attribute
            read from OUTSIDE multi_wall_tracker.py, by reading file and
            function, with a count. That is the surface a Rust tracker has to
            keep presenting as Python objects.
  static    an AST scan of lidar_front_wall_node.py: every `self.wall_tracker.X`,
            every attribute read on a track variable, and every `mwt.f(...)` call
            (the module-level helpers it hands tracks to).
"""

import argparse
import ast
import collections
import json
import os
import sys
from pathlib import Path

os.environ['WALL_DEBUG_TRACKS'] = '1'
sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_common as rc  # noqa: E402

from f1tenth_perception import multi_wall_tracker as mwt  # noqa: E402

NODE = rc.WS / 'src/f1tenth_perception/f1tenth_perception/lidar_front_wall_node.py'
TRACK_VARS = {'trk', 't', 'track', 'front', 'best', 'previous_track', 'tr'}


def runtime(bags, cycles_mode):
    reads = collections.Counter()
    own = 'multi_wall_tracker.py'

    def recorder(kind):
        def ga(self, name):
            if not name.startswith('__'):
                f = sys._getframe(1)
                fn = os.path.basename(f.f_code.co_filename)
                if fn != own:
                    reads[(kind, name, fn, f.f_code.co_name)] += 1
            return object.__getattribute__(self, name)
        return ga

    rc.rclpy_init()
    mwt.Track.__getattribute__ = recorder('Track')
    mwt.MultiWallTracker.__getattribute__ = recorder('MultiWallTracker')
    try:
        for bag in bags:
            src = rc.load(bag)
            rp = rc.Replay(src, overrides={'door_detection_enabled': True,
                                           'wall_tracker_impl': 'py'})
            for cycle in rc.cycles_of(src, cycles_mode):
                rp.step(cycle)
            rp.close()
            rc.cwr.close_ro_connections()
    finally:
        del mwt.Track.__getattribute__
        del mwt.MultiWallTracker.__getattribute__
    return reads


def static():
    tree = ast.parse(NODE.read_text())
    tracker_attrs, track_attrs, mwt_calls = collections.Counter(), collections.Counter(), \
        collections.Counter()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            v = node.value
            if (isinstance(v, ast.Attribute) and v.attr == 'wall_tracker'
                    and isinstance(v.value, ast.Name) and v.value.id == 'self'):
                tracker_attrs[node.attr] += 1
            elif isinstance(v, ast.Name) and v.id in TRACK_VARS:
                track_attrs[node.attr] += 1
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            f = node.func
            if isinstance(f.value, ast.Name) and f.value.id == 'mwt':
                mwt_calls[f.attr] += 1
    return tracker_attrs, track_attrs, mwt_calls


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--cycles', choices=('car', 'every'), default='car')
    ap.add_argument('--json', type=Path, default=None)
    cfg = ap.parse_args()
    reads = runtime(cfg.bags, cfg.cycles)
    tracker_attrs, track_attrs, mwt_calls = static()
    by_kind = collections.defaultdict(lambda: collections.defaultdict(dict))
    for (kind, name, fn, func), n in sorted(reads.items()):
        by_kind[kind][name][f'{fn}:{func}'] = n
    print('== runtime reads from outside multi_wall_tracker.py')
    for kind in ('MultiWallTracker', 'Track'):
        print(f'-- {kind}: {len(by_kind[kind])} attributes')
        for name in sorted(by_kind[kind]):
            where = by_kind[kind][name]
            print(f'   {name:24s} {sum(where.values()):8d}  ' +
                  ', '.join(f'{k} {v}' for k, v in sorted(where.items(), key=lambda kv: -kv[1])))
    print('== static (lidar_front_wall_node.py)')
    print('   self.wall_tracker.X:', dict(sorted(tracker_attrs.items())))
    print('   track.X:', dict(sorted(track_attrs.items())))
    print('   mwt.f(...):', dict(sorted(mwt_calls.items())))
    if cfg.json:
        cfg.json.parent.mkdir(parents=True, exist_ok=True)
        cfg.json.write_text(json.dumps({
            'bags': [str(rc.bag_path(b)) for b in cfg.bags], 'cycles': cfg.cycles,
            'runtime': {k: {n: dict(w) for n, w in v.items()} for k, v in by_kind.items()},
            'static': {'tracker_attrs': dict(tracker_attrs), 'track_attrs': dict(track_attrs),
                       'mwt_calls': dict(mwt_calls)}}, indent=1))
        print(f'wrote {cfg.json}')


if __name__ == '__main__':
    main()
