#!/usr/bin/env python3
"""Phase 2 follow-up: freeze the costmap_boundary inputs used by
verify_costmap_boundary_logic.py into a ROS-free .npz.

verify_costmap_boundary_logic.py compared Thor-computed extractions against
what the Orin (Humble) published, so a NumPy/libm difference between the two
platforms and an imperfect bag-based reconstruction of the node's cached
inputs would both show up the same way: as a small large-error tail. This
script writes down, for every tick, exactly the (map grid + metadata, pose,
parameters) that script passed to extract_boundary_constraints(), so
run_boundary_frozen.py can replay them bit-identically on both platforms.

Writes (all under output/phase2/):
  boundary_frozen_inputs.npz    the frozen inputs (run_boundary_frozen.py's input)
  boundary_humble_recorded.npz  what Humble actually published at each tick,
                                for compare_boundary_frozen.py --humble

Before writing, checks the freeze is lossless: every tick is run once
straight from the bag messages (verify_costmap_boundary_logic.reconstruct)
and once from the frozen arrays (run_boundary_frozen.run), and the two must
agree bit for bit.
"""
import bisect
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
import run_boundary_frozen as frozen  # noqa: E402
import verify_costmap_boundary_logic as v  # noqa: E402

OUT_DIR = REPO / 'output/phase2'
CLEARANCE_MATCH_NS = int(0.05 * 1e9)  # same window verify_costmap_boundary_logic.py uses


def main():
    maps = v.load_with_recv('/slam/map')
    poses = v.load_with_recv('/ekf_global/odometry/filtered')
    boundaries = v.load_with_recv('/costmap/boundaries')
    clearances = v.load_with_recv('/costmap/front_clearance')
    map_times = [t for t, _ in maps]
    pose_times = [t for t, _ in poses]
    clearance_times = [t for t, _ in clearances]

    ticks = []    # (bag_index, t_out, map_idx, pose_idx)
    skipped = []
    for bi, (t_out, _msg) in enumerate(boundaries):
        mi = bisect.bisect_right(map_times, t_out) - 1
        pi = bisect.bisect_right(pose_times, t_out) - 1
        if mi < 0 or pi < 0:
            skipped.append(bi)
            continue
        ticks.append((bi, t_out, mi, pi))

    used_maps = sorted({mi for _, _, mi, _ in ticks})
    local = {mi: k for k, mi in enumerate(used_maps)}
    map_chunks, offsets = [], [0]
    for mi in used_maps:
        d = np.frombuffer(bytes(maps[mi][1].data), dtype=np.int8)
        assert d.size == maps[mi][1].info.width * maps[mi][1].info.height
        map_chunks.append(d)
        offsets.append(offsets[-1] + d.size)

    def mfield(f):
        return np.array([f(maps[mi][1]) for mi in used_maps])

    inp = {
        'param_front_facing_max_rad': np.float64(v.FRONT_FACING_MAX_RAD),
        'param_side_window_min_rad': np.float64(v.SIDE_WINDOW_MIN_RAD),
        'param_side_window_max_rad': np.float64(v.SIDE_WINDOW_MAX_RAD),
        'param_occupied_threshold': np.int64(v.OCCUPIED_THRESHOLD),
        'param_max_range_m': np.float64(v.MAX_RANGE_M),
        'map_data': np.concatenate(map_chunks),
        'map_offsets': np.array(offsets, dtype=np.int64),
        'map_width': mfield(lambda m: m.info.width).astype(np.int64),
        'map_height': mfield(lambda m: m.info.height).astype(np.int64),
        'map_resolution': mfield(lambda m: m.info.resolution).astype(np.float64),
        'map_origin_x': mfield(lambda m: m.info.origin.position.x).astype(np.float64),
        'map_origin_y': mfield(lambda m: m.info.origin.position.y).astype(np.float64),
        'map_recv_ns': np.array([maps[mi][0] for mi in used_maps], dtype=np.int64),
        'map_bag_index': np.array(used_maps, dtype=np.int64),
        'tick_bag_index': np.array([t[0] for t in ticks], dtype=np.int64),
        'tick_out_recv_ns': np.array([t[1] for t in ticks], dtype=np.int64),
        'tick_map_idx': np.array([local[t[2]] for t in ticks], dtype=np.int64),
        'tick_pose_bag_index': np.array([t[3] for t in ticks], dtype=np.int64),
        'tick_pose_recv_ns': np.array([pose_times[t[3]] for t in ticks], dtype=np.int64),
        'tick_pose_x': np.array([poses[t[3]][1].pose.pose.position.x for t in ticks]),
        'tick_pose_y': np.array([poses[t[3]][1].pose.pose.position.y for t in ticks]),
        'tick_pose_quat_xyzw': np.array([
            [q.x, q.y, q.z, q.w] for q in
            (poses[t[3]][1].pose.pose.orientation for t in ticks)]),
        'n_boundaries_total': np.int64(len(boundaries)),
        'skipped_bag_index': np.array(skipped, dtype=np.int64),
        'source_bag': np.array(v.BAG),
    }

    # --- lossless check: bag-message path vs frozen-array path, bit for bit
    out = frozen.run(inp, frozen.load_module(str(REPO / frozen.MODULE_REL)))
    n_bad = 0
    for i, (bi, t_out, mi, pi) in enumerate(ticks):
        cons, clearance = v.reconstruct(maps[mi][1], poses[pi][1])
        frz = [(out[d + '_nx'][i], out[d + '_ny'][i], out[d + '_off'][i])
               for d in frozen.DIRECTIONS if out[d + '_present'][i]]
        same = (len(cons) == len(frz)
                and all(np.float64(a).tobytes() == np.float64(b).tobytes()
                        for c, f in zip(cons, frz) for a, b in zip(c, f))
                and np.float64(clearance).tobytes() == out['front_clearance'][i].tobytes())
        if not same:
            n_bad += 1
            print('LOSSY tick bag_index=%d: bag=%r frozen=%r' % (bi, cons, frz))
    if n_bad:
        sys.exit('freeze is not lossless (%d ticks differ) -- not writing' % n_bad)
    print('lossless check: %d/%d ticks bit-identical (bag path vs frozen path)'
          % (len(ticks), len(ticks)))

    # --- what Humble published at each frozen tick
    n = len(ticks)
    rec_n = np.zeros(n, dtype=np.int64)
    rec = np.full((n, 3, 3), np.nan)
    rec_clear = np.full(n, np.nan)
    for i, (bi, t_out, _, _) in enumerate(ticks):
        cs = boundaries[bi][1].constraints
        rec_n[i] = len(cs)
        for j, c in enumerate(cs):
            rec[i, j] = (c.normal[0], c.normal[1], c.offset)
        ci = bisect.bisect_left(clearance_times, t_out)
        cand = [k for k in (ci - 1, ci) if 0 <= k < len(clearance_times)]
        if cand:
            best = min(cand, key=lambda k: abs(clearance_times[k] - t_out))
            if abs(clearance_times[best] - t_out) < CLEARANCE_MATCH_NS:
                rec_clear[i] = clearances[best][1].data

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT_DIR / 'boundary_frozen_inputs.npz', **inp)
    np.savez_compressed(
        OUT_DIR / 'boundary_humble_recorded.npz',
        tick_bag_index=inp['tick_bag_index'], rec_n=rec_n,
        rec_nx_ny_off=rec, rec_front_clearance=rec_clear)
    print('ticks frozen: %d of %d /costmap/boundaries messages (%d skipped: no map/pose cached yet)'
          % (n, len(boundaries), len(skipped)))
    print('unique maps:  %d (%d cells total)' % (len(used_maps), offsets[-1]))
    print('wrote %s' % (OUT_DIR / 'boundary_frozen_inputs.npz'))
    print('wrote %s' % (OUT_DIR / 'boundary_humble_recorded.npz'))


if __name__ == '__main__':
    main()
