"""Unit tests of the pure helpers in tools/corridor_wall_replay.py, without ROS.

The tool imports rclpy (through scene_labels_replay) at module level, so these
tests do not import it: they lift the named pure functions and constants out
of its source with `ast` and run them in a namespace holding only the
standard library and numpy. Run:

    python3 -m pytest tools/test_corridor_wall_replay_helpers.py
"""

import ast
import bisect
import collections
import json
import math
import re
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

TOOL = Path(__file__).resolve().parent / 'corridor_wall_replay.py'
FUNCTIONS = {
    'split_order_key', 'latest', 'nearest_index', 'choose_rgb', 'choose_mask_source',
    'image_array', 'class_fractions', 'fraction_segments', 'tracker_steps', 'step_gaps',
    'wall_runs', 'classify_wall_episode', 'state_shares', 'error_stats', 'close_tracks',
    'pass_headline', 'num', 'pct', 'flat_headline', 'compare_rows', 'in_span', 'in_window',
    'clip_window', 'door_segments'}
CONSTANTS = {'RAW_PREFER_S', 'PHANTOM_MARGIN_M', 'END_WALL_RANGE_M', 'RHO_BL_CLOSE_M',
             '_ENC_CHANNELS', 'COMPARE_METRICS', 'DOOR_COLOUR', 'DOOR_OCCUPIED_COLOUR',
             'DOOR_STATE_NAMES'}


@pytest.fixture(scope='module')
def T():
    tree = ast.parse(TOOL.read_text(), feature_version=(3, 10))
    keep = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
            keep.append(node)
        elif isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id in CONSTANTS for t in node.targets):
            keep.append(node)
    found = {n.name for n in keep if isinstance(n, ast.FunctionDef)}
    assert found == FUNCTIONS, FUNCTIONS - found
    ns = {'math': math, 're': re, 'bisect': bisect, 'collections': collections,
          'np': np, 'json': json, 'Path': Path}
    exec(compile(ast.Module(body=keep, type_ignores=[]), str(TOOL), 'exec'), ns)
    return type('T', (), {k: staticmethod(v) if callable(v) else v for k, v in ns.items()})


def test_split_order_is_numeric(T):
    names = ['b_10.db3', 'b_2.db3', 'b_0.db3', 'b_1.db3']
    want = ['b_0.db3', 'b_1.db3', 'b_2.db3', 'b_10.db3']
    assert sorted(names, key=T.split_order_key) == want


def test_choose_rgb_prefers_raw_within_limit(T):
    raw = ([0.00, 0.20, 0.40], True)
    jpeg = ([0.18, 0.39], False)
    # raw 0.20 is 0.05 away, the JPEG 0.18 only 0.03: raw still wins
    k, j, age = T.choose_rgb(0.23, [raw, jpeg], 0.1)
    assert (k, j) == (0, 1) and age == pytest.approx(0.03)
    # no raw within 0.1 s: the nearest JPEG
    raw_far = ([0.0, 1.0], True)
    k, j, _age = T.choose_rgb(0.5, [raw_far, ([0.45], False)], 0.1)
    assert (k, j) == (1, 0)
    # only raw frames: the nearest one, however old
    k, j, age = T.choose_rgb(0.7, [raw_far], 0.1)
    assert (k, j) == (0, 1) and age == pytest.approx(-0.3)
    assert T.choose_rgb(0.0, [([], True)], 0.1) is None


def test_choose_mask_source_never_falls_back(T):
    assert T.choose_mask_source('auto', True, True) == ('recorded', None)
    assert T.choose_mask_source('auto', False, True) == (None, 'MASK NOT RECORDED')
    assert T.choose_mask_source('recorded', False, True)[0] is None
    assert T.choose_mask_source('replay', True, True) == ('replay', None)
    src, note = T.choose_mask_source('replay', True, False, has_depth=False)
    assert src is None and 'DEPTH NOT RECORDED' in note


def test_image_array_respects_step(T):
    h, w = 3, 4
    step = 4 * w + 8          # bgra8 rows padded by 8 bytes
    rows = []
    for r in range(h):
        px = np.arange(w * 4, dtype=np.uint8) + 10 * r
        rows.append(np.concatenate([px, np.full(8, 255, np.uint8)]))
    data = np.concatenate(rows).tobytes()
    arr = T.image_array(data, h, w, step, 'bgra8')
    assert arr.shape == (3, 4, 4)
    assert arr[1, 0, 0] == 10 and arr[2, 3, 3] == 20 + 15
    assert not (arr == 255).any()
    # the last row's padding may be missing
    assert T.image_array(data[:-8], h, w, step, 'bgra8').shape == (3, 4, 4)
    mono = T.image_array(bytes(range(12)), 3, 4, 4, 'mono8')
    assert mono.shape == (3, 4, 1) and mono[2, 3, 0] == 11
    assert T.image_array(b'\0' * 5, 3, 4, 4, 'mono8') is None
    assert T.image_array(b'\0' * 24, 3, 4, 8, '32FC1') is None


def test_class_fractions_and_segments(T):
    labels = np.array([[3, 3, 3, 7], [6, 6, 0, 7]], np.uint8)
    fr = T.class_fractions(labels, 8)
    assert fr[3] == pytest.approx(3 / 8) and fr[7] == pytest.approx(2 / 8)
    assert sum(fr.values()) == pytest.approx(1.0)
    segs = T.fraction_segments(fr, list(range(8)), 800)
    assert segs[0][1] == 0 and segs[-1][2] == 800
    assert all(a[2] == b[1] for a, b in zip(segs, segs[1:]))   # no gaps, no overlap
    assert [c for c, _a, _b in segs] == [0, 3, 6, 7]


def test_tracker_steps_old_per_scan_record(T):
    sk = [2, 3, 0, 1, 2, 3, 0, 1, 3, 0, 2, 0]
    debug = [{'tracker_skipped': k, 'tracker_decimation': 4} for k in sk]
    flags, info = T.tracker_steps(debug)
    assert flags == [k == 0 for k in sk]
    assert info['format'] == 'per_scan' and info['decimation'] == 4
    assert info['steps'] == 4
    # 1 -> 3 drops a payload; 3 -> 0 fine; 0 -> 2 drops one; 2 -> 0 early (break)
    assert info['breaks'] == 3
    assert info['missing_steps'] == 0
    # 3 -> 1: a step (0) went by unrecorded
    _f, info = T.tracker_steps([{'tracker_skipped': k, 'tracker_decimation': 4}
                                for k in (0, 1, 2, 3, 1, 2)])
    assert info['missing_steps'] == 1


def test_tracker_steps_new_per_step_record(T):
    flags, info = T.tracker_steps([{'tracker_skipped': 0}, {}, {'tracker_skipped': None}])
    assert flags == [True, True, True] and info['format'] == 'per_step'
    assert T.step_gaps([0.0, 0.1, 0.2, 0.5, 0.6])[0] == 1


def test_phantom_counts_empty_strip(T):
    nan = math.nan
    # empty strip on every cycle: the pre-v2 summary never counted this one
    assert T.classify_wall_episode([2.0, 2.1], [nan, nan])[:2] == (True, 'empty')
    # WALL 1.0 m short of the laser: phantom, short
    assert T.classify_wall_episode([2.0, 2.0, 2.0], [3.0, 3.1, 2.9])[:2] == (True, 'short')
    # a real wall
    assert T.classify_wall_episode([2.0, 2.05], [2.02, 2.0])[0] is False
    # one empty cycle in three does not make an honest wall a phantom
    assert T.classify_wall_episode([2.0, 2.0, 2.0], [2.0, nan, 2.0])[0] is False


def test_pass_headline(T):
    nan = math.nan
    times = [0.1 * i for i in range(12)]
    states = ['clear', 'wall', 'wall', 'clear', 'no_data', 'wall', 'wall', 'wall',
              'clear', 'wall', 'wall', None]
    values = [50, 4.0, 4.0, 50, None, 2.5, 2.4, 2.3, 50, 1.0, 1.0, None]
    truths = [nan, nan, nan, 8.0, 8.0, 2.5, 2.42, 2.25, 2.0, 1.2, 1.3, 1.0]
    moving = [True] * 12
    h = T.pass_headline(times, states, values, truths, moving, end_range=3.0)
    assert h['wall_episodes'] == 3
    assert h['phantom_front_episodes'] == 1
    assert h['phantom_front_by_kind'] == {'empty': 1}
    ew = h['end_wall_error']
    # strip < 3 m: cycles 5..11 -> 5 WALL, cycle 8 CLEAR and 11 None are misses
    assert ew['cycles'] == 7 and ew['n'] == 5 and ew['miss_cycles'] == 2
    assert ew['abs_max_m'] == pytest.approx(0.3)
    assert h['no_data_pct'] == pytest.approx(100 / 11, abs=0.01)
    assert h['cycles_without_state'] == 1


def test_close_tracks_confirmed_only(T):
    per_cycle = [[(1, True, 0.3), (2, False, 0.1)], [(1, True, 0.6)], [(3, True, -0.2)]]
    out = T.close_tracks(per_cycle, 0.5)
    assert out == {'n_ids': 2, 'ids': [1, 3], 'cycles': 2}


def test_compare_legacy_and_v2(T):
    legacy = {'summary': {
        'phantom_front_episodes': [{}, {}],
        'state_time_s': {'recorded': {'wall': 6.0, 'no_data': 2.0, 'clear': 2.0},
                         'replay': {'wall': 5.0, 'clear': 5.0},
                         'recorded_moving': {'no_data': 1.0, 'clear': 3.0}},
        'end_wall_error_m': {'recorded': {'p95': 0.4, 'max_abs': 1.5},
                             'replay': {'p95': 0.3, 'max_abs': 1.2}}}}
    before, notes = T.flat_headline(legacy)
    assert before['recorded']['phantom_front_episodes'] == 2
    assert before['recorded']['no_data_pct'] == 20.0
    assert before['recorded']['no_data_pct_moving'] == 25.0
    assert before['replay_rec']['all_wall_err_max_abs_m'] == 1.2
    v2 = {'summary': {'headline': {'passes': {
        'recorded': {'phantom_front_episodes': 3, 'no_data_pct': 10.0,
                     'all_wall_error': {'p95_m': 0.5, 'max_abs_m': 2.0},
                     'tracks_rho_lt_0_5': {'n_ids': 1}},
        'replay_rec': {'phantom_front_episodes': 0, 'no_data_pct': 5.0,
                       'tracks_rho_lt_0_5': {'n_ids': 0},
                       'tracks_rho_lt_0_5_all_live': {'n_ids': 2}}}}}}
    after, _n = T.flat_headline(v2)
    rows = T.compare_rows(before, after, notes)
    by = {(r[0], r[1]): r for r in rows}
    assert by[('phantom_front_episodes', 'recorded')][2:5] == [2, 3, 1]
    assert 'empty strip' in by[('phantom_front_episodes', 'recorded')][5]
    assert by[('no_data_pct', 'recorded')][4] == -10.0
    assert by[('tracks_rho_lt_0_5_all_live', 'replay_rec')][2:4] == [None, 2]
    assert 'pre-v2 replay' in by[('no_data_pct', 'replay_rec')][5]
    # metrics nobody has are not listed
    assert ('end_wall_err_abs_p95_m', 'recorded') not in by


def test_clip_window(T):
    assert T.clip_window([(1.0, 3.0), (5.0, 9.0)], None) == (1.0, 9.0)
    assert T.clip_window([(1.0, 3.0), (5.0, 9.0)], (2.0, 6.0)) == (2.0, 6.0)
    assert T.clip_window([(1.0, 3.0)], (4.0, 6.0)) is None
    assert T.clip_window([], None) is None


def test_door_segments_draws_each_door_in_base_link(T):
    """One segment per door, jamb to jamb in base_link, coloured by occupancy."""
    def door(i, state, occupied):
        return NS(id=i, state=state, occupied=occupied, width=0.85,
                  jamb0_bl=NS(x=3.0, y=-0.43), jamb1_bl=NS(x=3.0, y=0.42))
    msg = NS(doors=[door(1, 1, False), door(2, 2, True), door(3, 0, False)])
    segs = T.door_segments(msg)
    assert [s[0] for s in segs] == [(3.0, -0.43)] * 3
    assert segs[0][2] == T.DOOR_COLOUR and segs[0][3] == 3
    assert segs[0][4] == 'door 1 0.85m confirmed'
    # Coasting is dimmed; occupied is its own colour and says so.
    assert segs[1][2] == tuple(int(c * 0.55) for c in T.DOOR_OCCUPIED_COLOUR)
    assert segs[1][4].endswith('OCCUPIED')
    assert segs[2][3] == 1                       # tentative: thin
    assert T.door_segments(None) == []
