#!/usr/bin/env python3
"""The 42 tests of f1tenth_control/test/test_hard_stop.py as JSON replay cases.

Each test function becomes one case: the inputs it builds (the same scans,
times, lane commands and parameters), as ops of the hard_stop_replay protocol,
plus the outputs the PYTHON code gives for them (py_reference.PySession: the
real hard_stop.py functions and a real HardStopNode). The test's own
assertions are re-checked on those outputs here, so a case that drifted from
its test fails at generation time. Parametrised tests are one case with every
parameter (55 pytest items -> 42 cases).

Written to src/rust/f1tenth_hard_stop_rs/tests/data/test_cases.jsonl, which
the crate's `cargo test` (tests/test_cases.rs) replays against the Rust code
without Python, and parity_hard_stop.py re-derives from Python and runs
through the replay binary.

Two tests are not about the decision:
  - test_the_module_imports_nothing_from_the_behaviour_stack: for Rust, the
    crate's dependencies (Cargo.toml, package.xml) must name nothing from the
    behaviour stack: a "deps" op, checked on the Rust side only;
  - test_the_bt_s_own_estop_distance_is_untouched_by_this: the stack_params
    arithmetic is re-checked here; the case starts a node with the launch
    file's hard_stop_range_m and compares its startup line.

Time: the Python tests run the node on the real clock; the cases use a
synthetic one (scans 25 ms apart from t = 1000 s), which is what the
assertions are about. test_the_republish_stops_once_released ages the latch
by writing last_hit_time, as the test does (a "set_state" op).

Usage (workspace sourced): make_test_cases.py [--out FILE] [--check]
  --check: regenerate in memory and exit 1 if FILE differs.
"""

import argparse
import copy
import json
import math
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from py_reference import enc_float, PySession  # noqa: E402

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
DEFAULT_OUT = os.path.join(_REPO, 'src', 'rust', 'f1tenth_hard_stop_rs', 'tests', 'data',
                           'test_cases.jsonl')

# test_hard_stop.py's constants.
LASER_X = 0.305
FRONT_X = 0.405
REAR_X = -0.10
HALF_WIDTH = 0.136
TRIGGER_M = 0.20
SELF_MARGIN_M = 0.02
N_BEAMS = 1081
ANGLE_MIN = math.radians(-135.0)
ANGLE_INC = math.radians(0.25)
RANGE_MIN = 0.02
RANGE_MAX = 30.0
POLY = [REAR_X - SELF_MARGIN_M, FRONT_X + SELF_MARGIN_M,
        -(HALF_WIDTH + SELF_MARGIN_M), HALF_WIDTH + SELF_MARGIN_M]
T0 = 1000.0
SCAN_DT = 0.025

#: The node's parameters as the test's HardStopNode gets them by default:
#: the declared defaults, the URDF bumper and stack_params half width.
NODE_DEFAULTS = {
    'hard_stop_enabled': True, 'hard_stop_range_m': 0.20, 'hard_stop_debounce_scans': 2,
    'hard_stop_clear_s': 0.5, 'hard_stop_republish_hz': 20.0, 'scan_topic': '/scan',
    'stop_topic': '/safety_stop', 'diag_topic': '/safety/hard_stop',
    'footprint_front_x_m': FRONT_X, 'footprint_rear_x_m': REAR_X,
    'footprint_half_width_m': HALF_WIDTH, 'footprint_self_margin_m': SELF_MARGIN_M,
    'laser_x_m': LASER_X, 'hard_stop_allow_reverse': True, 'lane_timeout_s': 0.2,
}


def beam_index(bearing_deg):
    return int(round((math.radians(bearing_deg) - ANGLE_MIN) / ANGLE_INC))


def ranges_with(points, fill=5.0):
    r = [fill] * N_BEAMS
    for bearing_deg, rng in points:
        r[beam_index(bearing_deg)] = rng
    return r


def trh(ranges):
    return {'op': 'fn', 'fn': 'trigger_range_hits', 'args': {
        'ranges': [enc_float(r) for r in ranges], 'angle_min': ANGLE_MIN,
        'angle_increment': ANGLE_INC, 'range_min': RANGE_MIN, 'range_max': RANGE_MAX,
        'laser_x': LASER_X, 'polygon': POLY, 'trigger_range_m': TRIGGER_M}}


def self_hit(x, y):
    return {'op': 'fn', 'fn': 'is_self_hit', 'args': {'x': x, 'y': y, 'polygon': POLY}}


def state(ops, debounce=2, clear=0.5):
    return {'op': 'fn', 'fn': 'state',
            'args': {'debounce_scans': debounce, 'clear_s': clear, 'ops': ops}}


class Node:
    """Op builder for one node session on the synthetic clock."""

    def __init__(self, **params):
        self.t = T0
        self.ops = [{'op': 'node', 'params': dict(NODE_DEFAULTS, **params)}]

    def scan(self, ranges):
        self.t += SCAN_DT
        self.ops.append({'op': 'scan', 'clock': [self.t],
                         'ranges': [enc_float(r) for r in ranges], 'angle_min': ANGLE_MIN,
                         'angle_increment': ANGLE_INC, 'range_min': RANGE_MIN,
                         'range_max': RANGE_MAX})

    def republish(self):
        self.ops.append({'op': 'republish', 'clock': [self.t]})

    def drive(self, speed, lane='drive'):
        self.ops.append({'op': 'lane', 'lane': lane, 'speed': speed, 'clock': [self.t]})

    def set_last_hit(self, t):
        self.ops.append({'op': 'set_state', 'last_hit_time': t})


def stops(outs):
    return sum(1 for o in outs if o.get('stop'))


def diag(o):
    return json.loads(o['diag'])


SELF_ARC = [(b, 0.10) for b in range(95, 136)] + [(-b, 0.10) for b in range(95, 136)]


# ---------------------------------------------------------------- the 42
def cases():
    """[(test id, ops, check(outs))]."""
    out = []

    def case(name, ops, check):
        out.append((name, ops, check))

    k = 'TestTriggerAndDebounce::'

    def c1(o):
        found = o[0]['out']
        assert len(found) == 1 and abs(found[0][0] - 0.18) < 1e-6
        assert abs(found[0][2] - (LASER_X + 0.18)) < 1e-6
    case(k + 'test_a_return_at_0p18_m_ahead_is_a_trigger', [trh(ranges_with([(0.0, 0.18)]))], c1)

    def c2(o):
        assert o[0]['out']['results'] == [False, True]
        assert o[0]['out']['final']['trigger_count'] == 1
    case(k + 'test_it_engages_within_two_scans',
         [state([['update', True, 0.000], ['update', True, 0.025]])], c2)

    n = Node()
    n.scan(ranges_with([(0.0, 0.18)]))
    n.scan(ranges_with([(0.0, 0.18)]))

    def c3(o):
        assert o[0]['started'] and not o[1]['stop'] and o[2]['stop']
    case(k + 'test_the_node_publishes_a_zero_speed_stop_on_the_safety_lane', n.ops, c3)

    def c4(o):
        assert o[2]['stop']
        assert o[0]['stop_frame_id'] == 'base_link/hard_stop'
        assert o[0]['stop_frame_id'] not in ('', 'base_link', 'base_link/emergency',
                                             'base_link/obstacle')
    case(k + 'test_the_stop_is_attributable_in_a_bag', list(n.ops), c4)

    case(k + 'test_a_return_just_outside_the_radius_does_not_trigger',
         [trh(ranges_with([(0.0, 0.21)]))], lambda o: o[0]['out'] == [] or _fail())
    case(k + 'test_the_radius_is_inclusive',
         [trh(ranges_with([(0.0, TRIGGER_M)]))], lambda o: len(o[0]['out']) == 1 or _fail())
    bearings = [0.0, 20.0, -20.0, 40.0, -40.0, 75.0, -75.0, 90.0, -90.0]
    case(k + 'test_it_triggers_all_round_not_only_ahead',
         [trh(ranges_with([(b, 0.18)])) for b in bearings],
         lambda o: all(len(x['out']) == 1 for x in o) or _fail())

    band = list(range(0, 136))

    def c8(o):
        x_max, y_max = FRONT_X + SELF_MARGIN_M, HALF_WIDTH + SELF_MARGIN_M
        for deg, x in zip(band, o):
            b = math.radians(deg)
            inside = (LASER_X + 0.18 * math.cos(b) <= x_max) and (abs(0.18 * math.sin(b)) <= y_max)
            assert (x['out'] == []) == inside, deg
    case(k + 'test_the_blind_band_is_exactly_the_car_s_own_body',
         [trh(ranges_with([(float(d), 0.18)])) for d in band], c8)
    case(k + 'test_a_return_at_the_bumper_face_is_inside_the_mask',
         [trh(ranges_with([(0.0, 0.12)])), trh(ranges_with([(0.0, 0.13)]))],
         lambda o: (o[0]['out'] == [] and len(o[1]['out']) == 1) or _fail())
    case(k + 'test_no_returns_at_all_is_not_a_trigger',
         [trh([math.nan] * N_BEAMS), trh([math.inf] * N_BEAMS)],
         lambda o: all(x['out'] == [] for x in o) or _fail())
    case(k + 'test_a_value_under_range_min_is_not_a_trigger',
         [trh(ranges_with([(0.0, 0.01)]))], lambda o: o[0]['out'] == [] or _fail())
    case(k + 'test_the_nearest_hit_is_reported_first',
         [trh(ranges_with([(0.0, 0.19), (20.0, 0.14), (-20.0, 0.17)]))],
         lambda o: [round(h[0], 2) for h in o[0]['out']] == [0.14, 0.17, 0.19] or _fail())

    k = 'TestSelfHitMask::'
    case(k + 'test_the_polygon_covers_the_body_plus_the_margin',
         [{'op': 'fn', 'fn': 'footprint_polygon', 'args': {
             'front_x': FRONT_X, 'rear_x': REAR_X, 'half_width': HALF_WIDTH,
             'margin': SELF_MARGIN_M}}],
         lambda o: o[0]['out'] == POLY or _fail())
    case(k + 'test_a_return_on_the_rear_deck_is_a_self_hit',
         [self_hit(LASER_X - 0.18, 0.0), trh(ranges_with([(135.0, 0.18)])),
          trh(ranges_with([(-135.0, 0.18)]))],
         lambda o: (o[0]['out'] is True and o[1]['out'] == [] and o[2]['out'] == []) or _fail())
    chassis = [100.0, 120.0, 134.0, -100.0, -120.0, -134.0]
    ops = []
    for b in chassis:
        r = math.radians(b)
        ops += [trh(ranges_with([(b, 0.18)])),
                self_hit(LASER_X + 0.18 * math.cos(r), 0.18 * math.sin(r))]

    def c15(o):
        for i in range(0, len(o), 2):
            if o[i + 1]['out']:
                assert o[i]['out'] == []
    case(k + 'test_returns_across_the_chassis_never_trigger', ops, c15)

    n = Node()
    for _ in range(20):
        n.scan(ranges_with(SELF_ARC))

    def c16(o):
        assert stops(o) == 0 and o[-1]['state']['trigger_count'] == 0
    case(k + 'test_a_scan_that_is_nothing_but_self_hits_never_engages', n.ops, c16)
    n = Node()
    n.scan(ranges_with(SELF_ARC + [(0.0, 0.18)]))
    n.scan(ranges_with(SELF_ARC + [(0.0, 0.18)]))
    case(k + 'test_a_real_obstacle_still_triggers_through_the_self_hits', n.ops,
         lambda o: (o[-1]['state']['engaged'] and stops(o) == 1) or _fail())
    case(k + 'test_the_margin_is_what_decides_a_point_at_the_shell',
         [self_hit(0.0, HALF_WIDTH + 0.01), self_hit(0.0, HALF_WIDTH + SELF_MARGIN_M + 0.01)],
         lambda o: (o[0]['out'] is True and o[1]['out'] is False) or _fail())

    k = 'TestLatchAndRelease::'
    eng = [['update', True, 0.000], ['update', True, 0.025]]
    case(k + 'test_it_stays_engaged_while_the_zone_is_still_occupied',
         [state(eng + [['update', True, 0.05 + 0.025 * i] for i in range(40)])],
         lambda o: all(o[0]['out']['results'][2:]) or _fail())
    case(k + 'test_one_clear_scan_does_not_release_it', [state(eng + [['update', False, 0.050]])],
         lambda o: o[0]['out']['results'][-1] is True or _fail())
    case(k + 'test_it_releases_after_the_full_clear_hold',
         [state(eng + [['update', False, 0.400], ['update', False, 0.525]])],
         lambda o: o[0]['out']['results'][-2:] == [True, False] or _fail())
    case(k + 'test_a_hit_during_the_hold_restarts_the_clock',
         [state(eng + [['update', False, 0.300], ['update', True, 0.400],
                       ['update', False, 0.800], ['update', False, 0.950]])],
         lambda o: o[0]['out']['results'][-2:] == [True, False] or _fail())

    def c23(o):
        r = o[0]['out']['results']
        assert r[2]['trigger_count'] == 1 and r[3] is False
        assert r[-1]['engaged'] and r[-1]['trigger_count'] == 2
    case(k + 'test_re_engaging_counts_as_a_second_trigger',
         [state(eng + [['as_dict', 0, None], ['update', False, 0.6],
                       ['update', True, 0.7], ['update', True, 0.8], ['as_dict', 0, None]])],
         c23)
    case(k + 'test_the_hold_is_re_evaluated_without_any_scan',
         [state(eng + [['tick', 0.300], ['tick', 0.600]])],
         lambda o: o[0]['out']['results'][-2:] == [True, False] or _fail())
    n = Node()
    n.scan(ranges_with([(0.0, 0.18)]))
    n.scan(ranges_with([(0.0, 0.18)]))
    n.republish()
    n.republish()
    case(k + 'test_the_node_keeps_the_mux_lane_claimed_while_engaged', n.ops,
         lambda o: (o[3]['stop'] and o[4]['stop']) or _fail())
    n = Node()
    n.scan(ranges_with([(0.0, 0.18)]))
    n.scan(ranges_with([(0.0, 0.18)]))
    n.set_last_hit(n.t - 10.0)
    n.republish()
    n.republish()
    case(k + 'test_the_republish_stops_once_released', n.ops,
         lambda o: (not o[4]['stop'] and not o[5]['stop']) or _fail())

    k = 'TestIndependentOfTheBT::'
    out.append((k + 'test_the_module_imports_nothing_from_the_behaviour_stack',
                [{'op': 'deps'}], None))
    n = Node()
    case(k + 'test_it_subscribes_to_the_raw_scan_and_nothing_from_the_bt', n.ops,
         lambda o: o[0]['subscriptions'] == ['/calibration_drive', '/drive', '/scan',
                                             '/teleop'] or _fail())
    n = Node(hard_stop_allow_reverse=False)
    case(k + 'test_without_the_reverse_gate_the_scan_is_the_only_input', n.ops,
         lambda o: o[0]['subscriptions'] == ['/scan'] or _fail())
    n = Node()
    n.scan(ranges_with([(0.0, 0.15)]))
    n.scan(ranges_with([(0.0, 0.15)]))
    case(k + 'test_it_stops_the_car_with_no_bt_input_whatsoever', n.ops,
         lambda o: (o[-1]['state']['engaged'] and stops(o) == 1) or _fail())

    def c31(o):
        from f1tenth_params.param_defaults import get_value
        bt = (float(get_value('car_radius')) + float(get_value('obstacle_safety_margin_m'))
              + float(get_value('proximity_front_extra_margin_m')))
        assert abs(bt - 0.40) < 1e-9 and TRIGGER_M < bt
        assert 'range <= 0.200 m' in o[0]['startup']
    case(k + 'test_the_bt_s_own_estop_distance_is_untouched_by_this', Node().ops, c31)

    k = 'TestDiagnostic::'
    n = Node()
    n.scan(ranges_with([]))

    def c32(o):
        d = diag(o[1])
        assert d['engaged'] is False and d['trigger_count'] == 0
        assert d['n_hits_this_scan'] == 0 and d['nearest_range_m'] is None
    case(k + 'test_it_publishes_state_and_count_every_scan', n.ops, c32)
    n = Node()
    n.scan(ranges_with([(-30.0, 0.17)]))
    n.scan(ranges_with([(-30.0, 0.17)]))

    def c33(o):
        d = diag(o[2])
        assert d['engaged'] is True and d['trigger_count'] == 1
        assert abs(d['nearest_range_m'] - 0.17) <= 1e-3
        assert abs(d['nearest_bearing_deg'] + 30.0) <= 0.3
    case(k + 'test_a_trigger_is_reported_with_its_range_and_bearing', n.ops, c33)
    n = Node()
    n.scan(ranges_with([]))
    case(k + 'test_the_thresholds_in_force_are_in_the_payload', n.ops,
         lambda o: (diag(o[1])['debounce_scans'] == 2 and diag(o[1])['clear_s'] == 0.5)
         or _fail())

    n = Node(hard_stop_enabled=False)
    for _ in range(10):
        n.scan(ranges_with([(0.0, 0.10)]))
    n.republish()
    case('TestDisabled::test_disabled_never_claims_the_lane', n.ops,
         lambda o: (stops(o) == 0 and o[-1]['state']['trigger_count'] == 0) or _fail())

    k = 'TestReverseGate::'
    rule = [(False, True, True, -0.4, False), (True, True, True, -0.4, False),
            (True, True, True, 0.0, True), (True, True, True, 0.4, True),
            (True, True, True, None, True), (True, True, False, -0.4, True),
            (True, False, True, -0.4, True)]
    case(k + 'test_the_rule',
         [{'op': 'fn', 'fn': 'stop_now', 'args': {'engaged': e, 'allow_reverse': a,
                                                  'ahead_only': h, 'speed': s}}
          for e, a, h, s, _w in rule],
         lambda o: [x['out'] for x in o] == [w for *_x, w in rule] or _fail())
    latest = {'drive': [0.0, -0.4], 'teleop': [0.0, 0.0]}

    def ws(lat, now):
        return {'op': 'fn', 'fn': 'winning_speed',
                'args': {'latest': lat, 'now': now, 'timeout_s': 0.2}}
    case(k + 'test_the_mux_s_own_lane_choice',
         [ws(latest, 0.1), ws(latest, 0.25), ws({'drive': [0.0, -0.4]}, 0.1)],
         lambda o: [x['out'] for x in o] == [0.0, None, -0.4] or _fail())
    out.append((k + 'test_only_returns_past_the_bumper_are_ahead', 'all_ahead', None))

    def engaged_ahead(**params):
        n = Node(**params)
        n.scan(ranges_with([(0.0, 0.18)]))
        n.scan(ranges_with([(0.0, 0.18)]))
        return n
    n = engaged_ahead()
    n.drive(-0.4)
    n.republish()
    n.scan(ranges_with([(0.0, 0.18)]))

    def c39(o):
        assert o[2]['state']['engaged'] and o[2]['state']['ahead_only']
        assert not o[4]['stop'] and not o[5]['stop'], 'quiet while reversing'
        assert o[5]['state']['engaged'] and diag(o[5])['passing_reverse']
    case(k + 'test_a_reverse_command_passes_an_obstacle_ahead', n.ops, c39)
    n = engaged_ahead()
    n.drive(-0.4)
    n.drive(0.0)
    n.drive(0.5)
    case(k + 'test_the_first_non_negative_command_reclaims_the_lane_at_once', n.ops,
         lambda o: (not o[3]['stop'] and o[4]['stop'] and o[5]['stop']) or _fail())
    n = Node()
    n.scan(ranges_with([(75.0, 0.18)]))
    n.scan(ranges_with([(75.0, 0.18)]))
    n.drive(-0.4)
    n.republish()
    case(k + 'test_an_obstacle_beside_the_nose_blocks_reverse_too', n.ops,
         lambda o: o[4]['stop'] or _fail())
    n = engaged_ahead(hard_stop_allow_reverse=False)
    n.drive(-0.4)
    n.republish()
    case(k + 'test_off_it_blocks_every_direction', n.ops,
         lambda o: o[4]['stop'] or _fail())
    return out


def _fail():
    raise AssertionError('test assertion failed on the Python outputs')


def build():
    """[{test, ops, expected}] with the Python's outputs."""
    records = []
    for name, ops, check in cases():
        if ops == 'all_ahead':
            # Two steps: the hits come from trigger_range_hits first.
            s = PySession()
            ahead = s.run(dict(trh(ranges_with([(0.0, 0.18)])), id='h0'))['out']
            beside = s.run(dict(trh(ranges_with([(75.0, 0.18)])), id='h1'))['out']
            ops = [{'op': 'fn', 'fn': 'all_ahead', 'args': {'hits': ahead, 'polygon': POLY}},
                   {'op': 'fn', 'fn': 'all_ahead', 'args': {'hits': beside, 'polygon': POLY}}]

            def check(o, beside=beside):
                assert o[0]['out'] is True and beside and o[1]['out'] is False
        ops = copy.deepcopy(ops)
        for i, op in enumerate(ops):
            op['id'] = f'{name}#{i}'
        if check is None:
            records.append(dict(test=name, ops=ops, expected=None,
                                rust_only='deps',
                                forbidden=['py_trees', 'f1tenth_behavior', 'behaviours']))
            continue
        session = PySession()
        try:
            outs = [session.run(op) for op in ops]
        finally:
            session.close()
        check(outs)
        records.append(dict(test=name, ops=ops, expected=outs))
    return records


def dumps(records):
    return ''.join(json.dumps(r, allow_nan=False, sort_keys=True) + '\n' for r in records)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    try:
        records = build()
    finally:
        rclpy.try_shutdown()
    text = dumps(records)
    n_tests = len(records)
    n_ops = sum(len(r['ops']) for r in records)
    if a.check:
        with open(a.out) as f:
            current = f.read()
        if current != text:
            print(f'{a.out} is stale: regenerate with make_test_cases.py')
            return 1
        print(f'{a.out}: current ({n_tests} tests, {n_ops} ops)')
        return 0
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write(text)
    print(f'wrote {a.out}: {n_tests} tests, {n_ops} ops')
    return 0 if n_tests == 42 else 1


if __name__ == '__main__':
    sys.exit(main())
