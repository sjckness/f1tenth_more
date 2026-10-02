#!/usr/bin/env python3
"""Phase 4 BT replay metrics.

For each replay run (and the recorded live Humble bag, as a sanity reference):
  - the /mpc/goal_drive the BT published (must equal Phase 3's
    goal_drive.yaml field for field: the goal both stacks were injected with
    is the goal the real BT sends);
  - mission start: /mission/status RUNNING time;
  - the stop decision: first /mpc/hold true after RUNNING, and its delay after
    the last downward crossing of the stop threshold by /perception/front_distance
    (read from the INPUT bag -- the value the BT actually saw);
  - /mission/move_outcome fields (stop_reason, move_type, move_id, outcome);
  - the /mission/status state sequence;
  - the /behavior/tree_status transition sequence (active_lane + lane_statuses,
    consecutive duplicates collapsed) from the stop onward.

Pairs (Jazzy x Jazzy floor; Jazzy x Humble when the Orin runs exist) are
compared on: stop delay difference, equality of outcome fields, of the status
sequence and of the post-stop tree-status transition sequence.

Usage: bt_metrics.py --input-bag BT_INPUT --out FILE.json --runs LABEL=BAG ...
         [--recorded SRC_BAG] [--goal-yaml goal_drive.yaml] [--threshold 2.0]
Reads bags only; runs on either distro.
"""
import argparse
import json

import yaml

from bag_read import read_topic

T0_TOPIC = '/odometry/filtered'


def bag_t0(bag):
    return next(read_topic(bag, T0_TOPIC))[0] * 1e-9


def load_run(bag, t0):
    run = {}
    run['goal_drive'] = [
        {'t': t * 1e-9 - t0, 'mode': m.mode, 'turn_sign': m.turn_sign,
         'turn_mag_deg': m.turn_mag_deg, 'speed': m.speed, 'd_safe': m.d_safe}
        for t, m in read_topic(bag, '/mpc/goal_drive')]
    run['hold'] = [(t * 1e-9 - t0, bool(m.data)) for t, m in read_topic(bag, '/mpc/hold')]
    run['status'] = [(t * 1e-9 - t0, m.state) for t, m in read_topic(bag, '/mission/status')]
    run['outcome'] = [{'t': t * 1e-9 - t0, 'mission_id': m.mission_id, 'move_id': m.move_id,
                       'move_type': m.move_type, 'stop_reason': m.stop_reason,
                       'outcome': m.outcome, 'duration_s': m.duration_s}
                      for t, m in read_topic(bag, '/mission/move_outcome')]
    trans = []
    for t, m in read_topic(bag, '/behavior/tree_status'):
        key = (m.active_lane, tuple(m.lane_names), tuple(m.lane_statuses),
               m.safety_stop_active, m.stop_source, m.emergency_trip)
        if not trans or trans[-1][1] != key:
            trans.append((t * 1e-9 - t0, key))
    run['tree_transitions'] = trans
    return run


def stop_metrics(run, front, threshold):
    running = [t for t, s in run['status'] if s == 'RUNNING']
    t_run = running[0] if running else None
    holds = [t for t, v in run['hold'] if v and (t_run is None or t > t_run)]
    t_hold = holds[0] if holds else None
    crossing = None
    if t_hold is not None and t_run is not None:
        prev = None
        for t, v in front:
            if t > t_hold:
                break
            if t >= t_run and v <= threshold and (prev is None or prev > threshold):
                crossing = t
            prev = v if t >= t_run else prev
    return {'t_running': t_run, 't_hold': t_hold, 't_crossing': crossing,
            'stop_delay_s': (t_hold - crossing) if (t_hold is not None and crossing is not None) else None}


def post_stop_sequence(run, t_hold):
    if t_hold is None:
        return []
    seq = [k for t, k in run['tree_transitions'] if t >= t_hold - 0.15]
    return [list(k[:1]) + [list(k[2])] for k in seq]


def summarise(label, run, front, threshold):
    sm = stop_metrics(run, front, threshold)
    return {'label': label, **sm,
            'goal_drive': run['goal_drive'],
            'outcome': run['outcome'],
            'status_sequence': [s for _, s in run['status']],
            'post_stop_tree_sequence': post_stop_sequence(run, sm['t_hold']),
            'n_tree_transitions': len(run['tree_transitions'])}


def compare(a, b):
    def strip(o):
        return [{k: v for k, v in x.items() if k not in ('t', 'duration_s')} for x in o]
    d = None
    if a['stop_delay_s'] is not None and b['stop_delay_s'] is not None:
        d = a['stop_delay_s'] - b['stop_delay_s']
    return {'pair': f"{a['label']}_vs_{b['label']}",
            'd_t_hold_s': (a['t_hold'] - b['t_hold']) if a['t_hold'] and b['t_hold'] else None,
            'd_stop_delay_s': d,
            'outcome_equal': strip(a['outcome']) == strip(b['outcome']),
            'status_sequence_equal': a['status_sequence'] == b['status_sequence'],
            'post_stop_tree_sequence_equal':
                a['post_stop_tree_sequence'] == b['post_stop_tree_sequence'],
            'goal_drive_equal': strip(a['goal_drive']) == strip(b['goal_drive'])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input-bag', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--runs', nargs='+', required=True, help='LABEL=BAG (Jazzy runs)')
    ap.add_argument('--humble-runs', nargs='*', default=[], help='LABEL=BAG (Orin Humble runs)')
    ap.add_argument('--recorded', help='original bag (sanity reference only)')
    ap.add_argument('--goal-yaml', help="Phase 3's goal_drive.yaml")
    ap.add_argument('--threshold', type=float, default=2.0)
    args = ap.parse_args()

    t0 = bag_t0(args.input_bag)
    front = [(t * 1e-9 - t0, m.data) for t, m in read_topic(args.input_bag, '/perception/front_distance')]
    res = {'input_bag_t0': t0, 'threshold': args.threshold}
    jz = [summarise(lbl, load_run(bag, t0), front, args.threshold)
          for lbl, bag in (s.split('=', 1) for s in args.runs)]
    hb = [summarise(lbl, load_run(bag, t0), front, args.threshold)
          for lbl, bag in (s.split('=', 1) for s in args.humble_runs)]
    res['jazzy'] = jz
    res['humble'] = hb
    res['jazzy_floor'] = [compare(jz[i], jz[j]) for i in range(len(jz)) for j in range(i + 1, len(jz))]
    res['humble_floor'] = [compare(hb[i], hb[j]) for i in range(len(hb)) for j in range(i + 1, len(hb))]
    res['jazzy_vs_humble'] = [compare(a, b) for a in jz for b in hb]
    if args.goal_yaml:
        with open(args.goal_yaml) as f:
            g = yaml.safe_load(f)
        res['goal_drive_matches_phase3_yaml'] = [
            all(abs(float(x[k]) - float(g[k])) < 1e-6 if k != 'mode' else x[k] == g[k]
                for k in ('mode', 'turn_sign', 'turn_mag_deg', 'speed', 'd_safe'))
            for r in jz for x in r['goal_drive']]
    if args.recorded:
        rec_t0 = bag_t0(args.recorded)
        rec = load_run(args.recorded, rec_t0)
        res['recorded_humble_sanity'] = {
            'hold': rec['hold'], 'status': rec['status'], 'outcome': rec['outcome'],
            'post_stop_tree_sequence': post_stop_sequence(
                rec, next((t for t, v in rec['hold'] if v), None)),
            't0_offset_vs_input_s': rec_t0 - t0}
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=2, default=str)
    for r in jz + hb:
        print(f"{r['label']}: running {r['t_running']:.3f} crossing {r['t_crossing']} "
              f"hold {r['t_hold']} delay {r['stop_delay_s']} "
              f"outcome {[o['stop_reason'] for o in r['outcome']]} status {r['status_sequence']}")
    for c in res['jazzy_floor'] + res['jazzy_vs_humble']:
        print(c)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
