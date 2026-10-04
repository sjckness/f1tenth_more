#!/usr/bin/env python3
"""Phase 5, Step 6: compare two phase5_bringup.sh runs' launched process lists
and every node's parameters.

Processes come from procs_settled.txt (full command lines). Normalised before
comparing: pids dropped; launch_ros's per-run parameter temp files
(/tmp/launch_params_*) and the run directory replaced by placeholders.
Parameters come from params.json; nodes whose names are generated per run
(transform_listener_impl_*, launch_ros_*) are compared by multiset of their
parameter sets.

  compare_snapshots.py A_DIR B_DIR OUT_JSON [--sim-check]

--sim-check: additionally report, for B, every node whose use_sim_time is not
true, and every running driver executable (vesc_driver_node, urg_node_driver,
zed_wrapper / zed components, joy_node).
"""
import collections
import json
import os
import re
import sys

DRIVERS = ('vesc_driver_node', 'urg_node_driver', 'zed_wrapper', 'zed_camera', 'joy_node',
           'ackermann_to_vesc_node', 'vesc_to_odom_node', 'battery_voltage_check_node')
GENERATED = re.compile(r'(transform_listener_impl_[0-9a-f]+|launch_ros_\d+)$')


def procs(run):
    out = collections.Counter()
    for line in open(os.path.join(run, 'procs_settled.txt')):
        if not line.strip():
            continue
        args = line.split(None, 4)[4]
        args = args.replace(os.path.abspath(run), '<RUN>')
        args = re.sub(r'/tmp/launch_params_\w+', '<PARAMS_TMP>', args)
        out[args] += 1
    return out


def params(run):
    p = json.load(open(os.path.join(run, 'params.json')))['params']
    named = {n: v for n, v in p.items() if not GENERATED.search(n)}
    gen = collections.Counter(
        json.dumps(v, sort_keys=True) for n, v in p.items() if GENERATED.search(n))
    return named, gen


def main():
    a, b, out = sys.argv[1:4]
    sim_check = '--sim-check' in sys.argv
    pa, pb = procs(a), procs(b)
    na, ga = params(a)
    nb, gb = params(b)
    res = {
        'a': a, 'b': b,
        'processes_only_in_a': sorted((pa - pb).elements()),
        'processes_only_in_b': sorted((pb - pa).elements()),
        'process_count': [sum(pa.values()), sum(pb.values())],
        'nodes_only_in_a': sorted(set(na) - set(nb)),
        'nodes_only_in_b': sorted(set(nb) - set(na)),
        'param_differences': {},
        'generated_name_nodes_param_sets_equal': ga == gb,
    }
    for n in sorted(set(na) & set(nb)):
        diff = {k: [na[n].get(k), nb[n].get(k)]
                for k in sorted(set(na[n]) | set(nb[n])) if na[n].get(k) != nb[n].get(k)}
        if diff:
            res['param_differences'][n] = diff
    if sim_check:
        allb = json.load(open(os.path.join(b, 'params.json')))['params']
        res['b_nodes_not_on_sim_time'] = sorted(
            n for n, v in allb.items() if v.get('use_sim_time') is not True)
        res['b_nodes_total'] = len(allb)
        res['b_driver_processes'] = sorted(
            {d for args in pb for d in DRIVERS if d in args})
    json.dump(res, open(out, 'w'), indent=2)
    print(json.dumps({k: (v if not isinstance(v, (list, dict)) or len(v) < 30 else f'{len(v)} items')
                      for k, v in res.items()}, indent=2))


if __name__ == '__main__':
    main()
