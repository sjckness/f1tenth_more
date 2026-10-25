#!/usr/bin/env python3
"""
Compare two phase5_bringup.sh runs' process lists and every node's parameters.

The native stack's against the container's.

Runs on: anywhere with Python 3 (reads the run directories; no ROS).
Compares: procs_settled.txt (full command lines) and params.json of a
`snapshot` (or `step4`) run on each side. The differences are expected to be
exactly the car-profile delta (output/orin_container/params_delta.md) plus
the image's installed paths; anything else is a finding.

Processes are normalised before comparing: pids dropped; launch_ros's
per-run parameter files (/tmp/launch_params_*), the run directory (in
parameter values too) and the
workspace install prefix (--prefix-a / --prefix-b, e.g.
/home/fabiocar/dev_ws/f1tenth_more/install vs /opt/f1tenth_ws/install)
replaced by placeholders. Nodes whose names are generated per run
(transform_listener_impl_*, launch_ros_*) are compared as a multiset of
their parameter sets.

  compare_snapshots.py A_DIR B_DIR OUT_JSON [--prefix-a P] [--prefix-b P]
"""
import argparse
import collections
import json
import os
import re
import sys

GENERATED = re.compile(r'(transform_listener_impl_[0-9a-f]+|launch_ros_\d+)$')
_PARAMS_TMP = re.compile(r'/tmp/launch_params_\w+')


def procs(run, prefix=None):
    out = collections.Counter()
    with open(os.path.join(run, 'procs_settled.txt')) as f:
        for line in f:
            if not line.strip():
                continue
            parts = line.split(None, 4)
            args = parts[4].rstrip('\n') if len(parts) > 4 else ''
            args = args.replace(os.path.abspath(run), '<RUN>')
            if prefix:
                args = args.replace(prefix.rstrip('/'), '<INSTALL>')
            args = _PARAMS_TMP.sub('<PARAMS_TMP>', args)
            out[args] += 1
    return out


def _mask(value, run):
    """Replace the run directory in a parameter value by <RUN> (it differs per run)."""
    if isinstance(value, str):
        return value.replace(os.path.abspath(run), '<RUN>')
    if isinstance(value, list):
        return [_mask(v, run) for v in value]
    return value


def params(run):
    with open(os.path.join(run, 'params.json')) as f:
        p = json.load(f)['params']
    p = {n: {k: _mask(v, run) for k, v in vals.items()} for n, vals in p.items()}
    named = {n: v for n, v in p.items() if not GENERATED.search(n)}
    gen = collections.Counter(
        json.dumps(v, sort_keys=True) for n, v in p.items() if GENERATED.search(n))
    return named, gen


def compare(a, b, prefix_a=None, prefix_b=None):
    pa, pb = procs(a, prefix_a), procs(b, prefix_b)
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
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  compare_snapshots.py'):])
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('out')
    ap.add_argument('--prefix-a')
    ap.add_argument('--prefix-b')
    args = ap.parse_args(argv)
    res = compare(args.a, args.b, args.prefix_a, args.prefix_b)
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=2)
    print(json.dumps({k: (v if not isinstance(v, (list, dict)) or len(v) < 30
                          else '%d items' % len(v))
                      for k, v in res.items()}, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
