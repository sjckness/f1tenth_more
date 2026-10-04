#!/usr/bin/env python3
"""Summarise phase5_bringup.sh run directories (step4, cycles, snapshot) into
one JSON + a markdown table.

Usage: phase5_summary.py OUT_JSON RUN_DIR...
"""
import glob
import json
import os
import re
import sys


def _read(p, default=''):
    try:
        with open(p) as f:
            return f.read()
    except OSError:
        return default


def summarise(run):
    s = {'run': os.path.basename(run.rstrip('/'))}
    nodes = json.load(open(os.path.join(run, 'nodes_startup.json')))
    seen = nodes['seen']
    s['nodes'] = len(nodes['final'])
    s['startup_total_s'] = round(max(v['first'] for v in seen.values()), 2)
    cpath = os.path.join(run, 'component_nodes_settled.json')
    comp = json.load(open(cpath)) if os.path.exists(cpath) else {}
    s['startup_per_component_s'] = {
        c: (round(max(seen[n]['first'] for n in ns if n in seen), 2)
            if any(n in seen for n in ns) else None)
        for c, ns in sorted(comp.items())}
    s['components_without_nodes'] = sorted(c for c, ns in comp.items() if not ns)
    launch = _read(os.path.join(run, 'launch.log'))
    s['supervisor_crash_lines'] = len(re.findall(r"crashed \(exit", launch))
    s['shutdown_path'] = ('force-kill' if 'force-killing every tracked component' in launch
                          else 'graceful')
    sd = _read(os.path.join(run, 'shutdown.txt'))
    m = re.search(r'launch exit code: (-?\d+)', sd)
    s['launch_exit_code'] = int(m.group(1)) if m else None
    m = re.search(r'shutdown duration: ([\d.]+)', sd)
    s['shutdown_s'] = round(float(m.group(1)), 2) if m else None
    left = [line for line in _read(os.path.join(run, 'leftover_procs.txt')).splitlines() if line.strip()]
    s['leftover_procs'] = [line.split(None, 4)[-1][:90] for line in left
                           if 'fast-discovery-server' not in line]
    s['discovery_server_left_running'] = any('fast-discovery-server' in line for line in left)
    s['leftover_locks'] = _read(os.path.join(run, 'leftover_locks.txt')).split()
    before = [x for x in _read(os.path.join(run, 'shm_before.txt')).split() if 'fastrtps' in x]
    after = [x for x in _read(os.path.join(run, 'shm_after.txt')).split() if 'fastrtps' in x]
    s['shm_fastrtps_before'] = len(before)
    s['shm_fastrtps_after'] = len(after)
    sweeps = re.findall(r'FastDDS SHM sweep \(([^)]*\)?)\): (.*?)(?:\n|$)', launch)
    s['shm_sweeps'] = [f'{c}: {m[:110]}' for c, m in sweeps]
    de = _read(os.path.join(run, 'discovery_errors.txt'))
    s['matching_unexisting_participant'] = int(re.search(r'participant: (\d+)', de).group(1))
    s['discovery_database_errors'] = int(re.search(r'DISCOVERY_DATABASE \(any\): (\d+)', de).group(1))
    s['other_dds_errors'] = int(re.search(r'errors \(any\): (\d+)', de).group(1))
    tb = 0
    for f in glob.glob(os.path.join(run, 'supervisor', '*.log')):
        tb += _read(f).count('Traceback')
    s['tracebacks_in_component_logs'] = tb
    rates = os.path.join(run, 'rates.json')
    if os.path.exists(rates):
        r = json.load(open(rates))
        s['rates_hz'] = r['rates_hz']
        s['rates_missing'] = r['not_in_graph']
    return s


def main():
    out = [summarise(r) for r in sys.argv[2:]]
    json.dump(out, open(sys.argv[1], 'w'), indent=2)
    cols = ['run', 'nodes', 'startup_total_s', 'supervisor_crash_lines', 'shutdown_path',
            'shutdown_s', 'discovery_server_left_running', 'shm_fastrtps_before',
            'shm_fastrtps_after', 'matching_unexisting_participant',
            'discovery_database_errors', 'tracebacks_in_component_logs']
    print('| ' + ' | '.join(cols + ['other leftover procs', 'leftover locks']) + ' |')
    print('|' + '---|' * (len(cols) + 2))
    for s in out:
        print('| ' + ' | '.join(str(s[c]) for c in cols)
              + f" | {len(s['leftover_procs'])} | {' '.join(s['leftover_locks']) or '-'} |")


if __name__ == '__main__':
    main()
