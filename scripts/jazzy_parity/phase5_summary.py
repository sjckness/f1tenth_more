#!/usr/bin/env python3
"""
Summarise phase5_bringup.sh run directories into one JSON + a markdown table.

Runs on: anywhere with Python 3 (reads files only; no ROS). Typically on the
Orin host or linus over both sides' run directories.

Per run: node count and startup times (overall and per component), the
supervisor's watchdog respawns ("crashed (exit N) -- auto-respawning" and
"exceeded its restart budget"), the shutdown path and duration, components
SIGKILLed at shutdown, the process audits (proc_audit.py: duplicates,
orphans, leftovers after shutdown), Fast DDS SHM files before/after,
discovery error counts, tracebacks in component logs, rates, the CPU/memory
window (sys_sample.py), the mission states, the runs the mission logger
wrote and the archive syncs it requested (captured, never run).
parity_gates.py smoke turns these into the P9.2 verdict.

Usage: phase5_summary.py OUT_JSON RUN_DIR [RUN_DIR ...]
       (a cycles directory: pass its cycle_* subdirectories)
"""
import glob
import json
import os
import re
import sys

import yaml


def _read(p, default=''):
    try:
        with open(p, errors='replace') as f:
            return f.read()
    except OSError:
        return default


def _json(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _int(pattern, text):
    m = re.search(pattern, text)
    return int(m.group(1)) if m else None


def logger_runs(run):
    """List the runs the mission logger wrote under RUN/runs (state, bag messages)."""
    out = []
    for d in sorted(glob.glob(os.path.join(run, 'runs', '*', '*'))):
        if not os.path.isdir(d):
            continue
        meta = os.path.join(d, 'bag', 'metadata.yaml')
        msgs = None
        try:
            with open(meta) as f:
                info = yaml.safe_load(f)['rosbag2_bagfile_information']
            msgs = int(info.get('message_count', 0))
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
            pass
        out.append({'state': os.path.basename(os.path.dirname(d)),
                    'run_id': os.path.basename(d), 'bag_messages': msgs,
                    'extract': bool(glob.glob(os.path.join(d, '*.extract.parquet')))})
    return out


def archive_requests(run):
    """
    List the archive syncs the stack requested; the harness captured them all.

    phase5_bringup.sh points F1TENTH_ARCHIVE_TRIGGER at RUN/archive_trigger/
    request and puts a `systemctl` shim logging to RUN/systemctl_calls.txt
    first in the stack's PATH, so neither request leaves the run.
    """
    out = []
    trig = _read(os.path.join(run, 'archive_trigger', 'request')).strip()
    if trig:
        out.append('trigger ' + trig[:160])
    out += ['systemctl ' + line.strip()[:160]
            for line in _read(os.path.join(run, 'systemctl_calls.txt')).splitlines()
            if line.strip()]
    return out


def audits(run):
    """{label: {clean, duplicates, orphans, leftovers}} for every audit_*.json."""
    out = {}
    for p in sorted(glob.glob(os.path.join(run, 'audit_*.json'))
                    + glob.glob(os.path.join(run, 'restart_*', 'audit_after.json'))):
        a = _json(p)
        if a is None:
            continue
        label = os.path.relpath(p, run)[:-5].replace('audit_', '')
        out[label] = {'clean': a.get('clean'),
                      'duplicates': len(a.get('duplicate_node_names', {}))
                      + len(a.get('duplicate_command_lines', {})),
                      'duplicate_names': sorted(a.get('duplicate_node_names', {})),
                      'orphans': len(a.get('orphans', [])),
                      'leftovers': len(a.get('leftovers', []))}
    return out


def summarise(run):
    s = {'run': os.path.basename(run.rstrip('/')), 'path': os.path.abspath(run)}
    nodes = _json(os.path.join(run, 'nodes_startup.json')) or {'final': [], 'seen': {}}
    seen = nodes['seen']
    s['nodes'] = len(nodes['final'])
    s['startup_total_s'] = round(max(v['first'] for v in seen.values()), 2) if seen else None
    comp = _json(os.path.join(run, 'component_nodes_settled.json')) or {}
    s['startup_per_component_s'] = {
        c: (round(max(seen[n]['first'] for n in ns if n in seen), 2)
            if any(n in seen for n in ns) else None)
        for c, ns in sorted(comp.items())}
    s['components_without_nodes'] = sorted(c for c, ns in comp.items() if not ns)
    launch = _read(os.path.join(run, 'launch.log'))
    s['watchdog_respawns'] = len(re.findall(r'crashed \(exit -?\d+\) -- auto-respawning', launch))
    s['watchdog_budget_exceeded'] = len(re.findall(r'exceeded its restart budget', launch))
    s['shutdown_path'] = ('force-kill' if 'shutdown (force-kill)' in launch
                          else 'graceful' if 'Shutting down -- stopping all components' in launch
                          else 'unknown')
    s['components_sigkilled_at_shutdown'] = len(re.findall(r'killed via SIGKILL after', launch))
    sd = _read(os.path.join(run, 'shutdown.txt'))
    s['launch_exit_code'] = _int(r'launch exit code: (-?\d+)', sd)
    m = re.search(r'shutdown duration: (-?[\d.]+)', sd)
    s['shutdown_s'] = round(float(m.group(1)), 2) if m else None
    s['launch_needed_sigkill'] = 'SIGKILL' in sd
    s['audits'] = audits(run)
    s['audits_clean'] = all(a['clean'] for a in s['audits'].values()) if s['audits'] else None
    left = [line for line in _read(os.path.join(run, 'leftover_procs.txt')).splitlines()
            if line.strip()]
    s['leftover_procs'] = [line.split(None, 4)[-1][:100] for line in left
                           if not re.search(r'fast-discovery-server|fastdds(\.py)? discovery'
                                            r'|ensure_discovery_server', line)]
    s['discovery_server_left_running'] = any(
        re.search(r'fast-discovery-server|fastdds(\.py)? discovery', line) for line in left)
    # `path pid alive|dead` per line (older runs: the path only).
    locks = [line.split() for line in _read(os.path.join(run, 'leftover_locks.txt')).splitlines()
             if line.strip()]
    s['leftover_locks'] = [x[0] for x in locks]
    s['leftover_locks_live'] = [x[0] for x in locks if len(x) > 2 and x[2] == 'alive']
    before = [x for x in _read(os.path.join(run, 'shm_before.txt')).split() if 'fastrtps' in x]
    after = [x for x in _read(os.path.join(run, 'shm_after.txt')).split() if 'fastrtps' in x]
    s['shm_fastrtps_before'] = len(before)
    s['shm_fastrtps_after'] = len(after)
    sweeps = re.findall(r'FastDDS SHM sweep \(([^)]*\)?)\): (.*?)(?:\n|$)', launch)
    s['shm_sweeps'] = ['%s: %s' % (c, m[:110]) for c, m in sweeps]
    de = _read(os.path.join(run, 'discovery_errors.txt'))
    s['matching_unexisting_participant'] = _int(r'participant: (\d+)', de)
    s['discovery_database_errors'] = _int(r'DISCOVERY_DATABASE \(any\): (\d+)', de)
    s['other_dds_errors'] = _int(r'errors \(any\): (\d+)', de)
    s['tracebacks_in_component_logs'] = sum(
        _read(f).count('Traceback') for f in glob.glob(os.path.join(run, 'supervisor', '*.log')))
    for name in ('rates', 'rates_hold'):
        r = _json(os.path.join(run, name + '.json'))
        if r:
            s[name + '_hz'] = r['rates_hz']
            s[name + '_missing'] = r['not_in_graph']
    sysw = _json(os.path.join(run, 'sys_hold.json'))
    if sysw:
        s['cpu_total_util_pct'] = sysw.get('total_util_pct')
        s['cpu_per_core_util_pct'] = sysw.get('per_core_util_pct')
        s['stack_cpu_pct_of_core'] = sysw.get('stack_cpu_pct_of_core_total')
        s['stack_rss_mb'] = round((sysw.get('stack_rss_kb_total') or 0) / 1024.0, 1)
        s['mem_used_mb'] = (round(sysw['mem']['used_kb_end'] / 1024.0, 1)
                            if sysw.get('mem', {}).get('used_kb_end') else None)
    mission = _json(os.path.join(run, 'mission.json'))
    if mission is not None:
        s['mission_states'] = [x['state'] for x in mission.get('states', [])]
        s['mission_load_ok'] = bool((mission.get('load') or {}).get('success'))
        s['mission_start_ok'] = bool((mission.get('start') or {}).get('success'))
    elif os.path.exists(os.path.join(run, 'mission_skipped.txt')):
        s['mission_states'] = 'skipped (FEED=live without LIVE_MISSION=1)'
    s['logger_runs'] = logger_runs(run)
    s['archive_sync_requests'] = archive_requests(run)
    ctx = _json(os.path.join(run, 'context.json')) or {}
    s['context'] = {k: ctx.get(k) for k in ('host', 'in_container', 'python', 'nvpmodel',
                                            'jetson_clocks_likely')}
    s['context']['ros_distro'] = (ctx.get('env') or {}).get('ROS_DISTRO')
    s['context']['image_commit'] = (ctx.get('image_sources') or {}).get('commit')
    return s


COLS = ['run', 'nodes', 'startup_total_s', 'watchdog_respawns', 'shutdown_path', 'shutdown_s',
        'launch_exit_code', 'audits_clean', 'discovery_server_left_running',
        'shm_fastrtps_before', 'shm_fastrtps_after', 'matching_unexisting_participant',
        'tracebacks_in_component_logs']


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] in ('-h', '--help') or len(argv) < 2:
        print(__doc__)
        return 0 if argv and argv[0] in ('-h', '--help') else 2
    out = [summarise(r) for r in argv[1:]]
    with open(argv[0], 'w') as f:
        json.dump(out, f, indent=2)
    print('| ' + ' | '.join(COLS + ['leftover procs', 'leftover locks', 'logger runs']) + ' |')
    print('|' + '---|' * (len(COLS) + 3))
    for s in out:
        print('| ' + ' | '.join(str(s.get(c)) for c in COLS)
              + ' | %d | %s | %s |' % (len(s['leftover_procs']),
                                       ' '.join(s['leftover_locks']) or '-',
                                       ', '.join('%s:%s' % (r['state'], r['bag_messages'])
                                                 for r in s['logger_runs']) or '-'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
