#!/usr/bin/env python3
"""
Pass/fail verdicts of the plan's P9 checks (D3 parity, native vs container).

Runs on: anywhere with Python 3 (reads the JSON/text the harness wrote; no
ROS). Each subcommand prints `PASS <check>: ...` or `FAIL <check>: ...` plus
the detail, and exits 0 (pass) or 1 (fail); `table` only tabulates (exit 0).

  smoke RUN_DIR...                         P9.2: per phase5_bringup.sh run: every
                                           process audit clean (no duplicate node
                                           process, no orphan, nothing left after
                                           shutdown), launch exit 0, no leftover
                                           lock, 0 "Matching unexisting
                                           participant", watchdog respawns equal
                                           to the restart test's kills (0 outside
                                           step4), and, when a mission ran, a
                                           COMPLETE mission and a complete logger
                                           run with messages in its bag
  nodes NATIVE_RUN CONTAINER_RUN [--allow NODE ...]
                                           same node names on both sides
  rates NATIVE CONTAINER [--topics T ...] [--tol 0.05]
                                           P9.3: each gate topic's rate within
                                           +-tol of native (a run dir or a rates
                                           JSON; default topics: /scan,
                                           /camera/detections, /odometry/filtered,
                                           /tf, /costmap/boundaries, /drive)
  table NATIVE_RUN CONTAINER_RUN           P9.3: CPU per core, memory, stack CPU
                                           and the per-node table, side by side
  mpc-timing NATIVE.json CONTAINER.json [--max-ratio 1.05] [--metric period_ms]
             [--label L ...] [--min-runs 3]
                                           P9.4: median over the side's replays
                                           of the metric's p95, container <=
                                           ratio x native. Each mpc_timing.py
                                           output holds one label per replay
                                           (all labels carrying the metric, or
                                           --label); fewer than --min-runs
                                           replays on a side fails. Also tables
                                           each side's run-to-run spread
                                           (max / min p95), the noise floor to
                                           read the ratio against
  golden NATIVE.txt CONTAINER.txt          P9.5: llm_golden_check.py: every fixture
                                           equal on both sides and the same sha256
  zed-depth NATIVE.json CONTAINER.json [--tol 0.02]
                                           P9.5: zed_grab ROI depth medians within
                                           2 % of native
  lease LEASE.json [--min 3.8] [--max 5.5] P9.5: every lease_probe.py trial's
                                           removal after the kill inside
                                           [min, max] s. The 5 s lease is
                                           re-announced every 1 s, so removal
                                           lands anywhere in about [4.0, 5.0] s
                                           depending on where in that cycle the
                                           kill falls (decisions.md, W3 round
                                           1); the plan's 4.4-4.7 s band is
                                           printed as a note only. The 20 s
                                           default is far outside either
  watchdog RUN_DIR...                      P9.5: 0 supervisor respawns, 0 budget
                                           exhaustions (hold / cycles runs)
  tracker JSONSTAT.json [--max-p95 50]     P9.6 / R6: tracker_ms p95 below the gate
  tests REFERENCE_IDS CONTAINER_IDS [--allow FILE]
                                           P9.1: the same failing / erroring /
                                           skipped test IDs (test_results_summary.py
                                           --ids files); --allow lists documented
                                           aarch64 / environment differences, one
                                           ID per line
"""
import argparse
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase5_summary  # noqa: E402

DEFAULT_RATE_TOPICS = ('/scan', '/camera/detections', '/odometry/filtered', '/tf',
                       '/costmap/boundaries', '/drive')


def verdict(name, ok, lines):
    print('%s %s: %s' % ('PASS' if ok else 'FAIL', name, lines[0] if lines else ''))
    for line in lines[1:]:
        print('  ' + line)
    return 0 if ok else 1


def _load(p):
    with open(p) as f:
        return json.load(f)


# ---- smoke / watchdog -------------------------------------------------------

def expected_respawns(run):
    """Process groups the restart test SIGKILLed (each one respawn)."""
    n = 0
    for d in sorted(os.listdir(run)):
        f = os.path.join(run, d, 'killed_pgids.txt')
        if d.startswith('restart_') and os.path.exists(f):
            with open(f) as fh:
                n += len(fh.read().split())
    return n


def smoke_problems(s, expected_kills):
    """Problems of one phase5_summary.summarise() record (empty list = pass)."""
    bad = []
    if s.get('audits_clean') is False:
        bad.append('process audit not clean: %s' % {k: v for k, v in s['audits'].items()
                                                    if not v['clean']})
    if s.get('audits_clean') is None:
        bad.append('no process audit found')
    if s.get('leftover_procs'):
        bad.append('%d stack processes left after shutdown' % len(s['leftover_procs']))
    if s.get('leftover_locks_live'):
        bad.append('lock files held by a live process after shutdown: %s'
                   % s['leftover_locks_live'])
    if s.get('launch_exit_code') not in (0, None):
        bad.append('launch exit code %s' % s['launch_exit_code'])
    if s.get('launch_exit_code') is None:
        bad.append('no shutdown record')
    if s.get('matching_unexisting_participant'):
        bad.append('%d "Matching unexisting participant"' % s['matching_unexisting_participant'])
    if s.get('watchdog_respawns', 0) != expected_kills:
        bad.append('watchdog respawns %d, expected %d (restart test kills)'
                   % (s.get('watchdog_respawns', 0), expected_kills))
    if s.get('watchdog_budget_exceeded'):
        bad.append('a component exceeded its restart budget')
    states = s.get('mission_states')
    if isinstance(states, list):
        if 'COMPLETE' not in states:
            bad.append('mission did not COMPLETE: %s' % states)
        good = [r for r in s.get('logger_runs', [])
                if r['state'] == 'complete' and (r['bag_messages'] or 0) > 0]
        if not good:
            bad.append('no complete logger run with messages: %s' % s.get('logger_runs'))
    return bad


def cmd_smoke(a):
    rc = 0
    for run in a.runs:
        s = phase5_summary.summarise(run)
        bad = smoke_problems(s, expected_respawns(run))
        info = 'nodes %s, startup %s s, shutdown %s (%s s), respawns %s' % (
            s['nodes'], s['startup_total_s'], s['shutdown_path'], s['shutdown_s'],
            s['watchdog_respawns'])
        notes = []
        if s['tracebacks_in_component_logs']:
            notes.append('%d tracebacks in component logs (not gated)'
                         % s['tracebacks_in_component_logs'])
        stale = sorted(set(s.get('leftover_locks', [])) - set(s.get('leftover_locks_live', [])))
        if stale:
            notes.append('stale lock files left (dead pid; reclaimed at the next start, not '
                         'gated): %s' % stale)
        if s.get('discovery_server_left_running'):
            notes.append('Discovery Server left running after the launch (by design: '
                         'supervisor_bringup starts it through `sh fastdds`; not gated)')
        if s.get('archive_sync_requests'):
            notes.append('archive sync requested and captured in the run (never run): %s'
                         % ' | '.join(s['archive_sync_requests']))
        rc |= verdict('smoke %s' % s['run'], not bad, [info] + bad + notes)
    return rc


def cmd_watchdog(a):
    rc = 0
    for run in a.runs:
        s = phase5_summary.summarise(run)
        ok = s['watchdog_respawns'] == 0 and s['watchdog_budget_exceeded'] == 0
        rc |= verdict('watchdog %s' % s['run'], ok,
                      ['respawns %d, budget exceeded %d'
                       % (s['watchdog_respawns'], s['watchdog_budget_exceeded'])])
    return rc


# ---- nodes / rates / table --------------------------------------------------

def node_set(run):
    return set(_load(os.path.join(run, 'nodes_startup.json'))['final'])


def cmd_nodes(a):
    n, c = node_set(a.native), node_set(a.container)
    allow = set(a.allow)
    only_n, only_c = sorted(n - c - allow), sorted(c - n - allow)
    lines = ['native %d nodes, container %d' % (len(n), len(c))]
    lines += ['only native: %s' % only_n] if only_n else []
    lines += ['only container: %s' % only_c] if only_c else []
    return verdict('nodes', not only_n and not only_c, lines)


def load_rates(path):
    """rates_hz from a rates JSON, or from a run dir's rates_hold.json / rates.json."""
    if os.path.isdir(path):
        for name in ('rates_hold.json', 'rates.json'):
            p = os.path.join(path, name)
            if os.path.exists(p):
                return _load(p)['rates_hz']
        raise SystemExit('no rates JSON in %s' % path)
    return _load(path)['rates_hz']


def rate_problems(native, container, topics, tol):
    """(problems, table rows) for the gate topics; rows cover every topic."""
    bad, rows = [], []
    for t in sorted(set(native) | set(container)):
        n, c = native.get(t), container.get(t)
        ratio = (c / n) if (n and c is not None) else None
        gated = t in topics
        rows.append('%-40s native %8s  container %8s  ratio %s%s' % (
            t, n, c, '%.3f' % ratio if ratio is not None else '-', '  [gate]' if gated else ''))
    for t in topics:
        n, c = native.get(t), container.get(t)
        if not n:
            bad.append('%s: no native rate (%s)' % (t, n))
        elif c is None:
            bad.append('%s: not measured in the container' % t)
        elif abs(c - n) > tol * n:
            bad.append('%s: container %.2f Hz vs native %.2f Hz (%+.1f %%, tol %.0f %%)'
                       % (t, c, n, 100.0 * (c - n) / n, 100 * tol))
    return bad, rows


def cmd_rates(a):
    bad, rows = rate_problems(load_rates(a.native), load_rates(a.container),
                              a.topics or DEFAULT_RATE_TOPICS, a.tol)
    return verdict('rates', not bad, (bad or ['every gate topic within %.0f %%' % (100 * a.tol)])
                   + rows)


def cmd_table(a):
    sides = []
    for run in (a.native, a.container):
        p = os.path.join(run, 'sys_hold.json')
        sides.append(_load(p) if os.path.exists(p) else None)
    if not all(sides):
        print('no sys_hold.json in one of the runs')
        return 1
    n, c = sides
    print('| | native | container |')
    print('|---|---:|---:|')
    for core in sorted(set(n['per_core_util_pct']) | set(c['per_core_util_pct']),
                       key=lambda k: int(k[3:])):
        print('| %s busy %% | %s | %s |'
              % (core, n['per_core_util_pct'].get(core), c['per_core_util_pct'].get(core)))
    print('| total busy %% | %s | %s |' % (n['total_util_pct'], c['total_util_pct']))
    print('| stack CPU (%% of one core) | %s | %s |'
          % (n['stack_cpu_pct_of_core_total'], c['stack_cpu_pct_of_core_total']))
    print('| stack RSS MB | %.1f | %.1f |'
          % (n['stack_rss_kb_total'] / 1024.0, c['stack_rss_kb_total'] / 1024.0))
    print('| memory used MB (end) | %s | %s |' % (
        round(n['mem']['used_kb_end'] / 1024.0, 1) if n['mem']['used_kb_end'] else None,
        round(c['mem']['used_kb_end'] / 1024.0, 1) if c['mem']['used_kb_end'] else None))
    print('| GPU load %% | %s | %s |' % (n.get('gpu_load_pct'), c.get('gpu_load_pct')))
    print()
    print('| node / process | native CPU % | container CPU % | native RSS MB | container RSS MB |')
    print('|---|---:|---:|---:|---:|')
    for label in sorted(set(n['by_label']) | set(c['by_label'])):
        bn, bc = n['by_label'].get(label, {}), c['by_label'].get(label, {})
        print('| %s | %s | %s | %s | %s |' % (
            label, bn.get('cpu_pct_of_core'), bc.get('cpu_pct_of_core'),
            round(bn['rss_kb'] / 1024.0, 1) if bn else None,
            round(bc['rss_kb'] / 1024.0, 1) if bc else None))
    return 0


# ---- mpc timing -------------------------------------------------------------

def replay_p95(res, labels, metric):
    """[(label, p95)] of every replay (label) in an mpc_timing.py output carrying metric."""
    out = []
    for k in (labels or sorted(res)):
        s = res.get(k, {}).get(metric)
        if s and s.get('n', 1) and s.get('p95') is not None:
            out.append((k, float(s['p95'])))
    return out


def spread(vals):
    """Run-to-run spread of one side: max / min (nan for fewer than 2 values)."""
    v = [x for _, x in vals]
    return max(v) / min(v) if len(v) > 1 and min(v) > 0 else float('nan')


def cmd_mpc_timing(a):
    rn, rc = _load(a.native), _load(a.container)
    n, c = replay_p95(rn, a.label, a.metric), replay_p95(rc, a.label, a.metric)
    bad = ['%s: %d replay(s) carry %s, need >= %d (--min-runs)' % (side, len(v), a.metric,
                                                                   a.min_runs)
           for side, v in (('native', n), ('container', c)) if len(v) < a.min_runs]
    lines = []
    if n and c:
        mn, mc = statistics.median(x for _, x in n), statistics.median(x for _, x in c)
        ratio = mc / mn if mn else float('nan')
        if not mc <= a.max_ratio * mn:
            bad.append('median %s p95 ratio %.3f > %.2f' % (a.metric, ratio, a.max_ratio))
        lines.append('%s median p95 over replays: container %.2f ms (%d) vs native %.2f ms (%d):'
                     ' ratio %.3f, gate %.2f'
                     % (a.metric, mc, len(c), mn, len(n), ratio, a.max_ratio))
    for side, v in (('native', n), ('container', c)):
        lines.append('%-9s p95 per replay: %s; run-to-run spread max/min %.3f'
                     % (side, ', '.join('%s %.2f' % kv for kv in v) or '-', spread(v)))
    for key in ('solve_ms', 'period_ms'):
        if key == a.metric:
            continue
        on, oc = replay_p95(rn, None, key), replay_p95(rc, None, key)
        if on and oc:
            lines.append('%s median p95 (not gated): native %.2f (%d), container %.2f (%d)'
                         % (key, statistics.median(x for _, x in on), len(on),
                            statistics.median(x for _, x in oc), len(oc)))
    return verdict('mpc-timing', not bad, bad + lines if bad else lines)


# ---- golden -----------------------------------------------------------------

def parse_golden(text):
    """{fixture: (equal, sha)} and the summary (ok, total) from llm_golden_check.py."""
    rows, summary = {}, None
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1].startswith('equal=') and parts[2].startswith('sha256='):
            rows[parts[0]] = (parts[1] == 'equal=True', parts[2][7:])
        elif 'golden missions reproduced exactly' in line:
            frac = line.split(':')[1].split()[0]
            summary = tuple(int(x) for x in frac.split('/'))
    return rows, summary


def cmd_golden(a):
    with open(a.native) as f:
        rn, sn = parse_golden(f.read())
    with open(a.container) as f:
        rc, sc = parse_golden(f.read())
    bad = []
    for side, rows, s in (('native', rn, sn), ('container', rc, sc)):
        if not rows or s is None or s[0] != s[1]:
            bad.append('%s: %s fixtures reproduced' % (side, '%d/%d' % s if s else 'no'))
    for k in sorted(set(rn) | set(rc)):
        if k not in rn or k not in rc:
            bad.append('%s only on one side' % k)
        elif rn[k][1] != rc[k][1]:
            bad.append('%s: sha256 %s (native) vs %s (container)' % (k, rn[k][1], rc[k][1]))
    return verdict('golden', not bad, (bad or ['%d/%d fixtures, identical output on both sides'
                                               % sc]))


# ---- zed depth / lease / tracker -------------------------------------------

def zed_medians(res):
    return {r['name']: r['median_m'] for r in res.get('rois', [])}


def cmd_zed_depth(a):
    n, c = zed_medians(_load(a.native)), zed_medians(_load(a.container))
    bad, lines = [], []
    for k in sorted(set(n) | set(c)):
        vn, vc = n.get(k), c.get(k)
        if vn is None or vc is None:
            bad.append('%s: median missing (native %s, container %s)' % (k, vn, vc))
            continue
        rel = abs(vc - vn) / vn if vn else float('inf')
        lines.append('%-10s native %.4f m  container %.4f m  %+.2f %%'
                     % (k, vn, vc, 100 * (vc - vn) / vn))
        if rel > a.tol:
            bad.append('%s: %.2f %% apart (tol %.1f %%)' % (k, 100 * rel, 100 * a.tol))
    if not n:
        bad.append('no ROI medians in the native result')
    return verdict('zed-depth', not bad, (bad or ['every ROI median within %.1f %%'
                                                  % (100 * a.tol)]) + lines)


# The plan's P9.5 band. The gate is wider (see the docstring); trials outside
# this band are only noted.
PLAN_LEASE_BAND = (4.4, 4.7)


def cmd_lease(a):
    res = _load(a.file)
    vals = res.get('removal_after_kill_s') or []
    bad = [v for v in vals if v is None or not (a.min <= v <= a.max)]
    lines = ['%d trials, removal after kill %s s (gate %.1f..%.1f s), profile %s'
             % (len(vals), vals, a.min, a.max, res.get('profile'))]
    lo, hi = PLAN_LEASE_BAND
    off = [v for v in vals if v is not None and a.min <= v <= a.max and not lo <= v <= hi]
    if off:
        lines.append("note: %d of %d trials outside the plan's %.1f-%.1f s band: %s "
                     '(the 1 s announcement phase; not gated)'
                     % (len(off), len(vals), lo, hi, off))
    return verdict('lease', bool(vals) and not bad, lines)


def cmd_tracker(a):
    res = _load(a.file)
    st = res.get('stats', {})
    ok = st.get('n', 0) > 0 and st['p95'] < a.max_p95
    return verdict('tracker', ok, ['%s %s: n %s p50 %s p95 %s max %s (gate p95 < %s)'
                                   % (res.get('topic'), res.get('key'), st.get('n'),
                                      st.get('p50'), st.get('p95'), st.get('max'), a.max_p95)])


# ---- tests ------------------------------------------------------------------

def read_ids(path):
    with open(path) as f:
        return {line.strip() for line in f if line.strip() and not line.startswith('#')}


def cmd_tests(a):
    ref, got = read_ids(a.reference), read_ids(a.container)
    allow = read_ids(a.allow) if a.allow else set()
    only_ref, only_got = sorted(ref - got - allow), sorted(got - ref - allow)
    lines = ['reference %d IDs, container %d, allowed differences %d'
             % (len(ref), len(got), len(allow))]
    lines += ['only reference: ' + x for x in only_ref]
    lines += ['only container: ' + x for x in only_got]
    return verdict('tests', not only_ref and not only_got, lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  smoke'):])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('smoke')
    p.add_argument('runs', nargs='+')
    p = sub.add_parser('watchdog')
    p.add_argument('runs', nargs='+')
    p = sub.add_parser('nodes')
    p.add_argument('native')
    p.add_argument('container')
    p.add_argument('--allow', nargs='*', default=[])
    p = sub.add_parser('rates')
    p.add_argument('native')
    p.add_argument('container')
    p.add_argument('--topics', nargs='*')
    p.add_argument('--tol', type=float, default=0.05)
    p = sub.add_parser('table')
    p.add_argument('native')
    p.add_argument('container')
    p = sub.add_parser('mpc-timing')
    p.add_argument('native')
    p.add_argument('container')
    p.add_argument('--label', nargs='+', help='replays to use (default: every label)')
    p.add_argument('--metric', default='period_ms')
    p.add_argument('--min-runs', type=int, default=3)
    p.add_argument('--max-ratio', type=float, default=1.05)
    p = sub.add_parser('golden')
    p.add_argument('native')
    p.add_argument('container')
    p = sub.add_parser('zed-depth')
    p.add_argument('native')
    p.add_argument('container')
    p.add_argument('--tol', type=float, default=0.02)
    p = sub.add_parser('lease')
    p.add_argument('file')
    p.add_argument('--min', type=float, default=3.8)
    p.add_argument('--max', type=float, default=5.5)
    p = sub.add_parser('tracker')
    p.add_argument('file')
    p.add_argument('--max-p95', type=float, default=50.0)
    p = sub.add_parser('tests')
    p.add_argument('reference')
    p.add_argument('container')
    p.add_argument('--allow')
    a = ap.parse_args(argv)
    return {'smoke': cmd_smoke, 'watchdog': cmd_watchdog, 'nodes': cmd_nodes,
            'rates': cmd_rates, 'table': cmd_table, 'mpc-timing': cmd_mpc_timing,
            'golden': cmd_golden, 'zed-depth': cmd_zed_depth, 'lease': cmd_lease,
            'tracker': cmd_tracker, 'tests': cmd_tests}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main())
