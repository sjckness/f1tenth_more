#!/usr/bin/env python3
"""Fix batch 5: evaluate watchdog_live.sh's runs.

  watchdog_live_summary.py output/fix_batch_5/watchdog_live > summary.json

Reads, per test directory: launch.log (the supervisor's [health] lines, with
their epoch timestamps), health.jsonl (health_recorder.py), and the t_*.txt
marks the driver wrote. Prints one JSON object with the verdict inputs of
each test; the report states the verdicts.
"""
import json
import os
import re
import sys

LOG_RE = re.compile(r'\[(INFO|WARN|ERROR)\] \[(\d+\.\d+)\] \[component_supervisor_node\]: \[health\] (.*)')


def health_log(run):
    out = []
    path = os.path.join(run, 'launch.log')
    if not os.path.exists(path):
        return out
    for line in open(path, errors='replace'):
        m = LOG_RE.search(line)
        if m:
            out.append((float(m.group(2)), m.group(1), m.group(3).strip()))
    return out


def mark(run, name):
    p = os.path.join(run, f't_{name}.txt')
    return float(open(p).read()) if os.path.exists(p) else None


def changes(run, component=None):
    p = os.path.join(run, 'health.jsonl')
    if not os.path.exists(p):
        return []
    rows = [json.loads(l) for l in open(p) if l.strip()]
    return [r for r in rows if 'component' in r and (component is None or r['component'] == component)]


def status_of(row):
    return row['message'].split(':', 1)[0]


def between(rows, t0, t1):
    return [r for r in rows if (t0 is None or r[0] >= t0) and (t1 is None or r[0] < t1)]


def failures(log):
    return [r for r in log if 'LIVENESS FAILURE' in r[2] or 'FAILED' in r[2]]


def normal(run):
    log = health_log(run)
    t_ready, t1_end = mark(run, 'ready'), mark(run, 'test1_end')
    t_stop, t_feed, t_end = mark(run, 'sigstop'), mark(run, 'feed_stop'), mark(run, 'end')
    res = {'test1_window_s': round(t1_end - t_ready, 1) if t_ready and t1_end else None}
    res['test1_failures'] = [r[2] for r in failures(between(log, None, t1_end))]
    res['test1_non_ok_changes'] = [
        (round(r['t'] - t_ready, 1), r['component'], r['message'])
        for r in changes(run) if t_ready <= r['t'] < t1_end and status_of(r) not in ('OK',)]
    sw = [r for r in changes(run, 'swept_clearance') if t_stop and r['t'] >= t_stop]
    first = {}
    for r in sw:
        first.setdefault(status_of(r), round(r['t'] - t_stop, 2))
    res['test2_swept_timeline_s_after_sigstop'] = [
        (round(r['t'] - t_stop, 2), r['message']) for r in sw if t_feed is None or r['t'] < t_feed]
    res['test2_first'] = first
    res['test2_log'] = [(round(t - t_stop, 2), lvl, msg) for t, lvl, msg in between(log, t_stop, t_feed)]
    for name in ('clock_info', 'sigstop_after'):
        p = os.path.join(run, f'{name}.txt')
        res[name] = open(p).read().strip() if os.path.exists(p) else None
    others = [r[2] for r in failures(between(log, t_stop, t_feed)) if 'swept_clearance' not in r[2]]
    res['test2_other_component_failures'] = others
    res['feedstop_failures'] = [r[2] for r in failures(between(log, t_feed, None))]
    if t_feed and t_end:
        last = {}
        for r in changes(run):
            if r['t'] < t_end:
                last[r['component']] = r['message']
        res['feedstop_status_at_end'] = last
    p = os.path.join(run, 'ekf_stamps.json')
    if os.path.exists(p):
        res['ekf_stamps'] = {t: [(b[0], b[1]) for b in v['bins']] for t, v in json.load(open(p)).items()}
    return res


def failing(run):
    log = health_log(run)
    t_ready = mark(run, 'ready')
    sw = changes(run, 'swept_clearance')
    t0 = sw[0]['t'] if sw else 0.0
    return {
        'log': [(round(t - t0, 1), lvl, msg) for t, lvl, msg in log
                if 'swept_clearance' in msg],
        'timeline': [(round(r['t'] - t0, 1), r['message'][:120], r['values'].get('watchdog_restarts'))
                     for r in sw],
        'final': sw[-1]['message'] if sw else None,
        'restarts_logged': sum('restarted by the liveness watchdog' in m for _, _, m in log),
        'other_component_failures': [m for _, _, m in failures(log) if 'swept_clearance' not in m],
        't_ready_rel': round(t_ready - t0, 1) if t_ready else None,
    }


def sim(run):
    log = health_log(run)
    t_pause, t_resume = mark(run, 'pause'), mark(run, 'resume')
    return {
        'failures': [m for _, _, m in failures(log)],
        'clock_log': [(round(t - t_pause, 2), msg) for t, _, msg in log if 'sim clock' in msg],
        'changes_around_pause': [
            (round(r['t'] - t_pause, 1), r['component'], r['message'][:100])
            for r in changes(run) if t_pause - 5 <= r['t'] <= t_resume + 30],
        'pause_window_s': round(t_resume - t_pause, 1),
    }


def main():
    root = sys.argv[1]
    out = {}
    for name, fn in (('normal', normal), ('failing', failing), ('sim', sim)):
        run = os.path.join(root, name)
        if os.path.isdir(run):
            out[name] = fn(run)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
