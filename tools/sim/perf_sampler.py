#!/usr/bin/env python3
"""Sample CPU / RAM / VRAM of the stack and the Isaac container until killed (SIGTERM/SIGINT).

    python3 tools/sim/perf_sampler.py --out perf.json [--period 5] [--container f1t_isaac]

Stack = every process whose command line contains one of --stack-match (default: this
workspace's install/, the user-space overlay, /opt/ros/jazzy); per-process CPU from
psutil deltas over each period (100 % = one core). Isaac = the container's processes
(docker top -> pids, same accounting). GPU from nvidia-smi. Writes a summary on exit.
"""
import argparse
import json
import os
import signal
import statistics
import subprocess
import time

import psutil

_stop = False


def _handler(_s, _f):
    global _stop
    _stop = True


def _container_pids(name):
    try:
        out = subprocess.run(['docker', 'top', name, '-eo', 'pid'], capture_output=True,
                             text=True, timeout=10).stdout.split()[1:]
        return {int(p) for p in out if p.isdigit()}
    except Exception:  # noqa: BLE001 - container gone: no pids
        return set()


def _gpu():
    try:
        out = subprocess.run(['nvidia-smi', '--query-gpu=memory.used,memory.total,utilization.gpu',
                              '--format=csv,noheader,nounits'], capture_output=True, text=True,
                             timeout=10).stdout.strip().splitlines()[0]
        used, total, util = (float(x) for x in out.split(','))
        return {'vram_used_mib': used, 'vram_total_mib': total, 'gpu_util_pct': util}
    except Exception:  # noqa: BLE001
        return {}


def _summ(xs):
    if not xs:
        return None
    s = sorted(xs)
    return {'mean': round(statistics.mean(s), 1), 'p95': round(s[min(len(s) - 1,
                                                                    int(0.95 * len(s)))], 1),
            'max': round(s[-1], 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--period', type=float, default=5.0)
    ap.add_argument('--container', default='f1t_isaac')
    ap.add_argument('--stack-match', nargs='*', default=[
        os.path.expanduser('~/dev_ws/f1tenth_more_jc/install'),
        os.path.expanduser('~/dev_ws/jc_deps_ws/install'), '/opt/ros/jazzy'])
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)

    procs = {}
    samples = []
    psutil.cpu_percent(None)
    me = os.getpid()
    while not _stop:
        cpids = _container_pids(a.container)
        for p in psutil.process_iter(['pid', 'cmdline']):
            if p.pid in procs or p.pid == me:
                continue
            cmd = ' '.join(p.info['cmdline'] or [])
            if p.pid in cpids:
                grp = 'isaac'
            elif any(m in cmd for m in a.stack_match) and 'perf_sampler' not in cmd \
                    and 'e2e_checks' not in cmd:
                grp = 'stack'
            else:
                continue
            try:
                p.cpu_percent(None)
                procs[p.pid] = (p, grp, cmd[:160])
            except psutil.Error:
                pass
        end = time.time() + a.period
        while time.time() < end and not _stop:
            time.sleep(0.2)
        if _stop:
            break
        tot = {'stack': [0.0, 0.0, 0], 'isaac': [0.0, 0.0, 0]}
        top = []
        for pid, (p, grp, cmd) in list(procs.items()):
            try:
                c = p.cpu_percent(None)
                r = p.memory_info().rss / 2**20
            except psutil.Error:
                procs.pop(pid)
                continue
            tot[grp][0] += c
            tot[grp][1] += r
            tot[grp][2] += 1
            top.append((c, grp, cmd))
        vm = psutil.virtual_memory()
        samples.append({'t': round(time.time(), 1),
                        'stack_cpu_pct': round(tot['stack'][0], 1),
                        'stack_rss_mib': round(tot['stack'][1]),
                        'stack_procs': tot['stack'][2],
                        'isaac_cpu_pct': round(tot['isaac'][0], 1),
                        'isaac_rss_mib': round(tot['isaac'][1]),
                        'host_cpu_pct': psutil.cpu_percent(None),
                        'host_ram_used_mib': round((vm.total - vm.available) / 2**20),
                        **_gpu(),
                        'top5': [(round(c, 1), g, cmd) for c, g, cmd in
                                 sorted(top, reverse=True)[:5]]})
    keys = ['stack_cpu_pct', 'stack_rss_mib', 'isaac_cpu_pct', 'isaac_rss_mib', 'host_cpu_pct',
            'host_ram_used_mib', 'vram_used_mib', 'gpu_util_pct']
    summary = {k: _summ([s[k] for s in samples if k in s]) for k in keys}
    summary['samples'] = len(samples)
    summary['cpus'] = psutil.cpu_count()
    summary['ram_total_mib'] = round(psutil.virtual_memory().total / 2**20)
    with open(a.out, 'w') as f:
        json.dump({'summary': summary, 'samples': samples}, f, indent=1)


if __name__ == '__main__':
    main()
