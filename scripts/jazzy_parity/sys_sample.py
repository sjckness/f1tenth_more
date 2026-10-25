#!/usr/bin/env python3
"""
CPU per core, memory and per-process load over a time window, from /proc.

Runs on: the Orin host (native Humble) and inside the harness container (the
container shares the host kernel, so /proc/stat and /proc/meminfo are the
whole Orin there too; the per-process table only lists what that PID
namespace sees, which is the stack the harness started). Also linus.
Stdlib only.

Plan P9.3: "CPU per core and memory tabled" next to the topic rates, for the
native and the container stack under the same feed.

  sys_sample.py --duration S --out FILE.json [--interval 1.0]

Output:
  per_core_util_pct  {cpuN: busy %} over the window ((total - idle - iowait)
                     / total, from /proc/stat deltas)
  total_util_pct     the same for the aggregate `cpu` line
  mem                MemTotal / MemAvailable (kB) at start and end, the
                     lowest MemAvailable seen at any sample, used = total -
                     available
  gpu_load_pct       mean of the GPU load file samples (Jetson sysfs, per
                     mille / 10), when readable
  processes          every stack process (proc_audit.STACK_PATTERN): CPU %
                     of one core over the window (utime + stime delta), RSS
                     (kB) and threads at the end, node name when it has one
  by_label           the same summed per node name / executable
"""
import argparse
import collections
import glob
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import proc_audit  # noqa: E402

GPU_LOAD_FILES = ('/sys/devices/platform/gpu.0/load', '/sys/devices/gpu.0/load',
                  '/sys/devices/platform/17000000.gpu/load', '/sys/devices/17000000.ga10b/load')


def parse_proc_stat(text):
    """{'cpu': [user, nice, system, idle, iowait, irq, softirq, steal], 'cpu0': ...}."""
    out = {}
    for line in text.splitlines():
        parts = line.split()
        if parts and parts[0].startswith('cpu'):
            out[parts[0]] = [int(x) for x in parts[1:9]]
    return out


def busy_pct(a, b):
    """Busy share between two /proc/stat rows of one cpu, in percent (None if no time passed)."""
    total = sum(b) - sum(a)
    idle = (b[3] + b[4]) - (a[3] + a[4])
    if total <= 0:
        return None
    return round(100.0 * (total - idle) / total, 2)


def parse_meminfo(text):
    """{'MemTotal': kB, 'MemAvailable': kB, ...}."""
    out = {}
    for line in text.splitlines():
        key, _, rest = line.partition(':')
        parts = rest.split()
        if parts and parts[0].isdigit():
            out[key] = int(parts[0])
    return out


def proc_ticks(stat_text):
    """Return utime + stime (clock ticks) from a /proc/<pid>/stat line."""
    rest = stat_text[stat_text.rfind(')') + 2:].split()
    return int(rest[11]) + int(rest[12])


def proc_status(pid):
    """(rss_kb, threads) from /proc/<pid>/status, or (None, None)."""
    rss = threads = None
    try:
        with open('/proc/%d/status' % pid) as f:
            for line in f:
                if line.startswith('VmRSS:'):
                    rss = int(line.split()[1])
                elif line.startswith('Threads:'):
                    threads = int(line.split()[1])
    except OSError:
        pass
    return rss, threads


def read(path):
    with open(path) as f:
        return f.read()


def label_of(args):
    """Node name if it has one, else the executable's basename."""
    name = proc_audit.node_name(args)
    if name:
        return name
    words = args.split()
    for w in words:
        if '/lib/' in w or '/install/' in w or w.endswith('.launch.py'):
            return os.path.basename(w)
    return os.path.basename(words[0]) if words else '?'


def stack_ticks():
    out = {}
    for p in proc_audit.read_proc():
        if not proc_audit.is_stack(p['args'], os.getpid(), p['pid']):
            continue
        try:
            out[p['pid']] = (proc_ticks(read('/proc/%d/stat' % p['pid'])), p['args'])
        except (OSError, IndexError, ValueError):
            continue
    return out


def gpu_load():
    for path in GPU_LOAD_FILES + tuple(glob.glob('/sys/devices/platform/bus@0/*.gpu/load')):
        try:
            return int(read(path).strip()) / 10.0
        except (OSError, ValueError):
            continue
    return None


def sample(duration, interval):
    clk = os.sysconf('SC_CLK_TCK')
    stat0 = parse_proc_stat(read('/proc/stat'))
    mem0 = parse_meminfo(read('/proc/meminfo'))
    p0 = stack_ticks()
    t0 = time.monotonic()
    min_avail = mem0.get('MemAvailable')
    gpu = []
    while time.monotonic() - t0 < duration:
        time.sleep(min(interval, max(0.0, duration - (time.monotonic() - t0))))
        m = parse_meminfo(read('/proc/meminfo'))
        if m.get('MemAvailable') is not None:
            min_avail = min(min_avail, m['MemAvailable']) if min_avail else m['MemAvailable']
        g = gpu_load()
        if g is not None:
            gpu.append(g)
    wall = time.monotonic() - t0
    stat1 = parse_proc_stat(read('/proc/stat'))
    mem1 = parse_meminfo(read('/proc/meminfo'))
    p1 = stack_ticks()
    procs = []
    for pid, (ticks1, args) in sorted(p1.items()):
        if pid not in p0:
            continue  # started inside the window: no clean delta
        rss, threads = proc_status(pid)
        procs.append({'pid': pid, 'label': label_of(args),
                      'cpu_pct_of_core': round(100.0 * (ticks1 - p0[pid][0]) / clk / wall, 2),
                      'rss_kb': rss, 'threads': threads, 'args': args[:160]})
    by_label = collections.defaultdict(lambda: {'cpu_pct_of_core': 0.0, 'rss_kb': 0, 'n': 0})
    for p in procs:
        e = by_label[p['label']]
        e['cpu_pct_of_core'] = round(e['cpu_pct_of_core'] + p['cpu_pct_of_core'], 2)
        e['rss_kb'] += p['rss_kb'] or 0
        e['n'] += 1
    cores = sorted((k for k in stat1 if k != 'cpu' and k in stat0), key=lambda k: int(k[3:]))
    return {
        'duration_s': round(wall, 2),
        'per_core_util_pct': {k: busy_pct(stat0[k], stat1[k]) for k in cores},
        'total_util_pct': busy_pct(stat0['cpu'], stat1['cpu']),
        'mem': {'total_kb': mem1.get('MemTotal'),
                'available_kb_start': mem0.get('MemAvailable'),
                'available_kb_end': mem1.get('MemAvailable'),
                'available_kb_min': min_avail,
                'used_kb_end': (mem1['MemTotal'] - mem1['MemAvailable']
                                if 'MemTotal' in mem1 and 'MemAvailable' in mem1 else None)},
        'gpu_load_pct': round(sum(gpu) / len(gpu), 1) if gpu else None,
        'stack_cpu_pct_of_core_total': round(sum(p['cpu_pct_of_core'] for p in procs), 2),
        'stack_rss_kb_total': sum(p['rss_kb'] or 0 for p in procs),
        'processes': procs,
        'by_label': dict(sorted(by_label.items(), key=lambda kv: -kv[1]['cpu_pct_of_core'])),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--duration', type=float, required=True, help='window length, s')
    ap.add_argument('--interval', type=float, default=1.0, help='memory/GPU sample period, s')
    ap.add_argument('--out', required=True, help='JSON file to write')
    a = ap.parse_args(argv)
    res = sample(a.duration, a.interval)
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=2)
    print(json.dumps({'total_util_pct': res['total_util_pct'],
                      'stack_cpu_pct_of_core_total': res['stack_cpu_pct_of_core_total'],
                      'stack_rss_mb': round(res['stack_rss_kb_total'] / 1024.0, 1),
                      'mem_used_mb_end': (round(res['mem']['used_kb_end'] / 1024.0, 1)
                                          if res['mem']['used_kb_end'] else None)}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
