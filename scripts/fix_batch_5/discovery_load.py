#!/usr/bin/env python3
"""Fix batch 5 (H1, lease fix): discovery CPU and traffic of a running stack,
without root.

  discovery_load.py sample --out FILE
      One snapshot: CPU time (utime + stime, seconds) of every thread of every
      process on ROS_DOMAIN_ID (default: $ROS_DOMAIN_ID), summed per thread
      class, plus the Discovery Server process, plus the host's UDP counters
      (/proc/net/snmp) and loopback counters.
  discovery_load.py diff A B --out FILE
      B minus A, per class, per second of wall time between them. Only
      processes alive in both snapshots count (a process that started or
      exited in between would add or drop its whole lifetime).

Thread classes (Fast DDS 2.14 names its threads):
  dds.ev     the event thread: participant announcements, lease checks,
             heartbeats and their timers -- what the lease profile changes;
  dds.udp    UDP receive threads: discovery (metatraffic) and any UDP data;
  dds.shm    shared-memory receive threads: same-host data, and same-host
             metatraffic unicast;
  dds.other  dds.asyn (async writer), dds.shm.wdog, ...;
  app        everything else (the node's own executor threads).
The Discovery Server runs no node code, so its whole CPU is discovery.
The supervisor's whole CPU is reported separately (the liveness watchdog
runs in it).
"""
import argparse
import json
import os
import time

CLK = os.sysconf('SC_CLK_TCK')


def _thread_class(comm):
    if comm.startswith('dds.ev'):
        return 'dds.ev'
    if comm.startswith('dds.udp'):
        return 'dds.udp'
    if comm.startswith('dds.shm') and not comm.startswith('dds.shm.wdog'):
        return 'dds.shm'
    if comm.startswith('dds.'):
        return 'dds.other'
    return 'app'


def _environ(pid):
    try:
        with open(f'/proc/{pid}/environ', 'rb') as f:
            return dict(kv.split(b'=', 1) for kv in f.read().split(b'\0') if b'=' in kv)
    except OSError:
        return None


def _cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode(errors='replace')
    except OSError:
        return ''


def _task_cpu(pid, tid):
    with open(f'/proc/{pid}/task/{tid}/stat') as f:
        s = f.read()
    comm = s[s.index('(') + 1:s.rindex(')')]
    fields = s[s.rindex(')') + 2:].split()
    return comm, (int(fields[11]) + int(fields[12])) / CLK


def _snmp_udp():
    with open('/proc/net/snmp') as f:
        rows = [l.split() for l in f if l.startswith('Udp:')]
    return {k: int(v) for k, v in zip(rows[0][1:], rows[1][1:])}


def _lo():
    base = '/sys/class/net/lo/statistics/'
    return {k: int(open(base + k).read()) for k in ('rx_packets', 'rx_bytes')}


def sample(domain):
    procs = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        cmd = _cmdline(pid)
        is_ds = 'fast-discovery-server' in cmd
        env = _environ(pid)
        if env is None:
            continue
        if not is_ds and env.get(b'ROS_DOMAIN_ID') != domain.encode():
            continue
        # Only ROS participants: a process with at least one dds.* thread,
        # or the server itself.
        classes = {}
        try:
            for tid in os.listdir(f'/proc/{pid}/task'):
                comm, cpu = _task_cpu(pid, tid)
                c = _thread_class(comm)
                classes[c] = classes.get(c, 0.0) + cpu
        except OSError:
            continue
        if not is_ds and not any(c.startswith('dds') for c in classes):
            continue
        procs[pid] = {'cmd': cmd[:160], 'ds': is_ds, 'cpu': classes}
    return {'t': time.time(), 'procs': procs, 'udp': _snmp_udp(), 'lo': _lo()}


def diff(a, b):
    dt = b['t'] - a['t']
    out = {'window_s': round(dt, 2), 'participants': 0, 'cpu_s_per_s': {}, 'ds_cpu_s_per_s': 0.0,
           'supervisor_cpu_s_per_s': 0.0}
    for pid, pb in b['procs'].items():
        pa = a['procs'].get(pid)
        if pa is None or pa['cmd'] != pb['cmd']:
            continue
        if pb['ds']:
            out['ds_cpu_s_per_s'] += sum(pb['cpu'].values()) - sum(pa['cpu'].values())
            continue
        out['participants'] += 1
        if 'component_supervisor_node' in pb['cmd']:
            out['supervisor_cpu_s_per_s'] += sum(pb['cpu'].values()) - sum(pa['cpu'].values())
        for c, v in pb['cpu'].items():
            out['cpu_s_per_s'][c] = out['cpu_s_per_s'].get(c, 0.0) + v - pa['cpu'].get(c, 0.0)
    out['cpu_s_per_s'] = {c: round(v / dt, 4) for c, v in sorted(out['cpu_s_per_s'].items())}
    out['ds_cpu_s_per_s'] = round(out['ds_cpu_s_per_s'] / dt, 4)
    out['supervisor_cpu_s_per_s'] = round(out['supervisor_cpu_s_per_s'] / dt, 4)
    out['udp_per_s'] = {k: round((b['udp'][k] - a['udp'][k]) / dt, 1)
                        for k in ('InDatagrams', 'OutDatagrams', 'RcvbufErrors', 'InErrors')}
    out['lo_per_s'] = {k: round((b['lo'][k] - a['lo'][k]) / dt, 1) for k in b['lo']}
    return out


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('sample')
    s.add_argument('--out', required=True)
    s.add_argument('--domain', default=os.environ.get('ROS_DOMAIN_ID', '0'))
    d = sub.add_parser('diff')
    d.add_argument('a')
    d.add_argument('b')
    d.add_argument('--out', required=True)
    a = ap.parse_args()
    if a.cmd == 'sample':
        res = sample(a.domain)
    else:
        res = diff(json.load(open(a.a)), json.load(open(a.b)))
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
