#!/usr/bin/env python3
"""CPU % (100 % = one core) and RSS of named ROS node processes over a window.

    proc_cpu.py SECONDS NODE [NODE ...]   -> one line: PROC_CPU {node: {cpu_percent, rss_mb, ...}}
A node is matched by `__node:=NODE` or by its executable's file name in the command line.
"""
import json
import os
import sys
import time

import psutil


def find(name):
    for p in psutil.process_iter(['cmdline']):
        cmd = p.info['cmdline'] or []
        if f'__node:={name}' in cmd or any(os.path.basename(c) == name for c in cmd[:2]):
            return p
    return None


def main():
    seconds = float(sys.argv[1])
    procs = {n: find(n) for n in sys.argv[2:]}
    t0 = {n: p.cpu_times() for n, p in procs.items() if p}
    w0 = time.monotonic()
    time.sleep(seconds)
    wall = time.monotonic() - w0
    out = {}
    for n, p in procs.items():
        if not p:
            out[n] = None
            continue
        try:
            c = p.cpu_times()
            used = (c.user + c.system) - (t0[n].user + t0[n].system)
            out[n] = dict(pid=p.pid, cpu_percent=round(100 * used / wall, 2),
                          rss_mb=round(p.memory_info().rss / 2**20, 1),
                          exe=' '.join(p.cmdline()[:2])[-90:])
        except psutil.Error as exc:
            out[n] = dict(error=repr(exc))
    print('PROC_CPU ' + json.dumps(out))


if __name__ == '__main__':
    main()
