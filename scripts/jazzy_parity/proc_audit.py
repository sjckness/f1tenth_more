#!/usr/bin/env python3
"""
Process-table snapshot and audit of a running (or stopped) stack.

Runs on: the Orin host (native Humble), inside the harness container, linus.
Stdlib only; reads /proc directly (no `ps` column-width questions).

Plan P9.2 / backlog M23: after the 10 bring-up cycles and after the 5 min
hold there must be no duplicate node processes (one process per node name)
and no orphans (a stack process whose launch is gone). This is the check.

  proc_audit.py snapshot --out-all ALL.txt --out-stack STACK.txt
      One read of /proc. ALL.txt: every process; STACK.txt: the ones that
      belong to a ROS stack (STACK_PATTERN, minus the harness's own tools,
      HARNESS_EXCLUDE). Both as `pid pgid ppid etimes args`, the format
      phase5_bringup.sh has always written.
  proc_audit.py audit --procs-all ALL.txt [--launch-pid PID] [--expect-none]
                      [--out FILE.json]
      duplicates: a node name (`__ns:=` + `__node:=` on the command line), or
                  an identical command line once launch_ros's per-run
                  /tmp/launch_params_* files are masked, held by more than
                  one process;
      orphans:    with --launch-pid, every stack process that does not
                  descend from that launch (by ppid) -- a leftover of an
                  earlier run, or a component the supervisor lost track of;
                  without it, the stack processes reparented to pid 1;
      --expect-none: after a shutdown, any stack process at all is a
                  leftover (the Discovery Server is reported separately).
      Exit 0 when clean, 1 when anything was found.
  proc_audit.py sweep --pgid PGID [PGID ...]
      SIGKILL each process group that still exists, but only when every
      member is a stack process (a recycled group id of an unrelated
      process is left alone). Run AFTER the audit, so a leftover is first
      recorded, then removed before the next cycle.
"""
import argparse
import collections
import json
import os
import re
import signal
import sys

STACK_PATTERN = re.compile(
    r'ros2 launch|/install/|/opt/ros/[a-z]+/lib/|fast-discovery-server|fastdds(\.py)? discovery'
    r'|ensure_discovery_server|llama-server')
HARNESS_EXCLUDE = re.compile(
    r'stack_probe\.py|ros2cli/daemon|_ros2_daemon|phase5_bringup|ros2 bag (play|record)'
    r'|graph_stub_node\.py|restamp_play\.py|isolation_check\.py|proc_audit\.py'
    r'|sys_sample\.py|lease_probe\.py|dds_probe\.py|run_context\.py|replay_localization'
    r'|logger_check\.sh|diag_start_check\.sh|ds_probe\.sh|mpc_capture_node\.py'
    r'|bt_mission_driver\.py|publish_mission_status\.py|publish_logger_topics\.py'
    r'|\bgrep\b|colcon')
DS_PATTERN = re.compile(r'fast-discovery-server|fastdds(\.py)? discovery|ensure_discovery_server')
BUILD_TOOLS = frozenset(('rustc', 'cargo', 'cmake', 'make', 'gmake', 'ninja', 'cc1', 'cc1plus',
                         'as', 'ld', 'collect2', 'gcc', 'g++', 'cc', 'c++', 'clang', 'clang++'))
_PARAMS_TMP = re.compile(r'/tmp/launch_params_\w+')
_NODE = re.compile(r'__node:=(\S+)')
_NS = re.compile(r'__ns:=(\S+)')


def read_proc(root='/proc'):
    """Every process as a dict(pid, pgid, ppid, etimes, args), from /proc."""
    try:
        clk = os.sysconf('SC_CLK_TCK')
        with open(os.path.join(root, 'uptime')) as f:
            uptime = float(f.read().split()[0])
    except (OSError, ValueError):
        clk, uptime = 100, None
    out = []
    for name in os.listdir(root):
        if not name.isdigit():
            continue
        try:
            with open(os.path.join(root, name, 'stat')) as f:
                stat = f.read()
            with open(os.path.join(root, name, 'cmdline'), 'rb') as f:
                cmdline = f.read()
        except OSError:
            continue  # gone meanwhile, or not ours to read
        # comm may contain spaces and parentheses: split after the LAST ')'.
        rest = stat[stat.rfind(')') + 2:].split()
        ppid, pgid, start = int(rest[1]), int(rest[2]), int(rest[19])
        # One line per process: a `bash -c '<script>'` argument can hold newlines.
        args = cmdline.replace(b'\0', b' ').decode(errors='replace')
        args = args.replace('\r', ' ').replace('\n', ' ').strip()
        if not args:
            args = '[' + stat[stat.find('(') + 1:stat.rfind(')')] + ']'
        etimes = int(uptime - start / clk) if uptime is not None else -1
        out.append({'pid': int(name), 'pgid': pgid, 'ppid': ppid, 'etimes': etimes,
                    'args': args})
    out.sort(key=lambda p: p['pid'])
    return out


def format_line(p):
    """One process in phase5_bringup.sh's `pid pgid ppid etimes args` format."""
    return '%d %d %d %d %s' % (p['pid'], p['pgid'], p['ppid'], p['etimes'], p['args'])


def parse_lines(lines):
    """Inverse of format_line; blank and malformed lines skipped."""
    out = []
    for line in lines:
        parts = line.split(None, 4)
        if len(parts) < 4 or not all(re.match(r'-?\d+$', x) for x in parts[:4]):
            continue
        out.append({'pid': int(parts[0]), 'pgid': int(parts[1]), 'ppid': int(parts[2]),
                    'etimes': int(parts[3]), 'args': parts[4].rstrip('\n') if len(parts) > 4
                    else ''})
    return out


def _ignore_pattern():
    """PARITY_IGNORE (env, a regex; linus only): other worktrees' isolated runs."""
    text = os.environ.get('PARITY_IGNORE', '')
    return re.compile(text) if text else None


def is_stack(args, self_pid=None, pid=None, ignore=None):
    """
    Whether a command line belongs to a ROS stack and not to this harness.

    Compilers and build tools are never part of a stack, whatever paths their
    command lines name (a colcon build's `-I .../install/...`).
    """
    if pid is not None and pid == self_pid:
        return False
    words = args.split()
    if words and os.path.basename(words[0]) in BUILD_TOOLS:
        return False
    if ignore is None:
        ignore = _ignore_pattern()
    if ignore is not None and ignore.search(args):
        return False
    return bool(STACK_PATTERN.search(args)) and not HARNESS_EXCLUDE.search(args)


def node_name(args):
    """'/ns/name' from launch_ros's remap arguments, or None for an unnamed process."""
    m = _NODE.search(args)
    if not m:
        return None
    ns = _NS.search(args)
    prefix = ns.group(1).rstrip('/') if ns else ''
    return prefix + '/' + m.group(1)


def normalize(args):
    """Command line with the per-run launch parameter files masked."""
    return _PARAMS_TMP.sub('<PARAMS_TMP>', args)


def descendants(procs, root):
    """Pids that descend from root (root included), by the ppid chain."""
    children = collections.defaultdict(list)
    for p in procs:
        children[p['ppid']].append(p['pid'])
    seen, todo = set(), [root]
    while todo:
        pid = todo.pop()
        if pid in seen:
            continue
        seen.add(pid)
        todo.extend(children.get(pid, []))
    return seen


def audit(procs, launch_pid=None, expect_none=False):
    """Duplicates, orphans and (expect_none) leftovers among the stack processes."""
    stack = [p for p in procs if is_stack(p['args'])]
    by_name = collections.defaultdict(list)
    by_args = collections.defaultdict(list)
    for p in stack:
        name = node_name(p['args'])
        if name:
            by_name[name].append(p['pid'])
        by_args[normalize(p['args'])].append(p['pid'])
    dup_names = {n: sorted(pids) for n, pids in by_name.items() if len(pids) > 1}
    dup_args = {a[:200]: sorted(pids) for a, pids in by_args.items() if len(pids) > 1}
    # The Discovery Server is not a node: supervisor_bringup.launch.py starts it
    # through `sh fastdds`, which does not pass SIGINT on, so the server
    # outlives every launch by design (reused by the next one, or stopped by
    # f1tenth_ctl `discovery down`). Reported on its own, never as an orphan.
    nodes = [p for p in stack if not DS_PATTERN.search(p['args'])]
    if launch_pid:
        tree = descendants(procs, launch_pid)
        orphans = [p for p in nodes if p['pid'] not in tree]
        orphan_rule = 'not a descendant of launch pid %d' % launch_pid
    else:
        orphans = [p for p in nodes if p['ppid'] == 1]
        orphan_rule = 'reparented to pid 1'
    res = {
        'n_stack_processes': len(stack),
        'n_named_nodes': len(by_name),
        'duplicate_node_names': dup_names,
        'duplicate_command_lines': dup_args,
        'orphan_rule': orphan_rule,
        'orphans': [format_line(p)[:240] for p in orphans],
        'discovery_server_processes': [format_line(p)[:200] for p in stack
                                       if DS_PATTERN.search(p['args'])],
    }
    if expect_none:
        res['leftovers'] = [format_line(p)[:240] for p in stack
                            if not DS_PATTERN.search(p['args'])]
    res['clean'] = not (dup_names or dup_args or orphans or res.get('leftovers'))
    return res


def cmd_snapshot(a):
    procs = read_proc()
    me = os.getpid()
    stack = [p for p in procs if is_stack(p['args'], me, p['pid'])]
    if a.out_all:
        with open(a.out_all, 'w') as f:
            f.writelines(format_line(p) + '\n' for p in procs if p['pid'] != me)
    if a.out_stack:
        with open(a.out_stack, 'w') as f:
            f.writelines(format_line(p) + '\n' for p in stack)
    else:
        sys.stdout.writelines(format_line(p) + '\n' for p in stack)
    return 0


def cmd_audit(a):
    with open(a.procs_all) as f:
        procs = parse_lines(f)
    res = audit(procs, a.launch_pid, a.expect_none)
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(res, f, indent=2)
    summary = {k: res[k] for k in ('clean', 'n_stack_processes', 'n_named_nodes')}
    summary['duplicates'] = (len(res['duplicate_node_names'])
                             + len(res['duplicate_command_lines']))
    summary['orphans'] = len(res['orphans'])
    summary['leftovers'] = len(res.get('leftovers', []))
    print(json.dumps(summary))
    return 0 if res['clean'] else 1


def sweep_plan(procs, pgids):
    """{pgid: 'kill' | 'gone' | 'foreign: <args>'} for the requested groups."""
    plan = {}
    for g in pgids:
        members = [p for p in procs if p['pgid'] == g]
        if not members:
            plan[g] = 'gone'
            continue
        foreign = [p for p in members if not is_stack(p['args'])]
        plan[g] = 'foreign: ' + foreign[0]['args'][:120] if foreign else 'kill'
    return plan


def cmd_sweep(a):
    plan = sweep_plan(read_proc(), a.pgid)
    for g, what in sorted(plan.items()):
        if what == 'kill':
            try:
                os.killpg(g, signal.SIGKILL)
            except ProcessLookupError:
                what = 'gone'
            except PermissionError:
                what = 'not ours (EPERM)'
        print('pgid %d: %s' % (g, what))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  proc_audit.py'):])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('snapshot', help='read /proc once, write ALL and STACK lists')
    p.add_argument('--out-all')
    p.add_argument('--out-stack')
    p = sub.add_parser('audit', help='duplicates / orphans / leftovers of a snapshot')
    p.add_argument('--procs-all', required=True)
    p.add_argument('--launch-pid', type=int, default=0)
    p.add_argument('--expect-none', action='store_true')
    p.add_argument('--out')
    p = sub.add_parser('sweep', help='SIGKILL leftover stack process groups')
    p.add_argument('--pgid', type=int, nargs='+', required=True)
    a = ap.parse_args(argv)
    return {'snapshot': cmd_snapshot, 'audit': cmd_audit, 'sweep': cmd_sweep}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main())
