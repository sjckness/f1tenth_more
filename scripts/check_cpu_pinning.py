#!/usr/bin/env python3
"""Permanent verification for the CPU-pinning mechanism this stack now uses
uniformly (Step 6 reintroduction investigation, thread-pinning-leak fix):
every node that's supposed to be pinned to specific cores is launched with a
`taskset -c <cores>` prefix, NOT an in-process self-pin -- see ekf.launch.py,
foxglove_bridge.launch.py, slam.launch.py, behavior_bringup.launch.py,
detection.launch.py, and mpc_corr.launch.py's own matching comments for why
(the self-pin mechanism, os.sched_setaffinity(0, cores) called once from a
node's own __init__, only ever restricts the ONE thread executing that call
-- confirmed live to leave the vast majority of a process's threads fully
unpinned, with several actually caught executing on OTHER nodes' reserved
cores under real load).

This script re-runs that same manual check, permanently: for each node,
finds its PID, walks EVERY thread under /proc/<pid>/task/* (not just the
main one -- that was the exact gap that let the original leak go
unnoticed), checks each thread's Cpus_allowed mask against the node's
expected core set, AND samples which core each thread is ACTUALLY executing
on (/proc/<tid>/stat field 39) a few times a couple seconds apart -- an
allowed-but-unused mask is not the same guarantee as "never observed
running anywhere else," which is what actually matters for contention.

Usage:
  ./scripts/check_cpu_pinning.py                       # check every known
                                                         # node currently running
  ./scripts/check_cpu_pinning.py --node ekf_filter_node ekf_global_filter_node
  ./scripts/check_cpu_pinning.py --pid 12345 --cores 8,9   # ad-hoc: any PID
  ./scripts/check_cpu_pinning.py --samples 5 --sample-interval-sec 3.0

NODE_CORE_MAP below is this script's own copy of the node->core assignments,
matching each launch file's current hardcoded default. It is NOT sourced
from stack_params.yaml -- that file explicitly and deliberately does NOT
centralize these (see stack_params.yaml's own "cpu_affinity/nice ... are
deliberately NOT here" comment): the right core ids are machine-specific,
not a stack-wide default, so each launch file hardcodes its own directly.
There is no other single source of truth this script could read instead
without either contradicting that documented design choice or adding launch-
file introspection machinery for a handful of integers -- if a launch
file's default ever changes, update the matching entry here too (same
manual-sync convention already used between mpc_corr.launch.py's own
comments and f1tenth_perception/launch/detection.launch.py's, which
cross-reference each other's core assignments in prose, not in shared code).
"""
import argparse
import glob
import os
import sys
import time

# label -> (expected core ids, exact argv[0] basename, [extra cmdline
# substrings ALL of which must also be present]). The argv[0] basename check
# is the identity gate and must be an EXACT match, not a loose `ps`-style
# substring grep -- confirmed live this matters, not just theoretical: an
# early version of this script matched `foxglove_bridge` against the `ros2
# launch f1tenth_bringup foxglove_bridge.launch.py` WRAPPER process (argv[0]
# '/usr/bin/python3', but 'foxglove_bridge' appears in one of its own later
# args, the launch file's own name) instead of the real, taskset-wrapped
# `/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge` binary -- the exact
# same false-positive this investigation hit once manually with `pgrep -f`.
# argv[0]'s basename is what a taskset prefix's exec() replaces it with, so
# it reliably names the REAL binary regardless of what launched it. The
# extra-substrings list exists only for the one real ambiguity in this
# stack: both EKF instances share the identical argv[0] ('ekf_node'), so
# local vs. global is disambiguated via their own --ros-args
# '__node:=<name>' remap token instead, found in cmdline, not argv[0].
NODE_CORE_MAP = {
    'ekf_filter_node': ({0, 1}, 'ekf_node', ['__node:=ekf_filter_node']),
    'ekf_global_filter_node': ({0, 1}, 'ekf_node', ['__node:=ekf_global_filter_node']),
    'slam_toolbox': ({2}, 'async_slam_toolbox_node', []),
    'foxglove_bridge': ({3}, 'foxglove_bridge', []),
    'behavior_executor_node': ({4}, 'behavior_executor_node', []),
    'detection_3d_node': ({6, 7}, 'detection_3d_node', []),
    'obstacle_projector_node': ({6, 7}, 'obstacle_projector_node', []),
    'yolo_detector_node': ({8, 9}, 'yolo_detector_node', []),
    'mpc_corr': ({10, 11}, 'mpc_corr', []),
}


def _read_cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            raw = f.read()
    except (FileNotFoundError, ProcessLookupError):
        return []
    return [p.decode('utf-8', errors='replace') for p in raw.split(b'\x00') if p]


_INTERPRETERS = {'python3', 'python', 'python3.10', 'python3.8'}


def _find_pid(argv0_basename, extra_substrings):
    """Return the PID matching `argv0_basename` as the process's real
    identity, or None. The identity check is exact (not a substring grep --
    see NODE_CORE_MAP's own comment for why that matters), but "real
    identity" means one of two things depending on how taskset -c actually
    exec'd this node:
      - a compiled binary (ekf_node, foxglove_bridge,
        async_slam_toolbox_node): taskset's exec() replaces argv[0] with
        the binary's own path directly, so basename(cmdline[0]) IS the
        identity.
      - a Python entry-point node (behavior_executor_node, yolo_detector_
        node, detection_3d_node, obstacle_projector_node, mpc_corr):
        confirmed live that these run as `python3 <script-path> ...`, i.e.
        cmdline[0] is the INTERPRETER ('/usr/bin/python3'), not the node --
        checking only cmdline[0] here silently reports every one of these
        as "not running" even while alive (caught before this script was
        ever used for real: a first version did exactly that). When
        cmdline[0]'s basename is a known interpreter, the identity is
        cmdline[1]'s basename instead.
    Also requires every string in `extra_substrings` to appear in the full
    cmdline (used only to disambiguate the two EKF instances, which share
    the same argv[0]/identity otherwise).
    """
    for pid_dir in glob.glob('/proc/[0-9]*'):
        pid = os.path.basename(pid_dir)
        cmdline = _read_cmdline(pid)
        if not cmdline:
            continue
        identity = os.path.basename(cmdline[0])
        if identity in _INTERPRETERS and len(cmdline) > 1:
            identity = os.path.basename(cmdline[1])
        if identity != argv0_basename:
            continue
        joined = ' '.join(cmdline)
        if all(s in joined for s in extra_substrings):
            return int(pid)
    return None


def _list_tids(pid):
    try:
        return sorted(int(t) for t in os.listdir(f'/proc/{pid}/task'))
    except (FileNotFoundError, ProcessLookupError):
        return []


def _cpus_allowed(pid, tid):
    """Parse Cpus_allowed_list from /proc/<pid>/task/<tid>/status into a
    set of ints, e.g. '0,1' -> {0,1}, '6-9' -> {6,7,8,9}."""
    try:
        with open(f'/proc/{pid}/task/{tid}/status') as f:
            for line in f:
                if line.startswith('Cpus_allowed_list:'):
                    spec = line.split(':', 1)[1].strip()
                    cores = set()
                    for part in spec.split(','):
                        part = part.strip()
                        if not part:
                            continue
                        if '-' in part:
                            lo, hi = part.split('-')
                            cores.update(range(int(lo), int(hi) + 1))
                        else:
                            cores.add(int(part))
                    return cores
    except (FileNotFoundError, ProcessLookupError):
        pass
    return None


def _current_cpu(pid, tid):
    """Field 39 (0-indexed 38) of /proc/<pid>/task/<tid>/stat -- the CPU
    this thread was last observed executing on. Robust against the comm
    field containing spaces/parens by splitting after the last ')'."""
    try:
        with open(f'/proc/{pid}/task/{tid}/stat') as f:
            content = f.read()
        after_comm = content.rsplit(')', 1)[1]
        fields = after_comm.split()
        # fields[0] is state (field 3 overall); processor is field 39 overall
        # -> index 39 - 3 = 36 into `fields` (0-indexed after state).
        return int(fields[36])
    except (FileNotFoundError, ProcessLookupError, IndexError, ValueError):
        return None


def check_node(label, expected_cores, pid, samples, sample_interval_sec):
    tids = _list_tids(pid)
    if not tids:
        return None  # process vanished between discovery and check

    # Pass 1: allowed-mask check for every thread, right now.
    mask_violations = []  # (tid, actual_mask)
    for tid in tids:
        allowed = _cpus_allowed(pid, tid)
        if allowed is None:
            continue  # thread exited mid-scan
        if allowed != expected_cores:
            mask_violations.append((tid, allowed))

    # Pass 2: live-execution sampling, `samples` times, `sample_interval_sec`
    # apart -- same method used to manually catch Stage 3/4's live leaks
    # (an allowed mask can look correct while a thread that predates the
    # taskset exec, or one a library spawned with its own affinity call,
    # still runs elsewhere -- unlikely under this fix, but this is the
    # actual guarantee that matters, not just the permitted mask).
    exec_violations = {}  # tid -> set of offending cores observed
    for i in range(samples):
        current_tids = _list_tids(pid)
        for tid in current_tids:
            cpu = _current_cpu(pid, tid)
            if cpu is not None and cpu not in expected_cores:
                exec_violations.setdefault(tid, set()).add(cpu)
        if i < samples - 1:
            time.sleep(sample_interval_sec)

    return {
        'pid': pid,
        'thread_count': len(tids),
        'mask_violations': mask_violations,
        'exec_violations': exec_violations,
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--node', nargs='*', default=None,
        help='only check these node labels (default: every label in NODE_CORE_MAP '
             'that is currently running)')
    parser.add_argument(
        '--pid', type=int, default=None,
        help='ad-hoc mode: check this exact PID instead of auto-discovering by name '
             '(requires --cores too)')
    parser.add_argument(
        '--cores', default=None,
        help="ad-hoc mode: comma-separated expected core ids for --pid, e.g. '8,9'")
    parser.add_argument(
        '--samples', type=int, default=3,
        help='live-execution samples per node (default: 3)')
    parser.add_argument(
        '--sample-interval-sec', type=float, default=2.0,
        help='seconds between live-execution samples (default: 2.0, matching how '
             'this was manually verified during Step 6)')
    args = parser.parse_args()

    if args.pid is not None:
        if not args.cores:
            parser.error('--pid requires --cores')
        cores = {int(c) for c in args.cores.split(',') if c.strip()}
        targets = [(f'pid-{args.pid}', cores, args.pid)]
    else:
        labels = args.node if args.node else list(NODE_CORE_MAP.keys())
        targets = []
        for label in labels:
            if label not in NODE_CORE_MAP:
                sys.exit(f"error: unknown node label {label!r}. Known: "
                         f"{', '.join(NODE_CORE_MAP.keys())}")
            expected_cores, argv0_basename, extra_substrings = NODE_CORE_MAP[label]
            pid = _find_pid(argv0_basename, extra_substrings)
            targets.append((label, expected_cores, pid))

    print(f'Sampling {args.samples}x, {args.sample_interval_sec:.1f}s apart, per node...\n')

    overall_pass = True
    checked_any = False
    for label, expected_cores, pid in targets:
        expected_str = ','.join(str(c) for c in sorted(expected_cores))
        if pid is None:
            print(f'{label:28s} NOT RUNNING (skipped)')
            continue
        checked_any = True
        result = check_node(label, expected_cores, pid, args.samples, args.sample_interval_sec)
        if result is None:
            print(f'{label:28s} PID {pid} vanished mid-check (skipped)')
            continue

        n = result['thread_count']
        mask_bad = result['mask_violations']
        exec_bad = result['exec_violations']

        if not mask_bad and not exec_bad:
            print(f'{label:28s} PID {pid:<7d} PASS  ({n} threads, all on core(s) '
                  f'{expected_str})')
            continue

        overall_pass = False
        print(f'{label:28s} PID {pid:<7d} FAIL  ({n} threads, expected core(s) '
              f'{expected_str})')
        if mask_bad:
            print(f'  {len(mask_bad)} thread(s) with wrong Cpus_allowed mask:')
            for tid, allowed in mask_bad:
                allowed_str = ','.join(str(c) for c in sorted(allowed))
                print(f'    TID {tid}: allowed={{{allowed_str}}}')
        if exec_bad:
            print(f'  {len(exec_bad)} thread(s) OBSERVED EXECUTING outside '
                  f'{{{expected_str}}}:')
            for tid, cpus in sorted(exec_bad.items()):
                cpus_str = ','.join(str(c) for c in sorted(cpus))
                print(f'    TID {tid}: seen on cpu {{{cpus_str}}}')

    print()
    if not checked_any:
        print('NOTHING TO CHECK -- no requested node is currently running.')
        return 1
    if overall_pass:
        print('PASS -- every checked node stayed fully within its assigned core(s).')
        return 0
    print('FAIL -- see offending TIDs/cores above. If a node using the '
          "self-pin mechanism still exists somewhere, this is the exact "
          'leak Step 6 found: fix it the same way (taskset -c launch '
          'prefix, not os.sched_setaffinity from inside the node).')
    return 1


if __name__ == '__main__':
    sys.exit(main())
