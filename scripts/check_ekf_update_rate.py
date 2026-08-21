#!/usr/bin/env python3
"""Manual verification step for the core 0-1 EKF-pinning fix (see ekf.launch.py's
own module docstring, "cpu_affinity" paragraph, and this pass's own report):
scans component_supervisor_node's 'localization' log for
robot_localization's own "Failed to meet update rate!" warning -- the
concrete, live-observed symptom that first exposed cores 0-1 being
oversubscribed (4 processes -- both EKF instances, foxglove_bridge,
behavior_executor_node -- sharing a 2-core budget).

Usage:
  ./scripts/check_ekf_update_rate.py                    # default log path
  ./scripts/check_ekf_update_rate.py --log-dir /path/to/component_supervisor
  ./scripts/check_ekf_update_rate.py --file path/to/some.log
  ./scripts/check_ekf_update_rate.py --max-warnings 0 --max-delay-ms 40

Run this after a fresh `ros2 launch f1tenth_bringup supervisor_bringup.launch.py`
(or component_supervisor_node's own auto-start), ideally through both an
at-rest window AND a real-motion mission run -- see this pass's own report for
why "clears at rest" alone does not confirm the fix: the CPU-starvation theory
specifically predicts the symptom is much more visible under real angular-
velocity load, not just idle.

LIMITATION, stated plainly (not hidden): `ros2 launch`'s own per-process log
label for a `taskset`-prefixed Node is a generic, launch-order-assigned tag
(observed live as "taskset-1"/"taskset-2" for ekf_filter_node/
ekf_global_filter_node, in whichever order they happened to start) -- there is
no reliable per-line attribution back to "local" vs. "global" EKF from the log
text alone, only an aggregate count across whatever's pinned into that log
file. This script reports aggregate stats accordingly; it does not (and
cannot, from this log alone) tell the two EKF instances' warnings apart.
"""
import argparse
import glob
import os
import re
import sys

DEFAULT_LOG_DIR = os.path.expanduser('~/.ros/log/component_supervisor')
LOG_FILE_GLOB = 'f1tenth_localization_localization.launch.py.log'

# e.g. "[taskset-1] Failed to meet update rate! Took 0.07713214300000000023seconds."
_WARNING_RE = re.compile(
    r'Failed to meet update rate!\s*Took\s*([0-9]*\.?[0-9]+)\s*seconds', re.IGNORECASE)

# robot_localization's own ekf_node target period for this stack's frequency:
# 50.0 (f1tenth_bringup/config/ekf.yaml AND ekf_global.yaml -- both, by design,
# see ekf_global.yaml's own "matches the local EKF" comment) -> 20ms.
_TARGET_PERIOD_MS = 20.0


def _find_default_log_file(log_dir: str) -> str:
    candidates = sorted(glob.glob(os.path.join(log_dir, LOG_FILE_GLOB)))
    if not candidates:
        sys.exit(
            f"error: no '{LOG_FILE_GLOB}' found under {log_dir!r} -- pass "
            '--file explicitly, or --log-dir if component_supervisor is '
            'using a non-default log_dir.')
    # component_supervisor_node overwrites this file fresh on every
    # _start_component('localization') call (open(..., 'w') in
    # _ComponentProcess.start()) -- there is only ever one live copy, but
    # sort defensively in case a caller points --log-dir at somewhere with
    # rotated/renamed copies.
    return candidates[-1]


def scan(path: str):
    delays_ms = []
    with open(path, errors='replace') as f:
        for line in f:
            m = _WARNING_RE.search(line)
            if m:
                delays_ms.append(float(m.group(1)) * 1000.0)
    return delays_ms


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--log-dir', default=DEFAULT_LOG_DIR,
        help=f'component_supervisor log_dir (default: {DEFAULT_LOG_DIR})')
    parser.add_argument(
        '--file', default=None,
        help="scan this exact log file instead of searching --log-dir")
    parser.add_argument(
        '--max-warnings', type=int, default=5,
        help='fail if MORE than this many warnings are found (default: 5 -- a handful '
             'during the startup/calibration burst is tolerated; the original bug '
             'produced dozens)')
    parser.add_argument(
        '--max-delay-ms', type=float, default=50.0,
        help='fail if any single warning exceeds this delay, ms (default: 50.0 -- '
             f'2.5x the {_TARGET_PERIOD_MS:.0f}ms target period; the original bug '
             'showed up to 59ms)')
    args = parser.parse_args()

    path = args.file or _find_default_log_file(args.log_dir)
    if not os.path.isfile(path):
        sys.exit(f'error: {path!r} does not exist.')

    delays_ms = scan(path)

    print(f'Scanned: {path}')
    print(f'Target update period: {_TARGET_PERIOD_MS:.0f}ms (50Hz, both EKF instances)')
    print(f'"Failed to meet update rate!" warnings found: {len(delays_ms)}')

    if not delays_ms:
        print('PASS -- no update-rate warnings at all.')
        return 0

    worst = max(delays_ms)
    print(f'  worst delay: {worst:.1f}ms ({worst / _TARGET_PERIOD_MS:.1f}x target period)')
    print(f'  mean delay:  {sum(delays_ms) / len(delays_ms):.1f}ms')

    failed = False
    if len(delays_ms) > args.max_warnings:
        print(f'FAIL -- {len(delays_ms)} warnings exceeds --max-warnings={args.max_warnings}')
        failed = True
    if worst > args.max_delay_ms:
        print(f'FAIL -- worst delay {worst:.1f}ms exceeds --max-delay-ms={args.max_delay_ms}')
        failed = True

    if failed:
        print(
            '\nSee this log for the file this pass was validated against and the '
            '"Failed to meet update rate!" symptom this fix targets -- cores 0-1 '
            'may still be oversubscribed (check `taskset -pc <pid>` for '
            "ekf_node/foxglove_bridge/behavior_executor_node's live affinity, "
            'and `cat /proc/stat` per-core busy%% during the same window this '
            'log covers).')
        return 1

    print('PASS -- within tolerance.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
