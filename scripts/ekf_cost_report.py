#!/usr/bin/env python3
"""Summarize ekf_cost_observer_node's per-update cost metric from a run's bag.

This is the read side of the instrument added for the 50Hz -> 20Hz EKF retarget.
robot_localization's own "Failed to meet update rate!" warning compares
loop_elapsed against 1/frequency (ros_filter.cpp:2206), so it goes quiet the
moment frequency drops even if the underlying cost is unchanged;
ekf_cost_observer_node records the cost itself, and this script reads it back.

Usage:
  ./scripts/ekf_cost_report.py <run_dir_or_bag_dir> [<run_dir_or_bag_dir> ...]

Pass the 50Hz baseline run and the 20Hz run together to get a before/after
table. Accepts either an f1tenth_archive run directory (the bag/ subdirectory is
found automatically) or a bag directory directly.

WHAT THE NUMBERS MEAN, so a reader does not over-claim from them:

  cpu_ms_per_tick  CPU time consumed per filter tick. This is the SAME quantity
                   the 27.5ms/tick figure came from, so it compares directly
                   against it. It is CPU time, not wall time: on a loaded box it
                   is a LOWER BOUND on the loop_elapsed that robot_localization
                   itself measures.
  cpu_ms_per_meas  CPU time per delivered input measurement -- the `B` term.
                   Prediction is measurement-driven, so THIS is the number that
                   should be flat across a frequency change. If it moved, the
                   frequency change is not what caused it.
  meas_per_tick    Input measurements per tick. Expected to rise roughly in
                   proportion when the tick rate falls, because the same input
                   stream is being drained in fewer, bigger batches.
  tick_rate_hz     Achieved tick rate.

The agreement column is the instrument's own self-check: the ratio of the
filter's in-process tick count to the observer's subscribed count. Values far
from 1.0 mean one of the counts is wrong and the cost figures in the same window
are not trustworthy. It runs slightly above 1.0 by design (see the node's own
module docstring).
"""

import argparse
import os
import statistics
import sys

_TOPIC = '/diagnostics'
_STATUS_PREFIX = 'ekf_cost_observer:'

# Reported per filter, in the order a reader should look at them: cost first,
# then the workload that explains it, then the self-check.
_FIELDS = [
    'cpu_ms_per_tick',
    'cpu_ms_per_meas',
    'meas_per_tick',
    'tick_rate_hz',
    'cpu_percent_of_core',
    'period_ms_p90',
    'tick_count_agreement',
]


def _resolve_bag_dir(path):
    """Accept an archive run directory or a bag directory; return the bag dir."""
    if os.path.isdir(os.path.join(path, 'bag')):
        return os.path.join(path, 'bag')
    return path


def _storage_id(bag_dir):
    """rosbag2 storage plugin for this bag.

    The stack falls back to sqlite3 whenever the mcap plugin is absent, so a
    bag's plugin is a property of the box that recorded it, not a constant.
    Read it from metadata.yaml instead of assuming.
    """
    meta = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.isfile(meta):
        with open(meta) as handle:
            for line in handle:
                if 'storage_identifier' in line:
                    return line.split(':', 1)[1].strip().strip("'\"")
    return 'sqlite3'


def read_cost_samples(bag_dir):
    """Per-filter lists of metric dicts, one entry per observer publish."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from diagnostic_msgs.msg import DiagnosticArray

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=_storage_id(bag_dir)),
        rosbag2_py.ConverterOptions('', ''))
    if _TOPIC not in {t.name for t in reader.get_all_topics_and_types()}:
        return {}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[_TOPIC]))

    samples = {}
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        msg = deserialize_message(data, DiagnosticArray)
        for status in msg.status:
            if not status.name.startswith(_STATUS_PREFIX):
                continue
            label = status.name[len(_STATUS_PREFIX):].strip()
            entry = {}
            for kv in status.values:
                try:
                    entry[kv.key] = float(kv.value)
                except ValueError:
                    continue
            # Windows in which the filter published nothing carry no cost
            # information and would drag every median toward zero.
            if entry.get('ticks_inproc', 0.0) or entry.get('ticks_selfcount', 0.0):
                samples.setdefault(label, []).append(entry)
    return samples


def _stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        'n': len(ordered),
        'median': statistics.median(ordered),
        'mean': statistics.fmean(ordered),
        'p90': ordered[min(len(ordered) - 1, int(0.9 * len(ordered)))],
        'max': ordered[-1],
    }


def report(paths):
    """Print one block per input run. Returns the collected stats."""
    collected = {}
    for path in paths:
        bag_dir = _resolve_bag_dir(path)
        name = os.path.basename(os.path.normpath(path))
        print(f'\n=== {name} ===')
        if not os.path.isdir(bag_dir):
            print(f'  no such bag directory: {bag_dir}')
            continue

        samples = read_cost_samples(bag_dir)
        if not samples:
            print('  no ekf_cost_observer samples in this bag.')
            print('  (Recorded before the metric existed, or the observer was '
                  'not running.)')
            continue

        for label in sorted(samples):
            rows = samples[label]
            print(f'\n  [{label}]  {len(rows)} windows')
            print(f'    {"metric":<22}{"median":>10}{"mean":>10}'
                  f'{"p90":>10}{"max":>10}')
            for field in _FIELDS:
                stats = _stats([r[field] for r in rows if field in r])
                if stats is None:
                    continue
                print(f'    {field:<22}{stats["median"]:>10.2f}'
                      f'{stats["mean"]:>10.2f}{stats["p90"]:>10.2f}'
                      f'{stats["max"]:>10.2f}')
                collected.setdefault(name, {}).setdefault(label, {})[field] = stats

            agreement = _stats([r['tick_count_agreement'] for r in rows
                                if r.get('tick_count_agreement')])
            if agreement and not 0.9 <= agreement['median'] <= 1.3:
                print(f'    WARNING: tick counts disagree (median '
                      f'{agreement["median"]:.2f}); treat the cost figures '
                      f'above as unreliable.')
    return collected


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        'paths', nargs='+',
        help='Archive run directories or bag directories to summarize.')
    args = parser.parse_args(argv)

    collected = report(args.paths)
    if not collected:
        print('\nNothing to compare.')
        return 1

    if len(collected) > 1:
        print('\n=== before/after: cpu_ms_per_tick and cpu_ms_per_meas '
              '(medians) ===')
        labels = sorted({lab for run in collected.values() for lab in run})
        for label in labels:
            print(f'\n  [{label}]')
            for name, run in collected.items():
                per_tick = run.get(label, {}).get('cpu_ms_per_tick')
                per_meas = run.get(label, {}).get('cpu_ms_per_meas')
                if per_tick is None or per_meas is None:
                    continue
                print(f'    {name:<52}'
                      f'{per_tick["median"]:>8.2f} ms/tick'
                      f'{per_meas["median"]:>10.2f} ms/meas')
        print('\n  cpu_ms_per_meas is the term a frequency change should NOT '
              'move.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
