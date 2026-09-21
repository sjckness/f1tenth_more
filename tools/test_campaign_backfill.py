#!/usr/bin/env python3
"""Backfill campaign_results.csv values that a test could not record live.

Usage (after `source install/setup.bash`):
    python3 tools/test_campaign_backfill.py [campaign_folder] [--tests ID ...]
        [--archive DIR] [--ros-log DIR] [--dry-run]

Written for the 2026-09-21 session (P001-R001..R005, P002-R001, the default
--tests), where three inputs never reached the test folders: mission_loaded
arrived before the logger had a test open (the planner published
/test/plan_result only after start_mission), and obstacle_clearance_node was
not running.

Writes <campaign>/backfill.json and nothing else. The test folders are only
read; the archive bags are opened read-only (sqlite `mode=ro`, never through
a path sqlite could create). test_campaign_export then fills a column from
backfill.json only where the test's own files leave it EMPTY, and names every
such column in its `backfilled` column, so a backfilled value can never pass
for a live one.

What it fills, and from where:

  countdown_s          the planner's own ROS log line for this test,
                       "start_mission OK: starting in X s" -- only when no
                       event in events.jsonl carries countdown_s.
  standstill_jerk_rms  the export's own jerk measure (same filter, same
                       cutoff as export_settings.json) over test_start ->
                       mission_started, when mission_loaded was never
                       recorded. The countdown began ~0.1 s before
                       test_start, so the window misses that much of it.
  min_clear_raw_m      obstacle_clearance_node's own code, re-run over /scan in
  min_clear_m          the mission logger's archive bag for this run:
                       swept_clearance.scan_to_points (same valid-return
                       filter), obstacle_clearance.footprint_clearance, the
                       footprint from stack_params (swept_clearance_body_*,
                       as obstacle_clearance.launch.py reads it), base_link <-
                       laser from the bag's own /tf_static through a tf2
                       Buffer. Only scans inside the export's driving window
                       (mission_started -> first finished/aborted) count.
                       The bag clock is tied to the test's t by matching the
                       bag's /odom samples to kinematics.csv (the logger
                       recorded /odom that day). The recorder starts seconds
                       after the mission, so the bag covers only the end of
                       the drive: the value is tagged "partial NN%" and the
                       true minimum can only be lower. min_clear_m is 0.0 if
                       the test has an estop/contact event or the recomputed
                       clearance reaches 0 (what obstacle_clearance_node
                       would have reported as contact), else the raw value.

A column that cannot be backfilled is listed under "unavailable" with the
reason, and left empty.
"""

import argparse
import csv
import json
import math
import os
import re
import sqlite3
import statistics
from datetime import datetime
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from f1tenth_logger.test_campaign import export_campaign_csv as exp
from f1tenth_logger.test_campaign.robot_logger import DEFAULT_CAMPAIGN, find_root
from f1tenth_params.param_defaults import get_value
from f1tenth_perception.obstacle_clearance import footprint_clearance
from f1tenth_perception.swept_clearance import quaternion_to_rotation, scan_to_points

DEFAULT_TESTS = (
    'P001-R001-20260921T145508',
    'P001-R002-20260921T145711',
    'P001-R003-20260921T145854',
    'P001-R004-20260921T150023',
    'P001-R005-20260921T150157',
    'P002-R001-20260921T150345',
)
DEFAULT_ARCHIVE = Path.home() / 'f1tenth_archive' / 'complete'
DEFAULT_ROS_LOG = Path.home() / '.ros' / 'log'
TOOL = 'tools/test_campaign_backfill.py'
BASE_FRAME = 'base_link'
CONTACT_THRESHOLD_M = 0.0          # obstacle_clearance_node's default
FULL_COVERAGE = 0.98               # below this the minimum is tagged partial

#: llm_planner_node's line after a start was accepted, e.g.
#: [INFO] [1789995308.371979384] [llm_planner_node]: start_mission OK: starting in 3.0 s
START_LINE = re.compile(
    r'^\[\w+\] \[(\d+\.\d+)\] \[llm_planner_node\]: '
    r'start_mission OK: starting in ([0-9.]+) s')
ROS_LOG_NAME = re.compile(r'^python3_\d+_(\d+)\.log$')


# --------------------------------------------------------------------------
# the test folder
# --------------------------------------------------------------------------

def find_test(campaign, test_id):
    matches = [d for d in campaign.glob(f'*/{test_id}') if d.is_dir()]
    if len(matches) != 1:
        raise SystemExit(f'{test_id}: expected one folder under {campaign}, '
                         f'found {len(matches)}')
    return matches[0]


def drive_window(first):
    """(t_started, t_end) exactly as the export takes it, or None."""
    t_started = first.get('mission_started')
    ends = [first[k] for k in ('mission_finished', 'mission_aborted') if k in first]
    if t_started is None or not ends:
        return None
    return t_started, min(ends)


# --------------------------------------------------------------------------
# countdown_s: the planner's log
# --------------------------------------------------------------------------

def planner_countdown(ros_log, ros_start):
    """(countdown_s, log file, line stamp) for the start this test came from."""
    for path in sorted(ros_log.iterdir()):
        match = ROS_LOG_NAME.match(path.name)
        if match is None:
            continue
        started = int(match.group(1)) / 1000.0
        # the planner starts before its LLM call (~30 s here) and logs the
        # start a few ms before the logger opens the test
        if not ros_start - 3600.0 <= started <= ros_start:
            continue
        with open(path, encoding='utf-8', errors='replace') as fh:
            for line in fh:
                found = START_LINE.match(line)
                if found and abs(float(found.group(1)) - ros_start) <= 5.0:
                    return float(found.group(2)), path.name, float(found.group(1))
    return None


# --------------------------------------------------------------------------
# min_clear_raw_m: the archive bag
# --------------------------------------------------------------------------

def find_archive_run(archive, plan_id, ros_start, duration_s):
    """The mission logger's run of this test: same mission, started inside it."""
    hits = []
    for manifest in archive.glob('*/*.manifest.json'):
        try:
            data = json.loads(manifest.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            continue
        if data.get('mission_id') != plan_id or not data.get('start_time'):
            continue
        start = datetime.fromisoformat(data['start_time']).timestamp()
        if ros_start <= start <= ros_start + duration_s:
            hits.append((start, manifest.parent))
    if len(hits) != 1:
        return None, f'{len(hits)} archive runs of {plan_id} start inside this test'
    return hits[0][1], None


def read_bag(bag_dir, topics):
    """{topic: [(bag_t, msg)]}, straight from the sqlite file, read-only."""
    db = bag_dir / 'bag_0.db3'
    if not db.is_file():
        return None
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    try:
        rows = con.execute('SELECT id, name, type FROM topics').fetchall()
        known = {name: (tid, get_message(typ)) for tid, name, typ in rows}
        out = {topic: [] for topic in topics}
        for topic in topics:
            if topic not in known:
                continue
            tid, msg_type = known[topic]
            for stamp, data in con.execute(
                    'SELECT timestamp, data FROM messages WHERE topic_id = ? '
                    'ORDER BY timestamp', (tid,)):
                out[topic].append((stamp * 1e-9, deserialize_message(data, msg_type)))
        return out
    finally:
        con.close()


def header_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def align_to_test(odom, kinematics_csv):
    """Bag stamp minus test t, from /odom poses that also sit in kinematics.csv.

    The logger wrote x and y with 6 significant digits, so a bag pose printed
    the same way names its row exactly; only poses that name one row count.
    Returns (offset, n_matches, spread_s) or None.
    """
    rows = {}
    with open(kinematics_csv, newline='', encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            key = (row['x'], row['y'])
            rows.setdefault(key, []).append(float(row['t']))
    offsets = []
    for _, msg in odom:
        p = msg.pose.pose.position
        ts = rows.get((f'{p.x:.6g}', f'{p.y:.6g}'))
        if ts is not None and len(ts) == 1:
            offsets.append(header_seconds(msg) - ts[0])
    if len(offsets) < 3:
        return None
    return statistics.median(offsets), len(offsets), max(offsets) - min(offsets)


def footprint():
    front = float(get_value('swept_clearance_body_front_x_m'))
    rear = float(get_value('swept_clearance_body_rear_x_m'))
    half_width = float(get_value('swept_clearance_body_half_width_m'))
    return front - rear, 2.0 * half_width, rear


def recompute_clearance(test_dir, meta, window, archive):
    """({column: entry}, reason) -- entries for min_clear_raw_m and min_clear_m."""
    extra = meta.get('extra_meta') or {}
    plan_id, ros_start = extra.get('plan_id'), extra.get('ros_start_time')
    duration = (meta.get('summary') or meta).get('duration_s')
    if not plan_id or ros_start is None or duration is None:
        return None, 'meta.json lacks plan_id / ros_start_time / duration_s'
    run, why = find_archive_run(archive, plan_id, float(ros_start), float(duration))
    if run is None:
        return None, why
    bag = read_bag(run / 'bag', ['/scan', '/tf_static', '/odom'])
    if bag is None:
        return None, f'{run.name}: no bag_0.db3'
    if not bag['/scan']:
        return None, f'{run.name}: the archive bag holds no /scan at all'

    aligned = align_to_test(bag['/odom'], test_dir / 'kinematics.csv')
    if aligned is None:
        return None, f'{run.name}: bag /odom does not match kinematics.csv'
    offset, n_match, spread = aligned

    tf = Buffer()
    for _, msg in bag['/tf_static']:
        for transform in msg.transforms:
            tf.set_transform_static(transform, 'test_campaign_backfill')
    length, width, rear_x = footprint()

    t_started, t_end = window
    hits = []                    # (test t, clearance)
    cached = {}
    for _, scan in bag['/scan']:
        t = header_seconds(scan) - offset
        if not t_started <= t <= t_end:
            continue
        frame = scan.header.frame_id
        if frame not in cached:
            try:
                stamped = tf.lookup_transform(BASE_FRAME, frame, Time())
            except TransformException as exc:
                return None, f'{run.name}: no {BASE_FRAME} <- {frame} in /tf_static: {exc}'
            q, v = stamped.transform.rotation, stamped.transform.translation
            cached[frame] = (quaternion_to_rotation(q.x, q.y, q.z, q.w),
                             np.array([v.x, v.y, v.z]))
        points = scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                scan.range_min, scan.range_max, *cached[frame])
        value = footprint_clearance(points, length, width, rear_x)
        if math.isfinite(value):          # +inf is written empty by the logger
            hits.append((t, value))
    if not hits:
        return None, f'{run.name}: no /scan inside the driving window'

    first_t, last_t = hits[0][0], hits[-1][0]
    coverage = (last_t - first_t) / (t_end - t_started)
    tag = None if coverage >= FULL_COVERAGE else f'partial {coverage:.0%}'
    raw = min(v for _, v in hits)
    _, translation = next(iter(cached.values()))
    source = (
        f'obstacle_clearance_node geometry re-run over /scan of archive run '
        f'{run.name}: {len(hits)} scans at t={first_t:.2f}-{last_t:.2f} s of the '
        f'driving window {t_started:.2f}-{t_end:.2f} s ({coverage:.0%}); footprint '
        f'{length:.3f} x {width:.3f} m, tail at x={rear_x:+.3f}; base_link <- laser '
        f't=({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) from the '
        f"bag's /tf_static; bag clock tied to test t by {n_match} /odom poses "
        f'(spread {spread * 1000:.1f} ms)'
        + ('' if tag is None else '; the true minimum can only be lower')
    )

    names = {str(e.get('event', '')) for e in exp.read_jsonl(test_dir / 'events.jsonl')}
    touched = 'estop' in names or 'contact' in names or raw <= CONTACT_THRESHOLD_M
    entries = {
        'min_clear_raw_m': {'value': round(raw, 4), 'source': source, 'tag': tag},
        'min_clear_m': {
            'value': 0.0 if touched else round(raw, 4),
            'source': 'min_clear_raw_m above, with the export rule (0 on estop/contact, '
                      'and on a recomputed clearance <= 0)',
            'tag': tag,
        },
    }
    return entries, None


# --------------------------------------------------------------------------
# one test
# --------------------------------------------------------------------------

def backfill_test(test_dir, archive, ros_log, cutoff_hz):
    meta = json.loads((test_dir / 'meta.json').read_text(encoding='utf-8'))
    events = exp.read_jsonl(test_dir / 'events.jsonl')
    first = exp.first_event_times(events)
    columns, unavailable = {}, {}

    # countdown_s
    live = any(isinstance(e.get('countdown_s'), (int, float)) for e in events
               if e.get('event') in ('mission_loaded', 'mission_started'))
    if not live:
        ros_start = (meta.get('extra_meta') or {}).get('ros_start_time')
        found = None if ros_start is None else planner_countdown(ros_log, float(ros_start))
        if found is None:
            unavailable['countdown_s'] = 'no planner start_mission line near this test'
        else:
            value, log_name, stamp = found
            columns['countdown_s'] = {
                'value': value,
                'source': f'{ros_log / log_name} @ {stamp:.3f}: '
                          f"'start_mission OK: starting in {value} s'",
            }

    # standstill_jerk_rms
    if 'mission_loaded' not in first:
        if 'test_start' in first and 'mission_started' in first:
            window = (first['test_start'], first['mission_started'])
            imu = exp.read_csv_columns(test_dir / 'imu.csv', ['t', 'ax', 'ay'])
            mask = exp.in_window(imu['t'], window)
            value = exp.jerk_rms(imu['t'][mask], imu['ax'][mask], imu['ay'][mask],
                                 cutoff_hz)
            if value is None:
                unavailable['standstill_jerk_rms'] = (
                    f'too little IMU in test_start->mission_started {window}')
            else:
                columns['standstill_jerk_rms'] = {
                    'value': round(value, 6),
                    'source': f'imu.csv over test_start->mission_started '
                              f'({window[0]:.3f}-{window[1]:.3f} s), {int(mask.sum())} '
                              f'samples, cutoff {cutoff_hz:g} Hz; mission_loaded was '
                              f'never recorded, so the first ~0.1 s of the countdown '
                              f'is missing',
                }
        else:
            unavailable['standstill_jerk_rms'] = 'no test_start/mission_started event'

    # min_clear_raw_m / min_clear_m
    kin = exp.read_csv_columns(test_dir / 'kinematics.csv', ['t', 'obstacle_clearance'])
    window = drive_window(first)
    if np.isfinite(kin['obstacle_clearance']).any():
        pass                      # recorded live: nothing to backfill
    elif window is None:
        unavailable['min_clear_raw_m'] = 'no driving window (mission_started/end)'
    else:
        entries, why = recompute_clearance(test_dir, meta, window, archive)
        if entries is None:
            unavailable['min_clear_raw_m'] = why
        else:
            columns.update(entries)
    return {'columns': columns, 'unavailable': unavailable}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('campaign_folder', nargs='?', default=None)
    parser.add_argument('--tests', nargs='+', default=list(DEFAULT_TESTS))
    parser.add_argument('--archive', type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument('--ros-log', type=Path, default=DEFAULT_ROS_LOG)
    parser.add_argument('--dry-run', action='store_true',
                        help='print what would be written, write nothing')
    args = parser.parse_args(argv)

    campaign = (Path(args.campaign_folder).expanduser().resolve() if args.campaign_folder
                else find_root() / DEFAULT_CAMPAIGN)
    settings_path = campaign / exp.SETTINGS_NAME
    cutoff_hz = exp.DEFAULT_CUTOFF_HZ
    if settings_path.exists():
        cutoff_hz = float(json.loads(settings_path.read_text())['cutoff_hz'])

    out_path = campaign / exp.BACKFILL_NAME
    existing = {}
    if out_path.exists():
        existing = json.loads(out_path.read_text(encoding='utf-8')).get('tests') or {}

    results = {}
    for test_id in args.tests:
        results[test_id] = backfill_test(find_test(campaign, test_id), args.archive,
                                         args.ros_log, cutoff_hz)
        for column, entry in results[test_id]['columns'].items():
            tag = f"  [{entry['tag']}]" if entry.get('tag') else ''
            print(f'{test_id}  {column:<20} {entry["value"]}{tag}')
        for column, why in results[test_id]['unavailable'].items():
            print(f'{test_id}  {column:<20} UNAVAILABLE: {why}')

    if args.dry_run:
        return 0
    payload = {
        'written': datetime.now().isoformat(timespec='seconds'),
        'tool': TOOL,
        'note': 'Values here were NOT recorded live. test_campaign_export uses '
                'them only for columns the test folders leave empty, and names '
                "them in the 'backfilled' column.",
        'tests': {**existing, **results},
    }
    tmp = out_path.with_name(out_path.name + '.tmp')
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n',
                   encoding='utf-8')
    os.replace(tmp, out_path)
    print(f'written {out_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
