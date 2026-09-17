#!/usr/bin/env python3
"""Measure the car's stopping distance from the run archive. Read-only.

What object_stop_distance_m (stack_params.yaml) was taken from, and how to take
it again. It prints one row per stop event and the summary the parameter's
description quotes.

THE EVENT. mpc_corr's /drive speed goes from a command that was non-zero for at
least MIN_MOVING_CMD_S to exactly 0.0, and stays 0.0 for PERSIST_ZERO_S. t0 is
the bag receive time of the first zero. mpc_corr publishes a hard 0.0 on hold,
goal_reached and every other stop path, so this is "the command stepped to
zero", whatever stepped it.

THE DISTANCE, TWO WAYS.
  lidar     the dead-ahead /scan range (median within +-2 deg), interpolated to
            the zero command's header stamp, minus the same 0.5 s after the car
            came to rest. Moving along a ray shortens the range to whatever the
            ray hits by exactly the distance moved, so this is a true distance
            -- provided the ray keeps hitting the same static surface. Rows
            where it did not (negative or implausible values: a person walked
            through, the car yawed onto a different surface) are rejected.
  odometry  the /odometry/filtered path length from t0 until the measured speed
            stays at or below V_REST for REST_HOLD_S. Reported, never used: on
            these stops it reads 1.45x SHORT of the LiDAR, against 1.20x when
            cruising -- the wheels read short while braking.

NORMALISATION. Each usable stop is scaled to 0.4 m/s as d * (0.4 / v0)^2, with
v0 the /odometry/filtered speed at t0 -- odometry units, the same units
mpc_corr commands in, so the result answers "commanded 0.4, then zero: how far
does the car truly go". A latency-plus-deceleration fit d = v*tau + v^2/(2a)
over the raw usable points is printed beside it.

WHAT THE ARCHIVE CANNOT GIVE. The mission logger stops recording at the hold
that ends a mission, so an end-of-mission stop (object_reached, goal_reached,
an abort) has no data after its zero command. Every usable event is a
MID-mission stop.

READ-ONLY BY CONSTRUCTION. Bags are opened with sqlite URIs in mode=ro: a plain
sqlite3.connect on a wrong filename creates an empty file inside the archive.

Needs a sourced ROS 2 environment (message deserialisation):
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 tools/measure_stop_distance.py [--archive ~/f1tenth_archive/complete]
"""

import argparse
import glob
import math
import os
import sqlite3
import statistics

MIN_MOVING_CMD_S = 1.0
PERSIST_ZERO_S = 1.0
V_REST = 0.02
REST_HOLD_S = 0.5
LIDAR_HALF_DEG = 2.0
LIDAR_MAX_BRACKET_S = 0.1
USABLE_V0 = (0.33, 0.50)
USABLE_MAX_DYAW_DEG = 1.5
USABLE_MAX_D_M = 0.3
V_NORM = 0.4


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _read(con, topic, cls):
    from rclpy.serialization import deserialize_message
    row = con.execute('select id from topics where name=?', (topic,)).fetchone()
    if row is None:
        return []
    return [(ts * 1e-9, deserialize_message(data, cls)) for ts, data in con.execute(
        'select timestamp, data from messages where topic_id=? order by timestamp',
        (row[0],))]


def _load_run(run_dir):
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    streams = {'drive': [], 'filt': [], 'scan': []}
    for db in sorted(glob.glob(os.path.join(run_dir, 'bag', '*.db3'))):
        if not os.path.isfile(db) or os.path.getsize(db) == 0:
            continue
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        try:
            streams['drive'] += _read(con, '/drive', AckermannDriveStamped)
            streams['filt'] += _read(con, '/odometry/filtered', Odometry)
            streams['scan'] += _read(con, '/scan', LaserScan)
        finally:
            con.close()
    return streams


def stop_events(drive):
    """Yield (index, t0) of each sustained non-zero -> persistent zero command."""
    moving_since = None
    for i, (t, msg) in enumerate(drive):
        if msg.drive.speed != 0.0:
            if moving_since is None:
                moving_since = t
            continue
        if moving_since is not None and t - moving_since >= MIN_MOVING_CMD_S:
            if all(m.drive.speed == 0.0 for tt, m in drive[i:] if tt - t <= PERSIST_ZERO_S):
                yield i, t
        moving_since = None


def odometry_stop(filt, t0):
    """Return speed at t0, path length and time to rest, yaw change; None if it never rests."""
    after = [(t, m) for t, m in filt if t >= t0 - 1e-3]
    if len(after) < 5:
        return None
    t_first, first = after[0]
    px, py = first.pose.pose.position.x, first.pose.pose.position.y
    yaw0 = _yaw(first.pose.pose.orientation)
    path, rest = 0.0, None
    for t, m in after[1:]:
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        path += math.hypot(x - px, y - py)
        px, py = x, y
        if abs(m.twist.twist.linear.x) <= V_REST:
            if rest is None:
                rest = (t, path, _yaw(m.pose.pose.orientation), m)
            elif t - rest[0] >= REST_HOLD_S:
                dyaw = math.atan2(math.sin(rest[2] - yaw0), math.cos(rest[2] - yaw0))
                return dict(v0=first.twist.twist.linear.x, d=rest[1], t_stop=rest[0] - t0,
                            dyaw_deg=abs(math.degrees(dyaw)), rest_msg=rest[3])
        else:
            rest = None
    return None


def _dead_ahead(scan):
    vals = []
    for i, r in enumerate(scan.ranges):
        a = scan.angle_min + i * scan.angle_increment
        a = math.atan2(math.sin(a), math.cos(a))
        if (abs(a) <= math.radians(LIDAR_HALF_DEG) and scan.range_min < r < scan.range_max
                and math.isfinite(r)):
            vals.append(r)
    return statistics.median(vals) if len(vals) >= 3 else None


def lidar_range_at(scans, t):
    """Return the dead-ahead range at header time t, interpolated between bracketing scans."""
    before = after = None
    for _, m in scans:
        s = _stamp(m)
        if s <= t:
            before = (s, m)
        elif after is None:
            after = (s, m)
            break
    if before is None or after is None or after[0] - before[0] > LIDAR_MAX_BRACKET_S:
        return None
    ra, rb = _dead_ahead(before[1]), _dead_ahead(after[1])
    if ra is None or rb is None:
        return None
    return ra + (rb - ra) * (t - before[0]) / (after[0] - before[0])


def measure_run(run_dir):
    """Return one row per stop event in a run directory (see the module docstring)."""
    s = _load_run(run_dir)
    if not s['drive'] or not s['filt']:
        return []
    rows = []
    for index, t0 in stop_events(s['drive']):
        odo = odometry_stop(s['filt'], t0)
        if odo is None:
            continue
        pre = [m.drive.speed for t, m in s['drive'] if t0 - 0.55 <= t < t0]
        t0_h = _stamp(s['drive'][index][1])
        r0 = lidar_range_at(s['scan'], t0_h)
        r1 = lidar_range_at(s['scan'], _stamp(odo['rest_msg']) + REST_HOLD_S)
        d_lidar = (r0 - r1) if (r0 is not None and r1 is not None) else None
        rows.append(dict(run=os.path.basename(run_dir), last_cmd=pre[-1] if pre else None,
                         steady_cmd=bool(pre) and min(pre) >= 0.9 * max(pre),
                         v0=odo['v0'], d_odom=odo['d'], t_stop=odo['t_stop'],
                         dyaw_deg=odo['dyaw_deg'], r0=r0, d_lidar=d_lidar))
    return rows


def usable(row):
    """Say whether a stop has a trustworthy LiDAR distance at a normalisable speed."""
    return (row['d_lidar'] is not None and 0.0 < row['d_lidar'] < USABLE_MAX_D_M
            and row['dyaw_deg'] < USABLE_MAX_DYAW_DEG and row['steady_cmd']
            and USABLE_V0[0] <= row['v0'] <= USABLE_V0[1])


def fit_latency_decel(points):
    """Least squares for d = tau*v + k*v^2; returns (tau, a) with a = 1/(2k)."""
    s11 = sum(v * v for v, _ in points)
    s12 = sum(v ** 3 for v, _ in points)
    s22 = sum(v ** 4 for v, _ in points)
    b1 = sum(v * d for v, d in points)
    b2 = sum(v * v * d for v, d in points)
    det = s11 * s22 - s12 * s12
    tau = (b1 * s22 - b2 * s12) / det
    k = (s11 * b2 - s12 * b1) / det
    return tau, (1.0 / (2.0 * k) if k > 0.0 else math.inf)


def main(argv=None):
    """Scan the archive and print every stop event and the summary."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--archive', default=os.path.expanduser('~/f1tenth_archive/complete'))
    args = parser.parse_args(argv)

    rows = []
    for run in sorted(os.listdir(args.archive)):
        rows += measure_run(os.path.join(args.archive, run))

    print(f'{"run":<52} {"cmd":>5} {"v0":>6} {"d odom":>6} {"d lidar":>7} {"L/O":>5} '
          f'{"t_stop":>6} {"dyaw":>5} {"r0":>6} {"use":>3} {"d@0.4":>6}')
    good = []
    for row in rows:
        ok = usable(row)
        norm = row['d_lidar'] * (V_NORM / row['v0']) ** 2 if ok else None
        if ok:
            good.append((row, norm))
        dl = row['d_lidar']
        print(f'{row["run"]:<52} {row["last_cmd"] or 0:>5.2f} {row["v0"]:>6.3f} '
              f'{row["d_odom"]:>6.3f} {dl if dl is not None else math.nan:>7.3f} '
              f'{dl / row["d_odom"] if ok else math.nan:>5.2f} {row["t_stop"]:>6.2f} '
              f'{row["dyaw_deg"]:>5.1f} {row["r0"] if row["r0"] else math.nan:>6.2f} '
              f'{"Y" if ok else "-":>3} {norm if norm is not None else math.nan:>6.3f}')
    if len(good) < 2:
        print('\nfewer than two usable stops: no measurement')
        return 1
    norms = [n for _, n in good]
    ratios = [r['d_lidar'] / r['d_odom'] for r, _ in good]
    tau, a = fit_latency_decel([(r['v0'], r['d_lidar']) for r, _ in good])
    print(f'\nusable stops: {len(good)} of {len(rows)}')
    print(f'd at {V_NORM} m/s (LiDAR, normalised): median {statistics.median(norms):.3f} '
          f'mean {statistics.mean(norms):.3f} sd {statistics.pstdev(norms):.3f} '
          f'min {min(norms):.3f} max {max(norms):.3f}')
    print(f'LiDAR / odometry on the stop: median {statistics.median(ratios):.2f}')
    print(f'fit d = v*tau + v^2/(2a): tau {tau:.3f} s, a {a:.2f} m/s^2, '
          f'd({V_NORM}) = {tau * V_NORM + V_NORM ** 2 / (2 * a):.3f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
