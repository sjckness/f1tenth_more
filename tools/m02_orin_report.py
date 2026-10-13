#!/usr/bin/env python3
"""M02 R009-R011 post-mortem from the bags (campaign fix A5) -- run ON THE ORIN.

    source install/setup.bash
    python3 tools/m02_orin_report.py \
        --run R009=<run or bag folder>[,<campaign test folder>] \
        --run R010=... --run R011=... \
        [--class "potted plant" --class person] [--out m02_report.md]

A run folder is what mission_logger_node writes (~/f1tenth_archive/complete/
<run>, holding bag/); the optional campaign test folder is
<f1tenth_more>/<campaign>/<MISSION>/P###-R###-<stamp> (corridors.jsonl,
mpc.csv, meta.json). Everything is read from what was recorded; a section
whose inputs are missing says which topics it needed instead of guessing.

Per run:
  1. topics present, counts, duration.
  2. mask/detection timing: arrival lag (receive - header stamp) of
     /camera/detections and /camera/detection_masks; with the depth image
     recorded, depth stamp - stamp of the mask front_clearance_node would
     have used; with the A5 debug record, its mask_age_s.
  3. front_distance: recorded values (min, first time under 2.0 / 1.5 m) and,
     when the bag has the A5 ROI crops or the depth image, a recomputation
     (tools/replay_front_distance.py) next to it.
  4. first appearance of each --class: first 2D detection (time, score, box,
     touching the image border?), its range from /camera/detections_3d, and
     when the LiDAR first had returns at the object's (odom) position -- the
     answer to "placed late, outside the field of view, or detected late".
  5. the stall: the longest window in which /drive asked for 0 < v < 0.4 m/s
     or the car did not move while asked to; solver status counts in it; the
     obstacle (/perception/obstacles_2d) and the corridor (/mpc/corridor_
     markers) at its start: half-width at the obstacle, lateral offset c,
     effective footprint d = r + car_radius + margin, room needed per side,
     and whether any path existed (geometrically, and with the steering
     limit from where the obstacle was first seen).

Values the stack's params decide (car_radius, margin, wheelbase, steering
limit, camera half-FOV) default to the 2026-10-09 repo values; override them
with the flags if the Orin's stack_params differ.
"""

import argparse
import json
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from campaign_bag import (  # noqa: E402
    latest_before, nearest, pct, read_topics, resolve_bag, stamp_s, static_poses,
    yaw_of)

T_DETS = '/camera/detections'
T_MASKS = '/camera/detection_masks'
T_DETS3D = '/camera/detections_3d'
T_OBST = '/perception/obstacles_2d'
T_SCAN = '/scan'
T_ODOM = '/odometry/filtered'
T_TFS = '/tf_static'
T_DRIVE = '/drive'
T_SOLVER = '/mpc/solver_status'
T_CORR = '/mpc/corridor_markers'
T_FD = '/perception/front_distance'
T_FD_RAW = '/perception/debug/front_distance_raw'
T_FDBG = '/perception/debug/front_clearance'
T_CROP_D = '/perception/debug/front_roi_depth'
T_CROP_C = '/perception/debug/front_roi_class'
T_DEPTH = '/zed2/zed_node/depth/depth_registered'
T_HOLD = '/mpc/hold'
T_STATUS = '/mission/status'
T_BLOCKED = '/mpc/blocked'
ALL = [T_DETS, T_MASKS, T_DETS3D, T_OBST, T_SCAN, T_ODOM, T_TFS, T_DRIVE, T_SOLVER,
       T_CORR, T_FD, T_FD_RAW, T_FDBG, T_CROP_D, T_CROP_C, T_DEPTH, T_HOLD, T_STATUS,
       T_BLOCKED]


class Report:
    def __init__(self):
        self.lines = []

    def h(self, text):
        self.lines.append(f'\n## {text}\n')

    def p(self, text=''):
        self.lines.append(text)

    def text(self):
        return '\n'.join(self.lines) + '\n'


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return 'n/a'
    return f'{v:.{nd}f}'


def stats(values, unit='', nd=3):
    v = [x for x in values if x is not None and math.isfinite(x)]
    if not v:
        return 'n/a'
    return (f'n={len(v)} p50={fmt(pct(v, 50), nd)} p95={fmt(pct(v, 95), nd)} '
            f'max={fmt(max(v), nd)}{unit}')


def odom_pose_at(data, t):
    hit = nearest(data[T_ODOM], t)
    if hit is None:
        return None
    p = hit[1].pose.pose
    return (p.position.x, p.position.y, yaw_of(p.orientation),
            hit[1].twist.twist.linear.x)


def to_odom(pose, x, y):
    px, py, yaw = pose[:3]
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * x - s * y, py + s * x + c * y


def to_base(pose, ox, oy):
    px, py, yaw = pose[:3]
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = ox - px, oy - py
    return c * dx + s * dy, -s * dx + c * dy


# ---------------------------------------------------------------------------

def section_topics(r, data, bag):
    r.h('1. Bag')
    r.p(f'bag: `{bag}`')
    times = [t for v in data.values() for t, _ in v]
    if times:
        r.p(f'duration: {max(times) - min(times):.1f} s')
    for topic in ALL:
        r.p(f'- `{topic}`: {len(data[topic])}' + ('' if data[topic] else '  (absent)'))


def section_timing(r, data):
    r.h('2. Mask / detection timing vs depth')
    for topic in (T_DETS, T_MASKS):
        lags = [t - stamp_s(m.header) for t, m in data[topic]]
        r.p(f'- `{topic}` arrival lag (receive - stamp): {stats(lags, " s")}')
    if data[T_DEPTH] and data[T_MASKS]:
        ages = []
        for t, d in data[T_DEPTH]:
            m = latest_before(data[T_MASKS], t)
            if m is not None:
                ages.append(stamp_s(d.header) - stamp_s(m[1].header))
        r.p(f'- depth stamp - stamp of the latest mask received before it '
            f'(what front_clearance_node applied): {stats(ages, " s")}')
    else:
        r.p('- depth image not recorded: the mask age AT USE cannot be measured '
            'directly. It is at least the mask arrival lag above (the node applies '
            'the latest mask to every new depth frame, never checking the stamps).')
    if data[T_FDBG]:
        ages = []
        for _, m in data[T_FDBG]:
            try:
                ages.append(json.loads(m.data).get('mask_age_s'))
            except ValueError:
                pass
        r.p(f'- A5 debug record mask_age_s: {stats(ages, " s")}')


def section_front_distance(r, data, bag):
    r.h('3. front_distance')
    if not data[T_FD]:
        r.p(f'`{T_FD}` was NOT recorded (it is in the mission logger topics only '
            'since the A5 commit). Leak evidence must come from the run log.')
    else:
        vals = [(t, float(m.data)) for t, m in data[T_FD]]
        good = [v for _, v in vals if v > 0]
        r.p(f'recorded: {len(vals)} msgs, min {fmt(min(good) if good else None)} m, '
            f'-1.0 sentinels: {sum(1 for _, v in vals if v < 0)}')
        t0 = vals[0][0]
        for thr in (2.0, 1.5):
            first = next((t for t, v in vals if 0 < v < thr), None)
            r.p(f'- first value under {thr} m: '
                + ('never' if first is None else f't+{first - t0:.2f} s'))
    if (data[T_CROP_D] and data[T_CROP_C]) or (data[T_DEPTH] and data[T_DETS]):
        import replay_front_distance as rfd
        mode = 'crop' if data[T_CROP_D] else 'full'
        rows = (rfd.replay_crops(data, 'band', rfd._params()) if mode == 'crop'
                else rfd.replay_full(data, 'band'))
        diffs = [abs(x['recomputed_fd'] - x['recorded_fd']) for x in rows
                 if x.get('recomputed_fd') is not None and x.get('recorded_fd') is not None
                 and x['recorded_fd'] >= 0]
        r.p(f'recomputed ({mode} mode, {len(rows)} frames): '
            f'|recomputed - recorded| {stats(diffs, " m")}')
    else:
        r.p('recompute: not possible (needs the A5 ROI crops or the depth image).')


def _laser_pose(data):
    poses = static_poses(data[T_TFS])
    if not data[T_SCAN]:
        return None
    frame = data[T_SCAN][0][1].header.frame_id
    p = poses.get(frame)
    return None if p is None else (p[0], p[1], p[3])


def _scan_odom_points(scan, laser, pose):
    ranges = np.asarray(scan.ranges, dtype=float)
    a = scan.angle_min + np.arange(ranges.size) * scan.angle_increment + laser[2]
    ok = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
    bx = laser[0] + ranges[ok] * np.cos(a[ok])
    by = laser[1] + ranges[ok] * np.sin(a[ok])
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * bx - s * by, pose[1] + s * bx + c * by


def section_first_seen(r, data, classes, half_fov_deg):
    r.h('4. First appearance (camera vs LiDAR)')
    if not data[T_DETS]:
        r.p(f'`{T_DETS}` absent.')
        return
    poses = static_poses(data[T_TFS])
    laser = _laser_pose(data)
    t_start = data[T_DETS][0][0]
    for cls in classes:
        first = None
        for t, msg in data[T_DETS]:
            for det in msg.detections:
                hyp = det.results[0].hypothesis if det.results else None
                if hyp is not None and hyp.class_id == cls:
                    first = (t, msg, det, hyp)
                    break
            if first:
                break
        r.p(f'\n### class `{cls}`')
        if first is None:
            r.p('never detected.')
            continue
        t, msg, det, hyp = first
        bag_start = min(series[0][0] for series in data.values() if series)
        if t - bag_start < 0.5:
            r.p(f'- NOTE: already detected in the first {t - bag_start:.2f} s of the bag -- '
                'the bag started with it in view, so when it FIRST appeared is not in '
                'this bag (the mission logger starts on /mission/status RUNNING; see '
                'output/mission_recorder_late_start.md). Use the run log for earlier.')
        cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
        bw, bh = det.bbox.size_x, det.bbox.size_y
        border = (cx - bw / 2 <= 2 or cy - bh / 2 <= 2 or cx + bw / 2 >= 638
                  or cy + bh / 2 >= 358)  # inferred 640x360; see --help
        r.p(f'- first 2D detection: t+{t - t_start:.2f} s, score {hyp.score:.2f}, box '
            f'({cx:.0f},{cy:.0f}) {bw:.0f}x{bh:.0f} px'
            + (' -- TOUCHES THE IMAGE BORDER (640x360 assumed)' if border else ''))
        # 3D position from detections_3d at the same stamp (median over the
        # first ten frames that have the class), in odom.
        pts = []
        for t3, m3 in data[T_DETS3D]:
            if t3 < t - 0.5:
                continue
            frame = poses.get(m3.header.frame_id)
            pose = odom_pose_at(data, t3)
            if frame is None or pose is None:
                continue
            for d3 in m3.detections:
                h3 = d3.results[0].hypothesis if d3.results else None
                if h3 is None or h3.class_id != cls:
                    continue
                p = d3.bbox.center.position
                c, s = math.cos(frame[3]), math.sin(frame[3])
                bx, by = frame[0] + c * p.x - s * p.y, frame[1] + s * p.x + c * p.y
                pts.append((t3, *to_odom(pose, bx, by), math.hypot(bx, by)))
            if len(pts) >= 10:
                break
        if not pts:
            r.p(f'- no `{T_DETS3D}` with this class (or no TF/odometry): range unknown.')
            continue
        ox = float(np.median([p[1] for p in pts]))
        oy = float(np.median([p[2] for p in pts]))
        r.p(f'- first 3D range from base_link: {pts[0][3]:.2f} m; object at odom '
            f'({ox:.2f}, {oy:.2f}) (median of {len(pts)})')
        if laser is None:
            r.p(f'- LiDAR check skipped: no `{T_SCAN}` or no TF for its frame.')
            continue
        lidar_first = None
        in_fov_before = []
        for ts, scan in data[T_SCAN]:
            if ts > t + 0.5:
                break
            pose = odom_pose_at(data, ts)
            if pose is None:
                continue
            xs, ys = _scan_odom_points(scan, laser, pose)
            near = np.hypot(xs - ox, ys - oy) < 0.30
            bx, by = to_base(pose, ox, oy)
            rng = math.hypot(bx, by)
            if lidar_first is None and np.count_nonzero(near) >= 3:
                lidar_first = (ts, rng)
            if ts < t:
                in_fov_before.append((abs(math.degrees(math.atan2(by, bx))) <= half_fov_deg
                                      and bx > 0, rng))
        if lidar_first is None:
            r.p('- LiDAR: no returns at that position before the first detection.')
            if t - bag_start < 0.5:
                verdict = 'UNKNOWN from this bag: it was in view from the first frame'
            else:
                verdict = ('APPEARED LATE (placed or uncovered while the car approached): '
                           'neither sensor had it earlier')
        else:
            r.p(f'- LiDAR first returns at the position: t+{lidar_first[0] - t_start:.2f} s, '
                f'{lidar_first[1]:.2f} m from base_link')
            fov = [rng for ok, rng in in_fov_before if ok]
            r.p(f'- before the first detection it was inside the camera half-FOV '
                f'({half_fov_deg:.1f} deg) for {len(fov)} scans'
                + (f', at {min(fov):.2f}-{max(fov):.2f} m' if fov else ''))
            if lidar_first[1] > pts[0][3] + 0.5 and fov:
                verdict = 'DETECTED LATE: the LiDAR saw it earlier and it was in the field of view'
            elif lidar_first[1] > pts[0][3] + 0.5:
                verdict = 'OUTSIDE THE CAMERA FIELD OF VIEW until it was detected'
            else:
                verdict = 'APPEARED LATE: the LiDAR also first saw it at about that range'
        r.p(f'- **verdict (inferred from the above): {verdict}**')


def _corridor_at(data, t):
    """(centre, left, right) polylines (odom) of the last corridor published at or before t."""
    hit = latest_before(data[T_CORR], t)
    if hit is None:
        return None
    lines = {}
    for m in hit[1].markers:
        lines[m.ns] = np.array([[p.x, p.y] for p in m.points])
    if not {'corridor_centerline', 'corridor_left', 'corridor_right'} <= set(lines):
        return None
    return lines['corridor_centerline'], lines['corridor_left'], lines['corridor_right']


def section_stall(r, data, args):
    r.h('5. The stall: corridor, footprint, gap, solver')
    if not data[T_DRIVE]:
        r.p(f'`{T_DRIVE}` absent.')
        return
    # Stall ticks: asked to move below the floor, or asked to move and not moving.
    stall = []
    for t, m in data[T_DRIVE]:
        v_cmd = m.drive.speed
        pose = odom_pose_at(data, t)
        v = pose[3] if pose else math.nan
        stall.append((t, (0.0 < v_cmd < 0.4) or (v_cmd > 0.0 and abs(v) < 0.05), v_cmd,
                      m.drive.steering_angle))
    best, cur = None, None
    for t, s, *_ in stall:
        if s:
            cur = [t, t] if cur is None else [cur[0], t]
        else:
            if cur and (best is None or cur[1] - cur[0] > best[1] - best[0]):
                best = cur
            cur = None
    if cur and (best is None or cur[1] - cur[0] > best[1] - best[0]):
        best = cur
    if best is None or best[1] - best[0] < 1.0:
        r.p('no stall longer than 1 s.')
        return
    t0, t1 = best
    inside = [x for x in stall if t0 <= x[0] <= t1]
    r.p(f'stall: {t1 - t0:.1f} s (from t+{t0 - data[T_DRIVE][0][0]:.1f} s); commanded '
        f'speed {stats([x[2] for x in inside], " m/s")}; steering '
        f'{stats([abs(x[3]) for x in inside], " rad")}')
    sol = [m for t, m in data[T_SOLVER] if t0 <= t <= t1]
    if sol:
        counts = {}
        for m in sol:
            key = f'{m.status_message or m.status} (success={m.success})'
            counts[key] = counts.get(key, 0) + 1
        r.p('solver status in the stall: ' + ', '.join(
            f'{k}: {v}' for k, v in sorted(counts.items(), key=lambda kv: -kv[1])))
        r.p(f'solve time: {stats([m.solve_dt_sec * 1e3 for m in sol], " ms", 1)}')
    else:
        r.p(f'`{T_SOLVER}` absent in the stall.')

    pose = odom_pose_at(data, t0)
    obst = latest_before(data[T_OBST], t0 + 0.5)
    corr = _corridor_at(data, t0)
    if pose is None or obst is None or not obst[1].obstacles:
        r.p(f'no obstacle (`{T_OBST}`) or odometry at the stall start.')
        return
    # The obstacle nearest ahead, in odom.
    ahead = [(o.x, o.y, o.r) for o in obst[1].obstacles if o.x > 0]
    if not ahead:
        r.p('no obstacle ahead at the stall start.')
        return
    bx, by, rr = min(ahead, key=lambda o: math.hypot(o[0], o[1]))
    ox, oy = to_odom(pose, bx, by)
    r.p(f'obstacle (nearest ahead, `{T_OBST}`): base_link ({bx:.2f}, {by:.2f}), r {rr:.2f} m '
        f'(r = box width / 2, centred on the visible FRONT surface)')
    if corr is None:
        r.p(f'`{T_CORR}` absent: corridor geometry unknown.')
        return
    centre, left, right = corr
    i = int(np.argmin(np.hypot(centre[:, 0] - ox, centre[:, 1] - oy)))
    j = min(i + 1, len(centre) - 1)
    k = max(i - 1, 0)
    tx, ty = centre[j] - centre[k]
    n = np.array([-ty, tx]) / max(math.hypot(tx, ty), 1e-9)
    c = float(n @ (np.array([ox, oy]) - centre[i]))
    hw_l = float(np.min(np.hypot(left[:, 0] - centre[i][0], left[:, 1] - centre[i][1])))
    hw_r = float(np.min(np.hypot(right[:, 0] - centre[i][0], right[:, 1] - centre[i][1])))
    d = rr + args.car_radius + args.margin
    left_needed, right_needed = c + d, d - c
    r.p(f'corridor at the obstacle: half-width left {hw_l:.3f} / right {hw_r:.3f} m '
        f'(the rows bound the REAR AXLE); obstacle lateral offset c = {c:+.3f} m (+ left)')
    r.p(f'effective footprint d = r {rr:.2f} + car_radius {args.car_radius:.2f} + margin '
        f'{args.margin:.2f} = {d:.3f} m')
    r.p(f'- pass LEFT needs the axle at +{left_needed:.3f} m, corridor allows +{hw_l:.3f}: '
        + ('fits' if left_needed <= hw_l else f'short by {left_needed - hw_l:.3f} m'))
    r.p(f'- pass RIGHT needs the axle at -{right_needed:.3f} m, corridor allows -{hw_r:.3f}: '
        + ('fits' if right_needed <= hw_r else f'short by {right_needed - hw_r:.3f} m'))
    r.p(f'- free gap obstacle edge -> wall: left {hw_l - c - rr:+.3f} m, right '
        f'{hw_r + c - rr:+.3f} m (vs car_radius + margin {args.car_radius + args.margin:.2f})')
    geometric = left_needed <= hw_l or right_needed <= hw_r
    r_min = args.wheelbase / math.tan(args.delta_max)
    y_need = min(abs(left_needed - 0.0), abs(right_needed))
    # Two-arc S-curve at R_min: lateral y over forward D = 2 R sin(theta),
    # y = 2 R (1 - cos theta).
    if y_need >= 2 * r_min:
        d_need = math.inf
    else:
        cos_t = 1.0 - y_need / (2 * r_min)
        d_need = 2 * r_min * math.sqrt(max(1.0 - cos_t ** 2, 0.0))
    avail = bx - rr - args.nose - args.margin
    r.p(f'- steering: R_min = wheelbase {args.wheelbase} / tan({args.delta_max}) = '
        f'{r_min:.3f} m; shifting the axle {y_need:.3f} m with an S-curve needs '
        f'{fmt(d_need)} m of travel; available from the stall start: {avail:.2f} m '
        f'(obstacle front - nose {args.nose} - margin)')
    r.p(f'- **a feasible path existed (inferred): geometric {"YES" if geometric else "NO"}, '
        f'with the steering limit from the stall start '
        f'{"YES" if geometric and d_need <= avail else "NO"}**')


def run_one(label, bag_path, test_dir, args):
    r = Report()
    r.p(f'# {label}')
    bag = resolve_bag(bag_path)
    data = read_topics(bag, ALL)
    for v in data.values():
        v.sort(key=lambda x: x[0])
    section_topics(r, data, bag)
    section_timing(r, data)
    section_front_distance(r, data, bag)
    section_first_seen(r, data, args.cls or ['potted plant', 'person'], args.half_fov_deg)
    section_stall(r, data, args)
    if data[T_BLOCKED]:
        r.p(f'\nBLOCKED published: {data[T_BLOCKED][0][1].data}')
    if test_dir:
        r.h('6. Campaign test folder')
        td = pathlib.Path(test_dir).expanduser()
        meta = td / 'meta.json'
        if meta.exists():
            m = json.loads(meta.read_text())
            s = m.get('summary', {})
            r.p(f'outcome {s.get("auto_outcome")} reason {s.get("reason")!r} duration '
                f'{s.get("duration_s")} s')
        for name in ('corridors.jsonl', 'mpc.csv', 'events.jsonl', 'front.jsonl'):
            p = td / name
            r.p(f'- {name}: ' + (f'{sum(1 for _ in open(p))} lines' if p.exists()
                                 else 'absent'))
        corr = td / 'corridors.jsonl'
        if corr.exists():
            widths = set()
            for line in open(corr):
                try:
                    d = json.loads(line).get('meta', {}).get('definition', {}) or {}
                except ValueError:
                    continue
                widths.add((d.get('w0'), d.get('w1'), d.get('L')))
            r.p(f'- corridor (w0, w1, L) used: {sorted(widths, key=str)}')
    return r.text()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument('--run', action='append', required=True,
                    help='LABEL=<run or bag folder>[,<campaign test folder>]')
    ap.add_argument('--class', dest='cls', action='append',
                    help='detection class name to trace (repeatable); default '
                         '"potted plant" and person')
    ap.add_argument('--out', default=None, help='write the markdown report here too')
    ap.add_argument('--car-radius', type=float, default=0.20)
    ap.add_argument('--margin', type=float, default=0.12, help='obstacle_safety_margin_m')
    ap.add_argument('--wheelbase', type=float, default=0.305)
    ap.add_argument('--delta-max', type=float, default=0.319,
                    help='steering bound [rad] (mpc_steering_angle_max_rad)')
    ap.add_argument('--nose', type=float, default=None,
                    help='front bumper x [m]; default: the URDF base_front '
                         '(f1tenth_description.vehicle_frames, workspace sourced)')
    ap.add_argument('--half-fov-deg', type=float, default=50.5,
                    help='camera half horizontal FOV (fx 262.4, 640 px)')
    args = ap.parse_args()
    if args.nose is None:
        try:
            from f1tenth_description.vehicle_frames import base_front_x
        except ImportError:
            ap.error('--nose is required when f1tenth_description is not importable '
                     '(source the workspace, or pass the front bumper x)')
        args.nose = base_front_x()
    out = []
    for spec in args.run:
        label, _, rest = spec.partition('=')
        bag_path, _, test_dir = rest.partition(',')
        try:
            out.append(run_one(label, bag_path, test_dir or None, args))
        except Exception as exc:  # noqa: BLE001 - one bad run must not hide the others
            out.append(f'# {label}\n\nFAILED: {type(exc).__name__}: {exc}\n')
    text = '\n'.join(out)
    print(text)
    if args.out:
        pathlib.Path(args.out).write_text(text)
        print(f'wrote {args.out}', file=sys.stderr)


if __name__ == '__main__':
    main()
