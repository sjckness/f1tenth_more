#!/usr/bin/env python3
"""Front-wall calibration check on static raw clips, offline.

Usage (after `source install/setup.bash`):
    python3 tools/calib_wall_check.py CLIP [CLIP ...] [--json OUT]

CLIP is a raw clip folder (tools/record_raw.sh) already replayed with
tools/replay_raw.launch.py; its newest replay_*/ is used. The ground truth is
the front bumper (base_front) to wall distance parsed from the clip name
(`..._1m`, `..._2m`). The car stands still, square to a flat wall.

Static window: the longest stretch with |/odometry/filtered v| < 1 cm/s,
0.5 s trimmed at each end. Per clip, mean +- std over that window of:

  align  image stamps (/camera/camera_info) vs the nearest /scan stamp, and
         sim /clock at reception minus the image stamp (does the replay clock
         agree with the SVO stamps).
  a      LiDAR: a line fitted to the scan points ahead (|y| < 0.5 m) in
         base_link; distance from base_front and the wall normal's angle. The
         car is square, so a nonzero angle is the laser yaw error; the implied
         laser yaw offset is minus that angle.
  b      stereo: median base_link x of the depth pixels in a central wall ROI
         above the horizon, minus base_front x.
  c      camera ground row: the image row where the floor meets the wall (the
         dark gap under the skirting board: darkest row of the central columns'
         median intensity, sub-pixel, in the SVO's left image at the depth
         resolution), then that row's ray cast onto the floor z = 0 with the TF
         camera pose, as ModelLabeller in scene_labels_replay.py casts its rays.
         Its error is the camera pitch / height error. Not from depth: on these
         glossy, textureless floor tiles the stereo floor is wrong (it reads
         the wall's depth at 1 m and grows downwards at 2 m).
  d      /perception/walls front track d_perp_front, and /perception/virtual_wall
         (both from the front bumper).
  e      camera pitch (nose down +) and roll (about +x) from the mean ZED IMU
         specific force, rotated into the camera frame with the SVO's
         camera-IMU transform.

Then the c rows of exactly two clips give camera pitch and height above the
floor (2 equations, 2 unknowns: an exact solve, an estimate, no redundancy).
Nothing is applied anywhere.
"""

import argparse
import glob
import json
import math
import os
import re
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import brentq

from f1tenth_perception.ground_plane import quat_to_matrix

DEPTH = '/zed2/zed_node/depth/depth_registered'
DEPTH_INFO = '/zed2/zed_node/depth/camera_info'
WALL_ROWS = (-40, -5)       # ROI rows relative to cy: wall, above the horizon
ROI_HALF_W = 60             # central columns cx +- this
CORNER_BAND = (-5, 25)      # corner search rows around the zero-pitch prediction


def read(db3, topics):
    """{topic: [(recv_ns, msg)]} from a sqlite3 bag, opened read-only."""
    con = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
    types = {n: t for _, n, t in con.execute('SELECT id, name, type FROM topics')}
    ids = {n: i for i, n, _ in con.execute('SELECT id, name, type FROM topics')}
    out = {}
    for topic in topics:
        if topic not in ids:
            out[topic] = []
            continue
        cls = get_message(types[topic])
        out[topic] = [(t, deserialize_message(d, cls)) for t, d in con.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp',
            (ids[topic],))]
    con.close()
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def tf_tree(tf_msgs):
    """to_base(frame) -> (R, t) of frame in base_link, from /tf_static."""
    edges = {}
    for _, m in tf_msgs:
        for tr in m.transforms:
            q, p = tr.transform.rotation, tr.transform.translation
            edges[tr.child_frame_id] = (tr.header.frame_id, quat_to_matrix(q.x, q.y, q.z, q.w),
                                        np.array([p.x, p.y, p.z]))

    def to_base(frame):
        rot, trans = np.eye(3), np.zeros(3)
        while frame != 'base_link':
            parent, r, t = edges[frame]
            rot, trans = r @ rot, r @ trans + t
            frame = parent
        return rot, trans
    return to_base


def static_window(odom):
    t = np.array([stamp(m) for _, m in odom])
    still = np.array([abs(m.twist.twist.linear.x) < 0.01 for _, m in odom])
    best, start = (0.0, 0.0), None
    for ti, s in zip(np.append(t, t[-1]), np.append(still, False)):
        if s and start is None:
            start = ti
        elif not s and start is not None:
            best = max(best, (start, ti), key=lambda w: w[1] - w[0])
            start = None
    return best[0] + 0.5, best[1] - 0.5


def ms(values):
    v = np.asarray([x for x in values if np.isfinite(x)], float)
    return (float(v.mean()), float(v.std()), int(v.size)) if v.size else (math.nan, math.nan, 0)


def lidar_wall(scan, rot, trans, front_x):
    a = scan.angle_min + scan.angle_increment * np.arange(len(scan.ranges))
    r = np.asarray(scan.ranges)
    ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, 10.0)) & (abs(a) < 0.6)
    pts = (rot @ np.stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r)))[:, ok]).T + trans
    pts = pts[np.abs(pts[:, 1]) < 0.5, :2]
    if len(pts) < 20:
        return math.nan, math.nan
    keep = np.abs(pts[:, 0] - np.median(pts[:, 0])) < 0.15
    for _ in range(3):
        c = pts[keep].mean(0)
        n = np.linalg.svd(pts[keep] - c)[2][-1]
        n = n if n @ c > 0 else -n
        keep = np.abs((pts - c) @ n) < 0.02
    d = float(c @ n)
    return d - front_x * n[0], math.degrees(math.atan2(n[1], n[0]))


def depth_frame(msg):
    return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)


def stereo_wall(z, k, rot, trans, front_x):
    """b: median base_link x of the central wall ROI, minus base_front x."""
    fx, fy, cx, cy = k
    c0, c1 = int(cx) - ROI_HALF_W, int(cx) + ROI_HALF_W + 1
    r0, r1 = int(cy) + WALL_ROWS[0], int(cy) + WALL_ROWS[1]
    vv, uu = np.mgrid[r0:r1, c0:c1]
    zz = z[r0:r1, c0:c1]
    ok = np.isfinite(zz) & (zz > 0.2)
    if ok.sum() < 200:
        return math.nan
    rays = np.stack(((uu[ok] - cx) / fx, (vv[ok] - cy) / fy, np.ones(ok.sum())))
    return float(np.median((rot @ (rays * zz[ok]))[0] + trans[0])) - front_x


def floor_distance(v, k, rot, trans, front_x):
    """The ray of row v (principal column) cast onto z = 0, minus base_front x."""
    ray = rot @ np.array([0.0, (v - k[3]) / k[1], 1.0])
    if ray[2] >= 0:
        return math.nan
    return float(trans[0] - trans[2] / ray[2] * ray[0] - front_x)


def corner_rows(svo, window, k, size, v_pred, step=3):
    """Floor/wall corner row per SVO frame in window: the darkest central row."""
    import pyzed.sl as sl
    cam = sl.Camera()
    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    init.depth_mode = sl.DEPTH_MODE.NONE
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        return []
    img, rows, i = sl.Mat(), [], 0
    c0, c1 = int(k[2]) - ROI_HALF_W, int(k[2]) + ROI_HALF_W + 1
    r0, r1 = int(v_pred) + CORNER_BAND[0], int(v_pred) + CORNER_BAND[1]
    while cam.grab() == sl.ERROR_CODE.SUCCESS:
        t = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() * 1e-9
        i += 1
        if t > window[1]:
            break
        if t < window[0] or i % step:
            continue
        cam.retrieve_image(img, sl.VIEW.LEFT_GRAY, sl.MEM.CPU, sl.Resolution(*size))
        prof = np.median(img.get_data()[r0:r1, c0:c1].astype(float), axis=1)
        j = int(np.clip(np.argmin(prof), 1, len(prof) - 2))
        den = prof[j - 1] - 2 * prof[j] + prof[j + 1]
        rows.append(r0 + j + (0.5 * (prof[j - 1] - prof[j + 1]) / den if den > 0 else 0.0))
    cam.close()
    return rows


def camera_imu_rotation(svo):
    import pyzed.sl as sl
    cam = sl.Camera()
    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        return np.eye(3)
    rot = np.array(cam.get_camera_information().sensors_configuration
                   .camera_imu_transform.get_rotation_matrix().r)
    cam.close()
    return rot


def check(clip):
    clip = Path(clip).expanduser().resolve()
    truth = float(re.search(r'_(\d+(?:\.\d+)?)m', clip.name).group(1))
    replay = sorted(clip.glob('replay_*/bag/*.db3'))[-1]
    src = read(sorted((clip / 'bag').glob('*.db3'))[0],
               ['/scan', '/odometry/filtered', '/tf_static', '/zed2/zed_node/imu/data'])
    rep = read(replay, ['/camera/camera_info', DEPTH, DEPTH_INFO, '/perception/walls',
                        '/perception/virtual_wall'])
    to_base = tf_tree(src['/tf_static'])
    front_x = to_base('base_front')[1][0]
    t0, t1 = static_window(src['/odometry/filtered'])
    win = lambda t: t0 <= t <= t1  # noqa: E731
    res = {'clip': clip.name, 'replay': str(replay.parent.parent), 'truth_m': truth,
           'window_s': [t0, t1], 'front_x': float(front_x)}

    # Alignment.
    scan_t = np.array([stamp(m) for _, m in src['/scan']])
    img = [(t * 1e-9, stamp(m)) for t, m in rep['/camera/camera_info']]
    img = [(r, s) for r, s in img if scan_t[0] <= s <= scan_t[-1]]
    img_t = np.array([s for _, s in img])
    idx = np.clip(np.searchsorted(scan_t, img_t), 1, len(scan_t) - 1)
    near = np.minimum(abs(img_t - scan_t[idx - 1]), abs(img_t - scan_t[idx]))
    lag = np.array([r - s for r, s in img])
    res['align'] = {
        'img_scan_ms_median': float(np.median(near) * 1e3),
        'img_scan_ms_p95': float(np.percentile(near, 95) * 1e3),
        'clock_minus_img_ms_median': float(np.median(lag) * 1e3),
        'clock_minus_img_ms_p95': float(np.percentile(lag, 95) * 1e3),
        'scan_period_ms': float(np.median(np.diff(scan_t)) * 1e3),
        'n_img': int(len(img_t))}

    # a: LiDAR.
    rl, tl = to_base('laser')
    fits = [lidar_wall(m, rl, tl, front_x) for _, m in src['/scan'] if win(stamp(m))]
    res['a_lidar_m'] = ms([f[0] for f in fits])
    res['a_lidar_angle_deg'] = ms([f[1] for f in fits])

    # b, c: stereo depth.
    info = rep[DEPTH_INFO][0][1]
    k = (info.k[0], info.k[4], info.k[2], info.k[5])
    rc, tc = to_base(rep[DEPTH][0][1].header.frame_id)
    res['b_stereo_m'] = ms([stereo_wall(depth_frame(m), k, rc, tc, front_x)
                            for _, m in rep[DEPTH] if win(stamp(m))])
    svo = glob.glob(str(clip / '*.svo2'))[0]
    x_cam = res['b_stereo_m'][0] + front_x - tc[0]
    rows = corner_rows(svo, (t0, t1), k, (info.width, info.height),
                       k[3] + k[1] * tc[2] / x_cam)
    res['c_ground_row_m'] = ms([floor_distance(v, k, rc, tc, front_x) for v in rows])
    res['c_corner_row'] = ms(rows)
    res['camera'] = {'fy': k[1], 'cy': k[3], 'x': float(tc[0]), 'z': float(tc[2]),
                     'pitch_tf_deg': math.degrees(math.asin(-(rc @ [0, 0, 1])[2]))}

    # d: tracker and virtual wall.
    front = []
    for _, m in rep['/perception/walls']:
        if win(stamp(m)):
            d = [t.d_perp_front for t in m.tracks if t.label == 'front' and t.state == 1]
            front.append(min(d) if d else math.nan)
    res['d_walls_front_m'] = ms(front)
    res['d_virtual_wall_m'] = ms([m.data for t, m in rep['/perception/virtual_wall']
                                  if win(t * 1e-9) and m.data >= 0])

    # e: ZED IMU gravity.
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
                    for _, m in src['/zed2/zed_node/imu/data'] if win(stamp(m))])
    a_cam = camera_imu_rotation(svo) @ acc.mean(0)
    res['e_imu'] = {
        'pitch_deg': math.degrees(math.atan2(-a_cam[0], math.hypot(a_cam[1], a_cam[2]))),
        'roll_deg': math.degrees(math.atan2(a_cam[1], a_cam[2])),
        'pitch_raw_deg': math.degrees(math.atan2(-acc.mean(0)[0], math.hypot(*acc.mean(0)[1:]))),
        'g': float(np.linalg.norm(acc.mean(0))), 'n': int(len(acc))}
    return res


def fit_pitch_height(r1, r2):
    """Camera pitch (nose down +) and height above the floor from two corner rows."""
    cam = r1['camera']
    obs = []
    for r in (r1, r2):
        v = r['c_corner_row'][0]
        obs.append((math.atan((v - cam['cy']) / cam['fy']),
                    r['truth_m'] + r['front_x'] - cam['x']))
    (a1, x1), (a2, x2) = obs
    f = lambda hh: math.atan(hh / x1) - a1 - (math.atan(hh / x2) - a2)  # noqa: E731
    h = brentq(f, 0.05, 1.0)
    pitch = math.degrees(math.atan(h / x1) - a1)
    return {'pitch_deg': pitch, 'height_m': h,
            'pitch_offset_deg': pitch - cam['pitch_tf_deg'], 'height_offset_m': h - cam['z']}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--fit', nargs=2, type=int, default=None, metavar=('I', 'J'),
                    help='clip indices for the pitch/height solve (default: the first two)')
    ap.add_argument('--json', type=Path, default=None)
    cfg = ap.parse_args()
    results = [check(c) for c in cfg.clips]

    f = lambda m: '{:.3f} ± {:.3f}'.format(m[0], m[1])  # noqa: E731
    print('| clip | truth | a LiDAR | a angle deg | b stereo | c ground row | '
          'd walls front | d virtual wall |')
    print('|---|---|---|---|---|---|---|---|')
    for r in results:
        print(f"| {r['clip']} | {r['truth_m']:.2f} | {f(r['a_lidar_m'])} | "
              f"{f(r['a_lidar_angle_deg'])} | {f(r['b_stereo_m'])} | {f(r['c_ground_row_m'])} | "
              f"{f(r['d_walls_front_m'])} | {f(r['d_virtual_wall_m'])} |")
    for r in results:
        al, e = r['align'], r['e_imu']
        print(f"{r['clip']}: window {r['window_s'][1] - r['window_s'][0]:.1f} s; "
              f"img-scan {al['img_scan_ms_median']:.1f}/{al['img_scan_ms_p95']:.1f} ms "
              f"(median/p95, scan period {al['scan_period_ms']:.1f}); clock-img "
              f"{al['clock_minus_img_ms_median']:.1f}/{al['clock_minus_img_ms_p95']:.1f} ms; "
              f"corner row {r['c_corner_row'][0]:.2f}; IMU pitch {e['pitch_deg']:.3f} "
              f"roll {e['roll_deg']:.3f} deg (raw pitch {e['pitch_raw_deg']:.3f}, |g| {e['g']:.3f})")
    out = {'clips': results}
    pair = cfg.fit or ([0, 1] if len(results) >= 2 else None)
    if pair:
        out['fit'] = fit_pitch_height(results[pair[0]], results[pair[1]])
        print('fit ({} + {}): pitch {pitch_deg:.3f} deg (offset {pitch_offset_deg:+.3f}), '
              'height {height_m:.4f} m (offset {height_offset_m:+.4f})'.format(
                  results[pair[0]]['clip'], results[pair[1]]['clip'], **out['fit']))
    if cfg.json:
        cfg.json.write_text(json.dumps(out, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
