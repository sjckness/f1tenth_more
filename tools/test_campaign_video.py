#!/usr/bin/env python3
"""Replay video of one test-campaign folder, from its logged data alone.

Usage (no ROS needed; from the workspace root):
    PYTHONPATH=src/f1tenth_logger:src/f1tenth_params \
        python3 tools/test_campaign_video.py <test folder> [...] [--out-dir DIR]
        [--fps 10] [--pre-roll 2.0] [--full]

Written for the 2026-10-19 M02 session (P002-R012..R020), which ran without
the mission logger, so there is no bag and no camera stream to render. What a
test folder does hold is enough for a top-down replay:

  kinematics.csv       pose, speed, corridor/obstacle clearance (test time t)
  commands.csv         commanded speed and steering
  corridors.jsonl      every corridor rebuild; drawn with corridor_plot's own
                       v2 evaluation, so the walls are the planner's curves
  horizon.jsonl        each MPC solve's predicted horizon
  mpc.csv              solver status per solve
  corridor_debug.jsonl the MPC's obstacle set (obstacles_world) and target
  tracks.jsonl         the perception tracks, labelled by class
  events.jsonl         mission_started / estop / contact / ... banners

Frames are on the test clock (t in kinematics.csv), played back in real time
at --fps. By default the countdown is cut: the video starts --pre-roll s
before mission_started (or at the first sample when the mission never
started). --full keeps the whole test.

corridor_debug.jsonl is stamped in ROS time; meta.json's
extra_meta.ros_start_time puts it on the test clock. tracks.jsonl is in the
'map' frame and everything else in 'odom'; on this car the two coincide
(the MPC's obstacles_world, which is odom, holds the same positions), and the
video draws both as logged, without a transform.

Output: <out-dir>/<test_id>.mp4 (default: <workspace>/mission_videos/
test_campaign, gitignored).
"""

import argparse
import bisect
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

from f1tenth_logger.test_campaign.corridor_def import (  # noqa: E402
    SCHEMA_V2, evaluate, load_corridors,
)
from f1tenth_logger.test_campaign.horizon_log import load_horizons  # noqa: E402

WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_OUT = WORKSPACE / 'mission_videos' / 'test_campaign'

#: Car footprint in base_link (backfill.json: 0.525 x 0.272 m, tail at -0.082).
FOOT_LEN = 0.525
FOOT_WID = 0.272
FOOT_TAIL = -0.082

WALL_COLOR = 'tab:blue'
FILL_COLOR = 'tab:cyan'
CENTER_COLOR = 'teal'
HORIZON_COLOR = 'tab:orange'
OBST_COLOR = 'tab:red'
TRACK_COLOR = 'tab:purple'

#: Events worth a banner, and how long the banner stays up [s].
BANNER_EVENTS = {
    'mission_loaded', 'mission_started', 'mission_finished', 'mission_aborted',
    'estop', 'contact', 'delivery_failed', 'stale_mission_event', 'test_end',
}
BANNER_S = 2.0
ALARM_EVENTS = {'estop', 'contact', 'mission_aborted', 'delivery_failed'}


def _f(text):
    try:
        v = float(text)
    except (TypeError, ValueError):
        return None
    return v if v == v else None


def read_csv(path, columns):
    out = {c: [] for c in columns}
    if not path.exists():
        return out
    with open(path, newline='', encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            t = _f(row.get('t'))
            if t is None:
                continue
            for c in columns:
                out[c].append(t if c == 't' else
                              (row.get(c) if c == 'status' else _f(row.get(c))))
    return out


def read_jsonl(path):
    if not path.exists():
        return []
    out = []
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return out


def last_before(times, t):
    """Index of the last entry with time <= t, or None."""
    i = bisect.bisect_right(times, t) - 1
    return i if i >= 0 else None


def footprint(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(FOOT_TAIL, -FOOT_WID / 2), (FOOT_TAIL + FOOT_LEN, -FOOT_WID / 2),
           (FOOT_TAIL + FOOT_LEN, FOOT_WID / 2), (FOOT_TAIL, FOOT_WID / 2)]
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in pts]


def corridor_shapes(records):
    """(t, fill polygon, left wall, right wall, centreline or None) per rebuild."""
    shapes = []
    for r in records:
        if r.t is None:
            continue
        curves = evaluate(r) if r.schema == SCHEMA_V2 else None
        if curves is not None:
            xl, yl = list(curves['xL']), list(curves['yL'])
            xr, yr = list(curves['xR']), list(curves['yR'])
            fill = list(zip(xl + xr[::-1], yl + yr[::-1]))
            shapes.append((r.t, fill, (xl, yl), (xr, yr),
                           (list(curves['xc']), list(curves['yc']))))
        elif len(r.polygon) >= 3:
            poly = list(r.polygon)
            xs = [p[0] for p in poly] + [poly[0][0]]
            ys = [p[1] for p in poly] + [poly[0][1]]
            shapes.append((r.t, poly, (xs, ys), ([], []), None))
    shapes.sort(key=lambda s: s[0])
    return shapes


class TestData:
    def __init__(self, test_dir):
        d = Path(test_dir)
        self.dir = d
        self.meta = json.loads((d / 'meta.json').read_text(encoding='utf-8'))
        summary = self.meta.get('summary', self.meta)
        self.test_id = summary.get('test_id') or d.name
        self.mission = summary.get('mission') or d.parent.name
        self.outcome = summary.get('auto_outcome') or '?'
        self.reason = summary.get('reason') or ''
        self.prompt = self.meta.get('prompt_text', '')
        self.criterion = self.meta.get('success_criterion', '')

        self.kin = read_csv(d / 'kinematics.csv', [
            't', 'x', 'y', 'yaw', 'speed', 'corridor_clearance',
            'obstacle_clearance'])
        keep = [i for i, (x, y, yaw) in enumerate(zip(
            self.kin['x'], self.kin['y'], self.kin['yaw']))
            if None not in (x, y, yaw)]
        self.kin = {k: [v[i] for i in keep] for k, v in self.kin.items()}
        self.cmd = read_csv(d / 'commands.csv', ['t', 'cmd_speed', 'cmd_steer'])
        self.mpc = read_csv(d / 'mpc.csv', ['t', 'status', 'solve_time_ms'])

        self.corridors = corridor_shapes(load_corridors(d / 'corridors.jsonl')
                                         if (d / 'corridors.jsonl').exists() else [])
        self.corr_t = [s[0] for s in self.corridors]
        hz = load_horizons(d / 'horizon.jsonl') if (d / 'horizon.jsonl').exists() else []
        self.horizons = sorted((h for h in hz if h.t is not None and len(h)),
                               key=lambda h: h.t)
        self.hor_t = [h.t for h in self.horizons]

        ros0 = (self.meta.get('extra_meta') or {}).get('ros_start_time')
        self.debug = []
        for r in read_jsonl(d / 'corridor_debug.jsonl'):
            t = r.get('t')
            if t is None:
                continue
            if ros0 is not None and t > 1e8:
                t -= ros0
            elif t > 1e8:
                continue
            self.debug.append((t, r.get('obstacles_world') or [], r.get('target')))
        self.debug.sort(key=lambda r: r[0])
        self.debug_t = [r[0] for r in self.debug]

        self.tracks = sorted(((r['t'], r.get('tracks') or [])
                              for r in read_jsonl(d / 'tracks.jsonl')
                              if r.get('t') is not None), key=lambda r: r[0])
        self.tracks_t = [r[0] for r in self.tracks]

        self.events = []
        seen = set()
        for e in read_jsonl(d / 'events.jsonl'):
            key = (e.get('t'), e.get('event'))
            if key in seen or e.get('t') is None:
                continue
            seen.add(key)
            self.events.append(e)
        self.events.sort(key=lambda e: e['t'])

    def event_time(self, name):
        for e in self.events:
            if e.get('event') == name:
                return e['t']
        return None

    def time_span(self, pre_roll, full):
        t_end = max([self.kin['t'][-1]] if self.kin['t'] else [0.0])
        for e in self.events:
            t_end = max(t_end, e['t'])
        t0 = self.kin['t'][0] if self.kin['t'] else 0.0
        started = self.event_time('mission_started')
        if not full and started is not None:
            t0 = max(t0, started - pre_roll)
        return t0, t_end

    def extent(self):
        xs = list(self.kin['x'])
        ys = list(self.kin['y'])
        for _, fill, *_ in self.corridors:
            xs += [p[0] for p in fill]
            ys += [p[1] for p in fill]
        for _, obs, _ in self.debug:
            for o in obs:
                if isinstance(o, dict) and 'x' in o:
                    xs.append(o['x'])
                    ys.append(o['y'])
        if not xs:
            xs, ys = [0.0], [0.0]
        x0, x1 = min(xs) - 0.8, max(xs) + 0.8
        y0, y1 = min(ys) - 0.8, max(ys) + 0.8
        # Tracks can be far off to the side; widen for them only up to a cap.
        for _, trs in self.tracks:
            for tr in trs:
                tx, ty = tr.get('x'), tr.get('y')
                if tx is None or ty is None:
                    continue
                if x0 - 2 < tx < x1 + 2 and y0 - 2 < ty < y1 + 2:
                    x0, x1 = min(x0, tx - 0.5), max(x1, tx + 0.5)
                    y0, y1 = min(y0, ty - 0.5), max(y1, ty + 0.5)
        return x0, x1, y0, y1


def render(test_dir, out_dir, fps=10, pre_roll=2.0, full=False, dpi=100):
    D = TestData(test_dir)
    t0, t1 = D.time_span(pre_roll, full)
    n_frames = max(1, int(math.ceil((t1 - t0) * fps)) + 1)

    fig = plt.figure(figsize=(14.4, 8.1), dpi=dpi)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.75, 1], hspace=0.38,
                          wspace=0.16, left=0.05, right=0.975, top=0.87,
                          bottom=0.07)
    ax = fig.add_subplot(gs[:, 0])
    ax_v = fig.add_subplot(gs[0, 1])
    ax_s = fig.add_subplot(gs[1, 1], sharex=ax_v)
    ax_c = fig.add_subplot(gs[2, 1], sharex=ax_v)

    head = f'{D.test_id}   {D.mission}   auto: {D.outcome}'
    if D.reason:
        head += f' ({D.reason})'
    fig.text(0.05, 0.955, head, fontsize=13, weight='bold', va='center')
    fig.text(0.05, 0.92, f'prompt: "{D.prompt}"   criterion: {D.criterion}',
             fontsize=9.5, va='center', color='0.25')

    # --- top-down ---
    x0, x1, y0, y1 = D.extent()
    # Equal scale, with the short side padded out to the axes' own shape so
    # nothing in the extent is cropped.
    fig.canvas.draw()
    bbox = ax.get_window_extent()
    box_ratio = bbox.height / bbox.width
    if (y1 - y0) / (x1 - x0) < box_ratio:
        pad = ((x1 - x0) * box_ratio - (y1 - y0)) / 2
        y0, y1 = y0 - pad, y1 + pad
    else:
        pad = ((y1 - y0) / box_ratio - (x1 - x0)) / 2
        x0, x1 = x0 - pad, x1 + pad
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x [m] (odom)')
    ax.set_ylabel('y [m]')

    ghost = [ax.plot([], [], '-', color=WALL_COLOR, lw=0.6, alpha=0.18,
                     zorder=1)[0] for _ in range(2)]
    corr_fill = Polygon([[0, 0]], closed=True, color=FILL_COLOR, alpha=0.18,
                        lw=0, zorder=1, visible=False)
    ax.add_patch(corr_fill)
    wall_l, = ax.plot([], [], '-', color=WALL_COLOR, lw=1.4, zorder=2,
                      label='current corridor')
    wall_r, = ax.plot([], [], '-', color=WALL_COLOR, lw=1.4, zorder=2)
    centre, = ax.plot([], [], '--', color=CENTER_COLOR, lw=1.3, zorder=2,
                      label='centreline')
    horizon, = ax.plot([], [], ':', color=HORIZON_COLOR, lw=2.0, marker='.',
                       ms=3, zorder=4, label='MPC horizon')
    target, = ax.plot([], [], '*', color='gold', mec='black', ms=12, zorder=5,
                      label='MPC target')
    path, = ax.plot([], [], '-', color='black', lw=1.8, zorder=5,
                    label='driven')
    ax.plot([D.kin['x'][0]] if D.kin['x'] else [], [D.kin['y'][0]] if D.kin['y'] else [],
            'o', color='tab:green', mec='black', ms=8, zorder=6, label='start')
    car = Polygon(footprint(0, 0, 0), closed=True, fc='0.15', ec='black',
                  alpha=0.85, zorder=7)
    ax.add_patch(car)
    obst_patches = []
    track_marks, = ax.plot([], [], 'o', mfc='none', mec=TRACK_COLOR, mew=1.5,
                           ms=9, zorder=3, label='perception tracks (map)')
    track_labels = []
    ax.plot([], [], 'o', color=OBST_COLOR, alpha=0.35, ms=10,
            label='MPC obstacles')
    ax.legend(loc='upper left', fontsize=8, framealpha=0.85, ncol=2)
    status = ax.text(0.99, 0.02, '', transform=ax.transAxes, ha='right',
                     va='bottom', fontsize=10, family='monospace',
                     bbox=dict(fc='white', ec='0.6', alpha=0.9))
    banner = fig.text(0.62, 0.955, '', fontsize=14, weight='bold',
                      ha='left', va='center')

    # --- time plots (whole series, cursor moves) ---
    kt = D.kin['t']
    ax_v.plot(kt, D.kin['speed'], color='black', lw=1.2, label='measured')
    ax_v.step(D.cmd['t'], D.cmd['cmd_speed'], where='post', color='tab:blue',
              lw=1.0, label='cmd')
    ax_v.set_ylabel('speed [m/s]')
    ax_v.legend(loc='upper right', fontsize=8)
    ax_s.step(D.cmd['t'], D.cmd['cmd_steer'], where='post', color='tab:blue',
              lw=1.0)
    ax_s.set_ylabel('cmd steer [rad]')
    ax_c.plot(kt, D.kin['corridor_clearance'], color='tab:cyan', lw=1.2,
              label='corridor')
    ax_c.plot(kt, D.kin['obstacle_clearance'], color=OBST_COLOR, lw=1.2,
              label='obstacle')
    ax_c.axhline(0, color='0.4', lw=0.8)
    ax_c.set_ylabel('clearance [m]')
    ax_c.set_xlabel('test time t [s]')
    ax_c.legend(loc='upper right', fontsize=8)
    cursors = []
    for a in (ax_v, ax_s, ax_c):
        a.grid(True, alpha=0.3)
        a.set_xlim(t0, t1)
        for e in D.events:
            if e.get('event') in ALARM_EVENTS:
                a.axvline(e['t'], color='tab:red', lw=0.8, alpha=0.6)
            elif e.get('event') in ('mission_started', 'mission_finished'):
                a.axvline(e['t'], color='tab:green', lw=0.8, alpha=0.6, ls='--')
        cursors.append(a.axvline(t0, color='black', lw=1.0))
    # Clearances can spike to many metres when nothing is in range; keep the
    # interesting band readable.
    cvals = [v for v in D.kin['corridor_clearance'] + D.kin['obstacle_clearance']
             if v is not None]
    if cvals:
        ax_c.set_ylim(min(-0.1, min(cvals) - 0.05), min(max(cvals) + 0.1, 3.0))

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f'{D.test_id}.mp4'
    writer = FFMpegWriter(fps=fps, codec='libx264',
                          extra_args=['-pix_fmt', 'yuv420p', '-crf', '23',
                                      '-preset', 'medium'])
    mpc_t = D.mpc['t']
    with writer.saving(fig, str(out), dpi):
        for k in range(n_frames):
            t = t0 + k / fps
            i = last_before(kt, t)
            if i is not None:
                x, y, yaw = D.kin['x'][i], D.kin['y'][i], D.kin['yaw'][i]
                path.set_data(D.kin['x'][:i + 1], D.kin['y'][:i + 1])
                car.set_xy(footprint(x, y, yaw))
                spd = D.kin['speed'][i]
                cc = D.kin['corridor_clearance'][i]
                oc = D.kin['obstacle_clearance'][i]
            else:
                x = y = yaw = spd = cc = oc = None

            ci = last_before(D.corr_t, t)
            if ci is not None:
                _, fill, wl, wr, cl = D.corridors[ci]
                corr_fill.set_xy(fill)
                corr_fill.set_visible(True)
                wall_l.set_data(*wl)
                wall_r.set_data(*wr)
                centre.set_data(*(cl if cl else ([], [])))
                if ci > 0:
                    _, _, gl, gr, _ = D.corridors[ci - 1]
                    ghost[0].set_data(*gl)
                    ghost[1].set_data(*gr)
            hi = last_before(D.hor_t, t)
            if hi is not None and t - D.hor_t[hi] < 1.0:
                h = D.horizons[hi]
                hx, hy = list(h.x), list(h.y)
                if x is not None:
                    hx.insert(0, x)
                    hy.insert(0, y)
                horizon.set_data(hx, hy)
            else:
                horizon.set_data([], [])

            for p in obst_patches:
                p.remove()
            obst_patches.clear()
            di = last_before(D.debug_t, t)
            if di is not None and t - D.debug_t[di] < 1.0:
                _, obs, tgt = D.debug[di]
                for o in obs:
                    if isinstance(o, dict) and 'x' in o:
                        p = Circle((o['x'], o['y']), o.get('r', 0.25),
                                   color=OBST_COLOR, alpha=0.35, zorder=3)
                        ax.add_patch(p)
                        obst_patches.append(p)
                if isinstance(tgt, dict) and 'x' in tgt:
                    target.set_data([tgt['x']], [tgt['y']])
            else:
                target.set_data([], [])

            for lbl in track_labels:
                lbl.remove()
            track_labels.clear()
            ti = last_before(D.tracks_t, t)
            if ti is not None and t - D.tracks_t[ti] < 1.0:
                trs = [tr for tr in D.tracks[ti][1]
                       if tr.get('x') is not None and tr.get('y') is not None]
                track_marks.set_data([tr['x'] for tr in trs],
                                     [tr['y'] for tr in trs])
                for tr in trs:
                    if x0 < tr['x'] < x1 and y0 < tr['y'] < y1:
                        track_labels.append(ax.text(
                            tr['x'] + 0.12, tr['y'] + 0.12,
                            tr.get('class', ''), fontsize=7,
                            color=TRACK_COLOR, zorder=3))
            else:
                track_marks.set_data([], [])

            mi = last_before(mpc_t, t)
            mstat = D.mpc['status'][mi] if mi is not None and t - mpc_t[mi] < 0.5 else '-'

            def fmt(v, unit, nd=2):
                return f'{v:6.{nd}f} {unit}' if v is not None else '     -'
            status.set_text(
                f't        {t:6.1f} s\n'
                f'speed  {fmt(spd, "m/s")}\n'
                f'corr cl {fmt(cc, "m")}\n'
                f'obst cl {fmt(oc, "m")}\n'
                f'MPC     {mstat}')

            recent = [e for e in D.events
                      if e.get('event') in BANNER_EVENTS and e['t'] <= t < e['t'] + BANNER_S]
            if recent:
                e = recent[-1]
                txt = e['event'].upper().replace('_', ' ')
                if e.get('reason'):
                    txt += f': {e["reason"]}'[:60]
                banner.set_text(txt)
                banner.set_color('tab:red' if e['event'] in ALARM_EVENTS else 'tab:green')
            else:
                started = D.event_time('mission_started')
                if started is not None and t < started:
                    banner.set_text(f'countdown  T-{started - t:4.1f} s')
                    banner.set_color('0.35')
                else:
                    banner.set_text('')

            for c in cursors:
                c.set_xdata([t, t])
            writer.grab_frame()
    plt.close(fig)
    return out, n_frames, t0, t1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('test_dirs', nargs='+')
    parser.add_argument('--out-dir', default=str(DEFAULT_OUT))
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--pre-roll', type=float, default=2.0,
                        help='seconds kept before mission_started (default 2)')
    parser.add_argument('--full', action='store_true',
                        help='keep the whole test, countdown included')
    parser.add_argument('--dpi', type=int, default=100)
    args = parser.parse_args(argv)
    rc = 0
    for d in args.test_dirs:
        try:
            out, n, t0, t1 = render(d, args.out_dir, args.fps, args.pre_roll,
                                    args.full, args.dpi)
            print(f'{out}  ({n} frames, t {t0:.1f}..{t1:.1f} s)')
        except Exception as exc:  # one bad folder must not stop the batch
            print(f'{d}: FAILED: {exc!r}', file=sys.stderr)
            rc = 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
