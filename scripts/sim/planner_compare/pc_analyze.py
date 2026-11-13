#!/usr/bin/env python3
"""Compare the planner-controller runs of one session: report.md, summary.json/csv, plots.

Usage: pc_analyze.py OUT_DIR [--target 3.0]

OUT_DIR holds one subdirectory per mode (corridor_mpc, corridor_rrt,
p2p_geometric) as written by run.sh: pc_record.py's files plus run.json.

Everything is put in the START FRAME: the car's pose when the mission went
RUNNING is the origin, x forward, y to the left. Ground truth
(/sim/ground_truth/odom) is moved there with its own pose at that instant;
planned paths and corridors (odom frame) with the EKF's odom pose at that
instant. The EKF-vs-truth gap at the end is reported, so that alignment is
checked rather than assumed.

Metrics are deliberately few: where the car stopped against the commanded
distance, how far it left the straight line, how smooth the steering was, how
far it strayed from its own reference path, and what the controller process
cost in CPU.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import sys

import numpy as np

MODES = ('corridor_mpc', 'corridor_rrt', 'p2p_geometric')
NODE = {'corridor_mpc': 'mpc_corr', 'corridor_rrt': 'corridor_rrt',
        'p2p_geometric': 'p2p_geometric'}
# Reference categorical palette slots 1-3 (validated: CVD dE >= 9), plus a line
# style per mode so identity never rests on color alone.
COLOR = {'corridor_mpc': '#2a78d6', 'corridor_rrt': '#eb6834', 'p2p_geometric': '#1baf7a'}
STYLE = {'corridor_mpc': '-', 'corridor_rrt': '--', 'p2p_geometric': '-.'}
TERMINAL = ('COMPLETE', 'ABORTED')


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def read_csv(path):
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    out = {}
    for k in rows[0]:
        try:
            out[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            out[k] = np.array([r[k] for r in rows])
    return out


def read_jsonl(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [json.loads(ln) for ln in f if ln.strip()]


def pose_at(stream, tb, t):
    """(x, y, yaw) of the last sample at or before t (the first one if none)."""
    i = max(int(np.searchsorted(stream[tb], t, side='right')) - 1, 0)
    return stream['x'][i], stream['y'][i], stream['yaw'][i]


def to_frame(origin, xy):
    """Points (Nx2) expressed in the frame of ``origin`` (x, y, yaw)."""
    x0, y0, yaw0 = origin
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    c, s = math.cos(yaw0), math.sin(yaw0)
    dx, dy = xy[:, 0] - x0, xy[:, 1] - y0
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def dist_to_polyline(p, line):
    a, b = line[:-1], line[1:]
    ab = b - a
    denom = np.maximum((ab * ab).sum(1), 1e-12)
    t = np.clip(((p - a) * ab).sum(1) / denom, 0, 1)
    foot = a + t[:, None] * ab
    return float(np.min(np.hypot(*(p - foot).T)))


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else None


def r(v, n=3):
    return None if v is None or (isinstance(v, float) and not math.isfinite(v)) else round(v, n)


def centreline(polygon):
    """Corridor polygon (left forward + right backward, equal halves) -> centreline."""
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    n = len(poly) // 2
    if n < 2:
        return None
    left, right = poly[:n], poly[n:2 * n][::-1]
    return (left + right) / 2.0


def analyze_mode(d, mode, target):
    meta = json.load(open(os.path.join(d, 'meta.json')))
    run = json.load(open(os.path.join(d, 'run.json'))) if os.path.exists(
        os.path.join(d, 'run.json')) else {}
    tb = 't_sim' if meta.get('sim_time', True) else 't_wall'
    gt, odom = read_csv(os.path.join(d, 'gt.csv')), read_csv(os.path.join(d, 'odom.csv'))
    drive, cpu = read_csv(os.path.join(d, 'drive.csv')), read_csv(os.path.join(d, 'cpu.csv'))
    solver = read_csv(os.path.join(d, 'solver.csv'))
    paths = read_jsonl(os.path.join(d, 'paths.jsonl'))
    pstat = read_jsonl(os.path.join(d, 'planner_status.jsonl'))
    notes = []

    trail = meta['mission_trail']
    run_i = next((i for i, s in enumerate(trail) if s['state'] == 'RUNNING'), None)
    if run_i is None or not gt:
        return {'mode': mode, 'error': 'no RUNNING state or no ground truth recorded',
                'final_state': meta.get('final_state')}, None
    t_run, w_run = trail[run_i][tb], trail[run_i]['t_wall']
    end = next((s for s in trail[run_i + 1:] if s['state'] in TERMINAL), None)
    if end is None:
        notes.append('mission never left RUNNING within the recording (timeout)')
        t_end, w_end = gt[tb][-1], gt['t_wall'][-1]
        state = meta.get('final_state')
    else:
        t_end, w_end, state = end[tb], end['t_wall'], end['state']

    o_gt = pose_at(gt, tb, t_run)
    o_odom = pose_at(odom, tb, t_run) if odom else None
    after = gt[tb] >= t_run
    win = after & (gt[tb] <= t_end)
    P = to_frame(o_gt, np.column_stack([gt['x'], gt['y']]))
    yaw_rel = wrap(gt['yaw'] - o_gt[2])
    along, lat = P[:, 0], P[:, 1]
    i_end = max(int(np.searchsorted(gt[tb], t_end, side='right')) - 1, 0)
    seg = np.hypot(np.diff(P[after, 0]), np.diff(P[after, 1]))
    duration = t_end - t_run

    m = {
        'mode': mode, 'node': NODE[mode], 'final_state': state,
        'duration_s': r(duration, 2),
        'along_at_complete_m': r(along[i_end]),
        'along_stopped_m': r(along[-1]),
        'overshoot_m': r(along[-1] - target),
        'lat_max_abs_m': r(float(np.max(np.abs(lat[after])))),
        'lat_rms_m': r(float(np.sqrt(np.mean(lat[win] ** 2)))) if win.any() else None,
        'lat_final_m': r(lat[-1]),
        'heading_final_deg': r(math.degrees(yaw_rel[-1]), 2),
        'heading_max_abs_deg': r(math.degrees(float(np.max(np.abs(yaw_rel[after])))), 2),
        'driven_length_m': r(float(seg.sum())),
        'v_mean_mps': r(along[i_end] / duration) if duration > 0 else None,
        'v_max_mps': r(float(np.max(gt['v'][after]))),
    }
    if odom and o_odom is not None:
        Po = to_frame(o_odom, np.column_stack([odom['x'], odom['y']]))
        m['ekf_vs_gt_final_m'] = r(float(np.hypot(*(Po[-1] - P[-1]))))
    # steering and speed commands (/drive) in the mission window
    if drive:
        dw = (drive[tb] >= t_run) & (drive[tb] <= t_end)
        st = drive['steer'][dw]
        if len(st) > 1:
            m['steer_mean_abs_deg'] = r(math.degrees(float(np.mean(np.abs(st)))), 2)
            m['steer_max_abs_deg'] = r(math.degrees(float(np.max(np.abs(st)))), 2)
            m['steer_total_variation_deg'] = r(math.degrees(float(np.sum(np.abs(np.diff(st))))), 1)
            m['drive_msgs'] = int(dw.sum())
            m['drive_rate_hz'] = r(dw.sum() / duration, 1) if duration > 0 else None
    # reference path the controller tracked: planned path (new modes) or corridor centreline (MPC)
    refs = []
    for p in paths:
        if p['src'] == 'planner_path' and len(p['xy']) >= 2:
            refs.append((p[tb], np.asarray(p['xy'], dtype=float)))
        elif p['src'] == 'corridor' and mode == 'corridor_mpc':
            c = centreline(p['xy'])
            if c is not None:
                refs.append((p[tb], c))
    refs.sort(key=lambda e: e[0])
    m['reference'] = 'planned path' if mode != 'corridor_mpc' else 'corridor centreline'
    m['n_references'] = len([e for e in refs if t_run <= e[0] <= t_end])
    if refs and odom and o_odom is not None:
        ref_t = np.array([e[0] for e in refs])
        ref_xy = [to_frame(o_odom, e[1]) for e in refs]
        errs = []
        for k in np.nonzero(win)[0][::5]:
            j = int(np.searchsorted(ref_t, gt[tb][k], side='right')) - 1
            if j >= 0:
                errs.append(dist_to_polyline(P[k], ref_xy[j]))
        if errs:
            m['ref_track_err_mean_m'] = r(float(np.mean(errs)))
            m['ref_track_err_max_m'] = r(float(np.max(errs)))
        lat_ref = [float(np.max(np.abs(xy[:, 1]))) for t, xy in zip(ref_t, ref_xy)
                   if t_run <= t <= t_end]
        if lat_ref:
            m['ref_lat_max_abs_m'] = r(max(lat_ref))
    else:
        notes.append('no reference path recorded')
    # computational cost: the controller PROCESS (/proc), mission window and idle before it
    if cpu:
        tw = cpu['t_wall']

        def cpu_between(a, b):
            k = (tw >= a) & (tw <= b)
            if k.sum() < 2:
                return None, None, None
            dc = cpu['cpu_s'][k][-1] - cpu['cpu_s'][k][0]
            dw_ = tw[k][-1] - tw[k][0]
            ds = cpu['t_sim'][k][-1] - cpu['t_sim'][k][0]
            return dc, dw_, ds
        dc, dw_, ds = cpu_between(w_run, w_end)
        if dc is not None and dw_ > 0:
            m['cpu_core_pct'] = r(100 * dc / dw_, 1)
            m['cpu_ms_per_sim_s'] = r(1000 * dc / ds, 1) if ds > 0 else None
            m['rtf'] = r(ds / dw_, 3) if ds > 0 else None
            if m.get('drive_msgs'):
                m['cpu_ms_per_tick'] = r(1000 * dc / m['drive_msgs'], 2)
        k = (tw >= w_run) & (tw <= w_end)
        if k.sum() > 6:
            inst = np.diff(cpu['cpu_s'][k]) / np.maximum(np.diff(tw[k]), 1e-6) * 100
            win5 = np.convolve(inst, np.ones(5) / 5, mode='valid')   # ~1 s at 5 Hz
            m['cpu_core_pct_peak_1s'] = r(float(np.max(win5)), 1)
        di, dwi, _ = cpu_between(tw[0], w_run - 0.5)
        if di is not None and dwi > 1.0:
            m['cpu_core_pct_idle'] = r(100 * di / dwi, 1)
        m['rss_mb_max'] = r(float(np.max(cpu['rss_mb'])), 1)
        m['host_busy_pct_mean'] = r(float(np.mean(cpu['sys_busy_pct'][k])), 1) if k.any() else None
    # what each node reports about its own planning / solving
    pw = [s for s in pstat if t_run <= s.get(tb, -1) <= t_end and s.get('ok')]
    if pw:
        pm = np.array([s['plan_ms'] for s in pw])
        m['plans'] = len(pw)
        m['plan_ms_mean'], m['plan_ms_p95'], m['plan_ms_max'] = (
            r(float(pm.mean()), 2), r(pct(pm, 95), 2), r(float(pm.max()), 2))
        m['plan_failures'] = len([s for s in pstat if t_run <= s.get(tb, -1) <= t_end
                                  and not s.get('ok')])
    if solver:
        sw = (solver[tb] >= t_run) & (solver[tb] <= t_end)
        if sw.any():
            sd = solver['solve_dt_sec'][sw] * 1000
            m['solves'] = int(sw.sum())
            m['solve_success_pct'] = r(100 * float(np.mean(solver['success'][sw])), 1)
            m['solve_ms_mean'], m['solve_ms_p95'] = r(float(sd.mean()), 2), r(pct(sd, 95), 2)
            if float(np.max(sd)) == 0.0:
                notes.append('mpc_corr solve_dt_sec is 0 throughout (ROS clock in sim time): '
                             'use the process CPU columns for its cost')
    if state != 'COMPLETE':
        notes.append(f'mission ended {state}')
    if m.get('rtf') is not None and m['rtf'] < 0.9:
        notes.append(f'sim ran at RTF {m["rtf"]}: CPU % of a core is per WALL second; '
                     'cpu_ms_per_sim_s is the RTF-independent figure')
    m['llm_plan_s'] = run.get('llm_plan_seconds')
    m['mission_sha'] = run.get('mission_sha')
    m['notes'] = notes

    plot = {'tb': tb, 't_run': t_run, 't_end': t_end, 'gt_t': gt[tb] - t_run, 'P': P,
            'v': gt['v'], 'yaw': yaw_rel, 'refs': [], 'corridors': [], 'preds': [], 'tree': None}
    if drive:
        plot['drive_t'], plot['steer'], plot['speed_cmd'] = (
            drive[tb] - t_run, drive['steer'], drive['speed'])
    if odom and o_odom is not None:
        plot['odom'] = to_frame(o_odom, np.column_stack([odom['x'], odom['y']]))
        plot['odom_t'] = odom[tb] - t_run
        for p in paths:
            if not (t_run - 1.0 <= p[tb] <= t_end):
                continue
            xy = np.asarray(p['xy'], dtype=float).reshape(-1, 2)
            if len(xy) < 2:
                continue
            if p['src'] == 'planner_path':
                plot['refs'].append(to_frame(o_odom, xy))
            elif p['src'] == 'corridor':
                plot['corridors'].append(to_frame(o_odom, xy))
            elif p['src'] == 'mpc_pred':
                plot['preds'].append(to_frame(o_odom, xy))
            elif p['src'] == 'rrt_tree' and plot['tree'] is None:
                plot['tree'] = to_frame(o_odom, xy)
    if cpu:
        k = cpu['t_wall'] >= w_run - 3.0
        tw = cpu['t_wall'][k]
        inst = np.diff(cpu['cpu_s'][k]) / np.maximum(np.diff(tw), 1e-6) * 100
        plot['cpu_t'] = tw[1:] - w_run
        plot['cpu_pct'] = np.convolve(inst, np.ones(5) / 5, mode='same')
        plot['cpu_end'] = w_end - w_run
    return m, plot


def make_plots(out, plots, target):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'figure.facecolor': '#fcfcfb', 'axes.facecolor': '#fcfcfb',
                         'axes.edgecolor': '#52514e', 'axes.labelcolor': '#0b0b0b',
                         'text.color': '#0b0b0b', 'xtick.color': '#52514e',
                         'ytick.color': '#52514e', 'axes.grid': True, 'grid.color': '#e4e3df',
                         'grid.linewidth': 0.6, 'axes.spines.top': False,
                         'axes.spines.right': False, 'lines.linewidth': 2.0, 'font.size': 9})
    files = []
    modes = [m for m in MODES if m in plots]

    # 1. ground-truth trajectories, start frame
    fig, axes = plt.subplots(2, 1, figsize=(9, 6.5), gridspec_kw={'height_ratios': [1.4, 1]})
    for mode in modes:
        p = plots[mode]
        k = p['gt_t'] >= 0
        for ax in axes:
            ax.plot(p['P'][k, 0], p['P'][k, 1], STYLE[mode], color=COLOR[mode], label=mode)
            ax.plot(p['P'][k, 0][-1], p['P'][k, 1][-1], 'o', color=COLOR[mode], ms=6)
    for ax in axes:
        ax.axvline(target, color='#52514e', lw=1, ls=':')
        ax.axhline(0, color='#52514e', lw=0.8)
        ax.set_xlabel('along the start heading [m]')
    axes[0].set_aspect('equal')
    axes[0].set_ylim(-1.0, 1.0)
    axes[0].set_ylabel('lateral [m]')
    axes[0].set_title(f'Ground-truth trajectories (start frame; dotted line = {target:g} m '
                      'command, dot = where the car stopped)', loc='left')
    axes[0].legend(loc='upper left', frameon=False)
    axes[1].set_ylabel('lateral [m] (zoomed)')
    axes[1].set_title('Same, lateral axis stretched', loc='left')
    fig.tight_layout()
    f = os.path.join(out, 'trajectories.png')
    fig.savefig(f, dpi=130)
    plt.close(fig)
    files.append(f)

    # 2. reference path vs trajectory, one panel per mode
    fig, axes = plt.subplots(len(modes), 1, figsize=(9, 2.6 * len(modes)), sharex=True,
                             squeeze=False)
    for ax, mode in zip(axes[:, 0], modes):
        p = plots[mode]
        for i, c in enumerate(p['corridors']):
            if i == 0:
                ax.fill(c[:, 0], c[:, 1], color='#86b6ef', alpha=0.25, lw=0,
                        label=f'corridor (first filled, {len(p["corridors"])} built)')
            else:
                ax.plot(np.append(c[:, 0], c[0, 0]), np.append(c[:, 1], c[0, 1]),
                        color='#86b6ef', lw=0.6, alpha=0.5)
        if p['tree'] is not None and len(p['tree']) >= 2:
            seg = p['tree'].reshape(-1, 2, 2)
            for s in seg:
                ax.plot(s[:, 0], s[:, 1], color='#c3c2b7', lw=0.5)
        for i, xy in enumerate(p['preds'][::5]):
            ax.plot(xy[:, 0], xy[:, 1], color=COLOR[mode], lw=0.8, alpha=0.35,
                    label='MPC predicted horizon (every 5th)' if i == 0 else None)
        for i, xy in enumerate(p['refs']):
            ax.plot(xy[:, 0], xy[:, 1], color='#52514e', lw=0.8, alpha=0.5,
                    label=f'planned path ({len(p["refs"])} plans)' if i == 0 else None)
        k = p['gt_t'] >= 0
        ax.plot(p['P'][k, 0], p['P'][k, 1], STYLE[mode], color=COLOR[mode],
                label='trajectory (ground truth)')
        if 'odom' in p:
            ko = p['odom_t'] >= 0
            ax.plot(p['odom'][ko, 0], p['odom'][ko, 1], ':', color='#0b0b0b', lw=1.2,
                    label='trajectory (EKF odom)')
        ax.axvline(target, color='#52514e', lw=1, ls=':')
        ax.set_ylim(-1.0, 1.0)
        ax.set_ylabel('lateral [m]')
        ax.set_title(mode, loc='left')
        ax.legend(loc='upper left', frameon=False, fontsize=7, ncol=2)
    axes[-1, 0].set_xlabel('along the start heading [m]')
    fig.tight_layout()
    f = os.path.join(out, 'paths.png')
    fig.savefig(f, dpi=130)
    plt.close(fig)
    files.append(f)

    # 3. time series: speed, steering command, lateral offset
    fig, axes = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True)
    for mode in modes:
        p = plots[mode]
        k = p['gt_t'] >= -1.0
        axes[0].plot(p['gt_t'][k], p['v'][k], STYLE[mode], color=COLOR[mode], label=mode)
        if 'drive_t' in p:
            kd = p['drive_t'] >= -1.0
            axes[1].plot(p['drive_t'][kd], np.degrees(p['steer'][kd]), STYLE[mode],
                         color=COLOR[mode], label=mode)
        axes[2].plot(p['gt_t'][k], p['P'][k, 1] * 100, STYLE[mode], color=COLOR[mode],
                     label=mode)
        for ax in axes:
            ax.axvline(p['t_end'] - p['t_run'], color=COLOR[mode], lw=0.8, ls=':')
    axes[0].set_ylabel('speed (GT) [m/s]')
    axes[1].set_ylabel('steering cmd [deg]')
    axes[2].set_ylabel('lateral offset [cm]')
    axes[2].set_xlabel('time since the mission went RUNNING [s] (dotted: mission end)')
    axes[0].legend(loc='upper right', frameon=False)
    fig.tight_layout()
    f = os.path.join(out, 'timeseries.png')
    fig.savefig(f, dpi=130)
    plt.close(fig)
    files.append(f)

    # 4. CPU of the controller process
    if any('cpu_t' in plots[m] for m in modes):
        fig, ax = plt.subplots(1, 1, figsize=(9, 3.4))
        for mode in modes:
            p = plots[mode]
            if 'cpu_t' in p:
                ax.plot(p['cpu_t'], p['cpu_pct'], STYLE[mode], color=COLOR[mode],
                        label=f'{mode} ({NODE[mode]})')
        ax.set_ylabel('controller CPU [% of one core]')
        ax.set_xlabel('wall time since the mission went RUNNING [s] (~1 s moving average)')
        ax.set_ylim(bottom=0)
        ax.legend(loc='upper right', frameon=False)
        fig.tight_layout()
        f = os.path.join(out, 'cpu.png')
        fig.savefig(f, dpi=130)
        plt.close(fig)
        files.append(f)
    return files


ROWS = [
    ('final_state', 'mission result', ''),
    ('duration_s', 'mission duration', 's'),
    ('along_at_complete_m', 'distance at mission COMPLETE', 'm'),
    ('along_stopped_m', 'distance where the car stopped', 'm'),
    ('overshoot_m', 'stopped minus commanded', 'm'),
    ('lat_max_abs_m', 'max lateral offset', 'm'),
    ('lat_rms_m', 'RMS lateral offset (mission)', 'm'),
    ('heading_final_deg', 'final heading error', 'deg'),
    ('driven_length_m', 'driven length', 'm'),
    ('v_mean_mps', 'mean speed (mission)', 'm/s'),
    ('v_max_mps', 'max speed', 'm/s'),
    ('steer_mean_abs_deg', 'mean |steering cmd|', 'deg'),
    ('steer_max_abs_deg', 'max |steering cmd|', 'deg'),
    ('steer_total_variation_deg', 'steering total variation', 'deg'),
    ('reference', 'reference tracked', ''),
    ('n_references', 'references published', ''),
    ('ref_lat_max_abs_m', 'max lateral of the reference', 'm'),
    ('ref_track_err_mean_m', 'distance to reference, mean', 'm'),
    ('ref_track_err_max_m', 'distance to reference, max', 'm'),
    ('ekf_vs_gt_final_m', 'EKF vs ground truth at the end', 'm'),
    ('cpu_core_pct', 'controller CPU, mission', '% core'),
    ('cpu_core_pct_peak_1s', 'controller CPU, peak 1 s', '% core'),
    ('cpu_core_pct_idle', 'controller CPU, idle before', '% core'),
    ('cpu_ms_per_sim_s', 'controller CPU per sim second', 'ms'),
    ('cpu_ms_per_tick', 'controller CPU per /drive msg', 'ms'),
    ('drive_rate_hz', '/drive rate', 'Hz'),
    ('plans', 'replans (new modes)', ''),
    ('plan_ms_mean', 'plan time mean (wall)', 'ms'),
    ('plan_ms_p95', 'plan time p95', 'ms'),
    ('plan_ms_max', 'plan time max', 'ms'),
    ('solves', 'MPC solves', ''),
    ('solve_success_pct', 'MPC solve success', '%'),
    ('solve_ms_mean', 'MPC solve_dt mean (ROS clock)', 'ms'),
    ('rss_mb_max', 'controller RSS', 'MB'),
    ('rtf', 'sim real-time factor', ''),
    ('host_busy_pct_mean', 'host CPU busy (all cores)', '%'),
    ('llm_plan_s', 'LLM translation time', 's'),
]


def write_report(out, results, files, target, session):
    modes = [m['mode'] for m in results]
    lines = [f'# Planner-controller comparison: "{session.get("command", "?")}"', '']
    lines.append(f'Session `{os.path.basename(os.path.abspath(out))}`, host '
                 f'`{session.get("host", "?")}`, scene `{session.get("scene", "?")}` layout '
                 f'`{session.get("layout", "?")}`, branch `{session.get("git_branch", "?")}` '
                 f'@ `{session.get("git_commit", "?")}`. One run per mode, the same command '
                 'sent through the LLM planner each time.')
    lines.append('')
    shas = {m.get('mission_sha') for m in results}
    if len(shas) == 1 and None not in shas:
        lines.append('The LLM produced the **same mission** for every mode '
                     f'(moves sha `{shas.pop()[:12]}`):')
    else:
        lines.append('**The LLM missions differ between modes** (or one is missing): '
                     'compare them before comparing the runs. Per-mode `mission.json`.')
    mj = next((os.path.join(out, m, 'mission.json') for m in modes
               if os.path.exists(os.path.join(out, m, 'mission.json'))), None)
    if mj:
        lines += ['', '```json', json.dumps(json.load(open(mj)).get('moves'), indent=1), '```']
    lines += ['', '## Results', '', '| metric | unit | ' + ' | '.join(modes) + ' |',
              '|---|---|' + '---|' * len(modes)]
    for key, label, unit in ROWS:
        vals = [m.get(key) for m in results]
        if all(v is None for v in vals):
            continue
        lines.append(f'| {label} | {unit} | ' + ' | '.join(
            '-' if v is None else str(v) for v in vals) + ' |')
    lines += ['', 'Frame: the car\'s pose when the mission went RUNNING; x forward, y left. '
              f'"stopped minus commanded" is ground truth against {target:g} m. CPU is the '
              'controller process from /proc (utime+stime, all threads) over the mission\'s '
              'wall-clock window; "per sim second" removes the sim\'s real-time factor.', '']
    for f in files:
        lines.append(f'![{os.path.basename(f)}]({os.path.basename(f)})')
        lines.append('')
    notes = [(m['mode'], n) for m in results for n in m.get('notes', [])]
    notes += [(m, 'no run recorded (see logs/)') for m in session.get('modes', [])
              if m not in modes]
    notes += [(m['mode'], m['error']) for m in results if m.get('error')]
    if notes:
        lines += ['## Notes', '']
        lines += [f'- **{mode}**: {n}' for mode, n in notes]
        lines.append('')
    with open(os.path.join(out, 'report.md'), 'w') as fh:
        fh.write('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('out')
    ap.add_argument('--target', type=float, default=None,
                    help='commanded distance [m] (default: session.json, else 3.0)')
    args = ap.parse_args()
    session = {}
    sp = os.path.join(args.out, 'session.json')
    if os.path.exists(sp):
        session = json.load(open(sp))
    target = args.target if args.target is not None else float(session.get('target_m', 3.0))
    results, plots = [], {}
    for mode in MODES:
        d = os.path.join(args.out, mode)
        if not os.path.exists(os.path.join(d, 'meta.json')):
            continue
        m, p = analyze_mode(d, mode, target)
        results.append(m)
        if p is not None:
            plots[mode] = p
    if not results:
        print(f'no runs under {args.out}', file=sys.stderr)
        return 1
    files = make_plots(args.out, plots, target) if plots else []
    with open(os.path.join(args.out, 'summary.json'), 'w') as f:
        json.dump(results, f, indent=2)
    keys = [k for k, _, _ in ROWS]
    with open(os.path.join(args.out, 'summary.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['mode'] + keys)
        for m in results:
            w.writerow([m['mode']] + [m.get(k) for k in keys])
    write_report(args.out, results, files, target, session)
    print(f'report: {os.path.join(args.out, "report.md")}')
    for m in results:
        print(json.dumps({k: m.get(k) for k in ('mode', 'final_state', 'along_stopped_m',
                                                'lat_max_abs_m', 'cpu_core_pct',
                                                'cpu_ms_per_sim_s', 'notes')}))
    return 0


def mission_sha(path):
    """Hash of a mission's moves (mission_id differs per LLM call, the moves should not)."""
    moves = json.load(open(path)).get('moves')
    return hashlib.sha256(json.dumps(moves, sort_keys=True).encode()).hexdigest()


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--mission-sha':
        print(mission_sha(sys.argv[2]))
        sys.exit(0)
    sys.exit(main())
