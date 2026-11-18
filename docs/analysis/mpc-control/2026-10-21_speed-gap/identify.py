#!/usr/bin/env python3
"""Speed-loop system identification from archived F1TENTH car runs.

Written for output/speed_gap/offset_free.md section 1 (2026-10-21); its
results are identification.json next to it. Re-run, read-only on every run:

    python3 -I output/speed_gap/identify.py --out /tmp/ident --jobs 8 \
        --primary 2026-10-21T12-25-22 --primary corridor10m_20261019 --primary P002-R028 \
        ~/f1tenth_runs/* ~/bags/corridor10m_20261019T131815 ~/f1tenth_archive/mission_row/*

    python3 -I identify.py [--out DIR] [--boot N] [--primary NAME] RUN [RUN ...]

RUN is any of
  * a mission archive dir   (bag/*.db3 + *.manifest.json, e.g. ~/f1tenth_archive/mission_row/<run>)
  * a bare rosbag2 dir      (*.db3 + metadata.yaml, e.g. ~/bags/corridor10m_20261019T131815)
  * a campaign-recorder dir (commands.csv + kinematics.csv + meta.json)

Plant:  u = speed command [m/s] that reaches ackermann_to_vesc (/ackermann_drive, or
        /drive when the mux output is missing/incomplete in the bag), zero-order hold
        between consecutive command messages (~10 Hz);
        y = /odometry/filtered twist.linear.x (the MPC's measured speed) and,
        separately, /odom twist.linear.x (measured ERPM / speed_to_erpm_gain).
Model:  drive regime (u >= U_MIN):
            y[k+1] = a y[k] + (1-a) K u[k-d],   a = exp(-dt_k/tau)  (exact ZOH)
                                                 a = 1 - ts/tau      (Euler, reported too)
        optional offset form   y_ss = K1 (u - u0)
        release regime (u < U_MIN, incl. the stop): VESC output is zero (duty 0), the car
        coasts: y[k+1] = max(0, y[k] - a_c dt_k).
Reads bags read-only (AnyReader; falls back to sqlite ?mode=ro). Writes only into --out.
"""
import argparse
import csv
import datetime as dtm
import json
import math
import site
import sqlite3
import sys
from pathlib import Path

sys.path.append(site.getusersitepackages())  # rosbags lives in ~/.local; -I drops user site

import numpy as np  # noqa: E402
from scipy import optimize  # noqa: E402

TS = 0.1                      # MPC control period [s]
GAIN_NOW = 4516.9             # speed_to_erpm_gain today (vesc.yaml since the 2026-09-21 change)
GAIN_OLD = 5499.271647286143  # 2026-08-10 .. 2026-09-21 ~12:50 UTC
GAIN_SWITCH_UTC = dtm.datetime(2026, 9, 21, 12, 50, tzinfo=dtm.timezone.utc)
ERPM_TELEM_DB = 500.0         # vesc_driver erpm_deadband: |ERPM| < 500 published as exactly 0
ERPM_MIN_DRIVE = 500.0        # setpoints below ~450-540 ERPM produce no motor current (10-19 bag)
ZERO_EPS = 0.02               # |v| below this is a deadband zero (true speeds < 0.111 are never published)
HALF_WIN = 0.05               # centred window half-width for y at a tick
MAX_GAP = 0.25                # a command gap longer than this breaks the ZOH sequence
DMAX = 4
A_COAST = 1.5                 # coast-down decel [m/s^2] in the release regime; set from the stop events (--a-coast)
EXCLUDE_HOLD = False          # --exclude-hold: drop the samples the MPC used from the y windows
NOMINAL = (0.16, 1.04, 0.086)  # (tau, K1, u0) used for the 'nominal model' residuals (--nominal)

ACK = "float32 steering_angle\nfloat32 steering_angle_velocity\nfloat32 speed\nfloat32 acceleration\nfloat32 jerk\n"
ACKS = "std_msgs/Header header\nackermann_msgs/AckermannDrive drive\n"
VESC = ("float64 temp_fet\nfloat64 temp_motor\nfloat64 current_motor\nfloat64 current_input\n"
        "float64 avg_id\nfloat64 avg_iq\nfloat64 duty_cycle\nfloat64 speed\nfloat64 voltage_input\n"
        "float64 charge_drawn\nfloat64 charge_regen\nfloat64 energy_drawn\nfloat64 energy_regen\n"
        "int32 displacement\nint32 distance_traveled\nint32 fault_code\nfloat64 pid_pos_now\n"
        "int32 controller_id\nfloat64 ntc_temp_mos1\nfloat64 ntc_temp_mos2\nfloat64 ntc_temp_mos3\n"
        "float64 avg_vd\nfloat64 avg_vq\n")
VESCS = "std_msgs/Header header\nvesc_msgs/VescState state\n"

TOPICS = {
    '/drive': lambda m: (m.drive.speed,),
    '/ackermann_drive': lambda m: (m.drive.speed,),
    '/odom': lambda m: (m.twist.twist.linear.x,),
    '/odometry/filtered': lambda m: (m.twist.twist.linear.x,),
    '/sensors/core': lambda m: (m.state.speed, m.state.voltage_input, m.state.current_motor,
                                m.state.duty_cycle),
    '/slam/pose': lambda m: (m.pose.pose.position.x, m.pose.pose.position.y),
}


# ----------------------------------------------------------------------------- IO
def _typestore():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    for txt, name in ((ACK, 'ackermann_msgs/msg/AckermannDrive'),
                      (ACKS, 'ackermann_msgs/msg/AckermannDriveStamped'),
                      (VESC, 'vesc_msgs/msg/VescState'),
                      (VESCS, 'vesc_msgs/msg/VescStateStamped')):
        add.update(get_types_from_msg(txt, name))
    ts.register(add)
    return ts


def _row(topic, m, t_ns):
    h = getattr(m, 'header', None)
    st = h.stamp.sec + h.stamp.nanosec * 1e-9 if h is not None else np.nan
    return (t_ns * 1e-9, st) + tuple(TOPICS[topic](m))


def read_bag(bagdir):
    """{topic: array[n, 2+k]} with columns (receive_time, header_stamp, values...)."""
    bagdir = Path(bagdir)
    rows = {}
    try:
        from rosbags.highlevel import AnyReader
        with AnyReader([bagdir], default_typestore=_typestore()) as r:
            conns = [c for c in r.connections if c.topic in TOPICS]
            for c, t_ns, raw in r.messages(connections=conns):
                rows.setdefault(c.topic, []).append(_row(c.topic, r.deserialize(raw, c.msgtype), t_ns))
        how = 'AnyReader'
    except Exception as e:  # e.g. a metadata.yaml whose QoS block does not parse
        rows = {}
        ts = _typestore()
        for db in sorted(bagdir.glob('*.db3')):
            con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
            for tid, name, typ in con.execute('select id, name, type from topics'):
                if name not in TOPICS:
                    continue
                q = 'select timestamp, data from messages where topic_id=? order by timestamp'
                for t_ns, raw in con.execute(q, (tid,)):
                    rows.setdefault(name, []).append(_row(name, ts.deserialize_cdr(bytes(raw), typ), t_ns))
            con.close()
        how = 'sqlite ro (AnyReader: %s)' % str(e).splitlines()[0][:60]
    out = {k: np.array(v, dtype=float) for k, v in rows.items() if v}
    for k in out:
        out[k] = out[k][np.argsort(out[k][:, 1] if np.isfinite(out[k][:, 1]).all() else out[k][:, 0])]
    return out, how


def gain_at(start_utc):
    if start_utc is None:
        return GAIN_NOW, 'unknown date: assumed current gain'
    if start_utc < GAIN_SWITCH_UTC:
        return GAIN_OLD, 'pre-2026-09-21T12:50Z: 5499.27 (09-21 12:35-12:45 runs may be 5288.68)'
    return GAIN_NOW, 'post-2026-09-21T12:50Z: 4516.9'


def load_run(path):
    p = Path(path).expanduser()
    run = dict(name=p.name, path=str(p), notes=[])
    if (p / 'commands.csv').exists() and (p / 'kinematics.csv').exists():
        return _load_csv_run(p, run)
    bag = p / 'bag' if (p / 'bag').is_dir() else p
    man = sorted(p.glob('*.manifest.json'))
    meta = json.loads(man[0].read_text()) if man else {}
    data, how = read_bag(bag)
    run['kind'] = 'bag'
    run['reader'] = how
    run['commit'] = meta.get('git', {}).get('commit')
    run['dirty'] = meta.get('git', {}).get('dirty')
    t0 = meta.get('record_start_time') or meta.get('start_time')
    if t0:
        run['start_utc'] = dtm.datetime.fromisoformat(t0)
    else:
        ref = data.get('/drive', data.get('/odom'))
        run['start_utc'] = dtm.datetime.fromtimestamp(ref[0, 0], dtm.timezone.utc) if ref is not None else None
    run['mission_window'] = None
    if meta.get('mission_start_time') and meta.get('end_time'):
        run['mission_window'] = (dtm.datetime.fromisoformat(meta['mission_start_time']).timestamp(),
                                 dtm.datetime.fromisoformat(meta['end_time']).timestamp())
    run['gain'], run['gain_note'] = gain_at(run['start_utc'])
    run['raw'] = data
    run['y'] = {}
    if '/odom' in data:
        run['y']['odom'] = data['/odom'][:, [1, 2]]
    if '/slam/pose' in data:
        run['slam'] = data['/slam/pose'][:, [1, 2, 3]]
    sr = slam_ratio(run)
    run['slam_odom_ratio'] = sr
    if sr is not None:
        cands = {GAIN_NOW: 'SLAM/odom ratio %.3f -> 4516.9' % sr, 5288.68: 'SLAM/odom ratio %.3f -> 5288.68' % sr,
                 GAIN_OLD: 'SLAM/odom ratio %.3f -> 5499.27' % sr}
        best = min(cands, key=lambda gg: abs(math.log(GAIN_NOW * sr / gg)))
        if abs(math.log(GAIN_NOW * sr / best)) < 0.05:
            run['gain'], run['gain_note'] = best, cands[best]
    # stamp vs receive-time agreement
    run['timebase'] = {}
    for k, a in data.items():
        if np.isfinite(a[:, 1]).all():
            lag = a[:, 0] - a[:, 1]
            run['timebase'][k] = dict(n=len(a), lag_med_ms=float(np.median(lag) * 1e3),
                                      lag_p95_ms=float(np.percentile(lag, 95) * 1e3))
    # command: the mux output if it is complete, else /drive
    dr, ak = data.get('/drive'), data.get('/ackermann_drive')
    if ak is not None and dr is not None and len(ak) >= 0.95 * len(dr):
        u, run['u_topic'] = ak, '/ackermann_drive'
    elif dr is not None:
        u, run['u_topic'] = dr, '/drive'
    else:
        u, run['u_topic'] = ak, '/ackermann_drive'
    if dr is not None and ak is not None and len(ak) and len(dr):
        j = np.clip(np.searchsorted(ak[:, 1], dr[:, 1]), 0, len(ak) - 1)
        ok = np.abs(ak[j, 1] - dr[:, 1]) < 0.02
        run['drive_vs_mux_maxdiff'] = float(np.max(np.abs(ak[j[ok], 2] - dr[ok, 2]))) if ok.any() else None
    run['u'] = None if u is None else u[:, [1, 2]]
    run['y'] = {}
    for top, key in (('/odometry/filtered', 'filt'), ('/odom', 'odom')):
        if top in data:
            run['y'][key] = data[top][:, [1, 2]]
    if '/sensors/core' in data:
        c = data['/sensors/core']
        run['y']['core_erpm'] = c[:, [1, 2]]
        run['core'] = c
    if '/slam/pose' in data:
        run['slam'] = data['/slam/pose'][:, [1, 2, 3]]
    return run


def _load_csv_run(p, run):
    run['kind'] = 'csv'
    run['reader'] = 'campaign csv'
    meta = json.loads((p / 'meta.json').read_text()) if (p / 'meta.json').exists() else {}
    run['commit'] = meta.get('git_commit')
    run['dirty'] = None
    ros0 = (meta.get('extra_meta') or {}).get('ros_start_time')
    st = meta.get('start_time')
    run['start_utc'] = (dtm.datetime.fromtimestamp(ros0, dtm.timezone.utc) if ros0 else
                        (dtm.datetime.fromisoformat(st).astimezone(dtm.timezone.utc) if st else None))
    run['gain'], run['gain_note'] = gain_at(run['start_utc'])
    run['mission_window'] = None
    run['u_topic'] = '/drive (commands.csv)'

    def rd(f):
        with open(p / f) as fh:
            return list(csv.DictReader(fh))
    cm = [(float(r['t']), float(r['cmd_speed'])) for r in rd('commands.csv')
          if r.get('cmd_speed') not in (None, '') and r.get('source', '/drive') in ('/drive', '')]
    kin = []
    for r in rd('kinematics.csv'):
        try:
            vx, vy, yaw = float(r['vx']), float(r['vy']), float(r['yaw'])
        except (ValueError, TypeError):
            continue
        kin.append((float(r['t']), vx * math.cos(yaw) + vy * math.sin(yaw)))  # back to body frame
    run['u'] = np.array(sorted(cm))
    run['y'] = {'filt': np.array(sorted(kin))}
    run['timebase'] = {'note': 'campaign recorder t = header stamp (receive time if stamp > 5 s off)'}
    run['notes'].append('kinematics.csv vx,vy = /odometry/filtered twist rotated to odom frame; '
                        'rotated back with yaw to body-frame forward speed')
    return run


# ----------------------------------------------------------------------------- grid
def hold_mask(run, key, t):
    """False for the samples of `key` that the MPC used as its measured speed at a command
    tick (the latest /odometry/filtered sample, and the /odom or /sensors/core samples that
    fed it).  u[k] = v_hold[k] + a ts contains those samples' noise, so they must not
    appear on the output side of a fit."""
    ys = run['y'][key]
    keep = np.ones(len(ys), bool)
    fi = run['y'].get('filt')
    if fi is None or not len(fi):
        return keep
    j = np.clip(np.searchsorted(fi[:, 0], t, side='right') - 1, 0, None)
    if key == 'filt':
        keep[j] = False
        return keep
    jo = np.searchsorted(ys[:, 0], fi[j, 0], side='right') - 1
    for dj in (0, -1):
        keep[np.clip(jo + dj, 0, len(ys) - 1)] = False
    return keep


def tick_grid(run, key, exclude_hold=None):
    """Command ticks and y at each tick (mean of the samples in a centred +-50 ms window;
    a window holding any deadband zero is 0 if all zeros, else NaN = censored)."""
    u = run['u']
    t, uk = u[:, 0], u[:, 1]
    ys = run['y'][key]
    yt, yv = ys[:, 0], ys[:, 1]
    if key == 'core_erpm':
        yv = yv / run['gain']
    if EXCLUDE_HOLD if exclude_hold is None else exclude_hold:
        keep = hold_mask(run, key, t)
        yt, yv = yt[keep], yv[keep]
    lo = np.searchsorted(yt, t - HALF_WIN, side='right')
    hi = np.searchsorted(yt, t + HALF_WIN, side='right')
    n = hi - lo
    y = np.full(len(t), np.nan)
    zf = np.full(len(t), np.nan)
    for i in range(len(t)):
        if n[i]:
            w = yv[lo[i]:hi[i]]
            z = np.abs(w) < ZERO_EPS
            zf[i] = z.mean()
            y[i] = np.mean(w) if not z.any() else (0.0 if z.all() else np.nan)
    # latest-sample view (what the controller sees at the tick)
    j = np.searchsorted(yt, t, side='right') - 1
    yhold = np.where(j >= 0, yv[np.clip(j, 0, None)], np.nan)
    dt = np.diff(t, append=np.nan)
    brk = ~(dt <= MAX_GAP)            # True: no valid ZOH interval after tick i
    return dict(t=t, u=uk, y=y, zf=zf, n=n, yhold=yhold, dt=dt, brk=brk)


def episodes(g, u_min):
    """Index ranges [i0, i1] of drive episodes: from the last at-rest tick before the
    command rises, to the tick the car is back at rest (or the data ends/breaks)."""
    t, u, y = g['t'], g['u'], g['y']
    rest = (y == 0) & (np.abs(u) < 1e-3)
    eps, i, N = [], 0, len(t)
    while i < N:
        if u[i] > 1e-3 and not g['brk'][max(i - 1, 0)]:
            i0 = i - 1 if i > 0 and rest[i - 1] else i
            k = i
            while k < N - 1 and not g['brk'][k] and not (rest[k] and k > i and u[k] < 1e-3 and rest[k - 1]):
                k += 1
            eps.append((max(i0, 0), k))
            i = k + 1
        else:
            i += 1
    return [e for e in eps if e[1] - e[0] >= 8]


# ----------------------------------------------------------------------------- one-step (ARX)
def onestep_rows(g, d, u_min, sel=None):
    """Valid transitions k -> k+1 in the drive regime, with u[k-d]."""
    t, u, y, brk = g['t'], g['u'], g['y'], g['brk']
    N = len(t)
    ks = []
    for k in range(DMAX, N - 1):
        if sel is not None and not sel[k]:
            continue
        if brk[k - DMAX:k + 1].any():
            continue
        if not (np.isfinite(y[k]) and np.isfinite(y[k + 1])):
            continue
        if y[k] == 0 and y[k + 1] == 0:
            continue                      # rest or censored: no information
        if y[k] == 0 or y[k + 1] == 0:
            continue                      # deadband-censored end point
        if u[k - d] < u_min:
            continue                      # release regime (coast), modelled separately
        ks.append(k)
    return np.array(ks, dtype=int)


def fit_onestep(g, d, ks, w=None, offset=False):
    """Exact-ZOH one-step NLS with per-interval dt.  Returns dict."""
    y0, y1, uu, dt = g['y'][ks], g['y'][ks + 1], g['u'][ks - d], g['dt'][ks]
    w = np.ones(len(ks)) if w is None else w
    sw = np.sqrt(w)

    def res(p):
        tau, K = p[0], p[1]
        u0 = p[2] if offset else 0.0
        a = np.exp(-dt / tau)
        return sw * (y1 - (a * y0 + (1 - a) * K * (uu - u0)))
    p0 = [0.2, 0.8] + ([0.05] if offset else [])
    lb = [0.01, 0.1] + ([-0.3] if offset else [])
    ub = [5.0, 2.0] + ([0.3] if offset else [])
    r = optimize.least_squares(res, p0, bounds=(lb, ub))
    e = r.fun / np.where(sw > 0, sw, 1)
    out = dict(tau=float(r.x[0]), K=float(r.x[1]), n=int(len(ks)),
               rss=float(np.sum(w * e ** 2)), resid_std=float(np.sqrt(np.sum(w * e ** 2) / max(np.sum(w) - len(r.x), 1))))
    if offset:
        out['u0'] = float(r.x[2])
    return out


def fit_arx_linear(g, d, ks):
    """Plain linear LS y[k+1] = a y[k] + b u[k-d]; Euler and exact tau from a."""
    X = np.c_[g['y'][ks], g['u'][ks - d]]
    Y = g['y'][ks + 1]
    (a, b), *_ = np.linalg.lstsq(X, Y, rcond=None)
    e = Y - X @ [a, b]
    res = dict(a=float(a), b=float(b), K=float(b / (1 - a)) if a < 1 else np.nan,
               tau_euler=float(TS / (1 - a)) if a < 1 else np.inf,
               tau_exact=float(-TS / math.log(a)) if 0 < a < 1 else np.nan,
               resid_std=float(e.std(ddof=2)), n=int(len(ks)))
    # two-tap: y[k+1] = a y[k] + b0 u[k] + b1 u[k-1]  -> fractional delay
    X2 = np.c_[g['y'][ks], g['u'][ks], g['u'][ks - 1]]
    (a2, b0, b1), *_ = np.linalg.lstsq(X2, Y, rcond=None)
    res['twotap'] = dict(a=float(a2), b0=float(b0), b1=float(b1),
                         frac_delay_ticks=float(b1 / (b0 + b1)) if (b0 + b1) != 0 else np.nan)
    return res


def iv_rows(g, d, u_min):
    t, u, y, brk = g['t'], g['u'], g['y'], g['brk']
    ks = []
    for k in range(DMAX + 2, len(t) - 1):
        if brk[k - DMAX - 2:k + 1].any():
            continue
        if not (np.isfinite(y[k]) and np.isfinite(y[k + 1]) and np.isfinite(y[k - 2])):
            continue
        if min(y[k], y[k + 1], y[k - 2]) == 0:
            continue
        if min(u[k - d], u[k - d - 1], u[k - d - 2]) < u_min:
            continue
        ks.append(k)
    return np.array(ks, dtype=int)


def fit_iv(g, d, ks):
    """2SLS for y[k+1] = a y[k] + b u[k-d], instruments y[k-2], u[k-d-1], u[k-d-2].
    Consistent under the MPC's feedback u[k] = v_hold[k] + a ts when the measurement noise
    is white over > 1 tick; OLS is not (u[k] carries a measurement of y[k])."""
    if len(ks) < 20:
        return None
    X = np.c_[g['y'][ks], g['u'][ks - d]]
    Z = np.c_[g['y'][ks - 2], g['u'][ks - d - 1], g['u'][ks - d - 2]]
    Y = g['y'][ks + 1]
    P = Z @ np.linalg.lstsq(Z, X, rcond=None)[0]
    a, b = np.linalg.lstsq(P, Y, rcond=None)[0]
    e = Y - X @ [a, b]
    return dict(a=float(a), b=float(b), K=float(b / (1 - a)) if a < 1 else np.nan,
                tau=float(-TS / math.log(a)) if 0 < a < 1 else np.nan,
                tau_euler=float(TS / (1 - a)) if a < 1 else np.nan,
                resid_std=float(e.std(ddof=2)), n=int(len(ks)))


# ----------------------------------------------------------------------------- output error
def simulate(g, i0, i1, d, tau, K, u0, a_c, u_min, y_init):
    t, u, dt = g['t'], g['u'], g['dt']
    ys = np.empty(i1 - i0 + 1)
    ys[0] = y_init
    for j, k in enumerate(range(i0, i1)):
        uk = u[k - d] if k - d >= 0 else 0.0
        h = dt[k]
        if uk >= u_min:
            a = math.exp(-h / tau)
            ys[j + 1] = a * ys[j] + (1 - a) * K * max(uk - u0, 0.0)
        else:
            ys[j + 1] = max(0.0, ys[j] - a_c * h)
    return ys


def oe_residuals(p, g, eps, d, u_min, offset, w=None, coast=True):
    tau, K = p[0], p[1]
    u0 = p[2] if offset else 0.0
    a_c = A_COAST if coast else 1e3
    out = []
    ycens = 0.111
    for (i0, i1) in eps:
        y = g['y'][i0:i1 + 1]
        first = np.flatnonzero(np.isfinite(y))
        if not len(first):
            continue
        f = first[0]
        ys = simulate(g, i0 + f, i1, d, tau, K, u0, a_c, u_min, y[f])
        ym = y[f:]
        r = np.zeros(len(ym))
        ok = np.isfinite(ym) & (ym != 0)
        r[ok] = ym[ok] - ys[ok]
        cz = (ym == 0)                       # deadband zero: only penalise y_sim > 0.111
        r[cz] = -np.maximum(0.0, ys[cz] - ycens)
        out.append(r)
    r = np.concatenate(out) if out else np.zeros(1)
    return r if w is None else r * np.sqrt(w)


def fit_oe(g, eps, d, u_min, offset=False, w=None, coast=True):
    p0 = [0.25, 0.8] + ([0.05] if offset else [])
    lb = [0.02, 0.2] + ([-0.2] if offset else [])
    ub = [3.0, 2.0] + ([0.3] if offset else [])
    r = optimize.least_squares(oe_residuals, p0, bounds=(lb, ub),
                               args=(g, eps, d, u_min, offset, w, coast))
    e = oe_residuals(r.x, g, eps, d, u_min, offset, None, coast)
    out = dict(tau=float(r.x[0]), K=float(r.x[1]), cost=float(np.mean(e ** 2)), n=int(len(e)),
               sim_rmse=float(np.sqrt(np.mean(e ** 2))))
    if offset:
        out['u0'] = float(r.x[2])
    out['a_coast'] = A_COAST if coast else None
    return out


# ----------------------------------------------------------------------------- continuous-time OE
THETA_GRID = np.round(np.arange(-0.10, 0.32, 0.02), 3)


def ct_prepare(run, g, eps, key):
    """Per episode: command change times/values and the raw measurement samples."""
    ys = run['y'][key]
    yt, yv = ys[:, 0], ys[:, 1] / (run['gain'] if key == 'core_erpm' else 1.0)
    out = []
    for (i0, i1) in eps:
        f = np.flatnonzero(np.isfinite(g['y'][i0:i1 + 1]))
        if not len(f) or i1 - (i0 + f[0]) < 8:
            continue
        i0 = i0 + f[0]
        tc, uc = g['t'][i0:i1 + 1], g['u'][i0:i1 + 1]
        m = (yt >= tc[0]) & (yt <= tc[-1])
        out.append(dict(tc=tc, uc=uc, tm=yt[m], ym=yv[m], y0=float(g['y'][i0])))
    return out


def ct_residuals(p, prep, theta, u_min, offset, w=None):
    tau, K = p[0], p[1]
    u0 = p[2] if offset else 0.0
    a_c = A_COAST
    res = []
    for e in prep:
        s = e['tc'] + theta
        uc = e['uc']
        n = len(s)
        ystart = np.empty(n)
        y = e['y0']
        drive = uc >= u_min
        target = K * np.maximum(uc - u0, 0.0)
        h = np.diff(s, append=s[-1] + 1.0)
        for i in range(n):
            ystart[i] = y
            if drive[i]:
                y = target[i] + (y - target[i]) * math.exp(-h[i] / tau)
            else:
                y = max(0.0, y - a_c * h[i])
        tm, ym = e['tm'], e['ym']
        j = np.clip(np.searchsorted(s, tm, side='right') - 1, 0, None)
        dtm = np.maximum(tm - s[j], 0.0)     # samples before the first (shifted) command hold y0
        ysim = np.where(drive[j], target[j] + (ystart[j] - target[j]) * np.exp(-dtm / tau),
                        np.maximum(0.0, ystart[j] - a_c * dtm))
        r = np.where(np.abs(ym) >= ZERO_EPS, ym - ysim, -np.maximum(0.0, ysim - ERPM_TELEM_DB / GAIN_NOW))
        res.append(r)
    r = np.concatenate(res) if res else np.zeros(1)
    return r if w is None else r * np.sqrt(w)


def ct_fit(prep, theta, u_min, offset=False, w=None, p0=None):
    p0 = p0 or ([0.25, 0.85] + ([0.05] if offset else []))
    lb = [0.02, 0.2] + ([-0.2] if offset else [])
    ub = [3.0, 2.0] + ([0.3] if offset else [])
    r = optimize.least_squares(ct_residuals, p0, bounds=(lb, ub), args=(prep, theta, u_min, offset, w),
                               x_scale=[0.1, 0.1] + ([0.05] if offset else []))
    e = ct_residuals(r.x, prep, theta, u_min, offset)
    out = dict(theta=float(theta), tau=float(r.x[0]), K=float(r.x[1]), a_coast=A_COAST,
               rmse=float(np.sqrt(np.mean(e ** 2))), n=int(len(e)), x=r.x.tolist())
    if offset:
        out['u0'] = float(r.x[2])
    return out


def ct_analysis(run, g, eps, key, nboot=0, rng=None):
    u_min = ERPM_MIN_DRIVE / run['gain']
    prep = ct_prepare(run, g, eps, key)
    if not prep:
        return None
    out = dict(lin={}, off={})
    for th in THETA_GRID:
        out['lin'][float(th)] = ct_fit(prep, th, u_min)
        out['off'][float(th)] = ct_fit(prep, th, u_min, offset=True)
    for m in ('lin', 'off'):
        best = min(out[m], key=lambda th: out[m][th]['rmse'])
        out[m + '_best'] = out[m][best]
    if nboot and rng is not None:
        nres = len(ct_residuals(out['lin_best']['x'], prep, out['lin_best']['theta'], u_min, False))
        L = 100     # ~2 s of 50 Hz samples
        bt = dict(theta=[], tau=[], K=[], a_coast=[], theta_off=[], tau_off=[], K1=[], u0=[], eqlag=[])
        for _ in range(nboot):
            w = block_weights(nres, L, rng)
            for m, offset in (('lin', False), ('off', True)):
                costs = {}
                fits = {}
                for th in THETA_GRID[::2]:            # 0.04 s steps inside the bootstrap
                    f = ct_fit(prep, th, u_min, offset=offset, w=w, p0=out[m][float(th)]['x'])
                    e = ct_residuals(f['x'], prep, th, u_min, offset)
                    costs[float(th)] = float(np.sum(w * e ** 2))
                    fits[float(th)] = f
                tb = min(costs, key=costs.get)
                f = fits[tb]
                if m == 'lin':
                    bt['theta'].append(tb)
                    bt['tau'].append(f['tau'])
                    bt['K'].append(f['K'])
                    bt['a_coast'].append(f['a_coast'])
                    bt['eqlag'].append(tb + f['tau'])
                else:
                    bt['theta_off'].append(tb)
                    bt['tau_off'].append(f['tau'])
                    bt['K1'].append(f['K'])
                    bt['u0'].append(f['u0'])
        out['boot'] = {k + '_ci': ci(v) for k, v in bt.items()}
        out['boot']['n'] = nboot
        out['boot']['theta_freq'] = {str(t): int(np.sum(np.isclose(bt['theta'], t))) for t in THETA_GRID[::2]}
    return out


def print_ct(res):
    print('  continuous-time OE on raw samples (dead time theta, exact exponential between command changes, '
          'coast below 500 ERPM):')
    print('   theta | linear: tau     K   a_coast  rmse    | offset: tau    K1     u0   a_coast  rmse')
    for th in res['lin']:
        a, b = res['lin'][th], res['off'][th]
        print('   %.2f  |        %.3f  %.3f  %5.2f  %.4f  |        %.3f  %.3f  %.3f  %5.2f  %.4f' % (
            th, a['tau'], a['K'], a['a_coast'], a['rmse'], b['tau'], b['K'], b['u0'], b['a_coast'], b['rmse']))
    a, b = res['lin_best'], res['off_best']
    print('   best linear: theta %.2f tau %.3f K %.3f a_coast %.2f | best offset: theta %.2f tau %.3f K1 %.3f u0 %.3f' % (
        a['theta'], a['tau'], a['K'], a['a_coast'], b['theta'], b['tau'], b['K'], b['u0']))
    if 'boot' in res:
        bb = res['boot']
        print('   block bootstrap (%d reps, 2 s blocks): theta %s tau %s K %s a_coast %s theta+tau %s; theta freq %s' % (
            bb['n'], fmt_ci(bb['theta_ci']), fmt_ci(bb['tau_ci']), fmt_ci(bb['K_ci']), fmt_ci(bb['a_coast_ci']),
            fmt_ci(bb['eqlag_ci']), bb['theta_freq']))
        print('                                           offset: theta %s tau %s K1 %s u0 %s' % (
            fmt_ci(bb['theta_off_ci']), fmt_ci(bb['tau_off_ci']), fmt_ci(bb['K1_ci']), fmt_ci(bb['u0_ci'])))


# ----------------------------------------------------------------------------- bootstrap
def block_weights(n, L, rng):
    w = np.zeros(n)
    nb = int(math.ceil(n / L))
    for s in rng.integers(0, max(n - L, 1), size=nb):
        w[s:s + L] += 1
    return w * (n / w.sum())


def ci(x):
    x = np.asarray([v for v in x if np.isfinite(v)])
    return [float(np.percentile(x, 2.5)), float(np.percentile(x, 97.5))] if len(x) > 3 else [np.nan, np.nan]


# ----------------------------------------------------------------------------- steady state
def steady_blocks(run, g, key, u_min):
    """Non-overlapping 1 s blocks where the command has been flat for >= 1.5 s."""
    t, u = g['t'], g['u']
    ys = run['y'][key]
    yt, yv = ys[:, 0], ys[:, 1]
    if key == 'core_erpm':
        yv = yv / run['gain']
    blocks = []
    k = 0
    N = len(t)
    while k < N:
        tk = t[k]
        m1 = (t > tk - 1.5) & (t <= tk)
        if (t[k] - t[0] < 1.5 or g['brk'][m1].any() or m1.sum() < 12):
            k += 1
            continue
        a = np.median(u[(t > tk - 1.5) & (t <= tk - 1.0)])
        b = np.median(u[(t > tk - 1.0) & (t <= tk - 0.5)])
        c = np.median(u[(t > tk - 0.5) & (t <= tk)])
        if min(a, b, c) < u_min or max(a, b, c) - min(a, b, c) > 0.04:
            k += 1
            continue
        # block = the last 1.0 s; command flat for 0.5 s before it as well
        sel = (t > tk - 1.0) & (t <= tk)
        sy = (yt > tk - 1.0) & (yt <= tk)
        if sy.sum() < 5:
            k += 1
            continue
        w = yv[sy]
        zf = float(np.mean(np.abs(w) < ZERO_EPS))
        slam_v = None
        if 'slam' in run:
            sl = run['slam']
            a0 = np.searchsorted(sl[:, 0], tk - 1.5)
            a1 = np.searchsorted(sl[:, 0], tk + 0.5) - 1
            if a1 > a0 and sl[a1, 0] - sl[a0, 0] >= 0.8:
                slam_v = math.hypot(sl[a1, 1] - sl[a0, 1], sl[a1, 2] - sl[a0, 2]) / (sl[a1, 0] - sl[a0, 0])
        blocks.append(dict(t0=float(tk - 1.0), t1=float(tk), u=float(np.median(u[sel])),
                           u_std=float(np.std(u[sel])), y=float(np.median(w)), y_mean=float(np.mean(w)),
                           zero_frac=zf, n=int(sy.sum()), slam_v=slam_v))
        k = int(np.searchsorted(t, tk + 1.0, side='left'))
    return blocks


# ----------------------------------------------------------------------------- events
def stop_events(run, key='odom'):
    """Command steps from cruise (u >= 0.2 for 1 s) to < U_MIN: delay, coast-down shape."""
    if key not in run['y'] or run['u'] is None:
        return []
    u = run['u']
    ys = run['y'][key]
    gain = run['gain']
    umin = ERPM_MIN_DRIVE / gain
    out = []
    for i in range(10, len(u)):
        if not (u[i - 1, 1] >= 0.2 and u[i, 1] < umin):
            continue
        ts = u[i, 0]
        prev = (u[:, 0] > ts - 1.0) & (u[:, 0] < ts)
        if u[prev, 1].min() < 0.15 or (u[:, 0][(u[:, 0] > ts) & (u[:, 0] < ts + 1.0)].size and
                                       u[(u[:, 0] > ts) & (u[:, 0] < ts + 1.0), 1].max() >= umin):
            continue
        pre = ys[(ys[:, 0] > ts - 0.3) & (ys[:, 0] <= ts), 1]
        post_m = (ys[:, 0] > ts) & (ys[:, 0] < ts + 1.0)
        tp, vp = ys[post_m, 0] - ts, ys[post_m, 1]
        if len(pre) < 3 or len(tp) < 10 or np.median(pre) < 0.18:
            continue
        v0 = float(np.median(pre))
        nz = vp > ZERO_EPS
        # end of observable decay: first deadband zero
        iz = np.flatnonzero(~nz)
        tz = float(tp[iz[0]]) if len(iz) else np.nan
        obs = nz & (tp < (tz if np.isfinite(tz) else 1.0))
        ev = dict(t=float(ts), v0=v0, u_pre=float(np.median(u[prev, 1])), t_zero=tz, n_obs=int(obs.sum()))
        if obs.sum() >= 4:
            tt, vv = tp[obs], vp[obs]
            # linear ramp with dead time: v = v0 - a (t - theta)+
            def lin(p):
                return vv - (v0 - p[1] * np.maximum(tt - p[0], 0))

            def expo(p):
                return vv - v0 * np.exp(-np.maximum(tt - p[0], 0) / p[1])
            rl = optimize.least_squares(lin, [0.05, 1.0], bounds=([0, 0.05], [0.5, 20]))
            re = optimize.least_squares(expo, [0.05, 0.2], bounds=([0, 0.01], [0.5, 5]))
            ev.update(theta_lin=float(rl.x[0]), a_lin=float(rl.x[1]), rmse_lin=float(np.sqrt(np.mean(rl.fun ** 2))),
                      theta_exp=float(re.x[0]), tau_exp=float(re.x[1]), rmse_exp=float(np.sqrt(np.mean(re.fun ** 2))))
        # mean deceleration from v0 to the deadband (0.111)
        if np.isfinite(tz):
            ev['a_mean_to_db'] = float((v0 - ERPM_TELEM_DB / gain) / tz)
        out.append(ev)
    return out


def start_events(run, key='odom'):
    """Starts from rest (/odom zero for >= 1 s, command rising from 0): when does /odom first
    read non-zero (>= 0.111 m/s), relative to the first command >= ERPM_MIN_DRIVE?"""
    if key not in run['y'] or run['u'] is None:
        return []
    u, ys, gain = run['u'], run['y'][key], run['gain']
    out = []
    moving = np.abs(ys[:, 1]) > ZERO_EPS
    i = 1
    while i < len(u):
        if not (u[i - 1, 1] < 1e-3 and u[i, 1] > 1e-3):
            i += 1
            continue
        tc = u[i, 0]
        before = (ys[:, 0] > tc - 1.0) & (ys[:, 0] <= tc)
        if before.sum() < 10 or moving[before].any():
            i += 1
            continue
        ge = np.flatnonzero((u[:, 0] >= tc) & (u[:, 1] * gain >= ERPM_MIN_DRIVE))
        am = np.flatnonzero((ys[:, 0] > tc) & moving)
        if not len(ge) or not len(am):
            break
        t500, tm = u[ge[0], 0], ys[am[0], 0]
        win = (u[:, 0] >= tc) & (u[:, 0] <= tm)
        if tm - tc > 6.0 or (u[win, 1] < 1e-3).any():
            i += 1
            continue
        j = np.searchsorted(u[:, 0], tm - 0.02, side='right') - 1
        pre = (u[:, 0] >= tc) & (u[:, 0] < tm)
        out.append(dict(t=float(tc), gain=gain, dt_cmd_to_motion=float(tm - tc), dt_500_to_motion=float(tm - t500),
                        erpm_first_cmd=float(u[i, 1] * gain), erpm_first_ge500=float(u[ge[0], 1] * gain),
                        erpm_at_motion=float(u[j, 1] * gain),
                        erpm_max_before=float(u[pre, 1].max() * gain) if pre.any() else float(u[j, 1] * gain),
                        v_first=float(ys[am[0], 1])))
        i = int(np.searchsorted(u[:, 0], tm)) + 1
    return out


def slam_ratio(run):
    """SLAM path / /odom path over moving 1 s+ SLAM intervals (true/odom = G_run/G_true)."""
    if 'slam' not in run or 'odom' not in run['y']:
        return None
    s, o = run['slam'], run['y']['odom']
    num = den = 0.0
    i = 0
    while i < len(s) - 1:
        j = int(np.searchsorted(s[:, 0], s[i, 0] + 1.0))
        if j >= len(s) or s[j, 0] - s[i, 0] > 3.0:
            i += 1
            continue
        m = (o[:, 0] > s[i, 0]) & (o[:, 0] <= s[j, 0])
        if m.sum() >= 10:
            do = np.trapz(o[m, 1], o[m, 0])
            ds = math.hypot(s[j, 1] - s[i, 1], s[j, 2] - s[i, 2])
            if do > 0.15:
                num += ds
                den += do
        i = j
    return num / den if den > 1.0 else None


# ----------------------------------------------------------------------------- per run analysis
def analyse_run(run, key, nboot=0, rng=None, verbose=True):
    g = tick_grid(run, key)
    u_min = ERPM_MIN_DRIVE / run['gain']
    eps = episodes(g, u_min)
    res = dict(topic=key, n_ticks=int(len(g['t'])), n_episodes=len(eps),
               dt_med=float(np.nanmedian(g['dt'])), dt_p95=float(np.nanpercentile(g['dt'], 95)))
    ks_all = {d: onestep_rows(g, d, u_min) for d in range(DMAX + 1)}
    common = sorted(set.intersection(*[set(v.tolist()) for v in ks_all.values()]))
    common = np.array(common, dtype=int)
    res['n_transitions'] = int(len(common))
    res['n_censored_ticks'] = int(np.sum((g['y'] == 0) & (g['u'] >= u_min)) + np.sum(np.isnan(g['y']) & (g['n'] > 0)))
    if len(common) < 15:
        res['skip'] = 'too few valid transitions'
        return res, g, eps
    curve = {}
    for d in range(DMAX + 1):
        c = dict(onestep=fit_onestep(g, d, common), arx=fit_arx_linear(g, d, common))
        if eps:
            c['oe'] = fit_oe(g, eps, d, u_min)
        curve[d] = c
    res['d_curve'] = {d: dict(onestep_rss=curve[d]['onestep']['rss'] / curve[d]['onestep']['n'],
                              oe_cost=curve[d].get('oe', {}).get('cost', np.nan)) for d in curve}
    d_one = int(min(curve, key=lambda d: curve[d]['onestep']['rss']))
    d_oe = int(min(curve, key=lambda d: curve[d].get('oe', {}).get('cost', np.inf))) if eps else d_one
    res['d_onestep'], res['d_oe'] = d_one, d_oe
    res['fits'] = {d: curve[d] for d in curve}
    res['iv'] = {d: fit_iv(g, d, iv_rows(g, d, u_min)) for d in range(3)}
    # one-step residuals of the NOMINAL model (tau, K1, u0) at d = 0, exact ZOH, and of the
    # nominal tau with this run's own proportional K
    tau_n, k1_n, u0_n = NOMINAL
    k0 = ks_all[0]
    a_n = np.exp(-g['dt'][k0] / tau_n)
    pred_off = a_n * g['y'][k0] + (1 - a_n) * k1_n * np.maximum(g['u'][k0] - u0_n, 0)
    Kr = curve[0]['onestep']['K']
    pred_lin = a_n * g['y'][k0] + (1 - a_n) * Kr * g['u'][k0]
    res['nominal_resid'] = dict(tau=tau_n, K1=k1_n, u0=u0_n, n=int(len(k0)),
                                std_offset_model=float(np.std(g['y'][k0 + 1] - pred_off)),
                                mean_offset_model=float(np.mean(g['y'][k0 + 1] - pred_off)),
                                std_prop_model_runK=float(np.std(g['y'][k0 + 1] - pred_lin)), K_run=float(Kr))
    # offset model at the chosen delay
    res['offset_onestep'] = fit_onestep(g, d_one, common, offset=True)
    if eps:
        res['offset_oe'] = fit_oe(g, eps, d_oe, u_min, offset=True)
    # cross-correlation of first differences (closed loop: lag 0 is the feedback path)
    yy = g['y'].copy()
    du, dy = np.diff(g['u']), np.diff(yy)
    ok = np.isfinite(dy) & np.isfinite(du) & ~g['brk'][:-1]
    xc = {}
    for L in range(-3, 6):
        a = du[max(0, -L):len(du) - max(0, L)]
        b = dy[max(0, L):len(dy) - max(0, -L)]
        m = ok[max(0, -L):len(du) - max(0, L)] & ok[max(0, L):len(dy) - max(0, -L)]
        xc[L] = float(np.corrcoef(a[m], b[m])[0, 1]) if m.sum() > 10 else np.nan
    res['xcorr_du_dy'] = xc
    # bootstrap
    if nboot and rng is not None:
        L = 20
        bt = dict(tau=[], K=[], d=[], tau_oe=[], K_oe=[], d_oe=[], u0=[], K1=[], a_coast=[],
                  tau_iv=[], K_iv=[], resid=[])
        kiv = iv_rows(g, 0, u_min)
        for _ in range(nboot):
            if len(kiv) >= 20:
                nb = int(math.ceil(len(kiv) / L))
                pick = np.concatenate([np.arange(s0, min(s0 + L, len(kiv)))
                                       for s0 in rng.integers(0, max(len(kiv) - L, 1), size=nb)])
                fi = fit_iv(g, 0, kiv[pick])
                if fi is not None:
                    bt['tau_iv'].append(fi['tau'])
                    bt['K_iv'].append(fi['K'])
            w = block_weights(len(common), L, rng)
            rss = {d: fit_onestep(g, d, common, w=w) for d in range(DMAX + 1)}
            db = min(rss, key=lambda d: rss[d]['rss'])
            bt['d'].append(db)
            bt['tau'].append(rss[d_one]['tau'])
            bt['K'].append(rss[d_one]['K'])
            bt['resid'].append(rss[0]['resid_std'])
            if eps:
                # weights over the OE residual vector: blocks of consecutive ticks
                nres = len(oe_residuals([0.2, 0.8], g, eps, 0, u_min, False))
                wo = block_weights(nres, L, rng)
                oes = {d: fit_oe(g, eps, d, u_min, w=wo) for d in range(DMAX + 1)}
                for d in oes:  # weighted cost for d selection
                    e = oe_residuals([oes[d]['tau'], oes[d]['K']], g, eps, d, u_min, False)
                    oes[d]['wcost'] = float(np.sum(wo * e ** 2))
                bt['d_oe'].append(min(oes, key=lambda d: oes[d]['wcost']))
                bt['tau_oe'].append(oes[d_oe]['tau'])
                bt['K_oe'].append(oes[d_oe]['K'])
                bt['a_coast'].append(oes[d_oe]['a_coast'])
                fo = fit_oe(g, eps, d_oe, u_min, offset=True, w=wo)
                bt['u0'].append(fo['u0'])
                bt['K1'].append(fo['K'])
        res['boot'] = dict(n=nboot, block_ticks=L,
                           tau_onestep_ci=ci(bt['tau']), K_onestep_ci=ci(bt['K']),
                           d_onestep_freq={int(d): int(np.sum(np.array(bt['d']) == d)) for d in range(DMAX + 1)},
                           tau_oe_ci=ci(bt['tau_oe']), K_oe_ci=ci(bt['K_oe']),
                           d_oe_freq={int(d): int(np.sum(np.array(bt['d_oe']) == d)) for d in range(DMAX + 1)},
                           a_coast_ci=ci(bt['a_coast']), u0_oe_ci=ci(bt['u0']), K1_oe_ci=ci(bt['K1']),
                           tau_iv_d0_ci=ci(bt['tau_iv']), K_iv_d0_ci=ci(bt['K_iv']),
                           resid_std_d0_ci=ci(bt['resid']))
    return res, g, eps


def segment_fits(run, g, eps, key, d):
    """Fits restricted to start-up, steady and stop parts of each episode."""
    u_min = ERPM_MIN_DRIVE / run['gain']
    out = {}
    t = g['t']
    for (i0, i1) in eps:
        tm = np.flatnonzero((g['y'][i0:i1 + 1] > ZERO_EPS))
        if not len(tm):
            continue
        k_move = i0 + tm[0]
        # stop = first release after driving
        rel = np.flatnonzero((g['u'][k_move:i1 + 1] < u_min))
        k_stop = k_move + rel[0] if len(rel) else i1
        sel_start = np.zeros(len(t), bool)
        sel_start[i0:min(k_move + 25, k_stop)] = True        # first 2.5 s of motion
        sel_steady = np.zeros(len(t), bool)
        sel_steady[min(k_move + 25, k_stop):k_stop] = True
        for name, sel in (('startup', sel_start), ('steady', sel_steady)):
            ks = onestep_rows(g, d, u_min, sel)
            if len(ks) >= 8:
                out.setdefault(name, []).append(fit_onestep(g, d, ks))
        # OE on the start-up only
        if k_move + 25 < i1:
            out.setdefault('startup_oe', []).append(fit_oe(g, [(i0, min(k_move + 25, k_stop))], d, u_min,
                                                           coast=False))
    return out


# ----------------------------------------------------------------------------- pooled K(u)
def pooled_gain(blocks_by_run, key, rng, nboot=1000):
    """blocks_by_run: {run: [blocks]} with u, y in that run's m/s; plus run gain.
    Works in ERPM so the two gain eras pool; reports u in today's m/s (ERPM / 4516.9)."""
    rows = []
    for rn, (gain, bl) in blocks_by_run.items():
        for b in bl:
            if b['zero_frac'] >= 0.5:
                continue
            rows.append((rn, b['u'] * gain, b['y'] * gain))
    if not rows:
        return None
    runs = sorted(set(r[0] for r in rows))
    R = np.array([runs.index(r[0]) for r in rows])
    U = np.array([r[1] for r in rows])
    Y = np.array([r[2] for r in rows])
    edges = np.array([0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.65]) * GAIN_NOW

    def binstats(idx):
        Ub, Yb, Rb = U[idx], Y[idx], R[idx]
        out = []
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (Ub >= lo) & (Ub < hi)
            if m.sum() == 0:
                out.append(np.nan)
                continue
            # per-run median ratio, then median across runs (runs weighted equally)
            pr = [np.median(Yb[m & (Rb == r)] / Ub[m & (Rb == r)]) for r in np.unique(Rb[m])]
            out.append(np.median(pr))
        return out

    nominal = binstats(np.arange(len(U)))
    boots = []
    for _ in range(nboot):
        pick = rng.integers(0, len(runs), len(runs))
        idx = np.concatenate([np.flatnonzero(R == r) for r in pick])
        boots.append(binstats(idx))
    boots = np.array(boots)
    table = []
    for j, (lo, hi) in enumerate(zip(edges[:-1], edges[1:])):
        m = (U >= lo) & (U < hi)
        if not m.any():
            continue
        table.append(dict(u_lo=float(lo / GAIN_NOW), u_hi=float(hi / GAIN_NOW),
                          erpm_lo=float(lo), erpm_hi=float(hi),
                          n_blocks=int(m.sum()), n_runs=int(len(np.unique(R[m]))),
                          u_med=float(np.median(U[m]) / GAIN_NOW), y_med=float(np.median(Y[m]) / GAIN_NOW),
                          K=float(nominal[j]), K_ci=ci(boots[:, j]),
                          deficit_erpm=float(np.median(U[m] - Y[m]))))

    # model comparison on block level, leave-one-run-out CV, cluster bootstrap CIs
    def fit_models(idx):
        Ub, Yb = U[idx], Y[idx]
        k_prop = float(np.sum(Ub * Yb) / np.sum(Ub * Ub))
        A = np.c_[Ub, np.ones_like(Ub)]
        (k1, c), *_ = np.linalg.lstsq(A, Yb, rcond=None)
        A2 = np.c_[Ub, Ub ** 2]
        (q1, q2), *_ = np.linalg.lstsq(A2, Yb, rcond=None)
        return k_prop, (float(k1), float(-c / k1)), (float(q1), float(q2))

    kp, (k1, u0), (q1, q2) = fit_models(np.arange(len(U)))
    cv = dict(prop=[], offset=[], quad=[])
    for r in range(len(runs)):
        tr, te = R != r, R == r
        if te.sum() == 0 or len(np.unique(R[tr])) < 3:
            continue
        a, (b1, b0), (c1, c2) = fit_models(np.flatnonzero(tr))
        cv['prop'].append(np.sum((Y[te] - a * U[te]) ** 2))
        cv['offset'].append(np.sum((Y[te] - b1 * (U[te] - b0)) ** 2))
        cv['quad'].append(np.sum((Y[te] - c1 * U[te] - c2 * U[te] ** 2) ** 2))
    bt = dict(kp=[], k1=[], u0=[])
    for _ in range(nboot):
        pick = rng.integers(0, len(runs), len(runs))
        idx = np.concatenate([np.flatnonzero(R == r) for r in pick])
        a, (b1, b0), _q = fit_models(idx)
        bt['kp'].append(a)
        bt['k1'].append(b1)
        bt['u0'].append(b0)
    n = len(U)
    models = dict(
        proportional=dict(K=kp, K_ci=ci(bt['kp']),
                          rmse_erpm=float(np.sqrt(np.mean((Y - kp * U) ** 2))),
                          loro_cv_rmse_erpm=float(np.sqrt(np.sum(cv['prop']) / n))),
        offset=dict(K1=k1, K1_ci=ci(bt['k1']), u0_erpm=u0, u0_erpm_ci=ci(bt['u0']),
                    u0_mps_now=u0 / GAIN_NOW, u0_mps_now_ci=[v / GAIN_NOW for v in ci(bt['u0'])],
                    rmse_erpm=float(np.sqrt(np.mean((Y - k1 * (U - u0)) ** 2))),
                    loro_cv_rmse_erpm=float(np.sqrt(np.sum(cv['offset']) / n))),
        quadratic=dict(c1=q1, c2_per_erpm=q2,
                       rmse_erpm=float(np.sqrt(np.mean((Y - q1 * U - q2 * U ** 2) ** 2))),
                       loro_cv_rmse_erpm=float(np.sqrt(np.sum(cv['quad']) / n))))
    return dict(topic=key, n_blocks=int(n), n_runs=len(runs), table=table, models=models)


# ----------------------------------------------------------------------------- printing
def fmt_ci(v):
    return '[%.3f, %.3f]' % tuple(v) if v and all(np.isfinite(v)) else '[n/a]'


def print_run(run, res):
    print('\n' + '=' * 100)
    print('RUN %s  (%s, %s)  commit %s dirty=%s  start %s' % (
        run['name'], run['kind'], run['reader'], (run.get('commit') or '?')[:10], run.get('dirty'),
        run['start_utc'].isoformat(timespec='seconds') if run.get('start_utc') else '?'))
    print('  gain %.2f (%s); u topic %s; drive-vs-mux max |diff| %s' % (
        run['gain'], run['gain_note'], run['u_topic'], run.get('drive_vs_mux_maxdiff')))
    tb = run.get('timebase', {})
    for k, v in tb.items():
        if isinstance(v, dict):
            print('  timebase %-22s n=%5d  recv-stamp median %6.1f ms  p95 %7.1f ms' % (k, v['n'], v['lag_med_ms'], v['lag_p95_ms']))
    if 'skip' in res:
        print('  [%s] skipped: %s' % (res['topic'], res['skip']))
        return
    print('  [%s] ticks %d (dt median %.3f s, p95 %.3f), episodes %d, valid transitions %d, censored/mixed ticks %d' % (
        res['topic'], res['n_ticks'], res['dt_med'], res['dt_p95'], res['n_episodes'], res['n_transitions'],
        res['n_censored_ticks']))
    print('   d | one-step: tau_exact   K     RSS/N      | ARX lin: tau_eul tau_exa  K     | OE: tau    K    a_coast  cost')
    for d, c in res['fits'].items():
        o, a, e = c['onestep'], c['arx'], c.get('oe', {})
        print('   %d |          %6.3f  %6.3f  %.3e   |       %6.3f  %6.3f  %6.3f |   %6.3f %6.3f %6.2f  %.3e' % (
            d, o['tau'], o['K'], o['rss'] / o['n'], a['tau_euler'], a['tau_exact'], a['K'],
            e.get('tau', np.nan), e.get('K', np.nan), e.get('a_coast', np.nan), e.get('cost', np.nan)))
    a = res['fits'][res['d_onestep']]['arx']['twotap']
    print('  best d: one-step %d, OE %d; two-tap ARX fractional delay %.2f ticks (b0 %.3f b1 %.3f)' % (
        res['d_onestep'], res['d_oe'], a['frac_delay_ticks'], a['b0'], a['b1']))
    o = res['offset_onestep']
    print('  offset model one-step (d=%d): tau %.3f K1 %.3f u0 %.3f m/s, RSS/N %.3e' % (
        res['d_onestep'], o['tau'], o['K'], o['u0'], o['rss'] / o['n']))
    if 'offset_oe' in res:
        o = res['offset_oe']
        print('  offset model OE (d=%d): tau %.3f K1 %.3f u0 %.3f m/s a_coast %.2f, cost %.3e (linear OE %.3e)' % (
            res['d_oe'], o['tau'], o['K'], o['u0'], o['a_coast'], o['cost'], res['fits'][res['d_oe']]['oe']['cost']))
    nr = res.get('nominal_resid')
    if nr:
        print('  nominal model (tau %.2f, d 0): one-step resid std %.4f (offset K1 %.2f u0 %.3f; mean %.4f) | %.4f '
              '(proportional K %.3f), n %d' % (nr['tau'], nr['std_offset_model'], nr['K1'], nr['u0'],
                                               nr['mean_offset_model'], nr['std_prop_model_runK'], nr['K_run'], nr['n']))
    for d, f in res['iv'].items():
        if f:
            print('  IV (2SLS, instruments y[k-2], u[k-d-1], u[k-d-2]) d=%d: tau_exact %.3f tau_euler %.3f K %.3f '
                  'resid std %.4f n %d' % (d, f['tau'], f['tau_euler'], f['K'], f['resid_std'], f['n']))
    print('  xcorr(du, dy) by lag [ticks]: ' + ' '.join('%+d:%.2f' % (k, v) for k, v in res['xcorr_du_dy'].items()))
    if 'boot' in res:
        b = res['boot']
        print('  block bootstrap (%d reps, %d-tick blocks): one-step tau %s K %s d-freq %s' % (
            b['n'], b['block_ticks'], fmt_ci(b['tau_onestep_ci']), fmt_ci(b['K_onestep_ci']), b['d_onestep_freq']))
        print('                                         OE tau %s K %s a_coast %s d-freq %s' % (
            fmt_ci(b['tau_oe_ci']), fmt_ci(b['K_oe_ci']), fmt_ci(b['a_coast_ci']), b['d_oe_freq']))
        print('                                         OE offset: K1 %s u0 %s' % (fmt_ci(b['K1_oe_ci']), fmt_ci(b['u0_oe_ci'])))
        print('                                         IV d=0: tau %s K %s; one-step d=0 resid std %s' % (
            fmt_ci(b['tau_iv_d0_ci']), fmt_ci(b['K_iv_d0_ci']), fmt_ci(b['resid_std_d0_ci'])))


def jsonable(o):
    if isinstance(o, dict):
        return {str(k): jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [jsonable(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, dtm.datetime):
        return o.isoformat()
    return o


def process_run(job):
    """Worker: everything for one run.  Returns (printed text, run record, steady blocks,
    per-run fit rows, stop events, start events)."""
    import contextlib
    import io
    rp, isprim, a, seed = job
    args = argparse.Namespace(**a)
    globals()['A_COAST'] = args.a_coast
    globals()['EXCLUDE_HOLD'] = args.exclude_hold
    globals()['NOMINAL'] = tuple(float(v) for v in args.nominal.split(','))
    rng = np.random.default_rng(seed)
    blocks, per_run_fits, stops, starts = {}, [], [], []
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            run = load_run(rp)
        except Exception as e:
            return ('!! %s: %s\n' % (rp, e), None, {}, [], [], [])
        if run['u'] is None or not len(run['u']) or not run['y']:
            return None
        rr = dict(name=run['name'], kind=run['kind'], commit=run.get('commit'), dirty=run.get('dirty'),
                  start_utc=run.get('start_utc'), gain=run['gain'], gain_note=run['gain_note'],
                  u_topic=run['u_topic'], timebase=run.get('timebase'), slam_odom_ratio=slam_ratio(run),
                  topics={})
        for key in ('filt', 'odom', 'core_erpm'):
            if key not in run['y']:
                continue
            res, g, eps = analyse_run(run, key, nboot=args.boot if isprim else 0, rng=rng)
            if 'skip' not in res and (isprim or not args.quiet):
                print_run(run, res)
            if 'skip' not in res and isprim:
                res['segments'] = segment_fits(run, g, eps, key, res['d_oe'])
                for nm, lst in res['segments'].items():
                    for f in lst:
                        print('    segment %-10s tau %.3f K %.3f n %d' % (nm, f['tau'], f['K'], f['n']))
            if 'skip' not in res and (isprim or args.ct_all):
                ctr = ct_analysis(run, g, eps, key, nboot=args.ctboot if isprim else 0, rng=rng)
                if ctr is not None:
                    res['ct'] = ctr
                    if isprim:
                        print_ct(ctr)
            bl = steady_blocks(run, g, key, ERPM_MIN_DRIVE / run['gain'])
            res['steady_blocks'] = bl
            blocks[key] = (run['gain'], bl)
            if 'skip' not in res:
                d = res['d_oe']
                per_run_fits.append(dict(run=run['name'], topic=key, gain=run['gain'], d_oe=d,
                                         d_onestep=res['d_onestep'],
                                         tau_oe=res['fits'][d]['oe']['tau'] if 'oe' in res['fits'][d] else np.nan,
                                         K_oe=res['fits'][d]['oe']['K'] if 'oe' in res['fits'][d] else np.nan,
                                         tau_onestep=res['fits'][res['d_onestep']]['onestep']['tau'],
                                         K_onestep=res['fits'][res['d_onestep']]['onestep']['K'],
                                         resid_std=res['fits'][res['d_onestep']]['onestep']['resid_std'],
                                         n=res['n_transitions'], u_med=float(np.median(g['u'][g['u'] > 0.05])) if (g['u'] > 0.05).any() else np.nan))
                for dd in (0, 1):
                    f = res['iv'].get(dd)
                    per_run_fits[-1]['tau_iv%d' % dd] = f['tau'] if f else np.nan
                    per_run_fits[-1]['K_iv%d' % dd] = f['K'] if f else np.nan
                per_run_fits[-1]['resid_std_d0'] = res['fits'][0]['onestep']['resid_std']
                per_run_fits[-1]['nominal_resid_std'] = res['nominal_resid']['std_offset_model']
                if 'ct' in res:
                    lb, ob = res['ct']['lin_best'], res['ct']['off_best']
                    per_run_fits[-1].update(ct_theta=lb['theta'], ct_tau=lb['tau'], ct_K=lb['K'],
                                            ct_a_coast=lb['a_coast'], ct_rmse=lb['rmse'],
                                            ct_off_theta=ob['theta'], ct_off_tau=ob['tau'], ct_off_K1=ob['K'],
                                            ct_off_u0=ob['u0'], ct_off_rmse=ob['rmse'])
            if isprim:
                rr['topics'][key] = res
            else:
                rr['topics'][key] = {k: res[k] for k in res if k in ('d_oe', 'd_onestep', 'n_transitions', 'skip', 'steady_blocks')}
        se, st = stop_events(run), start_events(run)
        for e in se:
            e['run'] = run['name']
        for e in st:
            e['run'] = run['name']
        stops += se
        starts += st
    return buf.getvalue(), rr, blocks, per_run_fits, stops, starts


# ----------------------------------------------------------------------------- curated results
def _prim(allres, sub):
    for n, r in allres['runs'].items():
        if sub in n:
            return n, r
    return None, None


def curate(allres, primaries):
    """results.json: the nominal model, its intervals and where each number comes from."""
    out = dict(
        generated=dtm.datetime.now(dtm.timezone.utc).isoformat(timespec='seconds'),
        odometry_topic='/odometry/filtered twist.linear.x (the MPC measured speed); /odom reported alongside '
                       '(/odometry/filtered tracks /odom sample-for-sample: vx_variance 0)',
        command_topic='/ackermann_drive (mux output) when complete in the bag, else /drive; identical where both exist',
        timebase='header stamps; car-recorded bags: receive - stamp median 1-14 ms, p95 <= 21 ms. The 10-19 corridor '
                 'bag was recorded off-car (p95 0.2-1.2 s), so stamps, not receive times, are used everywhere.',
        units='u, y in m/s at speed_to_erpm_gain 4516.9 (today); runs at 5499.27 are pooled in ERPM',
        model=('drive regime (u >= 500 ERPM = 0.111 m/s): y[k+1] = a y[k] + (1-a) K(u) u[k-d], a = exp(-dt/tau); '
               'K(u) = K1 (1 - u0/u) (offset model).  Below ~500 ERPM the VESC gives no drive current.'))
    dyn = allres.get('archive_dynamics', {}).get('filt', {})
    st = allres.get('events', {}).get('stops_summary', {})
    sa = allres.get('events', {}).get('starts_summary', {})
    pg = allres.get('pooled', {}).get('filt', {})
    pgo = allres.get('pooled', {}).get('odom', {})
    nom = dict(
        d_ticks=0,
        d_evidence=dict(stop_dead_time_s_median=st.get('theta_exp_median'), stop_dead_time_s_ci95=st.get('theta_exp_ci'),
                        stop_dead_time_s_p90=st.get('theta_exp_p90'), n_stops=st.get('n'),
                        d_onestep_hist_archive=dyn.get('d_onestep_hist'), d_oe_hist_archive=dyn.get('d_oe_hist'),
                        ct_theta_hist_archive=dyn.get('ct_theta_hist'),
                        note='closed-loop d selection is biased to 0 (u[k] contains the sample v_hold[k]); the CT fit '
                             'even prefers non-causal theta < 0. The exogenous evidence is the stops (u -> 0).'),
        tau_s=dyn.get('tau_iv0_median'), tau_s_ci95=dyn.get('tau_iv0_median_ci'),
        tau_basis='median over archive runs of the IV (2SLS) one-step fit at d=0, exact ZOH; CI = bootstrap over runs',
        tau_run_iqr=dyn.get('tau_iv0_iqr'), tau_n_runs=dyn.get('n_iv0'),
        tau_crosschecks=dict(stop_step_exponential=st.get('tau_exp_median'), stop_step_ci95=st.get('tau_exp_ci'),
                             stop_step_iqr=st.get('tau_exp_iqr'),
                             oe_closed_loop_median=dyn.get('tau_oe_median'), oe_closed_loop_ci95=dyn.get('tau_oe_median_ci'),
                             iv_d1_median=dyn.get('tau_iv1_median')),
        euler_note='Euler a = 1 - ts/tau needs tau_euler = ts/(1-a): at tau 0.17 s, a = exp(-0.1/0.17) = 0.56 '
                   'equals Euler with tau_euler = 0.226 s; use the exact form at ts = 0.1',
        K_by_speed=[dict(u_lo=r['u_lo'], u_hi=r['u_hi'], u_med=r['u_med'], K=r['K'], K_ci95=r['K_ci'],
                         n_runs=r['n_runs'], n_blocks=r['n_blocks'], deficit_erpm=r['deficit_erpm'])
                    for r in pg.get('table', [])],
        K_by_speed_odom=[dict(u_lo=r['u_lo'], u_hi=r['u_hi'], K=r['K'], K_ci95=r['K_ci'], n_runs=r['n_runs'])
                         for r in pgo.get('table', [])],
        offset_model=pg.get('models', {}).get('offset'),
        proportional_model=pg.get('models', {}).get('proportional'),
        quadratic_model=pg.get('models', {}).get('quadratic'),
        resid_std_onestep_archive_median=dyn.get('resid_std_d0_median'))
    out['nominal'] = nom
    for sub in primaries:
        n, r = _prim(allres, sub)
        if not r:
            continue
        pr = dict(name=n, start_utc=r.get('start_utc'), commit=r.get('commit'), dirty=r.get('dirty'), gain=r['gain'],
                  gain_note=r['gain_note'], timebase=r.get('timebase'), topics={})
        for key, t in r['topics'].items():
            bl = [b for b in t.get('steady_blocks', []) if b['zero_frac'] < 0.5]
            kb = np.array([b['y'] / b['u'] for b in bl])
            rng = np.random.default_rng(7)
            steady = dict(n_blocks=len(bl), u_median=float(np.median([b['u'] for b in bl])) if bl else None,
                          K_median=float(np.median(kb)) if bl else None,
                          K_median_ci95=ci([np.median(rng.choice(kb, len(kb))) for _ in range(2000)]) if len(kb) > 3 else None,
                          K_blocks=[round(float(v), 4) for v in kb],
                          u_blocks=[round(b['u'], 4) for b in bl])
            if 'fits' not in t:
                pr['topics'][key] = dict(steady=steady, skip=t.get('skip'))
                continue
            q = dict(d_curve=t['d_curve'], d_onestep=t['d_onestep'], d_oe=t['d_oe'],
                     fits={d: dict(onestep_tau=f['onestep']['tau'], onestep_K=f['onestep']['K'],
                                   onestep_resid_std=f['onestep']['resid_std'],
                                   arx_tau_euler=f['arx']['tau_euler'], arx_tau_exact=f['arx']['tau_exact'],
                                   arx_K=f['arx']['K'], oe_tau=f.get('oe', {}).get('tau'), oe_K=f.get('oe', {}).get('K'))
                           for d, f in t['fits'].items()},
                     iv=t.get('iv'), offset_onestep=t.get('offset_onestep'), offset_oe=t.get('offset_oe'),
                     nominal_resid=t.get('nominal_resid'), boot=t.get('boot'),
                     ct_best=dict(linear=t.get('ct', {}).get('lin_best'), offset=t.get('ct', {}).get('off_best'),
                                  boot=t.get('ct', {}).get('boot')),
                     steady=steady,
                     segments=t.get('segments'))
            pr['topics'][key] = q
        out.setdefault('primary_runs', {})[sub] = pr
    out['stops'] = st
    out['starts'] = sa
    out['low_speed'] = allres.get('low_speed_table')
    out['sweep_20260807'] = allres.get('sweep_20260807')
    out['per_run'] = allres.get('per_run_table')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='*')
    ap.add_argument('--out', default=None, help='output dir for results json (nothing written if absent)')
    ap.add_argument('--boot', type=int, default=200, help='bootstrap reps for primary runs')
    ap.add_argument('--primary', action='append', default=[],
                    help='run-name substring(s) that get the full bootstrap (default: the first run)')
    ap.add_argument('--quiet', action='store_true', help='print only primary runs and pooled tables')
    ap.add_argument('--ctboot', type=int, default=40, help='bootstrap reps for the continuous-time fit (primary runs)')
    ap.add_argument('--ct-all', action='store_true', help='continuous-time fit (no bootstrap) on every run')
    ap.add_argument('--a-coast', type=float, default=A_COAST, help='coast-down decel used in the release regime')
    ap.add_argument('--jobs', type=int, default=1, help='parallel worker processes')
    ap.add_argument('--exclude-hold', action='store_true',
                    help='drop the samples the MPC used at each tick from the y windows')
    ap.add_argument('--nominal', default='%g,%g,%g' % NOMINAL, help='tau,K1,u0 of the nominal model for residuals')
    ap.add_argument('--sweep', action='append', default=[], help='speed_sweep_*.csv (target,actual) to tabulate')
    ap.add_argument('--curate-only', default=None, help='rebuild results.json from an identify_raw.json and exit')
    args = ap.parse_args()
    if args.curate_only:
        raw = json.loads(Path(args.curate_only).read_text())
        prim = args.primary or []
        dst = Path(args.out) if args.out else Path(args.curate_only).parent
        dst.mkdir(parents=True, exist_ok=True)
        (dst / 'results.json').write_text(json.dumps(jsonable(curate(raw, prim)), indent=1))
        print('wrote', dst / 'results.json')
        return
    globals()['A_COAST'] = args.a_coast
    globals()['EXCLUDE_HOLD'] = args.exclude_hold
    globals()['NOMINAL'] = tuple(float(v) for v in args.nominal.split(','))
    rng = np.random.default_rng(20261021)
    primaries = args.primary or [Path(args.runs[0]).name]
    allres = dict(runs={}, pooled={}, events={})
    blocks = {'filt': {}, 'odom': {}, 'core_erpm': {}}
    per_run_fits = []
    stops, starts = [], []
    jobs = []
    for i, rp in enumerate(args.runs):
        isprim = any(x in Path(rp).name for x in primaries)
        jobs.append((rp, isprim, vars(args), 20261021 + i))
    if args.jobs > 1:
        import multiprocessing as mp
        with mp.Pool(args.jobs) as pool:
            results = pool.map(process_run, jobs, chunksize=1)
    else:
        results = [process_run(j) for j in jobs]
    for r in results:
        if r is None:
            continue
        text, rr, blk, prf, se, st = r
        if text:
            print(text, end='')
        if rr is None:
            continue
        allres['runs'][rr['name']] = rr
        for key, v in blk.items():
            blocks[key][rr['name']] = v
        per_run_fits += prf
        stops += se
        starts += st

    # ---- pooled steady-state gain
    print('\n' + '=' * 100)
    print('STEADY-STATE GAIN, pooled over runs (1 s blocks with the command flat for >= 1.5 s; '
          'blocks with >= 50 % deadband zeros dropped)')
    for key in ('filt', 'odom', 'core_erpm'):
        if not blocks[key]:
            continue
        pg = pooled_gain(blocks[key], key, rng)
        if pg is None:
            continue
        allres['pooled'][key] = pg
        print('\n[%s] %d blocks from %d runs.  u in today\'s m/s (= ERPM / 4516.9); K = median over runs of '
              'per-run median(y/u); 95%% CI = run-cluster bootstrap' % (key, pg['n_blocks'], pg['n_runs']))
        print('   u bin [m/s]   ERPM bin     runs blocks  u_med  y_med    K      95% CI          deficit [ERPM]')
        for r in pg['table']:
            print('   %.2f-%.2f  %5.0f-%5.0f  %4d %5d   %.3f  %.3f  %.3f  %-16s %6.0f' % (
                r['u_lo'], r['u_hi'], r['erpm_lo'], r['erpm_hi'], r['n_runs'], r['n_blocks'], r['u_med'],
                r['y_med'], r['K'], fmt_ci(r['K_ci']), r['deficit_erpm']))
        m = pg['models']
        print('   models (block level, ERPM): proportional K=%.3f %s rmse %.0f cv %.0f | offset K1=%.3f %s u0=%.0f ERPM '
              '(%.3f m/s now) %s rmse %.0f cv %.0f | quadratic rmse %.0f cv %.0f' % (
                  m['proportional']['K'], fmt_ci(m['proportional']['K_ci']), m['proportional']['rmse_erpm'],
                  m['proportional']['loro_cv_rmse_erpm'], m['offset']['K1'], fmt_ci(m['offset']['K1_ci']),
                  m['offset']['u0_erpm'], m['offset']['u0_mps_now'], fmt_ci(m['offset']['u0_mps_now_ci']),
                  m['offset']['rmse_erpm'], m['offset']['loro_cv_rmse_erpm'], m['quadratic']['rmse_erpm'],
                  m['quadratic']['loro_cv_rmse_erpm']))

    # ---- per-run dynamic fits across the archive
    if len(per_run_fits) > 3:
        print('\n' + '=' * 100)
        print('PER-RUN DYNAMIC FITS (best d by OE cost; runs with >= 40 transitions)')
        allres['per_run_fits'] = per_run_fits
        for key in ('filt', 'odom'):
            P = [p for p in per_run_fits if p['topic'] == key and p['n'] >= 40 and np.isfinite(p['tau_oe'])]
            if len(P) < 3:
                continue
            tau = np.array([p['tau_oe'] for p in P])
            K = np.array([p['K_oe'] for p in P])
            dd = np.array([p['d_oe'] for p in P])
            d1 = np.array([p['d_onestep'] for p in P])
            rs = np.array([p['resid_std'] for p in P])
            bt = [np.median(rng.choice(tau, len(tau))) for _ in range(2000)]
            bk = [np.median(rng.choice(K, len(K))) for _ in range(2000)]
            tiv = np.array([p.get('tau_iv0', np.nan) for p in P], float)
            tiv = tiv[np.isfinite(tiv)]
            biv = [np.median(rng.choice(tiv, len(tiv))) for _ in range(2000)] if len(tiv) > 3 else []
            tiv1 = np.array([p.get('tau_iv1', np.nan) for p in P], float)
            tiv1 = tiv1[np.isfinite(tiv1)]
            kiv = np.array([p.get('K_iv0', np.nan) for p in P], float)
            kiv = kiv[np.isfinite(kiv)]
            r0 = np.array([p.get('resid_std_d0', np.nan) for p in P], float)
            ctt = np.array([p.get('ct_theta', np.nan) for p in P], float)
            summ_iv = dict(tau_iv0_median=float(np.median(tiv)) if len(tiv) else None, tau_iv0_median_ci=ci(biv),
                           tau_iv0_iqr=[float(np.percentile(tiv, 25)), float(np.percentile(tiv, 75))] if len(tiv) else None,
                           n_iv0=int(len(tiv)), tau_iv1_median=float(np.median(tiv1)) if len(tiv1) else None,
                           K_iv0_median=float(np.median(kiv)) if len(kiv) else None,
                           resid_std_d0_median=float(np.nanmedian(r0)),
                           ct_theta_hist={str(t): int(np.sum(np.isclose(ctt, t))) for t in THETA_GRID})
            summ = dict(n_runs=len(P), tau_oe_median=float(np.median(tau)), tau_oe_median_ci=ci(bt),
                        tau_oe_iqr=[float(np.percentile(tau, 25)), float(np.percentile(tau, 75))],
                        K_oe_median=float(np.median(K)), K_oe_median_ci=ci(bk),
                        d_oe_hist={int(d): int(np.sum(dd == d)) for d in range(DMAX + 1)},
                        d_onestep_hist={int(d): int(np.sum(d1 == d)) for d in range(DMAX + 1)},
                        resid_std_median=float(np.median(rs)), **summ_iv)
            allres.setdefault('archive_dynamics', {})[key] = summ
            print('[%s] IV d=0: tau median %.3f s (95%% CI of median %s, IQR %s, %d runs); IV d=1 tau median %s; '
                  'K_IV median %s; one-step d=0 resid std median %.4f' % (
                      key, summ['tau_iv0_median'] or np.nan, fmt_ci(summ['tau_iv0_median_ci']),
                      summ['tau_iv0_iqr'], summ['n_iv0'], summ['tau_iv1_median'], summ['K_iv0_median'],
                      summ['resid_std_d0_median']))
            if np.isfinite(ctt).any():
                print('[%s] CT-OE best theta histogram (negative = non-causal = feedback artefact): %s' % (
                    key, {k: v for k, v in summ['ct_theta_hist'].items() if v}))
            print('[%s] %d runs: tau_OE median %.3f s (95%% CI of median %s, IQR %.3f-%.3f); K_OE median %.3f %s; '
                  'd(OE) hist %s; d(one-step) hist %s; one-step resid std median %.4f' % (
                      key, len(P), summ['tau_oe_median'], fmt_ci(summ['tau_oe_median_ci']), *summ['tau_oe_iqr'],
                      summ['K_oe_median'], fmt_ci(summ['K_oe_median_ci']), summ['d_oe_hist'], summ['d_onestep_hist'],
                      summ['resid_std_median']))

    # ---- deadband / stick-slip: how often /odom reads exactly zero at a steady command
    zrows = []
    for rn, (gain, bl) in blocks.get('odom', {}).items():
        for b in bl:
            zrows.append((rn, b['u'] * gain, b['zero_frac'], b['y'] * gain, b.get('slam_v')))
    if zrows:
        print('\n' + '=' * 100)
        print('LOW-SPEED: steady 1 s blocks by commanded ERPM (all blocks, incl. >= 50 % zeros); '
              'SLAM speed = true speed over the block +-0.5 s')
        print('   ERPM bin     (m/s now)   runs blocks  zero-frac  blocks>=50%zero  median y/u   SLAM v [m/s] (n)')
        Zt = []
        for lo, hi in ((450, 600), (600, 700), (700, 800), (800, 900), (900, 1000), (1000, 1130), (1130, 1355),
                       (1355, 1581), (1581, 1807), (1807, 2033), (2033, 2258), (2258, 2484), (2484, 2936)):
            sel = [z for z in zrows if lo <= z[1] < hi]
            if not sel:
                continue
            zf = np.array([z[2] for z in sel])
            yu = np.array([z[3] / z[1] for z in sel])
            sv = [z[4] for z in sel if z[4] is not None]
            row = dict(erpm_lo=lo, erpm_hi=hi, u_lo=lo / GAIN_NOW, u_hi=hi / GAIN_NOW, n_blocks=len(sel),
                       n_runs=len(set(z[0] for z in sel)), zero_frac_mean=float(zf.mean()),
                       frac_blocks_ge50=float(np.mean(zf >= 0.5)), median_y_over_u=float(np.median(yu)),
                       slam_v_median=float(np.median(sv)) if sv else None, n_slam=len(sv))
            Zt.append(row)
            print('   %4d-%4d  (%.2f-%.2f)  %4d %5d    %.3f       %.2f          %.3f       %s' % (
                lo, hi, lo / GAIN_NOW, hi / GAIN_NOW, row['n_runs'], row['n_blocks'], row['zero_frac_mean'],
                row['frac_blocks_ge50'], row['median_y_over_u'],
                ('%.3f (%d)' % (row['slam_v_median'], row['n_slam'])) if sv else '-'))
        allres['low_speed_table'] = Zt

    # ---- events
    def boot_med(x, n=2000):
        x = np.asarray([v for v in x if v is not None and np.isfinite(v)])
        if len(x) < 4:
            return [np.nan, np.nan]
        return ci([np.median(rng.choice(x, len(x))) for _ in range(n)])

    print('\n' + '=' * 100)
    print('STOPS (command from cruise >= 0.2 m/s to < 500 ERPM; /odom 50 Hz): %d events' % len(stops))
    S = [s for s in stops if 'a_lin' in s]
    if S:
        def col(k):
            return [s.get(k) for s in S]

        def med(k):
            return float(np.nanmedian(np.array(col(k), float)))
        lin_better = int(np.sum([s['rmse_lin'] < s['rmse_exp'] for s in S]))
        summ = dict(n=len(S), n_runs=len(set(s['run'] for s in S)), v0_median=med('v0'),
                    theta_exp_median=med('theta_exp'), theta_exp_ci=boot_med(col('theta_exp')),
                    theta_exp_p90=float(np.percentile(col('theta_exp'), 90)),
                    tau_exp_median=med('tau_exp'), tau_exp_ci=boot_med(col('tau_exp')),
                    tau_exp_iqr=[float(np.percentile(col('tau_exp'), 25)), float(np.percentile(col('tau_exp'), 75))],
                    a_lin_median=med('a_lin'), a_mean_to_db_median=med('a_mean_to_db'),
                    a_mean_to_db_ci=boot_med(col('a_mean_to_db')), linear_better=lin_better,
                    rmse_lin_median=med('rmse_lin'), rmse_exp_median=med('rmse_exp'))
        allres['events']['stops_summary'] = summ
        print('  %d events from %d runs, v0 median %.3f m/s.  Exponential decay with dead time: theta median %.3f s '
              '%s (p90 %.3f), tau median %.3f s %s IQR %.3f-%.3f;  constant-decel fit worse in %d/%d '
              '(rmse %.3f vs %.3f);  mean decel to the 0.111 deadband %.2f m/s^2 %s' % (
                  summ['n'], summ['n_runs'], summ['v0_median'], summ['theta_exp_median'], fmt_ci(summ['theta_exp_ci']),
                  summ['theta_exp_p90'], summ['tau_exp_median'], fmt_ci(summ['tau_exp_ci']), *summ['tau_exp_iqr'],
                  summ['n'] - lin_better, summ['n'], summ['rmse_lin_median'], summ['rmse_exp_median'],
                  summ['a_mean_to_db_median'], fmt_ci(summ['a_mean_to_db_ci'])))
    allres['events']['stops'] = stops
    print('STARTS from rest (/odom zero >= 1 s, command rising from 0): %d events' % len(starts))
    if starts:
        def scol(k):
            return np.array([s[k] for s in starts], float)
        d5, ea, em = scol('dt_500_to_motion'), scol('erpm_at_motion'), scol('erpm_max_before')
        summ = dict(n=len(starts), n_runs=len(set(s['run'] for s in starts)),
                    dt_500_to_motion_median=float(np.median(d5)), dt_500_to_motion_ci=boot_med(d5),
                    dt_500_to_motion_p90=float(np.percentile(d5, 90)), dt_500_to_motion_max=float(d5.max()),
                    dt_cmd_to_motion_median=float(np.median(scol('dt_cmd_to_motion'))),
                    erpm_at_motion_median=float(np.median(ea)), erpm_at_motion_p10=float(np.percentile(ea, 10)),
                    erpm_at_motion_p90=float(np.percentile(ea, 90)),
                    erpm_max_before_median=float(np.median(em)), erpm_max_before_max=float(em.max()))
        allres['events']['starts_summary'] = summ
        print('  %d events from %d runs: /odom first non-zero %.3f s %s after the first command >= 500 ERPM '
              '(p90 %.3f, max %.3f); %.2f s after the first non-zero command (MPC ramp).  Command in force at motion: '
              'median %.0f ERPM (p10 %.0f, p90 %.0f) = %.3f m/s now; largest command seen before motion: median %.0f, '
              'max %.0f ERPM' % (summ['n'], summ['n_runs'], summ['dt_500_to_motion_median'],
                                 fmt_ci(summ['dt_500_to_motion_ci']), summ['dt_500_to_motion_p90'],
                                 summ['dt_500_to_motion_max'], summ['dt_cmd_to_motion_median'],
                                 summ['erpm_at_motion_median'], summ['erpm_at_motion_p10'],
                                 summ['erpm_at_motion_p90'], summ['erpm_at_motion_median'] / GAIN_NOW,
                                 summ['erpm_max_before_median'], summ['erpm_max_before_max']))
    allres['events']['starts'] = starts

    # ---- the August sweep (fixed-duration drives, tape-measured distance)
    if args.sweep:
        rows = []
        for f in args.sweep:
            with open(f) as fh:
                for r in csv.DictReader(fh):
                    rows.append((float(r['target_speed_mps']), float(r['actual_speed_mps'])))
        rows.sort()
        print('\n' + '=' * 100)
        print('SWEEP %s: actual = tape distance / 3.0 s (spin-up and stop included).  ERPM ratio = actual * 4516.9 '
              '/ (G_sweep * target); G_sweep not recorded (5499.27 = node default at its first commit and vesc.yaml '
              'from 2026-08-10; 4614 = vesc.yaml before)' % ', '.join(Path(f).name for f in args.sweep))
        print('   target  actual  actual/target | G=5499.27: cmd ERPM  ratio  deficit | G=4614: cmd ERPM  ratio  deficit')
        sw = []
        for tg, ac in rows:
            e1, e2 = GAIN_OLD * tg, 4614.0 * tg
            d = dict(target=tg, actual=ac, ratio=ac / tg, erpm_5499=e1, ratio_5499=ac * GAIN_NOW / e1,
                     deficit_5499=e1 - ac * GAIN_NOW, erpm_4614=e2, ratio_4614=ac * GAIN_NOW / e2,
                     deficit_4614=e2 - ac * GAIN_NOW)
            sw.append(d)
            print('   %.2f    %.3f   %.3f        |        %5.0f     %.3f  %5.0f   |       %5.0f     %.3f  %5.0f' % (
                tg, ac, d['ratio'], e1, d['ratio_5499'], d['deficit_5499'], e2, d['ratio_4614'], d['deficit_4614']))
        allres['sweep_20260807'] = sw

    # ---- per-run table
    print('\n' + '=' * 100)
    print('PER-RUN: steady K (median y/u over blocks, /odometry/filtered), level, gain, IV tau')
    prt = []
    for rn, rr in sorted(allres['runs'].items()):
        bl = (rr['topics'].get('filt') or {}).get('steady_blocks') or []
        bl = [b for b in bl if b['zero_frac'] < 0.5]
        pf = [p for p in per_run_fits if p['run'] == rn and p['topic'] == 'filt']
        row = dict(run=rn, start_utc=rr.get('start_utc'), commit=(rr.get('commit') or '')[:10], dirty=rr.get('dirty'),
                   gain=rr['gain'], gain_note=rr['gain_note'], u_topic=rr['u_topic'],
                   slam_odom_ratio=rr.get('slam_odom_ratio'), n_blocks=len(bl),
                   u_med=float(np.median([b['u'] for b in bl])) if bl else None,
                   K_steady=float(np.median([b['y'] / b['u'] for b in bl])) if bl else None,
                   tau_iv0=pf[0].get('tau_iv0') if pf else None, d_onestep=pf[0]['d_onestep'] if pf else None,
                   resid_std_d0=pf[0].get('resid_std_d0') if pf else None)
        prt.append(row)
        if bl or pf:
            print('  %-58s %s %-10s G %7.1f  u %s  K %s  tau_IV %s  d1 %s' % (
                rn[:58], str(row['start_utc'])[:16], row['commit'], row['gain'],
                '%.3f' % row['u_med'] if row['u_med'] else '  -  ', '%.3f' % row['K_steady'] if row['K_steady'] else '  -  ',
                '%.3f' % row['tau_iv0'] if row['tau_iv0'] is not None and np.isfinite(row['tau_iv0']) else '  -  ',
                row['d_onestep']))
    allres['per_run_table'] = prt

    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'identify_raw.json').write_text(json.dumps(jsonable(allres), indent=1))
        (out / 'results.json').write_text(json.dumps(jsonable(curate(allres, primaries)), indent=1))
        print('\nwrote', out / 'identify_raw.json', 'and', out / 'results.json')


if __name__ == '__main__':
    main()
