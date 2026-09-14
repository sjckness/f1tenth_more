#!/usr/bin/env python3
"""
One-step prediction error check for a kinematic-bicycle MPC.

Input: a CSV log with one row per control step:
    t, x, y, psi, v, steer_cmd, accel_cmd

    t         seconds, real clock, at the moment telemetry was received
    x, y      position, metres, global frame
    psi       heading, radians, global frame
    v         forward speed, m/s
    steer_cmd steering command as handed to the model, radians
    accel_cmd throttle/acceleration command as handed to the model

What it does:
    1. Replays the model one step ahead from every logged state and compares
       against the next logged state.
    2. Reports the error split into longitudinal / lateral / heading / speed
       in the body frame, so each component points at a different subsystem.
    3. Sweeps assumed actuator latency and reports the value that minimises
       heading error.
    4. Fits effective wheelbase and steering offset from the yaw rate.
    5. Fits throttle gain and drag from the speed.

Usage:
    python mpc_model_check.py log.csv --lf 2.67
    python mpc_model_check.py log.csv --lf 2.67 --latency 0.12 --plot out.png
    python mpc_model_check.py --demo demo.csv      # writes a synthetic log
"""

import argparse
import sys

import numpy as np


ALIASES = {
    "t":         ["t", "time", "stamp", "timestamp", "sec", "secs"],
    "x":         ["x", "px", "pos_x", "pose_x", "position_x", "x_m"],
    "y":         ["y", "py", "pos_y", "pose_y", "position_y", "y_m"],
    "psi":       ["psi", "yaw", "theta", "heading", "yaw_rad", "psi_rad"],
    "v":         ["v", "speed", "vel", "velocity", "vx", "v_x", "speed_mps"],
    "steer_cmd": ["steer_cmd", "steer", "steering", "steering_angle", "delta",
                  "steer_rad", "steering_angle_cmd", "delta_cmd"],
    "accel_cmd": ["accel_cmd", "accel", "acceleration", "a", "a_cmd",
                  "throttle", "throttle_cmd", "acc"],
}


def load_log(path):
    """Read the CSV and map whatever column names it has onto the canonical
    seven. Fails loudly and usefully rather than deep inside the model."""
    raw = np.genfromtxt(path, delimiter=",", names=True)
    found = list(raw.dtype.names or [])
    lowered = {c.lower().lstrip("_"): c for c in found}

    out, mapping, missing = {}, {}, []
    for canon, names in ALIASES.items():
        hit = next((lowered[n] for n in names if n in lowered), None)
        if hit is None:
            missing.append(canon)
        else:
            out[canon] = np.asarray(raw[hit], dtype=float)
            mapping[canon] = hit

    if missing:
        raise SystemExit(
            "Could not find these required columns: "
            + ", ".join(missing)
            + "\nColumns present in the file: "
            + ", ".join(found)
            + "\nEither rename the CSV header to  t,x,y,psi,v,steer_cmd,"
              "accel_cmd  or add the actual name to ALIASES at the top of "
              "this script."
        )

    renamed = {c: n for c, n in mapping.items() if c != n}
    if renamed:
        print("column mapping: "
              + ", ".join(f"{n} -> {c}" for c, n in renamed.items()))
    extra = [c for c in found if c not in mapping.values()]
    if extra:
        print("ignored columns: " + ", ".join(extra))
    return out


def valid_intervals(log, max_dt):
    """Boolean mask over the n-1 step intervals, False where the gap is too
    long to be a genuine consecutive control step.

    The controller does not log ticks that do not solve (hold, goal reached,
    no goal), so the CSV has holes. The one-step check reads dt straight from
    consecutive rows, which would turn a two-second hold into a single
    two-second 'prediction step' with enormous error, polluting every
    statistic in the report.

    Masking the interval is the correct fix; deleting the row after a gap is
    not, because the gap then simply reappears as an even larger interval
    between its neighbours.
    """
    dt = np.diff(log["t"])
    ok = dt <= max_dt
    if not ok.all():
        print(f"masked {int((~ok).sum())} of {len(dt)} intervals with a gap "
              f"> {max_dt*1e3:.0f} ms")
    return ok


def wrap(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def zoh(t_query, t_log, values):
    """Zero-order-hold sample of a command signal. Commands are piecewise
    constant between control steps, so this is the physically correct
    interpolation, not linear."""
    idx = np.searchsorted(t_log, t_query, side="right") - 1
    idx = np.clip(idx, 0, len(values) - 1)
    return values[idx]


def predict(log, lf, steer_sign, latency, n_sub=4):
    """Integrate the kinematic bicycle one control step forward from every
    logged state. Returns predicted (x, y, psi, v) at the next step.

    Model:
        x'   = x   + v cos(psi) dt
        y'   = y   + v sin(psi) dt
        psi' = psi + (v / Lf) * delta * dt
        v'   = v   + a dt

    Commands are resampled at (t - latency) inside each substep, so a
    non-zero latency means the command that was actually acting on the
    vehicle during this interval is the one issued earlier.
    """
    t = log["t"]
    x, y, psi, v = log["x"].copy(), log["y"].copy(), log["psi"].copy(), log["v"].copy()
    dt = np.diff(t)

    for i in range(n_sub):
        frac = (i + 0.5) / n_sub
        t_q = t[:-1] + frac * dt - latency
        delta = steer_sign * zoh(t_q, t, log["steer_cmd"])
        accel = zoh(t_q, t, log["accel_cmd"])
        h = dt / n_sub

        x[:-1] += v[:-1] * np.cos(psi[:-1]) * h
        y[:-1] += v[:-1] * np.sin(psi[:-1]) * h
        psi[:-1] += (v[:-1] / lf) * delta * h
        v[:-1] += accel * h

    return x[:-1], y[:-1], psi[:-1], v[:-1]


def body_frame_errors(log, pred, mask=None):
    """Error between prediction and measurement, rotated into the body frame
    of the starting pose. Each component isolates a different failure:

        longitudinal -> throttle gain, drag, speed scaling
        lateral      -> steering gain, latency, sign
        heading      -> effective wheelbase, steering offset
        speed        -> throttle model
    """
    xp, yp, psip, vp = pred
    xm, ym = log["x"][1:], log["y"][1:]
    psim, vm = log["psi"][1:], log["v"][1:]
    psi0 = log["psi"][:-1]

    dx, dy = xm - xp, ym - yp
    out = {
        "longitudinal": np.cos(psi0) * dx + np.sin(psi0) * dy,
        "lateral": -np.sin(psi0) * dx + np.cos(psi0) * dy,
        "heading": wrap(psim - psip),
        "speed": vm - vp,
    }
    if mask is not None:
        out = {k: v[mask] for k, v in out.items()}
    return out


def summarise(errs, dt, label=""):
    if label:
        print(f"\n--- {label} ---")
    print(f"{'component':<14}{'bias':>12}{'std':>12}{'rms':>12}{'rms rate':>14}")
    units = {
        "longitudinal": ("m", "m/s"),
        "lateral": ("m", "m/s"),
        "heading": ("rad", "rad/s"),
        "speed": ("m/s", "m/s^2"),
    }
    for k, e in errs.items():
        u, ur = units[k]
        rms = float(np.sqrt(np.mean(e**2)))
        rate = float(np.sqrt(np.mean((e / dt) ** 2)))
        print(
            f"{k:<14}{np.mean(e):>11.5f} {np.std(e):>11.5f} "
            f"{rms:>11.5f} {rate:>11.5f} {ur}"
        )


def latency_sweep(log, lf, steer_sign, mask=None, lo=0.0, hi=0.4, n=41):
    """RMS heading error as a function of assumed actuator latency.
    A clear minimum away from zero is the latency the vehicle actually has."""
    grid = np.linspace(lo, hi, n)
    rms = []
    for lat in grid:
        pred = predict(log, lf, steer_sign, lat)
        e = body_frame_errors(log, pred, mask)["heading"]
        rms.append(np.sqrt(np.mean(e**2)))
    rms = np.array(rms)
    return grid, rms, float(grid[int(np.argmin(rms))])


def fit_steering(log, mask=None, v_min=1.0):
    """Regress measured yaw rate on the commanded steering.

        psi_dot = a1 * (v * delta) + a2 * v
        => Lf_eff = 1 / a1
           steering offset = a2 / a1     (radians, in command units)

    Lf_eff is the wheelbase your model should be using. It absorbs any
    scaling between the command and the actual wheel angle, so if it comes
    out far from the geometric wheelbase, the steering gain is wrong, not
    the geometry.
    """
    dt = np.diff(log["t"])
    yaw_rate = wrap(np.diff(log["psi"])) / dt
    v = log["v"][:-1]
    delta = log["steer_cmd"][:-1]

    m = v > v_min
    if mask is not None:
        m &= mask
    if m.sum() < 20:
        return None

    A = np.column_stack([v[m] * delta[m], v[m]])
    coef, *_ = np.linalg.lstsq(A, yaw_rate[m], rcond=None)
    a1, a2 = coef
    resid = yaw_rate[m] - A @ coef
    r2 = 1.0 - np.var(resid) / np.var(yaw_rate[m])
    return {
        "lf_eff": 1.0 / a1 if a1 != 0 else np.inf,
        "offset": a2 / a1 if a1 != 0 else np.nan,
        "r2": float(r2),
        "n": int(m.sum()),
    }


def lateral_accel(log, mask=None):
    """Lateral acceleration a_lat = v * psi_dot, and whether the heading error
    grows with it.

    The kinematic bicycle assumes no tire slip. That holds while a_lat stays
    well under the friction limit (mu * g; on concrete with the F1TENTH
    default mu = 1.0489 that is about 10.3 m/s^2). If |heading error| is flat
    against a_lat, the kinematic model is adequate. If it fans out, the car is
    slipping and no weight tuning or wheelbase correction will fix it -- that
    is the signal to move to a dynamic single-track model.
    """
    dt = np.diff(log["t"])
    yaw_rate = wrap(np.diff(log["psi"])) / dt
    a = np.abs(log["v"][:-1] * yaw_rate)
    return a[mask] if mask is not None else a


def slip_check(a_lat, heading_err, mu=1.0489, g=9.81):
    """Split the samples at the median lateral acceleration and compare
    heading-error RMS in each half. A large ratio means slip."""
    limit = mu * g
    pct = np.percentile(a_lat, [50, 90, 99])
    med = pct[0]
    lo, hi = a_lat <= med, a_lat > med
    if lo.sum() < 10 or hi.sum() < 10:
        return None
    rms_lo = float(np.sqrt(np.mean(heading_err[lo] ** 2)))
    rms_hi = float(np.sqrt(np.mean(heading_err[hi] ** 2)))
    return {
        "p50": pct[0], "p90": pct[1], "p99": pct[2],
        "limit": limit, "frac_of_limit": pct[2] / limit,
        "rms_low": rms_lo, "rms_high": rms_hi,
        "ratio": rms_hi / rms_lo if rms_lo > 0 else np.inf,
    }


def implied_servo_gain(lf_eff, lf_model, gain_assumed):
    """Lf_eff and the steering calibration gain are the same quantity in
    different clothes:  Lf_eff = L * g_true / g_assumed.

    So a wheelbase fit from ordinary driving data is an independent estimate
    of the servo gain, without running a single calibration circle.
    """
    if lf_eff == 0 or not np.isfinite(lf_eff):
        return None
    return gain_assumed * lf_model / lf_eff


def fit_throttle(log, mask=None):
    """Regress measured acceleration on the throttle command and speed.

        v_dot = c1 * throttle + c2 * v + c3

    c1 is the throttle gain your model assumes is 1.0. c2 is lumped drag
    and rolling resistance, which the plain kinematic model ignores.
    """
    dt = np.diff(log["t"])
    accel_meas = np.diff(log["v"]) / dt
    A = np.column_stack(
        [log["accel_cmd"][:-1], log["v"][:-1], np.ones(len(dt))]
    )
    if mask is not None:
        A, accel_meas = A[mask], accel_meas[mask]
    if len(accel_meas) < 10:
        return None
    coef, *_ = np.linalg.lstsq(A, accel_meas, rcond=None)
    resid = accel_meas - A @ coef
    r2 = 1.0 - np.var(resid) / np.var(accel_meas)
    return {"gain": coef[0], "drag": coef[1], "bias": coef[2], "r2": float(r2)}


def make_plots(log, errs, dt, sweep, a_lat, path):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not available, skipping plots", file=sys.stderr)
        return

    t = (log["t"][1:] - log["t"][0])[-len(errs["heading"]):] \
        if len(log["t"]) - 1 != len(errs["heading"]) else log["t"][1:] - log["t"][0]
    fig, ax = plt.subplots(3, 2, figsize=(13, 10))

    for a, (k, e) in zip(ax.flat[:4], errs.items()):
        a.plot(t, e, lw=0.8)
        a.axhline(0, color="k", lw=0.6)
        a.axhline(np.mean(e), color="r", ls="--", lw=0.8, label=f"bias {np.mean(e):.4f}")
        a.set_title(f"{k} error")
        a.set_xlabel("t [s]")
        a.legend(fontsize=8)

    grid, rms, best = sweep
    ax[2, 0].plot(grid * 1e3, rms)
    ax[2, 0].axvline(best * 1e3, color="r", ls="--", label=f"min at {best*1e3:.0f} ms")
    ax[2, 0].set_title("heading RMS vs assumed latency")
    ax[2, 0].set_xlabel("latency [ms]")
    ax[2, 0].legend(fontsize=8)

    ax[2, 1].scatter(a_lat, np.abs(errs["heading"]) / dt, s=3, alpha=0.4)
    ax[2, 1].set_title("heading error rate vs lateral acceleration")
    ax[2, 1].set_xlabel("a_lat [m/s^2]  (flat = kinematic ok, fanning = slip)")
    ax[2, 1].set_ylabel("|rad/s|")

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"\nplots written to {path}")


def write_demo(path, seed=0):
    """Synthetic log from a 'vehicle' with a known steering gain error,
    steering offset, actuator latency and drag. Run the analysis on it to
    confirm the script recovers the injected values before trusting it on
    real data."""
    rng = np.random.default_rng(seed)
    lf_true, gain, offset, latency, drag = 2.90, 0.80, 0.020, 0.150, -0.050
    dt, n = 0.05, 4000

    t = np.arange(n) * dt + rng.normal(0, 0.002, n)
    t = np.maximum.accumulate(t)
    steer_cmd = 0.18 * np.sin(2 * np.pi * 0.12 * t) + 0.07 * np.sin(
        2 * np.pi * 0.41 * t + 1.0
    )
    accel_cmd = 0.6 + 0.5 * np.sin(2 * np.pi * 0.05 * t)

    x = np.zeros(n)
    y = np.zeros(n)
    psi = np.zeros(n)
    v = np.full(n, 8.0)
    lag = int(round(latency / dt))

    for k in range(n - 1):
        h = t[k + 1] - t[k]
        d = gain * (steer_cmd[max(k - lag, 0)] + offset)
        a = accel_cmd[max(k - lag, 0)] + drag * v[k]
        x[k + 1] = x[k] + v[k] * np.cos(psi[k]) * h
        y[k + 1] = y[k] + v[k] * np.sin(psi[k]) * h
        psi[k + 1] = psi[k] + (v[k] / lf_true) * d * h
        v[k + 1] = v[k] + a * h

    x += rng.normal(0, 0.01, n)
    y += rng.normal(0, 0.01, n)
    psi += rng.normal(0, 0.001, n)
    v += rng.normal(0, 0.02, n)

    data = np.column_stack([t, x, y, psi, v, steer_cmd, accel_cmd])
    np.savetxt(
        path,
        data,
        delimiter=",",
        header="t,t_mpc,delta_cmd,delta_real,a_cmd,v_cmd,wheel_speed_cmd,v_real,a_imu",
        comments="",
        fmt="%.6f",
    )
    print(f"demo log written to {path}")
    print(
        f"injected: Lf={lf_true}, steer gain={gain} (so Lf_eff={lf_true/gain:.3f}), "
        f"offset={offset}, latency={latency*1e3:.0f} ms, drag={drag}"
    )


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", nargs="?", help="CSV log file")
    p.add_argument("--lf", type=float, default=2.67, help="wheelbase used by the model")
    p.add_argument("--latency", type=float, default=0.0,
                   help="assumed actuator latency in seconds")
    p.add_argument("--sign", type=float, default=1.0, choices=[1.0, -1.0],
                   help="steering sign convention")
    p.add_argument("--max-dt", type=float, default=0.0,
                   help="drop rows following a gap longer than this [s]; "
                        "0 = auto (1.5x the median dt)")
    p.add_argument("--gain", type=float, default=-1.2135,
                   help="steering_angle_to_servo_gain your config assumes")
    p.add_argument("--plot", help="write diagnostic plots to this path")
    p.add_argument("--demo", help="write a synthetic log to this path and exit")
    args = p.parse_args()

    if args.demo:
        write_demo(args.demo)
        return
    if not args.log:
        p.error("need a log file (or --demo)")

    log = load_log(args.log)
    dt_all = np.diff(log["t"])
    nominal = float(np.median(dt_all)) if len(dt_all) else 0.0
    mask = valid_intervals(
        log, args.max_dt if args.max_dt > 0 else 1.5 * nominal)

    dt = dt_all[mask]
    n = int(mask.sum()) + 1
    print(f"{len(log['t'])} rows, {mask.sum()} usable intervals, "
          f"{log['t'][-1]-log['t'][0]:.1f} s")
    if n < 200:
        print(f"WARNING: {n} samples is far too few. The parameter fits need "
              "a few thousand rows with varied steering and speed; anything "
              "below ~200 will produce confident nonsense.")
    print(f"dt: mean {dt.mean()*1e3:.1f} ms, "
          f"min {dt.min()*1e3:.1f}, max {dt.max()*1e3:.1f}, "
          f"std {dt.std()*1e3:.1f} ms")

    # Sign convention. If the wrong sign fits far better, that is the bug.
    for s in (1.0, -1.0):
        e = body_frame_errors(
            log, predict(log, args.lf, s, args.latency), mask)["heading"]
        print(f"steering sign {s:+.0f}: heading RMS {np.sqrt(np.mean(e**2)):.5f} rad")

    pred = predict(log, args.lf, args.sign, args.latency)
    errs = body_frame_errors(log, pred, mask)
    summarise(errs, dt, f"one-step error (Lf={args.lf}, latency={args.latency*1e3:.0f} ms)")

    sweep = latency_sweep(log, args.lf, args.sign, mask)
    print(f"\nbest-fit actuator latency: {sweep[2]*1e3:.0f} ms "
          f"(RMS {sweep[1].min():.5f} vs {sweep[1][0]:.5f} at zero)")

    st = fit_steering(log, mask)
    if st:
        print(f"\nsteering fit (n={st['n']}, R^2={st['r2']:.4f})")
        print(f"  effective wheelbase : {st['lf_eff']:.3f} m  (model uses {args.lf})")
        print(f"  steering offset     : {st['offset']:+.4f} rad "
              f"({np.degrees(st['offset']):+.2f} deg)")

    if st:
        g = implied_servo_gain(st["lf_eff"], args.lf, args.gain)
        if g is not None:
            print(f"  implied servo gain  : {g:.4f} "
                  f"(config assumes {args.gain})")

    a_lat = lateral_accel(log, mask)
    sc = slip_check(a_lat, errs["heading"])
    if sc:
        print(f"\nlateral acceleration (m/s^2): p50 {sc['p50']:.2f}, "
              f"p90 {sc['p90']:.2f}, p99 {sc['p99']:.2f}")
        print(f"  friction limit ~{sc['limit']:.1f}, "
              f"p99 is {sc['frac_of_limit']*100:.0f}% of it")
        print(f"  heading RMS low-a_lat half : {sc['rms_low']:.5f}")
        print(f"  heading RMS high-a_lat half: {sc['rms_high']:.5f} "
              f"(ratio {sc['ratio']:.2f})")
        if sc["ratio"] > 2.0:
            print("  -> error grows with a_lat: tire slip, kinematic model "
                  "is at its limit")
        else:
            print("  -> error flat across a_lat: kinematic model adequate "
                  "in this data")

    th = fit_throttle(log, mask)
    if th:
        print(f"\nthrottle fit (R^2={th['r2']:.4f})")
        print(f"  gain : {th['gain']:.3f}   (model assumes 1.0)")
        print(f"  drag : {th['drag']:.4f} 1/s")
        print(f"  bias : {th['bias']:+.4f} m/s^2")

    if args.plot:
        make_plots(log, errs, dt, sweep, a_lat, args.plot)


if __name__ == "__main__":
    main()