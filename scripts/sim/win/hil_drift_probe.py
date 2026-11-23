#!/usr/bin/env python3
"""hil_drift_probe -- does the Isaac car stay put with every wheel target at 0? One Isaac per variant.

The bug (2026-11-23, Isaac alone, 3 poses): at rest the ground truth moves ~3.7 mm/s backwards in
the car's frame and /odom accumulates ~7.5 mm/s, while both publish ~-6.5 mm/s as velocity. This
runs Isaac headless (DDS on loopback, no Orin, no stack, no command ever sent) for --seconds of sim
time per variant, logs every 0.1 s (main.py --drift-log), and measures over the last --window s:

    gt      ground-truth displacement rate (|delta p| / window) and its component along the heading
    odom    /odom displacement rate, as published
    q_rear  rear wheel ANGLE change x radius (what /odom would say with wheel_speed_source=joint_position)
    w_rear  mean rear wheel joint VELOCITY x radius (what /odom integrates today)
    vx      mean chassis velocity PhysX reports (the ground truth's twist)

PASS for a variant: |gt| < 0.5 mm/s and |odom| < 0.5 mm/s. The odom column is the /odom of that
run (its wheel_speed_source); q_rear says what joint_position odometry would give.

    hil_drift_probe.bat                                base, hold, hold_pos at pose 16 of the office batch
    hil_drift_probe.bat --variants all                 every variant
    hil_drift_probe.bat --variants base,hold           only these
    hil_drift_probe.bat --pose 3 --seconds 90
    hil_drift_probe.bat --analyze scripts\\sim\\win\\drift_probe\\20261124_101500    re-analyse, no Isaac

Variants (config overrides, see sim/isaac/config/car_spec.yaml leatherback.* and sensors.yaml):
    base      as the batch runs today
    hold      stop_hold: the wheels servoed back to the angle where they stopped
    sleep     reapply_unchanged_targets=false: PhysX may put the car to sleep
    kd        drive_damping=10: a sane kd on the wheel drives instead of the USD's 5.7e9
    hz120     physics at 120 Hz instead of 60
    posodom   wheel_speed_source=joint_position (/odom from wheel angles; physics as base)
    hold_pos  hold + posodom: the candidate fix as a whole

Measured on linus (2026-11-23, base_room, 45 s sim, last 35 s; Docker Isaac 5.1):

    variant   gt     odom   q_rear  w_rear  vx     (mm/s)
    base      3.68   7.98   -3.68   -7.83   -6.57  the wheels ROLL with the creep; joint velocity 2.1x the angle's rate
    kd        3.68   7.91   -3.68   -7.81   -6.61  kd 10 instead of 5.7e9: no effect
    sleep     3.68   7.98   -3.68   -7.83   -6.57  no effect
    hz120     4.00   5.86   -4.00   -5.74   -5.37  velocity bias smaller (1.44x), creep the same
    hold_pos  0.00   0.00    0.00   -4.43   -2.99  PASS (the reported velocities stay biased; positions do not move)
  and with hold_pos, 0.4 m/s for 10 s then stop: drives 4.16 m (odom 4.25 = +2 % scale noise), then 0.00 mm/s.

Output: scripts/sim/win/drift_probe/<timestamp>/ (per variant: isaac log, CSV, CSV.json;
summary.md, summary.json). Read-only for the repo; needs nothing on the Orin.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))

LIMIT_MM_S = 0.5
POSES = REPO / "docs/analysis/simulation/2026-11-13_office-feasible-tests/data/door_runup_left_5m.json"
LB = "car_spec.leatherback"
VARIANTS = {
    "base": [],
    "hold": ["--set", f"{LB}.stop_hold=true"],
    "sleep": ["--set", f"{LB}.reapply_unchanged_targets=false"],
    "kd": ["--set", f"{LB}.drive_damping=10.0"],
    "hz120": ["--physics-hz", "120"],
    "posodom": ["--set", "sensors.odometry.wheel_speed_source=joint_position"],
    "hold_pos": ["--set", f"{LB}.stop_hold=true", "--set", "sensors.odometry.wheel_speed_source=joint_position"],
}


def log(msg: str) -> None:
    print(f"[drift {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ---- analysis (numpy only: also runs on linus on a copied folder) ------------------------------
def analyse(csv: Path, window: float) -> dict:
    import numpy as np

    meta = json.loads(Path(str(csv) + ".json").read_text(encoding="utf-8"))
    with open(csv, encoding="utf-8") as fh:
        cols = fh.readline().strip().split(",")
    d = np.loadtxt(csv, delimiter=",", skiprows=1, ndmin=2)
    if d.shape[0] < 10:
        return {"error": f"{d.shape[0]} rows in {csv.name}"}
    c = {k: d[:, i] for i, k in enumerate(cols)}
    t = c["t"]
    t1 = float(t[-1])
    t0 = t1 - window
    if t0 < 5.0:
        return {"error": f"run too short: {t1:.1f} s sim, window {window:.0f} s needs > {window + 5:.0f} s"}
    i0, i1 = int(np.searchsorted(t, t0)), len(t) - 1
    span = float(t[i1] - t[i0])
    r = float(meta["wheel_radius"])

    def disp(xk, yk, yawk):
        dx, dy = c[xk][i1] - c[xk][i0], c[yk][i1] - c[yk][i0]
        h = c[yawk][i0]
        return 1000 * math.hypot(dx, dy) / span, 1000 * (dx * math.cos(h) + dy * math.sin(h)) / span

    gt, gt_along = disp("gt_x", "gt_y", "gt_yaw")
    od, od_along = disp("odom_x", "odom_y", "odom_yaw")
    sl = slice(i0, i1 + 1)
    q_rear = np.mean([np.unwrap(c[k][sl]) for k in ("q_rl", "q_rr")], axis=0)
    q_front = np.mean([np.unwrap(c[k][sl]) for k in ("q_fl", "q_fr")], axis=0)
    q_rate = 1000 * float(q_rear[-1] - q_rear[0]) * r / span
    w_rate = 1000 * float(np.mean((c["w_rl"][sl] + c["w_rr"][sl]) / 2)) * r
    res = {
        "window_s": round(span, 1), "sim_s": round(t1, 1),
        "gt_mm_s": gt, "gt_along_mm_s": gt_along,
        "odom_mm_s": od, "odom_along_mm_s": od_along,
        "q_rear_mm_s": q_rate,
        "q_front_mm_s": 1000 * float(q_front[-1] - q_front[0]) * r / span,
        "w_rear_mm_s": w_rate,
        "vx_mm_s": 1000 * float(np.mean(c["gt_vx"][sl])),
        "vel_over_pos": (w_rate / q_rate) if abs(q_rate) > 1e-3 else None,
        "steer_abs_mrad": 1000 * float(np.max(np.abs(np.r_[c["steer_l"][sl], c["steer_r"][sl]]))),
        "tgt_w_rear_abs_max": float(np.max(np.abs(np.r_[c["tgt_w_rl"][sl], c["tgt_w_rr"][sl]]))),
        "gt_z_range_mm": 1000 * float(np.ptp(c["gt_z"][sl])),
        "timed_out_all": bool(np.all(c["timed_out"][sl] > 0)),
        "physics_hz": meta.get("physics_hz"), "wheel_speed_source": meta.get("wheel_speed_source"),
        "drive_kd": [meta["kds"][i] for i in meta["drive_idx"]] if "kds" in meta else None,
        "drive_kp": [meta["kps"][i] for i in meta["drive_idx"]] if "kps" in meta else None,
        "steer_kp": [meta["kps"][i] for i in meta["steer_idx"]] if "kps" in meta else None,
        "drive_max_effort": [meta["max_efforts"][i] for i in meta["drive_idx"]] if "max_efforts" in meta else None,
    }
    res["pass"] = abs(gt) < LIMIT_MM_S and abs(od) < LIMIT_MM_S
    return res


def summarise(outdir: Path, window: float) -> int:
    rows = {}
    for csv in sorted(outdir.glob("*.csv")):
        rows[csv.stem] = analyse(csv, window)
    if not rows:
        log(f"no CSV in {outdir}")
        return 1
    f = lambda v: "-" if v is None else (f"{v:+.2f}" if isinstance(v, float) else str(v))  # noqa: E731
    lines = [f"# Isaac drift at rest ({outdir.name})", "",
             f"Window: the last {window:.0f} s of each run; rates in mm/s; PASS = |gt| and |odom| < {LIMIT_MM_S}.", "",
             "| variant | gt | gt along | odom | odom along | q_rear | w_rear | vx | w/q | steer max mrad | PASS |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, r in rows.items():
        if "error" in r:
            lines.append(f"| {name} | {r['error']} |||||||||| no |")
            continue
        lines.append(f"| {name} | {f(r['gt_mm_s'])} | {f(r['gt_along_mm_s'])} | {f(r['odom_mm_s'])} | "
                     f"{f(r['odom_along_mm_s'])} | {f(r['q_rear_mm_s'])} | {f(r['w_rear_mm_s'])} | {f(r['vx_mm_s'])} | "
                     f"{f(r['vel_over_pos'])} | {r['steer_abs_mrad']:.3f} | {'**yes**' if r['pass'] else 'no'} |")
    first = next((r for r in rows.values() if "error" not in r), None)
    if first:
        lines += ["", f"Gains as Isaac reports them (base of the first run): drive kp {first['drive_kp']}, "
                  f"kd {first['drive_kd']}, max effort {first['drive_max_effort']}; steering kp {first['steer_kp']}."]
    lines += ["", "Reading it:",
              "- q_rear close to gt: the wheels ROLL with the creep (a brake on the wheels stops it).",
              "- q_rear ~0 while gt is not: the chassis SLIDES on the tyres (friction/solver creep).",
              "- w/q far from 1: the joint velocity PhysX reports is not the angle's derivative; "
              "/odom from joint_position follows the angle.",
              "- gt vs vx: the same mismatch on the chassis (the ground truth's twist)."]
    (outdir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (outdir / "summary.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    print("\n".join(lines))
    return 0 if all(r.get("pass") for r in rows.values()) else 3


# ---- runs ------------------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variants", default="base,hold,hold_pos", help=f"comma-separated, or all: {', '.join(VARIANTS)}")
    ap.add_argument("--pose", type=int, default=16, help="idx in the office batch poses file (1-based)")
    ap.add_argument("--seconds", type=float, default=75.0, help="sim seconds per run")
    ap.add_argument("--window", type=float, default=60.0, help="measured over the last WINDOW s")
    ap.add_argument("--timeout", type=float, default=900.0, help="wall seconds per run before it is killed")
    ap.add_argument("--realtime", action="store_true", help="paced like the batch (default: as fast as it goes)")
    ap.add_argument("--analyze", default="", metavar="DIR", help="only re-analyse an output folder")
    a = ap.parse_args()
    if a.analyze:
        return summarise(Path(a.analyze), a.window)

    names = list(VARIANTS) if a.variants == "all" else [v for v in a.variants.split(",") if v]
    bad = [v for v in names if v not in VARIANTS]
    if bad:
        log(f"unknown variant(s) {bad}; known: {', '.join(VARIANTS)}")
        return 2
    import hil_win  # the same environment and sim arguments as hil_win.py sim --net local
    cfg = hil_win.Cfg()
    cands = json.loads(POSES.read_text(encoding="utf-8"))
    cands = cands["candidates"] if isinstance(cands, dict) else cands
    st = cands[a.pose - 1]["start"]
    prof, _ds, _ip = hil_win.pick_profile(cfg, "local")
    env = hil_win.base_env(cfg, prof, None)
    ns = SimpleNamespace(world="office", layout="door_left", rate=None, gui=False, keyboard=False, set=[],
                         duration=a.seconds)
    base = [sys.executable, str(cfg.isaac_dir / "sim" / "main.py"), *hil_win.sim_args(cfg, ns),
            "--set", f"scene.car.start.x={st['x']}", "--set", f"scene.car.start.y={st['y']}",
            "--set", f"scene.car.start.z={st.get('z', 0.02)}", "--set", f"scene.car.start.yaw_deg={st['yaw_deg']}"]
    if not a.realtime:
        base.append("--no-realtime")
    outdir = HERE / "drift_probe" / time.strftime("%Y%m%d_%H%M%S")
    outdir.mkdir(parents=True)
    log(f"pose {a.pose} {st}, {a.seconds:.0f} s sim per variant, variants {names}; out {outdir}")
    for name in names:
        csv = outdir / f"{name}.csv"
        cmd = base + VARIANTS[name] + ["--drift-log", str(csv)]
        logf = outdir / f"{name}.log"
        log(f"{name}: Isaac -> {logf.name}")
        (outdir / f"{name}.cmd.txt").write_text(subprocess.list2cmdline(cmd), encoding="utf-8")
        t0 = time.monotonic()
        with open(logf, "w", encoding="utf-8", errors="replace") as out:
            p = subprocess.Popen(cmd, env=env, cwd=str(cfg.isaac_dir), stdout=out, stderr=subprocess.STDOUT)
            try:
                rc = p.wait(timeout=a.timeout)
            except subprocess.TimeoutExpired:
                p.kill()
                rc = p.wait()
                log(f"{name}: killed after {a.timeout:.0f} s wall")
            except KeyboardInterrupt:
                p.terminate()
                p.wait(timeout=40)
                raise
        log(f"{name}: rc {rc}, {time.monotonic() - t0:.0f} s wall")
        if csv.exists():
            r = analyse(csv, a.window)
            log(f"{name}: " + ("ERROR " + r["error"] if "error" in r else
                               f"gt {r['gt_mm_s']:.2f} mm/s, odom {r['odom_mm_s']:.2f} mm/s, "
                               f"q_rear {r['q_rear_mm_s']:+.2f}, w_rear {r['w_rear_mm_s']:+.2f} -> "
                               f"{'PASS' if r['pass'] else 'FAIL'}"))
        else:
            log(f"{name}: no drift log (see {logf.name})")
    return summarise(outdir, a.window)


if __name__ == "__main__":
    sys.exit(main())
