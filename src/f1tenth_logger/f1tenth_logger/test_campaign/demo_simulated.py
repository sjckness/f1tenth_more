#!/usr/bin/env python3
"""Exercise robot_logger, export_campaign_csv and analyze_tests end to end.

No hardware, no ROS: three missions driven by a kinematic stand-in for the
car, with a fake LLM, fake MPC solves, and the occasional contact or estop.
Some runs leave the corridor and abort, which is the point -- the analysis has
to show both outcomes.

    python3 -m f1tenth_logger.test_campaign.demo_simulated \
        [--root DIR] [--repetitions N] [--seed S]

It then does the round trip the campaign depends on:

1. export ``campaign_results.csv``,
2. fill in a few ``success`` / ``transl_ok`` / ``notes`` cells by hand,
3. run more tests and export again,
4. check the hand-entered values survived and the new tests were appended.

Everything lands in a temporary ``f1tenth_more`` root (printed at the end and
left in place for inspection) unless ``--root`` says otherwise.

Note on time: the drive loop passes its own simulated ``t`` to every logging
call, so the ``t`` columns cover a ~13 s drive while ``duration_s`` in
meta.json is the (much shorter) wall-clock time the simulation really took.
On the real robot you never pass ``t``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import tempfile
import time
from pathlib import Path

import yaml

from f1tenth_logger.test_campaign import analyze_tests, export_campaign_csv
from f1tenth_logger.test_campaign.robot_logger import TestLogger, corridor_from_centerline

CAMPAIGN = "first_test_campaing"
WIDTH = 1.2            # corridor width [m]
ROBOT_RADIUS = 0.3     # footprint radius [m]
CRUISE = 0.8           # target speed [m/s]
ACCEL = 0.5            # longitudinal acceleration [m/s^2]
IMU_HZ = 80.0
POSE_DIVISOR = 4       # 80 Hz / 4 = 20 Hz pose, command and MPC rate
VIOLATION_GRACE_S = 0.4  # the car keeps going this long after leaving the corridor
COUNTDOWN_S = 3.0        # standstill between mission_loaded and mission_started
POST_ROLL_S = 0.5        # the logger keeps recording this long after the end
GRAVITY = 9.81
MPC_OK_STATUSES = ("solved",)

PROMPTS = [
    {
        "prompt_num": 1,
        "mission": "M01_red_box",
        "text": "Drive down the corridor and stop at the red box.",
        "success_criterion": "robot stops within 0.3 m of the red box, "
                             "never leaving the corridor",
    },
    {
        "prompt_num": 2,
        "mission": "M02_door_stop",
        "text": "Follow the corridor around the bend and stop at the door.",
        "success_criterion": "robot reaches the door with the footprint inside "
                             "the corridor at every sample",
    },
    {
        "prompt_num": 3,
        "mission": "M03_cone_slalom",
        "text": "Slalom between the cones to the end of the corridor.",
        "success_criterion": "robot passes every cone on the commanded side "
                             "without touching the corridor boundary",
    },
]


# --------------------------------------------------------------------------
# the fake world
# --------------------------------------------------------------------------

class Centerline:
    """A polyline with arc-length lookup of position, tangent and normal."""

    def __init__(self, points):
        self.points = [(float(x), float(y)) for x, y in points]
        self.s = [0.0]
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            self.s.append(self.s[-1] + math.hypot(x1 - x0, y1 - y0))
        self.length = self.s[-1]
        # Per-vertex tangents by central difference, the same convention
        # corridor_from_centerline uses. Taking the segment direction instead
        # makes the normal jump at every vertex, and a laterally offset car
        # then teleports sideways and shows up as a speed spike.
        self.tangents = []
        for i, _ in enumerate(self.points):
            prev = self.points[max(i - 1, 0)]
            nxt = self.points[min(i + 1, len(self.points) - 1)]
            dx, dy = nxt[0] - prev[0], nxt[1] - prev[1]
            norm = math.hypot(dx, dy) or 1.0
            self.tangents.append((dx / norm, dy / norm))

    def at(self, s):
        """(x, y, tangent, left normal) at arc length ``s``, clamped to the ends."""
        s = min(max(s, 0.0), self.length)
        i = 1
        while i < len(self.s) - 1 and self.s[i] < s:
            i += 1
        (x0, y0), (x1, y1) = self.points[i - 1], self.points[i]
        span = self.s[i] - self.s[i - 1]
        u = 0.0 if span <= 0.0 else (s - self.s[i - 1]) / span
        x, y = x0 + u * (x1 - x0), y0 + u * (y1 - y0)
        (tx0, ty0), (tx1, ty1) = self.tangents[i - 1], self.tangents[i]
        tx, ty = tx0 + u * (tx1 - tx0), ty0 + u * (ty1 - ty0)
        norm = math.hypot(tx, ty) or 1.0
        tx, ty = tx / norm, ty / norm
        return x, y, (tx, ty), (-ty, tx)

    def shifted(self, offset):
        """The same line pushed ``offset`` metres to its left."""
        out = []
        for i, (x, y) in enumerate(self.points):
            prev = self.points[max(i - 1, 0)]
            nxt = self.points[min(i + 1, len(self.points) - 1)]
            dx, dy = nxt[0] - prev[0], nxt[1] - prev[1]
            norm = math.hypot(dx, dy) or 1.0
            out.append((x - dy / norm * offset, y + dx / norm * offset))
        return Centerline(out)


def mission_centerline(mission):
    """A different shape per mission: straight, bend, slalom."""
    if mission == "M01_red_box":
        return Centerline([(0.25 * i, 0.0) for i in range(33)])          # 8 m straight
    if mission == "M02_door_stop":
        radius, sweep = 6.0, math.radians(75.0)
        return Centerline(
            [
                (radius * math.sin(sweep * i / 32),
                 radius * (1 - math.cos(sweep * i / 32)))
                for i in range(33)
            ]
        )
    if mission == "M03_cone_slalom":
        return Centerline(
            [(0.28 * i, 0.9 * math.sin(0.28 * i * 0.8)) for i in range(33)]
        )
    raise ValueError(f"no centerline for mission {mission!r}")


def fake_llm(rng, prompt, mission, replan=False):
    """Stand-in for the planner: a 30-250 ms round trip and a plan string."""
    latency = rng.uniform(0.030, 0.250)
    time.sleep(latency * 0.35)          # "first token"
    yield
    time.sleep(latency * 0.65)          # rest of the stream
    yield (
        f'{{"mission": "{mission}", "replan": {str(replan).lower()}, '
        f'"action": "follow_corridor", "speed": {CRUISE}, '
        f'"prompt_chars": {len(prompt)}}}'
    )


def call_llm(logger, rng, prompt, mission, tag, replan=False):
    """Run one fake request inside the logger's llm_call block."""
    stream = fake_llm(rng, prompt, mission, replan)
    with logger.llm_call(prompt, tag=tag) as call:
        next(stream)
        call.first_token()
        call.set_response(next(stream))


class FakeSolver:
    """Mostly solves; now and then drops into a short infeasible burst."""

    def __init__(self, rng, burst_chance=0.02):
        self.rng = rng
        self.burst_chance = burst_chance
        self.burst_left = 0

    def solve(self):
        if self.burst_left > 0:
            self.burst_left -= 1
            status = "infeasible"
        elif self.rng.random() < self.burst_chance:
            self.burst_left = self.rng.randint(1, 4)
            status = "infeasible"
        else:
            status = "solved"
        return {
            "status": status,
            "solve_time_ms": max(0.5, self.rng.gauss(12.0, 3.5)),
            "cost": abs(self.rng.gauss(3.2, 1.5)),
            "iterations": self.rng.randint(4, 30),
        }


def standstill(log, rng, solver, duration, t_start, pose):
    """Motor on, vehicle not moving: the countdown and the post-roll.

    This is what standstill_jerk_rms measures -- the noise floor of this
    particular test, against which its driving jerk means something.
    """
    x, y, yaw = pose
    dt = 1.0 / IMU_HZ
    steps = int(round(duration * IMU_HZ))
    for step in range(steps):
        t = t_start + step * dt
        log.log_imu(
            ax=rng.gauss(0.0, 0.02),
            ay=rng.gauss(0.0, 0.02),
            az=GRAVITY + rng.gauss(0.0, 0.03),
            gx=rng.gauss(0.0, 0.005),
            gy=rng.gauss(0.0, 0.005),
            gz=rng.gauss(0.0, 0.005),
            imu_yaw=yaw,
            sensor_stamp=round(1.7883e9 + t, 6),
            t=t,
        )
        if step % POSE_DIVISOR == 0:
            solve = solver.solve()
            log.log_mpc(solve["status"], solve_time_ms=solve["solve_time_ms"],
                        cost=solve["cost"], iterations=solve["iterations"], t=t)
            log.log_state(x, y, yaw=yaw, t=t)
            log.log_command(cmd_speed=0.0, cmd_steer=0.0, cmd_throttle=0.0,
                            cmd_brake=0.0, source="controller", t=t)
    return t_start + steps * dt


# --------------------------------------------------------------------------
# one simulated run
# --------------------------------------------------------------------------

def run_once(entry, root, rng):
    mission = entry["mission"]
    prompt = entry["text"]
    centerline = mission_centerline(mission)
    solver = FakeSolver(rng)

    # How far the car wanders off the centerline. Half the corridor is 0.6 m
    # and the footprint is 0.3 m, so anything past ~0.3 m clips the boundary:
    # the spread below is chosen to give both outcomes.
    amplitude = rng.uniform(0.08, 0.36)
    wavelength = rng.uniform(3.0, 6.0)
    phase = rng.uniform(0.0, 2.0 * math.pi)
    # A thing standing beside the corridor, 0.9 m off the centerline: the car
    # passes it rather than driving through it, so obstacle_clearance dips and
    # stays positive unless something actually goes wrong.
    ox, oy, _, (onx, ony) = centerline.at(centerline.length * 0.75)
    obstacle = (ox + onx * 0.9, oy + ony * 0.9)
    estop_at = (rng.uniform(0.3, 0.8) * centerline.length
                if rng.random() < 0.12 else None)

    def drive_point(at_s):
        """Where the car is at arc length ``at_s``: centerline plus wander.

        The wander ramps in over the first 0.8 m, so every run starts on the
        centerline and an abort is something that happens while driving.
        """
        cx, cy, _, (nx, ny) = centerline.at(at_s)
        ramp = min(1.0, at_s / 0.8)
        lateral = ramp * amplitude * math.sin(
            2.0 * math.pi * at_s / wavelength + phase
        )
        return cx + nx * lateral, cy + ny * lateral

    def heading(at_s):
        """Tangent of the driven path, from a 4 cm difference around ``at_s``.

        Never from the last pose: at t=0 the car has moved microns and atan2
        of that is pure noise, which lands in the summary as a yaw-rate spike.
        """
        ahead = min(at_s + 0.02, centerline.length)
        behind = max(ahead - 0.04, 0.0)
        (ax, ay), (bx, by) = drive_point(ahead), drive_point(behind)
        return math.atan2(ay - by, ax - bx)

    with TestLogger(
        entry["prompt_num"],
        root=root,
        campaign=CAMPAIGN,
        robot_radius=ROBOT_RADIUS,
        mpc_ok_statuses=MPC_OK_STATUSES,
        extra_meta={
            "robot": "sim-f1tenth",
            "llm_model": "fake-llm-0.1",
            "simulated": True,
            "lateral_amplitude_m": round(amplitude, 4),
        },
    ) as log:
        call_llm(log, rng, prompt, mission, tag="initial")
        log.log_corridor(
            corridor_from_centerline(centerline.points, WIDTH),
            source="llm",
            meta={"width": WIDTH, "stage": "initial"},
            t=0.0,
        )

        plan_id = f"sim_{entry['prompt_num']:03d}_{rng.randrange(1 << 30):08x}"
        log.log_event("mission_loaded", t=0.0, plan_id=plan_id,
                      countdown_s=COUNTDOWN_S, reason="")
        start_pose = (*drive_point(0.0), heading(0.0))
        t = standstill(log, rng, solver, COUNTDOWN_S, 0.0, start_pose)
        log.log_event("mission_started", t=t, plan_id=plan_id, reason="")

        dt = 1.0 / IMU_HZ
        s = 0.0
        speed = 0.0
        prev_yaw = None
        step = 0
        replanned = False
        outcome = None
        violation = None        # (t, clearance) of the first breach

        drive_start = t
        while s < centerline.length:
            speed = min(CRUISE, speed + ACCEL * dt)
            s += speed * dt
            t += dt

            x, y = drive_point(s)
            yaw = heading(s)
            yaw_rate = 0.0
            if prev_yaw is not None:
                yaw_rate = (yaw - prev_yaw + math.pi) % (2 * math.pi) - math.pi
                yaw_rate /= dt

            # IMU at 80 Hz: longitudinal accel, centripetal lateral accel,
            # gravity on z because the frame is z-up and the sensor feels it.
            log.log_imu(
                ax=(ACCEL if speed < CRUISE else 0.0) + rng.gauss(0.0, 0.02),
                ay=speed * yaw_rate + rng.gauss(0.0, 0.03),
                az=GRAVITY + rng.gauss(0.0, 0.05),
                gx=rng.gauss(0.0, 0.01),
                gy=rng.gauss(0.0, 0.01),
                gz=yaw_rate + rng.gauss(0.0, 0.01),
                imu_yaw=yaw,
                sensor_stamp=round(1.7883e9 + t, 6),
                t=t,                     # simulated clock; the real car omits t
            )

            if step % POSE_DIVISOR == 0:
                solve = solver.solve()
                log.log_mpc(
                    solve["status"],
                    solve_time_ms=solve["solve_time_ms"],
                    cost=solve["cost"],
                    iterations=solve["iterations"],
                    t=t,
                )
                clearance = log.log_state(
                    x, y,
                    yaw=yaw,
                    # edge to obstacle, the way perception reports it
                    obstacle_clearance=(
                        math.hypot(x - obstacle[0], y - obstacle[1]) - ROBOT_RADIUS
                    ),
                    t=t,                 # ditto: on the real robot, do not pass t
                )
                log.log_command(
                    cmd_speed=speed,
                    cmd_steer=max(-0.4, min(0.4, yaw_rate * 0.3)),
                    cmd_yaw_rate=yaw_rate,
                    cmd_throttle=speed / CRUISE,
                    cmd_brake=0.0,
                    source="controller",
                    t=t,
                )
                if clearance is not None and clearance < 0.0 and violation is None:
                    violation = (t, clearance)
                    if rng.random() < 0.5:
                        log.log_event(
                            "contact", t=t,
                            cause="footprint crossed the corridor boundary",
                        )
                # The real car needs a moment to notice and stop, so the breach
                # lasts: that is the time viol_rate_pct measures.
                if violation is not None and t - violation[0] >= VIOLATION_GRACE_S:
                    outcome = (
                        "aborted",
                        f"corridor clearance {violation[1]:.3f} m < 0 "
                        f"at t={violation[0]:.2f} s",
                    )
                    break

            if estop_at is not None and s >= estop_at:
                log.log_event("estop", t=t, cause="operator pressed the button")
                log.log_command(cmd_speed=0.0, cmd_brake=1.0, source="estop", t=t)
                outcome = ("aborted", f"emergency stop at t={t:.2f} s")
                break

            if not replanned and s >= centerline.length * 0.5:
                replanned = True
                call_llm(log, rng, prompt, mission, tag="replan", replan=True)
                shift = rng.uniform(-0.08, 0.08)
                log.log_corridor(
                    corridor_from_centerline(
                        centerline.shifted(shift).points, WIDTH
                    ),
                    source="llm",
                    meta={"width": WIDTH, "stage": "replan",
                          "shift_m": round(shift, 4)},
                    t=t,
                )
                log.log_event(
                    "replan",
                    t=t,
                    reason="midpoint checkpoint",
                    shift_m=round(shift, 4),
                    travelled_m=round(s, 3),
                )

            prev_yaw = yaw
            step += 1

        if outcome is None:
            outcome = ("completed", "reached the end of the corridor")
        event = ("mission_finished" if outcome[0] == "completed"
                 else "mission_aborted")
        log.log_event(event, t=t, plan_id=plan_id, reason=outcome[1],
                      drive_duration_s=round(t - drive_start, 3))
        # the logger node keeps recording after the end; so does the demo, and
        # none of it may leak into the driving metrics
        standstill(log, rng, solver, POST_ROLL_S, t, (x, y, yaw))
        log.finish(*outcome)
        return log.summary


# --------------------------------------------------------------------------
# campaign, export round trip, self-check
# --------------------------------------------------------------------------

def write_prompt_table(campaign_dir):
    campaign_dir.mkdir(parents=True, exist_ok=True)
    with open(campaign_dir / "prompts.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(PROMPTS, fh, sort_keys=False, allow_unicode=True)


def run_batch(root, rng, repetitions, label):
    summaries = []
    for entry in PROMPTS:
        for _ in range(repetitions):
            summaries.append(run_once(entry, root, rng))
        done = [s for s in summaries if s["mission"] == entry["mission"]]
        print(f"  {label} {entry['mission']}: "
              f"{sum(s['auto_success'] for s in done)}/{len(done)} auto-completed")
    return summaries


def read_results(path):
    """campaign_results.csv as (fieldnames, rows), whatever separator it uses."""
    with open(path, encoding="utf-8-sig") as fh:
        header = fh.readline()
    delimiter = ";" if header.count(";") > header.count(",") else ","
    with open(path, newline="", encoding="utf-8-sig") as fh:
        reader = csv.DictReader(fh, delimiter=delimiter)
        return reader.fieldnames, list(reader), delimiter


def fill_in_by_hand(path, verdicts):
    """Stand in for the human: write the manual columns, touch nothing else."""
    fieldnames, rows, delimiter = read_results(path)
    for row in rows:
        entry = verdicts.get(row["test_id"])
        if entry is None:
            continue
        row.update(entry)
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames, delimiter=delimiter)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=None,
                        help="project root to write into (default: a temp folder)")
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args(argv)

    if args.root:
        root = Path(args.root).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
    else:
        root = Path(tempfile.mkdtemp(prefix="f1tenth_demo_")) / "f1tenth_more"
        root.mkdir(parents=True)

    campaign_dir = root / CAMPAIGN
    results_csv = campaign_dir / export_campaign_csv.RESULTS_NAME
    write_prompt_table(campaign_dir)
    rng = random.Random(args.seed)
    problems = []

    print(f"[1/5] simulating {len(PROMPTS)} prompts x {args.repetitions} "
          f"repetitions into {campaign_dir}")
    first = run_batch(root, rng, args.repetitions, "     ")

    print("\n[2/5] first export")
    export_campaign_csv.main([str(campaign_dir)])
    _, rows, _ = read_results(results_csv)
    if len(rows) != len(first):
        problems.append(f"export wrote {len(rows)} rows for {len(first)} tests")

    # Stand in for the human filling the sheet in: two tests per mission, one
    # of them deliberately disagreeing with the automatic hint.
    verdicts = {}
    for mission in sorted({r["mission"] for r in rows}):
        picked = [r for r in rows if r["mission"] == mission][:2]
        for i, row in enumerate(picked):
            verdicts[row["test_id"]] = {
                "success": "1" if i == 0 else "0",
                "transl_ok": "1",
                "notes": "plan ok; stopped early (accentué)" if i else "clean run",
            }
    print(f"\n[3/5] filling in {len(verdicts)} verdicts by hand "
          f"(the values the export must never touch)")
    fill_in_by_hand(results_csv, verdicts)

    print("\n[4/5] two more repetitions per prompt, then export again")
    second = run_batch(root, rng, 2, "     ")
    export_campaign_csv.main([str(campaign_dir)])

    _, rows_after, _ = read_results(results_csv)
    by_id = {r["test_id"]: r for r in rows_after}
    if len(rows_after) != len(first) + len(second):
        problems.append(
            f"after the second export there are {len(rows_after)} rows, "
            f"expected {len(first) + len(second)}"
        )
    for test_id, entry in verdicts.items():
        row = by_id.get(test_id)
        if row is None:
            problems.append(f"{test_id} disappeared from the results file")
            continue
        for column, expected in entry.items():
            if row.get(column) != expected:
                problems.append(
                    f"{test_id}.{column} was overwritten: "
                    f"{row.get(column)!r} != {expected!r}"
                )
    new_ids = {s["test_id"] for s in second}
    if not new_ids <= set(by_id):
        problems.append("the second batch was not appended to the results file")
    if any((by_id[i].get("success") or "").strip() for i in new_ids):
        problems.append("new tests arrived with a success verdict already filled in")
    automatic = [r for r in rows_after if r["test_id"] in verdicts]
    if not any((r.get("feas_pct") or "").strip() for r in automatic):
        problems.append("feas_pct was not recomputed for the hand-marked rows")

    def number(row, column):
        text = (row.get(column) or "").strip().replace(",", ".")
        try:
            return float(text)
        except ValueError:
            return None

    for column in ("llm_latency_ms", "n_replans", "countdown_s",
                   "drive_duration_s", "standstill_jerk_rms", "jerk_rms",
                   "viol_rate_pct", "feas_pct"):
        missing = [r["test_id"] for r in rows_after if number(r, column) is None]
        if missing:
            problems.append(
                f"{column} empty for {len(missing)} of {len(rows_after)} tests "
                f"(e.g. {missing[0]})"
            )
    if any(number(r, "countdown_s") != COUNTDOWN_S for r in rows_after):
        problems.append("countdown_s does not match the simulated countdown")
    louder = [r["test_id"] for r in rows_after
              if (number(r, "standstill_jerk_rms") or 0)
              >= (number(r, "jerk_rms") or 0)]
    if louder:
        problems.append(
            f"standstill is as rough as driving for {len(louder)} tests -- the "
            f"metric window is not being applied"
        )
    drive = [number(r, "drive_duration_s") for r in rows_after]
    if any(d is None or d <= 0 or d > 60 for d in drive):
        problems.append("drive_duration_s is not a plausible driving time")

    print("\n[5/5] analysis")
    rc = analyze_tests.main([str(campaign_dir), "--footprints", "2.0"])

    out_dir = campaign_dir / "analysis"
    expected = [
        campaign_dir / "campaign.json",
        campaign_dir / "prompts.yaml",
        campaign_dir / "results.csv",
        results_csv,
        campaign_dir / export_campaign_csv.SETTINGS_NAME,
        out_dir / "summary.csv",
        out_dir / "report.txt",
        out_dir / "dashboard.png",
        out_dir / "overview_map.png",
    ] + [out_dir / f"overview_map_{e['mission']}.png" for e in PROMPTS]

    print("\n--- self-check ---")
    for path in expected:
        ok = path.exists() and path.stat().st_size > 0
        print(f"  {'ok ' if ok else 'MISSING'} {path.relative_to(campaign_dir)}")
        if not ok:
            problems.append(f"missing or empty: {path}")

    summaries = first + second
    completed = sum(s["auto_success"] for s in summaries)
    print(f"  {completed} auto-completed, {len(summaries) - completed} auto-aborted, "
          f"{len(summaries)} total")
    if completed == 0:
        problems.append("no run completed -- the simulated drive never succeeds")
    if completed == len(summaries):
        problems.append("no run aborted -- the abort path was never exercised")

    events = [
        json.loads(line)
        for path in campaign_dir.glob("*/P*/events.jsonl")
        for line in path.read_text(encoding="utf-8").splitlines() if line.strip()
    ]
    names = {e["event"] for e in events}
    print(f"  events logged: {', '.join(sorted(names))}")
    for needed in ("estop", "contact", "replan"):
        if needed not in names:
            problems.append(f"no {needed!r} event was ever logged")

    mpc_rows = sum(
        len(path.read_text(encoding="utf-8").splitlines()) - 1
        for path in campaign_dir.glob("*/P*/mpc.csv")
    )
    infeasible = sum(
        1
        for path in campaign_dir.glob("*/P*/mpc.csv")
        for line in path.read_text(encoding="utf-8").splitlines()
        if ",infeasible," in line
    )
    print(f"  {mpc_rows} MPC solves logged, {infeasible} infeasible")
    if mpc_rows == 0:
        problems.append("no MPC solve was logged")
    if infeasible == 0:
        problems.append("no infeasible solve was logged, so feas_pct is untested")

    rows_csv = (campaign_dir / "results.csv").read_text(encoding="utf-8").splitlines()
    if len(rows_csv) - 1 != len(summaries):
        problems.append(
            f"results.csv holds {len(rows_csv) - 1} rows for {len(summaries)} runs"
        )
    if rc != 0:
        problems.append(f"analyze_tests returned {rc}")

    print(f"\nroot: {root}")
    if problems:
        print("FAILED:")
        for problem in problems:
            print(f"  - {problem}")
        return 1
    print("PASS: logging, MPC and safety events, the manual-column round trip, "
          "and every analysis output are in place.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
