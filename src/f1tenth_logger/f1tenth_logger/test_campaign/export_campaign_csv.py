#!/usr/bin/env python3
"""Turn a campaign's raw logs into one reviewable row per test.

    ros2 run f1tenth_logger test_campaign_export [campaign_folder]
        [--cutoff-hz 5] [--deadband-rad 0.02] [--excel-eu | --plain]

Writes ``<campaign>/campaign_results.csv``: the automatic metrics computed
from each test folder, next to three columns a human fills in by hand --
``success``, ``transl_ok`` and ``notes``.

**The manual columns are never overwritten.** Re-running the export reads the
existing file first, keeps every hand-entered value by ``test_id``, recomputes
only the automatic columns, and appends tests it has not seen. No row is ever
removed, including rows whose test folder has since disappeared.

The driving metrics (``viol_rate_pct``, ``min_clear_m``, ``feas_pct``,
``max_infeas_streak_s``, ``mpc_solve_time_p95_ms``, ``jerk_rms``,
``steer_rev_per_m``) are measured **only between ``mission_started`` and the
end of the mission**, so the countdown and the post-roll cannot dilute them;
a test with no ``mission_started`` event leaves them empty.
``standstill_jerk_rms`` is the same jerk measure over the countdown instead
-- this test's own noise floor.

The file is written to a temporary file and renamed, so an interrupted export
cannot leave a half-written CSV. If the target cannot be replaced because it is
open elsewhere (Excel), the export says so and writes
``campaign_results_NEW.csv`` instead.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

from f1tenth_logger.test_campaign.robot_logger import (
    DEFAULT_CAMPAIGN, find_root, parse_test_id)

RESULTS_NAME = "campaign_results.csv"
FALLBACK_NAME = "campaign_results_NEW.csv"
SETTINGS_NAME = "export_settings.json"
FILTER_ORDER = 2
DEFAULT_CUTOFF_HZ = 5.0
DEFAULT_DEADBAND_RAD = 0.02

#: Filled by hand, never by this script.
MANUAL_COLUMNS = ["success", "transl_ok", "notes"]

COLUMNS = [
    "mission",
    "test_id",
    "prompt_num",
    "repetition",
    "date",
    "time",
    "llm_latency_ms",
    "n_replans",
    "countdown_s",
    "drive_duration_s",
    "standstill_jerk_rms",
    "success",              # MANUAL
    "auto_outcome",
    "estop",
    "contact",
    "viol_rate_pct",
    "min_clear_m",
    "min_clear_raw_m",
    "feas_pct",
    "max_infeas_streak_s",
    "mpc_solve_time_p95_ms",
    "jerk_rms",
    "steer_rev_per_m",
    "transl_ok",            # MANUAL
    "notes",                # MANUAL
]


# --------------------------------------------------------------------------
# reading a test folder
# --------------------------------------------------------------------------

def read_csv_columns(path, columns):
    """{column: float array} with NaN for empty cells; missing file -> empty."""
    out = {c: [] for c in columns}
    if not path.exists():
        return {c: np.empty(0) for c in columns}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            for col in columns:
                raw = (row.get(col) or "").strip()
                try:
                    out[col].append(float(raw) if raw else math.nan)
                except ValueError:
                    out[col].append(math.nan)
    return {c: np.asarray(v, dtype=float) for c, v in out.items()}


def read_jsonl(path):
    if not path.exists():
        return []
    records = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return records


def read_mpc(path):
    """(t, status) lists plus solve times, in file order."""
    times, statuses, solve_ms = [], [], []
    if not path.exists():
        return times, statuses, np.empty(0)
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            raw_t = (row.get("t") or "").strip()
            try:
                times.append(float(raw_t) if raw_t else math.nan)
            except ValueError:
                times.append(math.nan)
            statuses.append((row.get("status") or "").strip())
            raw_ms = (row.get("solve_time_ms") or "").strip()
            try:
                solve_ms.append(float(raw_ms) if raw_ms else math.nan)
            except ValueError:
                solve_ms.append(math.nan)
    return times, statuses, np.asarray(solve_ms, dtype=float)


def read_llm_calls(path):
    """[(tag, latency_ms)] in file order."""
    calls = []
    if not path.exists():
        return calls
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            raw = (row.get("latency_ms") or "").strip()
            try:
                latency = float(raw) if raw else None
            except ValueError:
                latency = None
            calls.append(((row.get("tag") or "").strip(), latency))
    return calls


def first_event_times(events):
    """{event name: t of its first occurrence}."""
    first = {}
    for record in events:
        name = str(record.get("event", ""))
        stamp = record.get("t")
        if name and isinstance(stamp, (int, float)) and name not in first:
            first[name] = float(stamp)
    return first


def in_window(t, window):
    """Boolean mask of the samples inside ``window``; all False if there is none."""
    if window is None:
        return np.zeros(np.shape(t), dtype=bool)
    lo, hi = window
    return np.isfinite(t) & (t >= lo) & (t <= hi)


# --------------------------------------------------------------------------
# metrics
# --------------------------------------------------------------------------

def time_share_below_zero(t, values):
    """Share of *time* (not of samples) spent with ``values < 0``, in percent.

    Each sample holds until the next one, so the interval after sample i is
    charged to sample i. Samples without a value carry no time at all.
    """
    good = np.isfinite(t) & np.isfinite(values)
    if good.sum() < 2:
        return None
    ts, vs = t[good], values[good]
    order = np.argsort(ts, kind="stable")
    ts, vs = ts[order], vs[order]
    dt = np.diff(ts)
    dt = np.clip(dt, 0.0, None)
    total = float(dt.sum())
    if total <= 0.0:
        return None
    violating = float(dt[vs[:-1] < 0.0].sum())
    return 100.0 * violating / total


def feasibility(times, statuses, ok_statuses, run_end):
    """(feas_pct, longest run of consecutive not-ok solves in seconds).

    ``run_end`` closes a streak that was still open when the window ended.
    """
    if not statuses:
        return None, None
    ok = [s in ok_statuses for s in statuses]
    feas_pct = 100.0 * sum(ok) / len(ok)

    longest = 0.0
    start = None
    for stamp, is_ok in zip(times, ok):
        if not math.isfinite(stamp):
            continue
        if is_ok:
            if start is not None:
                longest = max(longest, stamp - start)
                start = None
        elif start is None:
            start = stamp
    if start is not None:
        # still not solving when the run ended
        finite = [x for x in times if math.isfinite(x)]
        end = run_end if run_end is not None else (finite[-1] if finite else start)
        longest = max(longest, max(end, start) - start)
    return feas_pct, longest


def _resample_uniform(t, values):
    """(uniform grid values, dt) at the median sample rate, or (None, None)."""
    good = np.isfinite(t) & np.isfinite(values)
    if good.sum() < 4:
        return None, None
    ts, vs = t[good], values[good]
    order = np.argsort(ts, kind="stable")
    ts, vs = ts[order], vs[order]
    steps = np.diff(ts)
    steps = steps[steps > 0]
    if steps.size == 0:
        return None, None
    dt = float(np.median(steps))
    if dt <= 0.0:
        return None, None
    grid = np.arange(ts[0], ts[-1] + dt * 0.5, dt)
    if grid.size < 4:
        return None, None
    return np.interp(grid, ts, vs), dt


def _zero_phase_lowpass(values, dt, cutoff_hz, order=FILTER_ORDER):
    """Butterworth filtfilt, or None when the record is too short for it."""
    nyquist = 0.5 / dt
    wn = cutoff_hz / nyquist
    if not 0.0 < wn < 1.0:
        # cutoff at or above Nyquist: nothing to remove, use the signal as is
        return values
    b, a = butter(order, wn, btype="low")
    padlen = 3 * max(len(a), len(b))
    if values.size <= padlen:
        return None
    return filtfilt(b, a, values)


def jerk_rms(t, ax, ay, cutoff_hz, min_span_s=2.0):
    """RMS horizontal jerk [m/s^3] from the IMU, gravity (az) ignored."""
    good = np.isfinite(t) & np.isfinite(ax) & np.isfinite(ay)
    if good.sum() < 4:
        return None
    span = float(t[good].max() - t[good].min())
    if span < min_span_s:
        return None
    ax_u, dt = _resample_uniform(t, ax)
    ay_u, dt_y = _resample_uniform(t, ay)
    if ax_u is None or ay_u is None or dt is None or dt_y is None:
        return None
    n = min(ax_u.size, ay_u.size)
    ax_f = _zero_phase_lowpass(ax_u[:n], dt, cutoff_hz)
    ay_f = _zero_phase_lowpass(ay_u[:n], dt, cutoff_hz)
    if ax_f is None or ay_f is None:
        return None
    jx = np.gradient(ax_f, dt)
    jy = np.gradient(ay_f, dt)
    return float(np.sqrt(np.mean(jx * jx + jy * jy)))


def steer_reversals(t, steer, cutoff_hz, deadband_rad):
    """Count of filtered steering sign changes that clear the deadband."""
    steer_u, dt = _resample_uniform(t, steer)
    if steer_u is None:
        return None
    filtered = _zero_phase_lowpass(steer_u, dt, cutoff_hz)
    if filtered is None:
        return None
    reversals = 0
    state = 0
    for value in filtered:
        if value > deadband_rad:
            side = 1
        elif value < -deadband_rad:
            side = -1
        else:
            side = state  # inside the deadband nothing has been decided yet
        if state != 0 and side != 0 and side != state:
            reversals += 1
        state = side
    return reversals


def metrics_for_test(test_dir, mission, cutoff_hz, deadband_rad):
    """Every automatic column for one test folder."""
    parsed = parse_test_id(test_dir.name)
    when = parsed["datetime"]

    meta = {}
    meta_path = test_dir / "meta.json"
    if meta_path.exists():
        try:
            with open(meta_path, encoding="utf-8") as fh:
                meta = json.load(fh)
        except (OSError, json.JSONDecodeError):
            meta = {}
    summary = meta.get("summary", meta)
    # A run with no meta.json never reached finish(): the automatic hint for it
    # is an abort, the same reading analyze_tests takes.
    auto_outcome = summary.get("auto_outcome") or "aborted"
    ok_statuses = frozenset(meta.get("mpc_ok_statuses") or ["solved"])
    path_length = summary.get("path_length")

    events = read_jsonl(test_dir / "events.jsonl")
    names = {str(e.get("event", "")) for e in events}
    estop = 1 if "estop" in names else 0
    contact = 1 if "contact" in names else 0

    kin = read_csv_columns(
        test_dir / "kinematics.csv", ["t", "corridor_clearance", "obstacle_clearance"]
    )
    imu = read_csv_columns(test_dir / "imu.csv", ["t", "ax", "ay"])
    cmd = read_csv_columns(test_dir / "commands.csv", ["t", "cmd_steer"])
    mpc_t, mpc_status, mpc_ms = read_mpc(test_dir / "mpc.csv")
    calls = read_llm_calls(test_dir / "llm_calls.csv")

    ends = [
        float(np.nanmax(arr)) for arr in (kin["t"], imu["t"], cmd["t"])
        if arr.size and np.isfinite(arr).any()
    ]
    finite_mpc_t = [x for x in mpc_t if math.isfinite(x)]
    if finite_mpc_t:
        ends.append(max(finite_mpc_t))
    run_end = max(ends) if ends else None

    # The mission lifecycle decides what is measured. Everything from
    # mission_loaded on is recorded, but only the driving part is scored.
    first = first_event_times(events)
    t_loaded = first.get("mission_loaded")
    t_started = first.get("mission_started")
    finished = [first[k] for k in ("mission_finished", "mission_aborted")
                if k in first]
    t_end = min(finished) if finished else run_end

    drive_window = None
    if t_started is not None:
        drive_window = (t_started, t_end if t_end is not None else math.inf)
    standstill_window = (
        (t_loaded, t_started)
        if t_loaded is not None and t_started is not None and t_started > t_loaded
        else None
    )

    kin_in = in_window(kin["t"], drive_window)
    imu_in = in_window(imu["t"], drive_window)
    cmd_in = in_window(cmd["t"], drive_window)
    mpc_in = [
        i for i, stamp in enumerate(mpc_t)
        if drive_window is not None and math.isfinite(stamp)
        and drive_window[0] <= stamp <= drive_window[1]
    ]

    obstacle = kin["obstacle_clearance"][kin_in]
    obstacle = obstacle[np.isfinite(obstacle)]
    min_clear_raw = float(obstacle.min()) if obstacle.size else None
    if min_clear_raw is None:
        min_clear = None
    elif estop or contact:
        min_clear = 0.0     # it touched something; the measured minimum lies
    else:
        min_clear = min_clear_raw

    feas_pct, infeas_streak = feasibility(
        [mpc_t[i] for i in mpc_in],
        [mpc_status[i] for i in mpc_in],
        ok_statuses,
        drive_window[1] if drive_window and math.isfinite(drive_window[1])
        else run_end,
    )
    # Windowed like the rest: a solve time from the countdown or the post-roll
    # is not a measurement of how the car drove.
    solve_ms = mpc_ms[mpc_in] if mpc_ms.size else mpc_ms
    solve_ms = solve_ms[np.isfinite(solve_ms)]

    reversals = steer_reversals(
        cmd["t"][cmd_in], cmd["cmd_steer"][cmd_in], cutoff_hz, deadband_rad
    )
    if reversals is None or not path_length or float(path_length) <= 0.0:
        steer_rev_per_m = None
    else:
        steer_rev_per_m = reversals / float(path_length)

    initial = next((c for c in calls if c[0] == "initial"), None)
    if initial is None and calls:
        initial = calls[0]          # an older tag: the first call is the initial
    replans = [c for c in calls if c[0] == "replan"]
    n_replans = len(replans) if replans else max(0, len(calls) - 1)

    countdown = None
    for record in events:
        if str(record.get("event", "")) in ("mission_loaded", "mission_started"):
            value = record.get("countdown_s")
            if isinstance(value, (int, float)):
                countdown = float(value)
                break
    if countdown is None and standstill_window is not None:
        countdown = standstill_window[1] - standstill_window[0]

    standstill_mask = in_window(imu["t"], standstill_window)

    return {
        "mission": mission,
        "test_id": test_dir.name,
        "prompt_num": parsed["prompt_num"],
        "repetition": parsed["repetition"],
        "date": when.strftime("%Y-%m-%d"),
        "time": when.strftime("%H:%M:%S"),
        "llm_latency_ms": None if initial is None else initial[1],
        "n_replans": n_replans,
        "countdown_s": countdown,
        "drive_duration_s": (
            None if drive_window is None or not math.isfinite(drive_window[1])
            else drive_window[1] - drive_window[0]
        ),
        "standstill_jerk_rms": jerk_rms(
            imu["t"][standstill_mask], imu["ax"][standstill_mask],
            imu["ay"][standstill_mask], cutoff_hz,
        ),
        "auto_outcome": auto_outcome,
        "estop": estop,
        "contact": contact,
        "viol_rate_pct": time_share_below_zero(
            kin["t"][kin_in], kin["corridor_clearance"][kin_in]
        ),
        "min_clear_m": min_clear,
        "min_clear_raw_m": min_clear_raw,
        "feas_pct": feas_pct,
        "max_infeas_streak_s": infeas_streak,
        "mpc_solve_time_p95_ms": (
            float(np.percentile(solve_ms, 95)) if solve_ms.size else None
        ),
        "jerk_rms": jerk_rms(
            imu["t"][imu_in], imu["ax"][imu_in], imu["ay"][imu_in], cutoff_hz
        ),
        "steer_rev_per_m": steer_rev_per_m,
    }


# --------------------------------------------------------------------------
# the CSV itself
# --------------------------------------------------------------------------

def sniff_delimiter(path):
    """``;`` or ``,``, whichever the existing header uses."""
    try:
        with open(path, encoding="utf-8-sig") as fh:
            header = fh.readline()
    except OSError:
        return ","
    return ";" if header.count(";") > header.count(",") else ","


def read_existing(path):
    """{test_id: full row} of the file as it stands, or {} if it is absent."""
    if not path.exists():
        return {}
    delimiter = sniff_delimiter(path)
    rows = {}
    try:
        with open(path, newline="", encoding="utf-8-sig") as fh:
            for row in csv.DictReader(fh, delimiter=delimiter):
                test_id = (row.get("test_id") or "").strip()
                if test_id:
                    rows[test_id] = {k: (v if v is not None else "")
                                     for k, v in row.items()}
    except OSError as exc:
        raise SystemExit(f"cannot read {path}: {exc}")
    return rows


def format_value(value, decimal_comma):
    if value is None:
        return ""
    if isinstance(value, bool):
        return str(int(value))
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return ""
        text = f"{value:.3f}"
        return text.replace(".", ",") if decimal_comma else text
    return str(value)


def write_rows(path, rows, delimiter, decimal_comma):
    """Temp file then rename, with the Excel-holds-the-file fallback."""
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.writer(fh, delimiter=delimiter)
        writer.writerow(COLUMNS)
        for row in rows:
            writer.writerow(
                [format_value(row.get(col), decimal_comma) for col in COLUMNS]
            )
        fh.flush()
        os.fsync(fh.fileno())
    try:
        os.replace(tmp, path)
        return path
    except PermissionError:
        fallback = path.with_name(FALLBACK_NAME)
        print(
            f"\n{path.name} is locked -- it is probably open in Excel.\n"
            f"Close it and run the export again. This run's results are in\n"
            f"  {fallback}\n",
            file=sys.stderr,
        )
        try:
            os.replace(tmp, fallback)
        except OSError as exc:
            tmp.unlink(missing_ok=True)
            raise SystemExit(f"could not write {fallback}: {exc}")
        return fallback


def merge(existing, computed):
    """Recompute the automatic columns, keep every manual value, drop nothing."""
    rows = {}
    kept_manual = 0
    for test_id, row in existing.items():
        rows[test_id] = {col: row.get(col, "") for col in COLUMNS}
    for row in computed:
        test_id = row["test_id"]
        previous = rows.get(test_id)
        merged = {col: "" for col in COLUMNS}
        if previous is not None:
            for col in MANUAL_COLUMNS:
                value = (previous.get(col) or "").strip()
                merged[col] = value
                if value:
                    kept_manual += 1
        merged.update(row)   # every automatic column, freshly computed
        rows[test_id] = merged
    ordered = sorted(
        rows.values(), key=lambda r: (str(r.get("mission", "")), str(r.get("test_id", "")))
    )
    return ordered, kept_manual


def scan_campaign(campaign_dir, cutoff_hz, deadband_rad):
    computed = []
    for mission_dir in sorted(p for p in campaign_dir.iterdir() if p.is_dir()):
        for test_dir in sorted(p for p in mission_dir.iterdir() if p.is_dir()):
            try:
                parse_test_id(test_dir.name)
            except ValueError:
                continue
            computed.append(
                metrics_for_test(test_dir, mission_dir.name, cutoff_hz, deadband_rad)
            )
    return computed


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Export one reviewable row per test into campaign_results.csv."
    )
    parser.add_argument(
        "campaign_folder", nargs="?", default=None,
        help="campaign folder (default: <f1tenth_more>/%s)" % DEFAULT_CAMPAIGN,
    )
    parser.add_argument("--cutoff-hz", type=float, default=DEFAULT_CUTOFF_HZ,
                        help="low-pass cutoff for jerk and steering [Hz]")
    parser.add_argument("--deadband-rad", type=float, default=DEFAULT_DEADBAND_RAD,
                        help="steering deadband for counting reversals [rad]")
    fmt = parser.add_mutually_exclusive_group()
    fmt.add_argument("--excel-eu", dest="excel_eu", action="store_true", default=True,
                     help="';' separator and decimal comma (default)")
    fmt.add_argument("--plain", dest="excel_eu", action="store_false",
                     help="',' separator and decimal point")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.campaign_folder:
        campaign_dir = Path(args.campaign_folder).expanduser().resolve()
    else:
        campaign_dir = find_root() / DEFAULT_CAMPAIGN
    if not campaign_dir.is_dir():
        raise SystemExit(f"campaign folder not found: {campaign_dir}")
    print(f"campaign: {campaign_dir}")
    if args.cutoff_hz <= 0:
        raise SystemExit("--cutoff-hz must be positive")
    if args.deadband_rad < 0:
        raise SystemExit("--deadband-rad cannot be negative")

    target = campaign_dir / RESULTS_NAME
    existing = read_existing(target)
    computed = scan_campaign(campaign_dir, args.cutoff_hz, args.deadband_rad)
    if not computed and not existing:
        print(f"no test folders found in {campaign_dir}")
        return 1

    rows, kept_manual = merge(existing, computed)
    new_ids = {r["test_id"] for r in computed} - set(existing)
    orphans = set(existing) - {r["test_id"] for r in computed}

    delimiter = ";" if args.excel_eu else ","
    written = write_rows(target, rows, delimiter, decimal_comma=args.excel_eu)

    settings = {
        "written": datetime.now().isoformat(timespec="seconds"),
        "campaign": str(campaign_dir),
        "cutoff_hz": args.cutoff_hz,
        "deadband_rad": args.deadband_rad,
        "filter_order": FILTER_ORDER,
        "csv_format": "excel-eu" if args.excel_eu else "plain",
        "separator": delimiter,
        "decimal": "," if args.excel_eu else ".",
        "n_tests": len(rows),
    }
    with open(campaign_dir / SETTINGS_NAME, "w", encoding="utf-8") as fh:
        json.dump(settings, fh, indent=2)
        fh.write("\n")

    filled = sum(1 for r in rows if str(r.get("success", "")).strip() != "")
    print(f"{written}")
    print(f"  {len(rows)} tests   {len(new_ids)} new   "
          f"{kept_manual} manual values kept   {filled} with a success verdict")
    if orphans:
        print(f"  {len(orphans)} row(s) kept for tests whose folder is gone: "
              f"{', '.join(sorted(orphans)[:3])}"
              + (" ..." if len(orphans) > 3 else ""))
    print(f"  fill in {', '.join(MANUAL_COLUMNS)} by hand; the export never "
          f"overwrites them")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
