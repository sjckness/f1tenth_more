#!/usr/bin/env python3
"""Doors campaign: one CSV row per test_campaign_logger run, analysis_v2 metrics.

Usage:
  batch:  python3 doors_csv.py [INPUT_DIR] [-o OUT.csv] [--selection selection.yaml]
          (INPUT_DIR default /scratch/fabiocar/f1tenth_container/archive/doors_campaign,
           every <INPUT_DIR>/<mission>/P###-R###-<time>/ folder; OUT default
           <INPUT_DIR>/doors_results.csv)
  single: python3 doors_csv.py <INPUT_DIR>/<mission>/P001-R003-20261110T101500 -o one.csv
  pick:   python3 doors_csv.py RUN_DIR RUN_DIR ... -o some.csv   (rows in the order given)

Columns, in this order: obstacle, Rep, success, plan_ok, exec_ok, safe, task_ok,
goal_error_m, lateral_error_max_m, viol_fixed_pct, min_clear_any_m,
min_clear_after_start_m, solver_usable_pct, unusable_streak_max_s, jerk_rms_mps3,
steer_direction_rev_per_m.

* plan_ok, task_ok, goal_error_m are always EMPTY: filled by hand.
* success = Kleene AND(plan_ok, exec_ok, safe, task_ok) as in RULES.md §5, with
  plan_ok and task_ok empty: "false" if exec_ok or safe is false, else EMPTY.
* obstacle: yes/no from selection.yaml (default <INPUT_DIR>/selection.yaml),
  analysis_v2 row format ``- [P001-R003, si|no, ...]`` (yes/no/true/false also
  accepted, a full test id or its P###-R### prefix). No entry: empty, and one
  warning listing every such run.
* No LLM plan (JSON missions): the mission file itself is the plan,
  <MISSIONS_DIR>/<plan_id>.json (default src/f1tenth_behavior/missions of this
  repo, --missions-dir). It segments the moves, so exec_ok, viol_fixed_pct and
  lateral_error_max_m (straight moves only, RULES.md §7) are computed.
* --drop-reset-origin: the local odometry sits at exactly (0, 0) (its reset
  value) until the first estimate after the start. When that first estimate is
  not next to (0, 0), RULES.md §4's pose check sees a jump and nulls every
  pose-based value of the run. With this flag the leading exact-(0, 0) rows of
  kinematics.csv are left out (in a temp copy; the run folder is never touched).
* Rep: the R### of the test id. Every other value comes from
  scripts.metrics.compute_run_metrics (RULES.md §6-7), unchanged.
* A run that fails to load or compute keeps its row (obstacle/Rep only) and is
  reported once on stderr; the batch goes on.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import tempfile
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from scripts.metrics import compute_run_metrics, kleene_and  # noqa: E402
from scripts.run_data import load_run_v2, parse_test_id  # noqa: E402

DEFAULT_INPUT = "/scratch/fabiocar/f1tenth_container/archive/doors_campaign"
DEFAULT_MISSIONS = HERE.parent.parent / "src" / "f1tenth_behavior" / "missions"
HEADER = [
    "obstacle", "Rep", "success", "plan_ok", "exec_ok", "safe", "task_ok",
    "goal_error_m", "lateral_error_max_m", "viol_fixed_pct", "min_clear_any_m",
    "min_clear_after_start_m", "solver_usable_pct", "unusable_streak_max_s",
    "jerk_rms_mps3", "steer_direction_rev_per_m",
]
MANUAL = ("plan_ok", "task_ok", "goal_error_m")
_YES = {"si", "sì", "yes", "y", "true", "1"}
_NO = {"no", "n", "false", "0"}


def _is_run_dir(path):
    try:
        parse_test_id(path.name)
    except ValueError:
        return False
    return path.is_dir()


def find_runs(path):
    path = Path(path)
    if _is_run_dir(path):
        return [path]
    return sorted((p for p in path.glob("*/*") if _is_run_dir(p)),
                  key=lambda p: (p.parent.name, p.name))


def load_obstacles(path):
    """{run code: True/False} from a selection.yaml; {} when the file is absent."""
    path = Path(path)
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    rows = data.get("runs", []) if isinstance(data, dict) else data
    out = {}
    for row in rows or []:
        if isinstance(row, (list, tuple)) and len(row) >= 2:
            code, flag = str(row[0]), str(row[1]).strip().lower()
        elif isinstance(row, dict) and len(row) == 1:
            (code, flag), = row.items()
            code, flag = str(code), str(flag).strip().lower()
        else:
            continue
        if flag in _YES or flag in ("true",):
            out[code] = True
        elif flag in _NO or flag in ("false",):
            out[code] = False
    return out


def obstacle_for(test_id, obstacles):
    for code, flag in obstacles.items():
        if test_id == code or test_id.startswith(code + "-"):
            return flag
    return None


def _cell(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return "" if not math.isfinite(value) else f"{value:.6g}"
    return str(value)


def mission_plan(run, missions_dir):
    """The mission JSON a run executed, when it has no LLM plan; else None."""
    if isinstance(run.plan, dict) and run.plan.get("moves"):
        return None
    meta = run.meta or {}
    plan_id = (meta.get("extra_meta") or {}).get("plan_id") or meta.get("plan_id")
    path = Path(missions_dir) / f"{plan_id}.json" if plan_id else None
    if path is None or not path.is_file():
        return None
    with open(path, encoding="utf-8") as fh:
        plan = json.load(fh)
    return plan if isinstance(plan, dict) and plan.get("moves") else None


def without_reset_origin(run_dir, tmp_root):
    """Temp copy of run_dir (symlinks) whose kinematics.csv drops the leading
    exact-(0, 0) rows. Returns (path, n_dropped); the original when none."""
    src = Path(run_dir) / "kinematics.csv"
    with open(src, newline="", encoding="utf-8") as fh:
        rows = list(csv.reader(fh))
    if len(rows) < 2:
        return Path(run_dir), 0
    head, body = rows[0], rows[1:]
    ix, iy = head.index("x"), head.index("y")
    n = 0
    while n < len(body) and body[n][ix] not in ("", None) and body[n][iy] not in ("", None) \
            and float(body[n][ix]) == 0.0 and float(body[n][iy]) == 0.0:
        n += 1
    if n == 0 or n == len(body):
        return Path(run_dir), 0
    dst = Path(tmp_root) / Path(run_dir).parent.name / Path(run_dir).name
    dst.mkdir(parents=True)
    for f in Path(run_dir).iterdir():
        if f.name != "kinematics.csv":
            os.symlink(f.resolve(), dst / f.name)
    with open(dst / "kinematics.csv", "w", newline="", encoding="utf-8") as fh:
        csv.writer(fh).writerows([head] + body[n:])
    return dst, n


def row_for(run_dir, obstacles, missions_dir=DEFAULT_MISSIONS, tmp_root=None):
    test_id = run_dir.name
    obstacle = obstacle_for(test_id, obstacles)
    row = {k: None for k in HEADER}
    row["obstacle"] = None if obstacle is None else ("yes" if obstacle else "no")
    row["Rep"] = parse_test_id(test_id)["repetition"]
    error = None
    try:
        src = run_dir
        if tmp_root is not None:
            src, _ = without_reset_origin(run_dir, tmp_root)
        run = load_run_v2(src)
        plan = mission_plan(run, missions_dir)
        if plan is not None:
            run.plan = plan
        m = compute_run_metrics(run, ostacolo=obstacle, verdicts={})
        for key in HEADER[2:]:
            if key not in MANUAL:
                row[key] = m.get(key)
        row["success"] = kleene_and(None, row["exec_ok"], row["safe"], None)
    except Exception as exc:  # noqa: BLE001 - one bad run must not stop the batch
        error = f"{type(exc).__name__}: {exc}"
    return row, obstacle is None, error


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("input", nargs="*", default=[DEFAULT_INPUT],
                    help="campaign folder (batch), or one or more run folders")
    ap.add_argument("-o", "--output", default=None, help="CSV to write")
    ap.add_argument("--selection", default=None,
                    help="selection.yaml with the obstacle flags")
    ap.add_argument("--drop-reset-origin", action="store_true",
                    help="leave out the leading exact-(0, 0) odometry rows before the pose check")
    ap.add_argument("--missions-dir", default=str(DEFAULT_MISSIONS),
                    help="mission JSONs used as the plan of a run with no LLM plan")
    args = ap.parse_args(argv)

    inputs = [Path(p) for p in args.input]
    runs = [r for p in inputs for r in find_runs(p)]
    inp = inputs[0]
    campaign = inp.parent.parent if _is_run_dir(inp) else inp
    selection = Path(args.selection) if args.selection else campaign / "selection.yaml"
    out_path = Path(args.output) if args.output else campaign / "doors_results.csv"
    obstacles = load_obstacles(selection)

    no_obstacle, errors = [], []
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = tempfile.TemporaryDirectory(prefix="doors_csv_") if args.drop_reset_origin else None
    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(HEADER)
        for run_dir in runs:
            row, missing, error = row_for(run_dir, obstacles, args.missions_dir,
                                          tmp.name if tmp else None)
            writer.writerow([_cell(row[k]) for k in HEADER])
            if missing:
                no_obstacle.append(run_dir.name)
            if error:
                errors.append(f"{run_dir.name}: {error}")

    if no_obstacle:
        print(f"WARNING: no obstacle flag in {selection} for {len(no_obstacle)} run(s), "
              f"left empty: {', '.join(no_obstacle)}", file=sys.stderr)
    for err in errors:
        print(f"ERROR (row kept, metrics empty) {err}", file=sys.stderr)
    print(f"{len(runs)} run(s) -> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
