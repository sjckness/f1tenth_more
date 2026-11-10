#!/usr/bin/env python3
"""Doors campaign: one CSV row per test_campaign_logger run, analysis_v2 metrics.

Usage:
  batch:  python3 doors_csv.py [INPUT_DIR] [-o OUT.csv] [--selection selection.yaml]
          (INPUT_DIR default /scratch/fabiocar/f1tenth_container/archive/doors_campaign,
           every <INPUT_DIR>/<mission>/P###-R###-<time>/ folder; OUT default
           <INPUT_DIR>/doors_results.csv)
  single: python3 doors_csv.py <INPUT_DIR>/<mission>/P001-R003-20261110T101500 -o one.csv

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
* Rep: the R### of the test id. Every other value comes from
  scripts.metrics.compute_run_metrics (RULES.md §6-7), unchanged.
* A run that fails to load or compute keeps its row (obstacle/Rep only) and is
  reported once on stderr; the batch goes on.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from scripts.metrics import compute_run_metrics, kleene_and  # noqa: E402
from scripts.run_data import load_run_v2, parse_test_id  # noqa: E402

DEFAULT_INPUT = "/scratch/fabiocar/f1tenth_container/archive/doors_campaign"
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


def row_for(run_dir, obstacles):
    test_id = run_dir.name
    obstacle = obstacle_for(test_id, obstacles)
    row = {k: None for k in HEADER}
    row["obstacle"] = None if obstacle is None else ("yes" if obstacle else "no")
    row["Rep"] = parse_test_id(test_id)["repetition"]
    error = None
    try:
        m = compute_run_metrics(load_run_v2(run_dir), ostacolo=obstacle, verdicts={})
        for key in HEADER[2:]:
            if key not in MANUAL:
                row[key] = m.get(key)
        row["success"] = kleene_and(None, row["exec_ok"], row["safe"], None)
    except Exception as exc:  # noqa: BLE001 - one bad run must not stop the batch
        error = f"{type(exc).__name__}: {exc}"
    return row, obstacle is None, error


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("input", nargs="?", default=DEFAULT_INPUT,
                    help="campaign folder (batch) or one run folder")
    ap.add_argument("-o", "--output", default=None, help="CSV to write")
    ap.add_argument("--selection", default=None,
                    help="selection.yaml with the obstacle flags")
    args = ap.parse_args(argv)

    inp = Path(args.input)
    runs = find_runs(inp)
    campaign = inp.parent.parent if _is_run_dir(inp) else inp
    selection = Path(args.selection) if args.selection else campaign / "selection.yaml"
    out_path = Path(args.output) if args.output else campaign / "doors_results.csv"
    obstacles = load_obstacles(selection)

    no_obstacle, errors = [], []
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(HEADER)
        for run_dir in runs:
            row, missing, error = row_for(run_dir, obstacles)
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
