#!/usr/bin/env python3
"""Aggregate analysis over the run subset chosen in run_selection.yaml.

Standalone script (NOT a colcon console_script): it imports
``f1tenth_logger.test_campaign.analyze_tests`` as a library (the package is
already installed/symlinked) and reuses its ``load_campaign``, ``Run``,
``METRIC_COLUMNS``, ``wilson``/``group_stats``/``build_summary``,
``overview_map`` and ``dashboard`` so the per-mission output is byte-for-byte
the same shape as ``test_campaign_analyze`` produced for M07.

For the 7 missions M01, M02, M03, M04, M05, M06, M08 (M07 is out of scope,
owned by CLEANUP/FINALIZE) it, for every repetition requested in
run_selection.yaml:

  * finds the matching folder(s) in the loaded campaign,
  * disambiguates duplicates by the date prefix rule in the config (M02 1-5),
  * marks missing / ambiguous / incomplete(corrupt) runs without guessing,
  * runs the M07 statistics/plots on the VALID subset only,

and writes per-mission ``report.txt`` / ``summary.csv`` / ``dashboard.png`` /
``overview_map.png`` / ``run_table.{csv,tsv}`` / ``mission_table.{csv,tsv}``
under ``first_test_campaign/analysis/<ShortID>/`` plus the combined
``summary_table.{csv,tsv}`` (7 mission rows) and ``run_table.{csv,tsv}``.

    python3 tools/campaign_aggregate.py [campaign_folder]
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import yaml

from f1tenth_logger.test_campaign import analyze_tests
from f1tenth_logger.test_campaign.robot_logger import find_root, DEFAULT_CAMPAIGN

# --------------------------------------------------------------------------
# Robustness shim: a crashed run can leave a *.csv with embedded NUL bytes
# (seen in M05/P005-R017, a run not even in the selection). The upstream
# csv.reader raises "_csv.Error: line contains NUL", which would abort
# load_campaign's whole-campaign scan before we ever reach the selection
# logic. We cannot edit the installed library, so wrap its module-level
# _read_csv_columns to strip NULs first. Behaviour is identical for clean
# files; a corrupt one yields whatever rows survive NUL removal (and such a
# run is flagged incomplete downstream anyway).
_orig_read_csv_columns = analyze_tests._read_csv_columns


def _nul_safe_read_csv_columns(path, columns):
    import csv as _csv
    import math as _math
    import numpy as _np
    from pathlib import Path as _Path

    path = _Path(path)
    if not path.exists():
        return {c: _np.empty(0) for c in columns}
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return {c: _np.empty(0) for c in columns}
    if "\x00" not in text:
        return _orig_read_csv_columns(path, columns)
    text = text.replace("\x00", "")
    out = {c: [] for c in columns}
    for row in _csv.DictReader(text.splitlines()):
        for col in columns:
            raw = (row.get(col) or "").strip()
            try:
                out[col].append(float(raw) if raw else _math.nan)
            except ValueError:
                out[col].append(_math.nan)
    return {c: _np.asarray(v, dtype=float) for c, v in out.items()}


analyze_tests._read_csv_columns = _nul_safe_read_csv_columns

# mission dir name -> short id used for the analysis subfolder / table rows
SHORT_ID = {
    "M01_forward_3m": "M01",
    "M02_approach_wall": "M02",
    "M03_bottle": "M03",
    "M04_person": "M04",
    "M05_turn_left_wall": "M05",
    "M06_turn_left_wall_then_person": "M06",
    "M08_turn_right_wall_then_person": "M08",
}
# M07 is deliberately absent: owned by CLEANUP/FINALIZE.

STATUS_VALID = "valid"
STATUS_MISSING = "missing"
STATUS_AMBIGUOUS = "ambiguous"
STATUS_INCOMPLETE = "incomplete"


def _date_prefix(run):
    """YYYYMMDD prefix embedded in the run folder name / test_id.

    Folder convention: P<nnn>-R<rrr>-<YYYYMMDDTHHMMSS>. The 3rd dash-field's
    leading 8 digits are the date.
    """
    name = run.path.name if run.path is not None else run.test_id
    parts = name.split("-")
    if len(parts) >= 3:
        stamp = parts[2]
        return stamp[:8]
    return ""


def _disambiguate_rule(mission_cfg, repetition):
    """Return the required date_prefix for this repetition, or None."""
    for rule in mission_cfg.get("disambiguate", []) or []:
        if repetition in (rule.get("repetition") or []):
            return rule.get("date_prefix")
    return None


def _is_incomplete(run):
    """Corrupt/incomplete per the card: aborted + (no meta.json OR empty kin)."""
    if run.auto_outcome != "aborted":
        return False
    if run.reason == "no meta.json":
        return True
    # kinematics.csv with 0 data rows -> no usable trajectory
    if run.t.size == 0:
        return True
    return False


def resolve_mission(mission_dir, mission_cfg, runs_by_mission):
    """Resolve every requested repetition of one mission.

    Returns ``(records, valid_runs)`` where ``records`` is a list of
    ``{repetition, status, reason, test_id, run}`` (run may be None) covering
    every requested repetition, and ``valid_runs`` the Run objects usable for
    statistics.
    """
    candidates = runs_by_mission.get(mission_dir, [])
    by_rep = {}
    for run in candidates:
        by_rep.setdefault(run.repetition, []).append(run)

    records, valid_runs = [], []
    for rep in mission_cfg.get("runs", []):
        matches = by_rep.get(rep, [])
        required_prefix = _disambiguate_rule(mission_cfg, rep)

        if required_prefix is not None and len(matches) > 1:
            filtered = [r for r in matches if _date_prefix(r) == required_prefix]
            if len(filtered) == 1:
                matches = filtered
            elif len(filtered) == 0:
                records.append({
                    "repetition": rep, "status": STATUS_AMBIGUOUS,
                    "reason": (f"{len(matches)} folders for repetition {rep}, "
                               f"none match date_prefix {required_prefix}"),
                    "test_id": "", "run": None,
                })
                continue
            else:
                matches = filtered  # still >1 -> handled below as ambiguous

        if len(matches) == 0:
            records.append({
                "repetition": rep, "status": STATUS_MISSING,
                "reason": f"nessuna cartella per repetition {rep}",
                "test_id": "", "run": None,
            })
            continue
        if len(matches) > 1:
            names = ", ".join(sorted(r.path.name for r in matches))
            records.append({
                "repetition": rep, "status": STATUS_AMBIGUOUS,
                "reason": f"{len(matches)} folders for repetition {rep}: {names}",
                "test_id": "", "run": None,
            })
            continue

        run = matches[0]
        if _is_incomplete(run):
            reason = run.reason or "aborted with no usable kinematics"
            records.append({
                "repetition": rep, "status": STATUS_INCOMPLETE,
                "reason": reason, "test_id": run.test_id, "run": run,
            })
            continue

        records.append({
            "repetition": rep, "status": STATUS_VALID,
            "reason": "", "test_id": run.test_id, "run": run,
        })
        valid_runs.append(run)

    return records, valid_runs


# --------------------------------------------------------------------------
# table writers
# --------------------------------------------------------------------------

RUN_TABLE_FIELDS = ["mission", "short_id", "repetition", "status", "reason",
                    "test_id", "auto_outcome", "manual_success"]


def _run_table_rows(mission_dir, short_id, records):
    rows = []
    for rec in records:
        run = rec["run"]
        rows.append({
            "mission": mission_dir,
            "short_id": short_id,
            "repetition": rec["repetition"],
            "status": rec["status"],
            "reason": rec["reason"],
            "test_id": rec["test_id"],
            "auto_outcome": run.auto_outcome if run is not None else "",
            "manual_success": ("" if run is None or run.manual_success is None
                               else run.manual_success),
        })
    return rows


def _write_table(path_no_ext, fieldnames, rows):
    """Write a .csv and a .tsv twin with identical content."""
    for ext, delim in ((".csv", ","), (".tsv", "\t")):
        p = path_no_ext.with_suffix(ext)
        with open(p, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames, delimiter=delim)
            writer.writeheader()
            for row in rows:
                writer.writerow({k: ("" if row.get(k) is None else row.get(k))
                                 for k in fieldnames})


def _summary_fieldnames(sample_row):
    # column order exactly as analyze_tests.build_summary emits it
    return list(sample_row.keys())


def _mission_table_row(mission_dir, short_id, mission_group_row,
                       n_requested, n_resolved, n_missing):
    """One row: the mission-level summary.csv row + bookkeeping columns."""
    row = {"mission": short_id}
    row.update(mission_group_row)
    # overwrite the inner "mission" (full dir) with the short id for clarity,
    # but keep a mission_dir column for traceability
    row["mission_dir"] = mission_dir
    row["n_requested"] = n_requested
    row["n_resolved"] = n_resolved
    row["n_missing"] = n_missing
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign_folder", nargs="?", default=None)
    args = parser.parse_args(argv)

    if args.campaign_folder:
        campaign_dir = Path(args.campaign_folder).expanduser().resolve()
    else:
        campaign_dir = find_root() / DEFAULT_CAMPAIGN
    if not campaign_dir.is_dir():
        raise NotADirectoryError(f"campaign folder not found: {campaign_dir}")

    analysis_dir = campaign_dir / "analysis"
    config_path = analysis_dir / "run_selection.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    print(f"campaign: {campaign_dir}")
    runs, has_manual, unreadable = analyze_tests.load_campaign(campaign_dir)

    runs_by_mission = {}
    for run in runs:
        runs_by_mission.setdefault(run.mission, []).append(run)

    # combined accumulators
    combined_mission_rows = []
    combined_run_rows = []
    mission_field_order = None  # settled from first mission_table row

    # process missions in short-id order (M01..M06, M08), M07 excluded
    for mission_dir in sorted(SHORT_ID, key=lambda m: SHORT_ID[m]):
        short_id = SHORT_ID[mission_dir]
        mission_cfg = config.get(mission_dir)
        if mission_cfg is None:
            print(f"WARNING: {mission_dir} absent from run_selection.yaml, skipped")
            continue

        records, valid_runs = resolve_mission(mission_dir, mission_cfg,
                                              runs_by_mission)
        n_requested = len(records)
        n_valid = len(valid_runs)
        n_missing = sum(1 for r in records if r["status"] == STATUS_MISSING)
        n_ambiguous = sum(1 for r in records if r["status"] == STATUS_AMBIGUOUS)
        n_incomplete = sum(1 for r in records if r["status"] == STATUS_INCOMPLETE)
        # "resolved" = a unique folder was identified (valid or incomplete)
        n_resolved = n_requested - n_missing - n_ambiguous

        print(f"\n{short_id} ({mission_dir}): requested {n_requested}, "
              f"valid {n_valid}, incomplete {n_incomplete}, "
              f"ambiguous {n_ambiguous}, missing {n_missing} "
              f"-> resolved {n_resolved}")

        out_dir = analysis_dir / short_id
        out_dir.mkdir(parents=True, exist_ok=True)

        # --- per-mission summary.csv / report.txt on the VALID subset -----
        rows = analyze_tests.build_summary(valid_runs) if valid_runs else []
        report = analyze_tests.format_report(
            campaign_dir, valid_runs, rows, has_manual, unreadable)
        (out_dir / "report.txt").write_text(report + "\n", encoding="utf-8")

        if rows:
            summary_fields = _summary_fieldnames(rows[0])
            with open(out_dir / "summary.csv", "w", newline="",
                      encoding="utf-8") as fh:
                w = csv.DictWriter(fh, fieldnames=summary_fields)
                w.writeheader()
                for row in rows:
                    w.writerow({k: ("" if v is None else v)
                                for k, v in row.items()})
            mission_group_row = next(r for r in rows if r["level"] == "mission")
        else:
            # no valid runs: emit an empty-shaped mission row for the tables
            mission_group_row = analyze_tests.group_stats(
                "mission", mission_dir, "", [])
            summary_fields = _summary_fieldnames(mission_group_row)
            with open(out_dir / "summary.csv", "w", newline="",
                      encoding="utf-8") as fh:
                w = csv.DictWriter(fh, fieldnames=summary_fields)
                w.writeheader()
                w.writerow({k: ("" if v is None else v)
                            for k, v in mission_group_row.items()})

        # --- plots on the VALID subset ------------------------------------
        analyze_tests.overview_map(valid_runs, mission_dir,
                                   out_dir / "overview_map.png")
        analyze_tests.dashboard(valid_runs, out_dir / "dashboard.png")

        # --- per-mission run_table (every requested repetition) -----------
        run_rows = _run_table_rows(mission_dir, short_id, records)
        _write_table(out_dir / "run_table", RUN_TABLE_FIELDS, run_rows)
        combined_run_rows.extend(run_rows)

        # --- per-mission mission_table (one row) --------------------------
        mission_row = _mission_table_row(
            mission_dir, short_id, mission_group_row,
            n_requested, n_resolved, n_missing)
        if mission_field_order is None:
            # mission, <summary cols...>, mission_dir, n_requested, n_resolved,
            # n_missing  (keep summary cols in their canonical order)
            summary_cols = [c for c in mission_row
                            if c not in ("mission", "mission_dir",
                                         "n_requested", "n_resolved",
                                         "n_missing")]
            mission_field_order = (["mission"] + summary_cols +
                                   ["mission_dir", "n_requested",
                                    "n_resolved", "n_missing"])
        _write_table(out_dir / "mission_table", mission_field_order,
                     [mission_row])
        combined_mission_rows.append(mission_row)

    # --- combined tables (7 missions, M07 excluded) -----------------------
    _write_table(analysis_dir / "summary_table", mission_field_order,
                 combined_mission_rows)
    _write_table(analysis_dir / "run_table", RUN_TABLE_FIELDS,
                 combined_run_rows)

    print(f"\ncombined summary_table rows: {len(combined_mission_rows)} "
          f"(expected 7)")
    print(f"combined run_table rows: {len(combined_run_rows)}")
    print(f"written under: {analysis_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
