#!/usr/bin/env python3
"""Export campaign runs into the MATLAB database.

    ros2 run f1tenth_logger export_matlab [campaign_folder]
        [--db-root DIR] [--archive DIR] [--mission M04] [--runs "14-15-16"]
        [--force] [--scan-decimate K]

Per test: ``<db_root>/runs/<test_id>.mat`` (MATLAB v5, compressed). The test
folder is the primary source and is always exported in full; the archive bag
of the same mission is added for the stretch it covers, when exactly one
archive run started within 2 s of the test's ``mission_started``. Every run
also lands in ``<db_root>/index/runs.csv`` (the campaign_results.csv columns
plus bag columns, plain CSV, '.' decimals, empty = missing).

Incremental: a run whose .mat exists with the same ``meta.exporter_version``
is skipped; ``--force`` re-exports. A failing run is reported and the batch
goes on. Nothing in the campaign folder or the archive is ever written.
"""

import argparse
import csv
import math
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import scipy.io

from f1tenth_logger.matlab_export import EXPORTER_VERSION
from f1tenth_logger.matlab_export import bag as bagmod
from f1tenth_logger.matlab_export.campaign import load_campaign_topics, load_meta, timing_for
from f1tenth_logger.matlab_export.convert import mat_name, str_cell
from f1tenth_logger.matlab_export.db_root import (
    YAML_RELPATH, atomic_write_bytes, get_db_root)
from f1tenth_logger.test_campaign import export_campaign_csv as campaign_csv
from f1tenth_logger.test_campaign.robot_logger import (
    DEFAULT_CAMPAIGN, find_root, parse_test_id)
from f1tenth_logger.test_campaign.run_select import (
    RunSelectionError, select_runs, short_id)

DEFAULT_ARCHIVE = "~/f1tenth_archive"
ARCHIVE_STATES = ("complete", "active", "incomplete")
MATCH_TOL_S = 2.0
INDEX_EXTRA = ["has_bag", "bag_coverage_pct", "bag_t_start_rel", "bag_t_end_rel",
               "archive_run_id", "bag_status", "bag_match_dt_s", "mat_file"]


# --------------------------------------------------------------------------
# the campaign and the archive
# --------------------------------------------------------------------------

def campaign_tests(campaign_dir):
    """``[(test_dir, mission)]`` for every test folder, sorted by mission, id."""
    tests = []
    for mission_dir in sorted(p for p in campaign_dir.iterdir() if p.is_dir()):
        for test_dir in sorted(p for p in mission_dir.iterdir() if p.is_dir()):
            try:
                parse_test_id(test_dir.name)
            except ValueError:
                continue
            tests.append((test_dir, mission_dir.name))
    return tests


def archive_runs(archive_dir):
    """``[{run_id, start, bag_dir}]`` from every manifest in the archive."""
    import json

    runs = []
    for state in ARCHIVE_STATES:
        for manifest in sorted((archive_dir / state).glob("*/*.manifest.json")):
            try:
                data = json.loads(manifest.read_text(encoding="utf-8"))
                start = datetime.fromisoformat(data["start_time"]).timestamp()
            except (OSError, ValueError, KeyError, TypeError):
                continue
            runs.append({"run_id": data.get("run_id") or manifest.parent.name,
                         "start": start, "bag_dir": manifest.parent / "bag", "state": state})
    return runs


def match_bag(timing, archive, tol=MATCH_TOL_S):
    """The archive run of this test's mission, or why there is none.

    Returns a dict with ``status`` (attached / empty_bag / bag_unreadable /
    no_match / ambiguous), the run id and offset when one was found, and the
    bag's receive-time window relative to the drive start.
    """
    start = timing["drive_start_ros"]
    near = sorted(archive, key=lambda r: abs(r["start"] - start))
    result = {"status": "no_match", "archive_run_id": "", "bag_dir": None,
              "dt": math.nan, "t_start_rel": math.nan, "t_end_rel": math.nan,
              "coverage_pct": math.nan, "count": 0}
    if not near:
        return result
    result["dt"] = near[0]["start"] - start
    hits = [r for r in near if abs(r["start"] - start) <= tol]
    if not hits:
        return result
    if len(hits) > 1:
        result["status"] = "ambiguous"
        result["archive_run_id"] = " ".join(r["run_id"] for r in hits)
        return result
    hit = hits[0]
    result.update(archive_run_id=hit["run_id"], bag_dir=hit["bag_dir"])
    try:
        b_start, b_end, count = bagmod.bag_window(hit["bag_dir"])
    except (OSError, ValueError, KeyError, TypeError):
        result["status"] = "bag_unreadable"
        return result
    result["count"] = count
    if count == 0:
        result.update(status="empty_bag", coverage_pct=0.0)
        return result
    t0, t1 = b_start - start, b_end - start
    drive_end = timing["drive_end_rel"]
    cover = math.nan
    if drive_end > 0:
        cover = 100.0 * max(0.0, min(t1, drive_end) - max(t0, 0.0)) / drive_end
    result.update(status="attached", t_start_rel=t0, t_end_rel=t1, coverage_pct=cover)
    return result


# --------------------------------------------------------------------------
# one run
# --------------------------------------------------------------------------

def git_commit(root):
    try:
        head = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                              capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(root), "status", "--porcelain",
                                "--untracked-files=no"],
                               capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "", math.nan
    return head, float(bool(dirty))


def topic_table(rows):
    """List of {name, type, count, struct, source} -> struct of columns."""
    return {
        "name": str_cell([r["name"] for r in rows]),
        "type": str_cell([r["type"] for r in rows]),
        "count": np.array([float(r["count"]) for r in rows]).reshape(-1, 1),
        "struct_name": str_cell([r["struct"] for r in rows]),
        "source": str_cell([r["source"] for r in rows]),
    }


def build_run(test_dir, mission, archive, typestore, repo, scan_decimate):
    """Everything that goes into one .mat, plus the match result."""
    import json

    topics, timing, test_meta = load_campaign_topics(test_dir)
    match = match_bag(timing, archive)
    rows = [{"name": t["topic"], "type": "campaign file", "count": len(t["t_ros"]),
             "struct": name, "source": "campaign"} for name, t in topics.items()]
    errors = []
    if match["status"] == "attached":
        btopics, btable, errors = bagmod.load_bag_topics(
            match["bag_dir"], timing, typestore, scan_decimate)
        for name, topic in btopics.items():
            key = name if name not in topics else f"bag_{name}"
            topics[key] = topic
            for row in btable:
                if row["struct"] == name:
                    row["struct"] = key
        rows += [dict(r, source="bag") for r in btable]
    head, dirty = repo
    summary = test_meta.get("summary") or {}
    meta = {
        "run_id": test_dir.name,
        "run_short": short_id(test_dir.name),
        "mission": mission,
        "prompt_num": float(summary.get("prompt_num", math.nan) or math.nan),
        "repetition": float(summary.get("repetition", math.nan) or math.nan),
        "prompt_text": str(test_meta.get("prompt_text") or ""),
        "auto_outcome": str(summary.get("auto_outcome") or ""),
        "test_dir": str(test_dir),
        "ros_start_time": timing["ros_start"],
        "t_drive_start_ros": timing["drive_start_ros"],
        "t_drive_start_source": timing["drive_start_source"],
        "t_drive_end_rel": timing["drive_end_rel"],
        "bag_status": match["status"],
        "archive_run_id": match["archive_run_id"],
        "bag_path": str(match["bag_dir"] or ""),
        "bag_match_dt_s": match["dt"],
        "bag_coverage_pct": match["coverage_pct"],
        "bag_t_start_rel": match["t_start_rel"],
        "bag_t_end_rel": match["t_end_rel"],
        "has_bag": float(match["status"] == "attached"),
        "scan_decimate": float(scan_decimate),
        "topics": topic_table(rows),
        "topic_errors": str_cell(errors),
        "export_time": datetime.now().isoformat(timespec="seconds"),
        "git_commit": head,
        "git_dirty": dirty,
        "exporter_version": EXPORTER_VERSION,
        "campaign_meta_json": json.dumps(test_meta),
    }
    out = {mat_name(k): v for k, v in topics.items()}
    out["meta"] = meta
    return out, match, errors


def existing_version(path):
    try:
        data = scipy.io.loadmat(str(path), variable_names=["meta"], squeeze_me=True,
                                struct_as_record=False)
        return str(data["meta"].exporter_version)
    except Exception:  # noqa: BLE001 - unreadable or old file: re-export it
        return None


def write_mat(path, data):
    def write(fh):
        scipy.io.savemat(fh, data, do_compression=True, oned_as="column",
                         long_field_names=True)
    return atomic_write_bytes(path, write)


# --------------------------------------------------------------------------
# the index
# --------------------------------------------------------------------------

def _fmt(value):
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return ""
    return campaign_csv.format_value(value, decimal_comma=False)


def write_index(db_root, campaign_dir, tests, matches):
    """``index/runs.csv`` over every campaign test, filtered or not."""
    existing = campaign_csv.read_existing(campaign_dir / campaign_csv.RESULTS_NAME)
    computed = [campaign_csv.metrics_for_test(d, m, campaign_csv.DEFAULT_CUTOFF_HZ,
                                              campaign_csv.DEFAULT_DEADBAND_RAD)
                for d, m in tests]
    backfill = campaign_csv.read_backfill(campaign_dir)
    for row in computed:
        campaign_csv.apply_backfill(row, backfill.get(row["test_id"], {}))
    known = {r["test_id"] for r in computed}
    rows, _ = campaign_csv.merge({k: v for k, v in existing.items() if k in known}, computed)
    columns = campaign_csv.COLUMNS + INDEX_EXTRA
    for row in rows:
        match = matches.get(row["test_id"])
        mat = db_root / "runs" / f"{row['test_id']}.mat"
        if match is not None:
            row.update(
                has_bag=int(match["status"] == "attached"),
                bag_coverage_pct=match["coverage_pct"],
                bag_t_start_rel=match["t_start_rel"],
                bag_t_end_rel=match["t_end_rel"],
                archive_run_id=match["archive_run_id"],
                bag_status=match["status"],
                bag_match_dt_s=match["dt"],
            )
        row["mat_file"] = f"runs/{mat.name}" if mat.exists() else ""
    path = db_root / "index" / "runs.csv"

    def write(fh):
        import io
        text = io.StringIO()
        writer = csv.writer(text, lineterminator="\n")
        writer.writerow(columns)
        for row in rows:
            writer.writerow([_fmt(row.get(col)) for col in columns])
        fh.write(text.getvalue().encode("utf-8"))
    atomic_write_bytes(path, write)
    return path, len(rows)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Export campaign runs (+ archive bags) into the MATLAB database.")
    parser.add_argument("campaign_folder", nargs="?", default=None,
                        help=f"campaign folder (default: <f1tenth_more>/{DEFAULT_CAMPAIGN})")
    parser.add_argument("--db-root", "--out", dest="db_root", default=None,
                        help="database root (default: $F1TENTH_MATLAB_DATA, then the "
                             "logger YAML's matlab_export.db_root, then ~/matlab_data)")
    parser.add_argument("--archive", default=DEFAULT_ARCHIVE,
                        help=f"mission run archive (default {DEFAULT_ARCHIVE})")
    parser.add_argument("--mission", default=None,
                        help="only this mission folder, or its prefix (M04)")
    parser.add_argument("--runs", default=None,
                        help='only these runs, e.g. "1-3-4-67-89" or "P004-R016"')
    parser.add_argument("--force", action="store_true",
                        help="re-export runs whose .mat is already current")
    parser.add_argument("--scan-decimate", type=int, default=1, metavar="K",
                        help="keep every K-th LaserScan (default 1: all)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.scan_decimate < 1:
        raise SystemExit("--scan-decimate must be >= 1")
    root = find_root()
    campaign_dir = (Path(args.campaign_folder).expanduser().resolve()
                    if args.campaign_folder else root / DEFAULT_CAMPAIGN)
    if not campaign_dir.is_dir():
        raise SystemExit(f"campaign folder not found: {campaign_dir}")
    db_root, how = get_db_root(args.db_root, yaml_path=root / YAML_RELPATH)
    archive_dir = Path(args.archive).expanduser()
    print(f"MATLAB db root: {db_root}   (from {how})")
    print(f"campaign:       {campaign_dir}")
    print(f"archive:        {archive_dir}")

    all_tests = campaign_tests(campaign_dir)
    tests = all_tests
    if args.mission:
        tests = [(d, m) for d, m in tests
                 if m == args.mission or m.split("_")[0] == args.mission]
        if not tests:
            missions = sorted({m for _, m in all_tests})
            raise SystemExit(f"no mission {args.mission!r}; available: {', '.join(missions)}")
    if args.runs:
        by_id = {d.name: (d, m) for d, m in tests}
        try:
            tests = [by_id[t] for t in select_runs(args.runs, list(by_id))]
        except RunSelectionError as exc:
            raise SystemExit(str(exc))

    archive = archive_runs(archive_dir)
    typestore, msg_sources = bagmod.make_typestore(root)
    print(f"message types:  {', '.join(f'{k} ({v})' for k, v in msg_sources.items())}")
    repo = git_commit(root)

    matches = {}
    for test_dir, _ in all_tests:  # the index needs every run's bag status
        try:
            matches[test_dir.name] = match_bag(timing_for(test_dir, load_meta(test_dir)),
                                               archive)
        except Exception:  # noqa: BLE001 - reported by the export loop if selected
            pass

    exported, skipped, failed, sizes = [], [], [], {}
    print(f"\n{len(tests)} run(s) selected\n")
    for test_dir, mission in tests:
        path = db_root / "runs" / f"{test_dir.name}.mat"
        if not args.force and path.exists() and existing_version(path) == EXPORTER_VERSION:
            skipped.append(test_dir.name)
            sizes[test_dir.name] = path.stat().st_size
            print(f"  skip    {test_dir.name}  (current, {path.stat().st_size / 1e6:.2f} MB)")
            continue
        try:
            data, match, errors = build_run(test_dir, mission, archive, typestore, repo,
                                            args.scan_decimate)
            write_mat(path, data)
        except Exception as exc:  # noqa: BLE001 - one bad run must not stop the batch
            failed.append((test_dir.name, f"{type(exc).__name__}: {exc}"))
            print(f"  FAILED  {test_dir.name}  {type(exc).__name__}: {exc}")
            continue
        matches[test_dir.name] = match
        exported.append(test_dir.name)
        sizes[test_dir.name] = path.stat().st_size
        bag = match["status"]
        if bag == "attached":
            bag += f" {match['coverage_pct']:.0f}%"
        note = f"  topic errors: {'; '.join(errors)}" if errors else ""
        print(f"  export  {test_dir.name}  {sizes[test_dir.name] / 1e6:6.2f} MB  bag: {bag}{note}")

    index_path, n_index = write_index(db_root, campaign_dir, all_tests, matches)

    selected = [d.name for d, _ in tests]
    attached = [t for t in selected if matches.get(t, {}).get("status") == "attached"]
    not_attached = [(t, matches.get(t)) for t in selected
                    if matches.get(t, {}).get("status") != "attached"]
    print("")
    print(f"exported {len(exported)}   skipped {len(skipped)}   failed {len(failed)}")
    print(f"total size of the selected runs: {sum(sizes.values()) / 1e6:.1f} MB")
    print(f"bag attached: {len(attached)} of {len(selected)}")
    for test_id, match in not_attached:
        if match is None:
            print(f"  {test_id}: no timing (see failure above)")
            continue
        extra = ""
        if match["status"] in ("no_match", "ambiguous") and not math.isnan(match["dt"]):
            extra = f" (nearest archive start {match['dt']:+.1f} s)"
        if match["archive_run_id"]:
            extra += f" {match['archive_run_id']}"
        print(f"  {test_id}: {match['status']}{extra}")
    for test_id, error in failed:
        print(f"  FAILED {test_id}: {error}")
    print(f"index: {index_path} ({n_index} runs)")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
