#!/usr/bin/env python3
"""TESTS node: RULES.md §10 checks + the two stop-condition tests + the
3-run end-to-end smoke test described in ``.sjck/cards/TESTS.md``.

Every check below is a plain function (importable, e.g.
``from scripts.verify_checks import check1_imu_conversion``) and also
reachable from the CLI (``python3 -m scripts.verify_checks <subcommand>``,
or just run this file directly). Nothing here ever reports a default
"pass": every check returns an explicit ``status`` of ``"pass"``,
``"fail"`` or ``"insufficient_data"`` plus the numbers it looked at, so a
caller can see *why*.

Read-only w.r.t. the real pipeline: this module never writes into
``selection.yaml``, a run folder, ``analysis/`` or ``campaign_results.csv``.
Its own scratch files (the temporary selection and the end-to-end test
output) live under ``analysis_v2/selection.yaml.test_tmp`` and
``analysis_v2/_test/`` only.
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent  # .../analysis_v2
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

from scripts.run_data import (  # noqa: E402
    DEFAULT_CAMPAIGN,
    AmbiguousRunError,
    RunNotFoundError,
    find_root,
    load_run_v2,
    resolve_run,
)
from scripts import metrics  # noqa: E402
from scripts import tables  # noqa: E402
import run_analysis  # noqa: E402  (HERE/run_analysis.py, the entrypoint we stop-test)

G_TARGET_MPS2 = 9.80665  # RULES §7/§10.1: same constant used to convert IMU g -> m/s^2

#: The 3 runs used for the end-to-end smoke test (card: "Run di prova").
#: Full IDs confirmed to exist locally (``find`` on 2026-11-03).
E2E_RUN_IDS = (
    "P001-R009-20260930T173424",
    "P003-R005-20260928T165545",
    "P004-R015-20261001T112447",
)

#: Confirmed exact folder names for the P002-R001 ambiguity (card: "verifica
#: il nome esatto della seconda cartella prima di scrivere il test").
P002_R001_FOLDERS = (
    "P002-R001-20260922T142524",
    "P002-R001-20261008T132715",
)

#: old_vs_new.csv 'comparability' values meaning "same definition, must match"
#: (tables.OLD_VS_NEW_SPECS: "same" and "same_unit_converted").
SAME_DEFINITION_KINDS = {"same", "same_unit_converted"}


# --------------------------------------------------------------------------
# Check 1 -- IMU conversion (RULES §10.1)
# --------------------------------------------------------------------------

def check1_imu_conversion(test_ids, root=None, tol_mps2=0.5):
    """``az`` during a still segment, after x9.80665, should be ~9.81 m/s^2.

    "Still segment" = IMU samples before ``drive_window`` starts (every run
    has a short stationary period before ``mission_started``); falls back
    to the first second of IMU data if there is no drive_window at all.
    """
    root = root or find_root()
    campaign_dir = root / DEFAULT_CAMPAIGN
    out = []
    for code in test_ids:
        rec = {"test_id": code}
        try:
            run_dir = resolve_run(code, [campaign_dir])
            run = load_run_v2(run_dir)
        except Exception as exc:  # noqa: BLE001
            rec.update(status="insufficient_data", detail=f"could not load run: {exc!r}")
            out.append(rec)
            continue

        t = run.imu.get("t")
        az = run.imu.get("az")
        if t is None or az is None or len(t) == 0:
            rec.update(status="insufficient_data", detail="no imu samples")
            out.append(rec)
            continue

        t0 = run.drive_window[0] if run.drive_window else float(np.min(t)) + 1.0
        still = t < t0
        if not np.any(still):
            rec.update(status="insufficient_data",
                        detail="no IMU samples before drive_window start")
            out.append(rec)
            continue

        az_mean_g = float(np.mean(az[still]))
        az_mps2 = az_mean_g * G_TARGET_MPS2
        ok = abs(az_mps2 - G_TARGET_MPS2) <= tol_mps2
        rec.update(
            status="pass" if ok else "fail",
            n_still_samples=int(np.sum(still)),
            az_mean_g=az_mean_g,
            az_mps2=az_mps2,
            target_mps2=G_TARGET_MPS2,
            tolerance_mps2=tol_mps2,
        )
        out.append(rec)
    return out


# --------------------------------------------------------------------------
# Check 2 -- old vs new coherence (RULES §10.2), reads an already-built
# old_vs_new.csv (produced by tables.build_old_vs_new_csv / build_all).
# --------------------------------------------------------------------------

def check2_old_vs_new_consistency(old_vs_new_csv_path, tol=1e-2):
    path = Path(old_vs_new_csv_path)
    if not path.is_file():
        return {"status": "insufficient_data", "detail": f"{path} does not exist"}

    with open(path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))

    checked, failed, skipped_null = [], [], []
    for r in rows:
        if r.get("comparability") not in SAME_DEFINITION_KINDS:
            continue
        old_v, new_v, diff = r.get("old_value_converted"), r.get("new_value"), r.get("diff")
        if old_v in (None, "", "None") or new_v in (None, "", "None"):
            skipped_null.append({"test_id": r.get("test_id"), "old_column": r.get("old_column")})
            continue
        try:
            d = float(diff) if diff not in (None, "", "None") else abs(float(new_v) - float(old_v))
        except ValueError:
            skipped_null.append({"test_id": r.get("test_id"), "old_column": r.get("old_column")})
            continue
        rec = {"test_id": r.get("test_id"), "old_column": r.get("old_column"),
               "new_column": r.get("new_column"), "diff": d}
        checked.append(rec)
        if abs(d) > tol:
            failed.append(rec)

    if not checked:
        status = "insufficient_data"
    elif failed:
        status = "fail"
    else:
        status = "pass"
    return {
        "status": status, "tolerance": tol,
        "n_checked": len(checked), "n_failed": len(failed),
        "n_skipped_null": len(skipped_null),
        "failed": failed, "skipped_null": skipped_null,
    }


# --------------------------------------------------------------------------
# Check 3 -- known cases (RULES §10.3)
# --------------------------------------------------------------------------

def _metrics_for(code, root, campaign_dir):
    run_dir = resolve_run(code, [campaign_dir])
    run = load_run_v2(run_dir)
    return metrics.compute_run_metrics(run)


def check3_known_cases(root=None):
    root = root or find_root()
    campaign_dir = root / DEFAULT_CAMPAIGN
    out = {}

    # P004-R015: solver_usable_pct ~45% (old) -> ~99% (new), ONLY because
    # metrics._solver() counts solved+solved_inaccurate as usable (verified
    # against the controller code per metrics.py's own comment next to
    # `solver_usable_pct`) -- i.e. the semantics is confirmed, so this check
    # reports the real computed number rather than forcing it.
    try:
        m = _metrics_for("P004-R015", root, campaign_dir)
        usable = m.get("solver_usable_pct")
        if not m.get("solver_valid") or usable is None:
            out["P004_R015_usable"] = {
                "status": "insufficient_data",
                "detail": f"solver_valid={m.get('solver_valid')} "
                          f"({m.get('solver_valid_why')})",
            }
        else:
            ok = usable >= 90.0
            out["P004_R015_usable"] = {
                "status": "pass" if ok else "fail",
                "old_approx_pct": 45.0, "new_solver_usable_pct": usable,
                "note": "semantics confirmed (solved+solved_inaccurate counted as "
                        "usable, see metrics.py _solver comment); threshold >=90% "
                        "chosen as 'near 99%' (assumption).",
            }
    except Exception as exc:  # noqa: BLE001
        out["P004_R015_usable"] = {"status": "insufficient_data", "detail": repr(exc)}

    # P004-R014 / P004-R015: pose_valid must be False.
    for code in ("P004-R014", "P004-R015"):
        key = f"{code.replace('-', '_')}_pose_valid"
        try:
            m = _metrics_for(code, root, campaign_dir)
            pv = m.get("pose_valid")
            out[key] = {
                "status": "pass" if pv is False else "fail",
                "pose_valid": pv, "pose_valid_why": m.get("pose_valid_why"),
            }
        except Exception as exc:  # noqa: BLE001
            out[key] = {"status": "insufficient_data", "detail": repr(exc)}

    # P003-R009: "resta critica" is not a formally-defined RULES.md column
    # (no manual_verdicts.yaml entry either, checked 2026-11-03), so this
    # check reports the quality/safety fields relevant to a manual
    # "critical" judgement instead of asserting a single pass/fail on an
    # undefined criterion (card: don't force the number).
    try:
        m = _metrics_for("P003-R009", root, campaign_dir)
        out["P003_R009_critical"] = {
            "status": "reported",
            "detail": "'critica' is not formally defined in RULES.md; reporting "
                      "quality/safety fields for manual judgement instead of "
                      "forcing a verdict.",
            "pose_valid": m.get("pose_valid"),
            "imu_valid": m.get("imu_valid"),
            "solver_valid": m.get("solver_valid"),
            "solver_infeasible_pct": m.get("solver_infeasible_pct"),
            "contact_lidar": m.get("contact_lidar"),
            "clearance_coverage_pct": m.get("clearance_coverage_pct"),
        }
    except Exception as exc:  # noqa: BLE001
        out["P003_R009_critical"] = {"status": "insufficient_data", "detail": repr(exc)}

    # P001-R008: jerk in 'moving' much higher than the old 0.315 (g/s, old
    # sheet units). Compared against jerk_rms_5hz_mps3, NOT the pipeline's
    # primary jerk_rms_mps3 (2 Hz cutoff) -- tables.OLD_VS_NEW_SPECS maps the
    # old "jerk_rms" column to jerk_rms_5hz_mps3 ("same_unit_converted"), so
    # that is the only like-for-like column; the 2 Hz value is ~2.5 m/s^3 for
    # this run (checked 2026-11-03), i.e. *lower* than old, which would
    # wrongly fail this check -- that is a cutoff-choice artifact, not
    # evidence against the RULES §10.3 claim, hence not used here.
    try:
        m = _metrics_for("P001-R008", root, campaign_dir)
        new_jerk = m.get("jerk_rms_5hz_mps3")
        old_converted = 0.315 * G_TARGET_MPS2
        if new_jerk is None:
            out["P001_R008_jerk"] = {
                "status": "insufficient_data",
                "detail": f"jerk_rms_5hz_mps3 is null ({m.get('jerk_rms_5hz_mps3_why')})",
            }
        else:
            ratio = new_jerk / old_converted if old_converted else None
            ok = ratio is not None and ratio >= 1.5  # "much higher" (assumption)
            out["P001_R008_jerk"] = {
                "status": "pass" if ok else "fail",
                "old_value_g_per_s": 0.315, "old_value_converted_mps3": old_converted,
                "new_jerk_rms_5hz_mps3": new_jerk, "ratio": ratio,
                "note": "compared against jerk_rms_5hz_mps3 (same filter cutoff as the "
                        "old pipeline, per OLD_VS_NEW_SPECS); 'much higher' taken as "
                        "ratio>=1.5 (assumption, RULES gives no number).",
            }
    except Exception as exc:  # noqa: BLE001
        out["P001_R008_jerk"] = {"status": "insufficient_data", "detail": repr(exc)}

    return out


# --------------------------------------------------------------------------
# Check 4 -- identical recompute (RULES §10.4)
# --------------------------------------------------------------------------

#: Columns allowed to differ between two recomputes of the same input
#: (declared non-deterministic sources). Empty unless proven otherwise by a
#: real diff -- see check4's own report when it finds any.
DECLARED_NONDETERMINISTIC_COLUMNS = ()


def check4_recompute_identical(selection_path, work_dir, root=None):
    root = root or find_root()
    work_dir = Path(work_dir)
    out_a = work_dir / "recompute_a"
    out_b = work_dir / "recompute_b"
    for d in (out_a, out_b):
        if d.exists():
            return {"status": "insufficient_data",
                    "detail": f"{d} already exists, refusing to overwrite a stale run"}

    tables.build_all(out_a, selection_path=selection_path, root=root)
    tables.build_all(out_b, selection_path=selection_path, root=root)

    csv_a, csv_b = out_a / "runs_full.csv", out_b / "runs_full.csv"
    if not (csv_a.is_file() and csv_b.is_file()):
        return {"status": "insufficient_data", "detail": "runs_full.csv missing in one of the two runs"}

    with open(csv_a, encoding="utf-8") as fh:
        rows_a = list(csv.DictReader(fh))
    with open(csv_b, encoding="utf-8") as fh:
        rows_b = list(csv.DictReader(fh))

    diffs = []
    if len(rows_a) != len(rows_b):
        diffs.append({"row": "*", "field": "*", "detail": f"{len(rows_a)} vs {len(rows_b)} rows"})
    else:
        for i, (ra, rb) in enumerate(zip(rows_a, rows_b)):
            for key in ra.keys():
                if key in DECLARED_NONDETERMINISTIC_COLUMNS:
                    continue
                if ra.get(key) != rb.get(key):
                    diffs.append({"row": i, "test_id": ra.get("test_id"), "field": key,
                                  "a": ra.get(key), "b": rb.get(key)})

    return {
        "status": "pass" if not diffs else "fail",
        "n_rows": len(rows_a), "n_diffs": len(diffs),
        "diffs": diffs[:50],
        "a": str(csv_a), "b": str(csv_b),
    }


# --------------------------------------------------------------------------
# Stop test 1 -- P002-R001 ambiguity (card: "Test di stop")
# --------------------------------------------------------------------------

def test_stop_ambiguous_run(root=None):
    """Calls ``scripts.run_data.resolve_run`` directly -- the exact function
    ``run_analysis.py``'s own ``_preflight_resolve`` calls during
    ``main()``'s preflight step, and which it never catches for
    ``AmbiguousRunError`` (only ``RunNotFoundError`` is swallowed there) --
    so this exercises the real stop condition, just not through ``main()``
    itself.
    """
    root = root or find_root()
    campaign_dir = root / DEFAULT_CAMPAIGN
    try:
        resolve_run("P002-R001", [campaign_dir])
    except AmbiguousRunError as exc:
        names = {p.name for p in exc.alternatives}
        ok = names == set(P002_R001_FOLDERS)
        return {
            "status": "pass" if ok else "fail",
            "detail": "resolve_run() (the exact function run_analysis.py's "
                      "_preflight_resolve calls, and never catches "
                      "AmbiguousRunError from) raises AmbiguousRunError and "
                      "lists every alternative, never picking one",
            "alternatives_found": sorted(names),
            "alternatives_expected": sorted(P002_R001_FOLDERS),
            "error_message": str(exc),
        }
    except RunNotFoundError as exc:
        return {"status": "fail", "detail": f"expected AmbiguousRunError, got RunNotFoundError: {exc}"}
    return {"status": "fail", "detail": "resolve_run did NOT stop: no exception raised"}


# --------------------------------------------------------------------------
# Stop test 2 -- output_folder already exists (card: "Test di stop")
# --------------------------------------------------------------------------

def test_stop_output_exists(tmp_dir):
    """Reproduce run_analysis.main()'s "output folder already exists" stop
    for the exact scenario the card asks for: relaunching with ``--output``
    pointed at a folder that already exists (and is non-empty -- post
    ENTRYPOINT_FIX/F015, the check is ``output_dir.exists() and
    any(output_dir.iterdir())``, unconditional on ``--output``), without
    touching the real ``selection.yaml`` or any real output folder.

    Monkeypatches ``run_analysis.HERE`` (module global) to a throwaway
    directory under ``_test/`` for the duration of the call only, so
    ``main()`` reads/writes there instead of the real
    ``analysis_v2/selection.yaml``. ``find_root()`` is unaffected (it is
    anchored on ``scripts/run_data.py``'s own ``__file__``, not on
    ``run_analysis.HERE``), so the ambiguity/campaign lookups above still
    see the real, read-only campaign data.

    ``--output B`` is passed with ``B`` *different* from the selection's own
    ``output_folder`` (so this is a genuine ``overridden=True`` case, not the
    no-``--output`` branch), and ``B`` is pre-populated with a marker file so
    it is non-empty (an empty pre-existing folder no longer trips the check,
    by design, post-F015).
    """
    tmp_dir = Path(tmp_dir)
    tmp_dir.mkdir(parents=True, exist_ok=True)
    existing_output = tmp_dir / "B"
    existing_output.mkdir(exist_ok=True)
    marker = existing_output / "marker.txt"
    marker.write_text("pre-existing output, must not be touched\n", encoding="utf-8")
    before = sorted(existing_output.iterdir())

    (tmp_dir / "selection.yaml").write_text(
        "output_folder: default_output\n"
        f"runs:\n  - [{E2E_RUN_IDS[0]}, no, si, no, no, no, no]\n",
        encoding="utf-8",
    )

    real_here = run_analysis.HERE
    run_analysis.HERE = tmp_dir
    try:
        rc = run_analysis.main(["--output", "B"])
    finally:
        run_analysis.HERE = real_here

    after = sorted(existing_output.iterdir())
    default_output_created = (tmp_dir / "default_output").exists()
    ok = rc != 0 and before == after and not default_output_created
    return {
        "status": "pass" if ok else "fail",
        "detail": "run_analysis.main(['--output', 'B']) with a pre-existing, "
                  "non-empty 'B' must return non-zero and write nothing "
                  "inside 'B' (the card's exact '--output pointed at an "
                  "already-existing folder' stop scenario, post F015/"
                  "ENTRYPOINT_FIX)",
        "returncode": rc, "dir_contents_before": [p.name for p in before],
        "dir_contents_after": [p.name for p in after],
        "selections_own_output_folder_created": default_output_created,
    }


# --------------------------------------------------------------------------
# End-to-end smoke test on the 3 "run di prova" (card)
# --------------------------------------------------------------------------

_E2E_SELECTION_TEMPLATE = (
    "output_folder: {output_folder}\n"
    "runs:\n"
    + "".join(f"  - [{rid}, no, si, si, si, si, si]\n" for rid in E2E_RUN_IDS)
)


def run_e2e_three_runs(output_dir, root=None, device="cuda"):
    """Metrics + png + both videos for the 3 test runs, via a temporary
    ``selection.yaml.test_tmp`` (never touches the real ``selection.yaml``)
    fed straight into ``run_analysis.main(['--selection', <tmp>, '--output',
    <output_dir>])`` -- the real entrypoint, not a hand-rolled
    reimplementation of its orchestration, so this exercises
    ``run_analysis._run_png``/``_run_video`` for real (card TESTS_FIX2,
    MEDIO#5).

    ``device`` is accepted for CLI-interface stability (``--device`` on this
    module's own CLI) but is currently a no-op: ``run_analysis.py`` has no
    ``--device`` flag of its own to forward it through, and
    ``video_mask.py``'s own default is already ``"cuda"`` with a
    CPU/unavailable-GPU fallback inside ``Segmenter``.
    """
    root = root or find_root()
    tmp_selection = HERE / "selection.yaml.test_tmp"
    tmp_selection.write_text(
        _E2E_SELECTION_TEMPLATE.format(output_folder=Path(output_dir).name),
        encoding="utf-8",
    )

    output_dir = Path(output_dir)
    # run_analysis.main() stops (card-validated, see ENTRYPOINT.md) if its
    # resolved output folder already exists and is non-empty; this e2e's
    # output_dir is always a throwaway scratch location (analysis_v2/_test/
    # e2e), so wipe any leftover from a previous run to keep this test
    # rerunnable, same as before this change (which overwrote unconditionally).
    if output_dir.exists():
        shutil.rmtree(output_dir)

    report = {"tmp_selection": str(tmp_selection), "run_ids": list(E2E_RUN_IDS)}
    try:
        rc = run_analysis.main(["--selection", str(tmp_selection), "--output", str(output_dir)])
        report["returncode"] = rc

        summary_path = output_dir / "summary.txt"
        report["summary_txt"] = (
            summary_path.read_text(encoding="utf-8") if summary_path.is_file() else None
        )

        png_singole_files = list(output_dir.glob("*/png_singole/*.png"))
        png_group_files = list(output_dir.glob("*/png_selezionate/*.png"))
        report["png"] = {"n_singole": len(png_singole_files), "n_groups": len(png_group_files)}
        report["png_singole_files"] = [str(p) for p in png_singole_files]
        report["png_group_files"] = [str(p) for p in png_group_files]

        overhead_videos = list(output_dir.glob("*/video_dall_alto/*"))
        mask_videos = list(output_dir.glob("*/video_maschera/*"))
        report["overhead_video_files"] = [str(p) for p in overhead_videos]
        report["mask_video_files"] = [str(p) for p in mask_videos]
        if not overhead_videos:
            report["overhead_video_missing_reason"] = (
                "no file found under <output_dir>/*/video_dall_alto/ -- see "
                "summary_txt above for why"
            )
        if not mask_videos:
            report["mask_video_missing_reason"] = (
                "no file found under <output_dir>/*/video_maschera/ -- GPU/model "
                "dependency may not be available in this environment; see "
                "summary_txt above -- not required to pass (card acceptance "
                "criteria)."
            )
    finally:
        tmp_selection.unlink(missing_ok=True)

    return report


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _json_default(o):
    if isinstance(o, Path):
        return str(o)
    return repr(o)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check", choices=("imu", "old-vs-new", "known-cases", "recompute",
                             "stop-ambiguous", "stop-output-exists", "e2e", "all"),
        default="all",
    )
    parser.add_argument("--old-vs-new-csv", default=None,
                         help="path to an old_vs_new.csv to check (for --check old-vs-new; "
                              "defaults to <output>/_test/e2e/old_vs_new.csv if present)")
    parser.add_argument("--output", default=str(HERE / "_test"),
                         help="where the e2e/stop/recompute scratch dirs are created "
                              "(default: analysis_v2/_test/)")
    parser.add_argument("--device", default="cuda",
                         help="--device passed to video_mask (default: cuda)")
    args = parser.parse_args(argv)

    root = find_root()
    base = Path(args.output)
    base.mkdir(parents=True, exist_ok=True)
    report = {}

    if args.check in ("imu", "all"):
        report["check1_imu_conversion"] = check1_imu_conversion(E2E_RUN_IDS, root=root)

    if args.check in ("stop-ambiguous", "all"):
        report["stop_test_ambiguous_run"] = test_stop_ambiguous_run(root=root)

    if args.check in ("stop-output-exists", "all"):
        report["stop_test_output_exists"] = test_stop_output_exists(base / "stop_output_exists")

    if args.check in ("known-cases", "all"):
        report["check3_known_cases"] = check3_known_cases(root=root)

    if args.check in ("e2e", "all"):
        report["e2e"] = run_e2e_three_runs(base / "e2e", root=root, device=args.device)
        old_vs_new_csv = base / "e2e" / "old_vs_new.csv"
    else:
        old_vs_new_csv = args.old_vs_new_csv or (base / "e2e" / "old_vs_new.csv")

    if args.check in ("old-vs-new", "all"):
        report["check2_old_vs_new"] = check2_old_vs_new_consistency(old_vs_new_csv)

    if args.check in ("recompute", "all"):
        tmp_selection = _write_recompute_tmp_selection(base)
        try:
            report["check4_recompute_identical"] = check4_recompute_identical(
                tmp_selection, base / "recompute", root=root,
            )
        finally:
            # ALTO#2 (REVIEW finale): the recompute selection must never be
            # left behind on disk either -- unlink unconditionally, same as
            # run_e2e_three_runs's own try/finally for its tmp selection.
            tmp_selection.unlink(missing_ok=True)

    out_path = base / "verify_report.json"
    out_path.write_text(json.dumps(report, indent=2, default=_json_default), encoding="utf-8")
    print(json.dumps(report, indent=2, default=_json_default))
    print(f"\nreport written to {out_path}", file=sys.stderr)
    return 0


def _write_recompute_tmp_selection(base):
    tmp = HERE / "selection.yaml.test_tmp"
    tmp.write_text(_E2E_SELECTION_TEMPLATE.format(output_folder="unused"), encoding="utf-8")
    return tmp


if __name__ == "__main__":
    sys.exit(main())
