#!/usr/bin/env python3
"""ENTRYPOINT of the analysis_v2 pipeline: ``python3 run_analysis.py``.

Reads ``selection.yaml``, resolves every listed run, then calls (in order)
the metrics/tables phase, the PNG phase and the video phase (overhead +
mask), printing and saving a ``summary.txt`` in the resolved output folder.

Stop conditions (never produce partial/misleading output for these):
  * a ``selection.yaml`` row has an empty/invalid si-no field -> hard error
    naming the row (raised by ``scripts.run_data.load_selection``).
  * a run code matches more than one folder -> hard error listing every
    alternative (``scripts.run_data.AmbiguousRunError``, never picked for
    you).
  * the resolved output folder (after applying ``--output``, if given) already
    exists on disk and is not empty -> hard error, nothing is written into it.
    This applies to the effective target folder regardless of whether it
    matches selection.yaml's own ``output_folder`` or an ``--output``
    override: pass ``--output`` pointing to a not-yet-existing folder for a
    recompute/test run.

Non-stop condition: an exception while computing/plotting/rendering ONE run
is caught, recorded, and never stops the rest of the batch -- it shows up in
``summary.txt`` instead.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from scripts.run_data import (  # noqa: E402
    AmbiguousRunError,
    DEFAULT_CAMPAIGN,
    RunNotFoundError,
    SelectionError,
    find_root,
    load_run_v2,
    load_selection,
    resolve_run,
)
from scripts import metrics as metrics_mod  # noqa: E402
from scripts import png as png_mod  # noqa: E402
from scripts import tables  # noqa: E402
from scripts import video_mask  # noqa: E402
from scripts import video_overhead  # noqa: E402

PHASES = ("metrics", "png", "video")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Orchestrate the analysis_v2 pipeline for the runs "
                     "listed in selection.yaml (metrics/tables, PNG, video)."
    )
    parser.add_argument(
        "--only", choices=PHASES, default=None,
        help="run only this phase instead of all three (metrics, png, video)",
    )
    parser.add_argument(
        "--output", default=None,
        help="override selection.yaml's output_folder for this run only "
             "(selection.yaml itself is never modified); use a name "
             "different from the real output_folder for a recompute/test run",
    )
    parser.add_argument(
        "--selection", default=None,
        help="use this selection.yaml file instead of the default one next "
             "to this script (e.g. a temporary file built by a caller/test); "
             "does not affect --output resolution, which still applies on "
             "top of whichever selection.yaml is used",
    )
    return parser.parse_args(argv)


def _copy_rules_md(root):
    """Copy ``first_test_campaign/analisys_v2/RULES.md`` (refuso) here, once.

    Idempotent: a no-op if the destination already exists (whether or not
    SCOUT_DATA already did it) -- never overwrites.
    """
    src = root / "first_test_campaign" / "analisys_v2" / "RULES.md"
    dst = HERE / "RULES.md"
    if dst.exists() or not src.is_file():
        return
    shutil.copyfile(src, dst)


def _preflight_resolve(selection, campaign_dir):
    """Resolve every selection.yaml row's run code once, up front.

    Returns ``(resolved, missing)``. Raises :class:`AmbiguousRunError`
    straight away (hard stop, card requirement) -- a run code matching more
    than one folder is never silently picked for the caller.
    """
    resolved = {}
    missing = []
    for row in selection.rows:
        try:
            run_dir = resolve_run(row.run, [campaign_dir])  # AmbiguousRunError propagates
        except RunNotFoundError as exc:
            missing.append((row, str(exc)))
            continue
        resolved[row.run] = run_dir
    return resolved, missing


def _run_metrics(selection_path, root, output_dir, errors):
    try:
        result = tables.build_all(output_dir, selection_path=selection_path, root=root)
    except Exception as exc:  # noqa: BLE001 - one phase crashing must not stop the others
        errors.append(("metrics", "*", f"metrics phase did not complete: {exc!r}"))
        return {"n_rows": 0, "resolve_errors": [], "crashed": True}
    for err in result.get("resolve_errors", []):
        errors.append(("metrics", err.get("run", "?"), err.get("error", "?")))
    return result


def _run_png(selection, resolved, output_dir, errors):
    """Render the per-run and per-group PNGs into the SAME group folder
    (``<output_dir>/<group_key>/png_singole|png_selezionate/``) that
    video_mask.py/video_overhead.py use for their videos (RULES.md §9),
    using metrics computed with this row's ``ostacolo`` (never ``None``)
    so the plotted values agree with results.csv/runs_full.csv.
    """
    n_singole = 0
    groups = {}

    for row in selection.rows:
        if not (row.png or row.selezionate):
            continue
        run_dir = resolved.get(row.run)
        if run_dir is None:
            errors.append(("png", row.run, "run not found, skipped"))
            continue
        try:
            run = load_run_v2(run_dir)
        except Exception as exc:  # noqa: BLE001 - isolate this run, keep going
            errors.append(("png", row.run, f"load_run_v2 failed: {exc!r}"))
            continue

        try:
            run_metrics = metrics_mod.compute_run_metrics(run, ostacolo=row.ostacolo)
        except Exception:  # noqa: BLE001 - let the render calls' own metrics
            # fallback (same function, same ostacolo) surface the error below.
            run_metrics = None

        label = png_mod.group_label(run.mission, row.ostacolo)
        group_dir = output_dir / label

        if row.png:
            try:
                png_mod.render_run_png(
                    run, group_dir / "png_singole" / f"{run.test_id}.png",
                    metrics=run_metrics, ostacolo=row.ostacolo,
                )
                n_singole += 1
            except Exception as exc:  # noqa: BLE001
                errors.append(("png", row.run, f"render_run_png failed: {exc!r}"))

        if row.selezionate:
            entry = groups.setdefault(label, {"runs": [], "metrics": [], "ostacolo": []})
            entry["runs"].append(run)
            entry["metrics"].append(run_metrics)
            entry["ostacolo"].append(row.ostacolo)

    n_groups = 0
    for label, entry in groups.items():
        try:
            png_mod.render_group_png(
                entry["runs"], output_dir / label / "png_selezionate" / f"{label}.png",
                ostacolo=entry["ostacolo"], metrics_list=entry["metrics"], label=label,
            )
            n_groups += 1
        except Exception as exc:  # noqa: BLE001
            errors.append(("png", label, f"render_group_png failed: {exc!r}"))

    return {"n_singole": n_singole, "n_groups": n_groups}


def _run_video(selection_path, output_dir, errors):
    results = {}
    for name, module in (("mask", video_mask), ("overhead", video_overhead)):
        try:
            returncode = module.main(
                ["--selection", str(selection_path), "--out-root", str(output_dir)]
            )
            results[name] = {"returncode": returncode}
        except Exception as exc:  # noqa: BLE001 - isolate this sub-phase, keep going
            errors.append(("video", name, f"video_{name} phase crashed: {exc!r}"))
            results[name] = {"returncode": 1, "crashed": True}
    return results


def _count_dq_failures(output_dir):
    """Count data_quality.csv rows with at least one failed flag, best-effort."""
    path = output_dir / "data_quality.csv"
    if not path.is_file():
        return None
    import csv

    n_failed = 0
    n_total = 0
    with open(path, encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            n_total += 1
            if (
                row.get("pose_valid") == "False"
                or row.get("imu_valid") == "False"
                or row.get("solver_valid") == "False"
                or row.get("clearance_source") == "none"
            ):
                n_failed += 1
    return n_failed, n_total


def _build_summary(selection, resolved, missing, phases_run, phase_results,
                    errors, elapsed, output_dir, overridden):
    lines = []
    lines.append("=== analysis_v2 run_analysis.py summary ===")
    lines.append(f"output_folder: {output_dir}" + (" (--output override)" if overridden else ""))
    lines.append(f"phases run: {', '.join(phases_run)}")
    lines.append(f"selection.yaml rows: {len(selection.rows)}")
    lines.append(f"runs resolved: {len(resolved)}")

    if missing:
        lines.append(f"runs missing ({len(missing)}):")
        for row, why in missing:
            lines.append(f"  - row {row.index} ({row.run}): {why}")
    else:
        lines.append("runs missing: none")

    for phase, result in phase_results.items():
        lines.append(f"--- {phase} ---")
        lines.append(f"  {result}")

    dq = _count_dq_failures(output_dir)
    if dq is not None:
        n_failed, n_total = dq
        lines.append(f"data-quality flags failed: {n_failed}/{n_total} run(s) "
                      f"with at least one failing flag (pose/imu/solver/clearance)")

    if errors:
        lines.append(f"errors / skipped ({len(errors)}):")
        for phase, run, why in errors:
            lines.append(f"  - [{phase}] {run}: {why}")
    else:
        lines.append("errors / skipped: none")

    lines.append(f"total time: {elapsed:.1f} s")
    return "\n".join(lines) + "\n"


def main(argv=None):
    args = parse_args(argv)
    t_start = time.monotonic()

    root = find_root()
    _copy_rules_md(root)

    selection_path = Path(args.selection).resolve() if args.selection else HERE / "selection.yaml"
    try:
        selection = load_selection(selection_path)
    except SelectionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    overridden = bool(args.output) and args.output != selection.output_folder
    output_name = args.output if args.output else selection.output_folder
    output_dir = HERE / output_name

    if output_dir.exists() and any(output_dir.iterdir()):
        print(
            f"ERROR: output folder already exists and is not empty, refusing to "
            f"overwrite: {output_dir}\n"
            f"(pass --output <other-name> pointing to a folder that does not "
            f"exist yet, for a recompute/test run)",
            file=sys.stderr,
        )
        return 2

    campaign_dir = root / DEFAULT_CAMPAIGN
    try:
        resolved, missing = _preflight_resolve(selection, campaign_dir)
    except AmbiguousRunError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    # Runs not found are not fatal (card: "missing is not a stop"); already
    # collected into `missing` by _preflight_resolve above, for the summary.
    output_dir.mkdir(parents=True, exist_ok=True)

    errors = []
    phase_results = {}
    phases_run = [args.only] if args.only else list(PHASES)

    if "metrics" in phases_run:
        phase_results["metrics"] = _run_metrics(selection_path, root, output_dir, errors)
    if "png" in phases_run:
        phase_results["png"] = _run_png(selection, resolved, output_dir, errors)
    if "video" in phases_run:
        phase_results["video"] = _run_video(selection_path, output_dir, errors)

    elapsed = time.monotonic() - t_start
    summary = _build_summary(
        selection, resolved, missing, phases_run, phase_results, errors, elapsed,
        output_dir, overridden,
    )
    print(summary)
    (output_dir / "summary.txt").write_text(summary, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
