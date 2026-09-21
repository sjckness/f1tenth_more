#!/usr/bin/env python3
"""Read a campaign written by robot_logger and report what happened.

    ros2 run f1tenth_logger test_campaign_analyze [campaign_folder]
        [--mission M] [--prompt N] [--footprints METERS] [--out DIR]

Success is the **manual** ``success`` column of ``campaign_results.csv``
(written by ``test_campaign_export``, filled in by hand), not the logger's
own outcome. A test with an empty ``success`` cell counts towards nothing: it
is excluded from k/N and listed as not yet evaluated, and its trajectory is
drawn dimmed with the automatic outcome shown only as a hint.

Produces a text report, ``summary.csv``, one ``overview_map`` per mission plus
a combined one, and ``dashboard.png``. Runs whose ``meta.json`` is missing are
counted as aborted with reason "no meta.json" -- a crashed run is data, not a
gap, so it is never silently dropped.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import Normalize, TwoSlopeNorm  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle, Patch  # noqa: E402

from f1tenth_logger.test_campaign.robot_logger import (  # noqa: E402
    DEFAULT_CAMPAIGN, find_root, parse_test_id)

Z = 1.96
CLEARANCE_LABEL = "clearance: robot edge to corridor boundary [m] (<0 = outside)"
RESULTS_NAME = "campaign_results.csv"

#: Per-mission quality metrics, reported as median and min..max.
METRIC_COLUMNS = [
    "viol_rate_pct",
    "min_clear_m",
    "feas_pct",
    "max_infeas_streak_s",
    "jerk_rms",
    "steer_rev_per_m",
]

PASS_COLOR = "#2ca02c"
FAIL_COLOR = "#d62728"
UNEVALUATED_COLOR = "#8c8c8c"
TRUE_WORDS = {"1", "1.0", "true", "yes", "y", "pass"}
FALSE_WORDS = {"0", "0.0", "false", "no", "n", "fail"}


# --------------------------------------------------------------------------
# loading
# --------------------------------------------------------------------------

@dataclass
class Run:
    mission: str
    test_id: str
    prompt_num: int
    repetition: int
    path: Path
    auto_outcome: str
    reason: str
    manual_success: int = None
    metrics: dict = field(default_factory=dict)
    meta: dict = field(default_factory=dict)
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    x: np.ndarray = field(default_factory=lambda: np.empty(0))
    y: np.ndarray = field(default_factory=lambda: np.empty(0))
    speed: np.ndarray = field(default_factory=lambda: np.empty(0))
    clearance: np.ndarray = field(default_factory=lambda: np.empty(0))
    latencies: list = field(default_factory=list)
    corridors: list = field(default_factory=list)

    @property
    def evaluated(self):
        return self.manual_success is not None

    @property
    def verdict(self):
        if self.manual_success is None:
            return "unevaluated"
        return "pass" if self.manual_success else "fail"

    @property
    def color(self):
        return {
            "pass": PASS_COLOR,
            "fail": FAIL_COLOR,
            "unevaluated": UNEVALUATED_COLOR,
        }[self.verdict]

    @property
    def robot_radius(self):
        return float(self.meta.get("robot_radius") or 0.0)

    @property
    def has_clearance(self):
        return bool(self.clearance.size) and bool(np.isfinite(self.clearance).any())


def _read_csv_columns(path, columns):
    """Return {column: float array}, NaN for empty cells. Missing file -> empty."""
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


def _as_float(text):
    """Parse a cell that may use a decimal comma; None if it is not a number."""
    text = (text or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        pass
    try:
        return float(text.replace(",", "."))
    except ValueError:
        return None


def load_manual_results(campaign_dir):
    """``({test_id: row}, unreadable)`` from campaign_results.csv.

    Returns empty results when the export has not been run yet. ``unreadable``
    lists cells whose ``success`` value is neither 0 nor 1.
    """
    path = campaign_dir / RESULTS_NAME
    if not path.exists():
        return {}, []
    with open(path, encoding="utf-8-sig") as fh:
        header = fh.readline()
    delimiter = ";" if header.count(";") > header.count(",") else ","

    rows, unreadable = {}, []
    with open(path, newline="", encoding="utf-8-sig") as fh:
        for raw in csv.DictReader(fh, delimiter=delimiter):
            test_id = (raw.get("test_id") or "").strip()
            if not test_id:
                continue
            verdict_text = (raw.get("success") or "").strip()
            lowered = verdict_text.lower()
            if not verdict_text:
                success = None
            elif lowered in TRUE_WORDS:
                success = 1
            elif lowered in FALSE_WORDS:
                success = 0
            else:
                success = None
                unreadable.append((test_id, verdict_text))
            rows[test_id] = {
                "success": success,
                "metrics": {c: _as_float(raw.get(c)) for c in METRIC_COLUMNS},
            }
    return rows, unreadable


def load_run(test_dir, mission):
    parsed = parse_test_id(test_dir.name)
    meta_path = test_dir / "meta.json"
    meta, auto_outcome, reason = {}, "aborted", "no meta.json"
    if meta_path.exists():
        try:
            with open(meta_path, encoding="utf-8") as fh:
                meta = json.load(fh)
        except (OSError, json.JSONDecodeError) as exc:
            reason = f"unreadable meta.json: {exc}"
        else:
            summary = meta.get("summary", meta)
            auto_outcome = summary.get("auto_outcome") or "aborted"
            reason = summary.get("reason") or ""

    kin = _read_csv_columns(
        test_dir / "kinematics.csv", ["t", "x", "y", "speed", "corridor_clearance"]
    )

    latencies = []
    llm_path = test_dir / "llm_calls.csv"
    if llm_path.exists():
        with open(llm_path, newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                raw = (row.get("latency_ms") or "").strip()
                if raw:
                    try:
                        latencies.append(float(raw))
                    except ValueError:
                        pass

    corridors = []
    corr_path = test_dir / "corridors.jsonl"
    if corr_path.exists():
        with open(corr_path, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                poly = record.get("polygon") or []
                if len(poly) >= 3:
                    corridors.append(np.asarray(poly, dtype=float))

    return Run(
        mission=mission,
        test_id=test_dir.name,
        prompt_num=parsed["prompt_num"],
        repetition=parsed["repetition"],
        path=test_dir,
        auto_outcome=auto_outcome,
        reason=reason,
        meta=meta,
        t=kin["t"],
        x=kin["x"],
        y=kin["y"],
        speed=kin["speed"],
        clearance=kin["corridor_clearance"],
        latencies=latencies,
        corridors=corridors,
    )


def load_campaign(campaign_dir):
    """``(runs, has_manual_file, unreadable_verdicts)``."""
    manual, unreadable = load_manual_results(campaign_dir)
    runs = []
    for mission_dir in sorted(p for p in campaign_dir.iterdir() if p.is_dir()):
        for test_dir in sorted(p for p in mission_dir.iterdir() if p.is_dir()):
            try:
                parse_test_id(test_dir.name)
            except ValueError:
                continue  # not a test folder
            run = load_run(test_dir, mission_dir.name)
            entry = manual.get(run.test_id)
            if entry is not None:
                run.manual_success = entry["success"]
                run.metrics = entry["metrics"]
            runs.append(run)
    runs.sort(key=lambda r: (r.mission, r.prompt_num, r.repetition))
    return runs, bool(manual), unreadable


# --------------------------------------------------------------------------
# statistics
# --------------------------------------------------------------------------

def wilson(k, n, z=Z):
    """95 % Wilson score interval for k successes out of n."""
    if n <= 0:
        return (0.0, 0.0)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    margin = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - margin), min(1.0, centre + margin))


def percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), q))


def metric_stats(runs, column):
    """(median, min, max, n) of one exported metric, or None if nothing has it."""
    values = [
        r.metrics.get(column) for r in runs if r.metrics.get(column) is not None
    ]
    if not values:
        return None
    return (statistics.median(values), min(values), max(values), len(values))


def group_stats(label_kind, mission, prompt, runs):
    evaluated = [r for r in runs if r.evaluated]
    n = len(evaluated)
    k = sum(r.manual_success for r in evaluated)
    lo, hi = wilson(k, n)
    latencies = [x for r in runs for x in r.latencies]
    row = {
        "level": label_kind,
        "mission": mission,
        "prompt_num": prompt,
        "n_tests": len(runs),
        "n": n,
        "k": k,
        "not_evaluated": len(runs) - n,
        "success_rate": (k / n) if n else None,
        "ci_low": lo if n else None,
        "ci_high": hi if n else None,
        "llm_latency_mean_ms": statistics.fmean(latencies) if latencies else None,
        "llm_latency_median_ms": statistics.median(latencies) if latencies else None,
        "llm_latency_p95_ms": percentile(latencies, 95),
        "n_llm_calls": len(latencies),
    }
    for column in METRIC_COLUMNS:
        stats = metric_stats(runs, column)
        row[f"{column}_median"] = None if stats is None else stats[0]
        row[f"{column}_min"] = None if stats is None else stats[1]
        row[f"{column}_max"] = None if stats is None else stats[2]
    return row


def build_summary(runs):
    rows = []
    for mission in sorted({r.mission for r in runs}):
        mission_runs = [r for r in runs if r.mission == mission]
        rows.append(group_stats("mission", mission, "", mission_runs))
        for prompt in sorted({r.prompt_num for r in mission_runs}):
            prompt_runs = [r for r in mission_runs if r.prompt_num == prompt]
            rows.append(group_stats("prompt", mission, prompt, prompt_runs))
    rows.append(group_stats("overall", "ALL", "", runs))
    return rows


def _fmt_ms(value):
    return "     -" if value is None else f"{value:6.0f}"


def _fmt_num(value):
    return "-" if value is None else f"{value:.3f}"


def format_report(campaign_dir, runs, rows, has_manual, unreadable):
    out = []
    evaluated = [r for r in runs if r.evaluated]
    out.append(f"campaign: {campaign_dir}")
    out.append(
        f"runs: {len(runs)}   missions: {len({r.mission for r in runs})}   "
        f"evaluated by hand: {len(evaluated)}   "
        f"not yet evaluated: {len(runs) - len(evaluated)}"
    )
    if not has_manual:
        out.append("")
        out.append(
            f"NOTE: there is no {RESULTS_NAME} in this campaign, so no test has a\n"
            f"      manual verdict yet. Run test_campaign_export, fill in the\n"
            f"      'success' column by hand, then run this again."
        )
    for test_id, text in unreadable:
        out.append(f"WARNING: {test_id} has success={text!r}, which is neither 0 "
                   f"nor 1 -- treated as not evaluated")
    out.append("")
    out.append("success is the MANUAL verdict; k/N counts evaluated tests only")
    header = (
        f"{'scope':<28}{'k/N':>9}{'succ':>7}{'95% CI':>16}{'n/e':>5}"
        f"{'mean':>7}{'med':>7}{'p95':>7}{'calls':>7}"
    )
    out.append(header)
    out.append("-" * len(header))
    for row in rows:
        if row["level"] == "mission":
            scope = row["mission"]
        elif row["level"] == "prompt":
            scope = f"  P{int(row['prompt_num']):03d}"
        else:
            out.append("-" * len(header))
            scope = "OVERALL"
        rate = "   -  " if row["success_rate"] is None else f"{row['success_rate']:5.0%} "
        if row["ci_low"] is None:
            ci = "-".rjust(16)
        else:
            ci = f"[{row['ci_low']:.2f}, {row['ci_high']:.2f}]".rjust(16)
        out.append(
            f"{scope:<28}{row['k']:>4}/{row['n']:<4}{rate:>7}{ci}"
            f"{row['not_evaluated']:>5}"
            f"{_fmt_ms(row['llm_latency_mean_ms'])}"
            f"{_fmt_ms(row['llm_latency_median_ms'])}"
            f"{_fmt_ms(row['llm_latency_p95_ms'])}"
            f"{row['n_llm_calls']:>7}"
        )
    out.append("")
    out.append("n/e = not yet evaluated. LLM latency in ms, over every recorded call.")

    out.append("")
    out.append(f"per-mission metrics from {RESULTS_NAME}: median (min .. max)")
    width = max(len(c) for c in METRIC_COLUMNS) + 2
    for row in rows:
        if row["level"] != "mission":
            continue
        out.append(f"  {row['mission']}")
        for column in METRIC_COLUMNS:
            median = row[f"{column}_median"]
            if median is None:
                out.append(f"    {column:<{width}}       -")
                continue
            out.append(
                f"    {column:<{width}}{median:8.3f}"
                f"   ({_fmt_num(row[column + '_min'])} .. "
                f"{_fmt_num(row[column + '_max'])})"
            )

    failed = [r for r in runs if r.verdict == "fail"]
    out.append("")
    out.append(f"failed by hand: {len(failed)}")
    for run in failed:
        out.append(f"  {run.mission:<18}{run.test_id}  auto={run.auto_outcome}  "
                   f"{run.reason}")

    aborted = [r for r in runs if r.auto_outcome != "completed"]
    out.append("")
    out.append(f"aborted by the logger (automatic hint only): {len(aborted)}")
    for run in aborted:
        out.append(
            f"  {run.mission:<18}{run.test_id}  [{run.verdict}]  "
            f"{run.reason or '(no reason recorded)'}"
        )

    pending = [r for r in runs if not r.evaluated]
    out.append("")
    out.append(f"not yet evaluated: {len(pending)}")
    for run in pending:
        out.append(f"  {run.mission:<18}{run.test_id}  auto={run.auto_outcome}")
    return "\n".join(out)


# --------------------------------------------------------------------------
# plots
# --------------------------------------------------------------------------

def _clearance_norm(runs):
    parts = [r.clearance[np.isfinite(r.clearance)] for r in runs if r.has_clearance]
    if not parts:
        return None
    values = np.concatenate(parts)
    if values.size == 0:
        return None
    lo, hi = float(values.min()), float(values.max())
    if lo < 0.0 < hi:
        return TwoSlopeNorm(vcenter=0.0, vmin=lo, vmax=hi)
    if math.isclose(lo, hi):
        return Normalize(vmin=lo - 0.05, vmax=hi + 0.05)
    return Normalize(vmin=lo, vmax=hi)


def _arc_length(x, y):
    steps = np.hypot(np.diff(x), np.diff(y))
    return np.concatenate([[0.0], np.cumsum(steps)])


def _end_marker(ax, run, x, y):
    """Black dot = passed, red X = failed, grey = not evaluated (auto shape)."""
    if run.verdict == "pass":
        ax.plot(x, y, "o", color="black", markersize=5, zorder=5)
    elif run.verdict == "fail":
        ax.plot(x, y, "x", color=FAIL_COLOR, markersize=9,
                markeredgewidth=2.2, zorder=5)
    elif run.auto_outcome == "completed":
        ax.plot(x, y, "o", color=UNEVALUATED_COLOR, markersize=5, zorder=5)
    else:
        ax.plot(x, y, "x", color=UNEVALUATED_COLOR, markersize=9,
                markeredgewidth=2.0, zorder=5)


def overview_map(runs, title, out_path, footprints=None):
    """Corridors + trajectories coloured by clearance, per mission or combined."""
    fig, ax = plt.subplots(figsize=(11, 8))
    norm = _clearance_norm(runs)
    mappable = None
    any_footprint = False

    for run in runs:
        for poly in run.corridors:
            ax.fill(
                poly[:, 0], poly[:, 1],
                facecolor="#4c78c8", edgecolor="#2f4f80",
                alpha=0.08, linewidth=0.6, zorder=1,
            )

    for run in runs:
        good = np.isfinite(run.x) & np.isfinite(run.y)
        if good.sum() < 2:
            continue
        x, y = run.x[good], run.y[good]
        clear = run.clearance[good] if run.clearance.size == good.size else np.empty(0)
        dimmed = not run.evaluated

        if norm is not None and clear.size == x.size and np.isfinite(clear).any():
            points = np.column_stack([x, y]).reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            values = 0.5 * (clear[:-1] + clear[1:])
            lc = LineCollection(
                segments, cmap="RdYlGn", norm=norm, linewidths=1.6,
                alpha=0.4 if dimmed else 1.0, zorder=3,
            )
            lc.set_array(values)
            mappable = ax.add_collection(lc)
        else:
            # no clearance recorded: fall back to the verdict
            ax.plot(x, y, color=run.color, linewidth=1.4,
                    alpha=0.4 if dimmed else 0.85, zorder=3)

        radius = run.robot_radius
        if radius > 0.0:
            if clear.size == x.size and np.isfinite(clear).any():
                idx = int(np.nanargmin(np.where(np.isfinite(clear), clear, np.inf)))
            else:
                idx = len(x) - 1
            ax.add_patch(
                Circle(
                    (x[idx], y[idx]), radius,
                    facecolor="none", edgecolor="black",
                    linewidth=1.1, alpha=0.4 if dimmed else 0.9, zorder=4,
                )
            )
            any_footprint = True

            if footprints and footprints > 0:
                travelled = _arc_length(x, y)
                marks = np.arange(0.0, travelled[-1] + 1e-9, footprints)
                for where in marks:
                    j = int(np.searchsorted(travelled, where))
                    j = min(j, len(x) - 1)
                    ax.add_patch(
                        Circle(
                            (x[j], y[j]), radius,
                            facecolor="none", edgecolor="black",
                            linewidth=0.6, alpha=0.18, zorder=2,
                        )
                    )

        _end_marker(ax, run, x[-1], y[-1])

    if mappable is not None:
        cbar = fig.colorbar(mappable, ax=ax, pad=0.02)
        cbar.set_label(CLEARANCE_LABEL)

    handles = [
        Patch(facecolor="#4c78c8", alpha=0.25, edgecolor="#2f4f80", label="corridor"),
        Line2D([], [], color="black", marker="o", linestyle="none",
               label="end: passed (manual)"),
        Line2D([], [], color=FAIL_COLOR, marker="x", linestyle="none",
               markeredgewidth=2.2, label="end: failed (manual)"),
        Line2D([], [], color=UNEVALUATED_COLOR, marker="o", linestyle="none",
               label="end: not evaluated (shape = auto outcome)"),
    ]
    if any_footprint:
        handles.append(
            Line2D([], [], color="black", marker="o", markerfacecolor="none",
                   linestyle="none", markersize=9,
                   label="footprint at min clearance")
        )
    if mappable is None:
        handles.insert(1, Line2D([], [], color=PASS_COLOR, label="run: passed"))
        handles.insert(2, Line2D([], [], color=FAIL_COLOR, label="run: failed"))
        handles.insert(3, Line2D([], [], color=UNEVALUATED_COLOR,
                                 label="run: not evaluated"))

    evaluated = [r for r in runs if r.evaluated]
    k = sum(r.manual_success for r in evaluated)
    pending = len(runs) - len(evaluated)
    suffix = f", {pending} not evaluated" if pending else ""
    ax.set_title(f"{title} -- {k}/{len(evaluated)} passed{suffix}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.autoscale_view()
    # A corridor is long and thin, so a fixed figure would leave the map a
    # sliver in the middle of the page. Size the page to the data instead, and
    # keep the legend out of the corridor.
    (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
    span_x = max(x1 - x0, 1e-6)
    span_y = max(y1 - y0, 1e-6)
    fig.set_size_inches(11.0, min(max(11.0 * span_y / span_x + 2.4, 4.5), 9.5))
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=8,
               framealpha=0.9)
    fig.tight_layout(rect=(0.0, 0.10, 1.0, 1.0))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return out_path


def dashboard(runs, out_path):
    """Clearance vs time, speed vs time, LLM latency histogram."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 11))
    ax_clear, ax_speed, ax_llm = axes

    for run in runs:
        good = np.isfinite(run.t) & np.isfinite(run.clearance)
        if good.sum() >= 2:
            ax_clear.plot(
                run.t[good], run.clearance[good], color=run.color,
                linewidth=0.9, alpha=0.4 if not run.evaluated else 0.7,
            )
    ax_clear.axhline(0.0, color="red", linestyle="--", linewidth=1.4)
    ax_clear.set_title("corridor clearance vs time")
    ax_clear.set_xlabel("t [s]")
    ax_clear.set_ylabel(CLEARANCE_LABEL, fontsize=8)
    ax_clear.grid(True, alpha=0.3)

    for run in runs:
        good = np.isfinite(run.t) & np.isfinite(run.speed)
        if good.sum() >= 2:
            ax_speed.plot(
                run.t[good], run.speed[good], color=run.color,
                linewidth=0.9, alpha=0.4 if not run.evaluated else 0.7,
            )
    ax_speed.set_title("speed vs time (manual verdict: green passed, "
                       "red failed, grey not evaluated)")
    ax_speed.set_xlabel("t [s]")
    ax_speed.set_ylabel("speed [m/s]")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend(
        handles=[
            Line2D([], [], color=PASS_COLOR, label="passed"),
            Line2D([], [], color=FAIL_COLOR, label="failed"),
            Line2D([], [], color=UNEVALUATED_COLOR, label="not evaluated"),
        ],
        loc="best", fontsize=8,
    )

    latencies = [x for r in runs for x in r.latencies]
    if latencies:
        ax_llm.hist(latencies, bins=min(40, max(5, len(latencies) // 3)),
                    color="#4c78c8", edgecolor="white")
        median = statistics.median(latencies)
        p95 = percentile(latencies, 95)
        ax_llm.axvline(median, color="black", linestyle="--", linewidth=1.4,
                       label=f"median {median:.0f} ms")
        ax_llm.axvline(p95, color=FAIL_COLOR, linestyle=":", linewidth=1.8,
                       label=f"p95 {p95:.0f} ms")
        ax_llm.legend(loc="best", fontsize=8)
    else:
        ax_llm.text(0.5, 0.5, "no LLM calls recorded",
                    ha="center", va="center", transform=ax_llm.transAxes)
    ax_llm.set_title(f"LLM latency ({len(latencies)} calls)")
    ax_llm.set_xlabel("latency [ms]")
    ax_llm.set_ylabel("calls")
    ax_llm.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return out_path


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Analyse an F1TENTH LLM test campaign."
    )
    parser.add_argument(
        "campaign_folder", nargs="?", default=None,
        help="campaign folder (default: <f1tenth_more>/%s)" % DEFAULT_CAMPAIGN,
    )
    parser.add_argument("--mission", default=None, help="keep only this mission")
    parser.add_argument("--prompt", type=int, default=None,
                        help="keep only this prompt_num")
    parser.add_argument("--footprints", type=float, default=None, metavar="METERS",
                        help="draw a faint footprint circle every N metres")
    parser.add_argument("--out", default=None,
                        help="output folder (default: <campaign>/analysis)")
    return parser.parse_args(argv)


def resolve_campaign(argument):
    if argument:
        path = Path(argument).expanduser().resolve()
        if not path.is_dir():
            raise NotADirectoryError(f"campaign folder not found: {path}")
        return path
    return find_root() / DEFAULT_CAMPAIGN


def main(argv=None):
    args = parse_args(argv)
    campaign_dir = resolve_campaign(args.campaign_folder)
    if not campaign_dir.is_dir():
        raise NotADirectoryError(f"campaign folder not found: {campaign_dir}")
    print(f"campaign: {campaign_dir}")

    runs, has_manual, unreadable = load_campaign(campaign_dir)
    if args.mission:
        runs = [r for r in runs if r.mission == args.mission]
    if args.prompt is not None:
        runs = [r for r in runs if r.prompt_num == args.prompt]
    if not runs:
        print(f"no test folders matched in {campaign_dir}")
        return 1

    out_dir = Path(args.out).expanduser() if args.out else campaign_dir / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = build_summary(runs)
    report = format_report(campaign_dir, runs, rows, has_manual, unreadable)
    print(report)
    (out_dir / "report.txt").write_text(report + "\n", encoding="utf-8")

    summary_csv = out_dir / "summary.csv"
    with open(summary_csv, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ("" if v is None else v) for k, v in row.items()})

    written = [summary_csv, out_dir / "report.txt"]
    for mission in sorted({r.mission for r in runs}):
        mission_runs = [r for r in runs if r.mission == mission]
        written.append(
            overview_map(
                mission_runs, mission,
                out_dir / f"overview_map_{mission}.png",
                footprints=args.footprints,
            )
        )
    written.append(
        overview_map(
            runs, "all missions", out_dir / "overview_map.png",
            footprints=args.footprints,
        )
    )
    written.append(dashboard(runs, out_dir / "dashboard.png"))

    print("")
    print(f"written to {out_dir}:")
    for path in written:
        print(f"  {Path(path).name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
