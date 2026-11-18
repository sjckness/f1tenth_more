"""Tables and aggregates for the ``analysis_v2`` pipeline (TABLES node).

Writes, inside an ``<output_folder>/`` given by the caller (ENTRYPOINT/TESTS,
never created by importing this module -- RULES.md §9 / the card):
``results.csv``, ``runs_full.csv``, ``groups.csv``, ``data_quality.csv``,
``attempts.csv``, ``old_vs_new.csv``, ``build_info.json`` and a finalized
``manual_verdicts.yaml``.

It never recomputes a metric: every number comes from
:func:`scripts.metrics.compute_run_metrics` (METRICS node, already
validated). This module only shapes that output into the files RULES.md §9
describes, plus the small bits of bookkeeping RULES.md §2 (attempts) and
§10.2 (old vs new) ask for.

Conventions kept from RULES.md §1: "missing" is ``None``/empty cell with a
reason, never ``0``; every number is traceable (``build_info.json``); CSVs
use ``,`` as separator and ``.`` as decimal point (the card, not the Italian
``;``/``,`` dialect of ``campaign_results.csv``, which stays read-only).
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import yaml

try:  # package import: ``from scripts.tables import ...``
    from .metrics import (
        G_MPS2,
        JERK_CUTOFF_HZ,
        LATERAL_MAX_M,
        PERSON_TARGET_M,
        PERSON_TOL_M,
        PLANT_MAX_GAP_M,
        STRAIGHT_TOL_M,
        TURN_TARGET_DEG,
        TURN_TOL_DEG,
        WALL_APPROACH_TARGET_M,
        WALL_APPROACH_TOL_M,
        WALL_TURN_START_TARGET_M,
        WALL_TURN_START_TOL_M,
        aggregate_group,
        compute_run_metrics,
        load_manual_verdicts,
    )
    from .run_data import (
        DEFAULT_CAMPAIGN,
        AmbiguousRunError,
        _csv_decimal_comma_float,
        _read_campaign_results_row,
        find_root,
        load_selection,
        parse_test_id,
        search_run,
    )
    from .video_mask import SEG_BRANCH, SEG_COMMIT, SEG_FILE
except ImportError:  # direct execution: ``python3 scripts/tables.py``
    from metrics import (  # type: ignore[no-redef]
        G_MPS2,
        JERK_CUTOFF_HZ,
        LATERAL_MAX_M,
        PERSON_TARGET_M,
        PERSON_TOL_M,
        PLANT_MAX_GAP_M,
        STRAIGHT_TOL_M,
        TURN_TARGET_DEG,
        TURN_TOL_DEG,
        WALL_APPROACH_TARGET_M,
        WALL_APPROACH_TOL_M,
        WALL_TURN_START_TARGET_M,
        WALL_TURN_START_TOL_M,
        aggregate_group,
        compute_run_metrics,
        load_manual_verdicts,
    )
    from run_data import (  # type: ignore[no-redef]
        DEFAULT_CAMPAIGN,
        AmbiguousRunError,
        _csv_decimal_comma_float,
        _read_campaign_results_row,
        find_root,
        load_selection,
        parse_test_id,
        search_run,
    )
    from video_mask import SEG_BRANCH, SEG_COMMIT, SEG_FILE  # type: ignore[no-redef]

__all__ = [
    "RESULTS_METRIC_COLS",
    "RESULTS_CSV_HEADER",
    "G_NUMBER_BY_MISSION_OSTACOLO",
    "OLD_VS_NEW_SPECS",
    "compute_all_run_metrics",
    "build_results_csv",
    "build_runs_full_csv",
    "build_groups_csv",
    "build_data_quality_csv",
    "build_attempts_csv",
    "build_old_vs_new_csv",
    "build_build_info_json",
    "finalize_manual_verdicts",
    "build_all",
]

# --------------------------------------------------------------------------
# constants
# --------------------------------------------------------------------------

#: RULES.md §6: old sheet "cmd N" / "G<n>" number for each (mission_code,
#: ostacolo) pair. G8 (potted plant + obstacle) has no run in the elenco
#: (RULES §6) so it is intentionally absent here. G11 covers both M06 and
#: M08 (RULES §12 D4: separated by mission, same old cmd number).
G_NUMBER_BY_MISSION_OSTACOLO = {
    ("M01", False): "G1",
    ("M02", False): "G2",
    ("M03", False): "G3",
    ("M04", False): "G4",
    ("M05", False): "G5",
    ("M01", True): "G6",
    ("M02", True): "G7",
    ("M04", True): "G9",
    ("M05", True): "G10",
    ("M06", False): "G11",
    ("M08", False): "G11",
    ("M07", False): "G12",
}

#: Task-required columns of results.csv (card, in this exact order), after
#: the "old sheet" columns (Phase, Cmd #, Command text, file, obstacle, Rep).
RESULTS_METRIC_COLS = [
    "success", "plan_ok", "exec_ok", "safe", "task_ok", "goal_error_m",
    "lateral_error_max_m", "viol_fixed_pct", "min_clear_any_m",
    "min_clear_after_start_m", "solver_usable_pct", "unusable_streak_max_s",
    "jerk_rms_mps3", "steer_direction_rev_per_m",
]

RESULTS_CSV_HEADER = (
    ["Phase", "Cmd #", "Command text", "file", "obstacle", "Rep"]
    + RESULTS_METRIC_COLS + ["note"]
)

#: RULES §5 three-level outcome columns, used by groups.csv.
_OUTCOME_COLS = ("plan_ok", "exec_ok", "safe", "task_ok", "success")

#: RULES §4 data-quality flags (+ their ``_why``), one row per run in
#: data_quality.csv.
_DQ_FLAG_COLS = [
    "pose_valid", "pose_valid_why",
    "clearance_coverage_pct", "clearance_coverage_pct_why",
    "clearance_source", "clearance_partial",
    "imu_valid", "imu_valid_why",
    "solver_valid", "solver_valid_why",
    "corridor_schema", "corridor_schema_why",
]

#: Run files read for build_info.json checksums (RULES §1.3 "ogni numero e'
#: tracciabile"). Not every run has every file (e.g. G7 only has the 5 CSVs,
#: RULES §11); missing files are simply absent from the checksum dict.
_RUN_FILES_FOR_CHECKSUM = (
    "meta.json", "plan.json", "events.jsonl", "kinematics.csv", "imu.csv",
    "commands.csv", "mpc.csv", "corridors.jsonl", "llm_calls.jsonl",
    "llm_calls.csv", "tracks.jsonl", "horizon.jsonl",
)

#: RULES §10.2 ("dove la definizione non cambia, il valore coincide") +
#: RULES §7 (metrics kept explicitly "for comparison"/"as diagnostic").
#: ``(old_column, new_column, comparability, note, old_value_converter)``.
#: ``old_column``/campaign_results.csv values are read-only (RULES §1/Fuori
#: scope); this only compares, never writes back.
OLD_VS_NEW_SPECS = [
    ("drive_duration_s", "drive_duration_s", "same",
     "stessa definizione (durata della finestra drive)", None),
    ("llm_latency_ms", "llm_latency_ms", "same",
     "stessa definizione (media latenza LLM sulle call ok, RULES §7)", None),
    ("jerk_rms", "jerk_rms_5hz_mps3", "same_unit_converted",
     "stessa definizione (RMS jerk, Butterworth ordine 2, cutoff 5 Hz, finestra "
     "moving, RULES §7); il vecchio valore e' in g/s (export_settings.json "
     "cutoff_hz=5.0), il nuovo in m/s^3: vecchio*9.80665 per confrontare",
     lambda v: v * G_MPS2),
    ("steer_rev_per_m", "steer_sign_rev_per_m", "same",
     "RULES §7: steer_sign_rev_per_m e' la vecchia metrica (inversioni di segno), "
     "tenuta per confronto", None),
    ("viol_rate_pct", "viol_rebuilt_pct", "same",
     "RULES §7: viol_rebuilt_pct e' la vecchia metrica (corridoio ricostruito ad "
     "ogni tick), tenuta come diagnostica", None),
    ("horizon_err_mean_m", "horizon_err_mean_m", "same", "stesso nome e definizione", None),
    ("horizon_err_max_m", "horizon_err_max_m", "same", "stesso nome e definizione", None),
    ("mpc_solve_time_p95_ms", "solve_time_p95_ms", "same",
     "stessa definizione (p95 di solve_time_ms su drive)", None),
    ("feas_pct", "solver_usable_pct", "changed",
     "RULES §7: la vecchia Feas. contava solved_inaccurate come infattibile; "
     "solver_usable_pct = solved + solved_inaccurate (confermato nel controller, "
     "METRICS); i valori NON devono coincidere", None),
    ("min_clear_raw_m", "min_clear_any_m", "maybe_same",
     "possibile stessa definizione (minimo della clearance grezza su drive), ma "
     "solo per clearance_source=live; per clearance_source=backfill il confronto "
     "corretto e' con backfill_min_clear_raw_m (vedi riga dedicata)", None),
    ("min_clear_m", "backfill_min_clear_m", "same_by_construction",
     "per clearance_source=backfill il valore nuovo e' preso direttamente da "
     "questa stessa colonna di campaign_results.csv (RULES §4): coincide per "
     "costruzione, non e' una verifica indipendente", None),
    ("min_clear_raw_m", "backfill_min_clear_raw_m", "same_by_construction",
     "idem, colonna raw (RULES §4 backfill)", None),
]


# --------------------------------------------------------------------------
# small formatting helpers
# --------------------------------------------------------------------------

def _fmt(value):
    """Cell text: ``None`` -> empty, float -> 6 significant decimals, ``.`` point."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "True" if value else "False"
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return repr(round(value, 6))
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, default=str, sort_keys=True)
    return str(value)


def _yes_no(value):
    """RULES columns named in English (``obstacle``, old sheet style): yes/no.

    No "si/no" source survives for the old sheet (not accessible, see
    manual_verdicts.yaml bozza); "yes"/"no" is the English-column default
    (assumption, see card report), distinct from selection.yaml's own
    Italian "si"/"no".
    """
    if value is True:
        return "yes"
    if value is False:
        return "no"
    return ""


def _mission_code(mission):
    return (mission or "").split("_")[0]


def _repetition(test_id):
    try:
        return parse_test_id(test_id)["repetition"]
    except (ValueError, TypeError):
        return None


def _cmd_sort_key(cmd):
    m = re.match(r"G(\d+)", cmd or "")
    return int(m.group(1)) if m else 999


def _auto_header(records, priority=()):
    cols = set()
    for rec in records:
        cols.update(rec.keys())
    rest = sorted(c for c in cols if c not in priority)
    return [c for c in priority if c in cols] + rest


def _write_csv(path, header, records):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)  # default dialect: comma separator
        writer.writerow(header)
        for rec in records:
            writer.writerow([_fmt(rec.get(col)) for col in header])


def _result_note(row):
    bits = []
    why = row.get("success_why")
    if why:
        bits.append(why)
    null_cols = [c for c in RESULTS_METRIC_COLS if row.get(c) is None]
    if null_cols:
        bits.append("null: " + ",".join(null_cols) + " (motivo in data_quality.csv)")
    return "; ".join(bits)


# --------------------------------------------------------------------------
# run resolution + metrics (uses METRICS/RUNDATA_LIB, never recomputes)
# --------------------------------------------------------------------------

def compute_all_run_metrics(selection, root=None, verdicts=None):
    """``(rows, errors)``: :func:`compute_run_metrics` for every selected row.

    ``selection`` is a :class:`run_data.Selection` (``load_selection``).
    Rows with ``metriche=no`` are skipped (nothing to compute for them; they
    still appear in ``attempts.csv``). One run failing to resolve/compute
    never stops the batch (its code lands in ``errors`` instead), matching
    RULES.md's "un fallimento dell'auto non e' un motivo di esclusione" and
    the batch-robustness ENTRYPOINT also requires.
    """
    root = Path(root) if root else find_root()
    verdicts = verdicts if verdicts is not None else load_manual_verdicts()
    rows, errors = [], []
    for row in selection.rows:
        if not row.metriche:
            continue
        try:
            run_dir = search_run(row.run)
        except AmbiguousRunError as exc:
            errors.append({"run": row.run, "error": str(exc)})
            continue
        if run_dir is None:
            errors.append({"run": row.run, "error": f"run not found: {row.run!r}"})
            continue
        try:
            metrics_row = compute_run_metrics(run_dir, ostacolo=row.ostacolo, verdicts=verdicts)
        except Exception as exc:  # never let one bad run stop the batch
            errors.append({"run": row.run, "error": repr(exc)})
            continue
        metrics_row["selezionate"] = row.selezionate
        rows.append(metrics_row)
    return rows, errors


# --------------------------------------------------------------------------
# results.csv (old "first test campaing" sheet layout, card)
# --------------------------------------------------------------------------

def build_results_csv(rows, path, g_number_map=None):
    """Old-sheet-style table (card): one row per run, sorted by cmd/phase/rep.

    ``Phase`` = mission folder name (e.g. ``M01_forward_3m``); ``Cmd #`` =
    the ``G<n>`` number of RULES.md §6/§11 (assumption: the old sheet's "cmd
    N" numbering, which RULES.md itself uses verbatim for "cmd 8" -- see
    card report). ``obstacle`` is yes/no, ``Rep`` the repetition number.
    """
    g_number_map = g_number_map or G_NUMBER_BY_MISSION_OSTACOLO
    records = []
    for row in rows:
        mcode = _mission_code(row.get("mission"))
        ostacolo = row.get("ostacolo")
        cmd = g_number_map.get((mcode, bool(ostacolo))) if ostacolo is not None else None
        rec = {
            "Phase": row.get("mission"),
            "Cmd #": cmd,
            "Command text": row.get("prompt_text"),
            "file": row.get("test_id"),
            "obstacle": _yes_no(ostacolo),
            "Rep": _repetition(row.get("test_id")),
        }
        for col in RESULTS_METRIC_COLS:
            rec[col] = row.get(col)
        rec["note"] = _result_note(row)
        records.append(rec)
    records.sort(key=lambda rec: (
        _cmd_sort_key(rec["Cmd #"]),
        rec["Phase"] or "",
        rec["Rep"] if rec["Rep"] is not None else -1,
    ))
    _write_csv(path, RESULTS_CSV_HEADER, records)
    return records


# --------------------------------------------------------------------------
# runs_full.csv (every metric, RULES §9)
# --------------------------------------------------------------------------

def build_runs_full_csv(rows, path):
    """Every key :func:`compute_run_metrics` produced, one row per run."""
    header = _auto_header(rows, priority=["test_id", "mission", "mission_code", "ostacolo"])
    _write_csv(path, header, rows)
    return header


# --------------------------------------------------------------------------
# groups.csv (RULES §8 aggregation)
# --------------------------------------------------------------------------

def build_groups_csv(rows, path):
    """Per mission+ostacolo group: Wilson 95% for the RULES §5 outcomes,
    median/min/max (valid N) for every continuous metric, plus
    ``n_selected``/``n_runs`` (RULES §8 "k/N selezionate e k/N tentativi
    validi" -- the "tentativi validi" half lives in attempts.csv, joined by
    mission+ostacolo there).
    """
    groups = {}
    for row in rows:
        key = (row.get("mission"), row.get("ostacolo"))
        groups.setdefault(key, []).append(row)

    records = []
    for (mission, ostacolo), grows in groups.items():
        agg = aggregate_group(grows)
        rec = {
            "mission": mission,
            "ostacolo": _yes_no(ostacolo),
            "n_runs": agg["n_runs"],
            "n_selected": sum(1 for r in grows if r.get("selezionate")),
            "mixed_code": agg["mixed_code"],
            "git_commits": "|".join(agg["git_commits"]),
        }
        for col in _OUTCOME_COLS:
            b = agg[col]
            rec[f"{col}_k"] = b["k"]
            rec[f"{col}_n"] = b["n"]
            rec[f"{col}_n_null"] = b["n_null"]
            rec[f"{col}_wilson95_lo"] = b["wilson95_lo"]
            rec[f"{col}_wilson95_hi"] = b["wilson95_hi"]
        skip = set(_OUTCOME_COLS) | {"n_runs", "git_commits", "mixed_code", "ostacolo_values"}
        for key2, val in agg.items():
            if key2 in skip or not isinstance(val, dict):
                continue
            for subk in ("median", "min", "max", "n_valid", "n_null"):
                rec[f"{key2}_{subk}"] = val.get(subk)
        records.append(rec)

    records.sort(key=lambda rec: (rec["mission"] or "", rec["ostacolo"]))
    header = _auto_header(records, priority=["mission", "ostacolo", "n_runs", "n_selected"])
    _write_csv(path, header, records)
    return records


# --------------------------------------------------------------------------
# data_quality.csv (RULES §4 flags + "motivo di ogni null")
# --------------------------------------------------------------------------

def build_data_quality_csv(rows, path):
    records = []
    for row in rows:
        rec = {"test_id": row.get("test_id"), "mission": row.get("mission"),
               "ostacolo": _yes_no(row.get("ostacolo"))}
        for col in _DQ_FLAG_COLS:
            rec[col] = row.get(col)
        null_items = []
        for key, val in row.items():
            if val is None and not key.endswith("_why"):
                why = row.get(f"{key}_why")
                if why:
                    null_items.append(f"{key}: {why}")
        rec["null_metrics"] = "; ".join(sorted(null_items))
        records.append(rec)
    header = ["test_id", "mission", "ostacolo"] + _DQ_FLAG_COLS + ["null_metrics"]
    _write_csv(path, header, records)
    return records


# --------------------------------------------------------------------------
# attempts.csv (RULES §2: every attempt, including the not-selected ones)
# --------------------------------------------------------------------------

#: Commented-out row inside a ``runs:`` block of selection.yaml, e.g.
#: ``  # - [P005-R016-..., si, si, si, si, si, si]  # note text``.
_COMMENTED_ROW_RE = re.compile(r"^\s*#\s*-\s*\[\s*([^,\]]+?)\s*,.*?\]\s*(?:#\s*(.*))?$")
#: Section header comment, e.g. ``  # --- G10 M05_turn_left_wall + ostacolo ...``.
_SECTION_HEADER_RE = re.compile(r"^\s*#\s*---\s*G\d+\s+(\S+)")


def _parse_selection_comments(selection_path):
    """Commented-out candidate rows in ``selection.yaml`` (RULES §2/§12 D6).

    These are real attempts SCOUT_DATA already looked at but left out of the
    active ``runs:`` list (e.g. G10's un-activated rows). Status is guessed
    from the free-text note SCOUT_DATA wrote next to each row; default is
    ``not_selected`` with that note as the mandatory reason.
    """
    text = Path(selection_path).read_text(encoding="utf-8")
    mission_hint = None
    out = []
    for line in text.splitlines():
        header = _SECTION_HEADER_RE.match(line)
        if header:
            mission_hint = header.group(1)
            continue
        match = _COMMENTED_ROW_RE.match(line)
        if not match:
            continue
        run_code = match.group(1).strip()
        note = (match.group(2) or "").strip()
        status = "not_selected"
        low = note.lower()
        if "mai partita" in low or "not_started" in low:
            status = "not_started"
        elif "log tronc" in low or "log_invalid" in low:
            status = "log_invalid"
        reason = note or "riga commentata in selection.yaml (non attivata)"
        out.append({
            "run": run_code, "mission": mission_hint, "ostacolo": "yes",
            "status": status, "reason": reason,
            "source": "selection.yaml (riga commentata, RULES.md §12 D6)",
        })
    return out


#: Extra attempts documented in RULES.md §11 / selection.yaml's own header
#: comments, for runs that are neither an active ``runs:`` row nor a
#: commented G10 candidate (so :func:`_parse_selection_comments` cannot find
#: them). Hardcoded because the source is prose, not a parseable row.
KNOWN_EXTRA_ATTEMPTS = [
    {"run": "P002-R001-20261008T132715", "mission": "M02_approach_wall", "ostacolo": "no",
     "status": "not_selected",
     "reason": ("sessione duplicata di P002-R001 (8/10/2026); per la v2 vale la sessione "
                "del 22/09 (RULES.md §11, nota finale)"),
     "source": "RULES.md §11 + selection.yaml (commento G2)"},
    # .sjck/interfaces/analysis_v2_data.md §1: "Id solo in campaign_results.csv, senza
    # cartella locale (non selezionati)" -- le 3 righe elencate lì, stesso pattern.
    {"run": "P002-R001-20260921T150345", "mission": "M02_approach_wall", "ostacolo": "no",
     "status": "log_invalid",
     "reason": "riga presente solo in campaign_results.csv, nessuna cartella run trovata su disco",
     "source": ".sjck/interfaces/analysis_v2_data.md §1"},
    {"run": "P001-R006-20260922T100451", "mission": "M01_forward_3m", "ostacolo": "no",
     "status": "log_invalid",
     "reason": "riga presente solo in campaign_results.csv, nessuna cartella run trovata su disco",
     "source": ".sjck/interfaces/analysis_v2_data.md §1"},
    {"run": "P003-R001-20260922T143542", "mission": "M03_bottle", "ostacolo": "no",
     "status": "log_invalid",
     "reason": "riga presente solo in campaign_results.csv, nessuna cartella run trovata su disco",
     "source": ".sjck/interfaces/analysis_v2_data.md §1"},
    {"run": "P007-R001-20261027T143116", "mission": "M07_stop_at_chair", "ostacolo": "no",
     "status": "not_selected",
     "reason": ("stessa run duplicata anche in "
                "M07_chair_then_straight_4m/P007-R001-20261027T165917 (missione richiesta "
                "dal task): usata quella cartella, non questa"),
     "source": "selection.yaml (commento G12)"},
]


def build_attempts_csv(selection, rows_by_test_id, path, extra_attempts=None):
    """RULES §2: every attempt of the selected sessions, selected or not."""
    extra_attempts = KNOWN_EXTRA_ATTEMPTS if extra_attempts is None else extra_attempts
    records = []
    for row in selection.rows:
        computed = rows_by_test_id.get(row.run)
        if not row.metriche:
            status, reason = "not_selected", "metriche=no in selection.yaml (nessuna metrica calcolata)"
        elif row.selezionate:
            status, reason = "selected", None
        else:
            status, reason = "not_selected", "selezionate=no in selection.yaml"
        records.append({
            "run": row.run,
            "mission": computed.get("mission") if computed else None,
            "ostacolo": _yes_no(row.ostacolo),
            "status": status,
            "reason": reason,
            "source": "selection.yaml",
        })
    for extra in extra_attempts:
        records.append(dict(extra))
    header = ["run", "mission", "ostacolo", "status", "reason", "source"]
    _write_csv(path, header, records)
    return records


# --------------------------------------------------------------------------
# old_vs_new.csv (RULES §10.2)
# --------------------------------------------------------------------------

def build_old_vs_new_csv(rows, path, campaign_dir):
    """One row per (run, comparison spec) in :data:`OLD_VS_NEW_SPECS`.

    ``campaign_dir`` is ``first_test_campaign/`` (read-only: only
    ``campaign_results.csv`` is read from it, never written -- Fuori scope).
    """
    records = []
    for row in rows:
        test_id = row.get("test_id")
        old_row = _read_campaign_results_row(campaign_dir, test_id)
        for old_col, new_col, kind, note, convert in OLD_VS_NEW_SPECS:
            old_raw = old_row.get(old_col) if old_row else None
            old_val = _csv_decimal_comma_float(old_raw) if old_row else None
            old_cmp = convert(old_val) if (convert and old_val is not None) else old_val
            new_val = row.get(new_col)
            diff = (new_val - old_cmp) if (old_cmp is not None and new_val is not None) else None
            records.append({
                "test_id": test_id, "mission": row.get("mission"),
                "old_column": old_col, "new_column": new_col,
                "old_value": old_val, "old_value_converted": old_cmp, "new_value": new_val,
                "diff": diff, "comparability": kind, "note": note,
            })
    header = ["test_id", "mission", "old_column", "new_column", "old_value",
              "old_value_converted", "new_value", "diff", "comparability", "note"]
    _write_csv(path, header, records)
    return records


# --------------------------------------------------------------------------
# build_info.json (RULES §1.3 traceability)
# --------------------------------------------------------------------------

def _git_commit(root):
    try:
        res = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=True, timeout=10,
        )
        return res.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def _checksum_sha256(path):
    h = hashlib.sha256()
    try:
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 16), b""):
                h.update(chunk)
    except OSError:
        return None
    return h.hexdigest()


def build_build_info_json(rows, path, root, selection_path=None):
    """Repo commit, segmentation pin (SCOUT_PERCEPTION), params and per-file
    checksums of every run read -- RULES §1.3 "ogni numero e' tracciabile".

    The segmentation commit/branch is kept for traceability even though this
    node generates no video/PNG (card: "tienilo per coerenza/tracciabilita'").
    """
    root = Path(root)
    checksums = {}
    for row in rows:
        run_dir = row.get("run_dir")
        if not run_dir:
            continue
        run_dir = Path(run_dir)
        files = {}
        for fname in _RUN_FILES_FOR_CHECKSUM:
            fpath = run_dir / fname
            if fpath.exists():
                files[fname] = _checksum_sha256(fpath)
        checksums[row.get("test_id")] = files

    info = {
        "written": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "repo_commit": _git_commit(root),
        "segmentation_source": {
            "branch": SEG_BRANCH,
            "commit": SEG_COMMIT,
            "file": SEG_FILE,
            "note": ("pin SCOUT_PERCEPTION/VIDEO_MASK, RULES.md §9; tenuto qui per "
                     "coerenza/tracciabilita', questo nodo non genera video/maschera"),
        },
        "params": {
            "jerk_cutoff_hz": JERK_CUTOFF_HZ,
            "g_mps2": G_MPS2,
            "wall_approach_target_m": WALL_APPROACH_TARGET_M,
            "wall_approach_tol_m": WALL_APPROACH_TOL_M,
            "wall_turn_start_target_m": WALL_TURN_START_TARGET_M,
            "wall_turn_start_tol_m": WALL_TURN_START_TOL_M,
            "turn_target_deg": TURN_TARGET_DEG,
            "turn_tol_deg": TURN_TOL_DEG,
            "person_target_m": PERSON_TARGET_M,
            "person_tol_m": PERSON_TOL_M,
            "straight_tol_m": STRAIGHT_TOL_M,
            "lateral_max_m": LATERAL_MAX_M,
            "plant_max_gap_m": PLANT_MAX_GAP_M,
        },
        "selection_yaml": str(selection_path) if selection_path else None,
        "n_runs": len(rows),
        "run_file_checksums_sha256": checksums,
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(info, fh, indent=2, sort_keys=True)
    return info


# --------------------------------------------------------------------------
# manual_verdicts.yaml finalization (card: keep SCOUT_DATA's entries)
# --------------------------------------------------------------------------

def finalize_manual_verdicts(src_path, dst_path, extra_note=None):
    """Copy the SCOUT_DATA bozza into ``dst_path``, marked final.

    Textual copy (not a round-trip through ``yaml.safe_load``/``safe_dump``)
    so the explanatory comment block at the top of the bozza -- the only
    place the per-run evidence table lives -- survives untouched. Only the
    ``status: draft`` line is flipped to ``final``. RULES.md §5/§10 hold no
    manual-contact/success note beyond the two already in the bozza
    (P002-R024/R026): §10's "casi noti" are data-quality notes (coverage,
    pose_valid), not contact/success overrides, so nothing else qualifies
    for ``manual_verdicts.yaml``.
    """
    text = Path(src_path).read_text(encoding="utf-8")
    note = extra_note or (
        "# FINALIZZATO (TABLES, " + datetime.now().strftime("%Y-%m-%d") + "): verdetti SCOUT_DATA "
        "per P002-R024/R026 confermati (fonte: RULES.md §5). Nessuna nota manuale aggiuntiva "
        "trovata in RULES.md §5/§10 (le \"casi noti\" di §10 sono note di qualita' dato, non "
        "override di contatto/successo). L'ambiguita' sulla riga \"26\" doppia (R026 vs R027) "
        "resta irrisolta: serve il foglio Google originale o il video (vedi 'ambiguity' sotto "
        "P002-R026 qui sotto).\n"
    )
    text = text.replace("status: draft", "status: final", 1)
    out_text = note + text
    yaml.safe_load(out_text)  # fail loudly here, not when some other reader chokes on it
    dst_path = Path(dst_path)
    dst_path.parent.mkdir(parents=True, exist_ok=True)
    dst_path.write_text(out_text, encoding="utf-8")
    return dst_path


# --------------------------------------------------------------------------
# orchestration (RULES §9 output tree; ENTRYPOINT/TESTS call this)
# --------------------------------------------------------------------------

def build_all(output_dir, selection_path=None, root=None, manual_verdicts_src=None):
    """Compute metrics for every selected run and write every RULES §9 file.

    Creates ``output_dir`` (and writes into it) -- the card's "non creare
    l'output folder qui" is about import-time side effects, not about what
    this function does when a caller (ENTRYPOINT/TESTS) actually invokes it
    with a concrete ``output_dir``.
    """
    root = Path(root) if root else find_root()
    campaign_dir = root / DEFAULT_CAMPAIGN
    selection_path = Path(selection_path) if selection_path else (
        Path(__file__).resolve().parent.parent / "selection.yaml")
    selection = load_selection(selection_path)

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    src_verdicts = manual_verdicts_src or (
        Path(__file__).resolve().parent.parent / "manual_verdicts.yaml")

    verdicts = load_manual_verdicts(src_verdicts)
    rows, resolve_errors = compute_all_run_metrics(selection, root=root, verdicts=verdicts)
    rows_by_test_id = {r["test_id"]: r for r in rows}

    build_results_csv(rows, output_dir / "results.csv")
    build_runs_full_csv(rows, output_dir / "runs_full.csv")
    build_groups_csv(rows, output_dir / "groups.csv")
    build_data_quality_csv(rows, output_dir / "data_quality.csv")
    extra_attempts = _parse_selection_comments(selection_path) + KNOWN_EXTRA_ATTEMPTS
    build_attempts_csv(selection, rows_by_test_id, output_dir / "attempts.csv",
                        extra_attempts=extra_attempts)
    build_old_vs_new_csv(rows, output_dir / "old_vs_new.csv", campaign_dir)
    build_build_info_json(rows, output_dir / "build_info.json", root, selection_path=selection_path)

    finalize_manual_verdicts(src_verdicts, output_dir / "manual_verdicts.yaml")

    return {
        "n_rows": len(rows),
        "resolve_errors": resolve_errors,
        "output_dir": str(output_dir),
    }


if __name__ == "__main__":  # debug: python3 scripts/tables.py <output_dir>
    import sys

    out = sys.argv[1] if len(sys.argv) > 1 else "risultati_v2"
    print(json.dumps(build_all(out), indent=2))
