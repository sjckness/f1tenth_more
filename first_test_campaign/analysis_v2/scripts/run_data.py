"""Shared run-data library for the ``analysis_v2`` pipeline (RUNDATA_LIB).

Resolves run codes from ``selection.yaml``, loads the raw files of a run
(``meta.json``, ``kinematics.csv``, ``imu.csv``, ``commands.csv``, ``mpc.csv``,
``events.jsonl``, ``corridors.jsonl``, ``plan.json``), computes the
``drive``/``moving``/``after_start`` windows and the 6 data-quality flags of
``RULES.md`` §4. Every later node (METRICS, VIDEO_MASK, ...) must import from
here instead of re-parsing run folders.

Scope (``RULES.md`` §1/§3/§4, see also the card):

* Read-only. Never writes into a run folder, ``analysis/``, or
  ``campaign_results.csv`` -- the latter is read (only for the
  ``backfilled`` column, to resolve ``clearance_source``, RULES.md §4) but
  never written.
* No bag/ROS reading: only the files a test already wrote to its folder,
  plus the campaign-level ``campaign_results.csv`` above.
* No derived metrics (jerk, viol_fixed_pct, solver_usable_pct, ...) and no
  PNG/video -- that is METRICS' and VIDEO_MASK's job.
* "Missing" is not "zero": a value that cannot be computed is ``None`` with a
  reason in the matching ``*_why`` field.

This module is deliberately ROS-free (no ``rclpy``, no
``f1tenth_logger`` import): ``parse_test_id``/``make_test_id``/``find_root``
below are small, self-contained re-implementations of the signatures
documented in ``.sjck/interfaces/campaign_tools.md``
(``f1tenth_logger/test_campaign/robot_logger.py``), kept API-compatible so
that behaviour matches, without requiring a sourced ROS workspace to run
``python3 -c "from scripts.run_data import ..."`` (see acceptance criteria).
"""

from __future__ import annotations

import csv
import io
import json
import math
import os
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

__all__ = [
    "DEFAULT_CAMPAIGN",
    "TEST_ID_RE",
    "parse_test_id",
    "make_test_id",
    "find_root",
    "RunResolutionError",
    "RunNotFoundError",
    "AmbiguousRunError",
    "resolve_run",
    "search_run",
    "SelectionError",
    "SelectionRow",
    "Selection",
    "load_selection",
    "RunV2",
    "load_run_v2",
    "group_key",
]

# --------------------------------------------------------------------------
# constants (RULES.md thresholds, exact values as written there)
# --------------------------------------------------------------------------

DEFAULT_CAMPAIGN = "first_test_campaign"

#: RULES.md §3: "moving" = drive samples with speed above this (m/s).
MOVING_SPEED_THRESHOLD_MPS = 0.05
#: RULES.md §3: "after_start" = drive samples after this much path (m).
AFTER_START_DISTANCE_M = 0.30
#: RULES.md §4: clearance_coverage_pct fails under this (%).
CLEARANCE_COVERAGE_FAIL_PCT = 50.0
#: RULES.md §4: between fail and this (%), the value is kept with a "partial" flag.
CLEARANCE_COVERAGE_PARTIAL_PCT = 90.0
#: RULES.md §4: imu_valid fails under this much data (s).
IMU_MIN_SECONDS = 2.0
#: RULES.md §4: imu_valid fails under this rate (Hz).
IMU_MIN_HZ = 40.0
#: RULES.md §4: the only schema that keeps corridor metrics.
CORRIDOR_SCHEMA_OK = "mpc_corr/v2"
#: Position-jump tolerance added on top of v_max*dt for pose_valid (m).
#: RULES.md does not give a number for this tolerance; chosen as a small
#: margin against odometry/logging jitter (assumption, see card report).
POSE_JUMP_TOLERANCE_M = 0.05


# --------------------------------------------------------------------------
# run ids and workspace root (ROS-free re-implementation, see module docstring)
# --------------------------------------------------------------------------

TEST_ID_RE = re.compile(r"^P(\d{3})-R(\d{3})-(\d{8}T\d{6})$")
_TS_FMT = "%Y%m%dT%H%M%S"
#: What marks the f1tenth_more workspace root (same marker as robot_logger.py).
_ROOT_MARKER = Path("src") / "f1tenth_logger" / "package.xml"


def parse_test_id(test_id):
    """Inverse of :func:`make_test_id`. Raises ``ValueError`` if malformed."""
    match = TEST_ID_RE.match(str(test_id).strip())
    if match is None:
        raise ValueError(
            f"not a test id: {test_id!r} "
            f"(expected P<nnn>-R<rrr>-<YYYYMMDDTHHMMSS>)"
        )
    return {
        "prompt_num": int(match.group(1)),
        "repetition": int(match.group(2)),
        "datetime": datetime.strptime(match.group(3), _TS_FMT),
    }


def make_test_id(prompt_num, repetition, when=None):
    """``P<nnn>-R<rrr>-<YYYYMMDDTHHMMSS>``, e.g. ``P004-R012-20260918T143512``."""
    when = when or datetime.now()
    return f"P{int(prompt_num):03d}-R{int(repetition):03d}-{when.strftime(_TS_FMT)}"


def find_root(explicit=None):
    """Locate the ``f1tenth_more`` workspace folder.

    Order: ``explicit`` argument, then ``$F1TENTH_MORE_ROOT``, then an
    upward search from this file for ``src/f1tenth_logger/package.xml``
    (same marker as ``robot_logger.find_root``, minus the ament/install
    lookup, which is irrelevant here since this module never runs under
    ``ros2 run``).
    """
    if explicit is not None and str(explicit) != "":
        root = Path(explicit).expanduser().resolve()
        if not root.is_dir():
            raise NotADirectoryError(f"root={explicit!r} is not a directory")
        return root

    env = os.environ.get("F1TENTH_MORE_ROOT")
    if env:
        root = Path(env).expanduser().resolve()
        if not root.is_dir():
            raise NotADirectoryError(
                f"F1TENTH_MORE_ROOT={env!r} is not a directory"
            )
        return root

    here = Path(__file__).resolve()
    for parent in (here, *here.parents):
        if (parent / _ROOT_MARKER).is_file():
            return parent

    raise RuntimeError(
        f"cannot locate the f1tenth_more workspace: no ancestor of {here} "
        f"holds {_ROOT_MARKER}. Pass an explicit root or set F1TENTH_MORE_ROOT."
    )


# --------------------------------------------------------------------------
# run resolution (analysis_v2_data.md §2)
# --------------------------------------------------------------------------

class RunResolutionError(Exception):
    """Base class for errors resolving a run code to a folder."""


class RunNotFoundError(RunResolutionError):
    def __init__(self, code, checked_paths):
        self.code = code
        self.checked_paths = [Path(p) for p in checked_paths]
        checked = ", ".join(str(p) for p in self.checked_paths) or "(none)"
        super().__init__(f"run {code!r} not found; checked: {checked}")


class AmbiguousRunError(RunResolutionError):
    """Raised instead of silently picking one of several matching folders."""

    def __init__(self, code, alternatives):
        self.code = code
        self.alternatives = [Path(p) for p in alternatives]
        listed = "\n  - ".join(str(p) for p in self.alternatives)
        super().__init__(
            f"run {code!r} is ambiguous, {len(self.alternatives)} matches "
            f"found (never pick one by date):\n  - {listed}"
        )


def resolve_run(code, search_paths):
    """Resolve a run ``code`` (full test id or short ``P0nn-R0nn``) to its folder.

    ``search_paths`` is a list of root directories, each expected to contain
    mission subfolders (``Mxx_.../<test_id>``), searched in the given order.
    All matches across every ``search_paths`` entry are collected before
    deciding: 0 matches -> :class:`RunNotFoundError`; more than 1 ->
    :class:`AmbiguousRunError` listing every alternative (this is the
    required behaviour for the "P002-R001 ambiguity" case: two folders with
    different session dates must both be reported, never resolved by
    picking the most recent or most local one).
    """
    code = str(code).strip()
    try:
        parse_test_id(code)
        is_full = True
    except ValueError:
        is_full = False

    matches = []
    for root in search_paths:
        root = Path(root).expanduser()
        if not root.is_dir():
            continue
        if is_full:
            matches.extend(sorted(root.glob(f"*/{code}")))
            direct = root / code
            if direct.is_dir():
                matches.append(direct)
        else:
            matches.extend(sorted(root.glob(f"*/{code}-*")))
            matches.extend(sorted(root.glob(f"{code}-*")))

    unique = []
    seen = set()
    for m in matches:
        key = m.resolve()
        if key not in seen:
            seen.add(key)
            unique.append(m)

    if not unique:
        raise RunNotFoundError(code, search_paths)
    if len(unique) > 1:
        raise AmbiguousRunError(code, unique)
    return unique[0]


def search_run(code):
    """Best-effort :func:`resolve_run` over the documented default locations.

    Order (``analysis_v2_data.md`` §1, card API): ``first_test_campaign/``,
    then ``~/f1tenth_archive/{complete,active,incomplete}``. No other Orin
    data path is known/mounted from this machine (checked on 2026-11-02:
    no ``/media`` or ``/mnt`` mount holding an archive, no
    ``F1TENTH_MORE_ORIN_ROOT``); if that variable is set it is appended as an
    extra search root. Returns ``None`` only when the code is not found
    anywhere; an ambiguity is never swallowed, it still raises
    :class:`AmbiguousRunError`.
    """
    root = find_root()
    home = Path.home()
    search_paths = [
        root / DEFAULT_CAMPAIGN,
        home / "f1tenth_archive" / "complete",
        home / "f1tenth_archive" / "active",
        home / "f1tenth_archive" / "incomplete",
    ]
    extra = os.environ.get("F1TENTH_MORE_ORIN_ROOT")
    if extra:
        search_paths.append(Path(extra))

    try:
        return resolve_run(code, search_paths)
    except RunNotFoundError:
        return None


# --------------------------------------------------------------------------
# selection.yaml (RULES.md §2, row format fixed by the orchestrator note in
# analysis_v2_data.md §7: positional list, not the dict schema described
# earlier in that same document)
# --------------------------------------------------------------------------

#: Order of the 6 si/no fields after the run code, per selection.yaml's header.
_SELECTION_BOOL_FIELDS = (
    "ostacolo",
    "metriche",
    "png",
    "selezionate",
    "maschera",
    "dall_alto",
)


class SelectionError(ValueError):
    """Raised on a malformed selection.yaml (empty/non si-no field, bad row)."""


@dataclass
class SelectionRow:
    run: str
    ostacolo: bool
    metriche: bool
    png: bool
    selezionate: bool
    maschera: bool
    dall_alto: bool
    index: int  # 1-based position among non-commented rows, for error messages


@dataclass
class Selection:
    output_folder: str
    rows: list = field(default_factory=list)

    def __iter__(self):
        return iter(self.rows)

    def __len__(self):
        return len(self.rows)


def _normalize_si_no(value):
    """``True``/``"si"`` -> ``"si"``; ``False``/``"no"`` -> ``"no"``; else ``None``.

    selection.yaml writes these fields unquoted (``si``/``no``, not
    ``"si"``/``"no"``). PyYAML's default (1.1) resolver reads the bareword
    ``no`` (and ``yes``/``true``/``false``/``on``/``off``) as a Python bool,
    not as the string ``"no"`` -- same family of bug as "Launch coerces
    'off' to False", here on the YAML loader instead of
    ``LaunchConfiguration``. ``si`` is not an English yes/no word so PyYAML
    leaves it as the string ``"si"``. This function normalizes both
    representations before validation ever sees the value, so it never
    assumes the value is already a string.
    """
    if isinstance(value, bool):
        return "si" if value else "no"
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("si", "no"):
            return lowered
    return None


def load_selection(path):
    """Parse ``selection.yaml`` into a :class:`Selection`.

    Row format (fixed, see module header comment in the file itself)::

        runs:
          - [run, ostacolo(si|no), metriche(si|no), png(si|no),
             selezionate(si|no), maschera(si|no), dall_alto(si|no)]

    Commented-out rows (``# - [...]``) are plain YAML comments and are
    already skipped by the parser -- they never reach this function.
    Every si/no field is validated explicitly: empty or anything other than
    si/no raises :class:`SelectionError` naming the row and field.
    """
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}

    output_folder = raw.get("output_folder")
    if not isinstance(output_folder, str) or not output_folder.strip():
        raise SelectionError(f"{path}: 'output_folder' is missing or empty")

    raw_runs = raw.get("runs") or []
    rows = []
    for i, entry in enumerate(raw_runs, start=1):
        if not isinstance(entry, (list, tuple)) or len(entry) != 7:
            raise SelectionError(
                f"{path}: row {i} malformed, expected 7 fields "
                f"[run, ostacolo, metriche, png, selezionate, maschera, "
                f"dall_alto], got {entry!r}"
            )
        run_code = entry[0]
        if not isinstance(run_code, str) or not run_code.strip():
            raise SelectionError(
                f"{path}: row {i}, 'run' field is empty or not a string: {entry!r}"
            )
        run_code = run_code.strip()

        values = {}
        for field_name, raw_value in zip(_SELECTION_BOOL_FIELDS, entry[1:]):
            normalized = _normalize_si_no(raw_value)
            if normalized is None:
                raise SelectionError(
                    f"{path}: row {i} ({run_code}), field '{field_name}' is "
                    f"empty or not si/no: {raw_value!r}"
                )
            values[field_name] = normalized == "si"

        rows.append(SelectionRow(run=run_code, index=i, **values))

    return Selection(output_folder=output_folder.strip(), rows=rows)


# --------------------------------------------------------------------------
# raw file readers (no bag/ROS; meta.json, *.csv, *.jsonl as already extracted)
# --------------------------------------------------------------------------

def _read_jsonl(path):
    """Parse a JSONL file, skipping blank/NUL-padded/malformed lines.

    Tolerates the known-bad cases from analysis_v2_data.md §5: NUL padding
    (truncated logs, e.g. P005-R017) and the occasional non-JSON line
    (tracks.jsonl on the same run). A line that is not valid JSON is simply
    skipped, never raised -- a partially-truncated file is still readable
    data, per RULES.md "absent does not mean zero".
    """
    records = []
    if not path.exists():
        return records
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip("\x00").strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return records


def _csv_dict_rows(path):
    """``csv.DictReader`` rows, tolerant of NUL padding (truncated logs, e.g. P005-R017).

    The stdlib ``csv`` module refuses a line containing a NUL byte
    (``_csv.Error: line contains NUL``); a truncated log padded to a 4096
    byte boundary is exactly that, so the NUL bytes are stripped before
    parsing. The file is still read in full first, not streamed, but these
    are per-run CSVs (tens of thousands of rows at most), not bags.
    """
    with open(path, encoding="utf-8", errors="replace") as fh:
        text = fh.read()
    if "\x00" in text:
        text = text.replace("\x00", "")
    return csv.DictReader(io.StringIO(text))


def _read_csv_numeric(path, columns):
    """``{column: float ndarray}``, NaN for empty/unparsable cells; missing file -> empty arrays."""
    out = {c: [] for c in columns}
    if not path.exists():
        return {c: np.empty(0) for c in columns}
    for row in _csv_dict_rows(path):
        for col in columns:
            raw = (row.get(col) or "").strip()
            try:
                out[col].append(float(raw) if raw else math.nan)
            except ValueError:
                out[col].append(math.nan)
    return {c: np.asarray(v, dtype=float) for c, v in out.items()}


def _read_csv_mixed(path, numeric_cols, string_cols):
    """Like :func:`_read_csv_numeric`, plus string columns (e.g. mpc status)."""
    numeric = {c: [] for c in numeric_cols}
    strings = {c: [] for c in string_cols}
    if not path.exists():
        out = {c: np.empty(0) for c in numeric_cols}
        out.update({c: np.asarray([], dtype=object) for c in string_cols})
        return out
    for row in _csv_dict_rows(path):
        for col in numeric_cols:
            raw = (row.get(col) or "").strip()
            try:
                numeric[col].append(float(raw) if raw else math.nan)
            except ValueError:
                numeric[col].append(math.nan)
        for col in string_cols:
            strings[col].append((row.get(col) or "").strip())
    out = {c: np.asarray(v, dtype=float) for c, v in numeric.items()}
    out.update({c: np.asarray(v, dtype=object) for c, v in strings.items()})
    return out


def _read_meta(run_dir):
    meta_path = run_dir / "meta.json"
    if not meta_path.exists():
        return {}
    try:
        with open(meta_path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, json.JSONDecodeError):
        return {}


def _read_plan(run_dir):
    plan_path = run_dir / "plan.json"
    if not plan_path.exists():
        return None
    try:
        with open(plan_path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None


# --------------------------------------------------------------------------
# RunV2 and its windows/flags (RULES.md §3/§4)
# --------------------------------------------------------------------------

@dataclass
class RunV2:
    """Raw data + computed windows/flags for one run, read-only.

    ``clearance`` is the LiDAR-to-footprint signal RULES.md §4/§7 talks
    about (``kinematics.csv``'s ``obstacle_clearance``); ``corridor_clearance``
    (distance to the MPC corridor boundary, a different thing, used for the
    ``viol_*`` metrics) is kept alongside it since METRICS needs both.

    When ``clearance_source=="backfill"`` (live coverage below 50%, but a
    ``campaign_results.csv`` backfill tag exists for this test_id, e.g.
    P001-R001...R005), ``backfill_min_clear_m``/``backfill_min_clear_raw_m``
    carry the already-computed values from that row.
    """

    test_id: str
    mission: str
    run_dir: Path
    meta: dict
    plan: dict | None
    eventi: list

    # raw traces (kinematics.csv), all aligned to ``ti``
    ti: np.ndarray
    x: np.ndarray
    y: np.ndarray
    speed: np.ndarray
    yaw: np.ndarray  # rad, kinematics.csv "yaw" -- for METRICS' heading_change_deg
    yaw_rate: np.ndarray  # rad/s, kinematics.csv "yaw_rate"
    corridor_clearance: np.ndarray
    obstacle_clearance: np.ndarray
    clearance: np.ndarray  # alias of obstacle_clearance, see class docstring

    corridors: list  # raw corridors.jsonl records (t, id, polygon, meta, ...)
    imu: dict  # {col: ndarray} from imu.csv
    solver_states: dict  # {col: ndarray} from mpc.csv (status is str ndarray)
    commands: dict  # {col: ndarray} from commands.csv (source is str ndarray)

    # windows (RULES.md §3)
    drive_window: tuple | None  # (t_start, t_end) in seconds, or None
    drive_inferred: bool  # True if no mission_started/finished/aborted events
    drive_why: str | None
    drive_idx: np.ndarray  # bool mask over ti
    moving_idx: np.ndarray | None  # None if pose_valid is False
    after_start_idx: np.ndarray | None  # None if pose_valid is False

    # the 6 data-quality flags (RULES.md §4)
    pose_valid: bool
    pose_valid_why: str | None
    clearance_coverage_pct: float | None
    clearance_coverage_pct_why: str | None
    clearance_source: str  # "live" | "backfill" | "none"
    clearance_partial: bool
    #: Set only when clearance_source=="backfill": the already-computed
    #: values from campaign_results.csv's row for this test_id, so METRICS
    #: does not need to re-read the campaign CSV (RULES.md §4 correction).
    backfill_min_clear_m: float | None
    backfill_min_clear_raw_m: float | None
    imu_valid: bool
    imu_valid_why: str | None
    solver_valid: bool
    solver_valid_why: str | None
    corridor_schema: str
    corridor_schema_why: str | None


def _compute_drive_window(eventi, ti, commands, solver_states):
    """``((t_start, t_end), inferred, why)`` per RULES.md §3.

    ``drive`` = from ``mission_started`` to the first of ``mission_finished``
    /``mission_aborted``. When there are no such events at all (e.g. the G7
    runs, which only have the 5 CSVs), the window is inferred from the first
    /last activity in ``commands.csv``/``mpc.csv`` and flagged as such.
    """
    started = [e.get("t") for e in eventi if e.get("event") == "mission_started"]
    started = [t for t in started if t is not None]
    ends = [
        e.get("t")
        for e in eventi
        if e.get("event") in ("mission_finished", "mission_aborted")
    ]
    ends = [t for t in ends if t is not None]

    if started:
        t0 = min(started)
        later_ends = [t for t in ends if t >= t0]
        if later_ends:
            t1 = min(later_ends)
        elif ti.size and np.isfinite(ti).any():
            t1 = float(np.nanmax(ti))
        else:
            t1 = t0
        return (t0, t1), False, None

    candidates_start = []
    candidates_end = []
    cmd_t = commands.get("t", np.empty(0))
    cmd_speed = commands.get("cmd_speed", np.empty(0))
    if cmd_t.size and cmd_speed.size:
        active = cmd_t[np.isfinite(cmd_speed) & (np.abs(cmd_speed) > 1e-6)]
        if active.size:
            candidates_start.append(float(active.min()))
            candidates_end.append(float(active.max()))
    solver_t = solver_states.get("t", np.empty(0))
    if solver_t.size and np.isfinite(solver_t).any():
        valid_t = solver_t[np.isfinite(solver_t)]
        candidates_start.append(float(valid_t.min()))
        candidates_end.append(float(valid_t.max()))

    if not candidates_start:
        return (
            None,
            True,
            "no mission_started/mission_finished/mission_aborted event, and "
            "no commands.csv/mpc.csv activity to infer the drive window from",
        )

    t0 = min(candidates_start)
    if ti.size and np.isfinite(ti).any():
        candidates_end.append(float(np.nanmax(ti)))
    t1 = max(candidates_end) if candidates_end else t0
    return (
        (t0, t1),
        True,
        f"no mission_started/mission_finished/mission_aborted event; drive "
        f"window inferred from first/last commands.csv/mpc.csv activity "
        f"({t0:.3f}..{t1:.3f} s)",
    )


def _check_pose_valid(ti, x, y, speed, commands, drive_idx, meta_max_speed,
                       tol_m=POSE_JUMP_TOLERANCE_M):
    """RULES.md §4 ``pose_valid``.

    Fails if: the path is 0 while commands were active, a jump between
    consecutive samples exceeds ``v_max*dt + tol``, or the total path
    exceeds ``v_max*duration`` (physically impossible given the run's own
    observed top speed). ``v_max`` is this run's own max observed speed in
    the drive window (falling back to ``meta.summary.max_speed`` if the
    speed column is all-NaN) -- this is what makes the P004-R017 case
    ("8.01 m in 13.5 s at 0.38 m/s max") actually trip the flag: 0.38*13.5 =
    5.13 m < 8.01 m observed.
    """
    if not drive_idx.any() or ti.size < 2:
        return False, "no kinematics.csv sample in the drive window"

    idx = np.flatnonzero(drive_idx)
    t_drive = ti[idx]
    x_drive = x[idx]
    y_drive = y[idx]
    speed_drive = speed[idx]

    valid_speed = speed_drive[np.isfinite(speed_drive)]
    if valid_speed.size:
        v_max = float(np.nanmax(valid_speed))
    elif meta_max_speed:
        v_max = float(meta_max_speed)
    else:
        v_max = None

    reasons = []

    dt = np.diff(t_drive)
    seg = np.sqrt(np.diff(x_drive) ** 2 + np.diff(y_drive) ** 2)
    finite = np.isfinite(seg) & np.isfinite(dt)
    path_len = float(np.nansum(seg[finite])) if finite.any() else 0.0

    if v_max is not None and v_max > 0 and finite.any():
        allowed = v_max * dt[finite] + tol_m
        jumps = seg[finite] > allowed
        if jumps.any():
            first = np.flatnonzero(jumps)[0]
            bad_t = t_drive[np.flatnonzero(finite)[first] + 1]
            reasons.append(
                f"position jump {seg[finite][first]:.3f} m at t={bad_t:.3f}s "
                f"exceeds v_max*dt+tol ({allowed[first]:.3f} m, "
                f"v_max={v_max:.3f} m/s)"
            )

    cmd_t = commands.get("t", np.empty(0))
    cmd_speed = commands.get("cmd_speed", np.empty(0))
    active_commands = False
    if cmd_t.size and t_drive.size:
        t0, t1 = float(np.nanmin(t_drive)), float(np.nanmax(t_drive))
        mask = (cmd_t >= t0) & (cmd_t <= t1)
        if mask.any():
            active_commands = bool(np.any(np.abs(cmd_speed[mask]) > 1e-6))

    if path_len == 0.0 and active_commands:
        reasons.append(
            "path length is 0 (x,y did not move) while cmd_speed was active "
            "in the drive window"
        )

    duration = float(np.nanmax(t_drive) - np.nanmin(t_drive)) if t_drive.size else 0.0
    if v_max is not None and duration > 0 and path_len > v_max * duration + tol_m:
        reasons.append(
            f"path {path_len:.3f} m is impossible in {duration:.3f} s at this "
            f"run's observed v_max {v_max:.3f} m/s (max plausible "
            f"{v_max * duration:.3f} m)"
        )

    if reasons:
        return False, "; ".join(reasons)
    return True, None


def _after_start_mask(x, y, drive_idx, threshold_m):
    mask = np.zeros_like(drive_idx)
    idx = np.flatnonzero(drive_idx)
    if idx.size < 2:
        return mask
    seg = np.sqrt(np.diff(x[idx]) ** 2 + np.diff(y[idx]) ** 2)
    seg = np.where(np.isfinite(seg), seg, 0.0)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    mask[idx] = cum >= threshold_m
    return mask


#: Matches the clearance tags export_campaign_csv/test_campaign_backfill write
#: into campaign_results.csv's ``backfilled`` column, e.g.
#: ``min_clear_raw_m(partial 39%)`` / ``min_clear_m(partial 39%)``.
_BACKFILL_CLEARANCE_TAG_RE = re.compile(
    r"min_clear(?:_raw)?_m\(partial\s+(\d+)\s*%\)"
)


def _csv_decimal_comma_float(text):
    """Parse a campaign_results.csv numeric cell, which may use a decimal comma."""
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


def _backfill_clearance_info(campaign_row):
    """``(coverage_pct, min_clear_m, min_clear_raw_m)`` from a campaign_results.csv row.

    ``campaign_row`` is one ``csv.DictReader`` row (``;``-delimited,
    decimal comma, see ``campaign_tools.md``). Returns ``(None, None, None)``
    if the row has no ``min_clear_*_m(partial NN%)`` tag in ``backfilled``
    (i.e. no backfill was ever done for this clearance value).
    """
    if campaign_row is None:
        return None, None, None
    match = _BACKFILL_CLEARANCE_TAG_RE.search(campaign_row.get("backfilled") or "")
    if match is None:
        return None, None, None
    coverage_pct = float(match.group(1))
    min_clear_m = _csv_decimal_comma_float(campaign_row.get("min_clear_m"))
    min_clear_raw_m = _csv_decimal_comma_float(campaign_row.get("min_clear_raw_m"))
    return coverage_pct, min_clear_m, min_clear_raw_m


def _read_campaign_results_row(campaign_dir, test_id):
    """One row of ``campaign_results.csv`` for ``test_id``, or ``None``.

    Read-only; never writes/caches into the campaign folder. ``;``-delimited,
    ``utf-8-sig`` (BOM), decimal comma -- the *other* CSV, not ``results.csv``
    (``campaign_tools.md`` "Two different CSVs -- do not confuse").
    """
    path = Path(campaign_dir) / "campaign_results.csv"
    if not path.exists():
        return None
    try:
        with open(path, encoding="utf-8-sig", newline="") as fh:
            header = fh.readline()
        delimiter = ";" if header.count(";") > header.count(",") else ","
        with open(path, encoding="utf-8-sig", newline="") as fh:
            for row in csv.DictReader(fh, delimiter=delimiter):
                if (row.get("test_id") or "").strip() == test_id:
                    return row
    except OSError:
        return None
    return None


def _check_clearance(obstacle_clearance, drive_idx, campaign_row):
    """RULES.md §4 ``clearance_coverage_pct``/``clearance_source``.

    Coverage = % of drive samples with a non-NaN ``obstacle_clearance``.
    At/above 50% it is ``clearance_source="live"`` (between 50 and 90% the
    value is kept with ``clearance_partial=True``, per RULES.md "tra 50% e
    90% resta il valore con flag partial").

    Below 50%, the live trace alone is not enough (RULES.md: "il 50% è
    volutamente stretto"). The known case P001-R001...R005 is exactly this:
    ``kinematics.csv``'s ``obstacle_clearance`` is entirely empty there, but
    a usable aggregate still exists from the bag-based backfill mechanism
    (``tools/test_campaign_backfill.py`` -> ``backfill.json`` -> the
    ``backfilled`` column of ``campaign_results.csv``, per
    ``campaign_tools.md``). So the live<50% branch looks up this run's row
    in ``campaign_results.csv`` (read-only, no bag/ROS touched) for a
    ``min_clear_*_m(partial NN%)`` tag: if found, reports
    ``clearance_source="backfill"``, ``clearance_coverage_pct=NN`` (always
    partial, RULES.md: "il vero minimo può essere più basso") and carries
    the already-computed ``min_clear_m``/``min_clear_raw_m`` values along so
    METRICS does not have to re-read the campaign CSV. If no such tag
    exists, the result stays ``clearance_source="none"`` with the live
    (low) coverage and a non-empty failure reason -- never an invented
    estimate.
    """
    n_drive = int(drive_idx.sum())
    if n_drive == 0:
        return None, "no sample in the drive window", "none", True, None, None

    values = obstacle_clearance[drive_idx]
    valid = np.isfinite(values)
    coverage_pct = 100.0 * float(valid.sum()) / n_drive

    if coverage_pct >= CLEARANCE_COVERAGE_FAIL_PCT:
        partial = coverage_pct < CLEARANCE_COVERAGE_PARTIAL_PCT
        return coverage_pct, None, "live", partial, None, None

    backfill_pct, min_clear_m, min_clear_raw_m = _backfill_clearance_info(campaign_row)
    if backfill_pct is not None:
        return backfill_pct, None, "backfill", True, min_clear_m, min_clear_raw_m

    why = (
        f"clearance_coverage_pct {coverage_pct:.1f}% is below the 50% floor "
        f"of the drive window, and campaign_results.csv has no "
        f"min_clear_*_m(partial NN%) backfill tag for this test_id"
    )
    return coverage_pct, why, "none", True, None, None


def _check_imu_valid(imu_t):
    valid = imu_t[np.isfinite(imu_t)] if imu_t.size else imu_t
    if valid.size < 2:
        return False, "fewer than 2 IMU samples available"
    duration = float(valid.max() - valid.min())
    hz = (valid.size - 1) / duration if duration > 0 else 0.0
    reasons = []
    if duration < IMU_MIN_SECONDS:
        reasons.append(f"IMU duration {duration:.2f}s < {IMU_MIN_SECONDS}s")
    if hz < IMU_MIN_HZ:
        reasons.append(f"IMU rate {hz:.1f} Hz < {IMU_MIN_HZ} Hz")
    if reasons:
        return False, "; ".join(reasons)
    return True, None


def _check_solver_valid(solver_t, drive_window):
    if solver_t.size == 0 or not np.isfinite(solver_t).any():
        return False, "mpc.csv is missing or empty"
    if drive_window is None:
        return False, "no drive window available, cannot check for MPC status in it"
    t0, t1 = drive_window
    valid_t = solver_t[np.isfinite(solver_t)]
    mask = (valid_t >= t0) & (valid_t <= t1)
    if not mask.any():
        return (
            False,
            f"no MPC status in mpc.csv within the drive window ({t0:.3f}..{t1:.3f}s)",
        )
    return True, None


def _check_corridor_schema(corridors):
    if not corridors:
        return "none", "no corridors.jsonl, or the file is empty"
    schema = None
    for record in corridors:
        meta = record.get("meta") or {}
        value = meta.get("schema")
        if value:
            schema = value
            break
    if schema is None:
        # v1 corridors.jsonl records have no "schema" key at all
        # (analysis_v2_data.md §5): absence of the key, with records
        # present, means the old schema.
        schema = "v1"
    if schema != CORRIDOR_SCHEMA_OK:
        return schema, f"corridor_schema={schema!r}, expected {CORRIDOR_SCHEMA_OK!r}"
    return schema, None


def load_run_v2(run_dir):
    """Load one run folder into a :class:`RunV2` with windows and quality flags."""
    run_dir = Path(run_dir)
    test_id = run_dir.name
    mission = run_dir.parent.name
    try:
        parse_test_id(test_id)
    except ValueError as exc:
        raise ValueError(f"{run_dir}: not a run folder: {exc}") from exc

    meta = _read_meta(run_dir)
    summary = meta.get("summary", meta) if meta else {}
    plan = _read_plan(run_dir)
    eventi = sorted(
        _read_jsonl(run_dir / "events.jsonl"), key=lambda e: e.get("t", 0.0)
    )

    kin = _read_csv_numeric(
        run_dir / "kinematics.csv",
        ["t", "x", "y", "speed", "yaw", "yaw_rate", "corridor_clearance", "obstacle_clearance"],
    )
    ti, x, y, speed = kin["t"], kin["x"], kin["y"], kin["speed"]
    yaw, yaw_rate = kin["yaw"], kin["yaw_rate"]
    corridor_clearance = kin["corridor_clearance"]
    obstacle_clearance = kin["obstacle_clearance"]

    commands = _read_csv_mixed(
        run_dir / "commands.csv",
        numeric_cols=["t", "cmd_speed", "cmd_steer", "cmd_yaw_rate", "cmd_throttle", "cmd_brake"],
        string_cols=["source"],
    )
    imu = _read_csv_numeric(
        run_dir / "imu.csv",
        ["t", "sensor_stamp", "ax", "ay", "az", "gx", "gy", "gz", "imu_yaw"],
    )
    solver_states = _read_csv_mixed(
        run_dir / "mpc.csv",
        numeric_cols=["t", "solve_time_ms", "cost", "iterations"],
        string_cols=["status"],
    )
    corridors = _read_jsonl(run_dir / "corridors.jsonl")

    drive_window, drive_inferred, drive_why = _compute_drive_window(
        eventi, ti, commands, solver_states
    )
    if drive_window is not None:
        t0, t1 = drive_window
        drive_idx = (ti >= t0) & (ti <= t1)
    else:
        drive_idx = np.zeros_like(ti, dtype=bool)

    pose_valid, pose_valid_why = _check_pose_valid(
        ti, x, y, speed, commands, drive_idx, summary.get("max_speed")
    )

    if pose_valid:
        moving_idx = drive_idx & (speed > MOVING_SPEED_THRESHOLD_MPS)
        after_start_idx = _after_start_mask(x, y, drive_idx, AFTER_START_DISTANCE_M)
    else:
        moving_idx = None
        after_start_idx = None

    campaign_row = _read_campaign_results_row(run_dir.parent.parent, test_id)
    (
        clearance_coverage_pct,
        clearance_coverage_why,
        clearance_source,
        clearance_partial,
        backfill_min_clear_m,
        backfill_min_clear_raw_m,
    ) = _check_clearance(obstacle_clearance, drive_idx, campaign_row)

    imu_valid, imu_valid_why = _check_imu_valid(imu["t"])
    solver_valid, solver_valid_why = _check_solver_valid(solver_states["t"], drive_window)
    corridor_schema, corridor_schema_why = _check_corridor_schema(corridors)

    return RunV2(
        test_id=test_id,
        mission=mission,
        run_dir=run_dir,
        meta=meta,
        plan=plan,
        eventi=eventi,
        ti=ti,
        x=x,
        y=y,
        speed=speed,
        yaw=yaw,
        yaw_rate=yaw_rate,
        corridor_clearance=corridor_clearance,
        obstacle_clearance=obstacle_clearance,
        clearance=obstacle_clearance,
        corridors=corridors,
        imu=imu,
        solver_states=solver_states,
        commands=commands,
        drive_window=drive_window,
        drive_inferred=drive_inferred,
        drive_why=drive_why,
        drive_idx=drive_idx,
        moving_idx=moving_idx,
        after_start_idx=after_start_idx,
        pose_valid=pose_valid,
        pose_valid_why=pose_valid_why,
        clearance_coverage_pct=clearance_coverage_pct,
        clearance_coverage_pct_why=clearance_coverage_why,
        clearance_source=clearance_source,
        clearance_partial=clearance_partial,
        backfill_min_clear_m=backfill_min_clear_m,
        backfill_min_clear_raw_m=backfill_min_clear_raw_m,
        imu_valid=imu_valid,
        imu_valid_why=imu_valid_why,
        solver_valid=solver_valid,
        solver_valid_why=solver_valid_why,
        corridor_schema=corridor_schema,
        corridor_schema_why=corridor_schema_why,
    )


# --------------------------------------------------------------------------
# canonical group key (RULES.md §6/§11, REVIEW finding MEDIO on groups.csv
# vs png.py/video_mask.py/video_overhead.py using three different
# conventions for "same mission+ostacolo group")
# --------------------------------------------------------------------------

def group_key(mission, ostacolo):
    """Canonical RULES.md §6/§11 group key, for both aggregation and the
    ``<Mxx_con_ostacolo|Mxx_no_ostacolo>`` output subfolder name of §9.

    ``mission`` must be the FULL mission folder name (e.g.
    ``"M06_turn_left_wall_then_person"``, ``run_dir.parent.name``/
    :attr:`RunV2.mission`), never just its ``Mxx`` numeric prefix.

    RULES.md §6 says the old sheet's "G" labels are historical only --
    "Il raggruppamento effettivo è quello della sezione 2" (mission +
    ostacolo) -- and §11 makes this explicit for the one case where it
    matters: P006/P008 both map to the old sheet's single "G11" row, but
    "D4, P006 e P008: separate come M06 e M08" -- they are, and must stay,
    two different groups. Folding on the ``Mxx`` prefix alone (what
    ``png.py``'s ``group_label``/``render_group_png`` and the
    ``group_folder`` helpers in ``video_mask.py``/``video_overhead.py``
    currently do, per the REVIEW finding this function fixes) would merge
    those two, and would also wrongly merge two genuinely different
    missions that happen to share a prefix on disk today
    (``M07_chair_then_straight_4m`` vs ``M07_stop_at_chair``).

    ``ostacolo`` accepts either a bool or the raw ``"si"``/``"no"`` string
    (e.g. a :class:`SelectionRow` field read before/without
    :func:`_normalize_si_no`), so callers can pass either representation.

    Returns ``f"{mission}_con_ostacolo"`` or ``f"{mission}_no_ostacolo"``:
    the same string for two runs iff they share both the full mission name
    and the obstacle flag -- i.e. iff RULES.md §6/§11 says they are the
    same group -- and callers should use this same string both as the
    ``groups.csv`` group identifier and as the output subfolder name, so
    that tables.py/png.py/video_mask.py/video_overhead.py all agree on
    what "the same group" means.
    """
    mission = str(mission).strip()
    if isinstance(ostacolo, str):
        has_obstacle = ostacolo.strip().lower() == "si"
    else:
        has_obstacle = bool(ostacolo)
    suffix = "con_ostacolo" if has_obstacle else "no_ostacolo"
    return f"{mission}_{suffix}"
