"""Per-run metrics engine for the ``analysis_v2`` pipeline (METRICS node).

For every :class:`run_data.RunV2` this module computes the outcome levels of
``RULES.md`` §5 (``plan_ok``/``exec_ok``/``safe``/``task_ok``/``success``),
the group criteria of §6 (with decisions D1/D2/D4/D5/D7 of §12) and the
metrics of §7. It also provides the §8 aggregation helpers (Wilson 95%,
median/min/max with valid N). It never writes files: CSV/YAML output is the
TABLES node's job.

Conventions (``RULES.md`` §1):

* "Missing" is not "zero": a metric that cannot be computed is ``None`` and
  ``<column>_why`` holds the reason. Every ``None`` metric has a ``_why``.
* Units are in the column name (``*_m``, ``*_s``, ``*_pct``, ``*_mps3``...).
* Three-valued logic for the outcome levels: ``True``/``False``/``None``
  (unknown). ``AND`` follows Kleene: any ``False`` -> ``False``; else any
  ``None`` -> ``None``; else ``True``.

Solver semantics (``.sjck/interfaces/analysis_v2_controller_perception.md``
§1, verified in ``mpc_solver.py``/``campaign_status.py``/``MPC_corr.py``):
``solved_inaccurate`` IS applied by the controller (``success=True``, warm
start kept), so RULES §7 ``solver_usable_pct = solved + solved_inaccurate``
is confirmed. Discrepancy with RULES: ``max_iter`` is ALSO applied as if it
were a solution (``success=True``), so an extra column
``solver_applied_pct = solved + solved_inaccurate + max_iter`` is reported;
``max_iter`` stays "applied but not converged" and is NOT counted as usable.

Jerk cutoff: 2 Hz (RULES §12 D3). The D3 spectrum (same interface file §3)
CONFIRMS 2 Hz (motion band 1.2-2.0 Hz, jerk peak 22-24 Hz = vibration); the
1 Hz and 5 Hz columns are kept for sensitivity. Note: filtfilt on a 2nd
order Butterworth is a 4th order magnitude response, -6 dB at the cutoff.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

import numpy as np

try:  # package import: ``from scripts.metrics import ...``
    from .run_data import (
        AFTER_START_DISTANCE_M,
        CORRIDOR_SCHEMA_OK,
        MOVING_SPEED_THRESHOLD_MPS,
        RunV2,
        _read_jsonl,
        load_run_v2,
    )
except ImportError:  # direct execution: ``python3 scripts/metrics.py``
    from run_data import (  # type: ignore[no-redef]
        AFTER_START_DISTANCE_M,
        CORRIDOR_SCHEMA_OK,
        MOVING_SPEED_THRESHOLD_MPS,
        RunV2,
        _read_jsonl,
        load_run_v2,
    )

__all__ = [
    "G_MPS2",
    "JERK_CUTOFF_HZ",
    "compute_run_metrics",
    "load_manual_verdicts",
    "expected_spec",
    "extract_plan",
    "segment_moves",
    "wilson",
    "summarize_continuous",
    "aggregate_group",
    "kleene_and",
]

# --------------------------------------------------------------------------
# constants (RULES.md values; every non-RULES default is marked "default")
# --------------------------------------------------------------------------

G_MPS2 = 9.80665  # RULES §7: IMU in g -> m/s^2
JERK_CUTOFF_HZ = 2.0  # RULES §12 D3
JERK_SENSITIVITY_HZ = (1.0, 5.0)  # RULES §7
IMU_RESAMPLE_HZ = 50.0  # same as the D3 spectrum study
BUTTER_ORDER = 2  # RULES §7

STEER_HYSTERESIS_RAD = 0.02  # RULES §7

# §6 task criteria
STRAIGHT_TOL_M = 0.15  # G1/G6/G12
LATERAL_MAX_M = 0.20  # G1/G6
WALL_APPROACH_TARGET_M, WALL_APPROACH_TOL_M = 0.5, 0.2  # D1
WALL_TURN_START_TARGET_M, WALL_TURN_START_TOL_M = 1.0, 0.2  # D2
TURN_TARGET_DEG, TURN_TOL_DEG = 90.0, 15.0  # G5
PERSON_TARGET_M, PERSON_TOL_M = 1.0, 0.3  # G4/G9/G11
PLANT_MAX_GAP_M = 0.5  # G3
FACING_MAX_DEG = 30.0  # default: "orientato verso di essa" has no number in RULES

# exec_ok tolerances against the plan value (RULES §5 says "valore del piano
# rispettato" without a number: defaults equal to the §6 task tolerances)
EXEC_DIST_TOL_M = STRAIGHT_TOL_M
EXEC_TURN_TOL_DEG = TURN_TOL_DEG

# move segmentation (no per-move event exists in the logs, see segment_moves)
SEG_DIST_EPS_M = 0.02
SEG_TURN_EPS_DEG = 2.0
TURN_ONSET_DEG = 3.0
TIMEOUT_SLACK_S = 1.0
#: corridors with |dpsi| below this are "straight" (logged straight corridors
#: carry dpsi ~1e-4..1e-3 rad of numerical noise; wall_turn ones ~0.06-1.1 rad)
STRAIGHT_DPSI_MAX_RAD = 0.05

VALUE_AT_MAX_DT_S = 0.3
EVENT_WINDOW_SLACK_S = 0.5
DEFAULT_ROBOT_RADIUS_M = 0.3  # meta.robot_radius on later runs; P001-R001..R005 log 0.0

SOLVER_USABLE = ("solved", "solved_inaccurate")
SOLVER_APPLIED = ("solved", "solved_inaccurate", "max_iter")

#: Fallback prompt by mission folder, only used when meta.json is missing
#: (G7, P002-R023..R027 only have the 5 CSVs).
MISSION_FALLBACK_PROMPT = {
    "M01_forward_3m": "go ahead 3 meters",
    "M02_approach_wall": "approach the wall",
    "M03_bottle": "go to the potted plant",
    "M04_person": "go to the person",
    "M05_turn_left_wall": "turn left at the wall",
    "M06_turn_left_wall_then_person": "turn left at the wall, then go to the person",
    "M07_chair_then_straight_4m": "go to the chair, then go straight 4m",
    "M08_turn_right_wall_then_person": "turn right at the wall then go to a person",
}


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------

def _f(value):
    """Plain float (JSON-friendly) or ``None`` for NaN/inf/None."""
    if value is None:
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def kleene_and(*values):
    """Three-valued AND: any False -> False; any None -> None; else True."""
    if any(v is False for v in values):
        return False
    if any(v is None for v in values):
        return None
    return True


def _set(out, key, value, why=None):
    """Store ``value`` and ``<key>_why`` (reason only when the value is None)."""
    out[key] = value
    out[f"{key}_why"] = None if value is not None else (why or "not computed")


def _summary(meta):
    if not meta:
        return {}
    return meta.get("summary", meta) or {}


def _finite_xy(run):
    return np.isfinite(run.ti) & np.isfinite(run.x) & np.isfinite(run.y)


def _value_at(t_query, t, values, max_dt=VALUE_AT_MAX_DT_S):
    """Nearest finite sample of ``values`` to ``t_query`` within ``max_dt``."""
    ok = np.isfinite(t) & np.isfinite(values)
    if not ok.any() or t_query is None:
        return None
    tt, vv = t[ok], values[ok]
    i = int(np.argmin(np.abs(tt - t_query)))
    if abs(tt[i] - t_query) > max_dt:
        return None
    return float(vv[i])


def _pose_at(run, t_query):
    """``(x, y, yaw_unwrapped)`` at the nearest kinematics sample, or None."""
    ok = _finite_xy(run)
    if not ok.any() or t_query is None:
        return None
    idx = np.flatnonzero(ok)
    i = idx[int(np.argmin(np.abs(run.ti[idx] - t_query)))]
    yaw_u = _unwrapped_yaw(run)
    return float(run.x[i]), float(run.y[i]), _f(yaw_u[i])


_YAW_CACHE: dict = {}


def _unwrapped_yaw(run):
    key = id(run)
    cached = _YAW_CACHE.get(key)
    if cached is not None and cached[0] is run:
        return cached[1]
    yaw = np.array(run.yaw, dtype=float)
    ok = np.isfinite(yaw)
    out = np.full_like(yaw, np.nan)
    if ok.any():
        out[ok] = np.unwrap(yaw[ok])
    _YAW_CACHE[key] = (run, out)
    return out


def _robot_radius(run):
    r = _f(_summary(run.meta).get("robot_radius", run.meta.get("robot_radius") if run.meta else None))
    if r is None or r <= 0:
        return DEFAULT_ROBOT_RADIUS_M, r
    return r, r


def _window_mask(t, t0, t1):
    return np.isfinite(t) & (t >= t0) & (t <= t1)


# --------------------------------------------------------------------------
# manual verdicts (RULES §5; file written by SCOUT_DATA)
# --------------------------------------------------------------------------

def load_manual_verdicts(path=None):
    """``{test_id: verdict dict}`` from ``manual_verdicts.yaml`` (empty if absent)."""
    import yaml

    if path is None:
        path = Path(__file__).resolve().parent.parent / "manual_verdicts.yaml"
    path = Path(path)
    if not path.exists():
        return {}
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    verdicts = raw.get("verdicts") or {}
    return {str(k): (v or {}) for k, v in verdicts.items()}


# --------------------------------------------------------------------------
# expected objective (from the prompt, RULES §2) and plan extraction
# --------------------------------------------------------------------------

_CLASS_ALIASES = {"plant": "potted plant", "pottedplant": "potted plant", "people": "person"}


def _norm_class(name):
    name = (name or "").strip().lower()
    name = re.sub(r"^(the|a|an)\s+", "", name)
    return _CLASS_ALIASES.get(name, name)


def expected_spec(prompt_text, mission=None):
    """Parse a campaign prompt into expected steps.

    Returns ``(steps, prompt_used, source, why)``. Step kinds:
    ``("straight", d_m)``, ``("approach_wall",)``, ``("wall_turn", sign)``,
    ``("go_to", class)``. ``steps`` is ``None`` when the prompt is unknown.
    """
    source = "meta.prompt_text"
    prompt = prompt_text
    if not prompt:
        prompt = MISSION_FALLBACK_PROMPT.get(mission or "")
        source = "mission_fallback"
    if not prompt:
        return None, None, None, "no prompt_text in meta.json and no fallback for this mission"
    parts = re.split(r",?\s*then\s+", prompt.strip().lower())
    steps = []
    for part in parts:
        m = re.search(r"go (?:ahead|straight|forward)\s+(\d+(?:\.\d+)?)\s*(?:m\b|meters?\b|metres?\b)", part)
        if m:
            steps.append(("straight", float(m.group(1))))
            continue
        if "approach" in part and "wall" in part:
            steps.append(("approach_wall",))
            continue
        m = re.search(r"turn (left|right)", part)
        if m and "wall" in part:
            steps.append(("wall_turn", 1.0 if m.group(1) == "left" else -1.0))
            continue
        m = re.search(r"go to (.+)$", part)
        if m:
            steps.append(("go_to", _norm_class(m.group(1))))
            continue
        return None, prompt, source, f"prompt part {part!r} not recognised"
    return steps, prompt, source, None


def extract_plan(run):
    """``(plan_dict, source)``: plan.json, else the last llm_calls.jsonl plan."""
    if isinstance(run.plan, dict) and run.plan.get("moves"):
        return run.plan, "plan.json"
    plan = None
    for rec in _read_jsonl(Path(run.run_dir) / "llm_calls.jsonl"):
        cand = rec.get("translated_plan")
        if not (isinstance(cand, dict) and cand.get("moves")):
            resp = rec.get("response")
            cand = None
            if isinstance(resp, str):
                try:
                    cand = json.loads(resp)
                except (json.JSONDecodeError, ValueError):
                    cand = None
            elif isinstance(resp, dict):
                cand = resp
        if isinstance(cand, dict) and cand.get("moves"):
            plan = cand
    if plan is not None:
        return plan, "llm_calls.jsonl"
    return None, None


def _move_kind(move):
    if "go_to_object" in move:
        return "go_to"
    mode = ((move.get("drive") or {}).get("mode") or "").lower()
    return mode or "other"


def _stop(move):
    return move.get("stop_condition") or {}


def _stop_value(move):
    sc = _stop(move)
    return _f(sc.get("distance", sc.get("value")))


def _match_step(step, move):
    """``(ok, why)`` for one expected step vs one plan move."""
    kind = _move_kind(move)
    sc = _stop(move)
    if step[0] == "straight":
        if kind != "straight" or sc.get("type") != "distance_reached":
            return False, f"expected straight/distance_reached, got {kind}/{sc.get('type')}"
        d = _stop_value(move)
        if d is None or abs(d - step[1]) > 0.01:
            return False, f"distance {d} != expected {step[1]}"
        return True, None
    if step[0] == "approach_wall":
        if kind != "straight" or sc.get("type") != "front_clearance":
            return False, f"expected straight/front_clearance, got {kind}/{sc.get('type')}"
        d = _stop_value(move)
        lo, hi = WALL_APPROACH_TARGET_M - WALL_APPROACH_TOL_M, WALL_APPROACH_TARGET_M + WALL_APPROACH_TOL_M
        if d is None or not (lo <= d <= hi):
            return False, f"front_clearance {d} m outside D1 {lo:.1f}..{hi:.1f} m (translation error)"
        return True, None
    if step[0] == "wall_turn":
        drive = move.get("drive") or {}
        if kind != "wall_turn" or sc.get("type") != "orientation_delta":
            return False, f"expected wall_turn/orientation_delta, got {kind}/{sc.get('type')}"
        sign = _f(drive.get("turn_sign"))
        if sign is None or math.copysign(1.0, sign) != step[1]:
            return False, f"turn_sign {sign} != expected {step[1]}"
        v = _stop_value(move)
        if v is None or abs(v - TURN_TARGET_DEG) > TURN_TOL_DEG:
            return False, f"orientation_delta {v} deg outside {TURN_TARGET_DEG}+-{TURN_TOL_DEG}"
        return True, None
    if step[0] == "go_to":
        if kind != "go_to":
            return False, f"expected go_to_object, got {kind}"
        cls = _norm_class((move.get("go_to_object") or {}).get("target_class"))
        if cls != step[1]:
            return False, f"target_class {cls!r} != expected {step[1]!r}"
        return True, None
    return False, f"unknown step {step!r}"


def _plan_ok(steps, plan):
    """RULES §5 ``plan_ok``: sequence and parameters vs the expected steps.

    Extra trailing ``straight`` moves after the expected sequence are
    tolerated (they do not contradict the prompt, e.g. the M05 plans add a
    short straight after the turn); any other extra move fails. Go-to gap
    values are not judged: the prompts do not state a distance.
    """
    if steps is None:
        return None, "expected objective unknown (prompt not parsed)", False
    if plan is None:
        return None, "no plan available (no plan.json, no plan in llm_calls.jsonl)", False
    moves = plan.get("moves") or []
    if len(moves) < len(steps):
        return False, f"plan has {len(moves)} moves, expected {len(steps)}", False
    for i, step in enumerate(steps):
        ok, why = _match_step(step, moves[i])
        if not ok:
            return False, f"move {i}: {why}", False
    extra = moves[len(steps):]
    bad = [m.get("id") for m in extra if _move_kind(m) != "straight"]
    if bad:
        return False, f"unexpected extra moves {bad}", True
    return True, None, True


# --------------------------------------------------------------------------
# move segmentation (no per-move events exist: inferred from odom/corridors)
# --------------------------------------------------------------------------

def _corridor_def(rec):
    meta = rec.get("meta") or {}
    return meta.get("definition") or {}


def _corridor_fallback_end(run, t_from, next_kind, cur_kind):
    """First corridor time after ``t_from`` that marks the next move."""
    seen_turn = False
    seen_obj = False
    for rec in run.corridors:
        t = _f(rec.get("t"))
        if t is None or t < t_from - 0.05:
            continue
        meta = rec.get("meta") or {}
        dpsi = _f(_corridor_def(rec).get("dpsi"))
        obj = bool(meta.get("object_mode"))
        if cur_kind == "go_to":
            if obj:
                seen_obj = True
            elif seen_obj:
                return t
            continue
        if next_kind == "go_to" and obj:
            return t
        if next_kind == "straight":
            if dpsi is not None and abs(dpsi) > STRAIGHT_DPSI_MAX_RAD:
                seen_turn = True
            elif seen_turn and not obj:
                return t
    return None


def segment_moves(run, plan):
    """Infer ``[{idx, id, kind, t0, t1, end_basis, reached}]`` for the plan moves.

    The logs have no per-move event, so boundaries are inferred:
    ``distance_reached d`` -> first sample with displacement >= d-0.02 m;
    ``orientation_delta V`` -> first sample with signed yaw change >= V-2 deg;
    ``object_reached`` -> corridor ``object_mode`` True->False switch;
    fallback -> the first corridor of the next move kind; the last move ends
    at the drive end. Returns ``(segments, why)``; ``segments`` is None when
    not computable (no drive window / no plan / invalid pose for a
    multi-move plan).
    """
    if plan is None:
        return None, "no plan"
    if run.drive_window is None:
        return None, "no drive window"
    moves = plan.get("moves") or []
    if not moves:
        return None, "plan has no moves"
    t0d, t1d = run.drive_window
    if len(moves) > 1 and not run.pose_valid:
        return None, f"pose_valid=false ({run.pose_valid_why}): multi-move boundaries need odometry"
    ok = _finite_xy(run) & _window_mask(run.ti, t0d, t1d)
    t = run.ti[ok]
    x, y = run.x[ok], run.y[ok]
    yaw_u = _unwrapped_yaw(run)[ok]
    segs = []
    cur = t0d
    for k, mv in enumerate(moves):
        kind = _move_kind(mv)
        sc = _stop(mv)
        last = k == len(moves) - 1
        end, basis = None, None
        if cur >= t1d - 1e-9 and k > 0:
            segs.append({"idx": k, "id": mv.get("id"), "kind": kind, "t0": t1d, "t1": t1d,
                         "end_basis": "not_started", "reached": False})
            continue
        if last:
            end, basis = t1d, "drive_end"
        else:
            sel = t >= cur
            ts_, xs, ys, ys_yaw = t[sel], x[sel], y[sel], yaw_u[sel]
            if ts_.size:
                if sc.get("type") == "distance_reached" and _stop_value(mv) is not None:
                    disp = np.hypot(xs - xs[0], ys - ys[0])
                    hit = np.flatnonzero(disp >= _stop_value(mv) - SEG_DIST_EPS_M)
                    if hit.size:
                        end, basis = float(ts_[hit[0]]), "odom_distance"
                elif sc.get("type") == "orientation_delta" and _stop_value(mv) is not None:
                    sign = _f((mv.get("drive") or {}).get("turn_sign")) or 1.0
                    dyaw = np.degrees(math.copysign(1.0, sign) * (ys_yaw - ys_yaw[0]))
                    hit = np.flatnonzero(np.nan_to_num(dyaw, nan=-1e9) >= _stop_value(mv) - SEG_TURN_EPS_DEG)
                    if hit.size:
                        end, basis = float(ts_[hit[0]]), "odom_yaw"
            if end is None:
                nxt = _move_kind(moves[k + 1])
                ce = _corridor_fallback_end(run, cur, nxt, kind)
                if ce is not None and ce < t1d:
                    end, basis = ce, "corridor_switch"
            if end is None:
                end, basis = t1d, "not_found_drive_end"
        reached = basis in ("odom_distance", "odom_yaw", "corridor_switch")
        if last:
            reached = any(e.get("event") == "mission_finished" for e in run.eventi)
        segs.append({"idx": k, "id": mv.get("id"), "kind": kind, "t0": float(cur),
                     "t1": float(end), "end_basis": basis, "reached": bool(reached)})
        cur = end
    return segs, None


# --------------------------------------------------------------------------
# geometry helpers for corridor/target metrics
# --------------------------------------------------------------------------

def _points_in_polygon(px, py, poly):
    """Vectorised even-odd ray casting."""
    xs, ys = poly[:, 0], poly[:, 1]
    x2, y2 = np.roll(xs, -1), np.roll(ys, -1)
    inside = np.zeros(px.shape, dtype=bool)
    for xa, ya, xb, yb in zip(xs, ys, x2, y2):
        cond = (ya > py) != (yb > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = xa + (py - ya) * (xb - xa) / (yb - ya)
        inside ^= cond & (px < xint)
    return inside


def _dist_to_polyline(px, py, line):
    a = line[:-1]
    b = line[1:]
    ab = b - a
    denom = np.maximum((ab ** 2).sum(axis=1), 1e-12)
    dx = px[:, None] - a[None, :, 0]
    dy = py[:, None] - a[None, :, 1]
    u = np.clip((dx * ab[None, :, 0] + dy * ab[None, :, 1]) / denom[None, :], 0.0, 1.0)
    cx = a[None, :, 0] + u * ab[None, :, 0]
    cy = a[None, :, 1] + u * ab[None, :, 1]
    return np.sqrt((px[:, None] - cx) ** 2 + (py[:, None] - cy) ** 2).min(axis=1)


def _target_distance(run, seg, cls, radius):
    """D5: robot-to-target at the end of a go-to move, from tracks.jsonl.

    Tracks are in frame ``map``, odometry in ``odom``; they are assumed to
    coincide (no localisation node, map->odom identity). The target is the
    ``cls`` track closest to the controller's last ``object_target`` (odom,
    corridors.jsonl) inside the move, else the closest to the robot.
    Returns ``(dict, why)``.
    """
    tracks = _read_jsonl(Path(run.run_dir) / "tracks.jsonl")
    if not tracks:
        return None, "no tracks.jsonl (D5: null, manual verdict with evidence)"
    t_end = seg["t1"]
    cand = [r for r in tracks if _f(r.get("t")) is not None and r["t"] <= t_end + EVENT_WINDOW_SLACK_S
            and any(_norm_class(tr.get("class")) == cls for tr in (r.get("tracks") or []))]
    if not cand:
        return None, f"no {cls!r} track up to the end of the move (D5)"
    rec = cand[-1]
    if t_end - rec["t"] > 2.0:
        return None, f"last {cls!r} track is {t_end - rec['t']:.1f}s older than the move end"
    pose = _pose_at(run, t_end)
    if pose is None or pose[2] is None:
        return None, "no robot pose at the move end"
    rx, ry, ryaw = pose
    ref = None
    for c in run.corridors:
        tc = _f(c.get("t"))
        meta = c.get("meta") or {}
        if tc is not None and seg["t0"] - 0.1 <= tc <= t_end and meta.get("object_target"):
            ref = meta["object_target"]
    trs = [tr for tr in rec["tracks"] if _norm_class(tr.get("class")) == cls
           and _f(tr.get("x")) is not None and _f(tr.get("y")) is not None]
    if ref is not None:
        tr = min(trs, key=lambda q: math.hypot(q["x"] - ref[0], q["y"] - ref[1]))
        basis = "track nearest to corridor object_target"
    else:
        tr = min(trs, key=lambda q: math.hypot(q["x"] - rx, q["y"] - ry))
        basis = "track nearest to robot"
    dist_c = math.hypot(tr["x"] - rx, tr["y"] - ry)
    width = _f(tr.get("width")) or 0.0
    bearing = math.degrees(math.atan2(tr["y"] - ry, tr["x"] - rx) - ryaw)
    bearing = (bearing + 180.0) % 360.0 - 180.0
    return {
        "center_m": dist_c,
        "gap_m": dist_c - radius - width / 2.0,
        "bearing_deg": bearing,
        "track_id": tr.get("id"),
        "track_age_s": t_end - rec["t"],
        "basis": basis,
    }, None


# --------------------------------------------------------------------------
# metric blocks
# --------------------------------------------------------------------------

def _context(run, out):
    meta = run.meta or {}
    summ = _summary(meta)
    _set(out, "git_commit", summ.get("git_commit") or meta.get("git_commit"), "no meta.json")
    odom_topic = None
    for c in run.corridors:
        odom_topic = (c.get("meta") or {}).get("odom_topic")
        if odom_topic:
            break
    if not odom_topic:
        odom_topic = (meta.get("extra_meta") or {}).get("odom_topic")
    _set(out, "odom_topic", odom_topic, "no odom_topic in corridors.jsonl/meta.json")
    radius, raw = _robot_radius(run)
    _set(out, "robot_radius_m", raw if raw is not None and raw > 0 else None,
         "meta.robot_radius missing or 0" + (f" (logged {raw})" if raw is not None else ""))
    out["robot_radius_used_m"] = radius

    # LLM latency: mean over ok calls of llm_calls.csv (same meaning as the old meta mean)
    import csv
    lat = []
    path = Path(run.run_dir) / "llm_calls.csv"
    n_calls = 0
    if path.exists():
        with open(path, encoding="utf-8", errors="replace") as fh:
            for row in csv.DictReader(fh.read().replace("\x00", "").splitlines()):
                n_calls += 1
                ok = (row.get("ok") or "").strip().lower() in ("1", "true", "yes")
                v = _f(row.get("latency_ms"))
                if ok and v is not None:
                    lat.append(v)
    _set(out, "llm_latency_ms", float(np.mean(lat)) if lat else _f(summ.get("llm_latency_mean_ms")),
         "no ok call with latency in llm_calls.csv nor meta")
    out["n_llm_calls"] = n_calls

    if run.drive_window is not None:
        _set(out, "drive_duration_s", float(run.drive_window[1] - run.drive_window[0]))
    else:
        _set(out, "drive_duration_s", None, run.drive_why)
    out["drive_inferred"] = run.drive_inferred
    out["drive_why"] = run.drive_why

    if run.pose_valid:
        idx = np.flatnonzero(run.drive_idx & _finite_xy(run))
        path_m = float(np.hypot(np.diff(run.x[idx]), np.diff(run.y[idx])).sum()) if idx.size > 1 else 0.0
        _set(out, "path_odom_m", path_m)
        sp = run.speed[run.drive_idx]
        sp = sp[np.isfinite(sp)]
        _set(out, "v_max_mps", float(sp.max()) if sp.size else None, "no speed sample in drive")
    else:
        _set(out, "path_odom_m", None, f"pose_valid=false: {run.pose_valid_why}")
        _set(out, "v_max_mps", None, f"pose_valid=false: {run.pose_valid_why}")

    ev_end = [e for e in run.eventi if e.get("event") == "test_end"]
    outcome = summ.get("auto_outcome") or (ev_end[-1].get("auto_outcome") if ev_end else None)
    _set(out, "auto_outcome", outcome, "no meta.json and no test_end event")
    aborted = [e for e in run.eventi if e.get("event") == "mission_aborted"]
    if aborted:
        reason = aborted[0].get("reason") or summ.get("reason") or "mission_aborted (no reason logged)"
        _set(out, "abort_reason", reason)
    else:
        _set(out, "abort_reason", None, "not aborted" if run.eventi else "no events.jsonl")


def _quality(run, out):
    out["pose_valid"] = run.pose_valid
    out["pose_valid_why"] = run.pose_valid_why
    out["clearance_coverage_pct"] = _f(run.clearance_coverage_pct)
    out["clearance_coverage_pct_why"] = run.clearance_coverage_pct_why
    out["clearance_source"] = run.clearance_source
    out["clearance_partial"] = run.clearance_partial
    out["imu_valid"] = run.imu_valid
    out["imu_valid_why"] = run.imu_valid_why
    out["solver_valid"] = run.solver_valid
    out["solver_valid_why"] = run.solver_valid_why
    out["corridor_schema"] = run.corridor_schema
    out["corridor_schema_why"] = run.corridor_schema_why


def _clearance(run, out):
    keys = ("clear_start_m", "clear_end_m", "min_clear_any_m", "t_min_clear_s",
            "min_clear_after_start_m", "contact_lidar")
    bf_why = None if run.clearance_source == "backfill" else (
        f"clearance_source={run.clearance_source}: no backfill value used")
    _set(out, "backfill_min_clear_m", _f(run.backfill_min_clear_m), bf_why or "not in campaign_results.csv")
    _set(out, "backfill_min_clear_raw_m", _f(run.backfill_min_clear_raw_m), bf_why or "not in campaign_results.csv")
    if run.clearance_source != "live":
        why = (f"clearance_source={run.clearance_source}: "
               f"{run.clearance_coverage_pct_why or 'live coverage below 50%'}")
        if run.clearance_source == "backfill":
            why = ("clearance_source=backfill (live coverage below 50%, RULES §4): "
                   "backfill value only in backfill_min_clear_m/backfill_min_clear_raw_m")
        for k in keys:
            _set(out, k, None, why)
    else:
        mask = run.drive_idx & np.isfinite(run.clearance)
        t, c = run.ti[mask], run.clearance[mask]
        if not t.size:
            for k in keys:
                _set(out, k, None, "no finite clearance in drive")
        else:
            _set(out, "clear_start_m", float(c[0]))
            _set(out, "clear_end_m", float(c[-1]))
            i = int(np.argmin(c))
            _set(out, "min_clear_any_m", float(c[i]))
            _set(out, "t_min_clear_s", float(t[i]))
            _set(out, "contact_lidar", bool(c[i] <= 0.0))
            if run.after_start_idx is None:
                _set(out, "min_clear_after_start_m", None, f"pose_valid=false: {run.pose_valid_why}")
            else:
                m2 = run.after_start_idx & np.isfinite(run.clearance)
                _set(out, "min_clear_after_start_m",
                     float(run.clearance[m2].min()) if m2.any() else None,
                     f"no finite clearance after the first {AFTER_START_DISTANCE_M} m")
    _set(out, "front_clear_end_m", None,
         "needs the LiDAR scan from the bag; run files hold only the min clearance (RULES §7)")
    _set(out, "min_obstacle_clear_m", None,
         "needs scan + tracks reconstruction from the bag, not available in run files (RULES §7)")


def _solver(run, out):
    keys = ("solver_solved_pct", "solver_inaccurate_pct", "solver_maxiter_pct",
            "solver_infeasible_pct", "solver_other_pct", "solver_usable_pct",
            "solver_applied_pct", "unusable_streak_max_s", "unapplied_streak_max_s",
            "solve_time_p95_ms")
    if not run.solver_valid:
        for k in keys:
            _set(out, k, None, f"solver_valid=false: {run.solver_valid_why}")
        out["solver_n_ticks"] = 0
        return
    st = run.solver_states
    t = st["t"]
    t0, t1 = run.drive_window
    m = _window_mask(t, t0, t1)
    order = np.argsort(t[m], kind="stable")
    tt = t[m][order]
    status = np.asarray(st["status"], dtype=object)[m][order]
    n = len(status)
    out["solver_n_ticks"] = n
    lab = np.array([str(s).strip().lower() for s in status], dtype=object)

    def pct(mask):
        return 100.0 * float(np.sum(mask)) / n

    solved = lab == "solved"
    inacc = lab == "solved_inaccurate"
    maxit = lab == "max_iter"
    infeas = lab == "infeasible"
    _set(out, "solver_solved_pct", pct(solved))
    _set(out, "solver_inaccurate_pct", pct(inacc))
    _set(out, "solver_maxiter_pct", pct(maxit))
    _set(out, "solver_infeasible_pct", pct(infeas))
    _set(out, "solver_other_pct", pct(~(solved | inacc | maxit | infeas)))
    # RULES §7 definition, confirmed by the controller code (interface §1.4)
    _set(out, "solver_usable_pct", pct(solved | inacc))
    # extra: max_iter is applied too (success=True in mpc_solver.py), see module doc
    _set(out, "solver_applied_pct", pct(solved | inacc | maxit))

    def streak(bad):
        if n == 0:
            return None
        dt_med = float(np.median(np.diff(tt))) if n > 1 else 0.0
        best, i = 0.0, 0
        while i < n:
            if bad[i]:
                j = i
                while j + 1 < n and bad[j + 1]:
                    j += 1
                end = tt[j + 1] if j + 1 < n else tt[j] + dt_med
                best = max(best, float(end - tt[i]))
                i = j + 1
            else:
                i += 1
        return best

    _set(out, "unusable_streak_max_s", streak(~(solved | inacc)))
    _set(out, "unapplied_streak_max_s", streak(~(solved | inacc | maxit)))
    stime = np.asarray(st["solve_time_ms"], dtype=float)[m]
    stime = stime[np.isfinite(stime)]
    _set(out, "solve_time_p95_ms", float(np.percentile(stime, 95)) if stime.size else None,
         "no finite solve_time_ms in drive")


def _jerk(run, out):
    cols = {JERK_CUTOFF_HZ: "jerk_rms_mps3", 1.0: "jerk_rms_1hz_mps3", 5.0: "jerk_rms_5hz_mps3"}
    if not run.imu_valid:
        for c in cols.values():
            _set(out, c, None, f"imu_valid=false: {run.imu_valid_why}")
        return
    if run.moving_idx is None:
        for c in cols.values():
            _set(out, c, None, f"moving window null, pose_valid=false: {run.pose_valid_why}")
        return
    from scipy.signal import butter, filtfilt

    t0, t1 = run.drive_window
    imu = run.imu
    ok = _window_mask(imu["t"], t0, t1) & np.isfinite(imu["ax"]) & np.isfinite(imu["ay"])
    ti = imu["t"][ok]
    order = np.argsort(ti)
    ti = ti[order]
    ax = imu["ax"][ok][order] * G_MPS2
    ay = imu["ay"][ok][order] * G_MPS2
    ti, uniq = np.unique(ti, return_index=True)
    ax, ay = ax[uniq], ay[uniq]
    fs = IMU_RESAMPLE_HZ
    if ti.size < 2 or ti[-1] - ti[0] < 2.0:
        for c in cols.values():
            _set(out, c, None, "less than 2 s of IMU data inside the drive window")
        return
    tg = np.arange(ti[0], ti[-1], 1.0 / fs)
    axg, ayg = np.interp(tg, ti, ax), np.interp(tg, ti, ay)
    kin_ok = np.isfinite(run.ti) & np.isfinite(run.speed)
    sp = np.interp(tg, run.ti[kin_ok], run.speed[kin_ok]) if kin_ok.any() else np.zeros_like(tg)
    moving = sp > MOVING_SPEED_THRESHOLD_MPS
    if not moving.any():
        for c in cols.values():
            _set(out, c, None, "no moving IMU sample in drive")
        return
    for fc, col in cols.items():
        b, a = butter(BUTTER_ORDER, fc / (fs / 2.0))
        if tg.size <= 3 * max(len(a), len(b)):
            _set(out, col, None, "too few IMU samples for filtfilt")
            continue
        fx, fy = filtfilt(b, a, axg), filtfilt(b, a, ayg)
        jx, jy = np.gradient(fx, 1.0 / fs), np.gradient(fy, 1.0 / fs)
        jh = np.hypot(jx, jy)[moving]
        _set(out, col, float(np.sqrt(np.mean(jh ** 2))))


def _steer(run, out):
    keys = ("steer_direction_rev_per_m", "steer_sign_rev_per_m", "steer_rate_rms_radps", "steer_abs_max_rad")
    if run.drive_window is None:
        for k in keys:
            _set(out, k, None, run.drive_why)
        return
    cmd = run.commands
    m = _window_mask(cmd["t"], *run.drive_window) & np.isfinite(cmd["cmd_steer"])
    t, s = cmd["t"][m], cmd["cmd_steer"][m]
    order = np.argsort(t, kind="stable")
    t, s = t[order], s[order]
    if s.size < 2:
        for k in keys:
            _set(out, k, None, "fewer than 2 steering commands in drive")
        return
    _set(out, "steer_abs_max_rad", float(np.abs(s).max()))
    dt = np.diff(t)
    good = dt > 1e-4
    rate = np.diff(s)[good] / dt[good]
    _set(out, "steer_rate_rms_radps", float(np.sqrt(np.mean(rate ** 2))) if rate.size else None,
         "no positive dt between commands")
    # direction reversals with hysteresis between extremes (RULES §7)
    rev, direction, ext = 0, 0, s[0]
    for v in s[1:]:
        if direction == 0:
            if v - ext >= STEER_HYSTERESIS_RAD:
                direction, ext = 1, v
            elif ext - v >= STEER_HYSTERESIS_RAD:
                direction, ext = -1, v
            else:
                continue
        elif direction > 0:
            if v > ext:
                ext = v
            elif ext - v >= STEER_HYSTERESIS_RAD:
                rev += 1
                direction, ext = -1, v
        else:
            if v < ext:
                ext = v
            elif v - ext >= STEER_HYSTERESIS_RAD:
                rev += 1
                direction, ext = 1, v
    sg = np.sign(s[s != 0])
    sign_rev = int(np.sum(sg[1:] != sg[:-1])) if sg.size > 1 else 0
    out["steer_direction_rev_n"] = rev
    out["steer_sign_rev_n"] = sign_rev
    path = out.get("path_odom_m")
    if path is None or path <= 0:
        why = out.get("path_odom_m_why") or "path_odom_m is 0"
        _set(out, "steer_direction_rev_per_m", None, f"path not valid: {why}")
        _set(out, "steer_sign_rev_per_m", None, f"path not valid: {why}")
    else:
        _set(out, "steer_direction_rev_per_m", rev / path)
        _set(out, "steer_sign_rev_per_m", sign_rev / path)


def _moving_pct(run, out):
    if run.moving_idx is None:
        _set(out, "moving_pct", None, f"pose_valid=false: {run.pose_valid_why}")
        return
    n = int(run.drive_idx.sum())
    _set(out, "moving_pct", 100.0 * float(run.moving_idx.sum()) / n if n else None, "no drive sample")


def _horizon(run, out):
    if not run.pose_valid:
        why = f"pose_valid=false: {run.pose_valid_why}"
        _set(out, "horizon_err_mean_m", None, why)
        _set(out, "horizon_err_max_m", None, why)
        return
    recs = _read_jsonl(Path(run.run_dir) / "horizon.jsonl")
    if not recs or run.drive_window is None:
        _set(out, "horizon_err_mean_m", None, "no horizon.jsonl")
        _set(out, "horizon_err_max_m", None, "no horizon.jsonl")
        return
    t0, t1 = run.drive_window
    ok = _finite_xy(run)
    kt, kx, ky = run.ti[ok], run.x[ok], run.y[ok]
    errs = []
    for r in recs:
        tr = _f(r.get("t"))
        xs, ys = r.get("x") or [], r.get("y") or []
        if tr is None or not (t0 <= tr <= t1) or r.get("frame_id", "odom") != "odom":
            continue
        ts = _f(r.get("ts")) or 0.1
        for k in range(1, min(len(xs), len(ys))):
            tk = tr + k * ts
            if tk > t1 or tk > kt[-1]:
                break
            px, py = _f(xs[k]), _f(ys[k])
            if px is None or py is None:
                continue
            errs.append(math.hypot(px - np.interp(tk, kt, kx), py - np.interp(tk, kt, ky)))
    if not errs:
        _set(out, "horizon_err_mean_m", None, "no horizon prediction inside drive")
        _set(out, "horizon_err_max_m", None, "no horizon prediction inside drive")
        return
    _set(out, "horizon_err_mean_m", float(np.mean(errs)))
    _set(out, "horizon_err_max_m", float(np.max(errs)))


def _corridor(run, plan, segs, out, radius):
    # rebuilt (old) metric: diagnostic only
    if run.corridor_schema != CORRIDOR_SCHEMA_OK:
        why = f"corridor_schema={run.corridor_schema!r} is not {CORRIDOR_SCHEMA_OK!r}"
        _set(out, "viol_rebuilt_pct", None, why)
        _set(out, "viol_fixed_pct", None, why)
        return
    cc = run.corridor_clearance[run.drive_idx]
    cc = cc[np.isfinite(cc)]
    _set(out, "viol_rebuilt_pct", 100.0 * float(np.mean(cc < 0.0)) if cc.size else None,
         "no corridor_clearance sample in drive")
    if not run.pose_valid:
        _set(out, "viol_fixed_pct", None, f"pose_valid=false: {run.pose_valid_why}")
        return
    if segs is None:
        _set(out, "viol_fixed_pct", None, "moves not segmentable")
        return
    straight = [s for s in segs if s["kind"] == "straight" and s["t1"] > s["t0"]]
    if not straight:
        _set(out, "viol_fixed_pct", None, "no straight move in the plan (RULES §7: straight only)")
        return
    n_tot, n_viol = 0, 0
    for s in straight:
        first = None
        for c in run.corridors:
            tc = _f(c.get("t"))
            if tc is None or tc < s["t0"] - 0.1 or tc > s["t1"]:
                continue
            d = _corridor_def(c)
            if abs(_f(d.get("dpsi")) or 0.0) > STRAIGHT_DPSI_MAX_RAD:
                continue
            first = c
            break
        if first is None or not first.get("polygon"):
            continue
        poly = np.asarray(first["polygon"], dtype=float)
        if poly.ndim != 2 or poly.shape[0] < 4:
            continue
        d = _corridor_def(first)
        meta = first.get("meta") or {}
        psi = _f(d.get("psiStart", meta.get("psi_ref")))
        length = _f(d.get("L", meta.get("length_m")))
        c0 = d.get("C0") or ((poly[0] + poly[-1]) / 2.0).tolist()
        if psi is None or length is None:
            continue
        half = poly.shape[0] // 2
        left, right = poly[:half], poly[half:]
        m = _finite_xy(run) & _window_mask(run.ti, s["t0"], s["t1"]) & run.drive_idx
        px, py = run.x[m], run.y[m]
        sl = (px - c0[0]) * math.cos(psi) + (py - c0[1]) * math.sin(psi)
        keep = (sl > 0.05) & (sl < length - 0.05)
        px, py = px[keep], py[keep]
        if not px.size:
            continue
        inside = _points_in_polygon(px, py, poly)
        dside = np.minimum(_dist_to_polyline(px, py, left), _dist_to_polyline(px, py, right))
        viol = (~inside) | (dside < radius)
        n_tot += px.size
        n_viol += int(viol.sum())
    out["viol_fixed_n_samples"] = n_tot
    _set(out, "viol_fixed_pct", 100.0 * n_viol / n_tot if n_tot else None,
         "no pose sample within the length of the first corridor of a straight move")


# --------------------------------------------------------------------------
# precision + task/exec per step
# --------------------------------------------------------------------------

def _seg_disp(run, seg):
    a, b = _pose_at(run, seg["t0"]), _pose_at(run, seg["t1"])
    if a is None or b is None:
        return None
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _seg_lateral(run, seg):
    a = _pose_at(run, seg["t0"])
    if a is None or a[2] is None:
        return None
    m = _finite_xy(run) & _window_mask(run.ti, seg["t0"], seg["t1"])
    if not m.any():
        return None
    lat = -(run.x[m] - a[0]) * math.sin(a[2]) + (run.y[m] - a[1]) * math.cos(a[2])
    return lat


def _seg_heading_deg(run, seg):
    a, b = _pose_at(run, seg["t0"]), _pose_at(run, seg["t1"])
    if a is None or b is None or a[2] is None or b[2] is None:
        return None
    return math.degrees(b[2] - a[2])


def _turn_onset(run, seg, sign):
    m = _finite_xy(run) & _window_mask(run.ti, seg["t0"], seg["t1"])
    yaw = _unwrapped_yaw(run)[m]
    t = run.ti[m]
    if not yaw.size or not np.isfinite(yaw[0]):
        return None
    d = np.degrees(sign * (yaw - yaw[0]))
    hit = np.flatnonzero(np.nan_to_num(d, nan=-1e9) >= TURN_ONSET_DEG)
    return float(t[hit[0]]) if hit.size else None


def _in(v, lo, hi):
    return None if v is None else bool(lo <= v <= hi)


def _wall_proxy_ok(clear_m, lo, hi):
    """Wall-distance criterion (D1/D2) from the any-return clearance.

    The frontal distance is not in the run files; the logged clearance is
    the distance from the footprint to the NEAREST LiDAR return, i.e. a
    lower bound of the distance to the wall. Above ``hi`` the criterion is
    certainly failed; inside ``[lo, hi]`` it is taken as met (in a wall
    approach the wall is the nearest return: proxy, see ``task_basis``);
    below ``lo`` the nearest return may be a side object, so the frontal
    distance is unknown -> None.
    """
    if clear_m is None:
        return None
    if clear_m > hi:
        return False
    if clear_m >= lo:
        return True
    return None


def _evaluate(run, steps, plan, plan_aligned, segs, segs_why, ostacolo, out, radius):
    """Fill precision metrics, task_ok and exec_ok (RULES §5/§6/§7)."""
    for k in ("final_displacement_odom_m", "goal_error_m", "plan_error_m", "lateral_error_max_m",
              "lateral_error_rms_m", "lateral_error_end_m", "heading_change_deg",
              "turn_start_t_s", "turn_start_clear_m", "target_dist_center_m", "target_gap_m",
              "target_bearing_deg"):
        out.setdefault(k, None)
        out.setdefault(f"{k}_why", "not applicable to this mission")
    out["task_basis"] = None

    # final displacement over the whole drive
    if not run.pose_valid:
        _set(out, "final_displacement_odom_m", None, f"pose_valid=false: {run.pose_valid_why}")
    elif run.drive_window is not None:
        a, b = _pose_at(run, run.drive_window[0]), _pose_at(run, run.drive_window[1])
        _set(out, "final_displacement_odom_m",
             math.hypot(b[0] - a[0], b[1] - a[1]) if a and b else None, "no pose at drive bounds")

    pose_why = f"pose_valid=false: {run.pose_valid_why}"
    moves = (plan or {}).get("moves") or []

    # ---- per-move precision + exec values
    exec_checks = []
    lat_all = []
    lat_end = None
    if segs is not None:
        for seg in segs:
            mv = moves[seg["idx"]]
            sc = _stop(mv)
            val = _stop_value(mv)
            if seg["kind"] == "straight" and run.pose_valid:
                lat = _seg_lateral(run, seg)
                if lat is not None and lat.size:
                    lat_all.append(lat)
                    lat_end = float(lat[-1])
            if sc.get("type") == "distance_reached" and val is not None:
                disp = _seg_disp(run, seg) if run.pose_valid else None
                exec_checks.append(None if disp is None else abs(disp - val) <= EXEC_DIST_TOL_M)
            elif sc.get("type") == "orientation_delta" and val is not None:
                hd = _seg_heading_deg(run, seg) if run.pose_valid else None
                sign = math.copysign(1.0, _f((mv.get("drive") or {}).get("turn_sign")) or 1.0)
                exec_checks.append(None if hd is None else abs(sign * hd - val) <= EXEC_TURN_TOL_DEG)
            else:
                # front_clearance / object_reached: the frontal scan is not in the
                # run files (front_clear_end_m is null, RULES §7), so the value is
                # taken as respected when the robot's own stop condition fired
                # (move reached; for the terminal move = mission_finished).
                exec_checks.append(True if seg["reached"] else None)
    if lat_all:
        cat = np.concatenate(lat_all)
        cat = cat[np.isfinite(cat)]
        _set(out, "lateral_error_max_m", float(np.abs(cat).max()) if cat.size else None, "no lateral sample")
        _set(out, "lateral_error_rms_m", float(np.sqrt(np.mean(cat ** 2))) if cat.size else None,
             "no lateral sample")
        _set(out, "lateral_error_end_m", abs(lat_end) if lat_end is not None else None, "no lateral sample")
    elif any(_move_kind(m) == "straight" for m in moves):
        why = pose_why if not run.pose_valid else (segs_why or "no straight segment with samples")
        for k in ("lateral_error_max_m", "lateral_error_rms_m", "lateral_error_end_m"):
            _set(out, k, None, why)

    # ---- timeout (D7)
    aborted = any(e.get("event") == "mission_aborted" for e in run.eventi)
    timeout_hit = None
    if segs is not None and moves:
        timeout_hit = False
        if aborted:
            for seg in segs:
                to = _f(moves[seg["idx"]].get("timeout_sec"))
                if to is not None and seg["t1"] - seg["t0"] >= to - TIMEOUT_SLACK_S:
                    timeout_hit = True
    out["timeout_hit"] = timeout_hit
    out["timeout_hit_why"] = None if timeout_hit is not None else "moves not segmentable"
    out["move_segments"] = segs
    out["move_segments_why"] = segs_why

    # ---- exec_ok (RULES §5, D7)
    finished = any(e.get("event") == "mission_finished" for e in run.eventi)
    started = any(e.get("event") == "mission_started" for e in run.eventi)
    if aborted:
        _set(out, "exec_ok", False)
        out["exec_ok_why"] = "mission_aborted" + (" at timeout (D7)" if timeout_hit else "")
    elif not run.eventi or not started:
        _set(out, "exec_ok", None, "no mission events (events.jsonl missing or mission never started)")
    elif not finished:
        _set(out, "exec_ok", False)
        out["exec_ok_why"] = "mission_finished never reached"
    elif plan is None:
        _set(out, "exec_ok", None, "mission finished but no plan to check values against")
    elif segs is None:
        _set(out, "exec_ok", None, f"mission finished, plan values not checkable: {segs_why}")
    else:
        v = kleene_and(*exec_checks)
        _set(out, "exec_ok", v, "mission finished, a plan value is not measurable (flag)")
        if v is False:
            out["exec_ok_why"] = "mission finished but a plan value was not respected"

    # ---- task_ok (RULES §6)
    if steps is None:
        _set(out, "task_ok", None, "no §6 criterion for this prompt")
        return
    if segs is not None and plan_aligned and len(segs) >= len(steps):
        # step i is judged on plan move i (trailing tolerated moves ignored)
        step_segs = segs[: len(steps)]
    elif len(steps) == 1 and run.drive_window is not None:
        # single-step prompt, plan missing/misaligned: judge on the whole drive
        step_segs = [{"idx": None, "kind": None, "t0": run.drive_window[0],
                      "t1": run.drive_window[1], "reached": finished}]
    else:
        _set(out, "task_ok", None, f"multi-step task needs move segments aligned to the plan "
                                   f"({segs_why or 'plan does not match the expected steps'})")
        return

    results, bases = [], []
    goal_error = None
    plan_error = None
    for step, seg in zip(steps, step_segs):
        kind = step[0]
        if kind == "straight":
            if not run.pose_valid:
                results.append(None)
                bases.append(f"straight {step[1]} m: {pose_why}")
                continue
            disp = _seg_disp(run, seg)
            lat = _seg_lateral(run, seg)
            lat = lat[np.isfinite(lat)] if lat is not None else None
            if lat is None or not lat.size or disp is None:
                results.append(None)
                bases.append("straight: no pose samples")
                continue
            ok = abs(disp - step[1]) <= STRAIGHT_TOL_M
            basis = f"straight disp {disp:.3f} m vs {step[1]}+-{STRAIGHT_TOL_M}"
            if len(steps) == 1:
                # G1/G6 only: lateral error (G6 with obstacle: at the end of the move)
                lat_val = abs(float(lat[-1])) if ostacolo else float(np.abs(lat).max())
                ok = ok and lat_val < LATERAL_MAX_M
                basis += f", lateral {'end' if ostacolo else 'max'} {lat_val:.3f} m < {LATERAL_MAX_M}"
            results.append(ok)
            goal_error = disp - step[1]
            bases.append(basis)
            if plan is not None and seg.get("idx") is not None and seg["idx"] < len(moves):
                pv = _stop_value(moves[seg["idx"]])
                if _stop(moves[seg["idx"]]).get("type") == "distance_reached" and pv is not None:
                    plan_error = disp - pv
        elif kind == "approach_wall":
            if run.clearance_source != "live":
                results.append(None)
                bases.append(f"approach_wall: clearance_source={run.clearance_source}")
                continue
            ce = _value_at(seg["t1"], run.ti, run.clearance, max_dt=1.0)
            lo = WALL_APPROACH_TARGET_M - WALL_APPROACH_TOL_M
            hi = WALL_APPROACH_TARGET_M + WALL_APPROACH_TOL_M
            results.append(_wall_proxy_ok(ce, lo, hi))
            if ce is not None:
                goal_error = ce - WALL_APPROACH_TARGET_M
                pv = _stop_value(moves[0]) if moves and _stop(moves[0]).get("type") == "front_clearance" else None
                if pv is not None:
                    plan_error = ce - pv
            bases.append(f"approach_wall: end clearance {ce} m (proxy of wall distance) in {lo:.1f}..{hi:.1f} (D1)")
        elif kind == "wall_turn":
            if not run.pose_valid:
                results.append(None)
                bases.append(f"wall_turn: {pose_why}")
                continue
            hd = _seg_heading_deg(run, seg)
            _set(out, "heading_change_deg", hd, "no yaw at move bounds")
            onset = _turn_onset(run, seg, step[1])
            _set(out, "turn_start_t_s", onset, f"yaw never changed by {TURN_ONSET_DEG} deg in the turn sign")
            cl = None
            if onset is not None and run.clearance_source == "live":
                cl = _value_at(onset, run.ti, run.clearance)
            _set(out, "turn_start_clear_m", cl,
                 "no turn onset" if onset is None else f"clearance_source={run.clearance_source} or no sample")
            head_ok = None if hd is None else abs(step[1] * hd - TURN_TARGET_DEG) <= TURN_TOL_DEG
            lo = WALL_TURN_START_TARGET_M - WALL_TURN_START_TOL_M
            hi = WALL_TURN_START_TARGET_M + WALL_TURN_START_TOL_M
            dist_ok = _wall_proxy_ok(cl, lo, hi)
            results.append(kleene_and(head_ok, dist_ok))
            bases.append(f"wall_turn: heading {hd} deg vs {TURN_TARGET_DEG}+-{TURN_TOL_DEG}, onset "
                         f"clearance {cl} m (proxy of wall distance) in {lo:.1f}..{hi:.1f} (D2)")
        elif kind == "go_to":
            cls = step[1]
            tgt, why = _target_distance(run, seg, cls, _robot_radius(run)[0]) if run.pose_valid else (None, pose_why)
            if tgt is not None:
                _set(out, "target_dist_center_m", tgt["center_m"])
                _set(out, "target_gap_m", tgt["gap_m"])
                _set(out, "target_bearing_deg", tgt["bearing_deg"])
            else:
                for k in ("target_dist_center_m", "target_gap_m", "target_bearing_deg"):
                    _set(out, k, None, why)
            gap = tgt["gap_m"] if tgt else None
            if cls == "person":
                results.append(_in(gap, PERSON_TARGET_M - PERSON_TOL_M, PERSON_TARGET_M + PERSON_TOL_M))
                if gap is not None:
                    goal_error = gap - PERSON_TARGET_M
                bases.append(f"go_to person: gap {gap} m in {PERSON_TARGET_M}+-{PERSON_TOL_M} (D5 tracks)")
            elif cls == "potted plant":
                facing = None if tgt is None else abs(tgt["bearing_deg"]) <= FACING_MAX_DEG
                results.append(kleene_and(_in(gap, -math.inf, PLANT_MAX_GAP_M), facing))
                bases.append(f"go_to plant: gap {gap} m <= {PLANT_MAX_GAP_M}, |bearing| <= {FACING_MAX_DEG} deg")
            else:
                results.append(None)
                bases.append(f"go_to {cls}: stop criterion to be clarified in RULES §6 (G12 note '0.9')")
    task = kleene_and(*results)
    out["task_basis"] = "; ".join(bases)
    _set(out, "task_ok", task, "a criterion is not measurable: " + out["task_basis"])
    if goal_error is not None:
        _set(out, "goal_error_m", goal_error)
    elif out.get("goal_error_m") is None:
        out["goal_error_m_why"] = "no distance-type criterion measured"
    if plan_error is not None:
        _set(out, "plan_error_m", plan_error)


# --------------------------------------------------------------------------
# public entry point
# --------------------------------------------------------------------------

def compute_run_metrics(run, ostacolo=None, verdicts=None):
    """All RULES §5-7 values for one run, as a flat dict (plus ``move_segments``).

    ``run`` is a :class:`RunV2` or a run folder path. ``ostacolo`` comes from
    ``selection.yaml`` (None = unknown). ``verdicts`` is the dict from
    :func:`load_manual_verdicts` (None -> loaded from the default path).
    """
    if not isinstance(run, RunV2):
        run = load_run_v2(run)
    if verdicts is None:
        verdicts = load_manual_verdicts()
    out = {"test_id": run.test_id, "mission": run.mission,
           "mission_code": run.mission.split("_")[0], "ostacolo": ostacolo,
           "run_dir": str(run.run_dir)}
    if run.drive_window is not None:
        out["drive_t0_s"], out["drive_t1_s"] = float(run.drive_window[0]), float(run.drive_window[1])
    else:
        _set(out, "drive_t0_s", None, run.drive_why)
        _set(out, "drive_t1_s", None, run.drive_why)

    _quality(run, out)
    _context(run, out)
    radius = out["robot_radius_used_m"]

    meta = run.meta or {}
    summ = _summary(meta)
    prompt_text = summ.get("prompt_text") or meta.get("prompt_text")
    steps, prompt_used, prompt_source, spec_why = expected_spec(prompt_text, run.mission)
    out["prompt_text"] = prompt_used
    out["prompt_source"] = prompt_source
    out["expected_steps"] = [list(s) for s in steps] if steps else None
    out["expected_steps_why"] = spec_why

    plan, plan_source = extract_plan(run)
    out["plan_source"] = plan_source
    plan_ok, plan_why, aligned = _plan_ok(steps, plan)
    _set(out, "plan_ok", plan_ok, plan_why)
    if plan_ok is False:
        out["plan_ok_why"] = plan_why
    elif plan_ok and plan and len(plan.get("moves") or []) > len(steps or []):
        out["plan_ok_why"] = "ok; extra trailing straight move(s) tolerated"
    # align moves to steps only by position; a mismatched plan can still be
    # judged on task when it has the same number of moves as expected steps
    if not aligned and plan is not None and steps is not None:
        aligned = len(plan.get("moves") or []) == len(steps)

    try:
        segs, segs_why = segment_moves(run, plan)
    except Exception as exc:  # never crash the batch on a malformed plan
        segs, segs_why = None, f"segmentation error: {exc!r}"

    _clearance(run, out)
    _solver(run, out)
    _jerk(run, out)
    _moving_pct(run, out)
    _steer(run, out)
    _horizon(run, out)
    _corridor(run, plan, segs, out, radius)
    _evaluate(run, steps, plan, aligned, segs, segs_why, ostacolo, out, radius)

    # ---- safe (RULES §5): estop, contact event, manual contact, LiDAR <= 0
    t0, t1 = run.drive_window if run.drive_window is not None else (-math.inf, math.inf)
    in_win = [e for e in run.eventi if _f(e.get("t")) is not None
              and t0 - EVENT_WINDOW_SLACK_S <= e["t"] <= t1 + EVENT_WINDOW_SLACK_S]
    estops = [e for e in in_win if e.get("event") == "estop"]
    contacts = [e for e in in_win if e.get("event") == "contact"]
    out["estop_n"] = len(estops) if run.eventi else None
    out["estop_causes"] = sorted({str(e.get("cause")) for e in estops}) if run.eventi else None
    out["contact_event_n"] = len(contacts) if run.eventi else None
    for k in ("estop_n", "estop_causes", "contact_event_n"):
        out[f"{k}_why"] = None if run.eventi else "no events.jsonl"
    v = verdicts.get(run.test_id) or {}
    cm = v.get("contact_manual")
    out["contact_manual"] = cm if isinstance(cm, bool) else None
    out["contact_manual_why"] = v.get("reason") if v else "no manual verdict"
    sm = v.get("success_manual")
    out["success_manual"] = sm if isinstance(sm, bool) else None
    out["success_manual_why"] = (
        (v.get("success_manual_reason") or v.get("reason") or "manual override, no reason given")
        if isinstance(sm, bool) else "no success_manual in manual_verdicts.yaml")
    if out["contact_manual"] is None and v:
        out["contact_manual_why"] = f"verdict present but contact_manual unset/ambiguous: {v.get('reason')}"
    reasons = []
    if estops:
        reasons.append(f"estop ({', '.join(out['estop_causes'])})")
    if contacts:
        reasons.append("contact event")
    if out["contact_manual"] is True:
        reasons.append("contact_manual (manual_verdicts.yaml)")
    if out.get("contact_lidar") is True:
        reasons.append("contact_lidar (clearance <= 0)")
    if reasons:
        _set(out, "safe", False)
        out["safe_why"] = "; ".join(reasons)
    elif not run.eventi:
        _set(out, "safe", None, "no events.jsonl: estop/contact events unknown")
    else:
        _set(out, "safe", True)

    succ = kleene_and(out["plan_ok"], out["exec_ok"], out["safe"], out["task_ok"])
    nulls = [k for k in ("plan_ok", "exec_ok", "safe", "task_ok") if out[k] is None]
    _set(out, "success", succ, f"unknown: {', '.join(nulls)} null")
    if succ is False:
        out["success_why"] = "false: " + ", ".join(
            k for k in ("plan_ok", "exec_ok", "safe", "task_ok") if out[k] is False)
    _YAW_CACHE.pop(id(run), None)
    return out


# --------------------------------------------------------------------------
# aggregation (RULES §8): helpers only, TABLES writes the files
# --------------------------------------------------------------------------

def wilson(k, n, z=1.96):
    """Wilson score interval ``(lo, hi)``, same signature as ``analyze_tests.wilson``."""
    if not n:
        return (None, None)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def summarize_continuous(values):
    """``{median, min, max, n_valid, n_null}`` (no mean, RULES §8)."""
    vals = [float(v) for v in values if _f(v) is not None]
    n_null = len(values) - len(vals)
    if not vals:
        return {"median": None, "min": None, "max": None, "n_valid": 0, "n_null": n_null}
    return {"median": float(np.median(vals)), "min": min(vals), "max": max(vals),
            "n_valid": len(vals), "n_null": n_null}


_BOOL_COLS = ("plan_ok", "exec_ok", "safe", "task_ok", "success")


def aggregate_group(rows, continuous_cols=None):
    """Aggregate per-run dicts of ONE group (same mission + same ostacolo).

    Booleans: ``k/N`` over non-null values with Wilson 95% plus the null
    count. Continuous: :func:`summarize_continuous`. Flags mixed
    ``git_commit`` values (RULES §8, not same-code comparison).
    """
    out = {"n_runs": len(rows)}
    for col in _BOOL_COLS:
        vals = [r.get(col) for r in rows]
        valid = [v for v in vals if v is not None]
        k = sum(1 for v in valid if v)
        lo, hi = wilson(k, len(valid))
        out[col] = {"k": k, "n": len(valid), "n_null": len(vals) - len(valid),
                    "wilson95_lo": lo, "wilson95_hi": hi}
    if continuous_cols is None:
        continuous_cols = sorted({k for r in rows for k, v in r.items()
                                  if not k.endswith("_why") and isinstance(v, float)})
    for col in continuous_cols:
        out[col] = summarize_continuous([r.get(col) for r in rows])
    commits = sorted({r.get("git_commit") for r in rows if r.get("git_commit")})
    out["git_commits"] = commits
    out["mixed_code"] = len(commits) > 1
    out["ostacolo_values"] = sorted({str(r.get("ostacolo")) for r in rows})
    return out


if __name__ == "__main__":  # debug: python3 scripts/metrics.py <run_dir>...
    import sys

    for arg in sys.argv[1:]:
        res = compute_run_metrics(arg)
        res.pop("move_segments", None)
        print(json.dumps(res, indent=1, default=str))
