"""Overhead ("dall'alto") real-time videos for the ``analysis_v2`` pipeline
(VIDEO_OVERHEAD).

Writes ``video_dall_alto/<run>.mp4`` for every run flagged ``dall_alto=si``
in ``selection.yaml``: white-background, top-down view, real time, with the
car's real footprint + driven trajectory, the active MPC corridor, the MPC
predicted horizon, tracked obstacles/targets (coloured by class), a side
panel (time, speed, steer, clearance, solver status, mission state) and a
legend.

Data sources (card ``Fuori scope``: no bag/ROS reading here):

* :mod:`run_data` (``RunV2``): ``kinematics.csv`` (x, y, yaw, speed), raw
  ``obstacle_clearance``, ``mpc.csv`` (solver status), ``commands.csv``
  (``cmd_steer``), ``corridors.jsonl`` records, ``events.jsonl``.
* ``tracks.jsonl``/``horizon.jsonl`` are read the same way :mod:`metrics`
  already does (``_read_jsonl`` on the run folder) -- these are already
  files the run itself wrote, not a bag, so this stays inside the card's
  "no bag/ROS" scope (``.sjck/interfaces/analysis_v2_data.md`` section 4's
  bag-archive note is about a *different*, harder path -- re-segmenting
  camera frames -- not needed here: ``tracks.jsonl``/``horizon.jsonl`` are
  already extracted per-run JSONL, present for the vast majority of runs,
  see that section's inventory. When a run has none, this module omits the
  layer and reports it, per the card, instead of falling back to a bag).
* Drawing constants (palette, per-stream max-age) and the car sagoma are
  taken from ``src/f1tenth_logger/f1tenth_logger/mission_render.py``
  (``.sjck/interfaces/analysis_v2_data.md`` section 3): only the small
  pure-Python pieces are reused here (literal colours/sizes/ages), since
  that module's own ``draw_frame``/``render_bag`` work on a *bag* dict and
  import ``matplotlib``/``pyarrow`` at module scope for a bag-replay CLI
  this module does not need; importing it would also pull in its
  ``argparse``/bag-discovery code for no benefit. No ``rclpy`` is imported
  anywhere in this file.
* Derived per-move series come from :mod:`metrics` (``extract_plan``,
  ``segment_moves``) -- never recomputed here, per the card.

Frame (single coherent transform for a whole run, declared once per the
card): see :data:`FRAME_TRANSFORM` and :func:`frame_transform`. In short,
every layer of every frame of one run is drawn in that run's own ``odom``
frame as already written by ``kinematics.csv``/``corridors.jsonl``/
``horizon.jsonl``; ``tracks.jsonl``'s ``map`` frame is plotted unchanged in
the same axes, since the car runs with no localisation node and
``map``->``odom`` is the identity (the same assumption ``metrics.py``'s
``_target_distance`` already documents and relies on). There is no
per-frame re-centring, rescaling or rotation: the view box is computed once
from the whole run and reused for every frame.

Usage (from ``first_test_campaign/analysis_v2``)::

    python3 -m scripts.video_overhead                  # every dall_alto=si run
    python3 -m scripts.video_overhead --runs P001-R008 --dry-run
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

try:  # package import (python3 -m scripts.video_overhead / from scripts.video_overhead)
    from .metrics import _f, _pose_at, _robot_radius, _value_at, extract_plan, segment_moves
    from .run_data import (
        DEFAULT_CAMPAIGN,
        RunResolutionError,
        _read_jsonl,
        find_root,
        group_key,
        load_run_v2,
        load_selection,
        resolve_run,
    )
except ImportError:  # direct script run: python3 scripts/video_overhead.py
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from metrics import (  # type: ignore[no-redef]
        _f,
        _pose_at,
        _robot_radius,
        _value_at,
        extract_plan,
        segment_moves,
    )
    from run_data import (  # type: ignore[no-redef]
        DEFAULT_CAMPAIGN,
        RunResolutionError,
        _read_jsonl,
        find_root,
        group_key,
        load_run_v2,
        load_selection,
        resolve_run,
    )

__all__ = [
    "FRAME_TRANSFORM",
    "frame_transform",
    "OverheadVideoResult",
    "render_overhead_video",
    "group_folder",
    "main",
]

# --------------------------------------------------------------------------
# frame transform (card requirement: one coherent transform, declared once)
# --------------------------------------------------------------------------

#: Exact string this module uses for ``build_info.json``'s ``frame_transform``
#: field (TABLES/this module may write that file; this is the single source
#: of truth for what the transform actually is).
FRAME_TRANSFORM = (
    "identity: map == odom (no localisation node running during the "
    "campaign). Every layer of a run -- kinematics.csv x/y/yaw (odom), "
    "corridors.jsonl polygons (odom), horizon.jsonl predicted states (odom) "
    "and tracks.jsonl detections (map) -- is drawn unchanged in one shared "
    "2D plane for the whole run; the view box is computed once from the "
    "full trajectory and never re-centred, rescaled or rotated frame to "
    "frame. Same assumption as metrics.py's _target_distance (D5)."
)


def frame_transform(x, y):
    """Identity transform used by every layer/frame of a run, see :data:`FRAME_TRANSFORM`."""
    return float(x), float(y)


# --------------------------------------------------------------------------
# palette + per-stream max age (mission_render.py literals, see module docstring)
# --------------------------------------------------------------------------

SURFACE_LIGHT = "#ffffff"  # card: "sfondo bianco" (mission_render's own SURFACE is dark)
PANEL_LIGHT = "#f0f0ee"
GRID_LIGHT = "#d8d8d4"
INK = "#1a1a19"
INK_MUTED = "#6b6a66"
CLASS_COLORS = ["#3987e5", "#d95926", "#199e70", "#8e44ad", "#c0392b", "#16a085"]
CLASS_OTHER = "#898781"
CORRIDOR_COLOR = "#3987e5"
HORIZON_COLOR = "#d55181"
CAR_COLOR = "#1a1a19"
STATUS = {"good": "#0ca30c", "warning": "#c98b00", "serious": "#d9631e", "critical": "#c23030"}

#: Same staleness gates mission_render.py's MAX_AGE uses for 'markers'
#: (camera/tracker stream) and the horizon (fresh every MPC solve). Both
#: match the actual per-run republish rate seen in ``horizon.jsonl``/
#: ``tracks.jsonl`` (~0.1s/~0.07s median), so a fixed age is safe here.
MAX_AGE_HORIZON_S = 0.5
MAX_AGE_TRACKS_S = 1.0
#: mission_render.py's own corridor max-age (0.5s = MPC_corr's
#: odom_stale_timeout_sec) assumes a live solver republishing close to every
#: control tick. Logged corridors.jsonl republishes only when the corridor
#: actually changes (observed ~1.0-1.1s apart on a quiet straight run,
#: analysis_v2_data.md does not give a number): a fixed 0.5s would blank the
#: corridor band for most of a video like that. Default (assumption, no
#: RULES.md number): 1.5x this run's own median corridor-record spacing,
#: floored at mission_render's 0.5s so a genuinely live run is unaffected.
MAX_AGE_CORRIDOR_FLOOR_S = 0.5
MAX_AGE_CORRIDOR_FACTOR = 1.5
#: How far "now" vs commands.csv/mpc.csv is still shown, not blanked (metrics.py's own default).
MAX_AGE_SCALAR_S = 0.3

#: Car sagoma (base_link triangle, metres) -- literally mission_render.py's
#: own ``body`` points in ``draw_frame`` ("sagoma reale" per the card).
CAR_BODY = ((0.26, 0.0), (-0.12, 0.13), (-0.05, 0.0), (-0.12, -0.13))

OUTPUT_FPS = 10.0  # default: matches video_mask.py's OUTPUT_FPS, real-time at playback
VIEW_PAD_M = 1.0
VIDEO_SUBDIR = "video_dall_alto"
#: Classes always in the legend first (campaign's own targets, same list as video_mask.py).
LEGEND_BASE_CLASSES = ("person", "chair", "bottle", "potted plant", "tv")


def _class_color(name, order):
    if name not in order:
        if len(order) < len(CLASS_COLORS):
            order[name] = len(order)
        else:
            order[name] = None
    slot = order[name]
    return CLASS_COLORS[slot] if slot is not None else CLASS_OTHER


# --------------------------------------------------------------------------
# result
# --------------------------------------------------------------------------

@dataclass
class OverheadVideoResult:
    run: str
    status: str  # "written" | "no_data" | "failed"
    video: str | None = None
    reason: str = ""
    n_frames: int = 0
    duration_s: float = 0.0
    fps: float = OUTPUT_FPS
    #: Data a card criterion was willing to omit per-frame rather than block
    #: the video (e.g. "no horizon.jsonl"), one string each.
    missing: list = field(default_factory=list)


# --------------------------------------------------------------------------
# per-run series (never recomputed from metrics.py -- read once, looked up per frame)
# --------------------------------------------------------------------------

def _corridor_max_age(corridors):
    """This run's corridor-record max age, see ``MAX_AGE_CORRIDOR_FACTOR``."""
    ts = sorted(_f(r.get("t")) for r in corridors if _f(r.get("t")) is not None)
    if len(ts) < 2:
        return MAX_AGE_CORRIDOR_FLOOR_S
    diffs = np.diff(np.asarray(ts, dtype=float))
    median_gap = float(np.median(diffs)) if diffs.size else 0.0
    return max(MAX_AGE_CORRIDOR_FLOOR_S, MAX_AGE_CORRIDOR_FACTOR * median_gap)


def _zoh_lookup(records, t_key="t"):
    """Sort ``records`` by ``t_key`` once; return a ``(t) -> last record with rec[t_key] <= t``."""
    recs = sorted((r for r in records if _f(r.get(t_key)) is not None), key=lambda r: r[t_key])
    times = np.array([r[t_key] for r in recs], dtype=float)

    def lookup(t, max_age=None):
        if times.size == 0:
            return None
        i = int(np.searchsorted(times, t, side="right")) - 1
        if i < 0:
            return None
        if max_age is not None and (t - times[i]) > max_age:
            return None
        return recs[i]

    return lookup


def _mission_state_at(eventi, t):
    """``(label, status_key)`` at ``t`` from ``events.jsonl`` (card: "stato missione").

    Terminal events (``mission_finished``/``mission_aborted``) stick once
    reached; ``estop``/``contact`` are shown for a short window after they
    fire (they are instantaneous log entries, not a state with its own end);
    otherwise "driving" once ``mission_started`` has happened, else "pre-roll".
    """
    past = [e for e in eventi if _f(e.get("t")) is not None and e["t"] <= t + 1e-9]
    if not past:
        return "pre-roll", "warning"
    past.sort(key=lambda e: e["t"])
    last_abort = next((e for e in reversed(past) if e.get("event") == "mission_aborted"), None)
    last_finish = next((e for e in reversed(past) if e.get("event") == "mission_finished"), None)
    if last_abort and (last_finish is None or last_abort["t"] >= last_finish["t"]):
        return f"ABORTED ({last_abort.get('reason', '?')})", "critical"
    if last_finish and (last_abort is None or last_finish["t"] > last_abort["t"]):
        return f"FINISHED ({last_finish.get('reason', '?')})", "good"
    for e in reversed(past):
        if e.get("event") in ("estop", "contact") and t - e["t"] <= 1.5:
            cause = e.get("cause", "?")
            return f"{e['event'].upper()} ({cause})", "critical"
    if any(e.get("event") == "mission_started" for e in past):
        return "driving", "good"
    return "pre-roll", "warning"


def _active_move_at(segs, t):
    if not segs:
        return None
    for seg in segs:
        if seg["t0"] - 0.05 <= t <= seg["t1"] + 0.05:
            return seg
    # between segments (e.g. just after the last one ends): show the latest started
    started = [s for s in segs if s["t0"] <= t]
    return started[-1] if started else None


# --------------------------------------------------------------------------
# rendering
# --------------------------------------------------------------------------

def _compute_view(x, y, tracks_records):
    xs = [v for v in x if math.isfinite(v)]
    ys = [v for v in y if math.isfinite(v)]
    for rec in tracks_records:
        for tr in rec.get("tracks") or []:
            tx, ty = _f(tr.get("x")), _f(tr.get("y"))
            if tx is not None and ty is not None:
                xs.append(tx)
                ys.append(ty)
    if not xs or not ys:
        return (-2.0, 2.0), (-2.0, 2.0)
    x0, x1 = min(xs) - VIEW_PAD_M, max(xs) + VIEW_PAD_M
    y0, y1 = min(ys) - VIEW_PAD_M, max(ys) + VIEW_PAD_M
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    half = max(0.5 * max(x1 - x0, y1 - y0), 1.5)
    return (cx - half, cx + half), (cy - half, cy + half)


def _draw_frame(ax_map, ax_panel, run, t, t0, t1, state):
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Circle
    from matplotlib.patches import Polygon as MplPolygon

    ax_map.clear()
    ax_panel.clear()

    ax_map.set_facecolor(SURFACE_LIGHT)
    ax_map.set_xlim(*state["xlim"])
    ax_map.set_ylim(*state["ylim"])
    ax_map.set_aspect("equal")
    ax_map.grid(True, color=GRID_LIGHT, linewidth=0.6, alpha=0.8)
    ax_map.tick_params(colors=INK_MUTED, labelsize=7)
    for spine in ax_map.spines.values():
        spine.set_color(GRID_LIGHT)
    ax_map.set_xlabel("x [m] (odom)", color=INK_MUTED, fontsize=8)
    ax_map.set_ylabel("y [m]", color=INK_MUTED, fontsize=8)
    ax_map.set_title(f"{run.test_id}  ({run.mission})", color=INK, fontsize=9, loc="left")

    # --- driven trajectory so far (card: "traiettoria percorsa") -----------
    ok = np.isfinite(run.ti) & np.isfinite(run.x) & np.isfinite(run.y) & (run.ti <= t)
    if ok.sum() >= 2:
        idx = np.flatnonzero(ok)
        pts = np.column_stack([run.x[idx], run.y[idx]])
        segs = np.stack([pts[:-1], pts[1:]], axis=1)
        age = np.clip((t - run.ti[idx][:-1]) / max(t1 - t0, 1e-6), 0.0, 1.0)
        alpha = 0.15 + 0.6 * (1.0 - age)
        lc = LineCollection(segs, colors=[(0.1, 0.1, 0.1, a) for a in alpha], linewidths=1.6, zorder=5)
        ax_map.add_collection(lc)

    # --- active MPC corridor (corridors.jsonl already gives the polygon) ---
    corr = state["corridor_at"](t, state["corridor_max_age_s"])
    if corr is not None:
        poly = [(p[0], p[1]) for p in (corr.get("polygon") or []) if len(p) >= 2]
        if len(poly) >= 3:
            ax_map.add_patch(MplPolygon(poly, closed=True, facecolor=CORRIDOR_COLOR,
                                        alpha=0.18, edgecolor=CORRIDOR_COLOR,
                                        linewidth=1.0, zorder=2))
    elif not state["has_corridor"]:
        pass  # already reported once in result.missing

    # --- MPC predicted horizon ----------------------------------------------
    hz = state["horizon_at"](t, MAX_AGE_HORIZON_S)
    if hz is not None:
        xs, ys = hz.get("x") or [], hz.get("y") or []
        n = min(len(xs), len(ys))
        if n >= 2:
            pts = np.array([[xs[k], ys[k]] for k in range(n)])
            segs = np.stack([pts[:-1], pts[1:]], axis=1)
            fade = np.linspace(0.9, 0.2, len(segs))
            widths = np.linspace(2.2, 0.6, len(segs))
            rgb = (0.835, 0.318, 0.506)  # HORIZON_COLOR as RGB
            ax_map.add_collection(LineCollection(segs, colors=[(*rgb, a) for a in fade],
                                                 linewidths=widths, zorder=7, capstyle="round"))
            ax_map.plot(pts[1:, 0], pts[1:, 1], linestyle="none", marker="o", markersize=2.4,
                       color=HORIZON_COLOR, alpha=0.8, zorder=7)

    # --- tracked obstacles/targets, coloured by class -----------------------
    trk = state["tracks_at"](t, MAX_AGE_TRACKS_S)
    if trk is not None:
        for tr in trk.get("tracks") or []:
            tx, ty = _f(tr.get("x")), _f(tr.get("y"))
            if tx is None or ty is None:
                continue
            cls = str(tr.get("class") or "?")
            color = _class_color(cls, state["class_order"])
            width = _f(tr.get("width")) or 0.2
            ax_map.add_patch(Circle((tx, ty), max(width / 2.0, 0.08), facecolor=color,
                                    alpha=0.85, edgecolor=INK, linewidth=0.8, zorder=8))
            label = f"{cls} #{tr.get('id', '?')}"
            score = _f(tr.get("score"))
            if score is not None:
                label += f" {score:.2f}"
            ax_map.text(tx + width / 2.0 + 0.05, ty, label, color=INK, fontsize=6.5, zorder=9)

    # --- car: real footprint + radius ring ---------------------------------
    pose = state["pose_fn"](t)
    if pose is not None:
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in CAR_BODY]
        ax_map.add_patch(MplPolygon(pts, closed=True, facecolor=CAR_COLOR,
                                    edgecolor=SURFACE_LIGHT, linewidth=1.0, zorder=10))
        ax_map.add_patch(Circle((x, y), state["robot_radius_m"], facecolor="none",
                                edgecolor=CAR_COLOR, alpha=0.4, linewidth=0.9,
                                linestyle=(0, (2, 3)), zorder=9))

    # --- side panel ----------------------------------------------------------
    ax_panel.set_facecolor(PANEL_LIGHT)
    ax_panel.set_xlim(0, 1)
    ax_panel.set_ylim(0, 1)
    ax_panel.set_xticks([])
    ax_panel.set_yticks([])
    for spine in ax_panel.spines.values():
        spine.set_color(GRID_LIGHT)

    speed = _value_at(t, run.ti, run.speed, max_dt=MAX_AGE_SCALAR_S)
    steer = state["steer_at"](t)
    clearance = _value_at(t, run.ti, run.clearance, max_dt=MAX_AGE_SCALAR_S)
    solver_rec = state["solver_at"](t)
    mission_label, mission_key = _mission_state_at(run.eventi, t)
    seg = _active_move_at(state["segs"], t)

    lines = []
    lines.append((f"t = {t - t0:6.2f} s / {t1 - t0:.2f} s", INK, 9.5, "bold"))
    lines.append((mission_label, STATUS.get(mission_key, INK), 9, "bold"))
    if seg is not None:
        lines.append((f"move {seg.get('idx')} ({seg.get('kind')})"
                      + ("  OK" if seg.get("reached") else "  ..."),
                      INK_MUTED, 7.5, "normal"))
    lines.append(("", INK, 3, "normal"))
    lines.append((f"speed   {speed:6.2f} m/s" if speed is not None else "speed   n/a",
                  INK, 8.5, "normal"))
    lines.append((f"steer   {math.degrees(steer):6.1f} deg" if steer is not None else "steer   n/a",
                  INK, 8.5, "normal"))
    lines.append((f"clearance {clearance:6.2f} m" if clearance is not None else "clearance n/a",
                  INK, 8.5, "normal"))
    if solver_rec is not None:
        status = str(solver_rec.get("status") or "?")
        color = STATUS["good"] if status in ("solved", "solved_inaccurate") else STATUS["critical"]
        lines.append((f"solver  {status}", color, 8.5, "normal"))
    else:
        lines.append(("solver  n/a", INK_MUTED, 8.5, "normal"))
    lines.append(("", INK, 5, "normal"))
    lines.append(("legend", INK_MUTED, 7.5, "bold"))
    lines.append(("  grey trail = path driven so far", INK, 7, "normal"))
    lines.append(("  blue band = active MPC corridor", CORRIDOR_COLOR, 7, "normal"))
    lines.append(("  pink ghost = MPC predicted horizon", HORIZON_COLOR, 7, "normal"))
    lines.append(("  dashed ring = car radius", INK, 7, "normal"))
    for cls, slot in sorted(state["class_order"].items(), key=lambda kv: (kv[1] is None, kv[1])):
        col = CLASS_COLORS[slot] if slot is not None else CLASS_OTHER
        lines.append((f"  {cls}", col, 7, "normal"))
    if state["missing"]:
        lines.append(("", INK, 4, "normal"))
        for m in state["missing"]:
            lines.append((f"  ({m})", INK_MUTED, 6.5, "normal"))

    y = 0.975
    for text, color, size, weight in lines:
        if text:
            ax_panel.text(0.04, y, text, color=color, fontsize=size, weight=weight,
                         va="top", ha="left", family="monospace",
                         transform=ax_panel.transAxes)
        y -= (size + 4.5) / 420.0


# --------------------------------------------------------------------------
# main entry point per run
# --------------------------------------------------------------------------

def group_folder(run_dir, ostacolo):
    """Canonical RULES.md §6/§9/§11 group folder name (see ``run_data.group_key``),
    same convention as video_mask.py."""
    mission = Path(run_dir).parent.name
    return group_key(mission, ostacolo)


def render_overhead_video(run_dir, out_dir, *, run=None, fps=OUTPUT_FPS, dry_run=False):
    """Make the overhead video of one run in ``out_dir``. Never raises for an expected case.

    Returns an :class:`OverheadVideoResult`. ``status`` is one of:

    ``written``
        ``<run>.mp4`` written.
    ``no_data``
        No drive window / no finite kinematics sample to draw from.
    ``failed``
        Unexpected error while rendering/writing (``reason``).

    Missing per-run layers (no ``corridors.jsonl``, no ``horizon.jsonl``, no
    ``tracks.jsonl``, no ``mpc.csv``/``commands.csv`` data) are reported in
    ``result.missing`` and simply omitted frame by frame -- never block the
    video, per the card.

    With ``dry_run=True`` only the decision/window is computed: no
    matplotlib figure or ffmpeg process is created.
    """
    run_dir = Path(run_dir)
    run_code = run or run_dir.name
    out_dir = Path(out_dir)
    res = OverheadVideoResult(run=run_code, status="no_data", fps=fps)

    try:
        rv = load_run_v2(run_dir)
    except Exception as exc:  # noqa: BLE001 - one bad run folder never stops the others
        res.status, res.reason = "failed", f"{type(exc).__name__} loading the run: {exc}"
        return res

    if rv.drive_window is not None:
        t0, t1 = rv.drive_window
    else:
        finite_t = rv.ti[np.isfinite(rv.ti)] if rv.ti.size else rv.ti
        if finite_t.size < 2:
            res.reason = "no drive window and no kinematics.csv sample to fall back on"
            return res
        t0, t1 = float(finite_t.min()), float(finite_t.max())
    if not math.isfinite(t0) or not math.isfinite(t1) or t1 <= t0:
        res.reason = f"drive window is empty or inverted ({t0}..{t1})"
        return res

    tracks_records = _read_jsonl(run_dir / "tracks.jsonl")
    horizon_records = _read_jsonl(run_dir / "horizon.jsonl")

    missing = []
    if not rv.corridors:
        missing.append("no corridors.jsonl: MPC corridor omitted from every frame")
    if not horizon_records:
        missing.append("no horizon.jsonl: MPC predicted horizon omitted from every frame")
    if not tracks_records:
        missing.append("no tracks.jsonl: tracked obstacles/targets omitted from every frame")
    if rv.solver_states["t"].size == 0:
        missing.append("no mpc.csv: solver status panel omitted")
    if rv.commands["t"].size == 0:
        missing.append("no commands.csv: steer panel omitted")
    res.missing = missing

    n_frames = max(1, int(round((t1 - t0) * fps)) + 1)
    res.n_frames = n_frames
    res.duration_s = t1 - t0
    video = out_dir / f"{run_code}.mp4"
    res.video = str(video)
    if dry_run:
        res.status = "written"  # decision only; nothing written yet
        return res

    plan, _plan_source = extract_plan(rv)
    segs, segs_why = segment_moves(rv, plan) if plan is not None else (None, "no plan")
    segs = segs or []
    if plan is None or segs_why:
        missing.append(f"mission-move panel omitted: {segs_why or 'no plan'}")
    robot_radius_m, _raw = _robot_radius(rv)
    corridor_at = _zoh_lookup(rv.corridors)
    horizon_at = _zoh_lookup(horizon_records)
    tracks_at = _zoh_lookup(tracks_records)

    cmd_t = rv.commands.get("t", np.empty(0))
    cmd_steer = rv.commands.get("cmd_steer", np.empty(0))

    def steer_at(tq):
        return _value_at(tq, cmd_t, cmd_steer, max_dt=MAX_AGE_SCALAR_S) if cmd_t.size else None

    solver_t = rv.solver_states.get("t", np.empty(0))
    solver_status = rv.solver_states.get("status", np.empty(0, dtype=object))

    def solver_at(tq):
        if solver_t.size == 0:
            return None
        ok = np.isfinite(solver_t)
        if not ok.any():
            return None
        tt = solver_t[ok]
        i = int(np.argmin(np.abs(tt - tq)))
        if abs(tt[i] - tq) > MAX_AGE_SCALAR_S:
            return None
        return {"status": solver_status[ok][i]}

    yaw_u = np.array(rv.yaw, dtype=float)
    yaw_ok = np.isfinite(yaw_u)
    if yaw_ok.any():
        yaw_u[yaw_ok] = np.unwrap(yaw_u[yaw_ok])

    def pose_fn(tq):
        p = _pose_at(rv, tq)
        if p is None or p[2] is None:
            return None
        x, y, yaw_unwrapped = p
        yaw = math.atan2(math.sin(yaw_unwrapped), math.cos(yaw_unwrapped))
        return x, y, yaw

    xlim, ylim = _compute_view(rv.x, rv.y, tracks_records)
    state = {
        "xlim": xlim,
        "ylim": ylim,
        "corridor_at": corridor_at,
        "horizon_at": horizon_at,
        "tracks_at": tracks_at,
        "pose_fn": pose_fn,
        "steer_at": steer_at,
        "solver_at": solver_at,
        "segs": segs,
        "robot_radius_m": robot_radius_m,
        "class_order": {},
        "missing": missing,
        "has_corridor": bool(rv.corridors),
        "corridor_max_age_s": _corridor_max_age(rv.corridors),
    }

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FFMpegWriter

        fig = plt.figure(figsize=(9.6, 6.0), dpi=120, facecolor=SURFACE_LIGHT)
        gs = fig.add_gridspec(1, 2, width_ratios=(3.0, 1.0), left=0.06, right=0.98,
                             top=0.95, bottom=0.08, wspace=0.03)
        ax_map = fig.add_subplot(gs[0, 0])
        ax_panel = fig.add_subplot(gs[0, 1])

        out_dir.mkdir(parents=True, exist_ok=True)
        writer = FFMpegWriter(fps=fps, codec="libx264",
                              extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
        times = [t0 + k / fps for k in range(n_frames)]
        with writer.saving(fig, str(video), dpi=120):
            for t in times:
                _draw_frame(ax_map, ax_panel, rv, t, t0, t1, state)
                writer.grab_frame()
        plt.close(fig)
        res.status = "written"
    except Exception as exc:  # noqa: BLE001 - one bad run never stops the others
        res.status, res.video = "failed", None
        res.reason = f"{type(exc).__name__}: {exc}"
    return res


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _summary_lines(results):
    lines = []
    for r in results:
        line = f"{r.run:28s} {r.status:10s} {r.video or '-'}"
        if r.reason:
            line += f"\n{'':39s}{r.reason}"
        for m in r.missing:
            line += f"\n{'':39s}missing: {m}"
        lines.append(line)
    lines.append("")
    for status in ("written", "no_data", "failed"):
        n = sum(1 for r in results if r.status == status)
        lines.append(f"  {status:10s} {n}")
    return lines


def main(argv=None):
    here = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--selection", type=Path, default=here / "selection.yaml")
    ap.add_argument("--out-root", type=Path, default=None,
                    help="default: <analysis_v2>/<output_folder from selection.yaml>")
    ap.add_argument("--runs", nargs="*", default=None,
                    help="only these run codes (full test_id or P0nn-R0nn)")
    ap.add_argument("--fps", type=float, default=OUTPUT_FPS)
    ap.add_argument("--dry-run", action="store_true",
                    help="decide per run and compute the window, do not render/write a video")
    args = ap.parse_args(argv)

    selection = load_selection(args.selection)
    out_root = args.out_root or (args.selection.resolve().parent / selection.output_folder)
    campaign = find_root() / DEFAULT_CAMPAIGN

    results = []
    for row in selection:
        if not row.dall_alto:
            continue
        if args.runs and not any(row.run == c or row.run.startswith(f"{c}-") for c in args.runs):
            continue
        try:
            run_dir = resolve_run(row.run, [campaign])
        except RunResolutionError as exc:
            results.append(OverheadVideoResult(run=row.run, status="failed", reason=str(exc)))
            continue
        out_dir = out_root / group_folder(run_dir, row.ostacolo) / VIDEO_SUBDIR
        res = render_overhead_video(run_dir, out_dir, run=row.run, fps=args.fps,
                                    dry_run=args.dry_run)
        results.append(res)
        print(f"[video_overhead] {res.run}: {res.status} {res.video or ''} {res.reason}", flush=True)

    print("\n".join(_summary_lines(results)))
    return 0 if all(r.status != "failed" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
