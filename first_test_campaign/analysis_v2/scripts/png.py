"""Static PNG figures for the ``analysis_v2`` pipeline (PNG node).

Two entry points, per the card:

* :func:`render_run_png` -- one figure per run (``png_singole/<test_id>.png``
  when ``selection.yaml``'s ``png`` flag is ``si``): trajectory + corridor +
  obstacles/target, speed, steering, clearance/solver-status over time.
* :func:`render_group_png` -- one figure per ``<mission>_<con|no>_ostacolo``
  group (:func:`run_data.group_key`, RULES.md §6/§11 -- the FULL mission
  folder name, not just its ``Mxx`` prefix), overlaying the runs flagged
  ``selezionate=si`` in that group.

This module only draws. All raw/derived data comes from ``run_data.py``
(``RunV2``: raw traces + ``drive``/``moving`` windows) and ``metrics.py``
(``compute_run_metrics``: the scalar RULES.md §5-7 values, e.g.
``min_clear_any_m``/``t_min_clear_s``/``solver_usable_pct``) -- nothing here
re-derives a metric, per the card's "non ricalcolare le metriche qui".

Colour palette: reused from ``f1tenth_logger.mission_render`` (ROS-free,
``.sjck/interfaces/analysis_v2_data.md`` §3) for visual consistency with the
MP4 renderer, via :func:`_mission_render_palette`. ``mission_render``'s
*drawing* functions (``draw_frame``, ``compute_view``, ...) are NOT reused:
they operate on a bag dict (``.extract.parquet``), while this module draws
straight from a run folder's own CSV/JSONL files (no bag, no ROS) -- the
interface doc is explicit that "tutto il disegno [in mission_render] lavora
sul bag dict, non sulle cartelle run". Only the palette constants and the
pure helpers (``class_color``) are signature-compatible with what this
module needs.

RULES.md §8 / the card's "Fuori scope": a selected-group figure never mixes
two missions, nor (when the caller supplies it) two different ``ostacolo``
values -- :func:`render_group_png` raises :class:`ValueError` instead of
silently drawing them together.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

try:  # package import: ``from scripts.png import ...``
    from .run_data import (
        MOVING_SPEED_THRESHOLD_MPS,
        RunV2,
        _read_jsonl,
        find_root,
        group_key,
        load_run_v2,
    )
    from .metrics import compute_run_metrics
except ImportError:  # direct execution: ``python3 scripts/png.py``
    from run_data import (  # type: ignore[no-redef]
        MOVING_SPEED_THRESHOLD_MPS,
        RunV2,
        _read_jsonl,
        find_root,
        group_key,
        load_run_v2,
    )
    from metrics import compute_run_metrics  # type: ignore[no-redef]

__all__ = ["render_run_png", "render_group_png", "group_label", "GroupKeyCollisionError"]

# --------------------------------------------------------------------------
# palette (reused from mission_render.py, see module docstring)
# --------------------------------------------------------------------------

#: Fallback palette, kept byte-identical to mission_render.py's own values,
#: used only if that module cannot be imported (e.g. run outside the
#: f1tenth_more tree with no ``src/`` to locate) -- never a different look.
_FALLBACK_PALETTE = {
    "SURFACE": "#1a1a19",
    "PANEL": "#121211",
    "INK": "#ffffff",
    "INK_SECONDARY": "#c3c2b7",
    "INK_MUTED": "#898781",
    "GRID": "#2c2c2a",
    "CLASS_COLORS": ["#3987e5", "#d95926", "#199e70"],
    "CLASS_OTHER": "#898781",
    "STATUS": {
        "good": "#0ca30c",
        "warning": "#fab219",
        "serious": "#ec835a",
        "critical": "#d03b3b",
    },
    "CORRIDOR_COLOR": "#3987e5",
    "HORIZON_COLOR": "#d55181",
    "CAR_COLOR": "#ffffff",
}

_PALETTE_CACHE: dict | None = None


def _mission_render_palette():
    """Palette dict + ``class_color`` fn, from ``mission_render`` if importable.

    ``mission_render.py`` is a plain file inside a ROS package
    (``src/f1tenth_logger/f1tenth_logger/``), not installed as a regular
    Python package here, so it is reached by adding its directory to
    ``sys.path`` (same ROS-free module the interface doc describes -- no
    ``rclpy`` import happens as a side effect). Cached after the first call.
    """
    global _PALETTE_CACHE
    if _PALETTE_CACHE is not None:
        return _PALETTE_CACHE

    try:
        root = find_root()
        mr_dir = root / "src" / "f1tenth_logger" / "f1tenth_logger"
        if str(mr_dir) not in sys.path:
            sys.path.insert(0, str(mr_dir))
        import mission_render as mr  # type: ignore

        palette = {
            "SURFACE": mr.SURFACE,
            "PANEL": mr.PANEL,
            "INK": mr.INK,
            "INK_SECONDARY": mr.INK_SECONDARY,
            "INK_MUTED": mr.INK_MUTED,
            "GRID": mr.GRID,
            "CLASS_COLORS": mr.CLASS_COLORS,
            "CLASS_OTHER": mr.CLASS_OTHER,
            "STATUS": mr.STATUS,
            "CORRIDOR_COLOR": mr.CORRIDOR_COLOR,
            "HORIZON_COLOR": mr.HORIZON_COLOR,
            "CAR_COLOR": mr.CAR_COLOR,
        }
        class_color = mr.class_color
    except Exception:
        palette = dict(_FALLBACK_PALETTE)

        def class_color(class_id, order):  # same contract as mr.class_color
            if class_id not in order:
                if len(order) < len(palette["CLASS_COLORS"]):
                    order[class_id] = len(order)
                else:
                    order[class_id] = None
            slot = order[class_id]
            return palette["CLASS_COLORS"][slot] if slot is not None else palette["CLASS_OTHER"]

    _PALETTE_CACHE = (palette, class_color)
    return _PALETTE_CACHE


#: Solver status -> palette STATUS key (RULES.md §7, metrics.py SOLVER_* tuples).
_SOLVER_STATUS_ORDER = ("infeasible", "other", "max_iter", "solved_inaccurate", "solved")
_SOLVER_STATUS_LEVEL_KEY = {
    "solved": "good",
    "solved_inaccurate": "warning",
    "max_iter": "serious",
    "infeasible": "critical",
    "other": None,  # -> INK_MUTED
}


# --------------------------------------------------------------------------
# small shared helpers
# --------------------------------------------------------------------------

def _as_run(run):
    return run if isinstance(run, RunV2) else load_run_v2(run)


def _window_mask(t, window):
    if window is None:
        return np.zeros_like(t, dtype=bool)
    t0, t1 = window
    return np.isfinite(t) & (t >= t0) & (t <= t1)


def _style_axes(ax, palette, title):
    ax.set_facecolor(palette["PANEL"])
    ax.grid(True, color=palette["GRID"], linewidth=0.6, alpha=0.8)
    ax.tick_params(colors=palette["INK_SECONDARY"], labelsize=8)
    for spine in ax.spines.values():
        spine.set_color(palette["GRID"])
    ax.set_title(title, color=palette["INK"], fontsize=10)
    ax.xaxis.label.set_color(palette["INK_SECONDARY"])
    ax.yaxis.label.set_color(palette["INK_SECONDARY"])


def _last_track_positions(run_dir, window, class_color_fn, order, palette):
    """``[(x, y, color, class)]``: last-seen position per track id in ``window``."""
    recs = _read_jsonl(Path(run_dir) / "tracks.jsonl")
    last = {}
    for rec in recs:
        t = rec.get("t")
        if t is None or window is None or not (window[0] - 0.5 <= t <= window[1] + 0.5):
            continue
        for tr in rec.get("tracks") or []:
            x, y = tr.get("x"), tr.get("y")
            if x is None or y is None:
                continue
            last[tr.get("id")] = (float(x), float(y), tr.get("class"), t)
    out = []
    for x, y, cls, _t in last.values():
        out.append((x, y, class_color_fn(cls, order), cls))
    return out


def _draw_corridors(ax, run, window, palette, max_count=6):
    recs = [r for r in run.corridors if r.get("t") is not None and window is not None
            and window[0] - 0.5 <= r["t"] <= window[1] + 0.5 and r.get("polygon")]
    if not recs:
        return
    step = max(1, len(recs) // max_count)
    for rec in recs[::step]:
        poly = np.asarray(rec["polygon"], dtype=float)
        if poly.ndim != 2 or poly.shape[0] < 3:
            continue
        closed = np.vstack([poly, poly[:1]])
        ax.fill(closed[:, 0], closed[:, 1], color=palette["CORRIDOR_COLOR"], alpha=0.06, zorder=1)
        ax.plot(closed[:, 0], closed[:, 1], color=palette["CORRIDOR_COLOR"], alpha=0.35,
                 linewidth=0.8, zorder=1)


def _draw_map_panel(ax, run, window, palette, class_order, label=None, color=None):
    """Trajectory (+ corridors + obstacles on single-run calls) for one run."""
    ok = np.isfinite(run.ti) & np.isfinite(run.x) & np.isfinite(run.y)
    if window is not None:
        ok = ok & _window_mask(run.ti, window)
    x, y = run.x[ok], run.y[ok]
    car_color = color or palette["CAR_COLOR"]
    if x.size:
        ax.plot(x, y, color=car_color, linewidth=1.6, label=label, zorder=3)
        ax.scatter([x[0]], [y[0]], marker="o", s=28, color=palette["STATUS"]["good"],
                    zorder=4, label=None)
        ax.scatter([x[-1]], [y[-1]], marker="s", s=28, color=palette["STATUS"]["critical"],
                    zorder=4, label=None)
    return x, y


def _draw_solver_clearance(ax, run, window, palette, metrics=None):
    """Clearance line (left) + solver-status step (right, twin axis).

    The minimum-clearance marker uses ``metrics``'s ``t_min_clear_s``/
    ``min_clear_any_m`` (RULES.md §1.6/§5-7, :func:`metrics.compute_run_metrics`)
    when given, instead of recomputing it with a local ``argmin`` -- the
    local ``argmin`` is only a fallback for when no metrics dict is passed.
    """
    t0, t1 = window if window is not None else (None, None)
    if run.clearance_source == "live":
        mask = _window_mask(run.ti, window) & np.isfinite(run.clearance)
        t, c = run.ti[mask], run.clearance[mask]
        if t.size:
            order = np.argsort(t, kind="stable")
            t, c = t[order], c[order]
            ax.plot(t, c, color=palette["INK"], linewidth=1.3, label="clearance (m)")
            t_min = metrics.get("t_min_clear_s") if metrics else None
            c_min = metrics.get("min_clear_any_m") if metrics else None
            if t_min is None or c_min is None:
                i = int(np.argmin(c))
                t_min, c_min = t[i], c[i]
            ax.scatter([t_min], [c_min], marker="v", s=36, color=palette["STATUS"]["critical"],
                        zorder=4)
        else:
            ax.text(0.02, 0.9, "clearance: nessun campione nella finestra drive",
                    transform=ax.transAxes, color=palette["INK_MUTED"], fontsize=8)
    else:
        ax.text(0.02, 0.9, f"clearance_source={run.clearance_source}: traccia live non usabile",
                transform=ax.transAxes, color=palette["INK_MUTED"], fontsize=8)
    ax.set_ylabel("clearance (m)")

    ax2 = ax.twinx()
    ax2.set_facecolor("none")
    ax2.tick_params(colors=palette["INK_SECONDARY"], labelsize=7)
    st = run.solver_states
    mask = _window_mask(st.get("t", np.empty(0)), window)
    t = st.get("t", np.empty(0))[mask]
    status = np.asarray(st.get("status", np.empty(0, dtype=object)))[mask]
    if t.size:
        order = np.argsort(t, kind="stable")
        t = t[order]
        lab = np.array([str(s).strip().lower() for s in status[order]], dtype=object)
        known = set(_SOLVER_STATUS_ORDER[:-1])  # all but "other"
        lab = np.where(np.isin(lab, list(known)), lab, "other")
        y = np.array([_SOLVER_STATUS_ORDER.index(v) for v in lab], dtype=float)
        colors = [
            palette["STATUS"].get(_SOLVER_STATUS_LEVEL_KEY[v], palette["INK_MUTED"])
            for v in lab
        ]
        ax2.scatter(t, y, c=colors, s=10, marker="|", zorder=2)
        ax2.set_yticks(range(len(_SOLVER_STATUS_ORDER)))
        ax2.set_yticklabels(_SOLVER_STATUS_ORDER, fontsize=7, color=palette["INK_SECONDARY"])
        ax2.set_ylim(-0.5, len(_SOLVER_STATUS_ORDER) - 0.5)
    else:
        ax2.text(0.02, 0.1, "solver: nessun campione nella finestra drive",
                 transform=ax.transAxes, color=palette["INK_MUTED"], fontsize=8)
    return ax2


def _finalize_figure(fig, palette, out_path):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.patch.set_facecolor(palette["SURFACE"])
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, facecolor=palette["SURFACE"])
    plt.close(fig)
    return out_path


class GroupKeyCollisionError(ValueError):
    """Two runs disagree on mission/ostacolo yet were asked to share one
    selected-group figure (RULES.md §6/§8/§11). Subclasses ``ValueError``
    so existing ``except ValueError`` callers keep working; the message
    always names the canonical :func:`run_data.group_key` involved so the
    cause is unambiguous (never a bare generic ``ValueError``).
    """


def group_label(mission, ostacolo):
    """Canonical group label for this mission+ostacolo combination.

    Delegates to :func:`run_data.group_key` (``RULES.md`` §6/§11) instead
    of the bare ``Mxx`` numeric prefix, so two missions that happen to
    share a prefix on disk (``M07_chair_then_straight_4m`` vs
    ``M07_stop_at_chair``) are never folded into one group/figure/
    filename -- matching the same convention ``tables.py``/``video_mask.py``/
    ``video_overhead.py`` use for the same group.
    """
    return group_key(mission, ostacolo)


# --------------------------------------------------------------------------
# render_run_png
# --------------------------------------------------------------------------

def render_run_png(run, out_path, metrics=None, ostacolo=None):
    """One PNG per run: map (traj+corridor+obstacles), speed, steer, clearance/solver.

    ``run`` is a :class:`RunV2` or a run folder path. ``metrics`` is the
    dict from :func:`metrics.compute_run_metrics`, as already computed by
    ``tables.build_all``; used as-is (never re-derived here) when given.
    ``ostacolo`` is the ``selection.yaml`` obstacle flag for this run; it is
    only consulted for the ``compute_run_metrics`` fallback below (so the
    recomputed metrics agree with ``results.csv``), and is ignored when
    ``metrics`` is already given. Returns the written :class:`Path`.
    """
    run = _as_run(run)
    if metrics is None:
        metrics = compute_run_metrics(run, ostacolo=ostacolo)
    palette, class_color_fn = _mission_render_palette()
    window = run.drive_window

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    ax_map, ax_speed, ax_steer, ax_cs = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

    _style_axes(ax_map, palette, f"{run.test_id} -- traiettoria / corridoio / ostacoli")
    _draw_corridors(ax_map, run, window, palette)
    x, y = _draw_map_panel(ax_map, run, window, palette, {}, color=palette["CAR_COLOR"])
    order = {}
    tracks = _last_track_positions(run.run_dir, window, class_color_fn, order, palette)
    for tx, ty, color, cls in tracks:
        ax_map.scatter([tx], [ty], marker="^", s=40, color=color, zorder=3, label=str(cls))
    if x.size:
        ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.set_xlabel("x odom (m)")
    ax_map.set_ylabel("y odom (m)")
    handles, labels = ax_map.get_legend_handles_labels()
    if handles:
        by_label = dict(zip(labels, handles))
        ax_map.legend(by_label.values(), by_label.keys(), fontsize=7,
                      facecolor=palette["PANEL"], labelcolor=palette["INK_SECONDARY"])

    _style_axes(ax_speed, palette, "velocita' (m/s)")
    mask = _window_mask(run.ti, window) & np.isfinite(run.speed)
    if mask.any():
        order_t = np.argsort(run.ti[mask], kind="stable")
        ax_speed.plot(run.ti[mask][order_t], run.speed[mask][order_t],
                       color=palette["CAR_COLOR"], linewidth=1.2)
        ax_speed.axhline(MOVING_SPEED_THRESHOLD_MPS, color=palette["INK_MUTED"],
                          linewidth=0.8, linestyle="--")
    else:
        ax_speed.text(0.05, 0.5, "nessun campione di velocita' nella finestra drive",
                       transform=ax_speed.transAxes, color=palette["INK_MUTED"], fontsize=8)
    ax_speed.set_xlabel("t (s)")
    ax_speed.set_ylabel("speed (m/s)")

    _style_axes(ax_steer, palette, "sterzo (rad)")
    cmd = run.commands
    mask = _window_mask(cmd.get("t", np.empty(0)), window) & np.isfinite(cmd.get("cmd_steer", np.empty(0)))
    if mask.any():
        t, s = cmd["t"][mask], cmd["cmd_steer"][mask]
        order_t = np.argsort(t, kind="stable")
        ax_steer.plot(t[order_t], s[order_t], color=palette["CLASS_COLORS"][1], linewidth=1.2)
    else:
        ax_steer.text(0.05, 0.5, "nessun comando di sterzo nella finestra drive",
                       transform=ax_steer.transAxes, color=palette["INK_MUTED"], fontsize=8)
    ax_steer.set_xlabel("t (s)")
    ax_steer.set_ylabel("cmd_steer (rad)")

    _style_axes(ax_cs, palette, "clearance e stato solver nel tempo")
    _draw_solver_clearance(ax_cs, run, window, palette, metrics=metrics)
    ax_cs.set_xlabel("t (s)")

    subtitle = (
        f"missione={run.mission}  drive={window}  "
        f"min_clear={metrics.get('min_clear_any_m')}  "
        f"solver_usable_pct={metrics.get('solver_usable_pct')}"
    )
    fig.suptitle(subtitle, color=palette["INK_SECONDARY"], fontsize=9)
    return _finalize_figure(fig, palette, out_path)


# --------------------------------------------------------------------------
# render_group_png
# --------------------------------------------------------------------------

def render_group_png(runs, out_path, ostacolo=None, metrics_list=None, label=None):
    """One PNG overlaying the given runs -- all must be ONE mission (+ ONE
    ``ostacolo`` value, RULES.md §8: "il gruppo con ostacolo non si mischia a
    quello senza").

    ``runs``: list of :class:`RunV2`/paths, all ``selezionate=si`` for the
    same :func:`run_data.group_key` group. ``ostacolo``: optional list
    parallel to ``runs`` (the selection.yaml flag, read by the caller --
    RULES.md: the group is defined by that flag, not by folder name); when
    given, a mismatch raises the same explicit error as a mission mismatch.
    ``metrics_list``: optional list parallel to ``runs``, the dicts from
    ``tables.build_all``/:func:`metrics.compute_run_metrics`; used as-is
    (e.g. for the per-run clearance-minimum marker) instead of being
    recomputed here when given. Raises :class:`GroupKeyCollisionError`
    (a :class:`ValueError`) instead of ever drawing two groups together.
    """
    run_objs = [_as_run(r) for r in runs]
    if not run_objs:
        raise ValueError("render_group_png: no runs given")

    missions = sorted({r.mission for r in run_objs})
    if len(missions) > 1:
        detail = ", ".join(f"{r.test_id}={r.mission}" for r in run_objs)
        raise GroupKeyCollisionError(
            f"render_group_png: runs from {len(missions)} different missions "
            f"{missions} map to different group_key() values and cannot be "
            f"drawn in one selected-group figure ({detail})"
        )

    if ostacolo is not None:
        if len(ostacolo) != len(run_objs):
            raise ValueError(
                f"render_group_png: ostacolo has {len(ostacolo)} values for "
                f"{len(run_objs)} runs"
            )
        values = sorted({bool(v) for v in ostacolo})
        if len(values) > 1:
            detail = ", ".join(
                f"{r.test_id}={group_key(r.mission, v)}" for r, v in zip(run_objs, ostacolo)
            )
            raise GroupKeyCollisionError(
                f"render_group_png: runs with mixed ostacolo flags {values} "
                f"map to different group_key() values and cannot be drawn in "
                f"one selected-group figure ({detail})"
            )
        ost = values[0]
    else:
        ost = None

    if metrics_list is None:
        metrics_list = [compute_run_metrics(r, ostacolo=ost) for r in run_objs]

    palette, class_color_fn = _mission_render_palette()
    mission = missions[0]
    title_label = label or group_label(mission, ost if ost is not None else False)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    ax_map, ax_speed, ax_steer, ax_cs = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]
    _style_axes(ax_map, palette, f"{title_label} -- traiettorie sovrapposte")
    _style_axes(ax_speed, palette, "velocita' (m/s)")
    _style_axes(ax_steer, palette, "sterzo (rad)")
    _style_axes(ax_cs, palette, "clearance nel tempo")

    cmap = plt.get_cmap("tab10")
    colors = [cmap(i % 10) for i in range(len(run_objs))]

    for run, color, m in zip(run_objs, colors, metrics_list):
        window = run.drive_window
        _draw_map_panel(ax_map, run, window, palette, {}, label=run.test_id, color=color)

        mask = _window_mask(run.ti, window) & np.isfinite(run.speed)
        if mask.any():
            order_t = np.argsort(run.ti[mask], kind="stable")
            ax_speed.plot(run.ti[mask][order_t], run.speed[mask][order_t],
                           color=color, linewidth=1.0, alpha=0.85, label=run.test_id)

        cmd = run.commands
        mask = _window_mask(cmd.get("t", np.empty(0)), window) & np.isfinite(cmd.get("cmd_steer", np.empty(0)))
        if mask.any():
            t, s = cmd["t"][mask], cmd["cmd_steer"][mask]
            order_t = np.argsort(t, kind="stable")
            ax_steer.plot(t[order_t], s[order_t], color=color, linewidth=1.0, alpha=0.85,
                           label=run.test_id)

        if run.clearance_source == "live":
            mask = _window_mask(run.ti, window) & np.isfinite(run.clearance)
            if mask.any():
                order_t = np.argsort(run.ti[mask], kind="stable")
                t_c, c_c = run.ti[mask][order_t], run.clearance[mask][order_t]
                ax_cs.plot(t_c, c_c, color=color, linewidth=1.0, alpha=0.85, label=run.test_id)
                # RULES.md §1.6: reuse m's (compute_run_metrics) own clearance
                # minimum instead of a local argmin, when that metric is
                # available -- argmin stays a fallback only.
                t_min = m.get("t_min_clear_s") if m else None
                c_min = m.get("min_clear_any_m") if m else None
                if t_min is None or c_min is None:
                    i = int(np.argmin(c_c))
                    t_min, c_min = t_c[i], c_c[i]
                ax_cs.scatter([t_min], [c_min], marker="v", s=30, color=color, zorder=4)

    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.set_xlabel("x odom (m)")
    ax_map.set_ylabel("y odom (m)")
    ax_speed.set_xlabel("t (s)")
    ax_speed.set_ylabel("speed (m/s)")
    ax_steer.set_xlabel("t (s)")
    ax_steer.set_ylabel("cmd_steer (rad)")
    ax_cs.set_xlabel("t (s)")
    ax_cs.set_ylabel("clearance (m)")

    for ax in (ax_map, ax_speed, ax_steer, ax_cs):
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            by_label = dict(zip(labels, handles))
            ax.legend(by_label.values(), by_label.keys(), fontsize=7,
                      facecolor=palette["PANEL"], labelcolor=palette["INK_SECONDARY"])

    fig.suptitle(f"{title_label}  (n={len(run_objs)} run selezionate)",
                 color=palette["INK_SECONDARY"], fontsize=9)
    return _finalize_figure(fig, palette, out_path)


# --------------------------------------------------------------------------
# CLI: generate the whole png_singole/ + png_selezionate/ tree from selection.yaml
# --------------------------------------------------------------------------

def _main(argv=None):
    import argparse

    try:
        from .run_data import load_selection, resolve_run
    except ImportError:
        from run_data import load_selection, resolve_run  # type: ignore[no-redef]

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--selection", default=None,
                         help="path to selection.yaml (default: analysis_v2/selection.yaml)")
    parser.add_argument("--campaign-dir", default=None,
                         help="root containing the Mxx_.../<test_id> folders")
    args = parser.parse_args(argv)

    root = find_root()
    analysis_dir = root / "first_test_campaign" / "analysis_v2"
    selection_path = Path(args.selection) if args.selection else analysis_dir / "selection.yaml"
    campaign_dir = Path(args.campaign_dir) if args.campaign_dir else root / "first_test_campaign"
    selection = load_selection(selection_path)
    out_root = analysis_dir / selection.output_folder

    singole_dir = out_root / "png_singole"
    selezionate_dir = out_root / "png_selezionate"

    groups: dict[str, list] = {}
    for row in selection:
        run_dir = resolve_run(row.run, [campaign_dir])
        run = load_run_v2(run_dir)
        if row.png:
            render_run_png(run, singole_dir / f"{run.test_id}.png", ostacolo=row.ostacolo)
        if row.selezionate:
            key = group_label(run.mission, row.ostacolo)
            groups.setdefault(key, []).append((run, row.ostacolo))

    for label, entries in groups.items():
        group_runs = [r for r, _ in entries]
        group_ostacolo = [o for _, o in entries]
        render_group_png(group_runs, selezionate_dir / f"{label}.png",
                          ostacolo=group_ostacolo, label=label)


if __name__ == "__main__":
    _main()
