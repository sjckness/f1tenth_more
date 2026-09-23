"""corridor_plot.py -- one figure per test: every corridor, and what was driven.

Reads a finished test folder and nothing else. No ROS, no replanning, no live
anything: corridors.jsonl for the geometry, kinematics.csv for the trajectory,
meta.json for the title.

WHAT IT DRAWS, and where the styling comes from. The palette is plot_corridor.
py's draw_snapshot(), the standalone viewer for mpc_corr's own debug snapshots,
so a figure from a log and a figure from the live planner read the same way:

    tab:blue    the two wall Beziers, and a cyan fill between them
    teal, dashed the centreline
    tab:gray x  Pend, the corridor's end point
    black       the trajectory actually driven, on top of everything

with two additions this file needs and a single snapshot did not: every
corridor of the test rather than one, faded by age so the newest is the most
solid, and start/end markers on the trajectory.

HOW THE CURVES ARE OBTAINED. For a v2 record they are EVALUATED from the
stored function definition, by corridor_geometry.corridor_curves -- the
planner's own function, at the planner's own corr_N. Not interpolated from the
logged samples, and deliberately not evaluated at a higher resolution: the
centreline is a cumsum Riemann sum, so a denser evaluation is a different
curve from the one the car drove (see corridor_def).

For a v1 record there is no definition, so the sampled boundary polygon is all
there is. It is drawn as a filled band with no centreline, and the figure says
so. Every run in first_test_campaing/ is v1.

    test_campaign_corridor_plot <test folder>
    test_campaign_corridor_plot --mission first_test_campaing/M02_approach_wall
    test_campaign_corridor_plot --campaign first_test_campaing --dpi 200
    test_campaign_corridor_plot <test folder> --split --show
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib

# Agg unless a window was asked for: this runs headless from analyze_tests and
# over ssh far more often than it runs interactively. --show re-selects a GUI
# backend before pyplot is imported, in main().
if "matplotlib.pyplot" not in sys.modules:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.cm import ScalarMappable  # noqa: E402
from matplotlib.colors import Normalize  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from f1tenth_logger.test_campaign.corridor_def import (  # noqa: E402
    SCHEMA_V2, evaluate, load_corridors,
)

__all__ = ["plot_test", "plot_many", "find_tests", "PLOT_NAME"]

PLOT_NAME = "corridor_plot.png"

WALL_COLOR = "tab:blue"
FILL_COLOR = "tab:cyan"
CENTER_COLOR = "teal"
PEND_COLOR = "tab:gray"
TRACK_COLOR = "black"
START_COLOR = "tab:green"
END_COLOR = "tab:red"

#: Oldest corridor's alpha; the newest is always 1.0.
ALPHA_MIN = 0.22

#: Walls are coloured by rebuild time along this colormap rather than held at a
#: flat tab:blue, so the colorbar beside the figure says something TRUE about
#: the lines: light is early, dark is late. Blues' dark end is essentially
#: draw_snapshot's tab:blue, so the figure still reads as that viewer's. An
#: alpha ramp under a flat colour would look the same and mean nothing the
#: colorbar could label.
WALL_CMAP = "Blues"
#: Keep the earliest corridor off the white end, where it would vanish.
WALL_CMAP_FLOOR = 0.35


def _warn(messages, text):
    messages.append(text)


def read_trajectory(test_dir):
    """(xs, ys) actually driven, or ([], []) when there is no usable pose data."""
    path = Path(test_dir) / "kinematics.csv"
    xs, ys = [], []
    try:
        handle = open(path, newline="", encoding="utf-8")
    except OSError:
        return xs, ys
    with handle:
        for row in csv.DictReader(handle):
            try:
                x = float(row["x"])
                y = float(row["y"])
            except (TypeError, ValueError, KeyError):
                continue
            if x == x and y == y:      # not NaN
                xs.append(x)
                ys.append(y)
    return xs, ys


def read_meta(test_dir):
    path = Path(test_dir) / "meta.json"
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, json.JSONDecodeError):
        return {}


def title_for(test_dir, meta, records):
    """test id, mission and outcome, plus the schema when it limits the figure."""
    test_dir = Path(test_dir)
    summary = meta.get("summary", meta)
    test_id = summary.get("test_id") or test_dir.name
    mission = summary.get("mission") or test_dir.parent.name
    outcome = summary.get("auto_outcome") or "unknown"
    line = f"{test_id}   {mission}   {outcome}"
    n_v2 = sum(1 for r in records if r.schema == SCHEMA_V2)
    if records and n_v2 == 0:
        line += "\nv1 log: sampled boundary only, no centreline"
    elif records and n_v2 < len(records):
        line += f"\n{n_v2}/{len(records)} corridors carry a v2 definition"
    return line


def wall_color(frac):
    """Wall colour for a corridor ``frac`` of the way through the test."""
    return plt.get_cmap(WALL_CMAP)(
        WALL_CMAP_FLOOR + (1.0 - WALL_CMAP_FLOOR) * frac)


def draw_corridor(ax, record, alpha, label=None, messages=None, color=None):
    """One corridor: evaluated curves for v2, the sampled polygon for v1."""
    color = WALL_COLOR if color is None else color
    curves = evaluate(record) if record.schema == SCHEMA_V2 else None

    if curves is not None:
        xl, yl = curves["xL"], curves["yL"]
        xr, yr = curves["xR"], curves["yR"]
        ax.fill(
            list(xl) + list(xr)[::-1], list(yl) + list(yr)[::-1],
            color=FILL_COLOR, alpha=0.15 * alpha, zorder=1, linewidth=0,
        )
        ax.plot(xl, yl, "-", color=color, linewidth=1.1, alpha=alpha,
                zorder=2, label=label)
        ax.plot(xr, yr, "-", color=color, linewidth=1.1, alpha=alpha,
                zorder=2)
        ax.plot(curves["xc"], curves["yc"], "--", color=CENTER_COLOR,
                linewidth=1.6, alpha=alpha, zorder=2)
        pend = curves["Pend"]
        ax.plot(pend[0], pend[1], "x", color=PEND_COLOR, markersize=7,
                markeredgewidth=1.4, alpha=alpha, zorder=3)
        return True

    poly = record.polygon
    if len(poly) < 3:
        if messages is not None:
            _warn(messages, f"corridor {record.id} has {len(poly)} vertices, skipped")
        return False
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    ax.fill(xs, ys, color=FILL_COLOR, alpha=0.15 * alpha, zorder=1, linewidth=0)
    ax.plot(xs + xs[:1], ys + ys[:1], "-", color=color, linewidth=1.1,
            alpha=alpha, zorder=2, label=label)
    return False


def _alphas(n):
    if n <= 1:
        return [1.0]
    return [ALPHA_MIN + (1.0 - ALPHA_MIN) * i / (n - 1) for i in range(n)]


def _draw_trajectory(ax, xs, ys):
    ax.plot(xs, ys, "-", color=TRACK_COLOR, linewidth=1.8, zorder=4,
            label="driven")
    ax.plot(xs[0], ys[0], "o", color=START_COLOR, markersize=9,
            markeredgecolor="black", markeredgewidth=0.8, zorder=6,
            label="start")
    ax.plot(xs[-1], ys[-1], "X", color=END_COLOR, markersize=11,
            markeredgecolor="black", markeredgewidth=0.8, zorder=6,
            label="end")


def _finish_axes(ax, grid=True):
    ax.set_aspect("equal", adjustable="datalim")
    if grid:
        ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def _legend_handles(any_centreline, has_track):
    handles = [
        Line2D([], [], color=wall_color(1.0), lw=1.4,
               label="corridor walls (light = early)"),
    ]
    if any_centreline:
        handles.append(
            Line2D([], [], color=CENTER_COLOR, lw=1.6, ls="--", label="centreline"))
        handles.append(
            Line2D([], [], color=PEND_COLOR, marker="x", ls="none",
                   label="corridor end (Pend)"))
    if has_track:
        handles += [
            Line2D([], [], color=TRACK_COLOR, lw=1.8, label="driven"),
            Line2D([], [], color=START_COLOR, marker="o", ls="none",
                   markeredgecolor="black", label="start"),
            Line2D([], [], color=END_COLOR, marker="X", ls="none",
                   markeredgecolor="black", label="end"),
        ]
    return handles


def _figure_single(records, xs, ys, title, messages):
    fig, ax = plt.subplots(figsize=(9, 8))
    any_centreline = False
    n = len(records)
    alphas = _alphas(n)
    for index, (record, alpha) in enumerate(zip(records, alphas)):
        frac = 0.0 if n <= 1 else index / (n - 1)
        any_centreline |= draw_corridor(ax, record, alpha, messages=messages,
                                        color=wall_color(frac))

    if xs:
        _draw_trajectory(ax, xs, ys)

    times = [r.t for r in records if isinstance(r.t, (int, float))]
    if len(times) > 1 and min(times) < max(times):
        # The wall colours ARE this colormap, sampled by rebuild order, so the
        # bar labels the lines rather than decorating them.
        mappable = ScalarMappable(
            norm=Normalize(min(times), max(times)),
            cmap=matplotlib.colors.LinearSegmentedColormap.from_list(
                "walls", [wall_color(0.0), wall_color(1.0)]))
        mappable.set_array([])
        bar = fig.colorbar(mappable, ax=ax, fraction=0.035, pad=0.02)
        bar.set_label("corridor rebuild time [s]")

    _finish_axes(ax)
    ax.set_title(title)
    handles = _legend_handles(any_centreline, bool(xs))
    ax.legend(handles=handles, loc="upper left", fontsize=8, framealpha=0.9)
    return fig


def _figure_split(records, xs, ys, title, messages):
    n = len(records)
    cols = min(4, n)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(3.2 * cols, 3.0 * rows),
                             squeeze=False)
    any_centreline = False
    for index, record in enumerate(records):
        ax = axes[index // cols][index % cols]
        frac = 0.0 if n <= 1 else index / (n - 1)
        any_centreline |= draw_corridor(ax, record, 1.0, messages=messages,
                                        color=wall_color(frac))
        if xs:
            _draw_trajectory(ax, xs, ys)
        _finish_axes(ax)
        stamp = "" if not isinstance(record.t, (int, float)) else f"  t={record.t:.1f}s"
        ax.set_title(f"#{record.id}{stamp}", fontsize=9)
        ax.tick_params(labelsize=7)
        ax.set_xlabel("")
        ax.set_ylabel("")
    for index in range(n, rows * cols):
        axes[index // cols][index % cols].axis("off")
    fig.suptitle(title)
    handles = _legend_handles(any_centreline, bool(xs))
    # Below the grid, not inside it: at 4 columns the last panel is exactly
    # where a corner legend lands.
    fig.legend(handles=handles, loc="lower center", ncol=len(handles),
               fontsize=8, framealpha=0.9, bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.06, 1, 0.96))
    return fig


def plot_test(test_dir, out=None, dpi=150, show=False, split=False, quiet=False):
    """Write ``corridor_plot.png`` for one test folder.

    Returns ``(path_or_None, warnings)``. A test with neither corridors nor a
    trajectory produces no file and one warning -- there is nothing to draw,
    which is a result, not a crash.
    """
    test_dir = Path(test_dir)
    messages = []

    records = load_corridors(test_dir / "corridors.jsonl")
    xs, ys = read_trajectory(test_dir)
    meta = read_meta(test_dir)

    if not records:
        _warn(messages, "no corridors.jsonl (or it is empty): corridors not drawn")
    if not xs:
        _warn(messages, "no usable poses in kinematics.csv: trajectory not drawn")
    if not meta:
        _warn(messages, "no meta.json: the title falls back to the folder name")

    if not records and not xs:
        _warn(messages, "nothing to plot")
        if not quiet:
            for text in messages:
                print(f"  [{test_dir.name}] {text}", file=sys.stderr)
        return None, messages

    title = title_for(test_dir, meta, records)
    if split and records:
        fig = _figure_split(records, xs, ys, title, messages)
    else:
        if split:
            _warn(messages, "--split needs corridors; drawing a single panel")
        fig = _figure_single(records, xs, ys, title, messages)

    target = Path(out) if out else test_dir / PLOT_NAME
    target.parent.mkdir(parents=True, exist_ok=True)
    if not (split and records):      # the split figure lays itself out
        fig.tight_layout()
    fig.savefig(target, dpi=dpi)

    if show:
        plt.show()
    plt.close(fig)

    if not quiet:
        for text in messages:
            print(f"  [{test_dir.name}] {text}", file=sys.stderr)
    return target, messages


TEST_DIR_GLOB = "P[0-9][0-9][0-9]-R[0-9][0-9][0-9]-*"


def find_tests(root, mission=None, campaign=None):
    """Test folders under a campaign, a mission, or a single test folder."""
    root = Path(root)
    if campaign:
        return sorted(p for p in Path(campaign).glob(f"*/{TEST_DIR_GLOB}")
                      if p.is_dir())
    if mission:
        return sorted(p for p in Path(mission).glob(TEST_DIR_GLOB) if p.is_dir())
    return [root]


def plot_many(test_dirs, dpi=150, split=False, show=False, quiet=False):
    """``(written, skipped)`` over several test folders."""
    written, skipped = [], []
    for test_dir in test_dirs:
        path, _ = plot_test(test_dir, dpi=dpi, split=split, show=show, quiet=quiet)
        (written if path else skipped).append(test_dir)
    return written, skipped


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Redraw a test's corridors and trajectory from its logs.")
    parser.add_argument("test_dir", nargs="?", default=None,
                        help="one test folder")
    parser.add_argument("--mission", default=None,
                        help="a mission folder: one figure per test in it")
    parser.add_argument("--campaign", default=None,
                        help="a campaign folder: one figure per test in it")
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("--split", action="store_true",
                        help="one small subplot per corridor, in time order")
    parser.add_argument("--show", action="store_true",
                        help="also open the figure in a window")
    parser.add_argument("--out", default=None,
                        help="write here instead of <test>/corridor_plot.png "
                             "(single test only)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not (args.test_dir or args.mission or args.campaign):
        print("give a test folder, --mission or --campaign", file=sys.stderr)
        return 2

    if args.show:
        # pyplot is already imported at module level under Agg; switching now
        # is the documented way round and is why --show is handled here.
        try:
            matplotlib.use("TkAgg", force=True)
        except Exception as exc:            # noqa: BLE001 - headless is normal
            print(f"--show unavailable ({exc}); writing the file only",
                  file=sys.stderr)
            args.show = False

    if args.test_dir and not (args.mission or args.campaign):
        path, _ = plot_test(args.test_dir, out=args.out, dpi=args.dpi,
                            show=args.show, split=args.split)
        if path is None:
            return 1
        print(path)
        return 0

    tests = find_tests(args.test_dir or ".", mission=args.mission,
                       campaign=args.campaign)
    if not tests:
        print("no test folders found", file=sys.stderr)
        return 1
    written, skipped = plot_many(tests, dpi=args.dpi, split=args.split)
    for path in written:
        print(Path(path) / PLOT_NAME)
    print(f"{len(written)} figure(s) written, {len(skipped)} skipped",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
