"""corridor_plot: what gets drawn, and that missing data warns instead of crashing.

No pixel comparison. What is asserted is structural -- how many corridors
reached the axes, whether a centreline was drawn, whether the file exists --
which is what actually regresses when the loader or the schema changes.

matplotlib is forced to Agg by the module under test.
"""

from __future__ import annotations

import csv
import json

import pytest

from f1tenth_logger.test_campaign import corridor_plot
from f1tenth_logger.test_campaign.corridor_def import SCHEMA_V2

mpc = pytest.importorskip(
    "f1tenth_params.corridor_geometry",
    reason="f1tenth_params not on the path (built workspace not sourced)",
)
np = pytest.importorskip("numpy")


# --------------------------------------------------------------------------
# a synthetic test folder
# --------------------------------------------------------------------------

def v2_record(index, x0=0.0, y0=0.0, psi=0.0, dpsi=0.3, n=40):
    geom = mpc.corridor_curves(x0, y0, psi, psi + dpsi, 3.0, n, dpsi=dpsi)
    polygon = (
        [[float(a), float(b)] for a, b in zip(geom["xL"], geom["yL"])]
        + [[float(a), float(b)] for a, b in
           list(zip(geom["xR"], geom["yR"]))[::-1]]
    )
    return {
        "t": float(index), "id": index, "source": "mpc_corr",
        "polygon": polygon,
        "meta": {
            "frame_id": "odom", "length_m": 3.0,
            "definition": {
                "type": SCHEMA_V2, "C0": [x0, y0], "psiStart": psi,
                "psiEnd": psi + dpsi, "dpsi": dpsi, "psiRefTurn": dpsi,
                "L": 3.0, "corr_N": n, "u_start": 0.0, "u_end": 0.40,
                "w0": 0.4333, "w1": 0.7667,
                "handle_frac": mpc.CORRIDOR_HANDLE_FRAC,
                "Pend": [float(geom["Pend"][0]), float(geom["Pend"][1])],
            },
        },
    }


def v1_record(index, x0=0.0):
    return {
        "t": float(index), "id": index, "source": "mpc_corr",
        "polygon": [[x0, 0.45], [x0 + 3, 0.75], [x0 + 3, -0.75], [x0, -0.45]],
        "meta": {"frame_id": "odom", "length_m": 3.0, "psi_ref": 0.0},
    }


def horizon_record(index, corridor_id, t, n=5, ts=0.1, dx=0.2, offset=0.0):
    """A straight horizon walking forward in x, offset in y by ``offset``."""
    return {
        "t": t, "i": index, "corridor_id": corridor_id, "frame_id": "odom",
        "ts": ts, "n": n,
        "x": [t * 2.0 + (k + 1) * dx for k in range(n)],
        "y": [offset] * n,
        "yaw": [0.0] * n,
        "v": [0.45] * n,
        "steer": [0.0] * n,
        "accel": [0.0] * n,
    }


def make_test(tmp_path, *, corridors=None, poses=10, meta=True, horizons=None,
              name="P001-R001-20260101T000000", mission="M01"):
    directory = tmp_path / mission / name
    directory.mkdir(parents=True)

    if horizons is not None:
        with open(directory / "horizon.jsonl", "w", encoding="utf-8") as fh:
            for record in horizons:
                fh.write(json.dumps(record) + "\n")

    if corridors is not None:
        with open(directory / "corridors.jsonl", "w", encoding="utf-8") as fh:
            for record in corridors:
                fh.write(json.dumps(record) + "\n")

    if poses:
        with open(directory / "kinematics.csv", "w", newline="",
                  encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["t", "x", "y", "yaw", "corridor_clearance"])
            for i in range(poses):
                writer.writerow([i * 0.1, i * 0.2, 0.0, 0.0, 0.4])

    if meta:
        with open(directory / "meta.json", "w", encoding="utf-8") as fh:
            json.dump({"summary": {"test_id": name, "mission": mission,
                                   "auto_outcome": "completed"}}, fh)
    return directory


def count_drawn(directory, split=False):
    """(corridors drawn, centreline drawn) by inspecting the axes, not pixels."""
    import matplotlib.pyplot as plt

    from f1tenth_logger.test_campaign.corridor_def import load_corridors

    records = load_corridors(directory / "corridors.jsonl")
    fig, ax = plt.subplots()
    walls = centrelines = 0
    for record in records:
        before = len(ax.lines)
        drew_curves = corridor_plot.draw_corridor(ax, record, 1.0)
        added = len(ax.lines) - before
        if added:
            walls += 1
        if drew_curves:
            centrelines += 1
    plt.close(fig)
    return walls, centrelines


# --------------------------------------------------------------------------
# the file
# --------------------------------------------------------------------------

def test_a_figure_is_written_into_the_test_folder(tmp_path):
    directory = make_test(
        tmp_path,
        corridors=[v2_record(i, i * 0.5) for i in range(4)],
        horizons=[horizon_record(i, i, t=i * 0.2) for i in range(4)],
    )
    path, warnings = corridor_plot.plot_test(directory, quiet=True)
    assert path == directory / corridor_plot.PLOT_NAME
    assert path.exists() and path.stat().st_size > 0
    assert warnings == []


def test_dpi_is_configurable(tmp_path):
    directory = make_test(tmp_path, corridors=[v2_record(0, 0.0)])
    small, _ = corridor_plot.plot_test(directory, dpi=50, quiet=True)
    size_small = small.stat().st_size
    big, _ = corridor_plot.plot_test(directory, dpi=200, quiet=True)
    assert big.stat().st_size > size_small


def test_out_overrides_the_destination(tmp_path):
    directory = make_test(tmp_path, corridors=[v2_record(0, 0.0)])
    target = tmp_path / "elsewhere" / "fig.png"
    path, _ = corridor_plot.plot_test(directory, out=target, quiet=True)
    assert path == target and target.exists()
    assert not (directory / corridor_plot.PLOT_NAME).exists()


# --------------------------------------------------------------------------
# how many corridors are drawn
# --------------------------------------------------------------------------

@pytest.mark.parametrize("n", [1, 3, 9])
def test_every_corridor_of_the_test_is_drawn(tmp_path, n):
    directory = make_test(
        tmp_path, corridors=[v2_record(i, i * 0.4) for i in range(n)])
    walls, centrelines = count_drawn(directory)
    assert walls == n
    assert centrelines == n


def test_a_v1_corridor_is_drawn_without_a_centreline(tmp_path):
    """The polygon is all a v1 record has; inventing a centreline from it would
    be a different curve from the planner's."""
    directory = make_test(tmp_path, corridors=[v1_record(i) for i in range(3)])
    walls, centrelines = count_drawn(directory)
    assert walls == 3
    assert centrelines == 0

    path, _ = corridor_plot.plot_test(directory, quiet=True)
    assert path.exists()


def test_a_mixed_file_draws_both(tmp_path):
    directory = make_test(tmp_path, corridors=[v1_record(0), v2_record(1, 1.0)])
    walls, centrelines = count_drawn(directory)
    assert walls == 2
    assert centrelines == 1


def test_the_title_carries_id_mission_and_outcome(tmp_path):
    directory = make_test(tmp_path, corridors=[v2_record(0, 0.0)])
    from f1tenth_logger.test_campaign.corridor_def import load_corridors
    title = corridor_plot.title_for(
        directory, corridor_plot.read_meta(directory),
        load_corridors(directory / "corridors.jsonl"))
    assert "P001-R001-20260101T000000" in title
    assert "M01" in title
    assert "completed" in title


def test_the_title_says_when_the_log_is_v1(tmp_path):
    directory = make_test(tmp_path, corridors=[v1_record(0)])
    from f1tenth_logger.test_campaign.corridor_def import load_corridors
    title = corridor_plot.title_for(
        directory, corridor_plot.read_meta(directory),
        load_corridors(directory / "corridors.jsonl"))
    assert "v1" in title


# --------------------------------------------------------------------------
# partial data: a warning, never a crash
# --------------------------------------------------------------------------

def test_corridors_but_no_trajectory(tmp_path):
    directory = make_test(tmp_path, corridors=[v2_record(0, 0.0)], poses=0)
    path, warnings = corridor_plot.plot_test(directory, quiet=True)
    assert path.exists()
    assert any("trajectory not drawn" in w for w in warnings)


def test_a_trajectory_but_no_corridors(tmp_path):
    directory = make_test(tmp_path, corridors=None)
    path, warnings = corridor_plot.plot_test(directory, quiet=True)
    assert path.exists()
    assert any("corridors not drawn" in w for w in warnings)


def test_an_empty_corridors_file(tmp_path):
    directory = make_test(tmp_path, corridors=[])
    path, warnings = corridor_plot.plot_test(directory, quiet=True)
    assert path.exists()
    assert any("corridors not drawn" in w for w in warnings)


def test_neither_corridors_nor_poses_writes_nothing_and_says_so(tmp_path):
    directory = make_test(tmp_path, corridors=None, poses=0, meta=False)
    path, warnings = corridor_plot.plot_test(directory, quiet=True)
    assert path is None
    assert any("nothing to plot" in w for w in warnings)
    assert not (directory / corridor_plot.PLOT_NAME).exists()


def test_a_missing_folder_does_not_raise(tmp_path):
    path, warnings = corridor_plot.plot_test(tmp_path / "nope", quiet=True)
    assert path is None and warnings


def test_a_truncated_corridor_line_is_skipped_not_fatal(tmp_path):
    directory = make_test(tmp_path, corridors=[v2_record(0, 0.0)])
    with open(directory / "corridors.jsonl", "a", encoding="utf-8") as fh:
        fh.write('{"t": 1.0, "id": 1, "poly')
    path, _ = corridor_plot.plot_test(directory, quiet=True)
    assert path.exists()
    walls, _ = count_drawn(directory)
    assert walls == 1


def test_a_degenerate_polygon_is_skipped_with_a_warning(tmp_path):
    bad = {"t": 0.0, "id": 0, "polygon": [[0.0, 0.0], [1.0, 0.0]], "meta": {}}
    directory = make_test(tmp_path, corridors=[bad, v1_record(1)])
    path, warnings = corridor_plot.plot_test(directory, quiet=True)
    assert path.exists()
    assert any("vertices, skipped" in w for w in warnings)


def test_missing_meta_falls_back_to_the_folder_name(tmp_path):
    directory = make_test(tmp_path, corridors=[v2_record(0, 0.0)], meta=False)
    path, warnings = corridor_plot.plot_test(directory, quiet=True)
    assert path.exists()
    assert any("no meta.json" in w for w in warnings)


# --------------------------------------------------------------------------
# --split
# --------------------------------------------------------------------------

@pytest.mark.parametrize("n", [1, 2, 5])
def test_split_makes_one_panel_per_corridor(tmp_path, n):
    directory = make_test(
        tmp_path, corridors=[v2_record(i, i * 0.4) for i in range(n)])
    path, _ = corridor_plot.plot_test(directory, split=True, quiet=True)
    assert path.exists() and path.stat().st_size > 0


def test_split_without_corridors_falls_back_to_one_panel(tmp_path):
    directory = make_test(tmp_path, corridors=None)
    path, warnings = corridor_plot.plot_test(directory, split=True, quiet=True)
    assert path.exists()
    assert any("--split needs corridors" in w for w in warnings)


# --------------------------------------------------------------------------
# batch
# --------------------------------------------------------------------------

def test_mission_mode_plots_every_test_in_it(tmp_path):
    for rep in (1, 2, 3):
        make_test(tmp_path, corridors=[v2_record(0, 0.0)],
                  name=f"P001-R00{rep}-2026010{rep}T000000")
    tests = corridor_plot.find_tests(tmp_path, mission=tmp_path / "M01")
    assert len(tests) == 3
    written, skipped = corridor_plot.plot_many(tests, quiet=True)
    assert len(written) == 3 and not skipped
    for directory in tests:
        assert (directory / corridor_plot.PLOT_NAME).exists()


def test_campaign_mode_spans_missions(tmp_path):
    make_test(tmp_path, corridors=[v2_record(0, 0.0)], mission="M01")
    make_test(tmp_path, corridors=[v2_record(0, 0.0)], mission="M02")
    tests = corridor_plot.find_tests(tmp_path, campaign=tmp_path)
    assert len(tests) == 2
    written, _ = corridor_plot.plot_many(tests, quiet=True)
    assert len(written) == 2


def test_a_test_with_nothing_is_counted_as_skipped_not_fatal(tmp_path):
    good = make_test(tmp_path, corridors=[v2_record(0, 0.0)])
    empty = make_test(tmp_path, corridors=None, poses=0, meta=False,
                      name="P001-R002-20260102T000000")
    written, skipped = corridor_plot.plot_many([good, empty], quiet=True)
    assert written == [good] and skipped == [empty]


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def test_cli_on_one_test(tmp_path, capsys):
    directory = make_test(tmp_path, corridors=[v2_record(0, 0.0)])
    assert corridor_plot.main([str(directory)]) == 0
    assert (directory / corridor_plot.PLOT_NAME).exists()
    assert corridor_plot.PLOT_NAME in capsys.readouterr().out


def test_cli_on_a_campaign(tmp_path, capsys):
    make_test(tmp_path, corridors=[v2_record(0, 0.0)], mission="M01")
    make_test(tmp_path, corridors=[v2_record(0, 0.0)], mission="M02")
    assert corridor_plot.main(["--campaign", str(tmp_path)]) == 0
    assert capsys.readouterr().out.count(corridor_plot.PLOT_NAME) == 2


def test_cli_with_no_target_is_an_error(capsys):
    assert corridor_plot.main([]) == 2


def test_cli_on_an_empty_test_reports_failure(tmp_path):
    directory = make_test(tmp_path, corridors=None, poses=0, meta=False)
    assert corridor_plot.main([str(directory)]) == 1


# --------------------------------------------------------------------------
# the predicted horizons
# --------------------------------------------------------------------------

def _horizon_lines(ax):
    """Axes lines drawn in the horizon colour, whatever else is on the plot."""
    return [line for line in ax.lines
            if line.get_color() == corridor_plot.HORIZON_COLOR]


def test_one_horizon_per_corridor_is_the_default(tmp_path):
    """Four corridors, twenty solves -> four horizons, not twenty.

    The default exists because a horizon is logged every solve: drawing them
    all is a solid orange field. The selection is by corridor_id, so this also
    pins that the plot uses the stamped id rather than matching on time.
    """
    horizons = [horizon_record(i, corridor_id=i // 5, t=i * 0.05)
                for i in range(20)]
    directory = make_test(tmp_path,
                          corridors=[v2_record(i, i * 0.5) for i in range(4)],
                          horizons=horizons)
    import matplotlib.pyplot as plt

    from f1tenth_logger.test_campaign.horizon_log import (
        load_horizons, select_by_corridor)

    picked = select_by_corridor(load_horizons(directory / "horizon.jsonl"))
    assert len(picked) == 4
    # the FIRST solve under each corridor, not the last
    assert [r.i for r in picked] == [0, 5, 10, 15]

    fig, ax = plt.subplots()
    for record in picked:
        corridor_plot.draw_horizon(ax, record, 1.0)
    assert len(_horizon_lines(ax)) == 8       # one path + one end dot each
    plt.close(fig)


def test_horizon_every_n_overrides_the_per_corridor_default(tmp_path):
    from f1tenth_logger.test_campaign.horizon_log import (
        load_horizons, select_every)

    horizons = [horizon_record(i, corridor_id=i // 5, t=i * 0.05)
                for i in range(20)]
    directory = make_test(tmp_path,
                          corridors=[v2_record(i) for i in range(4)],
                          horizons=horizons)
    records = load_horizons(directory / "horizon.jsonl")
    assert [r.i for r in select_every(records, 5)] == [0, 5, 10, 15]
    assert len(select_every(records, 1)) == 20
    assert select_every(records, 0) == []

    path, _ = corridor_plot.plot_test(directory, quiet=True, horizon_every=5)
    assert path.exists()


def test_no_horizons_flag_draws_none(tmp_path):
    directory = make_test(
        tmp_path, corridors=[v2_record(0)],
        horizons=[horizon_record(0, 0, t=0.1)])
    path, warnings = corridor_plot.plot_test(directory, quiet=True,
                                             horizons=False)
    assert path.exists()
    # the horizon file is never read, so its absence is never reported either
    assert not any("horizon" in w for w in warnings)


def test_a_test_without_horizons_still_plots_and_says_so(tmp_path):
    """Every run logged before horizon.jsonl existed must still draw."""
    directory = make_test(tmp_path, corridors=[v2_record(0)])
    path, warnings = corridor_plot.plot_test(directory, quiet=True)
    assert path.exists()
    assert any("no horizon.jsonl" in w for w in warnings)


def test_the_horizon_is_anchored_on_the_driven_pose(tmp_path):
    """The drawn path starts at the car, not one control period ahead.

    x_pred is x_1..x_N with x0 deliberately absent, so an unanchored horizon
    floats detached from the trajectory and the plan/driven comparison the
    figure exists for is lost.
    """
    import matplotlib.pyplot as plt

    from f1tenth_logger.test_campaign.horizon_log import load_horizons

    # kinematics: x = 0.2 * i at t = 0.1 * i, so at t = 0.25 the car is at 0.5
    directory = make_test(tmp_path, corridors=[v2_record(0)],
                          horizons=[horizon_record(0, 0, t=0.25)])
    t_traj, xs, ys = corridor_plot.read_trajectory_timed(directory)
    record = load_horizons(directory / "horizon.jsonl")[0]

    fig, ax = plt.subplots()
    corridor_plot.draw_horizon(ax, record, 1.0, t_traj, xs, ys)
    drawn = _horizon_lines(ax)[0].get_xydata()
    plt.close(fig)

    assert len(drawn) == len(record.x) + 1
    assert drawn[0][0] == pytest.approx(0.5)      # interpolated, not clamped
    assert drawn[1][0] == pytest.approx(record.x[0])


def test_an_unanchorable_horizon_is_drawn_from_its_first_step(tmp_path):
    """A solve outside the logged trajectory must not invent a start pose."""
    import matplotlib.pyplot as plt

    from f1tenth_logger.test_campaign.horizon_log import load_horizons

    directory = make_test(tmp_path, corridors=[v2_record(0)],
                          horizons=[horizon_record(0, 0, t=99.0)])
    t_traj, xs, ys = corridor_plot.read_trajectory_timed(directory)
    record = load_horizons(directory / "horizon.jsonl")[0]

    fig, ax = plt.subplots()
    corridor_plot.draw_horizon(ax, record, 1.0, t_traj, xs, ys)
    drawn = _horizon_lines(ax)[0].get_xydata()
    plt.close(fig)
    assert len(drawn) == len(record.x)
    assert drawn[0][0] == pytest.approx(record.x[0])


def test_split_panels_get_only_their_own_corridors_horizon(tmp_path):
    """The split figure is one corridor per panel; horizons must follow."""
    directory = make_test(
        tmp_path,
        corridors=[v2_record(i, i * 0.5) for i in range(3)],
        horizons=[horizon_record(i, corridor_id=i, t=i * 0.2) for i in range(3)],
    )
    path, _ = corridor_plot.plot_test(directory, split=True, quiet=True)
    assert path.exists() and path.stat().st_size > 0
