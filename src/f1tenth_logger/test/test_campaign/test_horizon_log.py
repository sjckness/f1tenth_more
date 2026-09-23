"""horizon_log: reading horizon.jsonl, picking which to draw, and the error.

The prediction error is the reason this module exists as shared code -- the
figure and the campaign CSV both report it -- so most of what is asserted here
is that number: what it measures, what it refuses to measure, and that it
returns "unmeasured" rather than zero when it cannot.
"""

from __future__ import annotations

import json

import pytest

from f1tenth_logger.test_campaign.horizon_log import (
    HorizonRecord, load_horizons, prediction_error, select_by_corridor,
    select_every,
)


def record(t=0.0, ts=0.1, n=5, i=0, corridor_id=0, x=None, y=None,
           frame_id="odom"):
    return {
        "t": t, "i": i, "corridor_id": corridor_id, "frame_id": frame_id,
        "ts": ts, "n": n,
        "x": list(x) if x is not None else [t + (k + 1) * ts for k in range(n)],
        "y": list(y) if y is not None else [0.0] * n,
        "yaw": [0.0] * n, "v": [1.0] * n,
        "steer": [0.0] * n, "accel": [0.0] * n,
    }


def write(tmp_path, records, name="horizon.jsonl"):
    path = tmp_path / name
    with open(path, "w", encoding="utf-8") as fh:
        for raw in records:
            fh.write(json.dumps(raw) + "\n")
    return path


#: t = 0.0 .. 1.0 at 0.1, with x == t and y == 0. Chosen so the "actual"
#: position at any time is that time, which makes every expected error below
#: readable without arithmetic.
def straight_drive(n=11, dt=0.1):
    times = [k * dt for k in range(n)]
    return times, list(times), [0.0] * n


# --------------------------------------------------------------------------
# reading
# --------------------------------------------------------------------------

def test_a_missing_file_is_an_empty_list_not_an_error(tmp_path):
    assert load_horizons(tmp_path / "nope.jsonl") == []


def test_a_truncated_last_line_is_skipped(tmp_path):
    """The file is appended to by a live node, so a half-line is expected."""
    path = write(tmp_path, [record(t=0.0), record(t=0.1)])
    with open(path, "a", encoding="utf-8") as fh:
        fh.write('{"t": 0.2, "x": [1.0], "y":')
    assert len(load_horizons(path)) == 2


def test_a_step_is_one_control_period_ahead_of_the_solve():
    """x_pred is x_1..x_N: the first step is never AT the solve time."""
    rec = HorizonRecord(record(t=2.0, ts=0.1, n=3))
    assert rec.step_times() == pytest.approx([2.1, 2.2, 2.3])


def test_a_record_without_ts_cannot_be_placed_in_time():
    raw = record(t=1.0)
    raw["ts"] = None
    rec = HorizonRecord(raw)
    assert len(rec) == 5          # still drawable
    assert not rec.timed          # but not measurable
    assert rec.step_times() == []


def test_a_nan_anywhere_drops_the_whole_array():
    """A horizon with a hole would shift every later step onto the wrong time."""
    raw = record(n=4)
    raw["x"] = [0.1, float("nan"), 0.3, 0.4]
    rec = HorizonRecord(raw)
    assert rec.x == []
    assert len(rec) == 0


def test_a_record_with_no_usable_points_is_not_loaded(tmp_path):
    raw = record(n=3)
    raw["x"] = []
    path = write(tmp_path, [raw, record(t=0.5)])
    assert [r.t for r in load_horizons(path)] == [0.5]


# --------------------------------------------------------------------------
# selection
# --------------------------------------------------------------------------

def test_by_corridor_takes_the_first_solve_under_each():
    """The plan the FRESH corridor produced, before the reference moved."""
    records = [HorizonRecord(record(t=i * 0.1, i=i, corridor_id=i // 4))
               for i in range(12)]
    picked = select_by_corridor(records)
    assert [r.corridor_id for r in picked] == [0, 1, 2]
    assert [r.i for r in picked] == [0, 4, 8]


def test_records_logged_before_any_corridor_group_under_none():
    records = [HorizonRecord(record(i=0, corridor_id=None)),
               HorizonRecord(record(i=1, corridor_id=None)),
               HorizonRecord(record(i=2, corridor_id=7))]
    assert [r.i for r in select_by_corridor(records)] == [0, 2]


def test_select_every_starts_at_the_first_and_refuses_zero():
    records = [HorizonRecord(record(i=i)) for i in range(10)]
    assert [r.i for r in select_every(records, 3)] == [0, 3, 6, 9]
    assert select_every(records, 0) == []
    assert select_every(records, None) == []


# --------------------------------------------------------------------------
# the prediction error
# --------------------------------------------------------------------------

def test_a_perfectly_tracked_horizon_has_no_error():
    times, xs, ys = straight_drive()
    records = [HorizonRecord(record(t=0.0, ts=0.1, n=5))]
    mean, worst, count = prediction_error(records, times, xs, ys)
    assert count == 5
    assert mean == pytest.approx(0.0, abs=1e-9)
    assert worst == pytest.approx(0.0, abs=1e-9)


def test_the_error_is_the_distance_to_where_the_car_actually_was():
    """A horizon offset sideways by 0.25 m reports exactly 0.25 m."""
    times, xs, ys = straight_drive()
    records = [HorizonRecord(record(t=0.0, ts=0.1, n=5, y=[0.25] * 5))]
    mean, worst, count = prediction_error(records, times, xs, ys)
    assert count == 5
    assert mean == pytest.approx(0.25)
    assert worst == pytest.approx(0.25)


def test_the_worst_step_is_reported_separately_from_the_mean():
    times, xs, ys = straight_drive()
    records = [HorizonRecord(
        record(t=0.0, ts=0.1, n=4, y=[0.0, 0.0, 0.0, 0.8]))]
    mean, worst, count = prediction_error(records, times, xs, ys)
    assert count == 4
    assert worst == pytest.approx(0.8)
    assert mean == pytest.approx(0.2)


def test_steps_past_the_end_of_the_log_are_dropped_not_extrapolated():
    """The last horizon of every test predicts past the recording.

    Counting that would charge the controller for the log stopping, and
    clamping to the final pose would invent a measurement.
    """
    times, xs, ys = straight_drive()             # spans 0.0 .. 1.0
    # steps at 0.85, 0.95, 1.05, 1.15, 1.25 -- only the first two are in span
    records = [HorizonRecord(record(t=0.75, ts=0.1, n=5))]
    _, _, count = prediction_error(records, times, xs, ys)
    assert count == 2


def test_solves_outside_the_drive_window_do_not_count():
    """Horizons predicted while stationary are trivially right."""
    times, xs, ys = straight_drive()
    records = [HorizonRecord(record(t=0.0, ts=0.1, n=3, i=0)),
               HorizonRecord(record(t=0.6, ts=0.1, n=3, i=1))]
    _, _, all_steps = prediction_error(records, times, xs, ys)
    _, _, windowed = prediction_error(records, times, xs, ys,
                                      window=(0.5, 1.0))
    assert all_steps == 6
    assert windowed == 3


def test_nothing_measurable_is_none_not_zero():
    """A caller must be able to tell "not measured" from "no error"."""
    times, xs, ys = straight_drive()
    assert prediction_error([], times, xs, ys) == (None, None, 0)
    assert prediction_error([HorizonRecord(record())], [], [], []) == (
        None, None, 0)

    untimed = record(t=0.0)
    untimed["ts"] = None
    assert prediction_error([HorizonRecord(untimed)], times, xs, ys) == (
        None, None, 0)


def test_the_error_interpolates_between_pose_samples():
    """A step landing between two samples is compared against the line."""
    # poses only at t = 0.0 and t = 1.0, x = 0 -> 10
    times, xs, ys = [0.0, 1.0], [0.0, 10.0], [0.0, 0.0]
    # one step at t = 0.5, predicted at x = 5.0 -> the interpolated truth
    raw = record(t=0.4, ts=0.1, n=1, x=[5.0], y=[0.0])
    mean, worst, count = prediction_error([HorizonRecord(raw)], times, xs, ys)
    assert count == 1
    assert mean == pytest.approx(0.0, abs=1e-9)


def test_unsorted_or_nan_poses_do_not_break_the_measure():
    """kinematics.csv carries NaN rows before the first finite difference."""
    times = [0.2, 0.0, 0.1, float("nan")]
    xs = [0.2, 0.0, 0.1, 5.0]
    ys = [0.0, 0.0, 0.0, 0.0]
    records = [HorizonRecord(record(t=0.0, ts=0.1, n=2))]
    mean, _, count = prediction_error(records, times, xs, ys)
    assert count == 2
    assert mean == pytest.approx(0.0, abs=1e-9)
