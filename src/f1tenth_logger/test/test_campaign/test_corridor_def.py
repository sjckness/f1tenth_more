"""corridors.jsonl: the two schemas, and the v2 round trip.

The load half is stdlib-only and always runs. The evaluate half needs
f1tenth_params on the path (same workspace, exec_depend) and numpy; it skips
rather than fails where they are absent, so this file is still useful on a
report machine with no ROS.
"""

from __future__ import annotations

import json

import pytest

from f1tenth_logger.test_campaign.corridor_def import (
    SCHEMA_V1,
    SCHEMA_V2,
    CorridorRecord,
    evaluate,
    load_corridors,
    schema_of,
)

mpc = pytest.importorskip(
    "f1tenth_params.corridor_geometry",
    reason="f1tenth_params not on the path (built workspace not sourced)",
)
np = pytest.importorskip("numpy")


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def build(x0=1.5, y0=-0.5, psi_start=0.2, dpsi=0.7, length=3.0, n=120,
          u_start=0.0, u_end=0.40, w0=0.4333, w1=0.7667, turn=False):
    """A corridor and the v2 record mpc_corr would log for it."""
    psi_end = psi_start + dpsi
    geom = mpc.corridor_curves(
        x0, y0, psi_start, psi_end, length, n,
        dpsi=(dpsi if turn else None),
        u_start=u_start, u_end=u_end, w0=w0, w1=w1,
    )
    record = {
        "t": 1.0,
        "id": 3,
        "source": "mpc_corr",
        "polygon": (
            [[float(x), float(y)] for x, y in zip(geom["xL"], geom["yL"])]
            + [[float(x), float(y)] for x, y in
               list(zip(geom["xR"], geom["yR"]))[::-1]]
        ),
        "meta": {
            "frame_id": "odom",
            "definition": {
                "type": SCHEMA_V2,
                "C0": [x0, y0],
                "psiStart": psi_start,
                "psiEnd": psi_end,
                "dpsi": float(geom["dpsi"]),
                "psiRefTurn": (dpsi if turn else None),
                "L": length,
                "corr_N": n,
                "u_start": u_start,
                "u_end": u_end,
                "w0": w0,
                "w1": w1,
                "handle_frac": mpc.CORRIDOR_HANDLE_FRAC,
                "ctrl_left": geom["ctrl_left"],
                "ctrl_right": geom["ctrl_right"],
                "Pend": [float(geom["Pend"][0]), float(geom["Pend"][1])],
            },
        },
    }
    return geom, record


def write_jsonl(path, records):
    with open(path, "w", encoding="utf-8") as fh:
        for r in records:
            fh.write(json.dumps(r) + "\n")
    return path


# --------------------------------------------------------------------------
# the round trip
# --------------------------------------------------------------------------

CURVES = ("xc", "yc", "xL", "yL", "xR", "yR", "tx", "ty", "nx", "ny",
          "halfWidth")


@pytest.mark.parametrize("turn", [False, True])
def test_a_logged_corridor_re_evaluates_bit_for_bit(tmp_path, turn):
    """THE GUARANTEE THE v2 SCHEMA EXISTS FOR.

    Not "to within a tolerance": the definition carries every argument at full
    precision and the evaluator is the planner's own function, so the arrays
    that come back are the identical floats. Any tolerance here would be
    hiding a divergence.
    """
    geom, record = build(turn=turn)
    loaded = load_corridors(write_jsonl(tmp_path / "corridors.jsonl", [record]))
    assert len(loaded) == 1 and loaded[0].schema == SCHEMA_V2

    again = evaluate(loaded[0])
    for key in CURVES:
        assert np.array_equal(again[key], geom[key]), f"{key} diverged"
    assert np.array_equal(again["Pend"], geom["Pend"])
    assert again["dpsi"] == geom["dpsi"]


def test_a_wall_turn_past_half_a_turn_survives_the_round_trip():
    """psi_ref alone cannot say which way round a 270 degree turn went, so the
    definition carries the signed, unwrapped rotation. Without it this corridor
    would come back bent the other way."""
    dpsi = np.deg2rad(270.0)
    geom, record = build(dpsi=dpsi, turn=True)
    from f1tenth_logger.test_campaign.corridor_def import CorridorRecord
    again = evaluate(CorridorRecord(record))
    assert again["dpsi"] == pytest.approx(dpsi)
    assert np.array_equal(again["xc"], geom["xc"])

    # and the shortest-branch wrap really would have gone the other way
    wrapped = mpc.wrap_pi(record["meta"]["definition"]["psiEnd"]
                          - record["meta"]["definition"]["psiStart"])
    assert wrapped < 0.0 < dpsi


def test_the_centreline_is_resolution_dependent_on_purpose():
    """The cumsum is a Riemann sum, so corr_N is part of the definition, not a
    rendering choice. Re-evaluating denser gives a DIFFERENT curve -- which is
    why evaluate() never resamples unless a test asks it to.

    HOW BIG THIS IS, because it is much bigger than a straight corridor
    suggests: on the 13 archived runs (max |dpsi| 1.81 deg) refining n=120 to
    n=4000 moves the corridor endpoint by 7.6e-5 m, which invites the
    conclusion that resolution does not matter. At the 0.7 rad ask used here
    it moves it by ~8.4e-3 m -- two orders larger, and comparable to the
    lateral errors the campaign is trying to measure. A plotter that "helpfully"
    evaluated at high resolution would draw a corridor the car never had.
    """
    geom, record = build(n=120)
    from f1tenth_logger.test_campaign.corridor_def import CorridorRecord
    rec = CorridorRecord(record)
    assert np.array_equal(evaluate(rec)["Pend"], geom["Pend"])

    def endpoint_error(n):
        ref = evaluate(rec, n=200000)["Pend"]
        return float(np.hypot(*(evaluate(rec, n=n)["Pend"] - ref)))

    coarse, fine = endpoint_error(120), endpoint_error(240)
    assert coarse > 1e-3, "the gap at the shipping n is not negligible"
    # first order: halving the step halves the error
    assert fine == pytest.approx(coarse / 2.0, rel=0.02)


# --------------------------------------------------------------------------
# v1 compatibility
# --------------------------------------------------------------------------

def test_a_bezier_centreline_round_trips_through_the_record():
    """The pose geometry's record must re-evaluate to the planner's curve.

    The two centrelines share one schema and differ only by a key, so this is
    the assertion that the key is read: evaluated as a heading ramp of length
    L the same record would draw a curve that never touches C1.
    """
    geom = mpc.corridor_curves_to_pose(
        0.0, 0.0, -0.1, 3.0, 0.8, 0.25, 120,
        w0=0.4333, w1=0.7667, length_ref=3.0)
    record = CorridorRecord({
        "t": 1.0, "id": 4, "source": "mpc_corr",
        "polygon": [[float(x), float(y)] for x, y in zip(geom["xL"], geom["yL"])],
        "meta": {"frame_id": "odom", "definition": {
            "type": SCHEMA_V2,
            "C0": [0.0, 0.0], "psiStart": -0.1, "psiEnd": 0.25,
            "dpsi": float(geom["dpsi"]), "psiRefTurn": None,
            "L": float(geom["length"]), "corr_N": 120,
            "u_start": 0.0, "u_end": 0.40, "w0": 0.4333, "w1": 0.7667,
            "handle_frac": mpc.CORRIDOR_HANDLE_FRAC,
            "centreline": "bezier",
            "C1": [3.0, 0.8],
            "handle_a": mpc.POSE_HANDLE_FRAC,
            "handle_b": mpc.POSE_HANDLE_FRAC,
        }},
    })
    again = evaluate(record)
    assert again is not None
    assert again["xc"][-1] == pytest.approx(3.0)
    assert again["yc"][-1] == pytest.approx(0.8)
    for key in ("xc", "yc", "xL", "yL", "halfWidth"):
        assert again[key] == pytest.approx(geom[key])


def test_a_record_with_no_centreline_key_is_a_ramp():
    """Every corridor logged before the pose geometry existed."""
    geom, record = build()
    loaded = CorridorRecord(record)
    assert "centreline" not in loaded.definition
    again = evaluate(loaded)
    assert again["xc"] == pytest.approx(geom["xc"])


def test_a_v1_polygon_only_file_still_loads(tmp_path):
    """Every run in first_test_campaing/ is this shape. It must load, report
    v1, and refuse to invent curves it does not have."""
    v1 = {
        "t": 3.07, "id": 1, "source": "mpc_corr",
        "polygon": [[0.0, 0.4], [3.0, 0.7], [3.0, -0.7], [0.0, -0.4]],
        "meta": {"frame_id": "odom", "psi_ref": 0.026, "length_m": 3.0,
                 "object_mode": False},
    }
    loaded = load_corridors(write_jsonl(tmp_path / "corridors.jsonl", [v1]))
    assert len(loaded) == 1
    rec = loaded[0]
    assert rec.schema == SCHEMA_V1
    assert rec.definition is None
    assert rec.dpsi is None
    assert rec.length_m == 3.0
    assert len(rec.polygon) == 4
    assert evaluate(rec) is None


def test_a_definition_missing_a_parameter_is_treated_as_v1():
    """Structural, not a version string: a record that CLAIMS v2 but cannot be
    evaluated must degrade to v1 here rather than raise inside the evaluator."""
    _, record = build()
    del record["meta"]["definition"]["corr_N"]
    assert schema_of(record) == SCHEMA_V1


def test_mixed_schemas_in_one_file(tmp_path):
    _, v2 = build()
    v1 = {"t": 0.0, "id": 0, "polygon": [[0, 0], [1, 0], [1, 1]], "meta": {}}
    loaded = load_corridors(write_jsonl(tmp_path / "c.jsonl", [v1, v2]))
    assert [r.schema for r in loaded] == [SCHEMA_V1, SCHEMA_V2]


# --------------------------------------------------------------------------
# the file itself
# --------------------------------------------------------------------------

def test_a_missing_file_is_empty_not_an_error(tmp_path):
    assert load_corridors(tmp_path / "nope.jsonl") == []


def test_a_truncated_last_line_is_skipped(tmp_path):
    """The node appends while it runs, so a half-written last line is normal."""
    _, record = build()
    path = tmp_path / "corridors.jsonl"
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(json.dumps(record) + "\n")
        fh.write('{"t": 2.0, "id": 4, "polyg')
    assert len(load_corridors(path)) == 1


def test_blank_lines_are_skipped(tmp_path):
    _, record = build()
    path = tmp_path / "corridors.jsonl"
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n" + json.dumps(record) + "\n\n")
    assert len(load_corridors(path)) == 1
