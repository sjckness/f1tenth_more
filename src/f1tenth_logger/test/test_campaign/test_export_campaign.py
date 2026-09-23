"""export_campaign_csv: the metric window, the numbers, and the manual columns.

Every test folder here is hand-built so each metric has a closed-form answer.
The window is [0, duration]: before it sits a standstill countdown at negative
t, after it sits deliberately awful post-roll data. So every expected value
below is also a test that the window holds -- if the export ever measured
outside the driving window, the junk would swamp it.
"""

from __future__ import annotations

import csv
import json
import math
import os
import random
import shutil

import numpy as np
import pytest
import yaml

from f1tenth_logger.test_campaign import analyze_tests
from f1tenth_logger.test_campaign import export_campaign_csv as exp

COUNTDOWN = 2.5   # standstill before mission_started, at negative t
POST_ROLL = 1.0   # junk after mission_finished that must not reach a metric
DURATION = 10.0

# 20 Hz solves, infeasible from t=1.00 to t=1.45, ok again at 1.50
MPC_ROWS = [
    [f"{i * 0.05:.6g}",
     "infeasible" if 1.0 <= i * 0.05 < 1.5 else "solved",
     f"{10 + (i % 7):.6g}", "1.5", "7"]
    for i in range(201)
]
# still not solving when the window ends
MPC_TAIL = [
    [f"{i * 0.05:.6g}", "solved" if i * 0.05 < 9.0 else "max_iter", "9", "1", "5"]
    for i in range(201)
]

# One solve a second, each predicting a constant lateral offset while the car
# in fact sits at (0, 0) for the whole recording: the error of every step IS
# that offset, so the pooled mean and the worst step are readable by eye. The
# first is solved during the countdown and is 9 m out -- it is there to be
# excluded, and it would swamp any mean that counted it.
HORIZONS = (
    [{"t": -1.0, "i": 0, "corridor_id": 0, "frame_id": "odom", "ts": 0.1,
      "x": [0.0] * 5, "y": [9.0] * 5}]
    + [{"t": float(k), "i": k, "corridor_id": k, "frame_id": "odom", "ts": 0.1,
        "x": [0.0] * 5, "y": [0.2] * 5} for k in (1, 2, 3)]
    + [{"t": 4.0, "i": 4, "corridor_id": 4, "frame_id": "odom", "ts": 0.1,
        "x": [0.0] * 5, "y": [0.5] * 5}]
)


def write_corridors(directory, schema, mode="off", shapes=None, cuts=()):
    """A corridors.jsonl of the requested schema, or none at all for None.

    The geometry is not what these tests measure -- viol_rate_pct reads the
    kinematics column -- so the definition carries the minimum
    corridor_def.schema_of requires rather than a realistic corridor.
    """
    if schema is None:
        return
    record = {"t": 0.0, "id": 0, "source": "mpc_corr",
              "polygon": [[0.0, 0.5], [3.0, 0.5], [3.0, -0.5], [0.0, -0.5]],
              "meta": {"frame_id": "odom", "length_m": 3.0}}
    if schema == "mpc_corr/v2":
        record["meta"]["object_shape"] = "straight"
        record["meta"]["definition"] = {
            "object_corridor_mode": mode,
            "type": "mpc_corr/v2", "C0": [0.0, 0.0], "psiStart": 0.0,
            "psiEnd": 0.0, "dpsi": 0.0, "psiRefTurn": None, "L": 3.0,
            "corr_N": 120, "u_start": 0.0, "u_end": 0.40,
            "w0": 0.4333, "w1": 0.7667, "handle_frac": 0.55,
        }
    with open(directory / "corridors.jsonl", "w", encoding="utf-8") as fh:
        for i, shape in enumerate(shapes or ["straight"]):
            line = json.loads(json.dumps(record))
            line["id"] = i
            line["meta"]["object_shape"] = shape
            if line["meta"].get("definition") is not None:
                line["meta"]["definition"]["cut"] = bool(i in cuts)
            fh.write(json.dumps(line) + "\n")


def _write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        writer.writerows(rows)


def build_test(campaign, test_id, *, clearance_window=(4.0, 6.0), obstacle_min=0.5,
               events=(), mpc=None, ok_statuses=("solved",), path_length=10.0,
               steer_hz=0.5, imu_seconds=5.0, imu_hz=1.0, mission_events=True,
               n_replans=1, corridor_schema="mpc_corr/v2", horizons=None,
               corridor_mode="off", corridor_shapes=None, corridor_cuts=()):
    directory = campaign / "M01" / test_id
    directory.mkdir(parents=True)

    if horizons is not None:
        with open(directory / "horizon.jsonl", "w", encoding="utf-8") as fh:
            for record in horizons:
                fh.write(json.dumps(record) + "\n")

    # viol_rate_pct is gated on the corridor schema (see metrics_for_test), so
    # a folder with no corridors.jsonl now yields an EMPTY one. These fixtures
    # exercise the metric, so they log a v2 corridor by default; pass
    # corridor_schema="v1" or None to exercise the gate itself.
    write_corridors(directory, corridor_schema, corridor_mode,
                    corridor_shapes, corridor_cuts)

    kinematics = []
    for i in range(int(COUNTDOWN * 20)):            # countdown: parked, safe
        t = -COUNTDOWN + i * 0.05
        kinematics.append([f"{t:.6g}", "0", "0", "", "", "", "", "", "", "", "",
                           "0.5", "2"])
    for i in range(int(DURATION * 20) + 1):         # the measured window
        t = i * 0.05
        inside = clearance_window[0] <= t < clearance_window[1]
        obstacle = obstacle_min + abs(t - DURATION / 2) * 0.1
        kinematics.append([f"{t:.6g}", "0", "0", "", "", "", "", "", "", "", "",
                           f"{-0.10 if inside else 0.30:.6g}", f"{obstacle:.6g}"])
    for i in range(int(POST_ROLL * 20)):            # post-roll: pure poison
        t = DURATION + 0.05 + i * 0.05
        kinematics.append([f"{t:.6g}", "0", "0", "", "", "", "", "", "", "", "",
                           "-5", "-9"])
    _write_csv(directory / "kinematics.csv",
               ["t", "x", "y", "yaw", "yaw_rate", "vx", "vy", "speed", "ax", "ay",
                "acc", "corridor_clearance", "obstacle_clearance"], kinematics)

    imu = []
    rng = random.Random(11)
    for i in range(int(COUNTDOWN * 80)):            # countdown: the noise floor
        t = -COUNTDOWN + i / 80.0
        imu.append([f"{t:.6g}", "", f"{rng.gauss(0, 0.02):.6g}",
                    f"{rng.gauss(0, 0.02):.6g}", "9.81", "0", "0", "0", ""])
    for i in range(int(imu_seconds * 80) + 1):      # a clean 1 Hz sine
        t = i / 80.0
        imu.append([f"{t:.6g}", "", f"{math.sin(2 * math.pi * imu_hz * t):.6g}", "0",
                    "9.81", "0", "0", "0", ""])
    for i in range(int(POST_ROLL * 80)):            # post-roll: violent noise
        t = DURATION + 0.05 + i / 80.0
        imu.append([f"{t:.6g}", "", f"{50 * ((i % 2) - 0.5):.6g}",
                    f"{50 * ((i % 2) - 0.5):.6g}", "9.81", "0", "0", "0", ""])
    _write_csv(directory / "imu.csv",
               ["t", "sensor_stamp", "ax", "ay", "az", "gx", "gy", "gz", "imu_yaw"],
               imu)

    commands = []
    for i in range(int(COUNTDOWN * 20)):
        commands.append([f"{-COUNTDOWN + i * 0.05:.6g}", "0", "0", "", "", "",
                         "controller"])
    for i in range(int(DURATION * 20) + 1):
        t = i * 0.05
        commands.append([f"{t:.6g}", "0.8",
                         f"{0.3 * math.sin(2 * math.pi * steer_hz * t):.6g}",
                         "", "", "", "controller"])
    for i in range(int(POST_ROLL * 20)):            # post-roll: steering slammed
        t = DURATION + 0.05 + i * 0.05
        commands.append([f"{t:.6g}", "0", f"{0.4 * ((i % 2) * 2 - 1):.6g}",
                         "", "", "", "controller"])
    _write_csv(directory / "commands.csv",
               ["t", "cmd_speed", "cmd_steer", "cmd_yaw_rate", "cmd_throttle",
                "cmd_brake", "source"], commands)

    if mpc is not None:
        poisoned = list(mpc) + [
            [f"{DURATION + 0.05 + i * 0.05:.6g}", "infeasible", "40", "9", "99"]
            for i in range(20)
        ]
        _write_csv(directory / "mpc.csv",
                   ["t", "status", "solve_time_ms", "cost", "iterations"], poisoned)

    calls = [["0", "initial", "-0.4", "-0.1", "250", "", "40", "80", "1", ""]]
    for i in range(n_replans):
        calls.append([str(i + 1), "replan", f"{1.0 + i}", f"{1.2 + i}", "120", "",
                      "40", "80", "1", ""])
    _write_csv(directory / "llm_calls.csv",
               ["call_idx", "tag", "t_sent", "t_received", "latency_ms", "ttft_ms",
                "prompt_chars", "response_chars", "ok", "error"], calls)

    with open(directory / "events.jsonl", "w", encoding="utf-8") as fh:
        if mission_events:
            fh.write(json.dumps({"t": -COUNTDOWN, "event": "mission_loaded",
                                 "plan_id": "plan-1",
                                 "countdown_s": COUNTDOWN}) + "\n")
            fh.write(json.dumps({"t": 0.0, "event": "mission_started",
                                 "plan_id": "plan-1"}) + "\n")
        for name, cause in events:
            fh.write(json.dumps({"t": 1.0, "event": name, "cause": cause}) + "\n")
        if mission_events:
            fh.write(json.dumps({"t": DURATION, "event": "mission_finished",
                                 "plan_id": "plan-1", "reason": "done"}) + "\n")

    meta = {
        "summary": {"test_id": test_id, "mission": "M01",
                    "auto_outcome": "completed", "auto_success": 1, "reason": "",
                    "path_length": path_length},
        "robot_radius": 0.3,
        "mpc_ok_statuses": list(ok_statuses),
        "auto_outcome": "completed",
        "auto_success": 1,
        "path_length": path_length,
    }
    (directory / "meta.json").write_text(json.dumps(meta, indent=2))
    return directory


def make_campaign(tmp_path):
    campaign = tmp_path / "f1tenth_more" / "first_test_campaing"
    campaign.mkdir(parents=True)
    with open(campaign / "prompts.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump([{"prompt_num": 1, "mission": "M01", "text": "go",
                         "success_criterion": "x"}], fh)
    return campaign


@pytest.fixture(scope="module")
def metrics(tmp_path_factory):
    """One campaign of hand-built tests, scanned once."""
    campaign = make_campaign(tmp_path_factory.mktemp("metrics"))
    build_test(campaign, "P001-R001-20260101T000000", mpc=MPC_ROWS)
    build_test(campaign, "P001-R002-20260101T000100", mpc=MPC_ROWS,
               events=[("contact", "hit a chair")], obstacle_min=0.4)
    build_test(campaign, "P001-R003-20260101T000200", mpc=MPC_TAIL,
               ok_statuses=("solved", "solved_inaccurate"))
    build_test(campaign, "P001-R004-20260101T000300", mpc=MPC_ROWS, imu_seconds=1.0)
    build_test(campaign, "P001-R005-20260101T000400", mpc=MPC_ROWS, path_length=0.0)
    build_test(campaign, "P001-R006-20260101T000500", mpc=MPC_ROWS,
               mission_events=False)
    build_test(campaign, "P001-R007-20260101T000600", mpc=MPC_ROWS,
               horizons=HORIZONS)
    rows = exp.scan_campaign(campaign, exp.DEFAULT_CUTOFF_HZ, exp.DEFAULT_DEADBAND_RAD)
    return campaign, {r["test_id"]: r for r in rows}


@pytest.fixture
def plain(metrics):
    return metrics[1]["P001-R001-20260101T000000"]


@pytest.fixture
def horizons(metrics):
    return metrics[1]["P001-R007-20260101T000600"]


# --------------------------------------------------------------------------
# the numbers
# --------------------------------------------------------------------------

def test_viol_rate_is_a_share_of_time_not_of_samples(plain):
    """2 s of a 10 s window, regardless of how the samples are spaced."""
    assert plain["viol_rate_pct"] == pytest.approx(20.0, abs=0.3)
    assert plain["corridor_schema"] == "mpc_corr/v2"


@pytest.mark.parametrize("schema,expected", [("v1", "v1"), (None, "none")])
def test_a_v1_test_gets_no_viol_rate_at_all(tmp_path, schema, expected):
    """THE GATE. On a v1 test the kinematics' corridor_clearance column was
    measured against the corridor's start cap, which the car sits on by
    construction, so any viol_rate_pct computed from it is an artifact -- 60-80%
    on the archived M02 runs, which never left their corridor. The export
    reports nothing rather than that.

    The clearance data here is IDENTICAL to the v2 fixture's (build_test writes
    the same kinematics.csv either way), so a non-empty answer would prove the
    gate is not reading the schema.
    """
    campaign = make_campaign(tmp_path)
    build_test(campaign, "P001-R001-20260101T000000", mpc=MPC_ROWS,
               corridor_schema=schema)
    rows = exp.scan_campaign(campaign, exp.DEFAULT_CUTOFF_HZ,
                             exp.DEFAULT_DEADBAND_RAD)
    row = rows[0]
    assert row["corridor_schema"] == expected
    assert row["viol_rate_pct"] is None
    # the metrics that do NOT come from the corridor are unaffected
    assert row["feas_pct"] is not None or row["min_clear_raw_m"] is not None


def test_a_mixed_file_counts_as_v1(tmp_path):
    """One v1 line is enough: the run's clearance column was written under the
    old measure for at least part of the drive."""
    campaign = make_campaign(tmp_path)
    build_test(campaign, "P001-R001-20260101T000000", mpc=MPC_ROWS)
    directory = campaign / "M01" / "P001-R001-20260101T000000"
    with open(directory / "corridors.jsonl", "a", encoding="utf-8") as fh:
        fh.write(json.dumps({"t": 1.0, "id": 1,
                             "polygon": [[0, 0], [1, 0], [1, 1]],
                             "meta": {}}) + "\n")
    rows = exp.scan_campaign(campaign, exp.DEFAULT_CUTOFF_HZ,
                             exp.DEFAULT_DEADBAND_RAD)
    assert rows[0]["corridor_schema"] == "v1"
    assert rows[0]["viol_rate_pct"] is None


def test_feasibility_percentage(plain):
    assert plain["feas_pct"] == pytest.approx(100 * (201 - 10) / 201, abs=0.01)


def test_infeasible_streak_runs_to_the_next_ok_solve(plain):
    assert plain["max_infeas_streak_s"] == pytest.approx(0.5)


def test_a_streak_still_open_at_the_end_runs_to_the_end_of_the_window(metrics):
    row = metrics[1]["P001-R003-20260101T000200"]
    assert row["max_infeas_streak_s"] == pytest.approx(1.0, abs=0.06)


def test_custom_ok_statuses_are_honoured(metrics):
    """max_iter is not in this run's ok set, so it counts as not solved."""
    row = metrics[1]["P001-R003-20260101T000200"]
    assert row["feas_pct"] == pytest.approx(100 * 180 / 201, abs=0.3)


def test_jerk_rms_of_a_known_sine(plain):
    """1 Hz at 1 m/s^2 differentiates to 2*pi, and rms of that is 2*pi/sqrt(2)."""
    assert plain["jerk_rms"] == pytest.approx(2 * math.pi / math.sqrt(2), abs=0.15)


def test_steering_reversals_per_metre(plain):
    """0.5 Hz over 10 s crosses the deadband at t=1..9: nine reversals, 10 m."""
    assert plain["steer_rev_per_m"] == pytest.approx(0.9, abs=0.11)


def test_p95_solve_time_excludes_the_post_roll(plain):
    """The post-roll solves are 40 ms; the window's are 10-16."""
    assert plain["mpc_solve_time_p95_ms"] == pytest.approx(16.0, abs=0.5)


def test_the_three_regimes_are_counted_per_test(tmp_path):
    """An 'arc' run that mostly fell back is not an arc run.

    The fallback fires wherever the pose fit needs more than full lock -- 11 of
    the 24 archived rebuilds -- so without these counts an A/B would average a
    mode against itself and call the difference noise.
    """
    campaign = make_campaign(tmp_path)
    build_test(campaign, "P001-R001-20260101T000000", mpc=MPC_ROWS,
               corridor_mode="arc",
               corridor_shapes=["pose_arc", "pose_arc", "straight", "arc"],
               corridor_cuts=(3,))
    row = exp.scan_campaign(campaign, exp.DEFAULT_CUTOFF_HZ,
                            exp.DEFAULT_DEADBAND_RAD)[0]
    assert row["object_corridor_mode"] == "arc"
    assert row["n_corr_pose_arc"] == 2
    assert row["n_corr_straight"] == 1
    assert row["n_corr_arc"] == 1
    # orthogonal to the shape: the cap cuts the ramp arc and the straight alike
    assert row["n_corr_cut"] == 1


def test_no_rebuilds_of_a_regime_is_zero_and_no_corridors_is_empty(tmp_path):
    """Zero is a measurement; empty is the absence of one."""
    campaign = make_campaign(tmp_path)
    build_test(campaign, "P001-R001-20260101T000000", mpc=MPC_ROWS)
    build_test(campaign, "P001-R002-20260101T000100", mpc=MPC_ROWS,
               corridor_schema=None)
    rows = {r["test_id"]: r for r in exp.scan_campaign(
        campaign, exp.DEFAULT_CUTOFF_HZ, exp.DEFAULT_DEADBAND_RAD)}
    assert rows["P001-R001-20260101T000000"]["n_corr_pose_arc"] == 0
    assert rows["P001-R002-20260101T000100"]["n_corr_pose_arc"] is None


def test_the_mode_column_says_what_the_geometry_was_asked_to_do(plain):
    """Beside object_shape, which says what it did.

    'arc_far' that never left the straight shape and 'off' produce the same
    shapes; only the mode tells a campaign that it compared a geometry with
    itself.
    """
    assert plain["object_corridor_mode"] == "off"
    assert plain["object_shape"] == "straight"


def test_a_test_whose_corridors_carry_no_mode_leaves_the_column_empty(tmp_path):
    """Every run recorded before the mode existed, and every v1 log."""
    campaign = make_campaign(tmp_path)
    build_test(campaign, "P001-R001-20260101T000000", mpc=MPC_ROWS,
               corridor_schema="v1")
    row = exp.scan_campaign(campaign, exp.DEFAULT_CUTOFF_HZ,
                            exp.DEFAULT_DEADBAND_RAD)[0]
    assert row["object_corridor_mode"] is None


def test_the_prediction_error_pools_every_predicted_step(horizons):
    """15 steps 0.20 m off and 5 steps 0.50 m off: 0.275 m mean, 0.50 m worst.

    Pooled over steps, not averaged per horizon -- a solve measured over two
    of its steps must not weigh as much as one measured over all five.
    """
    assert horizons["horizon_err_mean_m"] == pytest.approx(0.275)
    assert horizons["horizon_err_max_m"] == pytest.approx(0.5)


def test_a_horizon_solved_in_the_countdown_is_counted_but_not_measured(horizons):
    """The car had not moved yet, so its 9 m miss says nothing about tracking.

    n_horizons is every horizon in the file -- how much plan was logged -- so
    it still sees five while the error saw four.
    """
    assert horizons["n_horizons"] == 5
    assert horizons["horizon_err_max_m"] == pytest.approx(0.5)


def test_a_test_with_no_horizon_log_gets_empty_columns(plain):
    """Empty, not zero: every test recorded before the stream existed."""
    for column in ("horizon_err_mean_m", "horizon_err_max_m", "n_horizons"):
        assert plain[column] is None, column


def test_min_clearance_is_not_clamped_when_nothing_was_touched(plain):
    assert plain["min_clear_m"] == pytest.approx(0.5)
    assert plain["min_clear_raw_m"] == pytest.approx(0.5)
    assert plain["estop"] == 0 and plain["contact"] == 0


def test_contact_clamps_min_clear_but_not_the_raw_value(metrics):
    row = metrics[1]["P001-R002-20260101T000100"]
    assert row["contact"] == 1 and row["estop"] == 0
    assert row["min_clear_m"] == 0.0
    assert row["min_clear_raw_m"] == pytest.approx(0.4)


def test_date_and_time_come_from_the_test_id(plain):
    assert plain["date"] == "2026-01-01" and plain["time"] == "00:00:00"


def test_the_llm_columns(plain):
    assert plain["llm_latency_ms"] == pytest.approx(250.0)
    assert plain["n_replans"] == 1


def test_the_countdown_columns(plain):
    assert plain["countdown_s"] == pytest.approx(COUNTDOWN)
    assert plain["drive_duration_s"] == pytest.approx(DURATION)


def test_standstill_jerk_is_the_countdown_noise_floor(plain):
    floor = plain["standstill_jerk_rms"]
    assert floor is not None and 0.0 < floor < 1.0
    assert plain["jerk_rms"] > 10 * floor


@pytest.mark.parametrize("column", [
    "viol_rate_pct", "min_clear_m", "jerk_rms", "mpc_solve_time_p95_ms"])
def test_post_roll_junk_never_reaches_a_metric(plain, column):
    """Every one of these would be wild if the post-roll had been measured."""
    expected = {"viol_rate_pct": 20.0, "min_clear_m": 0.5,
                "jerk_rms": 2 * math.pi / math.sqrt(2),
                "mpc_solve_time_p95_ms": 16.0}[column]
    assert plain[column] == pytest.approx(expected, rel=0.05, abs=0.5)


def test_jerk_needs_two_seconds_of_imu(metrics):
    assert metrics[1]["P001-R004-20260101T000300"]["jerk_rms"] is None


def test_steering_needs_a_path_length(metrics):
    assert metrics[1]["P001-R005-20260101T000400"]["steer_rev_per_m"] is None


def test_a_deadband_wider_than_the_signal_gives_no_reversals():
    t = np.asarray([i * 0.05 for i in range(201)])
    steer = np.asarray([0.3 * math.sin(2 * math.pi * 0.5 * x) for x in t])
    assert exp.steer_reversals(t, steer, 5.0, 0.5) == 0


def test_without_mission_started_the_driving_metrics_are_empty(metrics):
    """Recorded, exported, but not scored: there was no driving window."""
    row = metrics[1]["P001-R006-20260101T000500"]
    for column in ("viol_rate_pct", "min_clear_m", "feas_pct",
                   "max_infeas_streak_s", "jerk_rms", "steer_rev_per_m",
                   "drive_duration_s", "mpc_solve_time_p95_ms"):
        assert row[column] is None, column
    assert row["auto_outcome"] == "completed" and row["n_replans"] == 1


# --------------------------------------------------------------------------
# the file
# --------------------------------------------------------------------------

@pytest.fixture
def written(tmp_path):
    """A campaign with two tests, exported once in the Excel-EU default."""
    campaign = make_campaign(tmp_path)
    build_test(campaign, "P001-R001-20260101T000000", mpc=MPC_ROWS)
    build_test(campaign, "P001-R002-20260101T000100", mpc=MPC_ROWS,
               events=[("contact", "hit a chair")], obstacle_min=0.4)
    assert exp.main([str(campaign)]) == 0
    return campaign, campaign / exp.RESULTS_NAME


def test_excel_eu_is_the_default(written):
    _, target = written
    raw = target.read_bytes()
    assert raw.startswith(b"\xef\xbb\xbf"), "Excel needs the BOM for accents"
    header = raw.decode("utf-8-sig").splitlines()[0]
    assert header.split(";") == exp.COLUMNS
    body = raw.decode("utf-8-sig").splitlines()[1]
    assert ";20,000;" in body, "decimal comma"
    assert all(len(cell.split(",")[1]) == 3
               for cell in body.split(";") if "," in cell and cell[0].isdigit())


def test_plain_switches_separator_and_decimal(written):
    campaign, target = written
    assert exp.main([str(campaign), "--plain", "--cutoff-hz", "8",
                     "--deadband-rad", "0.05"]) == 0
    header = target.read_text(encoding="utf-8-sig").splitlines()[0]
    assert header.split(",") == exp.COLUMNS
    settings = json.loads((campaign / exp.SETTINGS_NAME).read_text())
    assert settings["csv_format"] == "plain"
    assert settings["cutoff_hz"] == 8.0 and settings["deadband_rad"] == 0.05
    assert settings["filter_order"] == exp.FILTER_ORDER


def test_settings_record_the_filter(written):
    campaign, _ = written
    settings = json.loads((campaign / exp.SETTINGS_NAME).read_text())
    assert settings["cutoff_hz"] == exp.DEFAULT_CUTOFF_HZ
    assert settings["deadband_rad"] == exp.DEFAULT_DEADBAND_RAD
    assert settings["csv_format"] == "excel-eu"


def _read(target, delimiter=";"):
    with open(target, newline="", encoding="utf-8-sig") as fh:
        reader = csv.DictReader(fh, delimiter=delimiter)
        return reader.fieldnames, list(reader)


def test_manual_columns_survive_everything(written):
    """The whole point of the file: a human's verdict is never overwritten."""
    campaign, target = written
    fields, rows = _read(target)
    assert all(not r["success"] for r in rows), "success must start empty"

    marked = {
        "P001-R001-20260101T000000": ("1", "1", "went well; no notes needed"),
        "P001-R002-20260101T000100": ("0", "0", "hit the chair (café)"),
    }
    for row in rows:
        if row["test_id"] in marked:
            row["success"], row["transl_ok"], row["notes"] = marked[row["test_id"]]
    with open(target, "w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, delimiter=";")
        writer.writeheader()
        writer.writerows(rows)

    build_test(campaign, "P001-R003-20260101T000200", mpc=MPC_ROWS)   # a new test
    shutil.rmtree(campaign / "M01" / "P001-R002-20260101T000100")     # a vanished one
    assert exp.main([str(campaign)]) == 0

    _, after_rows = _read(target)
    after = {r["test_id"]: r for r in after_rows}
    for test_id, (success, transl, note) in marked.items():
        assert after[test_id]["success"] == success
        assert after[test_id]["transl_ok"] == transl
        assert after[test_id]["notes"] == note, "a ';' inside a note broke the row"
    assert after["P001-R003-20260101T000200"]["success"] == ""
    assert "P001-R002-20260101T000100" in after, "a row was deleted"
    assert after["P001-R001-20260101T000000"]["feas_pct"] != ""
    assert list(after) == sorted(after), "rows must be sorted by mission, then id"


def test_a_locked_file_falls_back_instead_of_losing_the_run(written, monkeypatch):
    """Excel holds campaign_results.csv open: say so, write the copy, lose nothing."""
    campaign, target = written
    before = target.read_text(encoding="utf-8-sig")
    real_replace = os.replace

    def refuse_the_target(src, dst):
        if str(dst).endswith(exp.RESULTS_NAME):
            raise PermissionError(13, "Permission denied")
        return real_replace(src, dst)

    monkeypatch.setattr(os, "replace", refuse_the_target)
    assert exp.main([str(campaign)]) == 0
    fallback = campaign / exp.FALLBACK_NAME
    assert fallback.exists()
    assert target.read_text(encoding="utf-8-sig") == before
    assert list(campaign.glob("*.tmp")) == []


# --------------------------------------------------------------------------
# the analysis reads the manual verdict
# --------------------------------------------------------------------------

@pytest.fixture
def evaluated(written):
    """One test marked pass, one marked fail, one left unevaluated."""
    campaign, target = written
    build_test(campaign, "P001-R003-20260101T000200", mpc=MPC_ROWS)
    exp.main([str(campaign)])
    fields, rows = _read(target)
    verdicts = {"P001-R001-20260101T000000": "1", "P001-R002-20260101T000100": "0"}
    for row in rows:
        row["success"] = verdicts.get(row["test_id"], "")
    with open(target, "w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, delimiter=";")
        writer.writeheader()
        writer.writerows(rows)
    return campaign, target


def test_the_manual_verdict_overrides_the_automatic_hint(evaluated):
    campaign, _ = evaluated
    runs, has_manual, unreadable = analyze_tests.load_campaign(campaign)
    assert has_manual and not unreadable
    verdicts = {r.test_id: r.verdict for r in runs}
    assert verdicts["P001-R001-20260101T000000"] == "pass"
    assert verdicts["P001-R002-20260101T000100"] == "fail"
    failed = [r for r in runs if r.test_id == "P001-R002-20260101T000100"][0]
    assert failed.auto_outcome == "completed", "the hint said completed; the human said no"
    assert verdicts["P001-R003-20260101T000200"] == "unevaluated"


def test_k_over_n_counts_only_evaluated_tests(evaluated):
    campaign, _ = evaluated
    runs, _, _ = analyze_tests.load_campaign(campaign)
    overall = [r for r in analyze_tests.build_summary(runs)
               if r["level"] == "overall"][0]
    assert (overall["k"], overall["n"]) == (1, 2)
    assert overall["n_tests"] == 3 and overall["not_evaluated"] == 1
    assert overall["feas_pct_median"] is not None


def test_a_verdict_that_is_neither_0_nor_1_is_flagged_not_guessed(evaluated):
    campaign, target = evaluated
    fields, rows = _read(target)
    for row in rows:
        if row["test_id"] == "P001-R003-20260101T000200":
            row["success"] = "maybe"
    with open(target, "w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, delimiter=";")
        writer.writeheader()
        writer.writerows(rows)
    runs, _, unreadable = analyze_tests.load_campaign(campaign)
    assert [text for _, text in unreadable] == ["maybe"]
    assert {r.test_id: r.verdict for r in runs}["P001-R003-20260101T000200"] \
        == "unevaluated"


def test_the_report_says_what_is_still_unevaluated(evaluated, tmp_path):
    campaign, _ = evaluated
    out = tmp_path / "analysis"
    assert analyze_tests.main([str(campaign), "--out", str(out)]) == 0
    report = (out / "report.txt").read_text()
    assert "not yet evaluated:" in report
    assert (out / "dashboard.png").exists()
