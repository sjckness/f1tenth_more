"""robot_logger: ids, geometry, thread safety, the summary, the abort paths.

Pure Python -- no ROS, no hardware. Everything here is either arithmetic with
a known answer or a property the rest of the campaign depends on.
"""

from __future__ import annotations

import csv
import json
import math
import os
import subprocess
import sys
import threading
from pathlib import Path

import pytest
import yaml

from f1tenth_logger.test_campaign import analyze_tests
from f1tenth_logger.test_campaign.robot_logger import (
    TestLogger,
    corridor_clearance,
    corridor_from_centerline,
    find_root,
    make_test_id,
    parse_test_id,
    signed_clearance,
    signed_wall_clearance,
    split_corridor_polygon,
)

PROMPTS = [
    {"prompt_num": 1, "mission": "M01", "text": "go", "success_criterion": "x"},
    {"prompt_num": 2, "mission": "M02", "text": "turn", "success_criterion": "y"},
]


@pytest.fixture
def root(tmp_path):
    """A fresh project root with a two-entry prompt table."""
    root = tmp_path / "f1tenth_more"
    campaign = root / "first_test_campaing"
    campaign.mkdir(parents=True)
    with open(campaign / "prompts.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(PROMPTS, fh)
    return root


@pytest.fixture
def campaign(root):
    return root / "first_test_campaing"


def logger_for(root, prompt=1, **kwargs):
    kwargs.setdefault("robot_radius", 0.3)
    return TestLogger(prompt, root=root, **kwargs)


# --------------------------------------------------------------------------
# test ids
# --------------------------------------------------------------------------

def test_test_id_round_trip():
    parsed = parse_test_id(make_test_id(4, 12))
    assert parsed["prompt_num"] == 4 and parsed["repetition"] == 12


def test_the_documented_example_parses():
    parsed = parse_test_id("P004-R012-20260918T143512")
    assert parsed["prompt_num"] == 4
    assert parsed["repetition"] == 12
    assert parsed["datetime"].year == 2026 and parsed["datetime"].minute == 35


@pytest.mark.parametrize("bad", [
    "P4-R12-20260918T143512",          # unpadded
    "P004-R012-2026-09-18",            # wrong stamp format
    "P004-R012-20260918T1435120",      # too long
    "nonsense",
])
def test_malformed_ids_are_rejected(bad):
    with pytest.raises(ValueError):
        parse_test_id(bad)


@pytest.mark.parametrize("bad", [-1, 1000])
def test_out_of_range_indices_are_rejected(bad):
    with pytest.raises(ValueError):
        make_test_id(bad, 0)


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------

SQUARE = [(0, 0), (4, 0), (4, 2), (0, 2)]


def test_clearance_inside_is_positive():
    assert signed_clearance(2, 1, SQUARE, 0.0) == pytest.approx(1.0)
    assert signed_clearance(2, 1, SQUARE, 0.3) == pytest.approx(0.7)


def test_clearance_outside_is_negative():
    assert signed_clearance(2, 2.5, SQUARE, 0.3) == pytest.approx(-0.8)


def test_clearance_on_the_edge_is_minus_the_radius():
    assert signed_clearance(2, 2.0, SQUARE, 0.3) == pytest.approx(-0.3)


def test_corridor_ends_are_extended_so_the_start_is_not_clipped():
    """The bug end_extension exists to prevent: a robot sitting exactly on the
    first centerline point reading a negative clearance before it has moved."""
    line = [(0.0, 0.0), (2.5, 0.0), (5.0, 0.0)]
    poly = corridor_from_centerline(line, 1.2)
    assert signed_clearance(0.0, 0.0, poly, 0.3) == pytest.approx(0.3)
    assert signed_clearance(5.0, 0.0, poly, 0.3) == pytest.approx(0.3)

    without = corridor_from_centerline(line, 1.2, end_extension=0.0)
    assert signed_clearance(0.0, 0.0, without, 0.3) == pytest.approx(-0.3)


def test_corridor_half_width_is_honoured():
    poly = corridor_from_centerline([(0.0, 0.0), (5.0, 0.0)], 1.2)
    assert signed_clearance(2.5, 0.45, poly, 0.0) == pytest.approx(0.15)
    assert signed_clearance(2.5, 0.7, poly, 0.0) < 0


def test_a_bend_stays_a_valid_polygon():
    bent = corridor_from_centerline([(0, 0), (3, 0), (3, 3)], 1.0)
    assert len(bent) >= 6
    assert signed_clearance(3, 0, bent, 0) > 0


def test_a_one_point_centerline_is_rejected():
    with pytest.raises(ValueError):
        corridor_from_centerline([(0, 0)], 1.0)


# --------------------------------------------------------------------------
# wall clearance: the end-cap artifact
# --------------------------------------------------------------------------

def straight_corridor(length=3.0, w0=0.4333, w1=0.7667, n=120):
    """A straight mpc_corr-shaped corridor along +x, widening w0 -> w1.

    Same vertex order corridor_payload emits: left wall forward, right wall
    back. Closed form, so every expected value below is exact arithmetic.
    """
    us = [i / (n - 1) for i in range(n)]
    left = [(length * u, w0 + (w1 - w0) * u) for u in us]
    right = [(length * u, -(w0 + (w1 - w0) * u)) for u in us]
    return left + right[::-1]


def test_a_car_at_the_corridor_start_reads_the_half_width_not_zero():
    """THE ARTIFACT THIS COMMIT REMOVES.

    Every mpc_corr corridor passes through the car's own position, so the car
    sits exactly on the polygon's start-cap edge and the all-edges measure
    returns 0 -- or -robot_radius with a radius set. Both values are in the 13
    archived runs. The walls-only measure returns the real half-width.
    """
    poly = straight_corridor()
    assert signed_clearance(0.0, 0.0, poly, 0.0) == pytest.approx(0.0)
    assert signed_clearance(0.0, 0.0, poly, 0.3) == pytest.approx(-0.3)

    assert corridor_clearance(0.0, 0.0, poly, 0.0) == pytest.approx(0.4333)
    assert corridor_clearance(0.0, 0.0, poly, 0.3) == pytest.approx(0.1333)


def test_wall_clearance_is_exact_away_from_the_caps():
    """Mid-corridor the two measures agree: this change only touches the caps.

    Parallel walls here (w1 == w0) so every expected number is the half-width
    exactly. On the real widening corridor the perpendicular distance to a
    sloped wall carries a cos(atan(dw/dL)) factor -- correct, but it would make
    these assertions approximate for a reason that has nothing to do with the
    cap being tested.
    """
    poly = straight_corridor(w0=0.5, w1=0.5)
    assert corridor_clearance(1.5, 0.0, poly, 0.0) == pytest.approx(0.5)
    assert corridor_clearance(1.5, 0.0, poly, 0.0) == pytest.approx(
        signed_clearance(1.5, 0.0, poly, 0.0), abs=1e-9
    )
    assert corridor_clearance(1.5, 0.2, poly, 0.0) == pytest.approx(0.3)
    assert corridor_clearance(1.5, -0.2, poly, 0.0) == pytest.approx(0.3)


def test_a_car_outside_a_wall_is_negative():
    poly = straight_corridor(w0=0.5, w1=0.5)
    assert corridor_clearance(1.5, 0.75, poly, 0.0) == pytest.approx(-0.25)
    assert corridor_clearance(1.5, -0.75, poly, 0.0) == pytest.approx(-0.25)
    # and at the start, where the old measure could not tell inside from out
    assert corridor_clearance(0.0, 0.6, poly, 0.0) < 0.0


def test_a_widening_wall_is_measured_perpendicular_to_itself():
    """The real corridor widens 0.4333 -> 0.7667 over 3 m, so the distance to a
    wall is the PERPENDICULAR one, shorter than the vertical offset by
    cos(atan(dw/dL)). Pinned so the slope is not quietly dropped."""
    poly = straight_corridor()
    slope = (0.7667 - 0.4333) / 3.0
    expected = 0.25 / math.hypot(1.0, slope)
    assert corridor_clearance(1.5, 0.85, poly, 0.0) == pytest.approx(-expected, abs=1e-6)


def test_the_end_cap_still_catches_an_overrun():
    """The one cap that is NOT extended.

    On the object branch the corridor ends exactly at the goal, and M03/M04
    are scored on stopping short of it, so a car past the end must read
    negative. Only the START cap is dropped.
    """
    poly = straight_corridor(length=3.0)
    assert corridor_clearance(2.9, 0.0, poly, 0.0) > 0.0
    assert corridor_clearance(3.2, 0.0, poly, 0.0) < 0.0


def test_split_corridor_polygon_round_trips():
    left, right = split_corridor_polygon(straight_corridor(n=5))
    assert len(left) == len(right) == 5
    assert left[0] == (0.0, 0.4333)
    assert right[0] == (0.0, -0.4333)
    assert left[-1][0] == pytest.approx(3.0)


def test_a_polygon_that_is_not_two_walls_falls_back():
    """An odd vertex count cannot be left + reversed right; rather than guess a
    split, corridor_clearance uses the all-edges measure."""
    assert split_corridor_polygon(SQUARE[:3]) is None
    triangle = [(0, 0), (4, 0), (2, 3)]
    assert corridor_clearance(2, 1, triangle, 0.0) == pytest.approx(
        signed_clearance(2, 1, triangle, 0.0)
    )


def test_wall_clearance_rejects_a_degenerate_wall():
    with pytest.raises(ValueError):
        signed_wall_clearance(0.0, 0.0, [(0.0, 1.0)], [(0.0, -1.0)], 0.0)


def test_the_logger_records_wall_clearance(root):
    """End to end: the number that reaches kinematics.csv is the wall one."""
    log = logger_for(root, robot_radius=0.0)
    log.log_corridor(straight_corridor(), t=0.0)
    assert log.log_state(0.0, 0.0, yaw=0.0, t=0.1) == pytest.approx(0.4333)
    summary = log.finish("completed")
    assert summary["min_corridor_clearance"] == pytest.approx(0.4333)


# --------------------------------------------------------------------------
# thread safety
# --------------------------------------------------------------------------

def test_concurrent_writers_lose_nothing(root):
    """ROS callbacks arrive on different threads; no row may be lost or torn."""
    log = logger_for(root)
    log.log_corridor(corridor_from_centerline([(0.0, 0.0), (5.0, 0.0)], 1.2), t=0.0)
    samples, workers = 300, 4
    barrier = threading.Barrier(workers)

    def worker(k):
        barrier.wait()
        for i in range(samples):
            t = i * 0.01
            log.log_imu(0.1, 0.2, 9.81, 0, 0, 0.05, t=t)
            log.log_state(i * 0.01, 0.0, yaw=0.0, t=t)
            log.log_command(cmd_speed=0.5, source=f"thread{k}", t=t)

    threads = [threading.Thread(target=worker, args=(k,)) for k in range(workers)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    summary = log.finish("completed")

    rows = list(csv.reader(open(log.dir / "kinematics.csv")))
    assert all(len(r) == 13 for r in rows), "a row was torn by a concurrent write"
    assert len(rows) - 1 == samples * workers
    assert summary["n_imu"] == samples * workers
    assert summary["n_cmd"] == samples * workers


# --------------------------------------------------------------------------
# derived quantities
# --------------------------------------------------------------------------

def test_velocity_acceleration_and_path_length_are_differenced(root):
    log = logger_for(root)
    log.log_state(0.0, 0.0, yaw=0.0, t=0.0)
    log.log_state(1.0, 0.0, yaw=0.0, t=1.0)
    log.log_state(3.0, 0.0, yaw=0.0, t=2.0)
    summary = log.finish("completed")
    assert summary["path_length"] == pytest.approx(3.0)
    assert summary["max_speed"] == pytest.approx(2.0)
    assert summary["max_acc"] == pytest.approx(1.0)
    assert summary["pose_rate_hz"] == pytest.approx(1.0)


def test_the_yaw_wrap_is_not_a_spike(root):
    """-pi to +pi is a small turn, not 62 rad/s."""
    log = logger_for(root)
    log.log_state(0.0, 0.0, yaw=math.pi - 0.05, t=0.0)
    log.log_state(0.1, 0.0, yaw=-math.pi + 0.05, t=0.1)
    summary = log.finish("completed")
    assert summary["max_abs_yaw_rate"] == pytest.approx(1.0, abs=0.01)


def test_rates_come_from_each_stream_own_stamps(root):
    log = logger_for(root)
    for i in range(9):
        log.log_imu(0.0, 0.0, 9.81, 0, 0, 0.0, t=i * 0.0125)
    for i in range(3):
        log.log_state(0.0, 0.0, t=i * 0.05)
    summary = log.finish("completed")
    assert summary["imu_rate_hz"] == pytest.approx(80.0)
    assert summary["pose_rate_hz"] == pytest.approx(20.0)
    assert summary["cmd_rate_hz"] is None


def test_clearance_is_exposed_live_for_the_car_to_abort_on(root):
    log = logger_for(root)
    poly = corridor_from_centerline([(0.0, 0.0), (5.0, 0.0)], 1.2)
    log.log_corridor(poly)
    log.log_state(2.5, 0.0)
    assert log.last_corridor_clearance == pytest.approx(0.3)
    log.log_state(2.5, 0.9)
    assert log.last_corridor_clearance < 0
    log.finish("aborted", "left the corridor")


# --------------------------------------------------------------------------
# LLM calls
# --------------------------------------------------------------------------

def test_llm_call_records_both_outcomes(root):
    log = logger_for(root)
    with log.llm_call("hello", tag="initial") as call:
        call.first_token()
        call.set_response("world!")
    with pytest.raises(TimeoutError):
        with log.llm_call("boom", tag="replan"):
            raise TimeoutError("server gone")
    summary = log.finish("completed")

    rows = list(csv.DictReader(open(log.dir / "llm_calls.csv")))
    assert summary["n_llm_calls"] == 2 and len(rows) == 2
    assert rows[0]["ok"] == "1" and rows[0]["response_chars"] == "6"
    assert float(rows[0]["ttft_ms"]) > 0
    assert rows[1]["ok"] == "0"
    assert rows[1]["error"] == "TimeoutError: server gone"
    first = json.loads(open(log.dir / "llm_calls.jsonl").readline())
    assert first["prompt"] == "hello" and first["response"] == "world!"


def test_record_llm_call_accepts_a_call_someone_else_timed(root):
    """What the ROS node does: the planner measured it, this only records it.

    t_sent is negative because the prompt went out before the test folder --
    and therefore the time base -- existed.
    """
    log = logger_for(root)
    log.record_llm_call("prompt", response="plan", tag="initial",
                        t_sent=-0.6, t_received=-0.1, latency_ms=500.0)
    summary = log.finish("completed")
    row = list(csv.DictReader(open(log.dir / "llm_calls.csv")))[0]
    assert float(row["t_sent"]) == pytest.approx(-0.6)
    assert float(row["latency_ms"]) == pytest.approx(500.0)
    assert row["tag"] == "initial"
    assert summary["llm_latency_mean_ms"] == pytest.approx(500.0)


# --------------------------------------------------------------------------
# closing a test
# --------------------------------------------------------------------------

def test_an_exception_aborts_with_its_type_and_message(root):
    with pytest.raises(RuntimeError):
        with logger_for(root) as log:
            directory = log.dir
            raise RuntimeError("boom")
    meta = json.loads((directory / "meta.json").read_text())
    assert meta["auto_outcome"] == "aborted"
    assert meta["reason"] == "exception: RuntimeError: boom"
    assert meta["auto_success"] == 0


def test_leaving_the_block_without_finishing_is_an_abort(root):
    with logger_for(root) as log:
        directory = log.dir
    meta = json.loads((directory / "meta.json").read_text())
    assert meta["reason"] == "no outcome recorded"


def test_a_process_that_exits_without_finishing_still_closes(root):
    """atexit: a crashed run is data, and must not be left half-written."""
    script = (
        "from f1tenth_logger.test_campaign.robot_logger import TestLogger\n"
        f"log = TestLogger(1, root={str(root)!r}, robot_radius=0.3)\n"
        "log.log_state(0.0, 0.0)\n"
        "print(log.dir)\n"
    )
    out = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    directory = Path(out.stdout.strip().splitlines()[-1])
    meta = json.loads((directory / "meta.json").read_text())
    assert meta["reason"] == "process exited without an outcome"


def test_finish_is_idempotent_and_validates_its_outcome(root):
    log = logger_for(root)
    first = log.finish("completed", "done")
    again = log.finish("aborted", "ignored")
    assert again["auto_outcome"] == "completed" and first == again
    with pytest.raises(ValueError):
        logger_for(root).finish("nope")


# --------------------------------------------------------------------------
# the campaign folder
# --------------------------------------------------------------------------

def test_repetitions_are_consecutive_and_folders_are_never_reused(root, campaign):
    for _ in range(3):
        logger_for(root).finish("completed")
    reps = sorted(int(p.name[6:9]) for p in (campaign / "M01").iterdir() if p.is_dir())
    assert reps == [1, 2, 3]


def test_campaign_and_mission_metadata_are_written(root, campaign):
    logger_for(root).finish("completed")
    assert (campaign / "M01" / "mission.json").exists()
    payload = json.loads((campaign / "campaign.json").read_text())
    assert "git_commit" in payload and "created" in payload


def test_results_csv_has_one_row_per_finished_run_in_spec_order(root, campaign):
    logger_for(root).finish("completed")
    logger_for(root).finish("aborted", "left the corridor")
    unfinished = logger_for(root)          # constructed, deliberately not finished
    rows = list(csv.DictReader(open(campaign / "results.csv")))
    assert len(rows) == 2
    assert list(rows[0])[:6] == [
        "test_id", "mission", "prompt_num", "repetition", "prompt_hash", "git_commit"]
    assert "auto_success" in rows[0] and "auto_outcome" in rows[0]
    assert "success" not in rows[0], "the manual verdict must not live here"
    assert unfinished.dir.exists() and not (unfinished.dir / "meta.json").exists()


def test_mpc_log(root):
    log = TestLogger(1, root=root, robot_radius=0.3,
                     mpc_ok_statuses=("solved", "solved_inaccurate"))
    for i, status in enumerate(
            ["solved", "infeasible", "infeasible", "solved_inaccurate"]):
        log.log_mpc(status, solve_time_ms=10.0 + i, cost=1.5, iterations=7, t=i * 0.05)
    log.finish("completed")
    rows = list(csv.DictReader(open(log.dir / "mpc.csv")))
    assert list(rows[0]) == ["t", "status", "solve_time_ms", "cost", "iterations"]
    assert [r["status"] for r in rows] == [
        "solved", "infeasible", "infeasible", "solved_inaccurate"]
    meta = json.loads((log.dir / "meta.json").read_text())
    assert meta["mpc_ok_statuses"] == ["solved", "solved_inaccurate"]


def test_an_empty_ok_status_set_is_rejected(root):
    with pytest.raises(ValueError):
        TestLogger(1, root=root, mpc_ok_statuses=())


# --------------------------------------------------------------------------
# the prompt table is the only source of folder names
# --------------------------------------------------------------------------

def test_an_unknown_prompt_is_refused(root):
    with pytest.raises(KeyError):
        TestLogger(99, root=root)
    with pytest.raises(KeyError):
        TestLogger("some text nobody tabulated", root=root)


def test_a_prompt_can_be_looked_up_by_its_exact_text(root):
    log = TestLogger("turn", root=root)
    assert log.mission == "M02" and log.prompt_num == 2
    log.finish("completed")


def test_a_changed_prompt_warns_and_is_recorded(root, campaign):
    logger_for(root, prompt=2).finish("completed")
    changed = [dict(p) for p in PROMPTS]
    changed[1]["text"] = "turn, but reworded"
    with open(campaign / "prompts.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(changed, fh)

    script = (
        "from f1tenth_logger.test_campaign.robot_logger import TestLogger\n"
        f"log = TestLogger(2, root={str(root)!r})\n"
        "print(log.dir)\n"
        "log.finish('completed')\n"
    )
    out = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    directory = Path(out.stdout.strip().splitlines()[-1])
    meta = json.loads((directory / "meta.json").read_text())
    assert meta["prompt_changed"] is True
    assert "WARNING" in out.stderr and "changed" in out.stderr


# --------------------------------------------------------------------------
# the analysis reads what the logger wrote
# --------------------------------------------------------------------------

def test_a_folder_with_no_meta_json_is_an_aborted_run(root, campaign):
    logger_for(root).finish("completed")
    (campaign / "M01" / "P001-R099-20260921T120000").mkdir()
    runs, has_manual, _ = analyze_tests.load_campaign(campaign)
    crashed = [r for r in runs if r.test_id.startswith("P001-R099")]
    assert len(crashed) == 1
    assert crashed[0].reason == "no meta.json"
    assert crashed[0].auto_outcome == "aborted"
    assert not has_manual
    assert all(r.verdict == "unevaluated" for r in runs)


@pytest.mark.parametrize("k,n,expected", [(7, 10, (0.3968, 0.8922)), (0, 10, (0.0, None))])
def test_wilson_interval(k, n, expected):
    low, high = analyze_tests.wilson(k, n)
    assert low == pytest.approx(expected[0], abs=1e-3)
    if expected[1] is not None:
        assert high == pytest.approx(expected[1], abs=1e-3)


def test_analysis_filters_and_reports(root, campaign, tmp_path):
    logger_for(root).finish("completed")
    logger_for(root, prompt=2).finish("aborted", "left the corridor")
    out = tmp_path / "analysis"
    assert analyze_tests.main([str(campaign), "--mission", "M01", "--out", str(out)]) == 0
    rows = list(csv.DictReader(open(out / "summary.csv")))
    assert {r["mission"] for r in rows} == {"M01", "ALL"}
    assert (out / "overview_map_M01.png").exists()
    assert not (out / "overview_map_M02.png").exists()
    assert analyze_tests.main([str(campaign), "--prompt", "77", "--out", str(out)]) == 1


# --------------------------------------------------------------------------
# finding the project root
# --------------------------------------------------------------------------

def test_the_upward_search_finds_the_workspace(monkeypatch):
    monkeypatch.delenv("F1TENTH_MORE_ROOT", raising=False)
    found = find_root()
    assert found.name == "f1tenth_more"
    assert (found / "src" / "f1tenth_logger" / "package.xml").is_file()


def _workspace(tmp_path):
    """A workspace with the two decoys that fooled the old name-based walk:
    src/f1tenth_more and install/f1tenth_more, both folders named like the
    workspace that are not it."""
    ws = tmp_path / "f1tenth_more"
    pkg = ws / "src" / "f1tenth_logger"
    (pkg / "f1tenth_logger" / "test_campaign").mkdir(parents=True)
    (pkg / "package.xml").write_text("<package/>")
    source = pkg / "f1tenth_logger" / "test_campaign" / "robot_logger.py"
    source.write_text("")
    (ws / "src" / "f1tenth_more").mkdir()
    (ws / "install" / "f1tenth_more").mkdir(parents=True)
    return ws, source


def test_from_source_the_search_finds_the_workspace_not_the_decoy(tmp_path, monkeypatch):
    monkeypatch.delenv("F1TENTH_MORE_ROOT", raising=False)
    ws, source = _workspace(tmp_path)
    assert find_root(anchors=[source]) == ws.resolve()


def test_from_a_colcon_install_the_search_finds_the_workspace(tmp_path, monkeypatch):
    monkeypatch.delenv("F1TENTH_MORE_ROOT", raising=False)
    ws, _ = _workspace(tmp_path)
    installed = (ws / "install" / "f1tenth_logger" / "lib" / "python3.10"
                 / "site-packages" / "f1tenth_logger" / "test_campaign")
    installed.mkdir(parents=True)
    (installed / "robot_logger.py").write_text("")
    assert find_root(anchors=[installed / "robot_logger.py"]) == ws.resolve()


def test_from_a_symlink_install_the_search_finds_the_workspace(tmp_path, monkeypatch):
    """--symlink-install imports from build/f1tenth_logger/f1tenth_logger, a
    symlink into src/: both the path as imported and as resolved work."""
    monkeypatch.delenv("F1TENTH_MORE_ROOT", raising=False)
    ws, source = _workspace(tmp_path)
    build = ws / "build" / "f1tenth_logger"
    build.mkdir(parents=True)
    (build / "f1tenth_logger").symlink_to(source.parent.parent, target_is_directory=True)
    imported = build / "f1tenth_logger" / "test_campaign" / "robot_logger.py"
    assert imported.is_file()
    assert find_root(anchors=[imported]) == ws.resolve()
    assert find_root(anchors=[imported.resolve()]) == ws.resolve()


def test_an_install_outside_the_workspace_is_a_clear_error(tmp_path, monkeypatch):
    monkeypatch.delenv("F1TENTH_MORE_ROOT", raising=False)
    elsewhere = tmp_path / "opt" / "f1tenth_more" / "robot_logger.py"
    elsewhere.parent.mkdir(parents=True)
    elsewhere.write_text("")
    with pytest.raises(RuntimeError, match="F1TENTH_MORE_ROOT"):
        find_root(anchors=[elsewhere])


def test_an_empty_root_parameter_means_not_given(monkeypatch, root):
    monkeypatch.setenv("F1TENTH_MORE_ROOT", str(root))
    assert find_root("") == root.resolve()


def test_the_environment_variable_wins(root, monkeypatch):
    monkeypatch.setenv("F1TENTH_MORE_ROOT", str(root))
    assert find_root() == root.resolve()


def test_a_bad_environment_variable_is_a_clear_error(monkeypatch):
    monkeypatch.setenv("F1TENTH_MORE_ROOT", "/definitely/not/here")
    with pytest.raises(NotADirectoryError):
        find_root()


def test_the_working_directory_is_never_consulted(root, monkeypatch, tmp_path):
    """ros2 run and ros2 launch both change it; the logger must not care."""
    monkeypatch.delenv("F1TENTH_MORE_ROOT", raising=False)
    monkeypatch.chdir(tmp_path)
    assert find_root() == find_root(None)
    assert find_root().name == "f1tenth_more"
    assert os.getcwd() != str(find_root())
