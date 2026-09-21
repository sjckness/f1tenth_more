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

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import analyze_tests  # noqa: E402
from robot_logger import (  # noqa: E402
    TestLogger,
    corridor_from_centerline,
    find_root,
    make_test_id,
    parse_test_id,
    signed_clearance,
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
        f"import sys; sys.path.insert(0, {str(HERE)!r})\n"
        "from robot_logger import TestLogger\n"
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
        f"import sys; sys.path.insert(0, {str(HERE)!r})\n"
        "from robot_logger import TestLogger\n"
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

def test_the_upward_search_finds_the_project(monkeypatch):
    monkeypatch.delenv("F1TENTH_MORE_ROOT", raising=False)
    found = find_root()
    assert found.name == "f1tenth_more" and (found / "tools").is_dir()


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
