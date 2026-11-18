"""auto_mode: /mission/status opens and closes tests (auto_trigger.py).

The decision table is checked as pure logic; then one in-process node (private
domain from conftest.py, no traffic needed) is fed a fake status sequence
straight into apply_mission_status, against a temp campaign root.
"""

from __future__ import annotations

import json
import time

import pytest
import yaml

from f1tenth_logger.test_campaign.auto_trigger import AutoTrigger, mission_id

JSON = "/opt/f1tenth_ws/install/share/f1tenth_behavior/missions/straight_2m_then_left_door.json"


def test_mission_id_is_the_json_stem():
    assert mission_id(JSON) == "straight_2m_then_left_door"


def test_decision_table():
    a = AutoTrigger()
    assert a.decide("LOADED", JSON, False, False) == []            # load: nothing
    assert a.decide("LOADED", JSON, False, False) == []            # repeat
    assert a.decide("RUNNING", JSON, False, False) == [
        ("open", "straight_2m_then_left_door")]
    assert a.decide("RUNNING", JSON, False, True) == []            # repeat
    assert a.decide("HOLDING", JSON, False, True) == []
    assert a.decide("RUNNING", JSON, False, True) == []
    assert a.decide("COMPLETE", JSON, False, True) == [("close", "COMPLETE")]
    assert a.decide("COMPLETE", JSON, False, False) == []          # repeat
    assert a.decide("RUNNING", JSON, False, False) == [
        ("open", "straight_2m_then_left_door")]
    assert a.decide("ABORTED", JSON, False, True) == [("close", "ABORTED")]
    assert a.decide("RUNNING", JSON, False, False)[0][0] == "open"
    assert a.decide("RUNNING", JSON, True, True) == [("close", "EMERGENCY_STOP")]
    assert a.decide("ABORTED", JSON, True, False) == []


def test_start_while_open_interrupts():
    a = AutoTrigger()
    a.decide("RUNNING", JSON, False, False)
    a.decide("LOADED", JSON, False, True)       # reload mid-run closes it ...
    a2 = AutoTrigger()
    a2._last = ("COMPLETE", JSON, False)        # ... and a start with one open
    assert a2.decide("RUNNING", JSON, False, True) == [
        ("close", "INTERRUPTED"), ("open", "straight_2m_then_left_door")]


def _metas(campaign):
    out = {}
    for meta in sorted(campaign.glob("D01_*/P001-R*/meta.json")):
        out[meta.parent.name.split("-")[1]] = json.loads(meta.read_text())
    return out


def _events(test_dir):
    path = test_dir / "events.jsonl"
    return [json.loads(x)["event"] for x in path.read_text().splitlines() if x.strip()]


def test_node_fed_a_fake_status_sequence(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("f1tenth_messages.msg")
    from f1tenth_logger.test_campaign.logger_node import TestLoggerNode

    campaign = tmp_path / "doors_campaign"
    campaign.mkdir()
    (campaign / "prompts.yaml").write_text(yaml.safe_dump([{
        "prompt_num": 1, "mission": "D01_straight_2m_then_left_door",
        "text": "straight_2m_then_left_door", "success_criterion": "x"}]))
    rclpy.init(args=["--ros-args", "-p", f"root:={tmp_path}",
                     "-p", "campaign:=doors_campaign", "-p", "auto_mode:=true",
                     "-p", "post_roll_s:=0.0", "-p", "pre_roll_s:=0.0"])
    node = TestLoggerNode()
    try:
        def feed(state, estop=False):
            node.apply_mission_status(state, JSON, estop)
            time.sleep(0.01)
            node._tick()                      # post_roll 0: pending close lands

        feed("LOADED")                        # load without start: nothing
        assert not list(campaign.glob("D01_*/P001-R*"))
        for state in ("RUNNING", "RUNNING", "COMPLETE", "COMPLETE"):     # R001
            feed(state)
        for state in ("LOADED", "RUNNING", "ABORTED"):                   # R002
            feed(state)
        feed("RUNNING")                                                  # R003 ...
        node._auto._last = ("COMPLETE", JSON, False)
        feed("RUNNING")                       # ... interrupted by R004
        feed("RUNNING", estop=True)           # R004: e-stop
    finally:
        node.close_open_test()
        node.destroy_node()
        rclpy.shutdown()

    metas = _metas(campaign)
    assert sorted(metas) == ["R001", "R002", "R003", "R004"]
    got = {r: (m["auto_outcome"], m["extra_meta"]["mission_outcome"]) for r, m in metas.items()}
    assert got == {
        "R001": ("completed", "COMPLETE"),
        "R002": ("aborted", "ABORTED"),
        "R003": ("aborted", "INTERRUPTED"),
        "R004": ("aborted", "EMERGENCY_STOP"),
    }
    r1 = next(campaign.glob("D01_*/P001-R001-*"))
    lifecycle = [e for e in _events(r1) if e.startswith("mission_")]
    assert lifecycle == ["mission_started", "mission_finished"]


def test_meta_records_the_planner_mode_actually_running(tmp_path):
    """meta.json's planner_controller_mode comes from the live controller node.

    As in mission_logger_node: unknown at open (none discovered yet), retried
    at close, where corridor_rrt is the one alive.
    """
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("f1tenth_messages.msg")
    from f1tenth_logger.test_campaign.logger_node import TestLoggerNode

    campaign = tmp_path / "doors_campaign"
    campaign.mkdir()
    (campaign / "prompts.yaml").write_text(yaml.safe_dump([{
        "prompt_num": 1, "mission": "D01_straight_2m_then_left_door",
        "text": "straight_2m_then_left_door", "success_criterion": "x"}]))
    rclpy.init(args=["--ros-args", "-p", f"root:={tmp_path}",
                     "-p", "campaign:=doors_campaign", "-p", "auto_mode:=true",
                     "-p", "post_roll_s:=0.0", "-p", "pre_roll_s:=0.0"])
    node = TestLoggerNode()
    alive = []
    node.get_node_names_and_namespaces = lambda: [(n, "/") for n in alive + ["other"]]
    node.get_publishers_info_by_topic = lambda topic: []
    try:
        node.apply_mission_status("RUNNING", JSON, False)
        assert node._test.extra_meta["planner_controller_mode"] == "unknown"
        alive.append("corridor_rrt")
        node.apply_mission_status("COMPLETE", JSON, False)
        time.sleep(0.01)
        node._tick()
    finally:
        node.close_open_test()
        node.destroy_node()
        rclpy.shutdown()

    meta = _metas(campaign)["R001"]
    assert meta["extra_meta"]["planner_controller_mode"] == "corridor_rrt"
