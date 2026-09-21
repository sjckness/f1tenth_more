"""test_campaign_logger against real DDS traffic, every branch of the lifecycle.

Starts the node as a subprocess and publishes odometry, IMU, drive commands,
MPC status, clearance and safety events at realistic rates, then walks it
through the whole lifecycle once in a module-scoped fixture. Each test below
asserts on one branch of that single run -- restarting the node per assertion
would turn a 40 s suite into a 5 minute one for no extra coverage.

Isolation is set up in conftest.py: a private ROS_DOMAIN_ID with
localhost-only discovery and no discovery server, applied before rclpy is
imported. This cannot reach the car's stack and the car's stack cannot reach
it. Skipped entirely when rclpy is missing.
"""

from __future__ import annotations

import csv
import json
import math
import random
import signal
import subprocess
import sys
import time

import pytest
import yaml

from conftest import requires_rclpy

rclpy = pytest.importorskip("rclpy")
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Float32, String  # noqa: E402

from f1tenth_logger.test_campaign import export_campaign_csv as exp  # noqa: E402
from f1tenth_logger.test_campaign.robot_logger import parse_test_id  # noqa: E402

pytestmark = requires_rclpy

CAMPAIGN = "first_test_campaing"
POST_ROLL_S = 0.5
LONG_TIMEOUT_S = 25.0     # the normal scenarios must never trip this
SHORT_TIMEOUT_S = 3.0     # the node is restarted with this to test the timeout
PRE_ROLL_S = 1.0
IMU_HZ, ODOM_HZ, DRIVE_HZ, MPC_HZ, CLEAR_HZ = 80.0, 40.0, 20.0, 20.0, 10.0

PROMPTS = [
    {"prompt_num": 0, "mission": "M00_calibration",
     "text": "Stand still with the motor on, then push the car forward by hand.",
     "success_criterion": "the log shows a clean standstill then a push"},
    {"prompt_num": 1, "mission": "M01_corridor",
     "text": "Drive down the corridor and stop at the red box.",
     "success_criterion": "stops at the box, inside the corridor"},
    {"prompt_num": 2, "mission": "M02_bend",
     "text": "Follow the corridor around the bend.",
     "success_criterion": "reaches the end without touching a wall"},
]


class Fake(Node):
    """Every publisher the logger subscribes to, at realistic rates."""

    def __init__(self):
        super().__init__("fake_stack")
        sensor = QoSProfile(depth=50)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        reliable = QoSProfile(depth=10)
        self.plan = self.create_publisher(String, "/test/plan_result", reliable)
        self.event = self.create_publisher(String, "/test/mission_event", reliable)
        self.corridor = self.create_publisher(String, "/corridor", reliable)
        self.safety = self.create_publisher(String, "/safety/event", reliable)
        self.odom = self.create_publisher(Odometry, "/odom", sensor)
        self.imu = self.create_publisher(Imu, "/sensors/imu/raw", sensor)
        self.drive = self.create_publisher(AckermannDriveStamped, "/drive", sensor)
        self.mpc = self.create_publisher(String, "/mpc/status", sensor)
        self.clear = self.create_publisher(Float32, "/obstacle_clearance", sensor)
        self.statuses = []
        self.create_subscription(
            String, "/test_campaign/logger_status",
            lambda msg: self.statuses.append(json.loads(msg.data)), reliable)
        self.x = self.y = self.yaw = 0.0
        self.rng = random.Random(4)

    def stamp(self):
        return self.get_clock().now().to_msg()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, publisher, payload):
        publisher.publish(String(data=json.dumps(payload)))
        rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_logger(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (self.plan.get_subscription_count() > 0
                    and self.odom.get_subscription_count() > 0
                    and self.event.get_subscription_count() > 0):
                deadline = time.monotonic() + 0.3
                while time.monotonic() < deadline:   # let the rest match
                    rclpy.spin_once(self, timeout_sec=0.01)
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def stream(self, seconds, speed=0.0, steer=0.0):
        """Publish every sensor for ``seconds``; speed 0 is a standstill."""
        dt = 1.0 / IMU_HZ
        for step in range(max(1, int(seconds * IMU_HZ))):
            started = time.monotonic()
            self.yaw += steer * speed * dt
            self.x += speed * math.cos(self.yaw) * dt
            self.y += speed * math.sin(self.yaw) * dt

            imu = Imu()
            imu.header.stamp = self.stamp()
            imu.header.frame_id = "imu_link"
            # a real IMU is never silent, so the standstill floor must not be
            # a trivially perfect zero
            imu.linear_acceleration.x = ((0.3 if speed else 0.0)
                                         + self.rng.gauss(0.0, 0.02))
            imu.linear_acceleration.y = ((0.2 * math.sin(step * 0.4) if speed else 0.0)
                                         + self.rng.gauss(0.0, 0.02))
            imu.linear_acceleration.z = 9.81 + self.rng.gauss(0.0, 0.03)
            imu.angular_velocity.z = steer * speed
            self.imu.publish(imu)

            if step % int(IMU_HZ / ODOM_HZ) == 0:
                odom = Odometry()
                odom.header.stamp = self.stamp()
                odom.header.frame_id = "odom"
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
                odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
                odom.twist.twist.linear.x = speed
                odom.twist.twist.angular.z = steer * speed
                self.odom.publish(odom)
            if step % int(IMU_HZ / DRIVE_HZ) == 0:
                cmd = AckermannDriveStamped()
                cmd.header.stamp = self.stamp()
                cmd.drive.speed = speed
                cmd.drive.steering_angle = steer
                self.drive.publish(cmd)
            if step % int(IMU_HZ / MPC_HZ) == 0:
                status = "infeasible" if (speed and 40 <= step < 46) else "solved"
                self.mpc.publish(String(data=json.dumps(
                    {"status": status, "solve_time_ms": 11.5, "cost": 2.0,
                     "iterations": 9})))
            if step % int(IMU_HZ / CLEAR_HZ) == 0:
                self.clear.publish(Float32(data=1.5 if speed else 3.0))

            rclpy.spin_once(self, timeout_sec=0.0)
            slack = dt - (time.monotonic() - started)
            if slack > 0:
                time.sleep(slack)

    def plan_result(self, prompt_num, kind="initial", status="ok", error="",
                    sent_ago=0.4, plan_id=None):
        now = self.now()
        text = next(p["text"] for p in PROMPTS if p["prompt_num"] == prompt_num)
        self.send(self.plan, {
            "prompt_num": prompt_num,
            "prompt_text": text,
            "kind": kind,
            "t_prompt_sent": now - sent_ago,
            "t_response_received": now,
            "latency_ms": sent_ago * 1000.0,
            "status": status,
            "error": error,
            "plan_id": plan_id or f"plan_{prompt_num}_{kind}",
            "plan": None if status != "ok" else {"moves": [{"type": "straight"}]},
        })

    def mission(self, event, **fields):
        self.send(self.event, {"event": event, **fields})


def start_node(root, timeout_s=LONG_TIMEOUT_S):
    # odom_topic pinned to /odom: the node now defaults (and its packaged
    # yaml, which it loads by itself here) to /odometry/filtered, and the
    # corridor-frame warning below needs poses from a different estimate.
    return subprocess.Popen(
        [sys.executable, "-m", "f1tenth_logger.test_campaign.logger_node", "--ros-args",
         "-p", "odom_topic:=/odom",
         "-p", f"root:={root}",
         "-p", "status_period_s:=1.0",
         "-p", f"campaign:={CAMPAIGN}",
         "-p", "robot_radius:=0.3",
         "-p", f"post_roll_s:={POST_ROLL_S}",
         "-p", f"max_test_duration_s:={timeout_s}",
         "-p", f"pre_roll_s:={PRE_ROLL_S}"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )


def _tests_in(campaign, mission):
    folder = campaign / mission
    if not folder.is_dir():
        return []
    found = []
    for child in sorted(folder.iterdir()):
        if not child.is_dir():
            continue
        try:
            parse_test_id(child.name)
        except ValueError:
            continue
        found.append(child)
    return found


def wait_for_meta(test_dir, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if (test_dir / "meta.json").exists():
            time.sleep(0.2)
            return json.loads((test_dir / "meta.json").read_text())
        time.sleep(0.1)
    return None


def wait_for_new_test(campaign, mission, known, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        new = [d for d in _tests_in(campaign, mission) if d not in known]
        if new:
            return new[0]
        time.sleep(0.1)
    return None


def read_events(test_dir):
    path = test_dir / "events.jsonl"
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    """One node, every branch of the lifecycle, run once."""
    root = tmp_path_factory.mktemp("node") / "f1tenth_more"
    campaign = root / CAMPAIGN
    campaign.mkdir(parents=True)
    with open(campaign / "prompts.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(PROMPTS, fh, sort_keys=False)

    node_proc = start_node(root)
    rclpy.init()
    fake = Fake()
    out = {"campaign": campaign}
    try:
        if not fake.wait_for_logger():
            node_proc.terminate()
            pytest.fail("the logger node never subscribed:\n"
                        + node_proc.communicate(timeout=5)[0])

        # 1. the real trigger, end to end, on the calibration prompt
        before = _tests_in(campaign, "M00_calibration")
        trigger = subprocess.Popen(
            [sys.executable, "-m", "f1tenth_logger.test_campaign.trigger", "0",
             "--root", str(root), "--campaign", CAMPAIGN, "--countdown", "2.5"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True)
        fake.stream(3.0, speed=0.0)            # the countdown, standing still
        fake.stream(2.0, speed=0.6)            # the push by hand
        trigger.communicate(input="\n", timeout=15)
        fake.stream(POST_ROLL_S + 0.5)
        calib = wait_for_new_test(campaign, "M00_calibration", before)
        out["calib"] = calib
        out["calib_meta"] = wait_for_meta(calib) if calib else None
        out["calib_count"] = len(_tests_in(campaign, "M00_calibration"))

        # 2. a normal test: initial, countdown, drive, replan, finish
        before = _tests_in(campaign, "M01_corridor")
        fake.stream(PRE_ROLL_S)                # fills the pre-roll ring buffer
        fake.plan_result(1, sent_ago=0.5)
        drive = wait_for_new_test(campaign, "M01_corridor", before)
        out["drive"] = drive
        fake.mission("mission_loaded", plan_id="plan_1_initial", countdown_s=2.4)
        fake.stream(2.4, speed=0.0)            # countdown: the noise floor
        fake.mission("mission_started", plan_id="plan_1_initial")
        fake.stream(1.5, speed=0.8, steer=0.15)
        fake.plan_result(1, kind="replan", sent_ago=0.2)
        fake.send(fake.corridor, {"id": 7, "width": 2.0,
                                  "odom_topic": "/odometry/filtered",
                                  "polygon": [[-1, -1], [9, -1], [9, 1], [-1, 1]]})
        fake.send(fake.safety, {"event": "contact", "cause": "clipped a cone"})
        # a mission event from some other plan must be flagged, not believed
        fake.mission("mission_loaded", plan_id="a-different-plan")
        fake.stream(1.5, speed=0.8, steer=-0.15)
        fake.mission("mission_finished", plan_id="plan_1_initial",
                     reason="reached the box")
        fake.stream(POST_ROLL_S + 0.5)
        out["drive_meta"] = wait_for_meta(drive)
        out["drive_count"] = len(_tests_in(campaign, "M01_corridor"))

        # 3. an LLM failure still produces a test
        before = _tests_in(campaign, "M02_bend")
        fake.plan_result(2, status="error", error="llama-server timed out")
        failed = wait_for_new_test(campaign, "M02_bend", before)
        out["failed"] = failed
        out["failed_meta"] = wait_for_meta(failed) if failed else None

        # 4. a second 'initial' while a test is open
        before = _tests_in(campaign, "M01_corridor")
        fake.plan_result(1)
        first = wait_for_new_test(campaign, "M01_corridor", before)
        fake.stream(0.4)
        before2 = _tests_in(campaign, "M01_corridor")
        fake.plan_result(1)
        second = wait_for_new_test(campaign, "M01_corridor", before2)
        out["superseded_meta"] = wait_for_meta(first)
        out["superseded_is_a_new_folder"] = second is not None and second != first

        # 5. abort during the countdown
        fake.mission("mission_loaded", plan_id="plan_1_initial", countdown_s=3.0)
        fake.stream(0.5)
        fake.mission("mission_aborted", plan_id="plan_1_initial",
                     reason="operator hit stop")
        fake.stream(POST_ROLL_S + 0.5)
        out["cancelled_meta"] = wait_for_meta(second)

        # 6. restart with a short timeout, then send no end message at all
        node_proc.send_signal(signal.SIGINT)
        out["first_output"] = node_proc.communicate(timeout=10)[0]
        out["statuses"] = list(fake.statuses)
        node_proc = start_node(root, timeout_s=SHORT_TIMEOUT_S)
        out["restarted"] = fake.wait_for_logger()
        before = _tests_in(campaign, "M02_bend")
        fake.plan_result(2)
        stale = wait_for_new_test(campaign, "M02_bend", before)
        fake.stream(SHORT_TIMEOUT_S + 1.5, speed=0.3)
        out["stale"] = stale
        out["stale_meta"] = wait_for_meta(stale)

        # 7. SIGINT with a test open
        before = _tests_in(campaign, "M01_corridor")
        fake.plan_result(1)
        open_test = wait_for_new_test(campaign, "M01_corridor", before)
        fake.mission("mission_loaded", plan_id="plan_1_initial", countdown_s=1.0)
        fake.stream(1.0, speed=0.5)
        out["open_before_sigint"] = not (open_test / "meta.json").exists()
        node_proc.send_signal(signal.SIGINT)
        try:
            out["node_output"] = node_proc.communicate(timeout=10)[0]
        except subprocess.TimeoutExpired:
            node_proc.kill()
            out["node_output"] = node_proc.communicate()[0]
        node_proc = None
        out["shutdown_meta"] = wait_for_meta(open_test, timeout=5.0)

        # 8. the export
        assert exp.main([str(campaign), "--plain"]) == 0
        with open(campaign / exp.RESULTS_NAME, newline="", encoding="utf-8") as fh:
            out["rows"] = {r["test_id"]: r for r in csv.DictReader(fh)}
        out["n_folders"] = sum(len(_tests_in(campaign, p["mission"])) for p in PROMPTS)
        yield out
    finally:
        try:
            fake.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        if node_proc is not None:
            node_proc.send_signal(signal.SIGINT)
            try:
                node_proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                node_proc.kill()


# --------------------------------------------------------------------------
# the trigger
# --------------------------------------------------------------------------

def test_the_trigger_produces_exactly_one_completed_test(session):
    assert session["calib"] is not None
    assert session["calib_count"] == 1
    assert session["calib"].parent.name == "M00_calibration"
    assert session["calib_meta"]["auto_outcome"] == "completed"


# --------------------------------------------------------------------------
# a normal test
# --------------------------------------------------------------------------

def test_an_initial_plan_result_opens_a_test_and_finishing_closes_it(session):
    assert session["drive"] is not None
    assert session["drive_meta"]["auto_outcome"] == "completed"
    assert session["drive_meta"]["reason"] == "reached the box"


def test_a_replan_joins_the_open_test_instead_of_starting_one(session):
    assert session["drive_count"] == 1
    calls = list(csv.DictReader(open(session["drive"] / "llm_calls.csv")))
    assert [c["tag"] for c in calls] == ["initial", "replan"]


def test_the_initial_prompt_was_sent_before_the_test_existed(session):
    """t=0 is the test folder, so the prompt that caused it is negative."""
    calls = list(csv.DictReader(open(session["drive"] / "llm_calls.csv")))
    assert float(calls[0]["t_sent"]) < 0.0
    assert float(calls[1]["t_sent"]) > 0.0


def test_every_stream_was_recorded_at_its_own_rate(session):
    summary = session["drive_meta"]["summary"]
    for name, published, got in (("imu", IMU_HZ, summary["imu_rate_hz"]),
                                 ("pose", ODOM_HZ, summary["pose_rate_hz"]),
                                 ("command", DRIVE_HZ, summary["cmd_rate_hz"])):
        assert got is not None, name
        assert 0.75 * published <= got <= 1.15 * published, f"{name}: {got} Hz"


def test_odometry_was_decoded_into_poses(session):
    rows = list(csv.DictReader(open(session["drive"] / "kinematics.csv")))
    assert len(rows) > 100
    assert any(abs(float(r["yaw"] or 0)) > 0.01 for r in rows), "quaternion -> yaw"
    assert any(r["obstacle_clearance"] for r in rows)


def test_the_pre_roll_wrote_samples_from_before_the_test_opened(session):
    rows = list(csv.DictReader(open(session["drive"] / "imu.csv")))
    assert min(float(r["t"]) for r in rows) < 0.0
    assert "pre_roll" in [e["event"] for e in read_events(session["drive"])]


def test_the_mission_and_safety_events_were_recorded(session):
    names = [e["event"] for e in read_events(session["drive"])]
    for needed in ("mission_loaded", "mission_started", "mission_finished", "contact"):
        assert needed in names, needed
    assert len((session["drive"] / "corridors.jsonl").read_text().splitlines()) == 1


def test_mpc_solves_were_recorded(session):
    assert len(list(csv.DictReader(open(session["drive"] / "mpc.csv")))) > 80


# --------------------------------------------------------------------------
# plan_id
# --------------------------------------------------------------------------

def test_the_plan_id_reaches_the_test_folder(session):
    """The planner's plan_id, the mission events' plan_id and the test agree."""
    meta = session["drive_meta"]
    assert meta["extra_meta"]["plan_id"] == "plan_1_initial"
    loaded = [e for e in read_events(session["drive"])
              if e["event"] == "mission_loaded"]
    assert loaded and loaded[0]["plan_id"] == "plan_1_initial"


def test_an_event_from_another_plan_is_flagged(session):
    mismatch = [e for e in read_events(session["drive"])
                if e["event"] == "plan_id_mismatch"]
    assert len(mismatch) == 1
    assert mismatch[0]["expected"] == "plan_1_initial"
    assert mismatch[0]["got"] == "a-different-plan"


# --------------------------------------------------------------------------
# the failure branches
# --------------------------------------------------------------------------

def test_a_failed_llm_call_still_produces_a_test(session):
    assert session["failed"] is not None
    assert session["failed_meta"]["auto_outcome"] == "aborted"
    assert "llama-server timed out" in session["failed_meta"]["reason"]
    row = list(csv.DictReader(open(session["failed"] / "llm_calls.csv")))[0]
    assert row["ok"] == "0"


def test_a_second_initial_supersedes_the_open_test(session):
    assert session["superseded_meta"]["reason"] == "superseded by new test"
    assert session["superseded_is_a_new_folder"]


def test_an_abort_before_the_start_is_cancelled_before_start(session):
    assert session["cancelled_meta"]["auto_outcome"] == "aborted"
    assert session["cancelled_meta"]["reason"] == "cancelled before start"


def test_a_test_with_no_end_message_times_out(session):
    assert session["restarted"], "the node did not come back up"
    assert session["stale_meta"]["auto_outcome"] == "aborted"
    assert session["stale_meta"]["reason"] == "timeout: no end message"


def test_ctrl_c_closes_the_open_test_cleanly(session):
    assert session["open_before_sigint"], "the test should still have been open"
    assert session["shutdown_meta"]["auto_outcome"] == "aborted"
    assert session["shutdown_meta"]["reason"] == "logger node shut down"
    assert "Traceback" not in (session["node_output"] or "")


# --------------------------------------------------------------------------
# the export over the driving window
# --------------------------------------------------------------------------

def test_the_export_covers_every_test_the_node_opened(session):
    assert len(session["rows"]) == session["n_folders"]


def test_the_driving_window_matches_what_was_published(session):
    row = session["rows"][session["drive"].name]
    assert 2.5 < float(row["drive_duration_s"]) < 4.5
    assert float(row["countdown_s"]) == pytest.approx(2.4, abs=0.3)
    assert float(row["llm_latency_ms"]) == pytest.approx(500.0, abs=1.0)
    assert row["n_replans"] == "1"


def test_the_standstill_floor_is_below_the_driving_jerk(session):
    row = session["rows"][session["drive"].name]
    floor = float(row["standstill_jerk_rms"])
    assert 0.0 < floor < 1.0
    assert float(row["jerk_rms"]) > 2 * floor


def test_contact_and_feasibility_reached_the_export(session):
    row = session["rows"][session["drive"].name]
    assert row["contact"] == "1"
    assert float(row["min_clear_m"]) == 0.0
    assert 0.0 < float(row["feas_pct"]) < 100.0


def test_a_test_that_never_started_has_no_driving_metrics(session):
    row = session["rows"][session["stale"].name]
    assert row["viol_rate_pct"] == "" and row["jerk_rms"] == ""


# --------------------------------------------------------------------------
# visibility: startup log and the status topic
# --------------------------------------------------------------------------

def test_startup_prints_the_absolute_campaign_folder_and_every_subscription(session):
    output = session["first_output"]
    assert f"campaign folder: {session['campaign']}" in output
    assert session["campaign"].is_absolute()
    for line in ("subscribed /odom (Odometry) best_effort depth 50",
                 "subscribed /test/plan_result (String) reliable depth 10",
                 "subscribed /safety/event (String) reliable depth 10",
                 "subscribed /obstacle_clearance (Float32) best_effort depth 50"):
        assert line in output, line


def test_an_open_test_publishes_status_lines_and_a_closing_one(session):
    drive_id = session["drive"].name
    mine = [s for s in session["statuses"] if s["test_id"] == drive_id]
    opened = [s for s in mine if s["state"] == "open"]
    closed = [s for s in mine if s["state"] == "closed"]
    # the drive test is open for ~6 s at status_period_s 1.0
    assert len(opened) >= 3
    assert opened[-1]["seconds"] > opened[0]["seconds"]
    assert opened[-1]["samples"]["imu"] > opened[0]["samples"]["imu"] > 0
    assert opened[-1]["samples"]["kinematics"] > 0
    assert opened[-1]["samples"]["obstacle_clearance"] > 0
    assert len(closed) == 1 and closed[0]["outcome"] == "completed"
    assert closed[0]["mission_started"] is True
    assert closed[0]["campaign_dir"] == str(session["campaign"])


def test_a_corridor_from_another_pose_estimate_is_warned_about(session):
    assert "corridors are built from /odometry/filtered but poses come from /odom" \
        in session["first_output"]
