#!/usr/bin/env python3
"""The campaign recorder: one long-lived rclpy node, one TestLogger per test.

    ros2 launch f1tenth_logger test_campaign_logger.launch.py

(parameters in config/test_campaign_logger.yaml; the launch file also pins
``root`` to the workspace). Node name ``test_campaign_logger``. Never started
by any bringup: leave it running in its own terminal for the whole session. It opens a test
when the planner announces an LLM result and closes it when the mission ends,
so tests accumulate without ever restarting the node.

PARAMETERS. ``ros2 run f1tenth_logger test_campaign_logger`` with no
``--params-file`` loads the packaged config/test_campaign_logger.yaml itself,
ahead of any ``-p`` given on the command line (which still wins). Before
that, a bare ``ros2 run`` -- how the 2026-09-21 session was started -- ran
on the code defaults: poses from raw /odom, robot_radius 0, no pre-roll. The
startup log names the file that was applied.

**It never calls the LLM.** The planner owns the call and publishes what it
measured; this node only records. Everything it knows arrives as messages:

  /test/plan_result    std_msgs/String, JSON, after EVERY LLM call
                       {prompt_num, prompt_text, kind: initial|replan,
                        t_prompt_sent, t_response_received, latency_ms,
                        status: ok|error, error, plan_id, translated_plan,
                        plan (deprecated alias), mission_path, llm_raw,
                        rejections}
                       and, when load/start fails after it,
                       {kind: delivery, status: error, stage, error, plan_id}
  /test/mission_event  std_msgs/String, JSON
                       {event: mission_loaded|mission_started|
                               mission_finished|mission_aborted,
                        plan_id, reason, countdown_s}

The planner publishes the initial plan_result BEFORE it aborts the previous
mission and loads the new one, so the test is open when mission_loaded
arrives.

Lifecycle, in one place:

* ``initial`` + ``ok``      -> open a test, save the plan, record the call.
* ``initial`` + ``error``   -> open a test anyway, record the call, close it
                               as aborted. A failed LLM call is a test.
* ``initial`` while open    -> close the open one as "superseded by new test".
* ``replan``                -> another row in the open test; never a new one.
* ``delivery`` (error)      -> load/start failed: recorded as
                               ``delivery_failed``, closed after the post-roll
                               as "<stage> failed: <error>".
* mission_finished/aborted  -> keep recording for ``post_roll_s``, then close.
* aborted before started    -> "cancelled before start".
* finished/aborted before this test's own mission_loaded/started
                            -> the previous mission, ended by the planner's
                               abort: recorded as ``stale_mission_event``,
                               acted on never.
* no /obstacle_clearance for ``clearance_grace_s`` after mission_started
                            -> an ERROR in the log and a
                               ``no_obstacle_clearance`` event, once per test.
* nothing for max_test_duration_s -> "timeout: no end message".
* Ctrl+C                    -> "logger node shut down", files closed cleanly.

plan_id: the planner puts its plan_id inside the plan, the mission node
echoes it on every mission event, and this node checks the two match. A
mismatch is warned about and written to events.jsonl as ``plan_id_mismatch``;
the event is still acted on, because a stalled test that times out loses more
than a mislabelled one that is flagged. The one exception is the stale end
event above, which is ignored whatever its plan_id: repetitions of one prompt
reuse the same plan_id, so the id cannot tell that event apart.

Time base: t=0 is the moment the test folder is created, so the prompt that
started it has a negative ``t_sent``. Sample times come from the message
header where there is one, and from the node clock otherwise.

QoS: the high-rate sensor streams are subscribed BEST_EFFORT, which also
matches RELIABLE publishers, so a sensor publisher can never fail to connect;
the low-rate control topics are RELIABLE so a mission event cannot be dropped.
Set ``best_effort_sensors:=false`` if a publisher needs the opposite.

Visibility: at startup it prints the absolute campaign folder and every
subscription with its QoS; it refuses to start if the workspace root cannot
be found, rather than writing a campaign somewhere unexpected. While a test
is open it logs one status line every ``status_period_s`` (10 s) -- test id,
seconds recorded, rows per stream -- and publishes the same as JSON on
``/test_campaign/logger_status`` (std_msgs/String), plus a final one when the
test closes. That is the only topic this node publishes.
"""

from __future__ import annotations

import json
import math
import signal
import sys
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from f1tenth_logger.test_campaign.robot_logger import (
    DEFAULT_CAMPAIGN, TestLogger, find_root)

try:
    from nav_msgs.msg import Odometry
except ImportError:  # pragma: no cover - a ROS install without nav_msgs
    Odometry = None
try:
    from sensor_msgs.msg import Imu
except ImportError:  # pragma: no cover
    Imu = None
try:
    from ackermann_msgs.msg import AckermannDriveStamped
except ImportError:  # pragma: no cover - ackermann_msgs is a separate package
    AckermannDriveStamped = None
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - not a sourced ROS environment
    get_package_share_directory = None

STAMP_SANITY_S = 5.0  # header stamps further than this from the clock are ignored
NODE_NAME = "test_campaign_logger"
STATUS_TOPIC = "/test_campaign/logger_status"

#: The mission lifecycle this node understands. mission/loader.py's _TEST_EVENTS
#: publishes exactly these four; f1tenth_behavior's test_mission_countdown.py
#: asserts the two lists stay equal, so neither side can rename an event alone.
MISSION_EVENTS = (
    "mission_loaded", "mission_started", "mission_finished", "mission_aborted",
)
CONFIG_NAME = "test_campaign_logger.yaml"


def _share_file(package, *parts):
    """A file in ``package``'s share directory, or None if there is none."""
    if get_package_share_directory is None:
        return None
    try:
        path = Path(get_package_share_directory(package)).joinpath(*parts)
    except Exception:  # noqa: BLE001 - PackageNotFoundError, or no index at all
        return None
    return path if path.is_file() else None


def with_packaged_params(args):
    """``(args, params_file)``: the packaged config in front, unless one was given.

    Placed first, in a ``--ros-args ... --`` section of its own, so every
    ``-p`` and ``--params-file`` already on the command line is parsed after
    it and still overrides it.
    """
    args = list(args)
    if "--params-file" in args:
        return args, None
    path = _share_file("f1tenth_logger", "config", CONFIG_NAME)
    if path is None:
        return args, None
    return args[:1] + ["--ros-args", "--params-file", str(path), "--"] + args[1:], path


def yaw_from_quaternion(x, y, z, w):
    """Yaw only, counter-clockwise positive, from a ROS quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class TestLoggerNode(Node):
    """Owns at most one open :class:`TestLogger` at a time."""

    def __init__(self, params_file=None):
        super().__init__(NODE_NAME)
        self._params_file = params_file   # the packaged yaml main() injected

        def param(name, default):
            return self.declare_parameter(name, default).value

        self._campaign = param("campaign", DEFAULT_CAMPAIGN)
        # Resolved once, here, and handed to every TestLogger: the folder a
        # campaign lands in must not depend on which test happens to open
        # first. find_root raises with its own explanation if there is none.
        self._root = find_root(param("root", "") or None)
        self._campaign_dir = self._root / str(self._campaign)
        self._robot_radius = float(param("robot_radius", 0.0))
        self._mpc_ok_statuses = tuple(param("mpc_ok_statuses", ["solved"]))
        self._robot_name = param("robot_name", "")
        self._llm_model = param("llm_model", "")

        self._plan_topic = param("plan_result_topic", "/test/plan_result")
        self._event_topic = param("mission_event_topic", "/test/mission_event")
        # Same as the yaml: /corridor is built from the local EKF, and raw
        # /odom drifts ~19 deg of yaw over 15 m.
        self._odom_topic = param("odom_topic", "/odometry/filtered")
        self._imu_topic = param("imu_topic", "/sensors/imu/raw")
        self._drive_topic = param("drive_topic", "/drive")
        self._mpc_topic = param("mpc_status_topic", "/mpc/status")
        self._corridor_topic = param("corridor_topic", "/corridor")
        self._clearance_topic = param("obstacle_clearance_topic",
                                      "/obstacle_clearance")
        self._safety_topic = param("safety_event_topic", "/safety/event")

        self._post_roll_s = float(param("post_roll_s", 2.0))
        self._pre_roll_s = float(param("pre_roll_s", 0.0))
        self._max_test_duration_s = float(param("max_test_duration_s", 300.0))
        self._status_period_s = float(param("status_period_s", 10.0))
        # obstacle_clearance_node publishes once per scan (40 Hz): this long
        # after mission_started with nothing from it means it is not running.
        self._clearance_grace_s = float(param("clearance_grace_s", 2.0))
        best_effort = bool(param("best_effort_sensors", True))

        # Open-test state. NOT self._logger: rclpy's Node already owns that
        # attribute and get_logger() returns it, so assigning to it silently
        # breaks every log call in the node.
        self._test = None
        self._t0 = None
        self._mission_started = False
        self._pending_close = None      # (outcome, reason, close_at)
        self._obstacle_clearance = None
        self._plan_id = ''
        self._pre_roll = deque()        # (t_ros, kind, payload)
        self._warned = set()
        self._shutting_down = False
        self._next_status = None        # node time of the next status line
        self._n_clearance = 0           # poses that carried a clearance
        self._lifecycle_seen = False    # this test's own loaded/started arrived
        self._n_clearance_driving = 0   # /obstacle_clearance msgs since started
        self._started_at = None         # node time this test's mission started
        self._clearance_warned = False
        self._subscribed = []           # (topic, type, qos) for the startup log

        reliable = QoSProfile(depth=10)
        sensor = QoSProfile(depth=50)
        if best_effort:
            sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self._subscribe(String, self._plan_topic, self._on_plan_result, reliable)
        self._subscribe(String, self._event_topic, self._on_mission_event, reliable)
        self._subscribe(String, self._mpc_topic, self._on_mpc, sensor)
        self._subscribe(String, self._corridor_topic, self._on_corridor, reliable)
        self._subscribe(String, self._safety_topic, self._on_safety, reliable)
        self._subscribe(Float32, self._clearance_topic, self._on_clearance, sensor)
        if Odometry is not None:
            self._subscribe(Odometry, self._odom_topic, self._on_odom, sensor)
        else:
            self._warn_once("nav_msgs", "nav_msgs is missing: no odometry logged")
        if Imu is not None:
            self._subscribe(Imu, self._imu_topic, self._on_imu, sensor)
        else:
            self._warn_once("sensor_msgs", "sensor_msgs is missing: no IMU logged")
        if AckermannDriveStamped is not None:
            self._subscribe(AckermannDriveStamped, self._drive_topic,
                            self._on_drive, sensor)
        else:
            self._warn_once(
                "ackermann_msgs",
                f"ackermann_msgs is not installed: {self._drive_topic} will not "
                f"be logged (everything else still works)",
            )

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_timer(0.1, self._tick)
        self._log_startup()

    def _subscribe(self, msg_type, topic, callback, qos):
        self.create_subscription(msg_type, topic, callback, qos)
        self._subscribed.append((topic, msg_type.__name__, qos))

    def _log_startup(self):
        log = self.get_logger()
        log.info(f"[test_campaign] campaign folder: {self._campaign_dir}")
        if self._params_file is not None:
            log.info(f"[test_campaign] parameters: {self._params_file} (packaged "
                     f"default, loaded because no --params-file was given)")
        log.info(
            f"[test_campaign] poses from {self._odom_topic}, robot_radius "
            f"{self._robot_radius:g} m, robot {self._robot_name or '-'}, "
            f"llm_model {self._llm_model or '-'}"
        )
        if not (self._campaign_dir / "prompts.yaml").is_file():
            log.warn(
                f"[test_campaign] no prompts.yaml in {self._campaign_dir}: every "
                f"test will be refused until it exists"
            )
        for topic, type_name, qos in self._subscribed:
            log.info(
                f"[test_campaign] subscribed {topic} ({type_name}) "
                f"{qos.reliability.name.lower()} depth {qos.depth}"
            )
        log.info(
            f"[test_campaign] publishing {STATUS_TOPIC} every "
            f"{self._status_period_s:g}s while a test is open; "
            f"post_roll={self._post_roll_s}s pre_roll={self._pre_roll_s}s "
            f"timeout={self._max_test_duration_s}s -- waiting for "
            f"{self._plan_topic}"
        )

    # -- helpers -----------------------------------------------------------

    def _warn_once(self, key, message):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(f"[test_campaign] {message}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp_seconds(self, header):
        """Header stamp in seconds, or the receive time if it is unusable."""
        now = self._now()
        try:
            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        except AttributeError:
            return now
        if stamp <= 0.0:
            return now
        if abs(stamp - now) > STAMP_SANITY_S:
            self._warn_once(
                "stamps",
                f"header stamps are {stamp - now:+.1f} s away from this node's "
                f"clock; logging receive times instead",
            )
            return now
        return stamp

    def _t(self, stamp=None):
        """A stamp in the open test's time base."""
        return (self._now() if stamp is None else stamp) - self._t0

    @staticmethod
    def _payload(msg, what, logger):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            logger.warn(f"[test_campaign] {what} is not valid JSON: {exc}")
            return None
        if not isinstance(data, dict):
            logger.warn(f"[test_campaign] {what} must be a JSON object")
            return None
        return data

    # -- test lifecycle ----------------------------------------------------

    def _open_test(self, prompt_num, prompt_text, plan_id):
        extra = {
            "robot": self._robot_name or None,
            "llm_model": self._llm_model or None,
            "plan_id": plan_id,
            "ros_start_time": self._now(),
            "source": NODE_NAME,
        }
        # prompt_num -1 (the planner's default when nobody passed --prompt-num)
        # means "resolve it from the text": prompts.yaml indexes both, and the
        # text is the thing the operator actually typed.
        try:
            key = int(prompt_num)
        except (TypeError, ValueError):
            key = None
        if key is None or key < 0:
            key = prompt_text
        try:
            logger = TestLogger(
                key,
                root=self._root,
                campaign=self._campaign,
                robot_radius=self._robot_radius,
                mpc_ok_statuses=self._mpc_ok_statuses,
                extra_meta=extra,
            )
        except Exception as exc:  # noqa: BLE001 - a bad message must not kill the node
            self.get_logger().error(
                f"[test_campaign] cannot open a test for prompt_num="
                f"{prompt_num!r} / text {str(prompt_text)[:60]!r}: "
                f"{type(exc).__name__}: {exc}"
            )
            return None

        self._test = logger
        self._t0 = self._now()
        self._mission_started = False
        self._pending_close = None
        self._obstacle_clearance = None
        self._n_clearance = 0
        self._lifecycle_seen = False
        self._n_clearance_driving = 0
        self._started_at = None
        self._clearance_warned = False
        self._plan_id = str(plan_id or "")
        self._next_status = self._t0 + self._status_period_s

        if prompt_text is not None and str(prompt_text) != logger.prompt_text:
            self.get_logger().warn(
                f"[test_campaign] {logger.test_id}: the prompt in the message is "
                f"not the one in prompts.yaml for prompt_num={prompt_num}; "
                f"recording both"
            )
            logger.log_event(
                "prompt_text_mismatch",
                t=0.0,
                from_message=str(prompt_text),
                from_table=logger.prompt_text,
            )
        self._flush_pre_roll()
        self.get_logger().info(
            f"[test_campaign] open {logger.mission}/{logger.test_id}"
        )
        return logger

    def _close_test(self, outcome, reason):
        if self._test is not None:
            # a test shorter than the grace period is still checked, once
            self._check_clearance(closing=True)
            # before the state below is reset, so the last line is the truth
            self._publish_status(self._test, state="closed", outcome=outcome)
        logger, self._test = self._test, None
        self._pending_close = None
        self._mission_started = False
        if logger is None:
            return
        summary = logger.finish(outcome, reason)
        message = (
            f"[test_campaign] close {logger.mission}/{logger.test_id} "
            f"-> {outcome} ({reason}); {summary['n_imu']} imu, "
            f"{summary['n_cmd']} cmd, {summary['n_llm_calls']} llm"
        )
        if self._shutting_down:
            # rclpy's context is already down here, so rosout would only print
            # "publisher's context is invalid" over the top of this
            print(message, flush=True)
        else:
            self.get_logger().info(message)

    def close_open_test(self, reason="logger node shut down"):
        """Called on shutdown so a live test is never left half-written."""
        self._shutting_down = True
        if self._test is not None:
            self._close_test("aborted", reason)

    def _status(self, logger, state, outcome=None):
        """The status line's content: test id, seconds recorded, rows per stream."""
        status = {
            "state": state,
            "test_id": logger.test_id,
            "mission": logger.mission,
            "seconds": round(self._now() - self._t0, 1),
            "mission_started": self._mission_started,
            "closing": self._pending_close is not None,
            "samples": {**logger.stream_counts(), "obstacle_clearance": self._n_clearance},
            "obstacle_clearance_after_start": self._n_clearance_driving,
            "campaign_dir": str(self._campaign_dir),
        }
        if outcome is not None:
            status["outcome"] = outcome
        return status

    def _publish_status(self, logger, state, outcome=None):
        if self._shutting_down:
            return
        try:
            status = self._status(logger, state, outcome)
            samples = " ".join(f"{k}={v}" for k, v in status["samples"].items())
            self.get_logger().info(
                f"[test_campaign] {state} {status['test_id']} "
                f"{status['seconds']:.0f}s: {samples}"
            )
            self._status_pub.publish(String(data=json.dumps(status)))
        except Exception as exc:  # noqa: BLE001 - a status line must not stop a test
            self._warn_once("status", f"status not published: {exc}")

    def _tick(self):
        if self._test is None:
            self._trim_pre_roll()
            return
        now = self._now()
        if self._next_status is not None and now >= self._next_status:
            self._next_status = now + self._status_period_s
            self._publish_status(self._test, state="open")
        if self._started_at is not None and not self._clearance_warned:
            self._check_clearance()
        if self._pending_close is not None:
            outcome, reason, close_at = self._pending_close
            if now >= close_at:
                self._close_test(outcome, reason)
            return
        if now - self._t0 > self._max_test_duration_s:
            self._close_test("aborted", "timeout: no end message")

    def _check_clearance(self, closing=False):
        """Say it loudly, once per test, when the clearance source is silent.

        Without /obstacle_clearance the test has no min_clear_m, and no
        contact event either: obstacle_clearance_node is the only source of
        both. A test without it looks complete and is not (every test of
        2026-09-21, when the component was still on-demand). It now starts
        with the stack, so silence here means it died or never came up.
        """
        if (self._test is None or self._clearance_warned
                or self._started_at is None or self._n_clearance_driving > 0):
            return
        waited = self._now() - self._started_at
        if not closing and waited < self._clearance_grace_s:
            return
        self._clearance_warned = True
        message = (
            f"[test_campaign] !!! {self._test.test_id}: NO {self._clearance_topic} "
            f"sample in {waited:.1f} s since mission_started -- min_clear_m, "
            f"min_clear_raw_m and contact will be EMPTY for this test. Start the "
            f"source: ./scripts/stackctl.py restart obstacle_clearance"
        )
        if self._shutting_down:
            print(message, flush=True)   # rosout is already down (see _close_test)
        else:
            self.get_logger().error(message)
        self._test.log_event(
            "no_obstacle_clearance", t=self._t(), topic=self._clearance_topic,
            seconds_since_start=round(waited, 3), at_close=closing,
        )

    # -- pre-roll ----------------------------------------------------------

    def _remember(self, kind, stamp, payload):
        if self._pre_roll_s <= 0.0:
            return
        self._pre_roll.append((stamp, kind, payload))
        self._trim_pre_roll()

    def _trim_pre_roll(self):
        if self._pre_roll_s <= 0.0:
            return
        cutoff = self._now() - self._pre_roll_s
        while self._pre_roll and self._pre_roll[0][0] < cutoff:
            self._pre_roll.popleft()

    def _flush_pre_roll(self):
        """Replay the ring buffer into the new test with negative times."""
        if not self._pre_roll:
            return
        cutoff = self._t0 - self._pre_roll_s
        replayed = 0
        for stamp, kind, payload in self._pre_roll:
            if stamp < cutoff:
                continue
            t = stamp - self._t0
            if kind == "imu":
                self._test.log_imu(**payload, t=t)
            elif kind == "state":
                self._test.log_state(**payload, t=t)
            elif kind == "command":
                self._test.log_command(**payload, t=t)
            replayed += 1
        self._pre_roll.clear()
        if replayed:
            self._test.log_event("pre_roll", t=0.0, samples=replayed,
                                 seconds=self._pre_roll_s)

    # -- planner and mission ----------------------------------------------

    def _on_plan_result(self, msg):
        data = self._payload(msg, self._plan_topic, self.get_logger())
        if data is None:
            return
        kind = str(data.get("kind") or "initial")
        status = str(data.get("status") or "ok")
        error = str(data.get("error") or "")
        # translated_plan since the planner sent both; plan is its old name
        plan = data.get("translated_plan", data.get("plan"))
        prompt_text = data.get("prompt_text")

        if kind == "delivery":
            self._on_delivery(data, status, error)
            return

        if kind == "replan":
            if self._test is None:
                self.get_logger().warn(
                    "[test_campaign] replan arrived with no test open -- ignored"
                )
                return
            self._record_call(data, kind, status, error, plan, prompt_text)
            return

        if kind != "initial":
            self.get_logger().warn(
                f"[test_campaign] unknown plan_result kind {kind!r} -- treating "
                f"it as 'initial'"
            )

        if self._test is not None:
            self._close_test("aborted", "superseded by new test")
        if self._open_test(data.get("prompt_num"), prompt_text,
                           data.get("plan_id")) is None:
            return
        self._record_call(data, "initial", status, error, plan, prompt_text)
        if status != "ok":
            self._close_test("aborted", f"LLM failure: {error or status}")

    def _record_call(self, data, kind, status, error, plan, prompt_text):
        t_sent = data.get("t_prompt_sent")
        t_recv = data.get("t_response_received")
        saved = None
        if plan is not None:
            saved = self._save_plan(data, kind, plan)
        self._test.record_llm_call(
            prompt_text if prompt_text is not None else self._test.prompt_text,
            response=None if plan is None else json.dumps(plan),
            tag=kind,
            t_sent=None if t_sent is None else self._t(float(t_sent)),
            t_received=None if t_recv is None else self._t(float(t_recv)),
            latency_ms=data.get("latency_ms"),
            ok=1 if status == "ok" else 0,
            error=error,
            llm_raw=data.get("llm_raw"),
            rejections=data.get("rejections"),
            translated_plan=plan,
            plan_file=None if saved is None else saved["file"],
            plan_hash=None if saved is None else saved["plan_hash"],
        )

    def _save_plan(self, data, kind, plan):
        """plan.json / plan_replan_N.json, checked against the loader's file.

        The reference is the file the planner handed /mission/load_mission
        (``mission_path``); without one, the path the planner always writes
        to, missions/llm_generated/<plan_id>.json in f1tenth_behavior's share.
        """
        plan_id = str(data.get("plan_id") or "")
        reference = data.get("mission_path")
        if not reference and plan_id:
            reference = _share_file(
                "f1tenth_behavior", "missions", "llm_generated", f"{plan_id}.json")
        try:
            saved = self._test.save_plan(
                plan, tag=kind, reference_path=reference, plan_id=plan_id or None)
        except Exception as exc:  # noqa: BLE001 - never lose the call row over it
            self.get_logger().error(
                f"[test_campaign] could not save the plan of {self._test.test_id}: "
                f"{type(exc).__name__}: {exc}")
            return None
        if saved is not None and not saved["match"]:
            self.get_logger().warn(
                f"[test_campaign] {self._test.test_id}/{saved['file']} does not match "
                f"{saved['reference'] or 'any loader file'} -- plan_file_mismatch "
                f"recorded")
        return saved

    def _on_delivery(self, data, status, error):
        """The planner's follow-up: load or start failed after the plan_result.

        Nothing else will end this test -- no mission event follows a failed
        load, and a failed start leaves the loader sitting at LOADED -- so it
        is closed here, after the usual post-roll, unless the mission somehow
        started anyway, in which case its own events end it.
        """
        stage = str(data.get("stage") or "delivery")
        if self._test is None:
            self.get_logger().warn(
                f"[test_campaign] {stage} result arrived with no test open -- ignored")
            return
        plan_id = str(data.get("plan_id") or "")
        if plan_id and self._plan_id and plan_id != self._plan_id:
            self.get_logger().warn(
                f"[test_campaign] {stage} result is for plan_id {plan_id!r}, but "
                f"the open test is {self._plan_id!r} -- recorded, not acted on")
            self._test.log_event("plan_id_mismatch", t=self._t(), for_event=stage,
                                 expected=self._plan_id, got=plan_id)
            return
        failed = status != "ok"
        self._test.log_event("delivery_failed" if failed else "delivery",
                             t=self._t(), stage=stage, error=error, plan_id=plan_id)
        if not failed:
            return
        self.get_logger().error(
            f"[test_campaign] {self._test.test_id}: {stage} FAILED ({error}) -- "
            f"closing the test")
        if not self._mission_started and self._pending_close is None:
            self._pending_close = (
                "aborted", f"{stage} failed: {error or 'no reason given'}",
                self._now() + self._post_roll_s,
            )

    def _on_mission_event(self, msg):
        data = self._payload(msg, self._event_topic, self.get_logger())
        if data is None:
            return
        event = str(data.get("event") or "")
        if self._test is None:
            self.get_logger().warn(
                f"[test_campaign] {event or 'mission event'} arrived with no test "
                f"open -- ignored"
            )
            return

        # The planner opens the test BEFORE it aborts whatever was running, so
        # that abort's mission_aborted lands in this test. It belongs to the
        # previous mission: acting on it would close this test as "cancelled
        # before start" before it began. This test's own lifecycle always
        # starts with its mission_loaded (or, if that was lost, its
        # mission_started), so an end event before either is stale. The
        # plan_id cannot decide it: repetitions of one prompt share one.
        if event in ("mission_finished", "mission_aborted") and not self._lifecycle_seen:
            self.get_logger().warn(
                f"[test_campaign] {event} (plan_id {data.get('plan_id')!r}) arrived "
                f"before this test's mission_loaded -- the previous mission's end, "
                f"recorded as stale_mission_event and ignored"
            )
            self._test.log_event(
                "stale_mission_event", t=self._t(), for_event=event,
                **{k: v for k, v in data.items() if k != "event"})
            return
        if event in ("mission_loaded", "mission_started"):
            self._lifecycle_seen = True

        # The planner's plan_id travels inside the plan to the mission node,
        # which echoes it here: a mismatch means these events belong to some
        # other mission, so it is recorded rather than quietly believed. Only
        # checked when both sides actually carry one.
        event_plan_id = str(data.get("plan_id") or "")
        if event_plan_id and self._plan_id and event_plan_id != self._plan_id:
            self.get_logger().warn(
                f"[test_campaign] {event or 'mission event'} carries plan_id "
                f"{event_plan_id!r}, but the open test is {self._plan_id!r} -- "
                f"recorded, and acted on, but the two may not belong together"
            )
            self._test.log_event(
                "plan_id_mismatch", t=self._t(), for_event=event,
                expected=self._plan_id, got=event_plan_id,
            )

        fields = {k: v for k, v in data.items() if k != "event"}
        self._test.log_event(event, t=self._t(), **fields)

        if event == "mission_started":
            self._mission_started = True
            if self._started_at is None:
                self._started_at = self._now()
        elif event in ("mission_finished", "mission_aborted"):
            reason = str(data.get("reason") or "")
            if event == "mission_finished":
                outcome = "completed"
            else:
                outcome = "aborted"
                reason = reason or "mission aborted"
            if event == "mission_aborted" and not self._mission_started:
                reason = "cancelled before start"
            self._pending_close = (
                outcome, reason, self._now() + self._post_roll_s
            )
        elif event not in MISSION_EVENTS:
            self.get_logger().warn(
                f"[test_campaign] unknown mission event {event!r} -- recorded, "
                f"but it ends nothing"
            )

    # -- data --------------------------------------------------------------

    def _on_odom(self, msg):
        stamp = self._stamp_seconds(msg.header)
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = yaw_from_quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w)
        # twist is in the child frame; the logger's vx/vy are world frame
        vx = twist.linear.x * math.cos(yaw) - twist.linear.y * math.sin(yaw)
        vy = twist.linear.x * math.sin(yaw) + twist.linear.y * math.cos(yaw)
        payload = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw,
            "vx": vx,
            "vy": vy,
            "yaw_rate": twist.angular.z,
        }
        if self._test is None:
            self._remember("state", stamp, payload)
            return
        clearance, self._obstacle_clearance = self._obstacle_clearance, None
        if clearance is not None:
            self._n_clearance += 1
        self._test.log_state(
            **payload, obstacle_clearance=clearance, t=self._t(stamp)
        )

    def _on_imu(self, msg):
        stamp = self._stamp_seconds(msg.header)
        payload = {
            "ax": msg.linear_acceleration.x,
            "ay": msg.linear_acceleration.y,
            "az": msg.linear_acceleration.z,
            "gx": msg.angular_velocity.x,
            "gy": msg.angular_velocity.y,
            "gz": msg.angular_velocity.z,
            "sensor_stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        if self._test is None:
            self._remember("imu", stamp, payload)
            return
        self._test.log_imu(**payload, t=self._t(stamp))

    def _on_drive(self, msg):
        stamp = self._stamp_seconds(msg.header)
        payload = {
            "cmd_speed": msg.drive.speed,
            "cmd_steer": msg.drive.steering_angle,
            "source": self._drive_topic,
        }
        if self._test is None:
            self._remember("command", stamp, payload)
            return
        self._test.log_command(**payload, t=self._t(stamp))

    def _on_mpc(self, msg):
        if self._test is None:
            return
        data = self._payload(msg, self._mpc_topic, self.get_logger())
        if data is None:
            return
        self._test.log_mpc(
            data.get("status", ""),
            solve_time_ms=data.get("solve_time_ms"),
            cost=data.get("cost"),
            iterations=data.get("iterations"),
            t=self._t(),
        )

    def _on_corridor(self, msg):
        if self._test is None:
            return
        data = self._payload(msg, self._corridor_topic, self.get_logger())
        if data is None:
            return
        corridor_odom = data.get("odom_topic")
        if corridor_odom and corridor_odom != self._odom_topic:
            self._warn_once(
                "corridor_frame",
                f"corridors are built from {corridor_odom} but poses come from "
                f"{self._odom_topic}: corridor_clearance compares two different "
                f"estimates. Set odom_topic:={corridor_odom}.",
            )
        polygon = data.get("polygon") or []
        if len(polygon) < 3:
            self.get_logger().warn(
                f"[test_campaign] corridor has {len(polygon)} points -- ignored"
            )
            return
        self._test.log_corridor(
            polygon,
            corridor_id=data.get("id"),
            source=data.get("source", self._corridor_topic),
            meta={k: v for k, v in data.items() if k not in ("polygon", "id")},
            t=self._t(),
        )

    def _on_clearance(self, msg):
        # handed to the next log_state, then forgotten: a pose carries the
        # clearance measured for it, never a stale one repeated for minutes
        self._obstacle_clearance = float(msg.data)
        if self._test is not None and self._mission_started:
            self._n_clearance_driving += 1

    def _on_safety(self, msg):
        if self._test is None:
            return
        data = self._payload(msg, self._safety_topic, self.get_logger())
        if data is None:
            return
        event = str(data.get("event") or "")
        if event not in ("contact", "estop"):
            self.get_logger().warn(
                f"[test_campaign] safety event {event!r} is neither 'contact' nor "
                f"'estop' -- recorded as is"
            )
        self._test.log_event(event, t=self._t(),
                             **{k: v for k, v in data.items() if k != "event"})


def main(argv=None):
    # A bare `ros2 run` must record with the campaign config too: see the
    # module docstring's PARAMETERS.
    args, params_file = with_packaged_params(sys.argv if argv is None else argv)
    rclpy.init(args=args)
    try:
        node = TestLoggerNode(params_file=params_file)
    except (RuntimeError, NotADirectoryError) as exc:
        # no workspace root: say where it looked and stop, rather than record
        # a campaign into a folder nobody will find
        print(f"[test_campaign] cannot start: {exc}", flush=True)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C reaches rclpy as either of these depending on how the signal
        # was delivered; both mean the same thing here. The context is already
        # down, so rosout would only complain -- print instead.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print("[test_campaign] interrupted", flush=True)
    finally:
        # Under ros2 launch a terminal Ctrl+C arrives TWICE: once from the
        # terminal, once forwarded by launch. The second must not land inside
        # close_open_test(), which is what writes the open test's meta.json --
        # found live, as a KeyboardInterrupt traceback out of destroy_node().
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close_open_test()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
