#!/usr/bin/env python3
"""The campaign recorder: one long-lived rclpy node, one TestLogger per test.

    python3 test_logger_node.py --ros-args \\
        -p campaign:=first_test_campaing -p robot_radius:=0.3

Leave it running in its own terminal for the whole session. It opens a test
when the planner announces an LLM result and closes it when the mission ends,
so tests accumulate without ever restarting the node.

**It never calls the LLM.** The planner owns the call and publishes what it
measured; this node only records. Everything it knows arrives as messages:

  /test/plan_result    std_msgs/String, JSON, after EVERY LLM call
                       {prompt_num, prompt_text, kind: initial|replan,
                        t_prompt_sent, t_response_received, latency_ms,
                        status: ok|error, error, plan_id, plan}
  /test/mission_event  std_msgs/String, JSON
                       {event: mission_loaded|mission_started|
                               mission_finished|mission_aborted,
                        plan_id, reason, countdown_s}

Lifecycle, in one place:

* ``initial`` + ``ok``      -> open a test, record the call, keep recording.
* ``initial`` + ``error``   -> open a test anyway, record the call, close it
                               as aborted. A failed LLM call is a test.
* ``initial`` while open    -> close the open one as "superseded by new test".
* ``replan``                -> another row in the open test; never a new one.
* mission_finished/aborted  -> keep recording for ``post_roll_s``, then close.
* aborted before started    -> "cancelled before start".
* nothing for max_test_duration_s -> "timeout: no end message".
* Ctrl+C                    -> "logger node shut down", files closed cleanly.

plan_id: the planner puts its plan_id inside the plan, the mission node
echoes it on every mission event, and this node checks the two match. A
mismatch is warned about and written to events.jsonl as ``plan_id_mismatch``;
the event is still acted on, because a stalled test that times out loses more
than a mislabelled one that is flagged.

Time base: t=0 is the moment the test folder is created, so the prompt that
started it has a negative ``t_sent``. Sample times come from the message
header where there is one, and from the node clock otherwise.

QoS: the high-rate sensor streams are subscribed BEST_EFFORT, which also
matches RELIABLE publishers, so a sensor publisher can never fail to connect;
the low-rate control topics are RELIABLE so a mission event cannot be dropped.
Set ``best_effort_sensors:=false`` if a publisher needs the opposite.
"""

from __future__ import annotations

import json
import math
import sys
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_logger import DEFAULT_CAMPAIGN, TestLogger  # noqa: E402

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

STAMP_SANITY_S = 5.0  # header stamps further than this from the clock are ignored

#: The mission lifecycle this node understands. mission/loader.py's _TEST_EVENTS
#: publishes exactly these four; test_mission_countdown.py asserts the two lists
#: stay equal, so neither side can rename an event on its own.
MISSION_EVENTS = (
    "mission_loaded", "mission_started", "mission_finished", "mission_aborted",
)


def yaw_from_quaternion(x, y, z, w):
    """Yaw only, counter-clockwise positive, from a ROS quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class TestLoggerNode(Node):
    """Owns at most one open :class:`TestLogger` at a time."""

    def __init__(self):
        super().__init__("test_logger_node")

        def param(name, default):
            return self.declare_parameter(name, default).value

        self._root = param("root", "") or None
        self._campaign = param("campaign", DEFAULT_CAMPAIGN)
        self._robot_radius = float(param("robot_radius", 0.0))
        self._mpc_ok_statuses = tuple(param("mpc_ok_statuses", ["solved"]))
        self._robot_name = param("robot_name", "")
        self._llm_model = param("llm_model", "")

        self._plan_topic = param("plan_result_topic", "/test/plan_result")
        self._event_topic = param("mission_event_topic", "/test/mission_event")
        self._odom_topic = param("odom_topic", "/odom")
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

        reliable = QoSProfile(depth=10)
        sensor = QoSProfile(depth=50)
        if best_effort:
            sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(String, self._plan_topic,
                                 self._on_plan_result, reliable)
        self.create_subscription(String, self._event_topic,
                                 self._on_mission_event, reliable)
        self.create_subscription(String, self._mpc_topic, self._on_mpc, sensor)
        self.create_subscription(String, self._corridor_topic,
                                 self._on_corridor, reliable)
        self.create_subscription(String, self._safety_topic,
                                 self._on_safety, reliable)
        self.create_subscription(Float32, self._clearance_topic,
                                 self._on_clearance, sensor)
        if Odometry is not None:
            self.create_subscription(Odometry, self._odom_topic,
                                     self._on_odom, sensor)
        else:
            self._warn_once("nav_msgs", "nav_msgs is missing: no odometry logged")
        if Imu is not None:
            self.create_subscription(Imu, self._imu_topic, self._on_imu, sensor)
        else:
            self._warn_once("sensor_msgs", "sensor_msgs is missing: no IMU logged")
        if AckermannDriveStamped is not None:
            self.create_subscription(AckermannDriveStamped, self._drive_topic,
                                     self._on_drive, sensor)
        else:
            self._warn_once(
                "ackermann_msgs",
                f"ackermann_msgs is not installed: {self._drive_topic} will not "
                f"be logged (everything else still works)",
            )

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"[test_logger] campaign={self._campaign} "
            f"post_roll={self._post_roll_s}s pre_roll={self._pre_roll_s}s "
            f"timeout={self._max_test_duration_s}s -- waiting for "
            f"{self._plan_topic}"
        )

    # -- helpers -----------------------------------------------------------

    def _warn_once(self, key, message):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(f"[test_logger] {message}")

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
            logger.warn(f"[test_logger] {what} is not valid JSON: {exc}")
            return None
        if not isinstance(data, dict):
            logger.warn(f"[test_logger] {what} must be a JSON object")
            return None
        return data

    # -- test lifecycle ----------------------------------------------------

    def _open_test(self, prompt_num, prompt_text, plan_id):
        extra = {
            "robot": self._robot_name or None,
            "llm_model": self._llm_model or None,
            "plan_id": plan_id,
            "ros_start_time": self._now(),
            "source": "test_logger_node",
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
                f"[test_logger] cannot open a test for prompt_num="
                f"{prompt_num!r} / text {str(prompt_text)[:60]!r}: "
                f"{type(exc).__name__}: {exc}"
            )
            return None

        self._test = logger
        self._t0 = self._now()
        self._mission_started = False
        self._pending_close = None
        self._obstacle_clearance = None
        self._plan_id = str(plan_id or "")

        if prompt_text is not None and str(prompt_text) != logger.prompt_text:
            self.get_logger().warn(
                f"[test_logger] {logger.test_id}: the prompt in the message is "
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
            f"[test_logger] open {logger.mission}/{logger.test_id}"
        )
        return logger

    def _close_test(self, outcome, reason):
        logger, self._test = self._test, None
        self._pending_close = None
        self._mission_started = False
        if logger is None:
            return
        summary = logger.finish(outcome, reason)
        message = (
            f"[test_logger] close {logger.mission}/{logger.test_id} "
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

    def _tick(self):
        if self._test is None:
            self._trim_pre_roll()
            return
        now = self._now()
        if self._pending_close is not None:
            outcome, reason, close_at = self._pending_close
            if now >= close_at:
                self._close_test(outcome, reason)
            return
        if now - self._t0 > self._max_test_duration_s:
            self._close_test("aborted", "timeout: no end message")

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
        plan = data.get("plan")
        prompt_text = data.get("prompt_text")

        if kind == "replan":
            if self._test is None:
                self.get_logger().warn(
                    "[test_logger] replan arrived with no test open -- ignored"
                )
                return
            self._record_call(data, kind, status, error, plan, prompt_text)
            return

        if kind != "initial":
            self.get_logger().warn(
                f"[test_logger] unknown plan_result kind {kind!r} -- treating "
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
        self._test.record_llm_call(
            prompt_text if prompt_text is not None else self._test.prompt_text,
            response=None if plan is None else json.dumps(plan),
            tag=kind,
            t_sent=None if t_sent is None else self._t(float(t_sent)),
            t_received=None if t_recv is None else self._t(float(t_recv)),
            latency_ms=data.get("latency_ms"),
            ok=1 if status == "ok" else 0,
            error=error,
        )

    def _on_mission_event(self, msg):
        data = self._payload(msg, self._event_topic, self.get_logger())
        if data is None:
            return
        event = str(data.get("event") or "")
        if self._test is None:
            self.get_logger().warn(
                f"[test_logger] {event or 'mission event'} arrived with no test "
                f"open -- ignored"
            )
            return

        # The planner's plan_id travels inside the plan to the mission node,
        # which echoes it here: a mismatch means these events belong to some
        # other mission, so it is recorded rather than quietly believed. Only
        # checked when both sides actually carry one.
        event_plan_id = str(data.get("plan_id") or "")
        if event_plan_id and self._plan_id and event_plan_id != self._plan_id:
            self.get_logger().warn(
                f"[test_logger] {event or 'mission event'} carries plan_id "
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
                f"[test_logger] unknown mission event {event!r} -- recorded, "
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
        polygon = data.get("polygon") or []
        if len(polygon) < 3:
            self.get_logger().warn(
                f"[test_logger] corridor has {len(polygon)} points -- ignored"
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

    def _on_safety(self, msg):
        if self._test is None:
            return
        data = self._payload(msg, self._safety_topic, self.get_logger())
        if data is None:
            return
        event = str(data.get("event") or "")
        if event not in ("contact", "estop"):
            self.get_logger().warn(
                f"[test_logger] safety event {event!r} is neither 'contact' nor "
                f"'estop' -- recorded as is"
            )
        self._test.log_event(event, t=self._t(),
                             **{k: v for k, v in data.items() if k != "event"})


def main(argv=None):
    rclpy.init(args=argv)
    node = TestLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C reaches rclpy as either of these depending on how the signal
        # was delivered; both mean the same thing here.
        node.get_logger().info("[test_logger] interrupted")
    finally:
        node.close_open_test()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
