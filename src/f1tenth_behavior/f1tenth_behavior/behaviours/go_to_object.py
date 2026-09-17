"""py_trees Action: run the current go_to_object move -- tracks in, ObjectGoal out.

Replaces object_goal_bridge. The decisions (which track, when the move has
failed) live in mission/object_handler.py; this behaviour owns the ROS side:

  in   /costmap/semantic_tracks  confirmed, map-frame, class-labelled tracks,
                                 stamped with the detection CAPTURE time
       GLOBAL_XY_KEY             the vehicle, map frame (CheckStopCondition's
                                 global EKF subscription)
       OBJECT_STATUS_KEY         mpc_corr's /mpc/object_status, for the
                                 terminal target_behind flag
  out  /mpc/goal_object          ObjectGoal every tick while there is a target,
                                 with ONE move_id for the whole move
       /mpc/goal_object_end      the move is over
       /mpc/hold                 on a failure outcome, which aborts the mission

EVERY TICK, NOT PER TRACKS MESSAGE. The point and its stamp change only when a
tracks message brings a gated track, but the message is republished at the
tree's rate so mpc_corr's refresh watchdog measures one thing -- is the
mission still ticking -- and not the perception rate as well. Perception
staleness is the handler's business (tracks_max_gap_sec, lost_grace_sec) and
reaches mpc_corr as target_age_s through the capture stamp.

Placed in mission_progress between PublishMoveGoal and CheckStopCondition. For
any other move type it only makes sure a previous object move is ended, and
returns SUCCESS. For an object move it returns SUCCESS while the approach is
alive (so CheckStopCondition can judge object_reached and the timeout) and
FAILURE with the mission aborted on target_not_found / target_lost /
target_unreachable -- the same shape CheckStopCondition's timeout-abort has.

reached and timeout end the move elsewhere: AdvanceMove advances or completes
(and completion holds, which ends object mode in mpc_corr), and
CheckStopCondition's timeout branches publish the end themselves. If the next
move is not an object move, this behaviour publishes the end on that tick.
"""

import math
import time

import py_trees
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection3DArray

from f1tenth_messages.msg import ObjectGoal

from f1tenth_behavior.mission.move_scoring import record_move_outcome, write_mission_summary
from f1tenth_behavior.mission.object_handler import (
    HandlerParams, ObjectHandler, Track, object_move_wire_id)
from f1tenth_behavior.mission.runtime import (
    GLOBAL_TURN_ACCUM_KEY,
    GLOBAL_XY_KEY,
    GLOBAL_YAW_KEY,
    MISSION_KEY,
    OBJECT_STATUS_KEY,
    MissionRuntimeState,
    ObjectApproachRecord,
)


def tracks_from_message(msg) -> list:
    """Return the confirmed tracks in a semantic_tracks Detection3DArray."""
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    tracks = []
    for det in msg.detections:
        if not det.results:
            continue
        hyp = det.results[0].hypothesis
        tracks.append(Track(
            track_id=str(det.id), class_id=str(hyp.class_id),
            x=float(det.bbox.center.position.x), y=float(det.bbox.center.position.y),
            score=float(hyp.score), stamp_sec=stamp,
            width=float(det.bbox.size.x)))
    return tracks


def stamp_from_sec(sec: float) -> Time:
    """Convert float seconds to a builtin_interfaces/Time."""
    whole = int(math.floor(sec))
    return Time(sec=whole, nanosec=int(round((sec - whole) * 1e9)) % 1000000000)


class GoToObject(py_trees.behaviour.Behaviour):
    """Run the current go_to_object move; see the module docstring."""

    def __init__(self, name='GoToObject',
                 tracks_topic='/costmap/semantic_tracks',
                 goal_topic='/mpc/goal_object',
                 end_topic='/mpc/goal_object_end',
                 hold_topic='/mpc/hold',
                 map_frame='map',
                 follow_gate_m=0.5,
                 tracks_max_gap_sec=0.5,
                 clock=time.monotonic):
        """Set topics and handler tuning; `clock` is injectable for tests."""
        super().__init__(name=name)
        self._tracks_topic = tracks_topic
        self._goal_topic = goal_topic
        self._end_topic = end_topic
        self._hold_topic = hold_topic
        self.map_frame = map_frame
        self.follow_gate_m = float(follow_gate_m)
        self.tracks_max_gap_sec = float(tracks_max_gap_sec)
        self._clock = clock
        self.node = None
        self.goal_pub = None
        self.end_pub = None
        self.hold_pub = None
        self.tracks = []
        self.tracks_received_sec = None
        self.handler = None
        self.active_wire_id = None
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=MISSION_KEY, access=py_trees.common.Access.WRITE)
        for key in (GLOBAL_XY_KEY, GLOBAL_YAW_KEY, GLOBAL_TURN_ACCUM_KEY, OBJECT_STATUS_KEY):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Create the tracks subscription and the three publishers."""
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("GoToObject.setup() didn't find 'node' in kwargs") from e
        self.node.create_subscription(
            Detection3DArray, self._tracks_topic, self._tracks_cb, 10)
        self.goal_pub = self.node.create_publisher(ObjectGoal, self._goal_topic, 10)
        self.end_pub = self.node.create_publisher(String, self._end_topic, 10)
        self.hold_pub = self.node.create_publisher(Bool, self._hold_topic, 10)

    def _tracks_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.node.get_logger().warn(
                f'[go_to_object] semantic tracks in frame {msg.header.frame_id!r}, '
                f'expected {self.map_frame!r}; ignored', throttle_duration_sec=2.0)
            return
        self.tracks = tracks_from_message(msg)
        self.tracks_received_sec = self._clock()

    def _bb(self, key):
        """Return a blackboard value, or None while nothing has written it yet."""
        try:
            return getattr(self.blackboard, key)
        except (KeyError, AttributeError):
            return None

    def _end_active(self, why):
        if self.active_wire_id is None:
            return
        self.end_pub.publish(String(data=self.active_wire_id))
        self.node.get_logger().info(
            f'[go_to_object] ended {self.active_wire_id!r}: {why}')
        self.active_wire_id = None
        self.handler = None

    def update(self):
        """Advance the handler one tick and publish or end accordingly."""
        state: MissionRuntimeState = getattr(self.blackboard, MISSION_KEY)
        move = state.current_move
        spec = getattr(move, 'go_to_object', None) if move is not None else None
        if spec is None:
            self._end_active('the mission moved on to a different move type')
            return py_trees.common.Status.SUCCESS

        now = self._clock()
        wire_id = object_move_wire_id(state.config.mission_id, state.run_generation, move.id)
        if wire_id != self.active_wire_id:
            self._end_active('the next object move started')
            self.handler = ObjectHandler(HandlerParams(
                target_class=spec.target_class, gap_m=spec.gap_m,
                nose_reach_m=spec.nose_reach_m,
                speed=spec.speed, acquire_timeout_sec=spec.acquire_timeout_sec,
                lost_grace_sec=spec.lost_grace_sec, follow_gate_m=self.follow_gate_m,
                tracks_max_gap_sec=self.tracks_max_gap_sec), start_sec=now)
            self.active_wire_id = wire_id
            state.object_record = ObjectApproachRecord(wire_move_id=wire_id)
            self.node.get_logger().info(
                f"[go_to_object] move '{move.id}' as {wire_id!r}: acquiring the nearest "
                f'{spec.target_class!r}, gap {spec.gap_m:.2f} m '
                f'(gap_min {spec.gap_min_m:.2f})')

        status = self._bb(OBJECT_STATUS_KEY)
        behind_terminal = (status is not None and status.move_id == wire_id
                           and status.target_behind_terminal)
        previous_phase = self.handler.phase
        step = self.handler.update(
            now, self.tracks, self.tracks_received_sec, self._bb(GLOBAL_XY_KEY),
            behind_terminal=behind_terminal)

        record = state.object_record
        record.phase = step.phase
        record.target_xy = step.target_xy
        record.track_id = step.track_id
        record.target_radius = step.target_radius
        record.gap_m = spec.gap_m
        record.nose_reach_m = spec.nose_reach_m
        if step.phase != previous_phase:
            self.node.get_logger().info(
                f'[go_to_object] {wire_id!r}: {previous_phase} -> {step.phase}'
                + (f' track={step.track_id}' if step.track_id else ''))

        if step.outcome is None:
            if step.target_xy is not None:
                self._publish_goal(wire_id, spec, step)
            return py_trees.common.Status.SUCCESS

        # target_not_found / target_lost / target_unreachable: the move FAILED.
        record.outcome = step.outcome
        self._end_active(step.outcome)
        self.hold_pub.publish(Bool(data=True))
        state.last_stop_reason = f'go_to_object:{step.outcome}'
        self.node.get_logger().error(
            f"[mission] Move '{move.id}' failed: {step.outcome} -- aborting mission.")
        record_move_outcome(
            state, self.node.get_logger(), move, state.last_stop_reason, now,
            end_global_xy=self._bb(GLOBAL_XY_KEY),
            end_global_yaw=self._bb(GLOBAL_YAW_KEY),
            global_turn_accum_deg=self._bb(GLOBAL_TURN_ACCUM_KEY))
        write_mission_summary(
            state.config.mission_id, state.move_outcomes, 'ABORTED',
            state.mission_start_wall_time, self.node.get_logger())
        state.abort()
        return py_trees.common.Status.FAILURE

    def _publish_goal(self, wire_id, spec, step):
        msg = ObjectGoal()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = stamp_from_sec(step.target_stamp_sec)
        msg.move_id = wire_id
        msg.target_class = spec.target_class
        msg.point.x = float(step.target_xy[0])
        msg.point.y = float(step.target_xy[1])
        # mpc_corr's standoff is a CENTRE distance: gap + nose_reach + radius.
        msg.standoff = float(step.centre_standoff)
        msg.speed = float(step.speed)
        msg.track_id = str(step.track_id or '')
        msg.gap_m = float(spec.gap_m)
        self.goal_pub.publish(msg)
