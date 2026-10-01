"""go_to_object and /drive-clamp data in the mission bag and the extract's run summary.

Three layers, each with the real code:
  * the logger records the new topics (its default topic list);
  * mission_extract.read_bag decodes them from a REAL rosbag2 written to
    tmp_path with rosbag2_py (synthetic messages, no hardware, no live graph);
  * object_summary turns them into the meta row's per-move summary, and
    write_extract / read_extract carry both through the parquet.

Run standalone: python3 -m pytest test/test_object_run_summary.py -v
"""

import json
import math

import pyarrow.parquet as pq
import pytest
import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from std_msgs.msg import String

from f1tenth_messages.msg import DriveClamp, MoveOutcome, ObjectApproachStatus, ObjectGoal

from f1tenth_logger import mission_render
from f1tenth_logger.mission_extract import read_bag, write_extract
from f1tenth_logger.mission_logger_node import _DEFAULT_TOPICS
from f1tenth_logger.object_summary import summarize_drive_clamp, summarize_object_approach

WIRE = 'go_to_person#3/move_0_go_to_person'


def test_the_logger_records_the_object_and_clamp_topics():
    for topic in ('/mpc/goal_object', '/mpc/goal_object_end', '/mpc/object_status',
                  '/mission/move_outcome', '/mpc/drive_clamp', '/costmap/semantic_tracks'):
        assert topic in _DEFAULT_TOPICS, topic


def _status(r, alpha=0.05, age=0.2, itr=False, behind=False, terminal=False, watchdog=False,
            latched=False, speed=0.4):
    msg = ObjectApproachStatus()
    msg.move_id = WIRE
    msg.r, msg.alpha, msg.target_age_s = r, alpha, age
    msg.inside_turn_radius, msg.target_behind = itr, behind
    msg.target_behind_terminal, msg.goal_watchdog = terminal, watchdog
    msg.stop_latched, msg.speed = latched, speed
    msg.track_id, msg.gap = '12', r + 0.5
    return msg


def _write_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = {
        '/mpc/object_status': 'f1tenth_messages/msg/ObjectApproachStatus',
        '/mpc/goal_object': 'f1tenth_messages/msg/ObjectGoal',
        '/mpc/goal_object_end': 'std_msgs/msg/String',
        '/mission/move_outcome': 'f1tenth_messages/msg/MoveOutcome',
        '/mpc/drive_clamp': 'f1tenth_messages/msg/DriveClamp',
    }
    for topic_id, (name, type_name) in enumerate(topics.items()):
        # id is a required positional arg as of this Jazzy rosbag2_py --
        # TopicMetadata(id: int, name: str, type: str, serialization_format:
        # str, ...). Older rosbag2_py accepted the no-id keyword form this
        # used to call; any unique int per topic is fine, the writer assigns
        # its own ids internally and ignores what's passed here.
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=topic_id, name=name, type=type_name, serialization_format='cdr'))

    t = 1_000_000_000

    def put(topic, msg, dt_ns=100_000_000):
        nonlocal t
        t += dt_ns
        writer.write(topic, serialize_message(msg), t)

    goal = ObjectGoal()
    goal.move_id, goal.target_class = WIRE, 'person'
    goal.point.x, goal.point.y, goal.standoff, goal.speed = 4.0, 0.8, 1.2, 0.4
    goal.header.stamp = Time(sec=12, nanosec=500000000)
    for r, itr, latched, speed in ((1.8, True, False, 0.4), (0.9, False, False, 0.42),
                                   (0.30, False, True, 0.4), (0.08, False, True, 0.0)):
        put('/mpc/goal_object', goal)
        put('/mpc/object_status', _status(r, itr=itr, latched=latched, speed=speed), dt_ns=1)
    put('/mpc/goal_object_end', String(data=WIRE))
    outcome = MoveOutcome()
    outcome.mission_id, outcome.move_id = 'go_to_person', 'move_0_go_to_person'
    outcome.move_type, outcome.outcome, outcome.wire_move_id = 'go_to_object', 'reached', WIRE
    outcome.stop_reason = 'stop_condition:object_reached'
    outcome.commanded, outcome.actual = 1.2, 1.28
    outcome.arrival_bearing_error_deg, outcome.track_gap_m = 2.9, 1.31
    outcome.score_percent = math.nan
    put('/mission/move_outcome', outcome)
    clamp = DriveClamp()
    clamp.requested_speed, clamp.applied_speed = -0.2, 0.0
    put('/mpc/drive_clamp', clamp)
    clamp2 = DriveClamp()
    clamp2.requested_speed, clamp2.applied_speed = 1.4, 1.0
    put('/mpc/drive_clamp', clamp2)
    # The writer closes (and flushes metadata.yaml) when it goes out of scope
    # on return; rosbag2_py's Humble SequentialWriter has no close().


@pytest.fixture(scope='module')
def bag(tmp_path_factory):
    path = tmp_path_factory.mktemp('bags') / 'object_run'
    _write_bag(path)
    return read_bag(path, 'global')


class TestReadBagDecodesTheNewTopics:

    def test_object_status_samples(self, bag):
        samples = bag['streams']['object_status'].v
        assert [round(s['r'], 3) for s in samples] == [1.8, 0.9, 0.3, 0.08]
        assert samples[0]['inside_turn_radius'] is True
        assert samples[-1]['move_id'] == WIRE
        assert [s['stop_latched'] for s in samples] == [False, False, True, True]
        assert samples[1]['speed'] == pytest.approx(0.42)
        assert samples[-1]['track_id'] == '12'
        assert samples[-1]['gap'] == pytest.approx(0.58)

    def test_goals_end_outcome_and_clamps(self, bag):
        assert len(bag['streams']['goal_object'].v) == 4
        assert bag['streams']['goal_object'].v[0]['stamp'] == pytest.approx(12.5)
        assert bag['streams']['goal_object_end'].v == [{'move_id': WIRE}]
        (outcome,) = bag['streams']['move_outcome'].v
        assert outcome['outcome'] == 'reached'
        assert [round(c['requested_speed'], 3) for c in bag['streams']['drive_clamp'].v] == [
            -0.2, 1.4]


class TestSummary:

    def test_the_per_move_summary(self, bag):
        s = bag['streams']
        (move,) = summarize_object_approach(
            s['object_status'], s['goal_object'], s['goal_object_end'],
            s['move_outcome']).values()
        assert move['goals'] == 4
        assert move['status_samples'] == 4
        assert move['final']['r'] == pytest.approx(0.08)
        assert move['final']['alpha'] == pytest.approx(0.05)
        assert move['final']['target_age_s'] == pytest.approx(0.2)
        assert move['min_r'] == pytest.approx(0.08)
        assert move['inside_turn_radius_samples'] == 1
        assert move['target_behind_terminal'] is False
        assert move['end_t'] is not None
        assert move['outcome'] == 'reached'
        assert move['stop_reason'] == 'stop_condition:object_reached'
        assert move['final_gap_or_range'] == pytest.approx(1.28)
        assert move['arrival_bearing_error_deg'] == pytest.approx(2.9)
        assert move['track_gap_m'] == pytest.approx(1.31)
        assert move['latched_r'] == pytest.approx(0.30)
        assert move['latched_t'] is not None
        assert move['max_speed'] == pytest.approx(0.42)
        assert move['final']['gap'] == pytest.approx(0.58)
        assert move['final']['track_id'] == '12'

    def test_the_clamp_summary(self, bag):
        clamp = summarize_drive_clamp(bag['streams']['drive_clamp'])
        assert clamp['events'] == 2
        assert clamp['max_requested'] == pytest.approx(1.4)
        assert clamp['min_requested'] == pytest.approx(-0.2)

    def test_a_run_without_object_moves_summarises_to_nothing(self):
        empty = mission_render.Stream(None)
        assert summarize_object_approach(empty, empty, empty, empty) == {}
        assert summarize_drive_clamp(empty)['events'] == 0


class TestTheExtractCarriesIt:

    def _extract(self, bag, tmp_path):
        bag = dict(bag, t0=bag['t0'] or 0.0, t1=bag['t1'] or 0.0)
        return write_extract(bag, tmp_path / 'r.extract.parquet', manifest={}, run_id='r')

    def test_the_meta_row_holds_the_run_summary(self, bag, tmp_path):
        path = self._extract(bag, tmp_path)
        table = pq.read_table(path).to_pydict()
        meta = json.loads(table['payload'][table['kind'].index('meta')])
        assert meta['object_approach'][WIRE]['outcome'] == 'reached'
        assert meta['object_approach'][WIRE]['final']['r'] == pytest.approx(0.08)
        assert meta['drive_clamp']['events'] == 2

    def test_the_event_streams_round_trip_as_dicts(self, bag, tmp_path):
        back = mission_render.read_extract(self._extract(bag, tmp_path))
        assert back['streams']['object_status'].v[-1]['move_id'] == WIRE
        assert back['streams']['move_outcome'].v[0]['wire_move_id'] == WIRE
