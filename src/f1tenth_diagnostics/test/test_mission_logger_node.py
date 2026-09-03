"""mission_logger_node.py tests -- automatic per-mission rosbag2 recording.

Same convention test_battery_voltage_check_node.py already uses: construct a
real (never spun) rclpy Node via parameter_overrides and call its callbacks
directly -- no executor, no live topics, and here, no real recorder either.

The recorder itself is deliberately NOT exercised here. Its threading/cancel/
metadata.yaml lifecycle and the BEST_EFFORT QoS-override behaviour were
verified LIVE against real rosbag2 on this box (see that node's own module
docstring); a unit test standing up a real Recorder would be testing rosbag2,
would need a live DDS graph, and -- as this session found the hard way -- would
silently record zero messages whenever the discovery server happens to be
down, making it a flaky test that fails for reasons unrelated to this node.
What IS unit-testable, and is what actually broke in the field, is the
DISPATCH: which transitions start a recording, which stop it, which do
neither. Those tests stub _start_recording/_stop_recording and assert on the
decisions.

Filesystem behaviour (sweeping, manifests) is tested for real against tmp_path
-- no stubbing, since that is pure local I/O with no ROS graph involved.

Run standalone: python3 -m pytest test/test_mission_logger_node.py -v
"""

import json
import os

import pytest
import rclpy
from rclpy.parameter import Parameter

from f1tenth_messages.msg import MissionStatus

from f1tenth_diagnostics.mission_logger_node import MissionLoggerNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_node(tmp_path, **overrides):
    params = [
        Parameter('bag_root', Parameter.Type.STRING, str(tmp_path)),
        Parameter('sweep_period_sec', Parameter.Type.DOUBLE, 1e6),  # never auto-fires
    ]
    for k, v in overrides.items():
        params.append(Parameter(k, value=v))
    return MissionLoggerNode(parameter_overrides=params)


def _status(state, json_path='/x/missions/bottle_then_person.json', estop=False):
    msg = MissionStatus()
    msg.state = state
    msg.json_path = json_path
    msg.emergency_stop_active = estop
    return msg


class _Spy:
    """Replaces _start_recording/_stop_recording so dispatch can be asserted
    without opening a real bag."""

    def __init__(self, node):
        self.started = []
        self.stopped = []
        node._start_recording = self._start
        node._stop_recording = self._stop
        self._node = node

    def _start(self, msg):
        self.started.append(msg)
        self._node._active_run = {'run_id': 'stub'}  # mirrors the real state flag

    def _stop(self, outcome):
        self.stopped.append(outcome)
        self._node._active_run = None


class TestLifecycleDispatch:

    def test_running_starts_recording(self, tmp_path):
        node = _make_node(tmp_path)
        spy = _Spy(node)
        node._on_status(_status('RUNNING'))
        assert len(spy.started) == 1
        assert spy.stopped == []
        node.destroy_node()

    def test_complete_stops_with_outcome(self, tmp_path):
        """The NORMAL SUCCESS path -- the transition that published nothing at
        all before the loader.py mission-end fix."""
        node = _make_node(tmp_path)
        spy = _Spy(node)
        node._on_status(_status('RUNNING'))
        node._on_status(_status('COMPLETE'))
        assert spy.stopped == ['COMPLETE']
        node.destroy_node()

    def test_aborted_stops_with_outcome(self, tmp_path):
        """Failure runs are the data you actually want -- an abort must stop
        recording just as reliably as success."""
        node = _make_node(tmp_path)
        spy = _Spy(node)
        node._on_status(_status('RUNNING'))
        node._on_status(_status('ABORTED'))
        assert spy.stopped == ['ABORTED']
        node.destroy_node()

    def test_holding_does_not_stop_recording(self, tmp_path):
        """HOLDING is a mid-mission pause, not an end state -- and is usually
        the most interesting part of the run to keep."""
        node = _make_node(tmp_path)
        spy = _Spy(node)
        node._on_status(_status('RUNNING'))
        node._on_status(_status('HOLDING'))
        assert spy.stopped == []
        assert len(spy.started) == 1, 'HOLDING must not start a second recording'
        node.destroy_node()

    def test_emergency_stop_latch_stops_recording(self, tmp_path):
        node = _make_node(tmp_path)
        spy = _Spy(node)
        node._on_status(_status('RUNNING'))
        node._on_status(_status('RUNNING', estop=True))
        assert spy.stopped == ['EMERGENCY_STOP']
        node.destroy_node()

    def test_repeated_running_does_not_restart(self, tmp_path):
        node = _make_node(tmp_path)
        spy = _Spy(node)
        for _ in range(5):
            node._on_status(_status('RUNNING'))
        assert len(spy.started) == 1
        node.destroy_node()

    def test_idle_without_active_run_does_nothing(self, tmp_path):
        node = _make_node(tmp_path)
        spy = _Spy(node)
        node._on_status(_status('IDLE'))
        assert spy.started == [] and spy.stopped == []
        node.destroy_node()

    def test_callback_never_raises(self, tmp_path):
        """A logger must never take down the mission node -- a malformed status
        must be swallowed and logged, not propagated into the executor."""
        node = _make_node(tmp_path)

        def _boom(msg):
            raise RuntimeError('simulated failure')

        node._start_recording = _boom
        node._on_status(_status('RUNNING'))  # must not raise
        node.destroy_node()


class TestStorageFallback:

    def test_unavailable_storage_falls_back(self, tmp_path):
        """mcap is not installed on this box; the node must degrade to a
        registered writer rather than failing every recording."""
        node = _make_node(tmp_path, storage_id='definitely_not_a_real_plugin')
        assert node.storage_id != 'definitely_not_a_real_plugin'
        import rosbag2_py
        assert node.storage_id in set(rosbag2_py.get_registered_writers())
        node.destroy_node()

    def test_available_storage_is_used_as_requested(self, tmp_path):
        node = _make_node(tmp_path, storage_id='sqlite3')
        assert node.storage_id == 'sqlite3'
        node.destroy_node()


class TestIncompleteBagSweep:

    def _bag(self, root, name, complete):
        d = os.path.join(str(root), name)
        os.makedirs(d)
        open(os.path.join(d, f'{name}_0.db3'), 'w').close()
        if complete:
            open(os.path.join(d, 'metadata.yaml'), 'w').close()
        return d

    def test_incomplete_bag_is_moved_not_deleted(self, tmp_path):
        """Default sweep_action must not destroy run data: a partial bag is
        often still readable directly, and the run cannot be re-collected."""
        node = _make_node(tmp_path)
        bad = self._bag(tmp_path, 'run_incomplete', complete=False)
        node._sweep_incomplete_bags(reason='test')
        assert not os.path.exists(bad)
        assert os.path.isdir(os.path.join(str(tmp_path), 'incomplete', 'run_incomplete'))
        node.destroy_node()

    def test_complete_bag_is_left_alone(self, tmp_path):
        node = _make_node(tmp_path)
        good = self._bag(tmp_path, 'run_complete', complete=True)
        node._sweep_incomplete_bags(reason='test')
        assert os.path.isdir(good)
        node.destroy_node()

    def test_in_progress_bag_is_never_swept(self, tmp_path):
        """The active recording has no metadata.yaml yet BY DEFINITION -- it is
        only written on cancel(). Sweeping it would destroy the run in flight."""
        node = _make_node(tmp_path)
        active = self._bag(tmp_path, 'run_active', complete=False)
        node._active_bag_dir = active
        node._sweep_incomplete_bags(reason='test')
        assert os.path.isdir(active)
        node.destroy_node()

    def test_delete_action_removes(self, tmp_path):
        node = _make_node(tmp_path, sweep_action='delete')
        bad = self._bag(tmp_path, 'run_incomplete', complete=False)
        node._sweep_incomplete_bags(reason='test')
        assert not os.path.exists(bad)
        assert not os.path.exists(os.path.join(str(tmp_path), 'incomplete'))
        node.destroy_node()


class TestRunMetadata:

    def _run(self, tmp_path):
        return {
            'mission_id': 'bottle_then_person', 'run_id': 'RUN',
            'start_time': 'T0', 'end_time': None, 'outcome': None,
            'bag_path': os.path.join(str(tmp_path), 'RUN'),
            'params_snapshot_path': os.path.join(str(tmp_path), 'RUN.params.yaml'),
            'manifest_path': os.path.join(str(tmp_path), 'RUN.manifest.json'),
            'storage_id': 'sqlite3', 'mission_json_path': '/x.json',
        }

    def test_manifest_has_the_fields_analysis_needs(self, tmp_path):
        node = _make_node(tmp_path)
        run = self._run(tmp_path)
        node._write_manifest(run)
        with open(run['manifest_path']) as fh:
            data = json.load(fh)
        for key in ('mission_id', 'start_time', 'end_time', 'outcome',
                    'bag_path', 'params_snapshot_path'):
            assert key in data, f'manifest missing {key}'
        node.destroy_node()

    def test_manifest_records_git_commit_and_dirty_flag(self, tmp_path):
        """A commit hash alone names the wrong tree whenever there are
        working-tree changes -- dirty must be recorded too."""
        node = _make_node(tmp_path)
        run = self._run(tmp_path)
        node._write_manifest(run)
        with open(run['manifest_path']) as fh:
            git = json.load(fh)['git']
        assert set(('commit', 'dirty', 'branch')) <= set(git)
        node.destroy_node()

    def test_params_snapshot_is_a_real_copy(self, tmp_path):
        """This session's real bugs were one-line param values -- the snapshot
        is often the most valuable artifact of a run."""
        node = _make_node(tmp_path)
        run = self._run(tmp_path)
        node._write_params_snapshot(run)
        assert os.path.isfile(run['params_snapshot_path'])
        assert 'yolo_model' in open(run['params_snapshot_path']).read()
        node.destroy_node()

    def test_resolved_params_snapshot_includes_key_switches(self, tmp_path):
        node = _make_node(tmp_path)
        resolved = node._resolved_params()
        for key in ('yolo_model', 'yolo_model_task', 'enable_slam'):
            assert key in resolved
        node.destroy_node()
