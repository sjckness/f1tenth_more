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
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.parameter import Parameter

from f1tenth_messages.msg import MissionStatus

from f1tenth_logger import mission_logger_node as mission_logger_node_module
from f1tenth_logger.mission_logger_node import (
    MissionLoggerNode, _read_lock_pid, _snapshot_indices, acquire_singleton_lock,
    release_singleton_lock)


def _make_dead_pid():
    """A PID that is definitely not running: spawn a trivial process, reap it,
    and reuse its number. A hardcoded large constant could collide with a real
    process and turn the stale-lock tests into coin flips."""
    proc = subprocess.Popen(['true'])
    proc.wait()
    return proc.pid


_DEAD_PID = _make_dead_pid()

# The child in the two-process lock test imports f1tenth_logger without a
# sourced overlay, so it needs the package's own parent on sys.path.
_PKG_PATH = os.path.dirname(os.path.dirname(
    os.path.abspath(mission_logger_node_module.__file__)))


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_node(tmp_path, **overrides):
    params = [
        Parameter('runs_dir', Parameter.Type.STRING, str(tmp_path)),
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
        """A run folder in the active/ tree: <run_id>/bag/, per the layout the
        node writes. Returns (run_dir, bag_dir)."""
        run_dir = os.path.join(str(root), 'active', name)
        bag_dir = os.path.join(run_dir, 'bag')
        os.makedirs(bag_dir)
        open(os.path.join(bag_dir, f'{name}_0.db3'), 'w').close()
        if complete:
            open(os.path.join(bag_dir, 'metadata.yaml'), 'w').close()
        return run_dir, bag_dir

    def test_incomplete_bag_is_moved_not_deleted(self, tmp_path):
        """Default sweep_action must not destroy run data: a partial bag is
        often still readable directly, and the run cannot be re-collected."""
        node = _make_node(tmp_path)
        run_dir, _ = self._bag(tmp_path, 'run_incomplete', complete=False)
        node._sweep_incomplete_bags(reason='test')
        assert not os.path.exists(run_dir)
        assert os.path.isdir(os.path.join(str(tmp_path), 'incomplete', 'run_incomplete'))
        node.destroy_node()

    def test_sweep_moves_the_whole_run_folder_not_just_the_bag(self, tmp_path):
        """The manifest and params snapshot are the most valuable part of a
        dead run -- quarantining the bag and orphaning its sidecars would
        discard exactly what explains why the recorder died."""
        node = _make_node(tmp_path)
        run_dir, _ = self._bag(tmp_path, 'run_incomplete', complete=False)
        open(os.path.join(run_dir, 'run_incomplete.manifest.json'), 'w').close()
        node._sweep_incomplete_bags(reason='test')
        quarantined = os.path.join(str(tmp_path), 'incomplete', 'run_incomplete')
        assert os.path.isfile(os.path.join(quarantined, 'run_incomplete.manifest.json'))
        assert os.path.isdir(os.path.join(quarantined, 'bag'))
        node.destroy_node()

    def test_complete_bag_is_left_alone(self, tmp_path):
        node = _make_node(tmp_path)
        run_dir, _ = self._bag(tmp_path, 'run_complete', complete=True)
        node._sweep_incomplete_bags(reason='test')
        assert os.path.isdir(run_dir)
        node.destroy_node()

    def test_in_progress_bag_is_never_swept(self, tmp_path):
        """The active recording has no metadata.yaml yet BY DEFINITION -- it is
        only written on cancel(). Sweeping it would destroy the run in flight."""
        node = _make_node(tmp_path)
        run_dir, bag_dir = self._bag(tmp_path, 'run_active', complete=False)
        node._active_bag_dir = bag_dir
        node._sweep_incomplete_bags(reason='test')
        assert os.path.isdir(run_dir)
        node.destroy_node()

    def test_completed_runs_are_not_swept(self, tmp_path):
        """complete/ is out of the sweep's scope entirely: a finalized run has
        already been renamed out of active/, and the sync unit copies from
        there. Sweeping it would delete archived runs."""
        node = _make_node(tmp_path)
        done = os.path.join(str(tmp_path), 'complete', 'run_done', 'bag')
        os.makedirs(done)
        node._sweep_incomplete_bags(reason='test')
        assert os.path.isdir(done)
        node.destroy_node()

    def test_delete_action_removes(self, tmp_path):
        node = _make_node(tmp_path, sweep_action='delete')
        run_dir, _ = self._bag(tmp_path, 'run_incomplete', complete=False)
        node._sweep_incomplete_bags(reason='test')
        assert not os.path.exists(run_dir)
        assert not os.path.exists(os.path.join(str(tmp_path), 'incomplete'))
        node.destroy_node()


class TestRunMetadata:

    def _run(self, tmp_path):
        run_dir = os.path.join(str(tmp_path), 'active', 'RUN')
        os.makedirs(run_dir, exist_ok=True)
        return {
            'mission_id': 'bottle_then_person', 'run_id': 'RUN',
            'start_time': 'T0', 'end_time': None, 'outcome': None,
            'run_dir': run_dir,
            'bag_path': os.path.join(run_dir, 'bag'),
            'params_snapshot_path': os.path.join(run_dir, 'RUN.params.yaml'),
            'manifest_path': os.path.join(run_dir, 'RUN.manifest.json'),
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


class TestSingletonLock:
    """The 2026-09-04 double-record: the supervisor's auto-started logger and a
    hand-run `ros2 run` one both recorded the same mission into one bag
    directory, leaving 31 topics and 0 messages. The guard is a PID file, so
    these tests are pure filesystem -- no ROS, no processes spawned."""

    def test_first_caller_acquires(self, tmp_path):
        lock = str(tmp_path / 'l.lock')
        assert acquire_singleton_lock(lock) == (True, None)
        assert os.path.isfile(lock)

    def test_second_caller_is_refused_and_told_who_holds_it(self, tmp_path):
        lock = str(tmp_path / 'l.lock')
        acquire_singleton_lock(lock)
        ok, holder = acquire_singleton_lock(lock)
        assert ok is False
        assert holder == os.getpid(), 'the refusal must name the live holder'

    def test_a_second_os_process_is_refused(self, tmp_path):
        """The real shape of the bug: two separate processes, not one calling
        twice. The child holds the lock while the parent tries to take it."""
        lock = str(tmp_path / 'l.lock')
        ready = str(tmp_path / 'ready')
        child = subprocess.Popen([
            sys.executable, '-c',
            'import os,sys,time\n'
            'sys.path.insert(0, os.environ["PKG_PATH"])\n'
            'from f1tenth_logger.mission_logger_node import acquire_singleton_lock\n'
            'ok, _ = acquire_singleton_lock(sys.argv[1])\n'
            'assert ok\n'
            'open(sys.argv[2], "w").close()\n'
            'time.sleep(30)\n',
            lock, ready],
            env={**os.environ, 'PKG_PATH': _PKG_PATH})
        try:
            for _ in range(200):
                if os.path.exists(ready):
                    break
                time.sleep(0.05)
            assert os.path.exists(ready), 'child never claimed the lock'
            ok, holder = acquire_singleton_lock(lock)
            assert ok is False
            assert holder == child.pid
        finally:
            child.kill()
            child.wait()

    def test_stale_lock_from_a_dead_pid_is_reclaimed(self, tmp_path):
        """A logger SIGKILLed mid-run cannot clean up after itself. If a stale
        lock blocked forever, one crash would disable recording for the rest of
        the machine's uptime -- worse than the bug being guarded against."""
        lock = str(tmp_path / 'l.lock')
        with open(lock, 'w') as fh:
            fh.write(str(_DEAD_PID))
        assert acquire_singleton_lock(lock) == (True, None)
        assert _read_lock_pid(lock) == os.getpid()

    def test_garbage_lock_file_is_reclaimed_not_fatal(self, tmp_path):
        """A truncated write (power loss mid-claim) must not wedge the logger."""
        lock = str(tmp_path / 'l.lock')
        with open(lock, 'w') as fh:
            fh.write('not-a-pid')
        assert acquire_singleton_lock(lock) == (True, None)

    def test_release_removes_our_own_lock(self, tmp_path):
        lock = str(tmp_path / 'l.lock')
        acquire_singleton_lock(lock)
        release_singleton_lock(lock)
        assert not os.path.exists(lock)

    def test_release_leaves_a_lock_owned_by_someone_else(self, tmp_path):
        """After we are declared stale and a successor reclaims the lock, our
        late cleanup must not unlink the live logger's claim."""
        lock = str(tmp_path / 'l.lock')
        with open(lock, 'w') as fh:
            fh.write(str(_DEAD_PID))
        release_singleton_lock(lock)
        assert os.path.exists(lock)


class TestMoveToComplete:

    def _staged(self, node, tmp_path, run_id='RUN'):
        run_dir = os.path.join(node.active_dir, run_id)
        os.makedirs(os.path.join(run_dir, 'bag'))
        open(os.path.join(run_dir, f'{run_id}.manifest.json'), 'w').close()
        return {
            'run_id': run_id, 'run_dir': run_dir,
            'bag_path': os.path.join(run_dir, 'bag'),
            'params_snapshot_path': os.path.join(run_dir, f'{run_id}.params.yaml'),
            'manifest_path': os.path.join(run_dir, f'{run_id}.manifest.json'),
        }

    def test_run_moves_whole_out_of_active(self, tmp_path):
        node = _make_node(tmp_path)
        run = self._staged(node, tmp_path)
        dest = node._move_to_complete(run)
        assert dest == os.path.join(node.complete_dir, 'RUN')
        assert not os.path.exists(os.path.join(node.active_dir, 'RUN')), \
            'a finalized run must not be left in active/ too'
        assert os.path.isdir(os.path.join(dest, 'bag'))
        assert os.path.isfile(os.path.join(dest, 'RUN.manifest.json'))
        node.destroy_node()

    def test_paths_are_repointed_at_the_new_home(self, tmp_path):
        """Finalize writes the manifest and extract AFTER the move -- if these
        still pointed into active/ it would resurrect the directory the run was
        just moved out of."""
        node = _make_node(tmp_path)
        run = self._staged(node, tmp_path)
        node._move_to_complete(run)
        for key in ('run_dir', 'bag_path', 'params_snapshot_path', 'manifest_path'):
            assert run[key].startswith(node.complete_dir), f'{key} still in active/'
        node.destroy_node()

    def test_a_colliding_run_id_is_filed_beside_never_merged(self, tmp_path):
        node = _make_node(tmp_path)
        os.makedirs(os.path.join(node.complete_dir, 'RUN'))
        run = self._staged(node, tmp_path)
        dest = node._move_to_complete(run)
        assert dest != os.path.join(node.complete_dir, 'RUN')
        assert os.path.isdir(os.path.join(node.complete_dir, 'RUN'))
        node.destroy_node()


class TestSnapshotSpacing:

    def test_fewer_messages_than_the_cap_takes_all_of_them(self):
        assert _snapshot_indices(3, 10) == [0, 1, 2]

    def test_spacing_spans_the_whole_run(self):
        """First AND last: the end of an aborted run is usually the part worth
        looking at, so snapshots must not cluster at the start."""
        idx = _snapshot_indices(100, 5)
        assert idx[0] == 0
        assert idx[-1] == 99
        assert len(idx) == 5

    def test_empty_and_degenerate_inputs_do_not_raise(self):
        assert _snapshot_indices(0, 10) == []
        assert _snapshot_indices(10, 0) == []
        assert _snapshot_indices(10, 1) == [0]
