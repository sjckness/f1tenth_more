"""Automatic per-mission rosbag2 recorder, keyed to mission lifecycle.

Records every mission run to its own bag with no operator action -- no manual
`ros2 bag record` to remember, no launch flag, no fixed timer. Subscribes to
f1tenth_behavior's own /mission/status (f1tenth_messages/MissionStatus,
transient-local) and starts/stops recording on the real state transitions:

    -> RUNNING                         start recording
    RUNNING/HOLDING -> COMPLETE        stop, outcome=COMPLETE   (success)
    RUNNING/HOLDING -> ABORTED         stop, outcome=ABORTED    (abort/e-stop)
    RUNNING/HOLDING -> IDLE/LOADED     stop, outcome=<state>    (reload/reset)
    emergency_stop_active goes True    stop, outcome=EMERGENCY_STOP

HOLDING deliberately does NOT stop recording -- it is a mid-mission pause
(the BT's own hold lane), not an end state, and the hold period is usually
the most interesting part of a run to look at afterwards.

DEPENDS ON A FIX MADE FOR THIS NODE, in f1tenth_behavior/mission/loader.py:
three of the four ways a mission can reach a terminal state (advance_move's
state.complete(), handle_object_action's and check_stop_condition's
state.abort()) happen inside a BT tick against MissionRuntimeState, which
holds no publisher -- so they never republished /mission/status at all, and
only an operator calling /mission/abort_mission did. A logger hooked to this
topic would have started on RUNNING and then never stopped, silently losing
every failure run. See that file's own MISSION-END EVENT GAP FIX comment.
Without that fix this node cannot work correctly, and no amount of logic
here can compensate -- the event simply does not exist to subscribe to.

WHY rosbag2_py.Recorder AND NOT `ros2 bag record` AS A SUBPROCESS:
  - start/stop is a method call tied directly to the status callback, not
    process signalling.
  - no second orphaned-process class. This box already accumulates orphaned
    /dev/shm fastrtps segments across restart cycles (enough, in one session,
    to break DDS discovery outright for new participants); a recorder
    subprocess that outlives an unclean shutdown would be the same failure
    shape again, on a system that has already demonstrated it.
  - the bag is opened at the same point the run metadata is written, so the
    two cannot disagree about which bag belongs to which run.
Recorder.record() blocks (it spins its own executor) but releases the GIL --
verified live on this box, not assumed -- so it runs on a worker thread and
cancel() stops it from the callback thread. cancel() is a CLEAN shutdown and
is what writes metadata.yaml; this node never kills the recorder any harder
than that, for the same reason the spec calls out SIGINT-not-SIGKILL for the
subprocess approach: readers and post-hoc analysis trust metadata.yaml, and a
bag without it reports wrong counts/duration or refuses to open.

STORAGE FORMAT: mcap requested by default, BUT NOT AVAILABLE ON THIS BOX AS
SHIPPED -- rosbag2_py.get_registered_writers() returns {'sqlite3'} only; the
rosbag2_storage_mcap plugin is not installed (checked live). Rather than fail
every mission recording on a missing plugin, _resolve_storage_id() below
checks the registered writers at startup and falls back to sqlite3 with a
loud one-time WARNING naming the exact package to install
(ros-humble-rosbag2-storage-mcap). Install it and this node switches to mcap
on the next start with no code or config change. The fallback is deliberately
noisy rather than silent: sqlite3 bags are what Foxglove reads least well, so
"it recorded fine" must not quietly mean "in the format you didn't want".

QoS -- THE TRAP THIS NODE EXPLICITLY GUARDS AGAINST: rosbag2's recorder does
not simply inherit a publisher's QoS, and a recorder subscribing RELIABLE to
a BEST_EFFORT publisher records NOTHING while looking completely healthy --
a gap that only surfaces during analysis, after the run is gone.
semantic_layer_node and costmap_boundary_node both publish BEST_EFFORT/
VOLATILE (see those files), as does anything matching MPC_corr.py's own odom
QoS precedent. best_effort_topics below is therefore an explicit override
list, passed through RecordOptions.topic_qos_profile_overrides; rosbag2 logs
'Overriding subscription profile for <topic>' per entry, which is the line to
grep for when confirming a run recorded what it should. Verified live: a
BEST_EFFORT topic recorded 20/20 messages with the override in place.

DISCOVERY LATENCY -- A REAL FLOOR ON USABLE MISSION LENGTH, measured live on
this box, not theorised: rosbag2's recorder does not subscribe instantly. It
opens the bag, logs 'Listening for topics...', and only then discovers and
subscribes, which took 2.4-3.4s across repeated runs here before the first
'Subscribed to topic' line appeared. A mission shorter than that records an
EMPTY but otherwise perfectly valid bag -- metadata.yaml present, message
count 0 -- which is exactly the kind of silent gap this node is supposed to
prevent, so it is called out rather than left to be rediscovered. Found the
hard way: a first live verification with a ~5s window recorded nothing at all
and looked like a QoS or graph-introspection bug; it was neither, just a
window barely longer than discovery. Real missions run far longer than this
and are unaffected, but do not trust a sub-5s run to have captured anything,
and do not use an empty bag from a very short run as evidence that recording
is broken. (rosbag2 logs 'All requested topics are subscribed. Stopping
discovery...' once it has them all -- that line is the honest 'recording is
actually live now' marker.)

Topics that do not exist at record time (e.g. /camera/detection_masks, which
only a -seg yolo_model publishes) are NOT an error -- rosbag2 keeps polling
for them and simply records nothing if they never appear. Verified live.

RUN METADATA, written ALONGSIDE the bag rather than inside it, so an analysis
script can enumerate and filter runs without opening a single bag:

    <bag_root>/2026-09-01T14-32-05_mission-<name>/          <- the bag itself
    <bag_root>/2026-09-01T14-32-05_mission-<name>.manifest.json
    <bag_root>/2026-09-01T14-32-05_mission-<name>.params.yaml

The manifest is written TWICE: once at start (outcome/end_time null) so a run
that dies mid-record still leaves an enumerable record of itself, and again at
stop with the real outcome. params.yaml is a byte copy of the resolved
stack_params.yaml actually in effect -- this session's real bugs were
one-line param values (yolo_model_task segment-vs-detect, a missing
initial_state), which no amount of bag data would have explained on its own,
so the params snapshot is often the single most valuable artifact here. The
manifest also carries the git commit AND a dirty flag, because a commit hash
alone describes the wrong tree whenever there are working-tree changes -- as
there were for every fix this session.

KNOWN LIMITATION, stated rather than papered over: resolved_params below is
read from stack_params.yaml, which captures the 4 stack-wide branching args
and every other key correctly, but does NOT capture a per-node launch-CLI
override (`enable_intelligence:=false` on the supervisor, say). Those are not
introspectable from another process without querying every node's parameters
individually. Treat resolved_params as "what stack_params.yaml said at record
time", which is exactly true, not as "every argument every node was started
with".

SWEEPING INCOMPLETE BAGS: a bag directory with no metadata.yaml is one whose
recorder died mid-record. Swept at startup AND on a periodic timer -- both,
deliberately: the /dev/shm orphan sweep in component_supervisor_node runs at
startup only, and that is exactly why orphans still accumulated to the point
of breaking discovery during a single long session. Same failure class, so
the fix gets the periodic half the other one is missing. Swept bags are MOVED
into <bag_root>/incomplete/, NOT deleted (sweep_action:='delete' opts into
deletion): a partial bag file is frequently still readable directly by mcap/
sqlite tooling even with no metadata.yaml, and destroying real run data that
cannot be re-collected is not a safe default for a janitor that runs on a
timer. The in-progress bag is always excluded by path.

Nothing in this node is allowed to take down a mission: every recorder,
filesystem, and metadata operation is wrapped, and a failure logs a clear
error and leaves the mission untouched. A missed recording is a bad day; a
mission node crashing mid-run because its logger could not open a file is a
much worse one.
"""

import datetime
import json
import os
import shutil
import subprocess
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

import rosbag2_py

from f1tenth_messages.msg import MissionStatus


# Mid-mission states: recording runs through both. HOLDING is a pause, not an
# end -- see module docstring.
_ACTIVE_STATES = frozenset({'RUNNING', 'HOLDING'})

# Must match f1tenth_behavior/mission/loader.py's own MISSION_STATUS_QOS --
# durability has to match on both ends or the "late subscriber still gets the
# current value" latch does not happen, and this node would miss a mission
# that was already RUNNING when it started.
_MISSION_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# Everything needed to reconstruct both what the robot was asked to do and what
# it actually perceived/estimated while doing it. Topics absent at record time
# are harmless (see module docstring).
_DEFAULT_TOPICS = [
    # --- localization / estimation ---
    '/odometry/filtered',                 # LOCAL EKF (odom frame). NOT
                                          # /ekf_local/... -- that name does
                                          # not exist in this stack; the local
                                          # EKF publishes unnamespaced.
    '/ekf_global/odometry/filtered',      # GLOBAL EKF (map frame)
    '/slam/pose',
    '/slam/pose_calibrated',
    '/odom',
    '/tf',
    '/tf_static',
    # --- perception ---
    '/camera/image_annotated',
    '/camera/detections',
    '/camera/detections_3d',
    '/camera/detection_masks',            # -seg configs only
    '/camera/detection_markers',
    '/scan',
    # --- costmap / semantic ---
    '/costmap/semantic_markers',
    '/costmap/boundaries',
    '/costmap/front_clearance',
    '/slam/map',
    # --- mission / command: what the robot was actually asked to do ---
    '/mission/status',
    '/mpc/goal_pose',
    '/mpc/goal_distance',
    '/mpc/goal_turn',                     # turn moves (TurnGoal). Was missing:
                                          # a mission's turn legs were invisible.
    '/mpc/hold',
    '/drive',                             # what the MPC ASKED for
    '/ackermann_drive',                   # what the VESC actually got
    # THE topic that explains a stop. Its omission was the single biggest gap
    # found by the 2026-09-01 analysis: 35% of all actuated samples came from
    # this lane (ackermann_mux priority 200, beating /drive's 10), and with it
    # unrecorded every stop had to be re-bucketed offline by replaying each BT
    # condition's predicate against recorded sensor data. Recording it makes
    # "who stopped the car" a direct read. The two Stop instances now stamp
    # distinct frame_ids ('base_link/emergency' vs 'base_link/obstacle', see
    # behavior_executor_node.create_root) so the source is unambiguous.
    '/safety_stop',
    # Which BT lane won each tick and which emergency condition tripped --
    # previously only in the per-component supervisor log, which is overwritten
    # on every supervisor restart (during the 2026-09-01 analysis it covered
    # only the window AFTER the last mission, so none of it survived).
    '/behavior/tree_status',
    # Solver convergence/timing per tick. Previously logger-only, recoverable
    # solely by parsing ~/.ros/log before rotation discarded it.
    '/mpc/solver_status',
    # The MPC's own reference corridor geometry (build_straight_corridor's
    # left/right Bezier wall polylines + centerline, MarkerArray, odom frame).
    # NOT the same thing as /costmap/boundaries: those are 3 hard halfspace
    # constraints, this is the widening funnel the solver's soft corridor cost
    # is shaped by -- the thing Foxglove's 3D panel shows. Previously live-only:
    # the offline replay video had nothing but the halfspaces to draw, so the
    # corridor appeared as three straight edge-to-edge lines instead of the
    # actual funnel. Recording it costs three ~120-point LINE_STRIPs at the
    # corridor rebuild rate.
    '/mpc/corridor_markers',
    # The camera -> MPC soft-avoidance path's own payload. With the camera
    # e-stop lane disabled this is how camera-seen obstacles reach the
    # controller at all, so a run that fails to avoid something is not
    # diagnosable without it.
    '/perception/obstacles_2d',
    # --- health, for correlating hz drops/dropped frames post-hoc ---
    '/diagnostics',
    '/diagnostics/system_status',
]

# Publishers using BEST_EFFORT/VOLATILE. Recording these without an explicit
# override silently captures nothing -- see module docstring's QoS paragraph.
_DEFAULT_BEST_EFFORT_TOPICS = [
    '/costmap/boundaries',
    '/costmap/front_clearance',
    '/odometry/filtered',
    '/ekf_global/odometry/filtered',
    '/odom',
]

# Snapshotted into the manifest for at-a-glance run comparison. The full
# stack_params.yaml copy is the authoritative record; this is the shortlist an
# analysis script can group runs by without parsing yaml.
_SNAPSHOT_PARAM_KEYS = [
    'camera_source', 'localization_source', 'use_behavior_tree', 'enable_nav2',
    'enable_slam', 'enable_intelligence', 'enable_sys_obs',
    'yolo_model', 'yolo_model_task', 'use_mask_depth', 'confidence_threshold',
]


def _utc_stamp():
    """Filesystem-safe UTC timestamp, sorts lexicographically by time."""
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H-%M-%S')


def _best_effort_qos(depth):
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _stack_params_path():
    """Real path of the stack_params.yaml actually in effect. realpath()
    matters: --symlink-install (this workspace's default) makes the installed
    copy a symlink back into src/, and the src file is the one whose content
    should be snapshotted and whose git state is meaningful."""
    return os.path.realpath(os.path.join(
        get_package_share_directory('f1tenth_params'), 'config', 'stack_params.yaml'))


def _find_git_root(start_path):
    """Walk up from `start_path` to the first directory containing .git."""
    d = os.path.dirname(start_path)
    while True:
        if os.path.isdir(os.path.join(d, '.git')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent


class MissionLoggerNode(Node):

    def __init__(self, **kwargs):
        super().__init__('mission_logger_node', **kwargs)

        default_bag_root = os.path.join(os.path.expanduser('~'), '.ros', 'mission_bags')
        self.bag_root = str(self.declare_parameter('bag_root', default_bag_root).value)
        self.requested_storage_id = str(self.declare_parameter('storage_id', 'mcap').value)
        self.topics = list(self.declare_parameter('topics', _DEFAULT_TOPICS).value)
        self.best_effort_topics = list(self.declare_parameter(
            'best_effort_topics', _DEFAULT_BEST_EFFORT_TOPICS).value)
        self.qos_depth = int(self.declare_parameter('qos_override_depth', 10).value)
        self.sweep_period_sec = float(self.declare_parameter('sweep_period_sec', 300.0).value)
        # 'move' (default) relocates into <bag_root>/incomplete/; 'delete'
        # removes outright -- opt-in only, see module docstring.
        self.sweep_action = str(self.declare_parameter('sweep_action', 'move').value)
        status_topic = str(self.declare_parameter('status_topic', '/mission/status').value)

        self.storage_id = self._resolve_storage_id()

        # Recorder lifecycle. _lock guards all of these together: the status
        # callback (executor thread) and the recorder worker thread both touch
        # them.
        self._lock = threading.RLock()
        self._recorder = None
        self._record_thread = None
        self._active_bag_dir = None
        self._active_run = None      # dict -> becomes the manifest

        self._last_state = None
        self._last_estop = False

        try:
            os.makedirs(self.bag_root, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(
                f'could not create bag_root "{self.bag_root}": {exc} -- recording will '
                'fail until this is fixed, but missions are unaffected.')

        self._sweep_incomplete_bags(reason='startup')
        self._sweep_timer = self.create_timer(
            self.sweep_period_sec, lambda: self._sweep_incomplete_bags(reason='periodic'))

        self._status_sub = self.create_subscription(
            MissionStatus, status_topic, self._on_status, _MISSION_STATUS_QOS)

        self.get_logger().info(
            f'mission_logger_node up: watching "{status_topic}", bags -> '
            f'"{self.bag_root}" (storage={self.storage_id}), {len(self.topics)} topics '
            f'({len(self.best_effort_topics)} with BEST_EFFORT overrides), '
            f'sweep every {self.sweep_period_sec:.0f}s (action={self.sweep_action}).')

    # -- storage -----------------------------------------------------------
    def _resolve_storage_id(self):
        """Requested storage plugin if actually registered, else sqlite3 with a
        loud warning -- see module docstring's STORAGE FORMAT paragraph."""
        try:
            available = set(rosbag2_py.get_registered_writers())
        except Exception as exc:  # noqa: BLE001 - never block startup on this
            self.get_logger().warn(
                f'could not query rosbag2 storage plugins ({exc}) -- using '
                f'"{self.requested_storage_id}" as requested and hoping for the best.')
            return self.requested_storage_id
        if self.requested_storage_id in available:
            return self.requested_storage_id
        fallback = 'sqlite3' if 'sqlite3' in available else next(iter(available), 'sqlite3')
        self.get_logger().warn(
            f'storage_id "{self.requested_storage_id}" is NOT a registered rosbag2 '
            f'writer on this system (registered: {sorted(available)}) -- falling back to '
            f'"{fallback}". For mcap: sudo apt install ros-humble-rosbag2-storage-mcap, '
            'then restart this node; no config change needed.')
        return fallback

    # -- mission lifecycle -------------------------------------------------
    def _on_status(self, msg: MissionStatus):
        """Start/stop recording on real mission transitions. Wrapped whole:
        this callback must never raise into the executor."""
        try:
            state = str(msg.state)
            estop_latched = bool(msg.emergency_stop_active) and not self._last_estop

            if state in _ACTIVE_STATES and self._active_run is None:
                self._start_recording(msg)
            elif self._active_run is not None and (
                    state not in _ACTIVE_STATES or estop_latched):
                outcome = 'EMERGENCY_STOP' if estop_latched else state
                self._stop_recording(outcome)

            self._last_state = state
            self._last_estop = bool(msg.emergency_stop_active)
        except Exception as exc:  # noqa: BLE001 - a logger must not kill anything
            self.get_logger().error(
                f'mission status handling failed: {exc} -- mission is unaffected.')

    def _start_recording(self, msg: MissionStatus):
        with self._lock:
            if self._active_run is not None:
                return
            mission_name = os.path.splitext(os.path.basename(str(msg.json_path)))[0] or 'unnamed'
            run_id = f'{_utc_stamp()}_mission-{mission_name}'
            bag_dir = os.path.join(self.bag_root, run_id)

            run = {
                'mission_id': mission_name,
                'run_id': run_id,
                'start_time': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'end_time': None,
                'outcome': None,
                'bag_path': bag_dir,
                'params_snapshot_path': os.path.join(self.bag_root, f'{run_id}.params.yaml'),
                'manifest_path': os.path.join(self.bag_root, f'{run_id}.manifest.json'),
                'storage_id': self.storage_id,
                'mission_json_path': str(msg.json_path),
            }

            try:
                storage_options = rosbag2_py.StorageOptions(
                    uri=bag_dir, storage_id=self.storage_id)
                record_options = rosbag2_py.RecordOptions()
                record_options.all = False
                record_options.topics = list(self.topics)
                record_options.is_discovery_disabled = False
                record_options.rmw_serialization_format = 'cdr'
                record_options.topic_polling_interval = datetime.timedelta(milliseconds=100)
                record_options.topic_qos_profile_overrides = {
                    t: _best_effort_qos(self.qos_depth) for t in self.best_effort_topics}

                recorder = rosbag2_py.Recorder()

                def _run():
                    try:
                        recorder.record(storage_options, record_options)
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().error(f'recorder thread failed: {exc}')

                thread = threading.Thread(
                    target=_run, name=f'mission_bag_{run_id}', daemon=True)
                thread.start()

                self._recorder = recorder
                self._record_thread = thread
                self._active_bag_dir = bag_dir
                self._active_run = run
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'FAILED to start recording for "{mission_name}": {exc} -- the mission '
                    'runs normally, but this run will not be captured.')
                self._recorder = None
                self._record_thread = None
                self._active_bag_dir = None
                self._active_run = None
                return

        # Sidecars written outside the lock -- filesystem work, and the manifest
        # is deliberately written NOW as well as at stop, so a run that dies
        # mid-record still leaves an enumerable record (see module docstring).
        self._write_params_snapshot(run)
        self._write_manifest(run)
        self.get_logger().info(f'recording "{run["run_id"]}" -> {bag_dir}')

    def _stop_recording(self, outcome):
        with self._lock:
            recorder, thread, run = self._recorder, self._record_thread, self._active_run
            self._recorder = None
            self._record_thread = None
            self._active_bag_dir = None
            self._active_run = None
        if run is None:
            return
        try:
            if recorder is not None:
                # CLEAN stop -- this is what writes metadata.yaml.
                recorder.cancel()
            if thread is not None:
                thread.join(timeout=15.0)
                if thread.is_alive():
                    self.get_logger().warn(
                        f'recorder thread for "{run["run_id"]}" did not exit within 15s; '
                        'the bag may be missing metadata.yaml and will be swept as '
                        'incomplete.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'error stopping recorder: {exc}')

        run['outcome'] = outcome
        run['end_time'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        self._write_manifest(run)
        self.get_logger().info(
            f'stopped recording "{run["run_id"]}" (outcome={outcome}) -> {run["bag_path"]}')

    # -- run metadata ------------------------------------------------------
    def _write_params_snapshot(self, run):
        try:
            shutil.copyfile(_stack_params_path(), run['params_snapshot_path'])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'could not snapshot stack_params.yaml: {exc}')

    def _write_manifest(self, run):
        payload = dict(run)
        payload['resolved_params'] = self._resolved_params()
        payload['git'] = self._git_state()
        try:
            tmp = run['manifest_path'] + '.tmp'
            with open(tmp, 'w') as fh:
                json.dump(payload, fh, indent=2, sort_keys=True)
            os.replace(tmp, run['manifest_path'])  # atomic -- never a half-written manifest
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'could not write manifest: {exc}')

    def _resolved_params(self):
        """Shortlist of stack_params values in effect -- see the KNOWN
        LIMITATION paragraph in the module docstring about launch-CLI
        overrides."""
        out = {}
        try:
            from f1tenth_params.param_defaults import get_value
            for key in _SNAPSHOT_PARAM_KEYS:
                try:
                    out[key] = get_value(key)
                except Exception:  # noqa: BLE001 - a renamed/removed key is not fatal
                    out[key] = None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'could not read resolved params: {exc}')
        return out

    def _git_state(self):
        """Commit AND dirty flag -- a hash alone names the wrong tree whenever
        there are uncommitted working-tree changes."""
        info = {'commit': None, 'dirty': None, 'branch': None}
        root = _find_git_root(_stack_params_path())
        if root is None:
            return info

        def _git(*args):
            return subprocess.run(
                ['git', '-C', root, *args],
                capture_output=True, text=True, timeout=10).stdout.strip()

        try:
            info['commit'] = _git('rev-parse', 'HEAD') or None
            info['branch'] = _git('rev-parse', '--abbrev-ref', 'HEAD') or None
            info['dirty'] = bool(_git('status', '--porcelain'))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'could not read git state: {exc}')
        return info

    # -- housekeeping ------------------------------------------------------
    def _sweep_incomplete_bags(self, reason):
        """Relocate (or delete) bag directories with no metadata.yaml -- see the
        SWEEPING paragraph in the module docstring."""
        try:
            if not os.path.isdir(self.bag_root):
                return
            quarantine = os.path.join(self.bag_root, 'incomplete')
            with self._lock:
                active = self._active_bag_dir
            swept = []
            for name in sorted(os.listdir(self.bag_root)):
                path = os.path.join(self.bag_root, name)
                if not os.path.isdir(path) or name == 'incomplete':
                    continue
                if active is not None and os.path.abspath(path) == os.path.abspath(active):
                    continue  # never touch the in-progress bag
                if os.path.isfile(os.path.join(path, 'metadata.yaml')):
                    continue
                try:
                    if self.sweep_action == 'delete':
                        shutil.rmtree(path)
                    else:
                        os.makedirs(quarantine, exist_ok=True)
                        dest = os.path.join(quarantine, name)
                        if os.path.exists(dest):
                            dest = f'{dest}.{_utc_stamp()}'
                        shutil.move(path, dest)
                    swept.append(name)
                except OSError as exc:
                    self.get_logger().warn(f'could not sweep incomplete bag "{name}": {exc}')
            if swept:
                verb = 'deleted' if self.sweep_action == 'delete' else 'moved to incomplete/'
                self.get_logger().warn(
                    f'bag sweep ({reason}): {len(swept)} incomplete bag(s) {verb} '
                    f'(no metadata.yaml -- recorder died mid-record): {swept}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'bag sweep ({reason}) failed: {exc}')

    def on_shutdown(self):
        """Stop cleanly if the process goes down mid-mission, so the bag still
        gets its metadata.yaml instead of becoming sweep fodder."""
        with self._lock:
            active = self._active_run is not None
        if active:
            self._stop_recording('INTERRUPTED')


def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
