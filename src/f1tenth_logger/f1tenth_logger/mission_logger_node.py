"""Automatic per-mission rosbag2 recorder, keyed to mission lifecycle.

Records every mission run to its own bag with no operator action -- no manual
`ros2 bag record` to remember, no launch flag, no fixed timer. Subscribes to
f1tenth_behavior's own /mission/status (f1tenth_messages/MissionStatus,
transient-local) and starts/stops recording on the real state transitions:

    -> LOADED                          start recording (pre-roll, see below)
    LOADED -> RUNNING                  mark the mission start, keep recording
    -> RUNNING with nothing open       start recording (logger came up late)
    LOADED/RUNNING/HOLDING -> COMPLETE stop, outcome=COMPLETE   (success)
    LOADED/RUNNING/HOLDING -> ABORTED  stop, outcome=ABORTED    (abort/e-stop)
    LOADED/RUNNING/HOLDING -> IDLE     stop, outcome=IDLE       (reset)
    RUNNING/HOLDING -> LOADED          stop, outcome=LOADED, start a new run
    LOADED -> LOADED, other json_path  stop, outcome=LOADED, start a new run
    LOADED for > loaded_timeout_sec    stop, outcome=NOT_STARTED
    emergency_stop_active goes True    stop, outcome=EMERGENCY_STOP

PRE-ROLL: recording starts at LOADED, not at RUNNING, because the recorder
needs 5-14 s of discovery before its first message (first test campaign,
2026-09/10: median 8.2 s after the mission start, 17 of 58 bags empty when it
started at RUNNING). f1tenth_behavior's mission_countdown_sec (LOADED ->
RUNNING) was raised to cover that window, and is 0.0 again since 2026-10-21
(stack_params.yaml), so the pre-roll is only what discovery itself manages. The manifest's start_time still
means the MISSION start (the RUNNING edge): the MATLAB exporter matches runs
on it within 2 s. record_start_time (the LOADED edge), mission_start_time and
preroll_s are additions. A run that never reaches RUNNING keeps
mission_start_time null and start_time = record_start_time.

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
'Subscribed to topic' line appeared (5-14 s on the car under the Discovery
Server -- see PRE-ROLL above). A mission shorter than that records an
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

RUN LAYOUT: one folder per run, in one of two states under
mission_logger_runs_dir (stack_params.yaml, default ~/f1tenth_archive).

    active/<run_id>/bag/                  <- open, being written
    active/<run_id>/<run_id>.manifest.json
    active/<run_id>/<run_id>.params.yaml

and at finalize the whole folder is RENAMED across:

    complete/<run_id>/bag/
    complete/<run_id>/<run_id>.manifest.json     (+ bag_bytes, bag_sha256)
    complete/<run_id>/<run_id>.params.yaml
    complete/<run_id>/<run_id>.extract.parquet
    complete/<run_id>/snapshots/<run_id>.NNN.jpg

A rename is atomic within a filesystem, so a run is never observably
half-present in either state. That is what lets f1tenth-archive.service rsync
complete/ unconditionally: "safe to copy" is answered by which directory the
run is in, not by a filter that has to correctly guess whether a .db3 is still
being appended to. Metadata sits alongside the bag rather than inside it so an
analysis script can enumerate and filter runs without opening a single bag.

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
into <runs_dir>/incomplete/, NOT deleted (sweep_action:='delete' opts into
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
import sys
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

import rosbag2_py

from f1tenth_messages.msg import MissionStatus


# States recording runs through. LOADED is the pre-roll (the start countdown),
# HOLDING is a pause, not an end -- see module docstring.
_RECORD_STATES = frozenset({'LOADED', 'RUNNING', 'HOLDING'})
_STARTED_STATES = frozenset({'RUNNING', 'HOLDING'})


def _decide(state, json_path, estop_latched, run, refused_json_path=None):
    """
    Return (action, outcome) for one /mission/status message.

    action is one of 'none', 'start', 'mark_started', 'stop', 'restart'
    (stop with outcome, then start). ``run`` is the open run dict or None.
    ``refused_json_path`` is the mission whose LOADED pre-roll already timed
    out: its LOADED republishes must not open a new run.
    """
    if run is None:
        if state == 'LOADED' and json_path == refused_json_path:
            return 'none', None
        return ('start', None) if state in _RECORD_STATES else ('none', None)
    if estop_latched:
        return 'stop', 'EMERGENCY_STOP'
    started = bool(run.get('mission_start_time'))
    if state == 'LOADED':
        if started or json_path != run.get('mission_json_path', json_path):
            return 'restart', 'LOADED'
        return 'none', None
    if state in _STARTED_STATES:
        return ('none', None) if started else ('mark_started', None)
    return 'stop', state


def _loaded_timed_out(run, now, timeout_s):
    """Tell whether an open run has sat at LOADED past timeout_s (<= 0: never)."""
    if run is None or timeout_s <= 0.0 or run.get('mission_start_time'):
        return False
    started = run.get('record_start_time')
    if not started:
        return False
    waited = (now - datetime.datetime.fromisoformat(started)).total_seconds()
    return waited > timeout_s


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
    # /camera/image_annotated (raw bgr8, 75% of a bag) is NOT in this list:
    # the 5 Hz debug video below shows the same detections. The
    # record_annotated_image param adds it back.
    '/camera/detections',
    '/camera/detections_3d',
    '/camera/detection_masks',            # -seg configs only
    '/camera/detection_markers',
    # The perception debug video (yolo_detector_node, ~16 KiB JPEG at 5 Hz)
    # and the wall/clearance intermediates behind it (front_clearance_node,
    # JSON per depth frame). Turned into an MP4 on the host by
    # mission_debug_video; /perception/front_distance itself is not recorded,
    # so the JSON is also the only record of the mission wall guard's input.
    '/camera/debug_image/compressed',
    '/perception/debug/front_wall',
    # Scene labels (enable_scene_labels, f1tenth_perception/scene_labels.py):
    # the virtual wall every wall consumer reads, its source/flags, the LiDAR
    # wall planes, the stereo WALL distance, the label grid (160 x 90 mono8,
    # ~14 KB at 5 Hz), its debug JSON and the unclassified OTHER clusters.
    # Absent (harmlessly) while the switch is off.
    '/perception/virtual_wall',
    '/perception/virtual_wall/debug',
    '/perception/wall_planes',
    # The multi-wall tracker (use_wall_tracker): up to 4 wall tracks in odom
    # (f1tenth_messages/MultiWallTrackArray, its front track is the heading
    # wall of the virtual wall) and their line/text markers.
    '/perception/walls',
    '/perception/walls/markers',
    # Door detection (lidar_front_wall_node, door_detection_enabled): the
    # doors (f1tenth_messages/DoorArray) and their Foxglove markers, so a
    # recorded run shows the doors exactly as the live view did. /viz/doors
    # is an ORIGINAL, not a relay copy: see _RECORDED_VIZ_ORIGINALS.
    '/perception/doors',
    '/viz/doors',
    '/perception/stereo_wall',
    '/perception/scene_labels',
    '/perception/scene_labels_debug',
    '/perception/unclassified_obstacles',
    # The whole-frame scene mask (scene_mask.py, front_clearance_scene_mask_
    # enabled): the per-class pixel fractions and the WALL point count in the
    # car-width strip, plus the WALL points themselves in base_link. The
    # overlay image is NOT recorded by default -- it is only rendered when
    # something subscribes, and a bgr8 frame at 5 Hz is ~3.5 MB/s.
    '/perception/scene_labels/stats',
    # The scene mask's own liveness (front_clearance_node's watchdog): the
    # three ages that tell a camera stall apart from a node stall, published
    # on a timer so it keeps arriving exactly when the mask does not.
    '/perception/scene_labels/health',
    '/perception/wall_points',
    # The per-cycle fusion: the stereo's column profile and the stateless wall
    # observations the tracker consumes. Both are small and both are the
    # evidence behind every wall decision a run made.
    '/perception/stereo_columns',
    '/perception/wall_observations',
    '/scan',
    # --- costmap / semantic ---
    '/costmap/semantic_markers',
    # The wall layer: confirmed walls as map-frame cells carrying their role,
    # and the track id per cell beside it (wall_raster.py).
    '/costmap/wall_cells',
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
    # go_to_object: the handler's goal, its end, mpc_corr's per-tick approach
    # geometry and flags, the scored outcome of every move, and the tracks the
    # handler chose from. Without these a go_to_object run is not diagnosable
    # from its bag (the outcome was only in the on-car mission_reports JSON).
    '/mpc/goal_object',
    '/mpc/goal_object_end',
    '/mpc/object_status',
    '/mission/move_outcome',
    '/costmap/semantic_tracks',
    # The semantic layer's objects with their memory state (TRACKED/MEMORY,
    # confidence, in_fov): what a remembered object looked like to the MPC.
    '/costmap/semantic_objects',
    # Every tick mpc_corr's /drive speed clamp changed a command.
    '/mpc/drive_clamp',
    # allow_reverse only: per-candidate scores, the winner and why (JSON).
    '/mpc/reverse/debug',
    # The reverse manoeuvre: mode, stuck reason and timer, rear readiness,
    # BT permission, attempts, termination reason -- every control tick.
    '/mpc/reverse_status',
    # --- the speed loop, end to end (output/speed_gap/report.md section 4,
    # offset_free.md) -- all small, ~10-50 Hz ---
    # mpc_corr's speed law on every /drive: the disturbance d, the command in
    # force, the estimator state and why it froze (SpeedChannelStatus).
    '/mpc/speed_channel',
    # The commanded ERPM out of ackermann_to_vesc, and the VESC's own
    # telemetry: measured ERPM, battery voltage, motor current, duty cycle.
    # Without the pair the command -> speed gap is an inference from /odom;
    # with it it is a measurement, and the voltage question answers itself.
    '/commands/motor/speed',
    '/sensors/core',
    # The mission's wall target for the stop ramp, once per move
    # (transient-local, recorded on rosbag2's own QoS adaptation like
    # /mission/status): whether the ramp had a target at all.
    '/mpc/stop_target',
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

# Recorded RELIABLE with a deep queue instead of rosbag2's default depth 10 --
# see _reliable_deep_qos() for the measurement that made this necessary and for
# why /tf_static is deliberately NOT in this list.
_DEFAULT_DEEP_QUEUE_TOPICS = [
    '/tf',
]

# foxglove_bridge's rate-limited copies (f1tenth_bringup/config/
# viz_relays.yaml). They carry nothing the sources above do not, at a fifth
# of the rate, so a bag never records them, even when a `topics` override
# names them.
_NEVER_RECORDED_PREFIXES = ('/viz/', '/viz_internal/')

# ...except these, which live under /viz/ but are published by a node in
# their own right, not relayed from a source a bag already has: /viz/doors is
# lidar_front_wall_node's door markers (door_markers.py), the only record of
# the doors as Foxglove drew them.
_RECORDED_VIZ_ORIGINALS = ('/viz/doors',)


def _relay_copy(topic):
    return topic.startswith(_NEVER_RECORDED_PREFIXES) and topic not in _RECORDED_VIZ_ORIGINALS


def recordable_topics(topics):
    """(topics to record, topics dropped because they are /viz relay copies)."""
    kept = [t for t in topics if not _relay_copy(t)]
    dropped = [t for t in topics if _relay_copy(t)]
    return kept, dropped


# Snapshotted into the manifest for at-a-glance run comparison. The full
# stack_params.yaml copy is the authoritative record; this is the shortlist an
# analysis script can group runs by without parsing yaml.
_SNAPSHOT_PARAM_KEYS = [
    'camera_source', 'localization_source', 'use_behavior_tree', 'enable_nav2',
    'enable_slam', 'enable_intelligence', 'enable_sys_obs',
    'yolo_model', 'yolo_model_task', 'use_mask_depth', 'confidence_threshold',
    # Obstacle disks were sized by object HEIGHT before 2026-09-16 (legacy);
    # runs on either side of that are not comparable without knowing which.
    'obstacle_radius_source', 'obstacle_class_margin_m',
]


def _utc_stamp():
    """Filesystem-safe UTC timestamp, sorts lexicographically by time."""
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H-%M-%S')


# -- single-instance lock ---------------------------------------------------
# Two logger processes both subscribe to the same latched /mission/status and
# both derive the same run_id from it, so they race to create one bag
# directory: one wins, the other logs "Database directory already exists" and
# retries into the winner's directory. That is not hypothetical -- it is what
# happened on 2026-09-04, when the supervisor's auto-started instance and a
# hand-run `ros2 run` one recorded the same mission and left a bag with 31
# topics and 0 messages.
#
# The lock is a PID file claimed with O_CREAT|O_EXCL (atomic, so simultaneous
# starts cannot both win) at a FIXED path, deliberately not derived from
# runs_dir: two instances pointed at different roots are still two subscribers
# fighting over one mission, so they must still collide here.
_DEFAULT_LOCK_PATH = '/tmp/mission_logger.lock'


def _lock_path():
    return os.environ.get('MISSION_LOGGER_LOCK', _DEFAULT_LOCK_PATH)


def _read_lock_pid(path):
    try:
        with open(path) as fh:
            return int(fh.read().strip())
    except (OSError, ValueError):
        return None


def _pid_alive(pid):
    """Signal 0 probes for existence without delivering anything. EPERM means
    the process exists and belongs to another user -- still alive."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return False
    return True


def acquire_singleton_lock(path):
    """Claim the single-logger lock. Returns (True, None) or (False, holder_pid).

    A lock naming a dead PID is stale (the owner crashed before it could clean
    up) and gets reclaimed rather than blocking forever -- a logger that
    refuses to start for the rest of the machine's uptime because of one
    SIGKILL would be a worse failure than the one this guards against.

    A live holder is refused unconditionally, including when the PID is our
    own. That last case is only reachable if a crashed logger left a lock and
    the kernel later handed its exact PID to us, and refusing is the right side
    to err on: the operator sees a message naming the lock file and deletes it,
    whereas the alternative reading ("that's me, so it must be stale") would
    hand a second logger a green light in the one situation this exists to
    prevent.
    """
    for _ in range(2):
        try:
            fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            holder = _read_lock_pid(path)
            if holder is not None and _pid_alive(holder):
                return False, holder
            try:
                os.unlink(path)
            except OSError:
                pass
            continue
        except OSError:
            # Unwritable lock directory: refuse rather than silently running
            # unguarded, which would put us back in the double-record case.
            return False, None
        with os.fdopen(fd, 'w') as fh:
            fh.write(str(os.getpid()))
        return True, None
    return False, _read_lock_pid(path)


def release_singleton_lock(path):
    """Drop the lock, but only if it is still ours -- never unlink a lock a
    successor process legitimately reclaimed after we were declared stale."""
    if _read_lock_pid(path) == os.getpid():
        try:
            os.unlink(path)
        except OSError:
            pass


def _default_runs_dir():
    """stack_params.yaml's mission_logger_runs_dir, so a bare `ros2 run` and
    mission_logger.launch.py land in the same tree without the launch file
    having to be in the picture."""
    try:
        from f1tenth_params.param_defaults import get_value
        return os.path.expanduser(str(get_value('mission_logger_runs_dir')))
    except Exception:  # noqa: BLE001 - a missing key must not block startup
        return os.path.join(os.path.expanduser('~'), 'f1tenth_archive')


# The annotated camera stream, recorded only with record_annotated_image.
_ANNOTATED_TOPIC = '/camera/image_annotated'

# Depth and the intrinsics to back-project it, recorded with record_depth.
# The depth is the 5 Hz copy mission_logger.launch.py's topic_tools throttle
# publishes: the 30 Hz original is ~27.6 MB/s.
_DEPTH_TOPICS = ['/zed2/zed_node/depth/depth_registered_5hz',
                 '/zed2/zed_node/depth/camera_info', '/camera/camera_info']

# Recording PROFILES (stack_params mission_logger_profile; cpu-cleanup R2,
# block 4a). 'default' adds nothing. 'campaign' adds, all at ~5 Hz, what the
# wall-tracker replay (tools/corridor_wall_replay.py) needs to show and
# re-run what the car saw:
#   /perception/scene_labels/classes  the per-pixel scene mask (mono8
#                                     640x360, front_clearance_node's 5 Hz
#                                     mask tick) -- the "scene-label image";
#   /camera/image_raw_5hz             the camera frame YOLO and the mask
#                                     actually use (bgra8 640x360), throttled
#                                     to 5 Hz by mission_logger.launch.py;
#   depth_registered_5hz + camera_infos (_DEPTH_TOPICS);
#   /perception/wall_observations     (already in _DEFAULT_TOPICS, ~10 Hz).
# Estimated ~+345 MB/min over the default list (classes 69, image 276,
# observations 0.7), ~650-850 MB/min in total: output/cpu_cleanup/report.md.
_PROFILE_TOPICS = {
    'default': [],
    'campaign': ['/perception/scene_labels/classes', '/camera/image_raw_5hz',
                 '/perception/wall_observations', *_DEPTH_TOPICS],
}


def profile_topics(profile):
    """Topics a recording profile adds; an unknown profile adds nothing."""
    return list(_PROFILE_TOPICS.get(str(profile), []))


# -- post-hoc snapshots -----------------------------------------------------
# The annotated stream when the bag has it, else the debug video's JPEGs.
_SNAPSHOT_TOPICS = (_ANNOTATED_TOPIC, '/camera/debug_image/compressed')
_SNAPSHOT_MAX = 10


def _snapshot_indices(count, limit):
    """Evenly spaced message positions, always including the first and last.

    Spread across the whole run rather than taking the first N: the interesting
    part of an aborted run is usually its end, and N frames from the opening
    second would show the same empty corridor N times.
    """
    if count <= 0 or limit <= 0:
        return []
    if count <= limit:
        return list(range(count))
    if limit == 1:
        return [0]
    step = (count - 1) / (limit - 1)
    return sorted({int(round(i * step)) for i in range(limit)})


def _open_snapshot_reader(bag_dir, storage_id, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    if topic not in {t.name for t in reader.get_all_topics_and_types()}:
        return None
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader


def _extract_snapshots(bag_dir, out_dir, run_id, storage_id,
                       topics=_SNAPSHOT_TOPICS, limit=_SNAPSHOT_MAX):
    """Up to `limit` evenly spaced JPEGs from the first of `topics` in the bag.

    Returns (paths written, topic used or None). A sensor_msgs/Image topic is
    re-encoded and a CompressedImage JPEG is written as it is.

    Two passes over the bag: the first only counts messages (raw, never
    deserialized) because rosbag2's sequential reader cannot seek or report a
    per-topic count up front, and the second decodes just the chosen frames.
    Decoding every frame to throw most away would cost far more than the extra
    pass.
    """
    import cv2
    from cv_bridge import CvBridge
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, Image

    reader = topic = None
    for topic in topics:
        reader = _open_snapshot_reader(bag_dir, storage_id, topic)
        if reader is not None:
            break
    if reader is None:
        return [], None
    compressed = topic.endswith('/compressed')
    total = 0
    while reader.has_next():
        reader.read_next()
        total += 1

    wanted = set(_snapshot_indices(total, limit))
    if not wanted:
        return [], topic

    reader = _open_snapshot_reader(bag_dir, storage_id, topic)
    bridge = CvBridge()
    os.makedirs(out_dir, exist_ok=True)
    written = []
    for idx in range(total):
        if not reader.has_next():
            break
        _t, raw, _stamp = reader.read_next()
        if idx not in wanted:
            continue
        path = os.path.join(out_dir, f'{run_id}.{len(written):03d}.jpg')
        if compressed:
            with open(path, 'wb') as f:
                f.write(bytes(deserialize_message(raw, CompressedImage).data))
            written.append(path)
            continue
        image = bridge.imgmsg_to_cv2(
            deserialize_message(raw, Image), desired_encoding='bgr8')
        if cv2.imwrite(path, image):
            written.append(path)
    return written, topic


def _best_effort_qos(depth):
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _reliable_deep_qos(depth):
    """RELIABLE + KEEP_LAST at a large depth, for topics whose MESSAGE RATE
    (not byte rate) outruns rosbag2's default queue.

    WHY RELIABLE AND NOT BEST_EFFORT, since best-effort is the usual answer to
    "recorder can't keep up": best-effort would license the middleware to
    discard silently under load, which is precisely the failure being fixed
    here -- and it would silently discard MORE, not less. Reliable also
    matches what tf2 broadcasters actually offer, so the subscription is
    compatible without adaptation. The queue DEPTH is the fix; the reliability
    setting is not.

    WHAT THIS FIXES, measured. rosbag2's default subscription QoS is
    KEEP_LAST depth 10. /tf carries every broadcaster in the stack on one
    topic -- both EKF instances, the ZED wrapper and robot_state_publisher,
    ~109Hz aggregate -- so depth 10 is about 92ms of tolerance before samples
    are overwritten. In run 2026-09-07T12-48-40 that ran out: the recorder's
    /tf receive-minus-header lag went from -3.9ms to +2286.8ms (peak
    +9930.6ms) partway through, while EVERY other topic's lag stayed flat
    (/odom 4.3 -> 4.2ms, /scan 0.7 -> 2.4ms, image_annotated 494 -> 502ms).
    One backlogged subscription, not a sick recorder. All four /tf publishers
    appeared to collapse at once, retaining 2.8-11.3% of their rates, which is
    the tell: four independent nodes cannot fail simultaneously, but they do
    share exactly one thing -- this subscription.

    THAT COST REAL ANALYSIS TIME, which is why the depth is generous rather
    than merely sufficient: an entire corridor-drift investigation was built
    on interpolating that 90%-missing stream, and the "drift" it found did not
    survive recomputation against non-interpolated samples (span 15.5deg ->
    3.0deg, trend +0.003deg/s). A dropped /tf sample is not a lost datum here,
    it is a wrong conclusion.

    NOT FOR /tf_static, deliberately: that one is TRANSIENT_LOCAL, and
    subscribing VOLATILE (as this profile does) would miss the latched backlog
    that IS its entire content -- a recorder that captures no static
    transforms at all, while looking healthy. Leave it on rosbag2's own
    adaptation.
    """
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
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

        self.runs_dir = os.path.expanduser(
            str(self.declare_parameter('runs_dir', _default_runs_dir()).value))
        # Two states, one rename apart. Everything downstream (the sync unit,
        # the runs CLI) decides "is this run safe to touch" by which of these
        # it is in, so nothing ever has to inspect a bag to find out.
        self.active_dir = os.path.join(self.runs_dir, 'active')
        self.complete_dir = os.path.join(self.runs_dir, 'complete')
        self.incomplete_dir = os.path.join(self.runs_dir, 'incomplete')
        self.requested_storage_id = str(self.declare_parameter('storage_id', 'mcap').value)
        topics = list(self.declare_parameter('topics', _DEFAULT_TOPICS).value)
        # Off by default: the raw annotated stream was 134 of the 179 MB of
        # the 2026-10-13 R007 bag, and the debug video covers it.
        if (bool(self.declare_parameter('record_annotated_image', False).value)
                and _ANNOTATED_TOPIC not in topics):
            topics.append(_ANNOTATED_TOPIC)
        if bool(self.declare_parameter('record_depth', False).value):
            topics += [t for t in _DEPTH_TOPICS if t not in topics]
        self.profile = str(self.declare_parameter('profile', 'default').value)
        if self.profile not in _PROFILE_TOPICS:
            self.get_logger().warn(
                f'unknown recording profile "{self.profile}" -- recording the default list '
                f'(known: {sorted(_PROFILE_TOPICS)})')
        topics += [t for t in profile_topics(self.profile) if t not in topics]
        self.topics, dropped = recordable_topics(topics)
        if dropped:
            self.get_logger().warn(f'not recording /viz relay copies: {dropped}')
        self.best_effort_topics = list(self.declare_parameter(
            'best_effort_topics', _DEFAULT_BEST_EFFORT_TOPICS).value)
        self.qos_depth = int(self.declare_parameter('qos_override_depth', 10).value)
        self.deep_queue_topics = list(self.declare_parameter(
            'deep_queue_topics', _DEFAULT_DEEP_QUEUE_TOPICS).value)
        # 500 at /tf's ~109Hz is ~4.6s of buffer, against the ~92ms that
        # rosbag2's default depth 10 gave. Comfortably over the >=100 floor the
        # fix calls for: the queue only costs memory when it is actually being
        # used, and the failure it prevents is silent.
        self.deep_queue_depth = int(self.declare_parameter(
            'deep_queue_depth', 500).value)
        self.sweep_period_sec = float(self.declare_parameter('sweep_period_sec', 300.0).value)
        # 'move' (default) relocates into <runs_dir>/incomplete/; 'delete'
        # removes outright -- opt-in only, see module docstring.
        self.sweep_action = str(self.declare_parameter('sweep_action', 'move').value)
        status_topic = str(self.declare_parameter('status_topic', '/mission/status').value)
        # Caps a mission loaded and never started (refused start, forgotten
        # load): the pre-roll would otherwise record images until the next
        # status change. 0 disables it.
        self.loaded_timeout_sec = float(self.declare_parameter(
            'loaded_timeout_sec', 120.0).value)

        self.storage_id = self._resolve_storage_id()

        # Recorder lifecycle. _lock guards all of these together: the status
        # callback (executor thread) and the recorder worker thread both touch
        # them.
        self._lock = threading.RLock()
        self._recorder = None
        self._record_thread = None
        self._active_bag_dir = None
        self._active_run = None      # dict -> becomes the manifest
        self._finalize_thread = None

        self._last_state = None
        self._last_estop = False
        self._refused_json_path = None

        try:
            os.makedirs(self.active_dir, exist_ok=True)
            os.makedirs(self.complete_dir, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(
                f'could not create run tree under "{self.runs_dir}": {exc} -- recording '
                'will fail until this is fixed, but missions are unaffected.')

        self._sweep_incomplete_bags(reason='startup')
        self._sweep_timer = self.create_timer(
            self.sweep_period_sec, lambda: self._sweep_incomplete_bags(reason='periodic'))
        self._loaded_timeout_timer = self.create_timer(1.0, self._check_loaded_timeout)

        self._status_sub = self.create_subscription(
            MissionStatus, status_topic, self._on_status, _MISSION_STATUS_QOS)

        self.get_logger().info(
            f'mission_logger_node up (pid {os.getpid()}, lock {_lock_path()}): watching '
            f'"{status_topic}", runs -> "{self.runs_dir}" (active/ -> complete/, '
            f'storage={self.storage_id}), {len(self.topics)} topics, profile {self.profile} '
            f'({len(self.best_effort_topics)} with BEST_EFFORT overrides), '
            f'sweep every {self.sweep_period_sec:.0f}s (action={self.sweep_action}), '
            f'recording from LOADED (timeout {self.loaded_timeout_sec:.0f}s). '
            'THIS IS THE LIVE LOGGER -- a second `ros2 run` will be refused.')

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
            if state != 'LOADED':
                self._refused_json_path = None

            action, outcome = _decide(
                state, str(msg.json_path), estop_latched, self._active_run,
                self._refused_json_path)
            if action in ('stop', 'restart'):
                self._stop_recording(outcome)
            if action in ('start', 'restart'):
                self._start_recording(msg)
            elif action == 'mark_started':
                self._mark_mission_started()

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
            # One folder per run from the first byte. The bag is always the
            # literal "bag" child, which is what lets load_manifest_for() find
            # a run's manifest by looking at the bag directory's parent.
            run_dir = os.path.join(self.active_dir, run_id)
            bag_dir = os.path.join(run_dir, 'bag')

            now = datetime.datetime.now(datetime.timezone.utc).isoformat()
            # start_time means the MISSION start (the exporter matches on it).
            # Opened at LOADED it is provisional and _mark_mission_started
            # moves it to the RUNNING edge; opened at RUNNING it is final.
            started = str(msg.state) in _STARTED_STATES
            run = {
                'mission_id': mission_name,
                'run_id': run_id,
                'start_time': now,
                'record_start_time': now,
                'mission_start_time': now if started else None,
                'preroll_s': 0.0 if started else None,
                'end_time': None,
                'outcome': None,
                'run_dir': run_dir,
                'bag_path': bag_dir,
                'params_snapshot_path': os.path.join(run_dir, f'{run_id}.params.yaml'),
                'manifest_path': os.path.join(run_dir, f'{run_id}.manifest.json'),
                'storage_id': self.storage_id,
                'profile': self.profile,
                'mission_json_path': str(msg.json_path),
            }

            try:
                # run_dir only: rosbag2 insists on creating bag_dir itself and
                # refuses to open one that already exists.
                os.makedirs(run_dir, exist_ok=True)
                # max_cache_size MUST be passed explicitly. rosbag2_py's
                # StorageOptions defaults it to 0, whereas `ros2 bag record`
                # -- the path this node replaced -- defaults to 100MiB, so
                # constructing StorageOptions without it silently gives the
                # recorder no write cache at all. Nothing warns: rosbag2 only
                # rejects a zero cache in snapshot mode.
                storage_options = rosbag2_py.StorageOptions(
                    uri=bag_dir, storage_id=self.storage_id,
                    max_cache_size=100 * 1024 * 1024)
                record_options = rosbag2_py.RecordOptions()
                record_options.all = False
                record_options.topics = list(self.topics)
                record_options.is_discovery_disabled = False
                record_options.rmw_serialization_format = 'cdr'
                record_options.topic_polling_interval = datetime.timedelta(milliseconds=100)
                # Deep-queue entries are applied after the best-effort ones
                # so that a topic listed in both lists lands RELIABLE+deep
                # rather than silently keeping whichever dict comprehension ran
                # last. The two lists are disjoint by default.
                qos_overrides = {
                    t: _best_effort_qos(self.qos_depth) for t in self.best_effort_topics}
                qos_overrides.update({
                    t: _reliable_deep_qos(self.deep_queue_depth)
                    for t in self.deep_queue_topics})
                record_options.topic_qos_profile_overrides = qos_overrides

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

    def _mark_mission_started(self):
        """Move start_time to now: the LOADED -> RUNNING edge of an open run."""
        with self._lock:
            run = self._active_run
            if run is None or run.get('mission_start_time'):
                return
            now = datetime.datetime.now(datetime.timezone.utc)
            record_start = datetime.datetime.fromisoformat(run['record_start_time'])
            run['mission_start_time'] = now.isoformat()
            run['start_time'] = run['mission_start_time']
            run['preroll_s'] = round((now - record_start).total_seconds(), 3)
        self._write_manifest(run)
        self.get_logger().info(
            f'mission started: "{run["run_id"]}" has been recording for '
            f'{run["preroll_s"]:.1f}s (pre-roll since LOADED).')

    def _check_loaded_timeout(self):
        """Stop a run that sat at LOADED past loaded_timeout_sec (timer)."""
        try:
            run = self._active_run
            now = datetime.datetime.now(datetime.timezone.utc)
            if not _loaded_timed_out(run, now, self.loaded_timeout_sec):
                return
            self.get_logger().warn(
                f'"{run["run_id"]}" stayed LOADED for over '
                f'{self.loaded_timeout_sec:.0f}s without starting -- stopping its '
                'pre-roll recording (outcome=NOT_STARTED).')
            self._refused_json_path = run.get('mission_json_path')
            self._stop_recording('NOT_STARTED')
        except Exception as exc:  # noqa: BLE001 - a logger must not kill anything
            self.get_logger().error(f'LOADED timeout check failed: {exc}')

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
        # Written while still in active/, so a run whose move fails below is
        # still a complete, enumerable record where it lies.
        self._write_manifest(run)

        moved = self._move_to_complete(run)
        self.get_logger().info(
            f'stopped recording "{run["run_id"]}" (outcome={outcome}) -> {run["run_dir"]}')
        if moved:
            self._start_finalize(run)

    def _move_to_complete(self, run):
        """active/<run_id>/ -> complete/<run_id>/, whole, by rename.

        os.rename and NOT copy-then-delete: a rename within one filesystem is
        atomic, so a run is either entirely in active/ or entirely in
        complete/ and never observably half-present. That is the whole basis
        on which f1tenth-archive.service syncs complete/ with no "is this
        finished?" filter. A cross-filesystem runs_dir would raise EXDEV here
        and is reported rather than papered over with a copy, because a copy
        would silently reintroduce the half-written-bag window.
        """
        src = run['run_dir']
        run_id = run['run_id']
        dest = os.path.join(self.complete_dir, run_id)
        try:
            os.makedirs(self.complete_dir, exist_ok=True)
            if os.path.exists(dest):
                dest = f'{dest}.{_utc_stamp()}'
                self.get_logger().warn(
                    f'"{run_id}" already exists in complete/ -- filing this one as '
                    f'"{os.path.basename(dest)}" rather than merging two runs.')
            os.rename(src, dest)
        except OSError as exc:
            self.get_logger().error(
                f'could not move "{run_id}" into complete/: {exc} -- the run is intact '
                f'in "{src}" and will not be synced until moved by hand.')
            return None

        run['run_dir'] = dest
        run['bag_path'] = os.path.join(dest, 'bag')
        run['params_snapshot_path'] = os.path.join(dest, f'{run_id}.params.yaml')
        run['manifest_path'] = os.path.join(dest, f'{run_id}.manifest.json')
        return dest

    # -- finalize ----------------------------------------------------------
    def _start_finalize(self, run):
        """Checksums, extract, snapshots and sync, off the executor thread.

        _stop_recording runs in the /mission/status callback: doing minutes of
        parquet extraction there would stall the subscription that notices the
        NEXT mission starting. The move above already happened synchronously,
        so the run is safely in complete/ before any of this begins.
        """
        thread = threading.Thread(
            target=self._finalize_run, args=(run,),
            name=f'mission_finalize_{run["run_id"]}', daemon=True)
        self._finalize_thread = thread
        thread.start()

    def _finalize_run(self, run):
        for step, fn in (
                ('manifest checksums', self._augment_manifest),
                ('parquet extract', self._write_extract),
                ('snapshots', self._write_snapshots),
                ('archive sync', self._spawn_sync)):
            try:
                fn(run)
            except Exception as exc:  # noqa: BLE001 - one bad step must not skip the rest
                self.get_logger().error(
                    f'finalize step "{step}" failed for "{run["run_id"]}": {exc} -- '
                    'the bag itself is safe in complete/.')

    def _augment_manifest(self, run):
        from f1tenth_logger.runs_migrate import bag_checksum
        checksums, total = bag_checksum(run['bag_path'])
        run['bag_bytes'] = total
        run['bag_sha256'] = checksums
        self._write_manifest(run)

    def _write_extract(self, run):
        # Lazy: extraction pulls pyarrow and the whole mission_extract stack,
        # which has no business being imported by a node that may never
        # finalize a run.
        from f1tenth_logger.mission_extract import extract_bag
        out = os.path.join(run['run_dir'], f'{run["run_id"]}.extract.parquet')
        extract_bag(run['bag_path'], out)
        self.get_logger().info(f'extract written: {out}')

    def _write_snapshots(self, run):
        """Evenly-spaced stills from the annotated camera stream, read back out
        of the finished bag rather than tapped live -- the recorder already
        captured those frames, and decoding them during the mission would cost
        CPU exactly when the car needs it."""
        written, topic = _extract_snapshots(
            run['bag_path'], os.path.join(run['run_dir'], 'snapshots'),
            run['run_id'], self.storage_id)
        if written:
            self.get_logger().info(
                f'{len(written)} snapshot(s) from {topic} -> '
                f'{os.path.join(run["run_dir"], "snapshots")}')
        else:
            self.get_logger().warn(
                f'no {" or ".join(_SNAPSHOT_TOPICS)} frames in "{run["run_id"]}" '
                '-- no snapshots.')

    def _spawn_sync(self, run):
        """Detached, never waited on. A failed sync is not a failed run: the
        bag is already safe in complete/, and f1tenth-archive.timer retries
        every 15 minutes for anything finalized while linus was unreachable."""
        subprocess.Popen(
            ['systemctl', '--user', '--no-block', 'start', 'f1tenth-archive.service'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.get_logger().info(
            f'archive sync spawned for "{run["run_id"]}" (detached, not awaited).')

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
            if not os.path.isdir(self.active_dir):
                return
            quarantine = self.incomplete_dir
            with self._lock:
                active = self._active_bag_dir
            swept = []
            for name in sorted(os.listdir(self.active_dir)):
                path = os.path.join(self.active_dir, name)
                if not os.path.isdir(path):
                    continue
                # A run folder is judged by its bag child: <run_id>/bag/.
                bag = os.path.join(path, 'bag')
                if active is not None and os.path.abspath(bag) == os.path.abspath(active):
                    continue  # never touch the in-progress bag
                if os.path.isfile(os.path.join(bag, 'metadata.yaml')):
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
        # Finalize runs on a daemon thread, which dies silently with the
        # process -- so a run stopped by the shutdown that just triggered it
        # would reach complete/ with no extract and no checksums. Wait for it.
        thread = self._finalize_thread
        if thread is not None and thread.is_alive():
            self.get_logger().info('waiting for finalize to finish before exit...')
            thread.join(timeout=120.0)
            if thread.is_alive():
                self.get_logger().warn(
                    'finalize did not finish within 120s -- the bag is safe in '
                    'complete/, but its extract/snapshots may be missing.')


def main(args=None):
    lock = _lock_path()
    acquired, holder = acquire_singleton_lock(lock)
    if not acquired:
        held = f' (pid {holder})' if holder else ''
        print(
            f'mission_logger_node: another mission logger is already running{held} -- '
            f'refusing to start a second one, because two loggers derive the same '
            f'run_id from the same /mission/status and race for one bag directory. '
            f'Lock: {lock}',
            file=sys.stderr)
        return 1

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
        release_singleton_lock(lock)
    return 0


if __name__ == '__main__':
    sys.exit(main())
