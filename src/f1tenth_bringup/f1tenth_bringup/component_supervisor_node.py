"""component_supervisor_node: spawns and independently restarts named "components"
(functional groups of the F1TENTH stack -- hardware, localization, perception,
control, navigation, behavior, diagnostics, intelligence, dev_tools, plus the
on-demand-only calibrate_hardware/startup_sequence) as separate `ros2 launch`
subprocesses, each in
its own process group (subprocess.Popen(..., start_new_session=True)), and exposes
two services:
  - f1tenth_messages/srv/RestartComponent: kill-and-relaunch one component.
  - f1tenth_messages/srv/ComponentControl: SHUTDOWN (kill and leave down --
    excluded from watchdog auto-respawn until explicitly START/RESTART-ed again),
    START (spawn if not already running), or RESTART (kill-if-running then
    respawn regardless) for one component.

A watchdog timer (watchdog_period_sec) polls every tracked subprocess and
auto-respawns any that exited on their own (a crash, not a supervisor-initiated
stop) -- except ones a ComponentControl SHUTDOWN marked manually_stopped. Each
subprocess has its own restart budget (max_auto_restarts within a trailing
restart_budget_window_sec): once exhausted, the watchdog stops trying and leaves
it down, logging clearly, until a manual START/RESTART (via either service)
resets it by replacing the process with a fresh one.

~/run_calibration (std_srvs/srv/Trigger): a purpose-named, fire-and-forget wrapper
around what a raw ComponentControl(component_name='calibrate_hardware', action=
RESTART) call already does today -- returns as soon as the two components are
(re)started, not once calibration actually finishes (that takes ~60-130s; watch
/calibration/in_progress, published by f1tenth_diagnostics' diagnostics_server_node,
or this component's own log for that). Also stops 'hardware' FIRST, which
ComponentControl on 'calibrate_hardware' alone does NOT do: both components' launch
trees open the same VESC serial port, and the vendored driver doesn't open it
exclusively -- running them concurrently risks two processes actually contending
for the same UART, not just a clean failure. Deliberately does NOT auto-restart
'hardware' once calibration finishes (same "not auto-sequenced" scope boundary
calibrate_hardware's own localization/navigation follow-up already has, see below)
-- watch /calibration/in_progress go back to false, then START/RESTART 'hardware'
yourself to resume driving with the fresh values.

This is the parallel bringup path alongside stack_bringup.launch.py (which stays
untouched as a working fallback -- single process, no per-component restart). See
f1tenth_bringup/config/components.yaml for the structural registry (WHAT each
component runs) -- WHICH components auto-start, and how navigation/behavior/
intelligence are modified or skipped, is decided here, once, at startup, by reading
the same 4 stack-wide branching values (f1tenth_params/config/stack_params.yaml)
that stack_bringup.launch.py itself uses, PLUS this node's own enable_intelligence
parameter for 'intelligence' specifically (component-auto-start pass -- a real,
CLI-overridable launch arg via supervisor_bringup.launch.py, calibration-style, NOT
one of the 4 stack-wide values; see self.enable_intelligence and stack_params.yaml's
own enable_intelligence comment for why this one component needed its own separate,
CLI-overridable flag instead of a baked-in-at-parse-time stack-wide value):

  behavior      auto-starts only if use_behavior_tree
  intelligence  auto-starts only if enable_intelligence (this node's own declared
                parameter, NOT a stack_params.yaml get_value() lookup -- see above). Starts
                ONLY llama-server (llm.launch.py's own start_server launch-arg
                now itself defaults 'true' -- components.yaml's intelligence
                entry doesn't need to override it any more) -- never
                llm_planner_node itself, which stays a separately,
                manually-invoked CLI tool regardless of this flag (see
                llm.launch.py's own module docstring for why).
  control       always auto-starts; f1tenth_control/ackermann_mux.launch.py only --
                safety_stop_controller was retired (superseded by the BT's own
                unconditional handle_obstacle lane, see f1tenth_behavior)
  navigation    always auto-starts; f1tenth_navigation/navigation.launch.py (its
                one launches-list entry) branches Nav2 vs. mpc_corr on enable_nav2
                itself -- component_supervisor_node doesn't gate this one, same as
                dev_tools/enable_foxglove below
  dev_tools     always auto-starts; foxglove_bridge.launch.py's own enable_foxglove
                param (default true) decides whether the Node inside it actually
                launches -- component_supervisor_node itself doesn't gate this one
  slam          always auto-starts; f1tenth_navigation/slam.launch.py's own
                enable_slam param (default TRUE -- corrected here, this
                comment previously said FALSE, stale since whenever the
                stack_params.yaml default was actually flipped) decides
                whether the actual async_slam_toolbox_node Node inside it
                actually launches -- component_supervisor_node itself
                doesn't gate this one, same pattern as dev_tools/
                enable_foxglove above (not navigation/enable_nav2, which
                IS one of the 4 stack-wide branching values)
  everything else (hardware, localization, perception, diagnostics)
                always auto-starts
  calibrate_hardware, startup_sequence
                NEVER auto-start -- registered (valid, restartable
                component_names) but only ever brought up on an explicit
                RestartComponent/ComponentControl request. startup_sequence
                moved here from always-auto-start on request: it's a
                steering-sweep visual "the stack is alive" check
                (f1tenth_bringup/stack_startup_sequence.py), not a
                functional dependency of anything else in the stack (no
                other node reads its output or waits on it) -- confirmed
                via a full grep before this change. Still available via
                `ros2 service call /restart_component
                f1tenth_messages/srv/RestartComponent
                "{component_name: 'startup_sequence'}"` if wanted on
                demand. stack_bringup.launch.py's own separate, single-
                process bringup path still includes it unconditionally --
                that file is untouched, this change is scoped to the
                supervisor's own auto-start behavior only.

calibrate_hardware runs vesc.launch.py with calibration:=true release_downstream:=
false: it cycles the VESC driver through a fresh calibration measurement exactly
like stack_bringup.launch.py's calibration:=true path, but does NOT itself release
ekf_node/Nav2 afterward (release_downstream:=false -- see vesc.launch.py), since
those are this supervisor's own independently-tracked localization/navigation
components, not nested includes of vesc.launch.py's own launch tree. This node does
NOT auto-sequence a full recalibration workflow: after calling
restart_component('calibrate_hardware') and confirming it finished (watch its log
for "fresh driver group v2 up", or wait roughly calibration_duration_sec plus a few
seconds), call restart_component('localization') and restart_component('navigation')
yourself to pick up the freshly-calibrated values. That's a deliberate scope
boundary -- RestartComponent's request is just a component_name, with no field for
"and then also restart these other components once this one settles", so automatic
cross-component sequencing would need a different service contract entirely.

'localization' deferred-start (automatic first-boot path -- calibration-restart
gap fix, following a live CPU-contention/EKF-audit investigation, see that pass's
own report): the paragraph above documents the MANUAL recalibration path's own
"restart localization yourself afterward" requirement -- the automatic first-boot
path had no equivalent at all. Confirmed live: 'hardware' and 'localization' are
both in _ALWAYS_AUTO_START, started from the same unordered `for name in
auto_start` loop below with no sequencing between them (observed ~2ms apart) --
ekf_filter_node/ekf_global_filter_node began integrating /odom + /sensors/imu/raw
from driver group v1 (running on WHATEVER gyro_bias_z was already in vesc.yaml
from the PREVIOUS calibration, not this boot's fresh one -- see f1tenth_hardware/
vesc.launch.py's own module docstring for the v1/v2 split) through the v1->v2
restart, with NO state reset once v2's freshly-calibrated stream came online
~60-130s later. Any bias integrated during that window stayed permanently baked
into the filter's running estimate -- the live-measured ~60deg static local-EKF
yaw offset at rest, from the investigation this fix follows up on, is consistent
with exactly this.

Fixed by deferring 'localization's own auto-start (see _defer_localization_start())
until /calibration/in_progress (f1tenth_diagnostics' diagnostics_server_node, pure
ROS-graph introspection, unaffected by which of the three ways calibration actually
got triggered -- see that node's own docstring) shows a genuine True->False
transition, not merely "reads False" (a real race: this subscription attaches at
__init__ time, essentially instantly, while the calibration nodes themselves don't
appear in the graph until several seconds into 'hardware's own battery-check +
driver-bringup sequence -- trusting the seeded-False initial value would silently
skip the wait it exists for). Bounded by localization_calibration_wait_timeout_sec
regardless (fail-open -- same "never block startup indefinitely" discipline vesc.
launch.py's own calibration-failure handling already uses) so a genuinely stuck
calibration, or the (structurally unlikely given the timing margin above, but not
impossible) case of missing the True phase entirely, can't hang 'localization' from
ever starting. Only 'localization' is deferred -- 'navigation' auto-starts
immediately as before, unchanged; out of this fix's own explicit scope. A manual
RestartComponent/ComponentControl request naming 'localization' while the wait is
still pending cancels it and honors the manual request immediately (see
_resolve_localization_deferral()'s own docstring) -- a human saying "start it now"
always wins over the automatic wait.

Scope note (flagged, not silently expanded): this fix does not touch or fold into
any other outstanding calibration-path work -- searched this repository's git
history, docs, and code comments for a "startup calibration audit" / prior
blocker list to fold into and found none; if one exists outside this repo, this
fix should be reconciled against it separately rather than assumed compatible.

Process model: each component's command(s) are spawned via subprocess.Popen with
start_new_session=True, which puts each ros2 launch invocation (and everything it in
turn spawns) in its own new session/process group, distinct from
component_supervisor_node's own -- and, since the immediate child becomes its own
session leader, its pgid is definitionally its own pid, no os.getpgid() lookup
needed. Restarting a component sends SIGINT to that whole process group
(os.killpg), not just the top-level ros2 launch PID -- SIGINT alone to just that PID
would leave every node ros2 launch spawned running as orphans. If the group hasn't
exited within restart_timeout_sec, it gets SIGKILL.

Clean shutdown: on a normal KeyboardInterrupt out of rclpy.spin() (Ctrl+C reaching
this process), main()'s try/finally already called shutdown_all(). That alone isn't
reliable for every way this node actually dies, though: plain SIGTERM has no Python
exception to unwind through by default (the interpreter just exits, skipping
finally, so shutdown_all() never runs), and a SIGKILL on this process's own pid
can't run any Python code at all. main() also installs explicit SIGINT/SIGTERM
handlers (signal.signal, registered just before rclpy.spin() starts) that call
shutdown_all() directly and exit -- covering the SIGTERM gap.

shutdown_all() stops every tracked component via _stop_all_components(), which
SIGINTs all of them up front and then waits on all of them concurrently against
one shared _SHUTDOWN_GRACE_SEC deadline (not sequentially, one full wait at a
time) -- this matters because `ros2 launch` gives a managed Node like this one
only ~5s after SIGINT before escalating to SIGTERM itself, and a sequential
sum-of-waits across ~10 tracked components routinely exceeds that regardless of
how short each individual wait is. When it does, that follow-up SIGTERM reaches
this same signal handler again -- reentrantly, while the first shutdown_all()
call is still paused mid-wait -- which is a real, observed failure mode, not a
hypothetical one: shutdown_all() is guarded by _shutdown_started (set at the
very start, before it's actually finished), so a second call from the reentrant
signal would just hit that guard and return instantly, and the handler would
then os._exit(0) anyway, abandoning whichever components the first call hadn't
gotten to yet -- precisely the orphans this whole mechanism exists to prevent.
The handler checks for exactly that (_shutdown_started True, _shutdown_done
still False) and escalates instead: force_kill_all() SIGKILLs every still-
tracked process group immediately, no waiting, so a second signal is always a
hard guarantee of a clean exit no matter how slow anything was being.

Single-instance lock (_SUPERVISOR_LOCK_PATH, claimed in main() before rclpy.init()):
this node must be a singleton, and nothing enforced that until a live session ran
three of them at once. Two supervisors don't merely duplicate work -- they actively
break each other in three separate ways, all observed:
  - Components holding an exclusive OS-level resource simply cannot start twice, so
    the second supervisor's copy dies instantly with exit code 1 and then burns its
    whole max_auto_restarts budget losing the same race: 'intelligence' (llama-server
    binds a fixed TCP port -- "couldn't bind HTTP server socket ... port 8083") and
    'diagnostics' (mission_logger_node's own single-instance lock, see f1tenth_logger/
    mission_logger_node.py's own "-- single-instance lock" section, which this one
    deliberately mirrors). Every other component tolerates a duplicate by merely
    double-publishing, which is exactly why this failure mode reads as "only
    diagnostics and intelligence are broken" rather than "the stack is launched twice".
  - _pgid_file is a fixed path under log_dir, so both instances read, rewrite and
    delete ONE shared file. The second instance's _sweep_stale_pgids() reads the
    first's LIVE pids, passes _looks_like_ros2_launch() on them (they are genuine
    `ros2 launch` processes), kills them as "stale", then _clear_pgid_file()s the
    survivor's own tracking out from under it -- after which nothing on the machine
    knows those process groups exist and the next SIGKILL orphans them permanently.
    An orphaned supervisor tree reparented to init is how this was found.
  - Per-component log files are opened 'w' (truncate) at a path derived only from
    package + launch file, so both instances clobber and interleave the same logs,
    which is what made the failure so hard to read.
Note that per-instance paths would NOT have been a fix for the middle point: the
stale sweep works precisely because the next instance knows where the previous one
left its file. A hard singleton is what makes that shared path safe, so the lock is
the whole fix rather than one half of it.

Startup safety sweep: none of the above helps if this process itself gets SIGKILLed
(a crash, an OOM kill, `kill -9`) -- nothing runs, so every process group it had
spawned is orphaned (reparented to init) with nothing left to ever clean it up, and
the next launch of this node would spawn a second full set on top, unaware the
first set is still alive. To catch that: every live process group this node has
spawned is mirrored to a small JSON file (self._pgid_file, under log_dir) on every
start/stop/respawn, and cleared on a clean shutdown_all(). On startup, before
spawning anything, _sweep_stale_pgids() checks that file: if it's non-empty, the
previous instance didn't get to clean up after itself, so each recorded pid is
checked for liveness and killed (SIGTERM, then SIGKILL after _SHUTDOWN_GRACE_SEC) if
still around -- logged as a warning either way, since it means something skipped a
graceful shutdown last time. A lightweight /proc/<pid>/cmdline check guards against
the (unlikely but possible) case of the recorded pid having been reused by an
unrelated process since the file was written.

FastDDS shared-memory hygiene (_sweep_stale_fastrtps_shm): a completely separate
concern from the process-group sweep above, root-caused live in the same session
this was added. Clean process-group termination (SIGINT/SIGTERM/SIGKILL, all of the
above) says nothing about whether the FastDDS SHM transport released its own
/dev/shm segment/lock files before the owning process exited -- those files are
independent OS-level artifacts that persist regardless of how cleanly the process
that created them went away, and they accumulate silently across sessions (249
found in one dev session, some >24h old). FastDDS's SHM transport draws from a
small, fixed pool of "well-known" discovery ports; once enough of that pool is
occupied by stale, orphaned files, new participants starting close together (this
node's own ~11-tree, several-dozen-node burst at startup is exactly that) lose the
race for a port and get stuck retrying indefinitely instead of failing cleanly or
falling back -- which is what actually caused the VESC/odom/perception/diagnostics/
behavior startup failures diagnosed earlier this session. Neither this node's own
process-group hygiene above nor `ros2`/FastDDS itself does anything about this on
its own, so it's swept explicitly here:
  - matches /dev/shm/*fastrtps* -- one glob catches both the per-participant segment
    family (fastrtps_<hex>, fastrtps_<hex>_el) and the well-known discovery-port
    family (fastrtps_port<N>, fastrtps_port<N>_el, sem.fastrtps_port<N>_mutex); a
    naive fastrtps_* prefix match misses the sem.fastrtps_* files
  - a file only counts as orphaned if no live process anywhere on the system still
    has it open (checked via /proc/<pid>/fd) or mapped (via /proc/<pid>/maps, since
    POSIX shared memory is commonly mmap'd and then has its creating fd closed) --
    never blanket-deleted
  - refuses outright (logs an error, removes nothing) if another live
    component_supervisor_node process is found, since that's the one case where
    deleting a "seemingly orphaned" file could pull it out from under a legitimate
    sibling instance
  - runs at startup (from _sweep_stale_pgids(), so it always executes before the
    first component tree is spawned, whether or not that call found any stale
    pgids of its own) and at shutdown (end of both shutdown_all() and
    force_kill_all(), once every tracked process group is confirmed gone) -- the
    shutdown-side sweep exists because this node's own components can just as
    easily leave this residue behind as anything else that's ever run on this
    machine.
"""

import glob
import json
import os
import signal
import subprocess
import sys
import time

import yaml

from f1tenth_messages.srv import ComponentControl, RestartComponent
from f1tenth_params.param_defaults import get_value

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# OS-level process-teardown buffer between "'hardware' confirmed stopped" and
# "start 'calibrate_hardware'" in ~/run_calibration (see that handler and the
# module docstring's own paragraph) -- same reasoning and same value as
# f1tenth_hardware/vesc.launch.py's _TEARDOWN_BUFFER_SEC: the OS/kernel needs a
# moment after a process exits before the serial port it held is actually free
# for a different process to reopen, a gap "the process is confirmed gone" alone
# doesn't cover.
_HARDWARE_TEARDOWN_BUFFER_SEC = 2.0

# Components that auto-start unconditionally.
_ALWAYS_AUTO_START = {'hardware', 'localization', 'navigation', 'perception',
                      'control', 'diagnostics', 'dev_tools', 'slam'}
# Registered (restartable by name) but never auto-started -- on-demand only.
# startup_sequence (the steering-sweep visual check) moved here on request --
# see module docstring's own "calibrate_hardware, startup_sequence" paragraph
# for why this is safe (no other node depends on it).
_NEVER_AUTO_START = {'calibrate_hardware', 'startup_sequence'}
# 'behavior' gates on a stack-wide branching value (get_value(), no CLI override --
# see module docstring). 'intelligence' is DIFFERENT as of the component-auto-start
# pass: 'enable_intelligence' here names this node's own declared parameter
# (self.enable_intelligence), not a stack_params.yaml get_value() lookup -- the auto_
# start-building loop below special-cases it for exactly that reason. Kept as a
# dict entry (not hardcoded inline) purely for readability/consistency with
# 'behavior''s own entry; see module docstring for why 'intelligence' needed a real,
# CLI-overridable flag instead of a baked-in-at-parse-time stack-wide value.
_CONDITIONAL_AUTO_START = {
    'behavior': 'use_behavior_tree',
    'intelligence': 'enable_intelligence',
}

# FastDDS SHM transport hygiene (see module docstring's own paragraph on this) --
# one glob catches both the fastrtps_<hex>/fastrtps_<hex>_el segment family and the
# fastrtps_port<N>/fastrtps_port<N>_el/sem.fastrtps_port<N>_mutex well-known-port
# family, since all of them contain 'fastrtps' as a substring.
_FASTRTPS_SHM_DIR = '/dev/shm'
_FASTRTPS_SHM_GLOB_PATTERN = '*fastrtps*'

# -- single-instance lock ------------------------------------------------------
# See the module docstring's own "Single-instance lock" paragraph for the three
# distinct ways two concurrent supervisors break each other, all observed live.
#
# A PID file claimed with O_CREAT|O_EXCL (atomic, so two supervisors racing from
# the same script cannot both win) at a FIXED path, deliberately NOT derived from
# log_dir: two supervisors pointed at different log_dirs are still two supervisors
# spawning two full stacks onto one machine's ports, serial devices and ROS graph,
# so they must still collide here. Same reasoning, and the same implementation
# shape, as f1tenth_logger/mission_logger_node.py's own single-instance lock --
# kept deliberately parallel so the two read as one pattern rather than two.
_DEFAULT_SUPERVISOR_LOCK_PATH = '/tmp/component_supervisor.lock'


def _supervisor_lock_path():
    return os.environ.get('COMPONENT_SUPERVISOR_LOCK', _DEFAULT_SUPERVISOR_LOCK_PATH)


def _read_lock_pid(path):
    try:
        with open(path) as fh:
            return int(fh.read().strip())
    except (OSError, ValueError):
        return None


def _pid_alive(pid):
    """Probe for a process without delivering anything.

    EPERM means it exists but belongs to another user -- still alive.
    """
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
    """Claim the single-supervisor lock. Returns (True, None) or (False, holder_pid).

    A lock naming a dead PID is stale -- the owner was SIGKILLed before it could
    clean up -- and gets reclaimed rather than blocking forever. Refusing to ever
    start again after one `kill -9` would be a worse failure than the one this
    guards against, and the reclaiming instance's own _sweep_stale_pgids() is
    precisely what cleans up after that dead owner.

    A live holder is refused unconditionally, including in the (only reachable
    after PID reuse) case where the recorded PID is our own: the operator gets a
    message naming the lock file and can delete it, whereas reading it as "that's
    me, so it must be stale" would green-light a second supervisor in exactly the
    situation this exists to prevent.
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
            # unguarded, which puts us right back in the double-stack case.
            return False, None
        with os.fdopen(fd, 'w') as fh:
            fh.write(str(os.getpid()))
        return True, None
    return False, _read_lock_pid(path)


def release_singleton_lock(path):
    """Drop the lock, but only if it is still ours.

    Never unlink a lock a successor legitimately reclaimed after we were
    declared stale.
    """
    if _read_lock_pid(path) == os.getpid():
        try:
            os.unlink(path)
        except OSError:
            pass


class _ComponentProcess:
    """One tracked `ros2 launch` subprocess -- one entry per components.yaml launches
    list item; several per component for e.g. perception (camera + detection).

    manually_stopped and the auto-restart timestamp history are per-subprocess (not
    per logical component) deliberately: _start_component always builds fresh
    _ComponentProcess instances (see start()'s callers), so both reset for free
    whenever a component is (re)started via either service -- no separate "clear"
    step needed.
    """

    def __init__(self, package, launch_file, args, log_path):
        self.package = package
        self.launch_file = launch_file
        self.args = args
        self.log_path = log_path
        self.popen = None
        self.log_file = None
        self.manually_stopped = False
        # Monotonic timestamps of recent watchdog-triggered (not manually-requested)
        # respawns -- see ComponentSupervisorNode._consume_restart_budget.
        self.auto_restart_timestamps = []

    @property
    def cmd(self):
        cmd = ['ros2', 'launch', self.package, self.launch_file]
        cmd += [f'{k}:={v}' for k, v in self.args.items()]
        return cmd

    def start(self):
        self.log_file = open(self.log_path, 'w')
        self.popen = subprocess.Popen(
            self.cmd, stdout=self.log_file, stderr=subprocess.STDOUT,
            start_new_session=True,
        )


class ComponentSupervisorNode(Node):

    # Final-shutdown-only grace period (shutdown_all(), and therefore both the new
    # signal handlers and the startup sweep -- see module docstring) -- deliberately
    # shorter than restart_timeout_sec, which stays whatever it was for the
    # RestartComponent/ComponentControl RESTART paths (still their own, unaffected
    # default). Kept short so a full shutdown across every tracked component can't
    # compound into minutes if a few are slow to exit.
    _SHUTDOWN_GRACE_SEC = 5.0

    def __init__(self):
        super().__init__('component_supervisor_node')

        components_config = str(self.declare_parameter('components_config', '').value)
        self.restart_timeout_sec = float(
            self.declare_parameter('restart_timeout_sec', 10.0).value)
        self.log_dir = os.path.expanduser(
            str(self.declare_parameter('log_dir', '/tmp/component_supervisor').value))
        os.makedirs(self.log_dir, exist_ok=True)
        self.watchdog_period_sec = float(
            self.declare_parameter('watchdog_period_sec', 2.0).value)
        self.max_auto_restarts = int(
            self.declare_parameter('max_auto_restarts', 3).value)
        self.restart_budget_window_sec = float(
            self.declare_parameter('restart_budget_window_sec', 60.0).value)
        # calibration-single-source-of-truth pass: f1tenth_params/config/
        # stack_params.yaml's own `calibration` key is now the ONLY declaration
        # of this value -- components.yaml's `hardware` entry no longer hardcodes
        # a 'true'/'false' literal (see that file's own `hardware` entry comment).
        # Wired from supervisor_bringup.launch.py's own `calibration` launch
        # argument (default sourced from that same stack_params.yaml key via
        # get_default(), same pattern as components_config/log_dir/etc. above);
        # the True default here is only a fallback for a bare `ros2 run`/test
        # invocation that skips the launch file entirely. Consumed below, right
        # after the registry loads: injected into 'hardware's own args dict so
        # both the real `ros2 launch ... calibration:=<value>` invocation
        # (_start_component, via that same dict) and _hardware_will_calibrate()
        # read the identical live value -- one source, not a second read path.
        self.calibration = bool(self.declare_parameter('calibration', True).value)
        # component-auto-start pass: whether 'intelligence' (llama-server) auto-
        # starts -- same "declared parameter, wired from supervisor_bringup.
        # launch.py's own launch argument, default sourced from stack_params.yaml"
        # pattern as self.calibration directly above, not the get_value()-from-
        # _CONDITIONAL_AUTO_START path 'behavior' still uses (see that dict's own
        # comment for why 'intelligence' needed to be different: a real CLI
        # override, like calibration has, not a value only stack_params.yaml
        # itself can change). The True default here is only a fallback for a
        # bare `ros2 run`/test invocation that skips the launch file entirely --
        # matches stack_params.yaml's own enable_intelligence default (flipped
        # true there -- 'intelligence' now auto-starts with the supervisor by
        # default; this literal was previously False and had drifted out of
        # sync with that key, same manual-mirroring caveat every other literal
        # default in this __init__ already carries, e.g. self.calibration
        # above -- stack_params.yaml/the launch-passed parameter is still the
        # actual source of truth, this is only what a launch-file-bypassing
        # invocation falls back to).
        self.enable_intelligence = bool(
            self.declare_parameter('enable_intelligence', True).value)
        # 'localization' deferred-start (calibration-restart gap fix) -- see module
        # docstring's own paragraph and _defer_localization_start()'s docstring.
        # Generous margin above the documented ~60-130s calibration-cycle figure
        # (f1tenth_hardware/vesc.launch.py's own module docstring, diagnostics_
        # server_node.py's own docstring) -- fail-open backstop, not a tight bound.
        self.localization_calibration_wait_timeout_sec = float(
            self.declare_parameter('localization_calibration_wait_timeout_sec', 180.0).value)

        # {component_name: [pid, ...]} mirror of every live process group this node
        # has spawned -- see module docstring's "Startup safety sweep" paragraph.
        # Reuses log_dir (already the established runtime-state directory for this
        # node) rather than inventing a separate location.
        self._pgid_file = os.path.join(self.log_dir, 'tracked_pgids.json')
        # _shutdown_started flips the instant a shutdown attempt begins (before it's
        # necessarily finished); _shutdown_done only once it actually has. The gap
        # between them is what main()'s _handle_signal uses to detect a repeat
        # signal arriving mid-shutdown -- see force_kill_all()'s own docstring.
        self._shutdown_started = False
        self._shutdown_done = False
        self._sweep_stale_pgids()

        with open(components_config) as f:
            registry_raw = yaml.safe_load(f)['components']
        self._registry = {
            name: [dict(entry, args=entry.get('args', {})) for entry in entries]
            for name, entries in registry_raw.items()
        }

        # calibration-single-source-of-truth pass (see self.calibration's own
        # declare_parameter comment above) -- pulled out to its own method
        # (defined alongside _hardware_will_calibrate() further down) purely so
        # it's unit-testable the same duck-typed way as the rest of this file's
        # calibration/deferred-start logic, with no rclpy context needed. Must
        # run before anything reads or launches from self._registry.
        self._apply_calibration_override()

        # Some individual launch files within a multi-launch component legitimately
        # do nothing at all when their own feature flag is off -- an empty (or
        # IfCondition-gated-to-nothing) LaunchDescription, so `ros2 launch` has
        # nothing left to track and exits cleanly (code 0) almost immediately.
        # The watchdog (_on_watchdog_tick below) can't distinguish that from a real
        # crash -- it treats ANY exit as needing a respawn, burns through the whole
        # restart budget in seconds, then gets stuck re-logging "crashed ... NOT
        # auto-respawning" every tick forever. A real, observed failure mode (this
        # is what a "perception crashes" report turned out to be: lidar.launch.py's
        # urg_node is IfCondition-gated on use_lidar, default false). NOT fixed by
        # changing the watchdog to ignore exit-code-0 in general -- vesc.launch.py's
        # own crash handler tears down its whole launch tree via Shutdown() when
        # vesc_driver_node dies for real, and that also exits 0 (a "clean launch-
        # tool shutdown" from ros2 launch's own perspective), so that would silently
        # disable the hardware crash-recovery safety net instead. Filtered out of
        # the registry entirely here instead -- the same idea _CONDITIONAL_AUTO_START
        # below already applies to a whole component (behavior/intelligence), just
        # applied to one entry inside a multi-launch component instead. Extend this
        # dict if another launch file within a multi-launch component ever grows the
        # same shape (system_observer.launch.py/enable_sys_obs is the next-most-
        # likely candidate -- see f1tenth_diagnostics/README.md -- currently dormant
        # only because enable_sys_obs defaults true).
        #
        # slam.launch.py/costmap.launch.py/enable_slam added by the first SLAM
        # integration pass -- 'slam' has TWO entries (slam.launch.py itself,
        # plus the two-layer-costmap bringup, both gated on the same enable_
        # slam flag -- see costmap.launch.py's own module docstring for why
        # they share the flag rather than each having their own), so with
        # enable_slam false (the default) BOTH get filtered out, emptying
        # self._registry['slam'] entirely -- confirmed this is handled
        # cleanly: _start_component's own for loop over an empty list just
        # tracks zero processes for that component, no crash, nothing for the
        # watchdog to (mis)respawn.
        _SKIP_LAUNCH_FILE_IF_DISABLED = {
            'lidar.launch.py': 'use_lidar',
            'slam.launch.py': 'enable_slam',
            'costmap.launch.py': 'enable_slam',
        }
        for entries in self._registry.values():
            entries[:] = [
                entry for entry in entries
                if not (entry['launch_file'] in _SKIP_LAUNCH_FILE_IF_DISABLED
                        and not get_value(_SKIP_LAUNCH_FILE_IF_DISABLED[entry['launch_file']]))
            ]

        self._processes = {}  # {component_name: [_ComponentProcess, ...]}

        categorized = _ALWAYS_AUTO_START | _NEVER_AUTO_START | set(_CONDITIONAL_AUTO_START)
        uncategorized = set(self._registry) - categorized
        if uncategorized:
            self.get_logger().warn(
                f'[component_supervisor] {sorted(uncategorized)} in components.yaml '
                f'but not in any of _ALWAYS_AUTO_START/_NEVER_AUTO_START/'
                f'_CONDITIONAL_AUTO_START -- will NOT auto-start. Add it to one of '
                f'those sets in component_supervisor_node.py.'
            )

        auto_start = set(_ALWAYS_AUTO_START)
        for name, flag in _CONDITIONAL_AUTO_START.items():
            # 'intelligence' reads this node's own live declared parameter
            # (self.enable_intelligence), not a stack_params.yaml get_value()
            # lookup -- see _CONDITIONAL_AUTO_START's own comment for why.
            # Every other entry ('behavior' today) keeps the original
            # get_value(flag) path unchanged.
            gate = self.enable_intelligence if name == 'intelligence' else get_value(flag)
            if gate:
                auto_start.add(name)

        self.get_logger().info(
            f'[component_supervisor] Registry: {sorted(self._registry)}. '
            f'Auto-starting: {sorted(auto_start)}. Log dir: {self.log_dir}'
        )

        # 'localization' deferred-start (calibration-restart gap fix) -- see module
        # docstring's own paragraph. Only applies when BOTH 'hardware' and
        # 'localization' are actually auto-starting together AND 'hardware' is
        # actually about to run a real calibration cycle (calibration:=true in its
        # own registered launch args) -- every other case (calibration:=false, or a
        # customized registry/auto_start that doesn't include one or the other)
        # falls through to the exact same immediate-start behavior as before this
        # fix, unchanged.
        defer_localization = (
            'localization' in auto_start
            and 'hardware' in auto_start
            and self._hardware_will_calibrate()
        )
        # Default True ("nothing pending, safe no-op") -- only set False, briefly,
        # inside _defer_localization_start() itself. Every guard in this file that
        # checks this flag (the two calibration-status callbacks, and the manual-
        # override check in _on_restart_component/_on_control_component) is then
        # always safe to evaluate unconditionally, deferral or not.
        self._localization_deferred_started = True
        self._localization_calib_sub = None
        self._localization_calib_timeout_timer = None

        for name in auto_start:
            if name == 'localization' and defer_localization:
                continue  # started later -- see _defer_localization_start() below
            self._start_component(name)
        if defer_localization:
            self._defer_localization_start()

        self._restart_srv = self.create_service(
            RestartComponent, 'restart_component', self._on_restart_component)
        self._control_srv = self.create_service(
            ComponentControl, '~/control_component', self._on_control_component)
        self._run_calibration_srv = self.create_service(
            Trigger, '~/run_calibration', self._on_run_calibration)

        self._watchdog_timer = self.create_timer(
            self.watchdog_period_sec, self._on_watchdog_tick)

    # -- process lifecycle ----------------------------------------------------

    def _log_path(self, package, launch_file):
        safe_name = f'{package}_{launch_file}'.replace('/', '_')
        return os.path.join(self.log_dir, f'{safe_name}.log')

    # -- pgid tracking file (startup safety sweep -- see module docstring) --------

    def _write_pgid_file(self):
        """Overwrite self._pgid_file with every currently-live process group this
        node has spawned, keyed by component name. Called after every state change
        (start/stop/respawn) so it's always an accurate snapshot -- if this process
        dies uncleanly, the next instance's _sweep_stale_pgids() trusts whatever was
        last written here.
        """
        state = {
            name: [proc.popen.pid for proc in procs
                   if proc.popen is not None and proc.popen.poll() is None]
            for name, procs in self._processes.items()
        }
        state = {name: pids for name, pids in state.items() if pids}
        try:
            with open(self._pgid_file, 'w') as f:
                json.dump(state, f)
        except OSError as exc:
            self.get_logger().warn(
                f'[component_supervisor] Failed to write pgid tracking file '
                f'{self._pgid_file}: {exc}')

    def _clear_pgid_file(self):
        try:
            os.remove(self._pgid_file)
        except FileNotFoundError:
            pass
        except OSError as exc:
            self.get_logger().warn(
                f'[component_supervisor] Failed to remove pgid tracking file '
                f'{self._pgid_file}: {exc}')

    def _looks_like_ros2_launch(self, pid):
        """Best-effort guard against pid reuse: a pid recorded in a stale pgid file
        could, in principle, have been recycled by the OS for an unrelated process
        since that file was written. Cheap enough to always check before killpg-ing
        something this instance never spawned itself.
        """
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmdline = f.read().decode(errors='replace')
        except OSError:
            return False
        return 'ros2' in cmdline and 'launch' in cmdline

    def _kill_pgid(self, pgid, label):
        """SIGTERM-then-SIGKILL a process group by raw pid, polling for exit instead
        of Popen.wait() -- used by the startup sweep for groups spawned by a
        *previous*, now-dead instance of this node, so they aren't this process's
        children and can't be waited on. Mirrors _stop_component's escalation shape;
        SIGTERM rather than SIGINT since there's no live `ros2 launch` here for
        SIGINT's graceful-nested-teardown semantics to matter -- these are already
        orphaned, we just want them gone.
        """
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return f'{label} (pgid {pgid}): already gone'
        deadline = time.monotonic() + self._SHUTDOWN_GRACE_SEC
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return f'{label} (pgid {pgid}): exited after SIGTERM'
            time.sleep(0.2)
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            return f'{label} (pgid {pgid}): exited after SIGTERM'
        return (f'{label} (pgid {pgid}): killed via SIGKILL after '
                f'{self._SHUTDOWN_GRACE_SEC:.0f}s grace')

    def _sweep_stale_pgids(self):
        """Startup safety net for an unclean previous exit (e.g. `kill -9` on this
        node's own pid, an OOM kill -- anything that skips shutdown_all() entirely).
        start_new_session=True means every spawned process group survives its
        parent's death, reparented to init -- nothing else would ever notice or
        clean them up, and the next launch of this node would spawn a second full
        set on top of them. Called once, before anything is spawned.
        """
        if not os.path.exists(self._pgid_file):
            # Still sweep FastDDS SHM residue -- see this method's tail comment
            # and module docstring; that residue isn't tied to this file at all.
            self._sweep_stale_fastrtps_shm('startup')
            return
        try:
            with open(self._pgid_file) as f:
                stale = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'[component_supervisor] Could not read stale pgid file '
                f'{self._pgid_file}: {exc} -- ignoring.')
            self._clear_pgid_file()
            self._sweep_stale_fastrtps_shm('startup')
            return

        cleaned = []
        for name, pids in stale.items():
            for pid in pids:
                try:
                    os.killpg(pid, 0)
                except (ProcessLookupError, PermissionError):
                    continue  # already gone, or pid reused by something we don't own
                if not self._looks_like_ros2_launch(pid):
                    self.get_logger().warn(
                        f"[component_supervisor] Stale pgid {pid} recorded for "
                        f"'{name}' no longer looks like a ros2 launch process -- "
                        f'skipping (likely pid reuse).')
                    continue
                cleaned.append(self._kill_pgid(pid, f"stale '{name}'"))

        if cleaned:
            self.get_logger().warn(
                '[component_supervisor] Found leftover process groups from an '
                f'unclean previous shutdown -- cleaned up: {"; ".join(cleaned)}')
        self._clear_pgid_file()
        # Runs regardless of whether the pgid sweep above found anything of its
        # own -- FastDDS SHM residue can come from any previous process on this
        # machine, not just this node's own tracked components. See module
        # docstring's "FastDDS shared-memory hygiene" paragraph.
        self._sweep_stale_fastrtps_shm('startup')

    # -- FastDDS SHM hygiene (see module docstring) --------------------------------

    def _fastrtps_shm_candidates(self):
        """Every /dev/shm entry that looks like FastDDS SHM transport state --
        see module docstring for why a single glob is enough to catch both file
        families this needs to match.
        """
        return sorted(
            glob.glob(os.path.join(_FASTRTPS_SHM_DIR, _FASTRTPS_SHM_GLOB_PATTERN)))

    def _paths_open_by_any_process(self):
        """Every absolute path currently referenced by some live process's open
        file descriptors or memory mappings -- one scan of /proc, reused against
        every candidate file in the same sweep call rather than re-scanning per
        file. Checks both /proc/<pid>/fd (an open()'d file or named semaphore)
        and /proc/<pid>/maps (mmap'd shared memory can have its creating fd
        closed right after the mapping is made, a common pattern for POSIX
        shared memory -- relying on fd/ alone would misclassify those as
        orphaned). Best-effort: a process that exits mid-scan, or whose /proc
        entries this node can't read, is silently skipped rather than raising --
        this is a hygiene sweep, not something any component's startup should
        ever fail over.
        """
        open_paths = set()
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            fd_dir = f'/proc/{entry}/fd'
            try:
                for fd in os.listdir(fd_dir):
                    try:
                        open_paths.add(os.readlink(os.path.join(fd_dir, fd)))
                    except OSError:
                        continue
            except OSError:
                continue  # process exited mid-scan, or /proc/<pid>/fd unreadable
            try:
                with open(f'/proc/{entry}/maps') as f:
                    for line in f:
                        fields = line.split()
                        if len(fields) >= 6 and fields[5].startswith(
                                _FASTRTPS_SHM_DIR + '/'):
                            open_paths.add(fields[5])
            except OSError:
                continue
        return open_paths

    def _other_supervisor_pids(self):
        """Live component_supervisor_node processes other than this one. A direct,
        explicit check kept separate from the generic liveness scan above: per
        the requester's own callout, a second live supervisor instance is the one
        case where "nothing has this fd open right now" isn't a good enough
        reason to delete something, since that sibling's own components could
        still (re)acquire it. Refuse outright rather than reason about it further.

        Matches on argv element *basename* (os.path.basename(arg) ==
        'component_supervisor_node'), not a substring search over the whole
        cmdline blob -- caught live during this feature's own verification: a
        naive substring check self-matched the verification harness's own
        `python3 -c "...component_supervisor_node..."` invocation (the entire
        -c script text is one argv element), which would have made this refuse
        to ever sweep anything the moment it was tested. Checking argv basenames
        instead only matches the actual launched executable path (.../lib/
        f1tenth_bringup/component_supervisor_node, no extension, exactly how
        ros2 run installs console-script-style entry points), not incidental
        mentions of the name anywhere else in a command line.
        """
        my_pid = os.getpid()
        others = []
        for entry in os.listdir('/proc'):
            if not entry.isdigit() or int(entry) == my_pid:
                continue
            try:
                with open(f'/proc/{entry}/cmdline', 'rb') as f:
                    argv = f.read().split(b'\0')
            except OSError:
                continue
            if any(os.path.basename(arg.decode(errors='replace'))
                   == 'component_supervisor_node' for arg in argv if arg):
                others.append(int(entry))
        return others

    def _sweep_stale_fastrtps_shm(self, context):
        """Remove /dev/shm FastDDS files nothing has open anymore. `context` is
        just a short label for the log line (e.g. 'startup', 'shutdown') -- no
        behavior depends on it. See module docstring's "FastDDS shared-memory
        hygiene" paragraph for why this exists and when it's called.
        """
        others = self._other_supervisor_pids()
        if others:
            self.get_logger().error(
                f'[component_supervisor] FastDDS SHM sweep ({context}): refusing '
                f'to remove anything -- {len(others)} other '
                f'component_supervisor_node process(es) still alive (pid(s) '
                f'{others}). Deleting shared transport files while a sibling '
                'instance may still be using them would be actively harmful, '
                'not just wasteful.'
            )
            return

        candidates = self._fastrtps_shm_candidates()
        if not candidates:
            self.get_logger().info(
                f'[component_supervisor] FastDDS SHM sweep ({context}): no '
                f'fastrtps files found under {_FASTRTPS_SHM_DIR} -- nothing to do.'
            )
            return

        open_paths = self._paths_open_by_any_process()
        stale = [p for p in candidates if p not in open_paths]
        live = [p for p in candidates if p in open_paths]

        if not stale:
            self.get_logger().info(
                f'[component_supervisor] FastDDS SHM sweep ({context}): '
                f'{len(candidates)} fastrtps file(s) found, all still held open '
                'by a live process -- nothing removed.'
            )
            return

        removed = []
        failed = []
        for path in stale:
            try:
                os.remove(path)
                removed.append(path)
            except OSError as exc:
                failed.append((path, exc))

        self.get_logger().warn(
            f'[component_supervisor] FastDDS SHM sweep ({context}): found '
            f'{len(candidates)} fastrtps file(s) ({len(live)} still in use, '
            f'{len(stale)} orphaned) -- removed {len(removed)}/{len(stale)} '
            f'orphaned file(s). Sample removed: {removed[:5]}.'
        )
        if failed:
            self.get_logger().warn(
                f'[component_supervisor] FastDDS SHM sweep ({context}): failed '
                f'to remove {len(failed)} file(s): '
                f'{[(p, str(e)) for p, e in failed[:5]]}.'
            )

    # -- process lifecycle proper --------------------------------------------------

    def _start_component(self, name):
        procs = []
        for entry in self._registry[name]:
            proc = _ComponentProcess(
                entry['package'], entry['launch_file'], entry['args'],
                self._log_path(entry['package'], entry['launch_file']))
            proc.start()
            self.get_logger().info(
                f"[component_supervisor] Started '{name}': {' '.join(proc.cmd)} "
                f'(pid {proc.popen.pid}, log: {proc.log_path})')
            procs.append(proc)
        self._processes[name] = procs
        self._write_pgid_file()

    # -- 'localization' deferred-start (calibration-restart gap fix) --------------
    # See module docstring's own matching paragraph for the bug this closes and
    # the full reasoning. Four pieces: a static check of 'hardware's own launch
    # args (does this even apply this run), the deferred-start setup itself (one
    # subscription + one timeout timer), the two callbacks that can each trigger
    # the actual start, and one idempotent resolver both of them (and any manual
    # service request naming 'localization') funnel through so it only ever
    # actually happens once.

    def _apply_calibration_override(self):
        """calibration-single-source-of-truth pass: overwrites 'hardware's own
        registered args with self.calibration (the live declared parameter,
        wired from supervisor_bringup.launch.py's own `calibration` launch
        argument, itself defaulting from f1tenth_params/config/stack_params.yaml's
        `calibration` key) -- components.yaml's `hardware` entry no longer
        carries a 'calibration' key of its own at all (see that file's own
        `hardware` entry comment). Called once, from __init__, right after the
        registry loads and before anything reads or launches from it -- both the
        real `ros2 launch ... calibration:=<value>` invocation (_start_component,
        via this same args dict) and _hardware_will_calibrate() (ditto) then see
        the identical live value; there is no second read path. Deliberately
        scoped to 'hardware' only, not 'calibrate_hardware' -- that component
        stays an explicit, always-calibrate action, untouched by this toggle
        (components.yaml itself still hardcodes 'true' there). Mutates each
        entry's args dict in place -- safe: every entry in self._registry is
        built fresh from this process's own yaml.safe_load() of components.yaml
        (see __init__), not shared with anything else."""
        for entry in self._registry.get('hardware', []):
            entry['args']['calibration'] = 'true' if self.calibration else 'false'

    def _hardware_will_calibrate(self):
        """True if any of 'hardware' component's own registered launch entries
        passes calibration:=true -- as of the calibration-single-source-of-truth
        pass, that's the live self.calibration parameter as injected by
        _apply_calibration_override() above, not a components.yaml literal (see
        that method's own docstring). Still read directly from the already-
        parsed registry rather than re-deriving from self.calibration/stack_
        params.yaml separately, so this can never disagree with what 'hardware'
        is actually about to be launched with."""
        return any(
            str(entry.get('args', {}).get('calibration', '')).strip().lower() == 'true'
            for entry in self._registry.get('hardware', [])
        )

    def _defer_localization_start(self):
        """Called at most once, from __init__, only when defer_localization was
        True there (see that call site's own comment for the exact conditions).
        Watches /calibration/in_progress (f1tenth_diagnostics' diagnostics_server_
        node, TRANSIENT_LOCAL so a late-attaching subscription still gets the
        current value immediately -- QoS matched exactly to that node's own
        publisher) for a True->False transition, then starts 'localization'.
        Bounded by localization_calibration_wait_timeout_sec regardless (fail-open
        -- see module docstring)."""
        self._calib_seen_in_progress = False
        self._localization_deferred_started = False
        calibration_status_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._localization_calib_sub = self.create_subscription(
            Bool, '/calibration/in_progress',
            self._on_calibration_status_for_localization, calibration_status_qos)
        self._localization_calib_timeout_timer = self.create_timer(
            self.localization_calibration_wait_timeout_sec,
            self._on_localization_calibration_wait_timeout)
        self.get_logger().info(
            "[component_supervisor] Deferring 'localization' auto-start until "
            'startup calibration genuinely completes (watching '
            '/calibration/in_progress for a True->False transition, timeout '
            f'{self.localization_calibration_wait_timeout_sec:.0f}s) -- avoids '
            'ekf_filter_node/ekf_global_filter_node ever integrating pre-'
            "calibration IMU data. See this method's own docstring / module "
            "docstring's 'localization deferred-start' paragraph."
        )

    def _on_calibration_status_for_localization(self, msg):
        if msg.data:
            self._calib_seen_in_progress = True
            return
        if not self._calib_seen_in_progress:
            return  # still the seeded/initial False -- not a real completion yet
        if self._resolve_localization_deferral():
            self.get_logger().info(
                "[component_supervisor] 'localization' STARTED -- startup "
                'calibration completed (/calibration/in_progress True->False '
                'observed). Deferred specifically so ekf_filter_node/'
                'ekf_global_filter_node never integrate pre-calibration IMU '
                "data -- see this file's own module docstring, 'localization "
                "deferred-start' paragraph, if a static EKF yaw offset like the "
                'one the investigation behind this fix found ever recurs.'
            )
            self._start_component('localization')

    def _on_localization_calibration_wait_timeout(self):
        if self._resolve_localization_deferral():
            self.get_logger().warn(
                "[component_supervisor] 'localization' START TIMED OUT waiting for "
                f'startup calibration ({self.localization_calibration_wait_timeout_sec:.0f}s) '
                '-- starting anyway (fail-open, same "never block startup '
                'indefinitely" discipline vesc.launch.py\'s own calibration-'
                'failure handling already uses). Either /calibration/in_progress '
                'was never observed True (diagnostics_server_node slow to start, '
                'or calibration finished implausibly fast), or the calibration '
                "cycle itself is taking abnormally long -- check hardware's own "
                'launch log.'
            )
            self._start_component('localization')

    def _resolve_localization_deferral(self):
        """Idempotent: tears down the deferred-start machinery (subscription +
        timeout timer) the FIRST time any of its three triggers reaches here (the
        completion callback, the timeout callback, or a manual RestartComponent/
        ComponentControl request naming 'localization' arriving mid-wait -- see
        _on_restart_component/_on_control_component's own matching calls). Returns
        True the first time (caller should go ahead and act -- start the
        component, or just log the cancellation), False on every later call
        (already resolved, caller should no-op) -- callers never need to check
        self._localization_deferred_started directly, just this return value."""
        if self._localization_deferred_started:
            return False
        self._localization_deferred_started = True
        if self._localization_calib_sub is not None:
            self.destroy_subscription(self._localization_calib_sub)
            self._localization_calib_sub = None
        if self._localization_calib_timeout_timer is not None:
            self._localization_calib_timeout_timer.cancel()
            self._localization_calib_timeout_timer = None
        return True

    def _stop_component(self, name, timeout=None):
        """SIGINT (then SIGKILL if needed) every tracked process for `name`. Returns
        one status string per process, e.g. 'clean SIGINT exit in 1.2s' or 'killed
        via SIGKILL after 10.0s timeout' -- so callers (the service response, or
        shutdown logging) can tell whether each process was well-behaved.

        timeout defaults to restart_timeout_sec (the RestartComponent/ComponentControl
        RESTART paths' existing, unchanged behavior) -- shutdown_all() passes
        _SHUTDOWN_GRACE_SEC instead, its own shorter, dedicated grace period.
        """
        if timeout is None:
            timeout = self.restart_timeout_sec
        results = []
        for proc in self._processes.get(name, []):
            if proc.popen is None or proc.popen.poll() is not None:
                results.append(f'{proc.launch_file}: was not running')
                continue
            # start_new_session=True made this process its own session leader, so its
            # pgid is definitionally its own pid.
            pgid = proc.popen.pid
            start = time.monotonic()
            try:
                os.killpg(pgid, signal.SIGINT)
            except ProcessLookupError:
                results.append(f'{proc.launch_file}: exited before SIGINT was sent')
                continue
            try:
                proc.popen.wait(timeout=timeout)
                elapsed = time.monotonic() - start
                results.append(f'{proc.launch_file}: clean SIGINT exit in {elapsed:.1f}s')
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                    proc.popen.wait(timeout=5.0)
                except ProcessLookupError:
                    pass
                results.append(
                    f'{proc.launch_file}: killed via SIGKILL after '
                    f'{timeout:.1f}s timeout')
            finally:
                if proc.log_file:
                    proc.log_file.close()
        self._write_pgid_file()
        return results

    def _shutdown_component(self, name):
        """Like _stop_component, but also marks every tracked process for `name` as
        manually_stopped -- unlike a plain restart, the watchdog must NOT bring these
        back on its own afterward. Mutates the existing _ComponentProcess objects in
        place (rather than replacing them, the way _start_component does), since
        there's nothing to replace them with -- the component is meant to stay down.
        """
        results = self._stop_component(name)
        for proc in self._processes.get(name, []):
            proc.manually_stopped = True
        return results

    def _stop_all_components(self, timeout):
        """shutdown_all()'s own stop routine -- NOT a loop over _stop_component(),
        deliberately: that would SIGINT one component, wait up to `timeout` for it,
        THEN move to the next, so total wall-clock cost is sum(per-component wait),
        not max(per-component wait). With ~10 tracked components that summed cost
        routinely exceeds `ros2 launch`'s own default ~5s patience for this node's
        process to exit after SIGINT before it escalates to SIGTERM itself -- see
        module docstring's "Clean shutdown" paragraph for how that surfaced live.

        Here every process group gets SIGINT up front (a burst of os.killpg calls,
        no waiting between them), then all of them are polled concurrently against
        one shared deadline, with only the stragglers still alive once it passes
        escalated to SIGKILL. Total cost is ~max(per-component exit time), not the
        sum -- comfortably fits inside external patience for the normal case where
        most components exit within a second or so of SIGINT.
        """
        results = {}
        pending = []
        for name, procs in self._processes.items():
            for proc in procs:
                if proc.popen is None or proc.popen.poll() is not None:
                    results[(name, proc.launch_file)] = 'was not running'
                    continue
                try:
                    os.killpg(proc.popen.pid, signal.SIGINT)
                except ProcessLookupError:
                    results[(name, proc.launch_file)] = 'exited before SIGINT was sent'
                    continue
                pending.append((name, proc))

        start = time.monotonic()
        deadline = start + timeout
        while pending and time.monotonic() < deadline:
            still_pending = []
            for name, proc in pending:
                if proc.popen.poll() is not None:
                    elapsed = time.monotonic() - start
                    results[(name, proc.launch_file)] = (
                        f'clean SIGINT exit in {elapsed:.1f}s')
                else:
                    still_pending.append((name, proc))
            pending = still_pending
            if pending:
                time.sleep(0.1)

        for name, proc in pending:
            try:
                os.killpg(proc.popen.pid, signal.SIGKILL)
                proc.popen.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
            results[(name, proc.launch_file)] = (
                f'killed via SIGKILL after {timeout:.1f}s timeout')

        for procs in self._processes.values():
            for proc in procs:
                if proc.log_file:
                    proc.log_file.close()

        self._write_pgid_file()
        return results

    def force_kill_all(self):
        """SIGKILL every still-tracked process group immediately, no SIGINT, no
        waiting. Only reached when a second shutdown signal arrives while a first,
        graceful shutdown_all() is still in progress (see main()'s _handle_signal)
        -- e.g. `ros2 launch` itself escalating from SIGINT to SIGTERM because
        the first attempt didn't finish within its own patience. At that point
        trying to still be graceful is how components get abandoned (the actual
        live bug this fix closes -- see module docstring): a repeat signal here
        means "something is out of patience," so the only response that can't
        leave orphans regardless of how slow anything is is an immediate,
        unconditional kill.
        """
        self._shutdown_started = True
        for procs in self._processes.values():
            for proc in procs:
                proc.manually_stopped = True
                if proc.popen is None or proc.popen.poll() is not None:
                    continue
                try:
                    os.killpg(proc.popen.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                if proc.log_file:
                    proc.log_file.close()
        self._shutdown_done = True
        self._clear_pgid_file()
        # SIGKILL above doesn't wait/poll (deliberately -- see this method's own
        # docstring, that's the whole point of this path), so give the kernel a
        # brief moment to actually reclaim each process's fds/mappings before
        # checking what FastDDS SHM residue this shutdown left behind. Purely in
        # service of the new sweep below -- does not change the kill itself.
        time.sleep(0.3)
        self._sweep_stale_fastrtps_shm('shutdown (force-kill)')

    # -- watchdog -----------------------------------------------------------------

    def _consume_restart_budget(self, proc):
        """Prune this process's auto-restart timestamps older than
        restart_budget_window_sec, then check/record a new attempt. Returns True if
        within budget (go ahead and respawn), False if exhausted (leave it down).
        """
        now = time.monotonic()
        proc.auto_restart_timestamps = [
            t for t in proc.auto_restart_timestamps
            if now - t < self.restart_budget_window_sec
        ]
        if len(proc.auto_restart_timestamps) >= self.max_auto_restarts:
            return False
        proc.auto_restart_timestamps.append(now)
        return True

    def _on_watchdog_tick(self):
        respawned = False
        for name, procs in self._processes.items():
            for proc in procs:
                if proc.manually_stopped:
                    continue
                if proc.popen is None or proc.popen.poll() is None:
                    continue  # never started, or still alive -- nothing to do
                returncode = proc.popen.returncode
                if not self._consume_restart_budget(proc):
                    self.get_logger().error(
                        f"[component_supervisor] '{name}' ({proc.launch_file}) "
                        f'crashed (exit {returncode}) and exceeded its restart '
                        f'budget ({self.max_auto_restarts} within '
                        f'{self.restart_budget_window_sec:.0f}s) -- NOT '
                        f'auto-respawning. Use ~/control_component START or RESTART '
                        f'to bring it back manually.')
                    continue
                self.get_logger().warn(
                    f"[component_supervisor] '{name}' ({proc.launch_file}) crashed "
                    f'(exit {returncode}) -- auto-respawning.')
                if proc.log_file:
                    proc.log_file.close()
                proc.start()
                respawned = True
        # A respawn gets a fresh pid -- refresh the on-disk mirror so a later
        # unclean-exit sweep targets the current process, not the crashed one.
        if respawned:
            self._write_pgid_file()

    # -- service ----------------------------------------------------------------

    def _on_restart_component(self, request, response):
        name = request.component_name
        if name not in self._registry:
            response.success = False
            response.message = (
                f"Unknown component '{name}'. Valid names: {sorted(self._registry)}")
            self.get_logger().warn(f'[component_supervisor] {response.message}')
            return response

        self.get_logger().info(f"[component_supervisor] Restart requested: '{name}'")
        # 'localization' deferred-start (calibration-restart gap fix): a manual
        # request always wins over the automatic post-calibration wait -- cancel
        # it here first so it can't ALSO fire _start_component('localization')
        # later and double-start what this call is about to (re)start itself. A
        # no-op (returns False) if no wait is pending, or it already resolved.
        if name == 'localization' and self._resolve_localization_deferral():
            self.get_logger().info(
                "[component_supervisor] Manual restart of 'localization' arrived "
                'while its post-calibration deferred start was still pending -- '
                'cancelling the wait, honoring the manual request now.')
        stop_results = self._stop_component(name)
        self._start_component(name)

        response.success = True
        response.message = f"'{name}' restarted -- " + '; '.join(stop_results)
        self.get_logger().info(f'[component_supervisor] {response.message}')
        return response

    def _on_control_component(self, request, response):
        name = request.component_name
        if name not in self._registry:
            response.success = False
            response.message = (
                f"Unknown component '{name}'. Valid names: {sorted(self._registry)}")
            self.get_logger().warn(f'[component_supervisor] {response.message}')
            return response

        # 'localization' deferred-start (calibration-restart gap fix): cancel any
        # pending post-calibration wait before acting on ANY of SHUTDOWN/START/
        # RESTART for 'localization' -- a manual SHUTDOWN in particular must
        # actually prevent the later automatic start (there would otherwise be
        # nothing yet in self._processes['localization'] for SHUTDOWN's own
        # manually_stopped marking to apply to, and the deferred wait would still
        # go on to start it once calibration finished, silently overriding the
        # SHUTDOWN). No-op (returns False) if no wait is pending, or it already
        # resolved.
        if name == 'localization' and self._resolve_localization_deferral():
            self.get_logger().info(
                "[component_supervisor] Manual request for 'localization' arrived "
                'while its post-calibration deferred start was still pending -- '
                'cancelling the wait, honoring the manual request now.')

        if request.action == ComponentControl.Request.SHUTDOWN:
            self.get_logger().info(f"[component_supervisor] Shutdown requested: '{name}'")
            results = self._shutdown_component(name)
            response.success = True
            response.message = (
                f"'{name}' shut down -- " + ('; '.join(results) if results
                                              else 'was not running') +
                ' (will not auto-respawn until START/RESTART).')

        elif request.action == ComponentControl.Request.START:
            self.get_logger().info(f"[component_supervisor] Start requested: '{name}'")
            procs = self._processes.get(name, [])
            already_running = any(
                p.popen is not None and p.popen.poll() is None for p in procs)
            if already_running:
                for p in procs:
                    p.manually_stopped = False
                response.message = f"'{name}' already running -- cleared manually_stopped."
            else:
                self._start_component(name)
                response.message = f"'{name}' started."
            response.success = True

        elif request.action == ComponentControl.Request.RESTART:
            self.get_logger().info(f"[component_supervisor] Restart requested: '{name}'")
            stop_results = self._stop_component(name)
            self._start_component(name)
            response.success = True
            response.message = (
                f"'{name}' restarted -- " +
                ('; '.join(stop_results) if stop_results else 'was not running'))

        else:
            response.success = False
            response.message = (
                f'Unknown action {request.action}. Valid: SHUTDOWN=0, START=1, '
                'RESTART=2.')
            self.get_logger().warn(f'[component_supervisor] {response.message}')
            return response

        self.get_logger().info(f'[component_supervisor] {response.message}')
        return response

    def _on_run_calibration(self, request, response):
        """See module docstring's ~/run_calibration paragraph for the full
        reasoning (why 'hardware' is stopped first, why this doesn't wait for
        calibration to actually finish, why it doesn't auto-restart 'hardware'
        afterward)."""
        if 'calibrate_hardware' not in self._registry:
            response.success = False
            response.message = (
                "'calibrate_hardware' is not registered in components.yaml -- "
                'cannot run calibration.')
            self.get_logger().warn(f'[component_supervisor] {response.message}')
            return response

        already_running = any(
            proc.popen is not None and proc.popen.poll() is None
            for proc in self._processes.get('calibrate_hardware', []))
        if already_running:
            response.success = False
            response.message = (
                "'calibrate_hardware' is already running -- not starting a second "
                'one on top of it. Watch /calibration/in_progress for it to finish.')
            self.get_logger().warn(f'[component_supervisor] {response.message}')
            return response

        self.get_logger().info(
            "[component_supervisor] Run-calibration requested -- stopping 'hardware' "
            "first (shares the VESC serial port with 'calibrate_hardware', can't "
            "safely run both at once), then starting 'calibrate_hardware'.")
        stop_results = self._shutdown_component('hardware')
        time.sleep(_HARDWARE_TEARDOWN_BUFFER_SEC)
        self._start_component('calibrate_hardware')

        response.success = True
        response.message = (
            "'hardware' stopped, 'calibrate_hardware' started -- this does NOT wait "
            'for calibration to finish (typically 60-130s): watch '
            '/calibration/in_progress (published by '
            "f1tenth_diagnostics' diagnostics_server_node) go back to false, then "
            "START or RESTART 'hardware' yourself to resume driving with the fresh "
            'values (same manual follow-up calibrate_hardware already documented for '
            'localization/navigation -- this does not auto-sequence that either). '
            'Stop results: ' + ('; '.join(stop_results) if stop_results
                                 else "'hardware' was not running"))
        self.get_logger().info(f'[component_supervisor] {response.message}')
        return response

    # -- shutdown -----------------------------------------------------------------

    def shutdown_all(self):
        """Stop every tracked component (in parallel -- see _stop_all_components)
        and clear the pgid file. Guarded by _shutdown_started, not _shutdown_done,
        so a second call while this one is still running (main()'s _handle_signal
        reachable again mid-shutdown, e.g. a signal interrupting the wait loop
        below) is a no-op here -- that case is main()'s to handle via
        force_kill_all() instead, not by re-entering this method. Uses
        _SHUTDOWN_GRACE_SEC rather than each component's normal restart_timeout_sec
        so this can't compound into minutes if a few components are slow to exit.
        """
        if self._shutdown_started:
            return
        self._shutdown_started = True
        for procs in self._processes.values():
            for proc in procs:
                proc.manually_stopped = True
        self.get_logger().info(
            '[component_supervisor] Shutting down -- stopping all components.')
        for (name, launch_file), result in self._stop_all_components(
                self._SHUTDOWN_GRACE_SEC).items():
            self.get_logger().info(f"[component_supervisor] '{name}' ({launch_file}): {result}")
        self._shutdown_done = True
        self._clear_pgid_file()
        # Every tracked process group is confirmed gone at this point (
        # _stop_all_components() only returns once each has exited or been
        # SIGKILLed-and-waited-on) -- safe to check for FastDDS SHM residue this
        # shutdown itself may have left behind. See module docstring.
        self._sweep_stale_fastrtps_shm('shutdown')


def main(args=None):
    # Claimed BEFORE rclpy.init()/ComponentSupervisorNode(), because that
    # constructor runs _sweep_stale_pgids() -- which, with a live peer
    # supervisor around, is itself the destructive step: it reads the peer's
    # LIVE pids out of the shared _pgid_file and kills them as "stale" (see the
    # module docstring's own "Single-instance lock" paragraph). Refusing here,
    # before any of that, is the only point at which a second supervisor can
    # still be stopped without having already damaged the first.
    lock = _supervisor_lock_path()
    acquired, holder = acquire_singleton_lock(lock)
    if not acquired:
        held = f' (pid {holder})' if holder else ''
        print(
            f'component_supervisor_node: another supervisor is already running{held} '
            f'-- refusing to start a second one. Two supervisors spawn two full '
            f'stacks: components holding an exclusive resource (intelligence/'
            f'llama-server\'s TCP port, diagnostics/mission_logger_node\'s own lock) '
            f'crash-loop until their restart budget is gone, and the two instances '
            f'corrupt each other\'s shared process tracking. Stop the running stack '
            f'first, or if it is already gone, remove the lock: {lock}',
            file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = ComponentSupervisorNode()

    def _handle_signal(signum, frame):
        # Default Python disposition for SIGTERM is an immediate process exit with
        # no exception raised -- try/finally below would never run, leaving every
        # tracked component orphaned. Registering this explicitly (for SIGINT too,
        # so both paths behave identically regardless of rclpy's own internal signal
        # handling) makes shutdown run deterministically either way, then exits
        # directly rather than returning control back into rclpy.spin().
        #
        # A second signal arriving while shutdown_all() from the first is still
        # in progress (_shutdown_started True, _shutdown_done still False) is a
        # real, observed case -- not hypothetical: `ros2 launch` gives a managed
        # Node ~5s after SIGINT before escalating to SIGTERM itself, and
        # shutdown_all() can take longer than that across many tracked
        # components. Signals interrupt whatever this (single) thread is
        # blocked on, so that second signal reaches this same handler again,
        # reentrantly, while the first call is still paused mid-loop -- calling
        # shutdown_all() again here would just hit its own now-True
        # _shutdown_started guard and return instantly, then os._exit(0) below
        # would abandon everything the first call hadn't gotten to yet (this is
        # exactly how the orphans this whole mechanism exists to prevent still
        # got left behind). Escalate instead: stop being graceful and
        # force-kill everything immediately.
        if node._shutdown_started and not node._shutdown_done:
            node.get_logger().warn(
                f'[component_supervisor] Received signal {signum} while a '
                'shutdown was already in progress -- force-killing every '
                'tracked component immediately.')
            node.force_kill_all()
        else:
            node.get_logger().info(
                f'[component_supervisor] Received signal {signum} -- shutting '
                'down all tracked components.')
            node.shutdown_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # os._exit() skips main()'s own finally, so the lock has to be dropped
        # here explicitly -- leaving it behind would make the next launch refuse
        # to start over a PID that is about to stop existing.
        release_singleton_lock(lock)
        os._exit(0)

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Defense in depth for any exit path that isn't one of the two signals above
        # (e.g. rclpy.spin() returning on its own) -- a no-op if _handle_signal
        # already ran, since shutdown_all() is idempotent.
        node.shutdown_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        release_singleton_lock(lock)
    return 0


if __name__ == '__main__':
    sys.exit(main())
