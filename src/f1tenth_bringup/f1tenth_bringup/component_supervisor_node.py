"""component_supervisor_node: spawns and independently restarts named "components"
(functional groups of the F1TENTH stack -- hardware, localization, perception,
control, navigation, behavior, diagnostics, intelligence, dev_tools, startup_sequence,
plus the on-demand-only calibrate_hardware) as separate `ros2 launch` subprocesses,
each in
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

This is the parallel bringup path alongside stack_bringup.launch.py (which stays
untouched as a working fallback -- single process, no per-component restart). See
f1tenth_bringup/config/components.yaml for the structural registry (WHAT each
component runs) -- WHICH components auto-start, and how navigation/behavior/
intelligence are modified or skipped, is decided here, once, at startup, by reading
the same 5 stack-wide branching values (f1tenth_params/config/stack_params.yaml)
that stack_bringup.launch.py itself uses:

  behavior      auto-starts only if use_behavior_tree
  intelligence  auto-starts only if enable_llm
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
  everything else (hardware, localization, perception, diagnostics, startup_sequence)
                always auto-starts
  calibrate_hardware
                NEVER auto-starts -- registered (a valid, restartable component_name)
                but only ever brought up on an explicit RestartComponent request.

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
import time

import yaml

from f1tenth_messages.srv import ComponentControl, RestartComponent
from f1tenth_params.param_defaults import get_value

import rclpy
from rclpy.node import Node

# Components that auto-start unconditionally.
_ALWAYS_AUTO_START = {'hardware', 'localization', 'navigation', 'perception',
                       'control', 'diagnostics', 'dev_tools', 'startup_sequence'}
# Registered (restartable by name) but never auto-started -- on-demand only.
_NEVER_AUTO_START = {'calibrate_hardware'}
# Gated on one of the 6 stack-wide branching values (see module docstring).
_CONDITIONAL_AUTO_START = {
    'behavior': 'use_behavior_tree',
    'intelligence': 'enable_llm',
}

# FastDDS SHM transport hygiene (see module docstring's own paragraph on this) --
# one glob catches both the fastrtps_<hex>/fastrtps_<hex>_el segment family and the
# fastrtps_port<N>/fastrtps_port<N>_el/sem.fastrtps_port<N>_mutex well-known-port
# family, since all of them contain 'fastrtps' as a substring.
_FASTRTPS_SHM_DIR = '/dev/shm'
_FASTRTPS_SHM_GLOB_PATTERN = '*fastrtps*'


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
            if get_value(flag):
                auto_start.add(name)

        self.get_logger().info(
            f'[component_supervisor] Registry: {sorted(self._registry)}. '
            f'Auto-starting: {sorted(auto_start)}. Log dir: {self.log_dir}'
        )
        for name in auto_start:
            self._start_component(name)

        self._restart_srv = self.create_service(
            RestartComponent, 'restart_component', self._on_restart_component)
        self._control_srv = self.create_service(
            ComponentControl, '~/control_component', self._on_control_component)

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


if __name__ == '__main__':
    main()
