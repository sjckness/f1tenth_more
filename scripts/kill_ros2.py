#!/usr/bin/env python3
"""Kill all running ROS 2 processes (nodes, launch trees, daemon, common tools).

Usage:
  ./scripts/kill_ros2.py              # ask for confirmation, then kill
  ./scripts/kill_ros2.py -y           # kill without confirmation
  ./scripts/kill_ros2.py -n           # dry run: only list what would be killed
  ./scripts/kill_ros2.py -t 10        # allow 10s grace period per signal (default 5)

Detects processes via:
  - executable under /opt/ros/<distro>/ or a colcon install/ prefix
  - `ros2 ...` CLI invocations (launch, run, daemon, bag, ...)
  - common ROS-adjacent binaries (rviz2, gzserver/gzclient, foxglove_bridge,
    component_container[_mt|_isolated], robot_state_publisher, ...)

Deliberately does NOT match on the ROS_DISTRO/ROS_VERSION environment
variables: on a machine where ROS is sourced in the shell profile, every
process in the session inherits them (editor, language servers, this very
script's shell), so that signal is far too broad and would kill unrelated
processes.

Escalates SIGINT -> SIGTERM -> SIGKILL, waiting up to --timeout seconds
between each step for processes to exit on their own.

Once every process is confirmed gone it also purges the runtime state that
outlives them and silently breaks the NEXT launch -- FastDDS /dev/shm
segments and the singleton/pgid files (see purge_runtime_state). Killing
processes alone is not a clean slate; leftover SHM in particular presents
as "foxglove shows nothing", not as a stale-file problem. Skipped if
anything survived SIGKILL, since deleting segments a live participant still
holds is itself a failure mode.
"""
import argparse
import glob
import os
import signal
import subprocess
import sys
import time

try:
    import psutil
except ImportError:
    sys.exit("error: this script requires 'psutil' (pip install psutil)")

ROS_BIN_NAMES = {
    "rviz2",
    "gzserver",
    "gzclient",
    "gazebo",
    "foxglove_bridge",
    "robot_state_publisher",
    "component_container",
    "component_container_mt",
    "component_container_isolated",
    "micro_ros_agent",
    "rosbridge_websocket",
    "_ros2_daemon",
}

# Substrings matched against the full joined cmdline for processes that
# don't have a telltale binary path (e.g. `python3 -c "..."` launches).
ROS_CMDLINE_SUBSTRINGS = (
    "ros2cli.daemon",
    "ros2-daemon",
)


def is_ros2_process(proc: psutil.Process) -> bool:
    try:
        cmdline = proc.cmdline()
        exe = proc.exe()
        name = proc.name()
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        return False

    if name in ROS_BIN_NAMES:
        return True

    if exe and exe.startswith("/opt/ros/"):
        return True

    # `exe` is the interpreter (e.g. /usr/bin/python3) for scripts launched
    # as `python3 <path>`, so the ROS-ish path usually shows up in argv
    # instead of exe -- inspect both.
    paths = ([exe] if exe else []) + cmdline
    if any(p.startswith("/opt/ros/") for p in paths):
        return True
    if any("/install/" in p for p in paths):
        return True
    if any(os.path.basename(p) in ROS_BIN_NAMES for p in paths):
        return True

    if cmdline:
        first = cmdline[0]
        if first == "ros2" or first.endswith("/ros2"):
            return True
        # e.g. `python3 /opt/ros/humble/bin/ros2 launch ...`
        if len(cmdline) > 1 and cmdline[1].endswith("/ros2"):
            return True
        joined = " ".join(cmdline)
        if any(sub in joined for sub in ROS_CMDLINE_SUBSTRINGS):
            return True

    return False


def find_ros2_processes() -> list:
    self_pid = os.getpid()
    matches = []
    for proc in psutil.process_iter(["pid", "name"]):
        if proc.pid == self_pid:
            continue
        if is_ros2_process(proc):
            matches.append(proc)
    return matches


def describe(proc: psutil.Process) -> str:
    try:
        cmd = " ".join(proc.cmdline()) or proc.name()
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        cmd = "<gone>"
    return f"  pid={proc.pid:<7} {cmd}"


def wait_for_exit(procs: list, timeout: float) -> list:
    """Return the subset of procs still alive after `timeout` seconds."""
    deadline = time.monotonic() + timeout
    alive = list(procs)
    while alive and time.monotonic() < deadline:
        alive = [p for p in alive if p.is_running() and p.status() != psutil.STATUS_ZOMBIE]
        if alive:
            time.sleep(0.2)
    return [p for p in alive if p.is_running() and p.status() != psutil.STATUS_ZOMBIE]


def send_signal(procs: list, sig: signal.Signals) -> None:
    for proc in procs:
        try:
            proc.send_signal(sig)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass


def stop_ros2_daemon() -> None:
    """Best-effort clean shutdown of the `ros2 daemon` via its own CLI."""
    try:
        subprocess.run(
            ["ros2", "daemon", "stop"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass


# Runtime state that survives process death and breaks the NEXT launch. Killing
# processes alone is not a clean slate:
#   - /dev/shm/*fastrtps*: FastDDS SHM transport segments and its well-known
#     discovery-port files. These are ordinary files; nothing releases them when
#     a process is SIGKILLed, they accumulate across sessions (316 observed in
#     one dev session), and once enough of the small fixed discovery-port pool is
#     occupied, new participants lose the race for a port and hang instead of
#     failing loudly -- which reads as "foxglove shows nothing"/"ros2 topic list
#     is empty", not as a leftover-file problem.
#   - the singleton PID locks: component_supervisor_node and mission_logger_node
#     each refuse to start while their lock names a LIVE pid. Both reclaim a lock
#     whose pid is dead, so these are usually harmless -- removed anyway, since a
#     recycled pid can otherwise block a start for no real reason.
#   - tracked_pgids.json: component_supervisor_node's own crash-recovery record
#     of the process groups it spawned. Once every one of them is dead (which is
#     exactly what this script just guaranteed) it describes nothing, and leaving
#     it makes the next instance's startup sweep chase pids that are already gone.
_STATE_FILES = (
    "/tmp/component_supervisor.lock",
    "/tmp/mission_logger.lock",
    os.path.expanduser("~/.ros/log/component_supervisor/tracked_pgids.json"),
)


def purge_runtime_state(dry_run: bool) -> None:
    """Remove FastDDS SHM residue and stale singleton/state files.

    Only safe once every ROS process is confirmed gone -- deleting a segment a
    live participant still holds produces "Failed init_port fastrtps_portN:
    open_and_lock_file failed" in whatever is still running, so this is called
    at the very end of main(), never before the kill stages.
    """
    shm = sorted(glob.glob("/dev/shm/*fastrtps*"))
    if shm:
        total = sum(os.path.getsize(f) for f in shm if os.path.exists(f))
        verb = "Would remove" if dry_run else "Removing"
        print(f"\n{verb} {len(shm)} FastDDS /dev/shm file(s) ({total / 1e6:.1f} MB)")
        if not dry_run:
            for f in shm:
                try:
                    os.unlink(f)
                except OSError as exc:
                    print(f"  could not remove {f}: {exc}")
    else:
        print("\nNo FastDDS /dev/shm residue.")

    for f in _STATE_FILES:
        if not os.path.exists(f):
            continue
        if dry_run:
            print(f"Would remove {f}")
            continue
        try:
            os.unlink(f)
            print(f"Removed {f}")
        except OSError as exc:
            print(f"  could not remove {f}: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-n", "--dry-run", action="store_true", help="only list matching processes, don't kill them")
    parser.add_argument("-y", "--yes", action="store_true", help="skip the confirmation prompt")
    parser.add_argument("-t", "--timeout", type=float, default=5.0, help="grace period (s) between signal escalations, default 5")
    args = parser.parse_args()

    procs = find_ros2_processes()
    if not procs:
        print("No ROS 2 processes found.")
        return 0

    print(f"Found {len(procs)} ROS 2 process(es):")
    for proc in procs:
        print(describe(proc))

    if args.dry_run:
        print("\nDry run: nothing killed.")
        purge_runtime_state(dry_run=True)
        return 0

    if not args.yes:
        reply = input(f"\nKill these {len(procs)} process(es)? [y/N] ").strip().lower()
        if reply not in ("y", "yes"):
            print("Aborted.")
            return 1

    stop_ros2_daemon()

    procs = [p for p in procs if p.is_running() and p.status() != psutil.STATUS_ZOMBIE]
    if not procs:
        print("\nAll ROS 2 processes terminated.")
        return 0

    stages = [
        ("SIGINT", signal.SIGINT),
        ("SIGTERM", signal.SIGTERM),
        ("SIGKILL", signal.SIGKILL),
    ]

    alive = procs
    for stage_name, sig in stages:
        alive = [p for p in alive if p.is_running() and p.status() != psutil.STATUS_ZOMBIE]
        if not alive:
            break
        print(f"\nSending {stage_name} to {len(alive)} process(es)...")
        send_signal(alive, sig)
        alive = wait_for_exit(alive, args.timeout)

    if alive:
        print(f"\n{len(alive)} process(es) survived SIGKILL (permission issue?):")
        for proc in alive:
            print(describe(proc))
        # Deliberately NOT purging here: something is still alive and may still
        # hold SHM segments, and deleting those out from under it is what
        # produces "open_and_lock_file failed" in the survivor.
        return 1

    print("\nAll ROS 2 processes terminated.")
    purge_runtime_state(dry_run=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
