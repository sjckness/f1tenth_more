"""
/proc scanning: ROS processes of this user, and who holds a UDP port.

The matching rules are scripts/kill_ros2.py's, reimplemented on /proc
because that script needs psutil. Two differences: the caller and its
ancestors are never matched (the ssh shell, the bot in local mode), and the
discovery server is told apart so killall can leave it up.
"""

from dataclasses import dataclass
import os
import re
import signal
import time
from typing import Callable, Dict, Iterable, List, Optional, Set

# Process names (/proc/<pid>/comm, 15 chars max) that are ROS whatever their
# arguments are.
_ROS_NAMES = {
    'rviz2', 'gzserver', 'gzclient', 'gazebo', 'foxglove_bridge',
    'robot_state_pub', 'component_conta', 'micro_ros_agent', 'rosbridge_webso',
    '_ros2_daemon',
}
_DISCOVERY_RE = re.compile(
    r'(fast-discovery-server|fastdds(\.py)?\s+discovery|ensure_discovery_server\.py)')


@dataclass
class Proc:
    pid: int
    ppid: int
    uid: int
    comm: str
    argv: List[str]

    @property
    def cmdline(self) -> str:
        return ' '.join(self.argv)


def read_proc(pid: int) -> Optional[Proc]:
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        with open(f'/proc/{pid}/stat') as f:
            stat = f.read()
        with open(f'/proc/{pid}/status') as f:
            uid = next(int(line.split()[1]) for line in f if line.startswith('Uid:'))
    except (OSError, StopIteration, ValueError):
        return None
    # comm may contain spaces and parentheses: split on the last ')'.
    comm = stat[stat.find('(') + 1:stat.rfind(')')]
    ppid = int(stat[stat.rfind(')') + 2:].split()[1])
    return Proc(pid, ppid, uid, comm, argv)


def all_procs() -> List[Proc]:
    out = []
    for name in os.listdir('/proc'):
        if name.isdigit():
            p = read_proc(int(name))
            if p is not None and p.argv:
                out.append(p)
    return out


def ancestors(pid: int, table: Dict[int, Proc]) -> Set[int]:
    seen = set()
    while pid and pid not in seen:
        seen.add(pid)
        p = table.get(pid)
        if p is None:
            p = read_proc(pid)
        if p is None:
            break
        pid = p.ppid
    return seen


def is_discovery(p: Proc) -> bool:
    return bool(_DISCOVERY_RE.search(p.cmdline))


def is_ros(p: Proc) -> bool:
    """kill_ros2.py's rules: names, /opt/ros/ paths, /install/ paths, ros2."""
    if p.comm in _ROS_NAMES:
        return True
    if is_discovery(p):
        return True
    for arg in p.argv:
        if arg.startswith('/opt/ros/') or '/install/' in arg:
            return True
    if '--ros-args' in p.argv:
        return True
    for arg in p.argv[:2]:
        if os.path.basename(arg) == 'ros2':
            return True
    return 'ros2cli.daemon' in p.cmdline or 'ros2-daemon' in p.cmdline


def ros_processes(procs: Iterable[Proc], uid: int, exclude: Set[int],
                  include_discovery: bool) -> List[Proc]:
    out = []
    for p in procs:
        if p.uid != uid or p.pid in exclude:
            continue
        if not is_ros(p):
            continue
        if is_discovery(p) and not include_discovery:
            continue
        out.append(p)
    return out


def discovery_children(procs: Iterable[Proc], discovery_pids: Set[int]) -> Set[int]:
    """Return the `tail -f /dev/null` ensure_discovery_server.py idles on."""
    return {p.pid for p in procs if p.ppid in discovery_pids}


def matching(procs: Iterable[Proc], pattern: str, uid: int,
             exclude: Set[int]) -> List[Proc]:
    rx = re.compile(pattern)
    return [p for p in procs if p.uid == uid and p.pid not in exclude
            and rx.search(p.cmdline)]


def own_exclusions() -> Set[int]:
    """Return this process, its ancestors and the user's systemd manager."""
    table = {}
    excl = ancestors(os.getpid(), table)
    for p in all_procs():
        if p.comm == 'systemd' and p.uid == os.getuid():
            excl.add(p.pid)
    return excl


def alive(pid: int) -> bool:
    try:
        with open(f'/proc/{pid}/stat') as f:
            state = f.read().rsplit(')', 1)[1].split()[0]
        return state != 'Z'
    except (OSError, IndexError):
        return False


def signal_ladder(pids: List[int], sigint_wait: float, sigterm_wait: float,
                  sigkill_wait: float = 2.0,
                  kill: Callable[[int, int], None] = os.kill,
                  is_alive: Callable[[int], bool] = alive) -> dict:
    """SIGINT every pid, wait; SIGTERM the survivors, wait; then SIGKILL."""
    remaining = list(pids)
    sent = []
    t0 = time.monotonic()
    for sig, wait in ((signal.SIGINT, sigint_wait), (signal.SIGTERM, sigterm_wait),
                      (signal.SIGKILL, sigkill_wait)):
        if not remaining:
            break
        sent.append(sig.name)
        for pid in remaining:
            try:
                kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        deadline = time.monotonic() + wait
        while True:
            remaining = [p for p in remaining if is_alive(p)]
            if not remaining or time.monotonic() >= deadline:
                break
            time.sleep(0.1)
    return {'signals': sent, 'survivors': remaining,
            'elapsed_s': round(time.monotonic() - t0, 2)}


# ---- UDP port owner (discovery server) ------------------------------------

def _hex_addr(address: str, port: int) -> str:
    parts = [int(x) for x in address.split('.')]
    return '%02X%02X%02X%02X:%04X' % (parts[3], parts[2], parts[1], parts[0], port)


def udp_inodes(address: str, port: int, table: str = '/proc/net/udp') -> List[int]:
    """Inodes of IPv4 UDP sockets bound to address:port (or 0.0.0.0:port)."""
    wanted = {_hex_addr(address, port), _hex_addr('0.0.0.0', port)}
    out = []
    try:
        with open(table) as f:
            next(f)
            for line in f:
                cols = line.split()
                if len(cols) > 9 and cols[1] in wanted:
                    out.append(int(cols[9]))
    except (OSError, StopIteration):
        pass
    return out


def pids_with_inodes(inodes: Iterable[int]) -> List[int]:
    targets = {f'socket:[{i}]' for i in inodes}
    if not targets:
        return []
    out = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        fd_dir = f'/proc/{name}/fd'
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.readlink(f'{fd_dir}/{fd}') in targets:
                        out.append(int(name))
                        break
                except OSError:
                    continue
        except OSError:
            continue
    return out
