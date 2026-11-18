"""
One managed process without systemd: the proc backend's runner (procunits.py).

    python3 -m f1tenth_ctl.runner --unit UNIT --dir DIR --log LOG --workdir WD
                                  [--kill-timeout S] -- COMMAND...

It does for one process what `systemd-run --user` does for the systemd
backend, in a container, which has no user manager:

- Detaches: double fork + setsid. The runner belongs neither to its caller
  (an ssh or `docker exec` session, the gateway) nor to the caller's session;
  it is re-parented to the container's init (tini). The process started as
  above (the launcher) returns as soon as COMMAND runs: exit 0 and
  `ok <pid>` on stdout, or exit 1 and why on stderr.
- Child subreaper (PR_SET_CHILD_SUBREAPER): what COMMAND leaves behind (a
  daemon, an orphaned grandchild) is re-parented to the runner instead of
  init, so the runner's descendants are the unit's "control group".
- COMMAND runs in its own session and process group (SIGINT to that group is
  Ctrl-C, as systemd does for a service), in WD, stdin /dev/null, stdout and
  stderr appended to LOG. Signal dispositions are the defaults again.
- DIR/proc.json, once COMMAND runs: the runner's and COMMAND's pid with their
  /proc start times and the boot id, so a reused pid is never taken for them.
- KillMode=control-group: when COMMAND ends, whatever is left below the
  runner gets SIGTERM, and SIGKILL after S seconds. SIGTERM to the runner
  (procunits' `systemctl stop`) does the same to the whole tree, COMMAND
  included.
- DIR/exit.json, then the runner exits: how COMMAND ended, in systemd's terms
  (ExecMainCode, ExecMainStatus, Result), for units.classify().

Stdlib only, and nothing else of f1tenth_ctl: the gateway's venv runs it too.
"""

import argparse
import ctypes
from datetime import datetime
import json
import os
import select
import signal
import subprocess
import sys
import time
from typing import List, Optional

PR_SET_CHILD_SUBREAPER = 36
# systemd's default SuccessExitStatus: a main process ended by one of these
# signals ended cleanly (Result=success).
CLEAN_SIGNALS = (signal.SIGHUP, signal.SIGINT, signal.SIGTERM, signal.SIGPIPE)
# How the main process ended, as systemd's ExecMainCode (the si_code).
CLD_EXITED, CLD_KILLED, CLD_DUMPED = 1, 2, 3
# systemd's exit status for a command that could not be executed.
EXIT_EXEC = 203
# While stopping: how often the tree is looked at (processes that are not
# the runner's children end without telling it). Otherwise the runner sleeps
# until a signal (SIGCHLD, SIGTERM) wakes it: no polling, no CPU.
POLL_S = 0.1
_DEAD = ('Z', 'X', 'x')


# ---- /proc ----------------------------------------------------------------

def proc_stat(pid: int) -> Optional[dict]:
    """Return state, ppid, pgid and start time (clock ticks since boot) of a pid."""
    try:
        with open(f'/proc/{int(pid)}/stat') as f:
            data = f.read()
    except (OSError, ValueError):
        return None
    # comm may contain spaces and parentheses: split after the last ')'.
    fields = data[data.rfind(')') + 2:].split()
    try:
        return {'state': fields[0], 'ppid': int(fields[1]), 'pgid': int(fields[2]),
                'start': int(fields[19])}
    except (IndexError, ValueError):
        return None


def boot_id() -> str:
    try:
        with open('/proc/sys/kernel/random/boot_id') as f:
            return f.read().strip()
    except OSError:
        return ''


def alive(pid, start) -> bool:
    """Say whether `pid` runs (not a zombie) and is the process that started at `start`."""
    if not pid or start is None:
        return False
    st = proc_stat(pid)
    return st is not None and st['start'] == start and st['state'] not in _DEAD


def descendants(root: int) -> List[int]:
    """Every live process below `root` (zombies cannot be signalled: left out)."""
    children, states = {}, {}
    try:
        names = os.listdir('/proc')
    except OSError:
        return []
    for name in names:
        if not name.isdigit():
            continue
        st = proc_stat(int(name))
        if st is None:
            continue
        children.setdefault(st['ppid'], []).append(int(name))
        states[int(name)] = st['state']
    out, seen, todo = [], set(), list(children.get(root, []))
    while todo:
        pid = todo.pop()
        if pid in seen:
            continue
        seen.add(pid)
        if states.get(pid) not in _DEAD:
            out.append(pid)
        todo.extend(children.get(pid, []))
    return sorted(out)


def write_json(path: str, data: dict) -> None:
    """Write `data` to `path` atomically (a reader never sees half a file)."""
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f)
    os.replace(tmp, path)


def exit_record(status: int, stop_requested: bool = False, escalated: bool = False) -> dict:
    """Describe a wait status as systemd would (pure)."""
    if os.WIFEXITED(status):
        code, value = CLD_EXITED, os.WEXITSTATUS(status)
        result = 'success' if value == 0 else 'exit-code'
    else:
        value = os.WTERMSIG(status)
        if os.WCOREDUMP(status):
            code, result = CLD_DUMPED, 'core-dump'
        else:
            code = CLD_KILLED
            result = 'success' if value in CLEAN_SIGNALS else 'signal'
    if stop_requested and escalated and result != 'success':
        # Stopped, but only SIGKILL after the timeout ended it.
        result = 'timeout'
    return {'code': code, 'status': value, 'result': result}


def describe(rec: dict) -> str:
    if rec.get('code') == CLD_EXITED:
        how = f"exit {rec.get('status')}"
    else:
        try:
            how = signal.Signals(int(rec.get('status'))).name
        except (TypeError, ValueError):
            how = f"signal {rec.get('status')}"
    return f"{how} ({rec.get('result')})"


def set_subreaper() -> bool:
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        return libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) == 0
    except (OSError, AttributeError):
        return False


def _now() -> str:
    return datetime.now().astimezone().isoformat(timespec='seconds')


# ---- the runner -------------------------------------------------------------

class Runner:
    """The detached runner process: COMMAND, its tree, its records."""

    def __init__(self, args):
        self.args = args
        self.child = None
        self.stop_at = None      # monotonic time of the stop request, or of COMMAND's end
        self.stop_requested = False
        self.escalated = False   # SIGKILL went to the tree
        self.wake_r = None       # the self-pipe signals are written to

    def arm_wakeup(self) -> None:
        """Make every caught signal (SIGCHLD above all) wake sleep()."""
        self.wake_r, wake_w = os.pipe()
        os.set_blocking(self.wake_r, False)
        os.set_blocking(wake_w, False)
        signal.set_wakeup_fd(wake_w)
        signal.signal(signal.SIGCHLD, lambda *_: None)

    def sleep(self, timeout) -> None:
        """Wait for a signal, at most `timeout` s (None: until one comes)."""
        if self.wake_r is None:
            time.sleep(POLL_S if timeout is None else min(timeout, POLL_S))
            return
        try:
            select.select([self.wake_r], [], [], timeout)
            while os.read(self.wake_r, 512):
                pass
        except (BlockingIOError, InterruptedError):
            pass

    def log(self, text: str) -> None:
        try:
            os.write(2, f'[f1tenth_ctl] {_now()} {text}\n'.encode())
        except OSError:
            pass

    def signal_tree(self, sig) -> None:
        for pid in descendants(os.getpid()):
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass

    def on_sigterm(self, _signum, _frame) -> None:
        """`systemctl stop`: SIGTERM to the whole tree now, SIGKILL after the timeout."""
        self.stop_requested = True
        if self.stop_at is None:
            self.stop_at = time.monotonic()
            self.signal_tree(signal.SIGTERM)

    def reap(self) -> Optional[int]:
        """Reap every child that ended; return COMMAND's wait status once it did."""
        status = None
        while True:
            try:
                pid, st = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                if status is None and self.child is not None and self.child.returncode is None:
                    # No child left at all, COMMAND included: it cannot be
                    # waited for any more (never expected). Exit 255.
                    self.log(f'{self.args.unit}: lost track of pid {self.child.pid}')
                    return 255 << 8
                return status
            if pid == 0:
                return status
            if self.child is not None and pid == self.child.pid:
                status = st

    def kill_after_timeout(self) -> None:
        if (self.stop_at is not None and not self.escalated
                and time.monotonic() >= self.stop_at + self.args.kill_timeout):
            self.escalated = True
            self.signal_tree(signal.SIGKILL)

    def wait_command(self) -> int:
        while True:
            status = self.reap()
            if status is not None:
                return status
            self.kill_after_timeout()
            if self.stop_at is None:
                self.sleep(None)            # until SIGCHLD or SIGTERM
            else:
                self.sleep(POLL_S)

    def drain(self) -> None:
        """KillMode=control-group: stop what COMMAND left behind."""
        self.reap()
        if not descendants(os.getpid()):
            return
        if self.stop_at is None:
            self.stop_at = time.monotonic()
            self.signal_tree(signal.SIGTERM)
        while descendants(os.getpid()):
            self.reap()
            if self.escalated and time.monotonic() >= self.stop_at + self.args.kill_timeout + 5.0:
                self.log(f'{self.args.unit}: processes survive SIGKILL: '
                         f'{descendants(os.getpid())}')
                break
            self.kill_after_timeout()
            self.sleep(POLL_S)
        self.reap()

    def run(self, notify) -> int:
        args = self.args
        signal.signal(signal.SIGTERM, self.on_sigterm)
        # Caught, not ignored: a caught signal is back to its default in
        # COMMAND after exec, an ignored one would stay ignored.
        for sig in (signal.SIGINT, signal.SIGHUP):
            signal.signal(sig, lambda *_: None)
        os.chdir('/')
        os.makedirs(args.dir, exist_ok=True)
        log_fd = os.open(args.log, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        null = os.open(os.devnull, os.O_RDWR)
        os.dup2(null, 0)
        os.dup2(log_fd, 1)
        os.dup2(log_fd, 2)
        for fd in (null, log_fd):
            if fd > 2:
                os.close(fd)
        subreaper = set_subreaper()
        try:
            os.makedirs(args.workdir, exist_ok=True)
        except OSError:
            pass                 # Popen reports it
        try:
            # restore_signals (default): SIGPIPE and SIGXFSZ back to default.
            self.child = subprocess.Popen(
                args.command, cwd=args.workdir, stdin=subprocess.DEVNULL, stdout=1, stderr=2,
                start_new_session=True, close_fds=True)
        except OSError as exc:
            rec = {'code': CLD_EXITED, 'status': EXIT_EXEC, 'result': 'exit-code',
                   'error': str(exc), 'time': time.time(), 'boot_id': boot_id(),
                   'unit': args.unit, 'pid': None}
            write_json(os.path.join(args.dir, 'exit.json'), rec)
            self.log(f'{args.unit}: cannot run {args.command[0]}: {exc}')
            notify(f'error cannot run {args.command[0]}: {exc}')
            return 1
        me, child = proc_stat(os.getpid()), proc_stat(self.child.pid)
        info = {'unit': args.unit, 'runner_pid': os.getpid(), 'runner_start': me and me['start'],
                'pid': self.child.pid, 'pid_start': child and child['start'],
                'pgid': self.child.pid, 'started_at': time.time(), 'boot_id': boot_id(),
                'subreaper': subreaper, 'kill_timeout_s': args.kill_timeout,
                'log': args.log, 'workdir': args.workdir}
        write_json(os.path.join(args.dir, 'proc.json'), info)
        notify(f'ok {self.child.pid}')
        if not subreaper:
            self.log(f'{args.unit}: PR_SET_CHILD_SUBREAPER failed: orphans escape the runner')
        self.arm_wakeup()
        status = self.wait_command()
        self.child.returncode = -1   # reaped here: Popen must not wait for it again
        self.drain()
        rec = exit_record(status, self.stop_requested, self.escalated)
        rec.update(time=time.time(), boot_id=boot_id(), unit=args.unit, pid=self.child.pid)
        write_json(os.path.join(args.dir, 'exit.json'), rec)
        self.log(f'end {args.unit}: {describe(rec)}')
        return 0


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog='f1tenth_ctl.runner', description=__doc__.split('\n\n')[0])
    ap.add_argument('--unit', required=True)
    ap.add_argument('--dir', required=True, help='state directory of this unit')
    ap.add_argument('--log', required=True)
    ap.add_argument('--workdir', required=True)
    ap.add_argument('--kill-timeout', type=float, default=5.0,
                    help='SIGTERM to SIGKILL, for what is left after COMMAND (s)')
    ap.add_argument('command', nargs=argparse.REMAINDER)
    args = ap.parse_args(argv)
    if args.command[:1] == ['--']:
        args.command = args.command[1:]
    if not args.command:
        ap.error('no COMMAND after --')
    # The runner itself moves to / (it must not hold a directory busy).
    for key in ('dir', 'log', 'workdir'):
        setattr(args, key, os.path.abspath(getattr(args, key)))
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    read_end, write_end = os.pipe()
    pid = os.fork()
    if pid:
        # The launcher: wait for the runner to say that COMMAND runs.
        os.close(write_end)
        chunks = []
        while True:
            chunk = os.read(read_end, 4096)
            if not chunk:
                break
            chunks.append(chunk)
        os.close(read_end)
        os.waitpid(pid, 0)
        text = b''.join(chunks).decode(errors='replace').strip()
        if text.startswith('ok '):
            print(text)
            return 0
        print(text or 'the runner exited before starting the command', file=sys.stderr)
        return 1
    code = 1
    sent = []

    def notify(text):
        # Once only: the launcher returns at EOF, and the fd may be reused after.
        if sent:
            return
        sent.append(text)
        try:
            os.write(write_end, text.encode())
        except OSError:
            pass
        os.close(write_end)
    try:
        os.close(read_end)
        os.setsid()
        if os.fork():
            os._exit(0)
        code = Runner(args).run(notify)
    except BaseException as exc:  # noqa: B036 - nothing may return into the caller's code
        notify(f'error {type(exc).__name__}: {exc}')
    os._exit(code)


if __name__ == '__main__':
    sys.exit(main())
