"""
How the bot reaches the car: f1tenth_ctl over ssh, or run locally.

    ssh -o BatchMode=yes -o ConnectTimeout=5 [-i key] <host> --
        <ctl_command> <args> [--env <ctl_env>] --json

Every call has a hard timeout. Whatever happens (timeout, ssh failure, a
crash on the far side) the caller gets one dict with at least `ok`, and
`transport_error` set when f1tenth_ctl's own answer never arrived.
"""

import json
import os
import shlex
import subprocess
import time
from typing import List, Optional

from .config import Target


def parse_ctl_output(stdout: str, stderr: str, returncode: int) -> dict:
    """Return f1tenth_ctl's JSON answer: the last line that is a JSON object."""
    for line in reversed((stdout or '').strip().splitlines()):
        line = line.strip()
        if not line.startswith('{'):
            continue
        try:
            obj = json.loads(line)
        except ValueError:
            continue
        if isinstance(obj, dict):
            obj.setdefault('ok', returncode == 0)
            obj['_rc'] = returncode
            return obj
    tail = ' | '.join((stderr or '').strip().splitlines()[-2:]) or 'no output'
    if returncode == 255:
        msg = tail if tail.startswith('ssh:') else f'ssh: {tail}'
        return {'ok': False, 'transport_error': 'ssh', 'error': msg, '_rc': 255}
    if returncode == 127:
        return {'ok': False, 'transport_error': 'not_found',
                'error': f'f1tenth_ctl not found: {tail}', '_rc': 127}
    return {'ok': False, 'transport_error': 'parse',
            'error': f'no JSON from f1tenth_ctl (exit {returncode}): {tail}',
            '_rc': returncode}


class Transport:
    def __init__(self, target: Target, runner=subprocess.run):
        self.target = target
        self.run_proc = runner

    # ---- command lines ------------------------------------------------------

    def ssh_base(self) -> List[str]:
        t = self.target
        cmd = ['ssh', '-o', 'BatchMode=yes', '-o', f'ConnectTimeout={int(t.connect_timeout_s)}',
               '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2']
        if t.ssh_key:
            cmd += ['-i', os.path.expanduser(t.ssh_key)]
        for opt in t.ssh_options:
            cmd += ['-o', opt]
        return cmd

    def ctl_argv(self, args: List[str]) -> List[str]:
        """Build the command that runs f1tenth_ctl, local or remote."""
        t = self.target
        args = list(args)
        if t.ctl_env:
            args += ['--env', t.ctl_env]
        args.append('--json')
        if t.local:
            return [os.path.expanduser(t.ctl_command), *args]
        # The remote shell expands a leading ~ of ctl_command; every argument
        # is quoted.
        remote = t.ctl_command + ' ' + ' '.join(shlex.quote(a) for a in args)
        return [*self.ssh_base(), t.host, '--', remote]

    # ---- calls ----------------------------------------------------------------

    def ctl(self, args: List[str], timeout: float) -> dict:
        argv = self.ctl_argv(args)
        t0 = time.monotonic()
        try:
            proc = self.run_proc(argv, capture_output=True, text=True, timeout=timeout,
                                 stdin=subprocess.DEVNULL)
        except subprocess.TimeoutExpired:
            return {'ok': False, 'transport_error': 'timeout',
                    'error': f'no answer within {timeout:g} s', '_elapsed_s': timeout}
        except OSError as exc:
            return {'ok': False, 'transport_error': 'exec', 'error': str(exc)}
        res = parse_ctl_output(proc.stdout, proc.stderr, proc.returncode)
        res['_elapsed_s'] = round(time.monotonic() - t0, 2)
        return res

    def reachability(self, timeout: float) -> dict:
        """Ping (if enabled) and make an ssh round trip: say whether the car is online."""
        t = self.target
        if t.local:
            return {'online': True, 'ping': None, 'ssh': None, 'local': True}
        out = {'online': False, 'ping': None, 'ssh': False}
        if t.ping:
            out['ping'] = self._ping(min(timeout, 3.0))
        try:
            proc = self.run_proc([*self.ssh_base(), t.host, 'true'], capture_output=True,
                                 text=True, timeout=timeout, stdin=subprocess.DEVNULL)
            out['ssh'] = proc.returncode == 0
            if proc.returncode != 0:
                out['error'] = (proc.stderr.strip().splitlines() or ['ssh failed'])[-1]
        except subprocess.TimeoutExpired:
            out['error'] = f'ssh: no answer within {timeout:g} s'
        except OSError as exc:
            out['error'] = str(exc)
        out['online'] = bool(out['ssh'])
        return out

    def _ping(self, timeout: float) -> Optional[bool]:
        try:
            proc = self.run_proc(['ping', '-c', '1', '-W', str(max(1, int(timeout))),
                                  self.target.host], capture_output=True, text=True,
                                 timeout=timeout + 1, stdin=subprocess.DEVNULL)
            return proc.returncode == 0
        except (subprocess.TimeoutExpired, OSError):
            return False
