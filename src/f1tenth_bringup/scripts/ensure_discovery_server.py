#!/usr/bin/env python3
"""Idempotent launcher for the Fast-DDS Discovery Server (DDS-churn
investigation follow-up -- see stack_params.yaml's own discovery_server_
address/_port comment for the original root-cause writeup).

Root problem this fixes: the discovery server is deliberately built to
survive component_supervisor_node's own restarts and outlive individual
launch sessions (see supervisor_bringup.launch.py/stack_bringup.launch.py's
own module docstrings) -- but the ExecuteProcess action that starts it never
checked whether one was ALREADY alive first. Confirmed live: relaunching the
stack while an earlier session's server (deliberately still running, by
design) still holds the port produces "Discovery Server wasn't able to
allocate the specified listening port", and the launch action's own
respawn=True then retries against the SAME still-occupied port forever,
tangled with a second, genuinely separate "fast-discovery-server tool not
found!" message -- confirmed by reading fastdds' own discovery/parser.py
(eProsima's wrapper): that second message is a Python-level exception-
handler string (subprocess.run() itself raising, e.g. because the target
couldn't be exec'd at all), architecturally distinct from a normal nonzero
exit from a real "port already in use" failure (which prints nothing extra
of its own, just sys.exit(returncode)) -- NOT the same problem reworded,
and not caused by a PATH/environment gap (checked: a genuinely fresh
interactive shell resolves fastdds/fast-discovery-server identically to an
already-working session; both already correctly land on /opt/ros/humble/bin
via .bashrc). Most likely a transient side effect of the respawn loop
itself hammering the same occupied port, not something with its own fix.

This script is the actual fix: check once, up front, whether a UDP listener
already answers at address:port. If yes, log that and idle harmlessly
forever (respawn=True never even triggers, since this process never exits
with an error) -- reusing the existing server instead of fighting it for
the port. If no, exec the real fastdds discovery server directly (replacing
this process, so respawn=True still works exactly as before if THIS
instance ever dies).

Usage: ensure_discovery_server.py <address> <port>
"""
import socket
import subprocess
import sys


def port_is_taken(address: str, port: int) -> bool:
    """The only reliable way to ask "is anything listening on this UDP
    port" -- UDP has no connection state to query, so the real test is
    trying to bind it ourselves. Bind succeeds -> nothing was there (and
    this probe socket is closed immediately, freeing it again for the real
    server to bind for real). Bind fails with OSError (EADDRINUSE) ->
    something already holds it.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind((address, port))
        return False
    except OSError:
        return True
    finally:
        probe.close()


def main():
    if len(sys.argv) != 3:
        print(f'usage: {sys.argv[0]} <address> <port>', file=sys.stderr)
        sys.exit(2)
    address = sys.argv[1]
    port = int(sys.argv[2])

    if port_is_taken(address, port):
        # flush=True on both prints below -- REAL BUG FOUND LIVE: stdout is
        # fully buffered (not line-buffered) whenever it isn't a real TTY,
        # which is always true here (ros2 launch captures it, then redirects
        # again to a log file) -- confirmed live, the "reusing it" message
        # never appeared in the log at all for the idle branch (which never
        # exits, so the buffer never got a chance to flush on its own;
        # process-exit-triggered flushing is what makes the OTHER branch's
        # print look fine without this, since exec() on a clean line only
        # happens after it, not because buffering wasn't an issue there too).
        print(f'[ensure_discovery_server] {address}:{port} already has a live '
              'listener (an earlier session\'s server, deliberately still running '
              'by design -- see stack_params.yaml\'s own discovery_server_address/'
              '_port comment) -- reusing it, not starting a duplicate. This '
              'process will now idle harmlessly for the life of this launch.',
              flush=True)
        # Block forever without erroring -- respawn=True on the launch action
        # wrapping this script must never see a nonzero exit here, or it will
        # respawn-loop against the same already-occupied port exactly like
        # the bug this script fixes.
        signal_wait = subprocess.Popen(['tail', '-f', '/dev/null'])
        signal_wait.wait()
        return

    print(f'[ensure_discovery_server] {address}:{port} is free -- starting a '
          'fresh fastdds discovery server.', flush=True)
    # exec, not subprocess.run -- replaces THIS process's image entirely, so
    # the launch action's own PID becomes the real server (matching what
    # directly running `fastdds discovery ...` as the cmd would have done),
    # and respawn=True still means what it always meant: restart if the
    # real server process itself dies.
    #
    # '/bin/sh', <path>, <args...> -- REAL BUG FOUND LIVE (first version of
    # this call was os.execvp('fastdds', ['fastdds', 'discovery', ...])):
    # failed every time with the exact same OSError: [Errno 8] Exec format
    # error already root-caused once in this investigation for the launch-
    # file cmd= case (/opt/ros/humble/bin/fastdds is a shebang-less shell
    # script; a direct exec -- os.execvp here, subprocess.Popen(shell=False)
    # there -- has no shebang to dispatch on, unlike bash's own interactive
    # ENOEXEC->/bin/sh fallback). Same fix, same reasoning: route through an
    # explicit shell. Passed as '/bin/sh', '<path-to-script>', '<args...>'
    # here (not '-c', '<one string>') since execvp's own argv list already
    # keeps each argument separate -- no string-quoting/escaping needed at
    # all, simpler than the launch-file cmd='s '-c' form had to be.
    import os
    import shutil
    fastdds_path = shutil.which('fastdds')
    if fastdds_path is None:
        print('[ensure_discovery_server] fastdds not found on PATH -- cannot '
              'start the discovery server.', file=sys.stderr)
        sys.exit(1)
    os.execvp('/bin/sh', ['sh', fastdds_path, 'discovery', '--server-id', '0',
                           '--udp-address', address, '--udp-port', str(port)])


if __name__ == '__main__':
    main()
