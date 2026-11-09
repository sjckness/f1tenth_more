"""
The mission daemon: `mission-load` and `mission-start` through the READY monitor's node.

Each of those CLI calls used to start a ROS probe of its own: Python, rclpy and, above
all, 1.5-4 s of discovery behind the Discovery Server before it could call a service
(measured on the Orin: 2.7-4 s per call). The monitor (monitor.py, rosprobe.monitor) is
already a long-lived participant that has discovered everything and holds a
/mission/status subscription, so it also serves these two requests over a local unix
socket, and the CLI is a thin client of it (`request()`).

Why the monitor and not a new daemon: it already holds the subscriptions and the READY
checks (so `start` re-checks READY on live data, not on a file), it is already managed
(started by status/connect, stopped with the stack), and a second long-lived participant
would repeat the same discovery. What it must never become is a dependency of stopping
the car:

  * abort_mission and the e-stop are NOT served here and the CLI never sends them here.
    `estop`, `estop-reset` and `killall` keep their own one-shot probes (rosprobe.estop...),
    which talk to the loader directly. A dead, hung or busy daemon cannot delay them.
  * the monitor's loop is the daemon's only worker; the sockets are served by threads that
    never touch ROS.

Protocol (one request per connection, newline-delimited JSON):
    server -> {"hello": 1, "v": 1, "pid": N, "loop_age_s": x}      at once, from the thread
    client -> {"op": "load" | "start" | "ping", "spec": {...}}     only after a good hello
    server -> {"result": {...}}        the same dict rosprobe.mission() returns
            | {"fallback": "reason"}   NOTHING was executed: the client uses the old path

A client falls back (to the old one-shot probe, same behaviour as before) when anything
fails BEFORE it sent the request: no socket, a stale socket file, no hello in 0.2 s, a hello
whose monitor loop is older than 1 s (a hung loop), or a "fallback" reply (status
subscription lost or stale, spec that does not match the daemon). After the request was sent
there is no fallback, as there never was after a probe started: an unanswered request is an
error saying the start may have happened. A REFUSAL (not READY, nothing loaded, e-stop...) is
a normal result and is never retried through another path.

The socket is mode 0600 in the state directory, unix only: no network listener.
"""

import json
import os
import queue
import socket
import threading
import time
from typing import Callable, Optional, Tuple

PROTOCOL = 1
SOCK_NAME = 'missiond.sock'
CONNECT_S = 0.2           # connect and hello: past this the client takes the old path
LOOP_MAX_AGE_S = 1.0      # a monitor loop that has not turned for this long is hung
REQUEST_READ_S = 5.0      # a client that connected must send its request within this
QUEUE_WAIT_S = 40.0       # a request waits this long for the one in front of it
OPS = ('load', 'start', 'ping')


def sock_path(state_dir: str) -> str:
    return os.path.join(state_dir, SOCK_NAME)


def _recv_line(conn: socket.socket, timeout: float, limit: int = 1 << 20) -> Optional[bytes]:
    """Read one newline-terminated line, or None on EOF/timeout/too long."""
    conn.settimeout(timeout)
    buf = b''
    end = time.monotonic() + timeout
    try:
        while b'\n' not in buf and len(buf) < limit:
            left = end - time.monotonic()
            if left <= 0:
                return None
            conn.settimeout(left)
            chunk = conn.recv(65536)
            if not chunk:
                return None
            buf += chunk
    except (socket.timeout, OSError):
        return None
    return buf.split(b'\n', 1)[0] if b'\n' in buf else None


def _send(conn: socket.socket, obj: dict) -> bool:
    try:
        conn.settimeout(5.0)
        conn.sendall(json.dumps(obj, default=str).encode() + b'\n')
        return True
    except OSError:
        return False


# ---------------------------------------------------------------- server

class Job:
    """One accepted request, waiting for the monitor's loop."""

    def __init__(self, req: dict):
        self.req = req
        self.done = threading.Event()
        self.reply: Optional[dict] = None
        self.cancelled = False
        self.started = False
        self.t_queued = time.monotonic()


class Server:
    """
    Serve the unix socket.

    Connection threads speak the protocol; `poll()`, called from the owner's loop, runs at
    most one queued job through `execute(req) -> reply dict`.
    """

    def __init__(self, path: str, execute: Callable[[dict], dict],
                 log: Callable[[str], None] = lambda _m: None, clock=time.monotonic):
        self.path = path
        self.execute = execute
        self.log = log
        self.clock = clock
        self.jobs: 'queue.Queue[Job]' = queue.Queue()
        self.last_poll = clock()
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._closing = False

    # -- lifecycle
    def start(self) -> bool:
        """Bind the socket (0600). False when another live daemon owns it, or on error."""
        if os.path.exists(self.path):
            probe = socket.socket(socket.AF_UNIX)
            probe.settimeout(0.3)
            try:
                probe.connect(self.path)
                probe.close()
                self.log(f'missiond: {self.path} is served by another process: not starting')
                return False
            except OSError:
                probe.close()
                try:
                    os.unlink(self.path)         # nobody listens: a leftover file
                except OSError:
                    pass
        old = os.umask(0o177)                    # created 0600, never wider
        try:
            s = socket.socket(socket.AF_UNIX)
            s.bind(self.path)
        except OSError as exc:
            self.log(f'missiond: cannot bind {self.path}: {exc}')
            return False
        finally:
            os.umask(old)
        try:
            os.chmod(self.path, 0o600)
        except OSError:
            pass
        s.listen(16)
        s.settimeout(0.2)
        self._sock = s
        self._thread = threading.Thread(target=self._accept_loop, daemon=True,
                                        name='missiond-accept')
        self._thread.start()
        self.log(f'missiond: serving {self.path}')
        return True

    def close(self) -> None:
        self._closing = True
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        try:
            os.unlink(self.path)
        except OSError:
            pass
        while True:                              # queued clients get a fallback, not silence
            try:
                job = self.jobs.get_nowait()
            except queue.Empty:
                break
            job.reply = {'fallback': 'daemon stopping'}
            job.done.set()

    # -- the loop side
    def loop_age(self) -> float:
        return self.clock() - self.last_poll

    def beat(self) -> None:
        self.last_poll = self.clock()

    def poll(self) -> bool:
        """Run one queued job (if any); return whether one ran. Call from the owner's loop."""
        self.beat()
        try:
            job = self.jobs.get_nowait()
        except queue.Empty:
            return False
        if job.cancelled or job.done.is_set():
            return True
        job.started = True
        t0 = self.clock()
        try:
            job.reply = self.execute(job.req)
        except Exception as exc:  # noqa: BLE001 - one bad request must not stop the monitor
            self.log(f'missiond: {job.req.get("op")} raised {type(exc).__name__}: {exc}')
            job.reply = {'fallback': f'daemon error: {type(exc).__name__}'}
        self.log(f'missiond: {job.req.get("op")} {self._summary(job.reply)} '
                 f'queued {t0 - job.t_queued:.3f} s, ran {self.clock() - t0:.3f} s')
        job.done.set()
        self.beat()
        return True

    @staticmethod
    def _summary(reply: dict) -> str:
        if 'fallback' in reply:
            return f"-> fallback ({reply['fallback']})"
        res = reply.get('result') or {}
        return '-> ok' if res.get('ok') else f"-> refused/failed ({res.get('error')})"

    # -- the thread side
    def _accept_loop(self) -> None:
        while not self._closing:
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn: socket.socket) -> None:
        try:
            if not _send(conn, {'hello': 1, 'v': PROTOCOL, 'pid': os.getpid(),
                                'loop_age_s': round(self.loop_age(), 3)}):
                return
            line = _recv_line(conn, REQUEST_READ_S)
            if line is None:
                return                           # it never sent a request: nothing to run
            try:
                req = json.loads(line)
                if req.get('op') not in OPS or not isinstance(req.get('spec'), dict):
                    raise ValueError('bad request')
            except (ValueError, AttributeError):
                _send(conn, {'fallback': 'bad request'})
                return
            job = Job(req)
            self.jobs.put(job)
            deadline = self.clock() + QUEUE_WAIT_S
            while not job.done.wait(0.05):
                if self._peer_gone(conn) and not job.started:
                    job.cancelled = True         # the client left before its turn
                    return
                if self.clock() > deadline and not job.started:
                    job.cancelled = True
                    _send(conn, {'fallback': 'queue timeout'})
                    return
            _send(conn, job.reply or {'fallback': 'no reply'})
        finally:
            try:
                conn.close()
            except OSError:
                pass

    @staticmethod
    def _peer_gone(conn: socket.socket) -> bool:
        try:
            conn.setblocking(False)
            return conn.recv(1, socket.MSG_PEEK) == b''
        except BlockingIOError:
            return False
        except OSError:
            return True


# ---------------------------------------------------------------- client

def request(path: str, op: str, spec: dict, timeout_s: float,
            connect_s: float = CONNECT_S) -> Tuple[str, object]:
    """
    Ask the daemon. Return ('result', dict) | ('fallback', why) | ('error', message).

    'fallback' only when nothing was sent, or the daemon said it ran nothing; once the
    request is out an unanswered call is an 'error', never a second attempt.
    """
    if not os.path.exists(path):
        return 'fallback', 'no daemon socket'
    s = socket.socket(socket.AF_UNIX)
    try:
        s.settimeout(connect_s)
        s.connect(path)
    except OSError as exc:
        s.close()
        return 'fallback', f'cannot connect: {type(exc).__name__}'
    try:
        line = _recv_line(s, connect_s)
        try:
            hello = json.loads(line) if line else None
        except ValueError:
            hello = None
        if not isinstance(hello, dict) or hello.get('hello') != 1 or hello.get('v') != PROTOCOL:
            return 'fallback', 'no hello'
        if float(hello.get('loop_age_s', 99)) > LOOP_MAX_AGE_S:
            return 'fallback', f"daemon loop stuck ({hello.get('loop_age_s')} s)"
        if not _send(s, {'op': op, 'spec': spec}):
            return 'fallback', 'could not send'
        # From here on the request may run: no fallback.
        reply_line = _recv_line(s, timeout_s)
        if reply_line is None:
            return 'error', (f'mission daemon gave no answer in {timeout_s:g} s '
                             f'(the {op} may have happened: check mission-status)')
        try:
            reply = json.loads(reply_line)
        except ValueError:
            return 'error', 'mission daemon answered garbage'
        if 'fallback' in reply:
            return 'fallback', str(reply['fallback'])
        if isinstance(reply.get('result'), dict):
            return 'result', reply['result']
        return 'error', 'mission daemon answered without a result'
    finally:
        s.close()


# ---------------------------------------------------------------- the ROS side

class RosOps:
    """
    Execute a request on the monitor's node.

    Built by rosprobe.monitor with its node, executor, the /mission/status samples, the
    service clients, the READY meter and its checks.

    stale_reason() is what keeps a daemon from answering on a lost view of the mission: the
    status topic republishes on change only, so the age of the last message says nothing;
    instead the sample must have been received from the publishers that exist NOW.
    """

    def __init__(self, node, ex, statuses, load_cli, start_cli, load_type, meter, checks, spec,
                 tick: Callable[[float], None] = None, core=None, ready=None,
                 estop_latched=lambda _checks: None):
        self.node, self.ex, self.statuses = node, ex, statuses
        self.load_cli, self.start_cli, self.load_type = load_cli, start_cli, load_type
        self.meter, self.checks, self.spec = meter, checks, spec
        self.tick = tick
        self.core = core
        self.ready = ready
        self.estop_latched = estop_latched
        self.topic = spec.get('status_topic', '/mission/status')
        self.gids_at_sample = None

    def note_status(self) -> None:
        """Remember who published the sample just received (call from the status callback)."""
        self.gids_at_sample = self._gids()

    def _gids(self):
        try:
            return sorted(bytes(i.endpoint_gid).hex() for i in
                          self.node.get_publishers_info_by_topic(self.topic))
        except Exception:  # noqa: BLE001 - graph API trouble means "do not trust the view"
            return None

    def stale_reason(self) -> Optional[str]:
        """Return why the status view cannot be trusted now, or None."""
        if not self.statuses:
            return 'no /mission/status received yet'
        gids = self._gids()
        if not gids:
            return f'no publisher on {self.topic}'
        if gids != self.gids_at_sample:
            return f'{self.topic} publishers changed since the last sample'
        return None

    def spec_mismatch(self, spec: dict) -> Optional[str]:
        for key in ('status_topic', 'status_type', 'load_service', 'start_service',
                    'use_sim_time'):
            if key in spec and spec[key] != self.spec.get(key, spec[key]):
                return f'{key} differs from the monitor\'s'
        return None

    def ready_refusal(self) -> Optional[dict]:
        """Re-check READY on live measurements; return the refusal dict, or None when ready."""
        for _ in range(5):                       # let waiting callbacks run first
            self.ex.spin_once(timeout_sec=0)
        measured = self.meter.measure(max(float(self.spec.get('window_s', 1.0)), 2.0))
        verdict = self.ready({'checks': self.checks}, measured, True, True)
        if verdict['ready']:
            return None
        failing = verdict['failing']
        # The same words the CLI's own READY check uses for the same cause.
        if self.estop_latched(verdict['checks']):
            why = 'the e-stop is latched (estop-reset first)'
        else:
            why = 'not READY: ' + ', '.join(failing)
        return {'ok': False, 'error': why, 'ready_refused': True, 'failing': failing}

    def execute(self, req: dict) -> dict:
        op = req['op']
        spec = dict(req['spec'], op=op)
        if op == 'ping':
            return {'result': {'ok': True}}
        why = self.spec_mismatch(spec) or self.stale_reason()
        if why:
            return {'fallback': why}
        if op == 'start' and spec.get('require_ready'):
            refused = self.ready_refusal()
            if refused:
                return {'result': refused}
        result = self.core(spec, self.ex, self.statuses, self.load_cli, self.start_cli,
                           self.load_type, self.tick)
        return {'result': result}
