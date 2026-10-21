"""
The heartbeat from the car to the base: packet, link state, UDP client.

PACKET. 24 bytes, network byte order. The car sends it; the base echoes it
byte for byte (scripts/base/heartbeat_responder.py), so the round trip is
measured on the car's own monotonic clock and needs no clock sync.

    magic    4s  b'F1HB'
    version  B   1
    path     B   index of the path it went out on (0 tailscale, 1 local)
    flags    H   0
    nonce    I   random per start: replies meant for an earlier run are dropped
    seq      I   round number
    t_ns     Q   car's monotonic clock at send [ns], for packet captures only

ROUNDS. Every period_s one round goes out: the same seq to every path. The
fallback is therefore always warm: when Tailscale stops answering, the local
path already has a fresh RTT and takes over at once. A round is answered
when any path replies within timeout_s, lost when none does.

PATHS, in priority order (tailscale first, then the hotspot's local
address). A path is UP while its last reply is younger than timeout_s, and
FRESH while it is younger than degraded_silence_s. The active path is the
first UP one, with two refinements:
- fallback: when the active path stops being FRESH and another one is, the
  first FRESH one takes over (after degraded_silence_s, not timeout_s);
- failback: a higher-priority path takes over from the active one only
  after failback_s FRESH without a gap, so a flapping Tailscale does not
  flip the path every period.

STATE (LinkMonitor.evaluate):
    LOST       no path UP. During the first timeout_s after start, before any
               reply, the state is DEGRADED instead (not proven lost yet).
    DEGRADED   the active path's last reply is older than degraded_silence_s,
               or its loss over window_s is above degraded_loss_pct, or its
               mean RTT over window_s is above degraded_rtt_ms; and until
               none of that has been true for ok_after_s (no OK/DEGRADED
               flicker at a threshold).
    OK         otherwise.
"""

from collections import OrderedDict
from dataclasses import dataclass, field
import ipaddress
import math
import os
import select
import socket
import struct
import threading
import time
from typing import Callable, List, Optional, Sequence, Tuple

MAGIC = b'F1HB'
VERSION = 1
PACKET = struct.Struct('!4sBBHIIQ')

STATE_OK, STATE_DEGRADED, STATE_LOST = 0, 1, 2
STATE_NAMES = {STATE_OK: 'OK', STATE_DEGRADED: 'DEGRADED', STATE_LOST: 'LOST'}


@dataclass
class Packet:
    path: int
    nonce: int
    seq: int
    t_ns: int


def encode(path: int, nonce: int, seq: int, t_ns: int) -> bytes:
    return PACKET.pack(MAGIC, VERSION, path, 0, nonce & 0xFFFFFFFF, seq & 0xFFFFFFFF,
                       t_ns & 0xFFFFFFFFFFFFFFFF)


def decode(data: bytes) -> Optional[Packet]:
    if len(data) != PACKET.size:
        return None
    magic, version, path, _flags, nonce, seq, t_ns = PACKET.unpack(data)
    if magic != MAGIC or version != VERSION:
        return None
    return Packet(path, nonce, seq, t_ns)


# ---- link state, no I/O ---------------------------------------------------------------


class _Round:
    __slots__ = ('seq', 't', 'rtt', 'answered', 'finalized')

    def __init__(self, seq: int, t: float, n_paths: int):
        self.seq = seq
        self.t = t
        self.rtt: List[Optional[float]] = [None] * n_paths    # seconds, per path
        self.answered = False
        self.finalized = False


@dataclass
class PathView:
    name: str
    up: bool
    rtt_last_ms: Optional[float]
    rtt_mean_ms: Optional[float]
    loss_pct: Optional[float]


@dataclass
class LinkView:
    state: int
    active: Optional[int]
    rtt_ms: Optional[float]             # last, active path
    rtt_mean_ms: Optional[float]        # window mean, active path
    loss_pct: Optional[float]           # window, any path
    since_last_reply_s: float           # inf before the first reply
    rounds_sent: int
    rounds_answered: int
    rounds_lost: int
    late_replies: int
    paths: List[PathView] = field(default_factory=list)

    @property
    def state_name(self) -> str:
        return STATE_NAMES[self.state]


class LinkMonitor:
    """Rounds sent, replies received -> per-path and link statistics and the state."""

    def __init__(self, path_names: Sequence[str], start_t: float, timeout_s: float = 2.0,
                 window_s: float = 10.0, degraded_loss_pct: float = 20.0,
                 degraded_rtt_ms: float = 250.0, degraded_silence_s: float = 1.2,
                 failback_s: float = 3.0, ok_after_s: float = 2.0):
        if not path_names:
            raise ValueError('at least one path')
        if timeout_s <= 0 or window_s <= 0:
            raise ValueError('timeout_s and window_s must be > 0')
        self.names = list(path_names)
        self.start_t = start_t
        self.timeout_s = timeout_s
        self.window_s = window_s
        self.degraded_loss_pct = degraded_loss_pct
        self.degraded_rtt_ms = degraded_rtt_ms
        self.degraded_silence_s = degraded_silence_s
        self.failback_s = failback_s
        self.ok_after_s = ok_after_s
        n = len(self.names)
        self.rounds: 'OrderedDict[int, _Round]' = OrderedDict()
        self.last_reply: List[Optional[float]] = [None] * n
        self.fresh_since: List[Optional[float]] = [None] * n
        self.clear_since: Optional[float] = None     # active path fine since
        self.last_rtt: List[Optional[float]] = [None] * n
        self.active: Optional[int] = None
        self.sent = self.answered = self.lost = self.late = 0

    def on_send(self, seq: int, t: float) -> None:
        self.rounds[seq] = _Round(seq, t, len(self.names))
        self.sent += 1

    def on_reply(self, path: int, seq: int, t: float) -> Optional[float]:
        """Record a reply; return its RTT [s], or None (unknown, duplicate or late)."""
        r = self.rounds.get(seq)
        if r is None or not 0 <= path < len(self.names) or r.rtt[path] is not None:
            return None
        rtt = t - r.t
        if rtt < 0 or rtt > self.timeout_s or r.finalized:
            self.late += 1
            return None
        r.rtt[path] = rtt
        if not r.answered:
            r.answered = True
            self.answered += 1
        last = self.last_reply[path]
        if last is None or t - last > self.degraded_silence_s or \
                self.fresh_since[path] is None:
            self.fresh_since[path] = t
        self.last_reply[path] = max(t, last or t)
        self.last_rtt[path] = rtt
        return rtt

    def _is_up(self, i: int, now: float) -> bool:
        last = self.last_reply[i]
        return last is not None and now - last <= self.timeout_s

    def _choose_active(self, now: float, up: List[bool]) -> Optional[int]:
        candidates = [i for i, u in enumerate(up) if u]
        if not candidates:
            return None
        fresh = [i for i in candidates if now - self.last_reply[i] <= self.degraded_silence_s]
        cur = self.active
        if cur is None or not up[cur]:
            return (fresh or candidates)[0]
        if cur not in fresh and fresh:
            return fresh[0]
        for i in fresh:
            if i < cur and now - self.fresh_since[i] >= self.failback_s:
                return i
        return cur

    def evaluate(self, now: float) -> LinkView:
        # Rounds past the timeout without an answer are lost, once.
        for r in self.rounds.values():
            if not r.finalized and now - r.t > self.timeout_s:
                r.finalized = True
                if not r.answered:
                    self.lost += 1
        horizon = now - (self.window_s + self.timeout_s + 1.0)
        while self.rounds and next(iter(self.rounds.values())).t < horizon:
            self.rounds.popitem(last=False)

        n = len(self.names)
        up = [self._is_up(i, now) for i in range(n)]
        for i in range(n):
            if not up[i] or now - self.last_reply[i] > self.degraded_silence_s:
                self.fresh_since[i] = None
        self.active = self._choose_active(now, up)

        window = [r for r in self.rounds.values() if r.t >= now - self.window_s]
        paths = []
        for i in range(n):
            decided = [r for r in window if r.rtt[i] is not None or r.finalized]
            rtts = [r.rtt[i] for r in window if r.rtt[i] is not None]
            lost = sum(1 for r in decided if r.rtt[i] is None)
            paths.append(PathView(
                name=self.names[i], up=up[i],
                rtt_last_ms=_ms(self.last_rtt[i]) if up[i] else None,
                rtt_mean_ms=_ms(sum(rtts) / len(rtts)) if rtts else None,
                loss_pct=100.0 * lost / len(decided) if decided else None))
        decided = [r for r in window if r.answered or r.finalized]
        link_loss = (100.0 * sum(1 for r in decided if not r.answered) / len(decided)
                     if decided else None)
        replies = [t for t in self.last_reply if t is not None]
        since = now - max(replies) if replies else math.inf

        a = self.active
        if a is None:
            proving = not replies and now - self.start_t <= self.timeout_s
            state = STATE_DEGRADED if proving else STATE_LOST
            self.clear_since = None
        else:
            p = paths[a]
            slow = p.rtt_mean_ms is not None and p.rtt_mean_ms > self.degraded_rtt_ms
            lossy = p.loss_pct is not None and p.loss_pct > self.degraded_loss_pct
            silent = now - self.last_reply[a] > self.degraded_silence_s
            if slow or lossy or silent:
                self.clear_since = None
                state = STATE_DEGRADED
            else:
                if self.clear_since is None:
                    self.clear_since = now
                held = now - self.clear_since < self.ok_after_s
                state = STATE_DEGRADED if held else STATE_OK
        return LinkView(
            state=state, active=a,
            rtt_ms=paths[a].rtt_last_ms if a is not None else None,
            rtt_mean_ms=paths[a].rtt_mean_ms if a is not None else None,
            loss_pct=link_loss, since_last_reply_s=since, rounds_sent=self.sent,
            rounds_answered=self.answered, rounds_lost=self.lost, late_replies=self.late,
            paths=paths)


def _ms(seconds: Optional[float]) -> Optional[float]:
    return None if seconds is None else seconds * 1000.0


# ---- UDP client ------------------------------------------------------------------------


def is_ip_literal(host: str) -> bool:
    try:
        ipaddress.ip_address(host)
        return True
    except ValueError:
        return False


class HeartbeatClient:
    """
    One UDP socket, a receiver thread and, for host names, a resolver thread.

    send_round() never blocks: the socket is non-blocking and a host name that
    is not resolved yet (MagicDNS without Tailscale) just counts as a lost
    packet on that path. Name resolution, which can hang for seconds without
    DNS, runs only in the resolver thread.
    """

    def __init__(self, paths: Sequence[Tuple[str, str, int]], monitor: LinkMonitor,
                 resolve_period_s: float = 30.0, clock: Callable[[], float] = time.monotonic):
        self.paths = [(name, host, int(port)) for name, host, port in paths]
        self.monitor = monitor
        self.resolve_period_s = resolve_period_s
        self.clock = clock
        self.lock = threading.Lock()
        self.nonce = int.from_bytes(os.urandom(4), 'big')
        self.seq = 0
        self.send_errors = [0] * len(self.paths)
        self.bad_packets = 0
        self.resolved: List[Optional[Tuple[str, int]]] = [
            (host, port) if is_ip_literal(host) else None for _, host, port in self.paths]
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self.sock: Optional[socket.socket] = None

    def addresses(self) -> List[str]:
        return [f'{host}:{port}' for _, host, port in self.paths]

    def start(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 0))
        self.sock.setblocking(False)
        self._threads.append(threading.Thread(target=self._receive_loop, name='hb-rx',
                                              daemon=True))
        if any(r is None for r in self.resolved):
            self._threads.append(threading.Thread(target=self._resolve_loop, name='hb-dns',
                                                  daemon=True))
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        if self.sock is not None:
            self.sock.close()

    def send_round(self, now: Optional[float] = None) -> int:
        now = self.clock() if now is None else now
        with self.lock:
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            seq = self.seq
            self.monitor.on_send(seq, now)
        for i, addr in enumerate(self.resolved):
            if addr is None:
                self.send_errors[i] += 1
                continue
            try:
                self.sock.sendto(encode(i, self.nonce, seq, int(now * 1e9)), addr)
            except OSError:         # ENETUNREACH without Tailscale, EAGAIN, ...
                self.send_errors[i] += 1
        return seq

    def evaluate(self, now: Optional[float] = None) -> LinkView:
        with self.lock:
            return self.monitor.evaluate(self.clock() if now is None else now)

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self.sock], [], [], 0.5)
            except (OSError, ValueError):
                return
            if not ready:
                continue
            while True:
                try:
                    data, _ = self.sock.recvfrom(256)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    break
                t = self.clock()
                pkt = decode(data)
                if pkt is None or pkt.nonce != self.nonce:
                    self.bad_packets += 1
                    continue
                with self.lock:
                    self.monitor.on_reply(pkt.path, pkt.seq, t)

    def _resolve_loop(self) -> None:
        while not self._stop.is_set():
            for i, (_, host, port) in enumerate(self.paths):
                if is_ip_literal(host):
                    continue
                try:
                    info = socket.getaddrinfo(host, port, socket.AF_INET, socket.SOCK_DGRAM)
                    self.resolved[i] = info[0][4][:2]
                except (OSError, UnicodeError):
                    pass            # keep the last good address, if any
            self._stop.wait(self.resolve_period_s)
