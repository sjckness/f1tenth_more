"""
Manual driving: what the gateway publishes on /teleop, and when it stops.

Pure logic, no ROS. The ROS bridge (or the mock) calls tick() at publish_hz
from its own thread and publishes what it returns; the WebSocket handler
calls acquire/release/revoke/on_input and publishes the zero they return at
once. Every method takes `now` (monotonic seconds), so tests drive time.

The rules (SPEC section 5, docs/pwa/SPEC.md):

- One driver at a time: acquire() gives the lease to one client id, a second
  one gets "busy".
- A command moves the car only while the driver's dead-man is held and its
  last valid input is younger than watchdog_s (250 ms). Past that: zero, and
  the driver has to lift the thumb (deadman false) before anything moves
  again, so a stalled link that comes back does not restart the car by itself.
- Dead-man released: zero at once, returned to the caller to publish without
  waiting for the next tick.
- After any zero: zero_tail_s of zeros, then nothing at all, so the mux's
  /teleop lane (priority 100, 0.2 s timeout) times out and stops masking
  the navigation lane under it.
- Speed: 0 inside the dead zone and for thr < 0 (no reverse in v1);
  otherwise from MIN_MOVING_SPEED_MPS (the car does not move slower) up to
  the driver's limit, which is never above max_speed_mps, which is never
  above HARD_MAX_SPEED_MPS.
- Rate limits: speed rises by at most accel_mps2 (it falls at once),
  steering moves by at most steer_rate_radps, at most max_input_hz inputs a
  second are taken (a release is never dropped).
"""

from collections import deque
from dataclasses import dataclass
import math
import threading
from typing import List, Optional, Tuple

# Server-side ceiling for manual driving, whatever the config says (decision
# of 2026-10-20). Raising it is a code change on purpose.
HARD_MAX_SPEED_MPS = 1.0
# The car does not move below this: stack_params.yaml min_moving_speed_mps.
# test_teleop.py checks the two stay equal.
MIN_MOVING_SPEED_MPS = 0.4


@dataclass
class TeleopConfig:
    max_speed_mps: float = 1.0
    default_limit_mps: float = 0.6
    limit_step_mps: float = 0.1
    deadzone: float = 0.15
    max_steer_rad: float = 0.30
    accel_mps2: float = 1.0
    steer_rate_radps: float = 1.5
    watchdog_s: float = 0.25
    zero_tail_s: float = 0.5
    publish_hz: float = 50.0
    max_input_hz: float = 50.0

    def __post_init__(self):
        self.max_speed_mps = min(float(self.max_speed_mps), HARD_MAX_SPEED_MPS)
        if self.max_speed_mps < MIN_MOVING_SPEED_MPS:
            raise ValueError(f"max_speed_mps {self.max_speed_mps} is below the car's "
                             f'minimum moving speed {MIN_MOVING_SPEED_MPS}')
        self.default_limit_mps = clamp(float(self.default_limit_mps),
                                       MIN_MOVING_SPEED_MPS, self.max_speed_mps)
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError('deadzone must be in [0, 1)')
        if self.max_steer_rad <= 0 or self.watchdog_s <= 0 or self.publish_hz <= 0:
            raise ValueError('max_steer_rad, watchdog_s and publish_hz must be > 0')


@dataclass
class Command:
    speed: float
    steer: float
    reason: str  # drive | zero


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _finite(x) -> Optional[float]:
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        return None
    x = float(x)
    return x if math.isfinite(x) else None


def speed_for(thr: float, limit: float, deadzone: float) -> float:
    """Gas stick (-1..1) to a speed: 0, or MIN_MOVING_SPEED_MPS..limit."""
    if thr <= deadzone:
        return 0.0
    frac = (thr - deadzone) / (1.0 - deadzone)
    return MIN_MOVING_SPEED_MPS + clamp(frac, 0.0, 1.0) * (limit - MIN_MOVING_SPEED_MPS)


class Teleop:
    def __init__(self, cfg: TeleopConfig):
        self.cfg = cfg
        self._lock = threading.Lock()
        self.holder: Optional[str] = None
        self.limit = cfg.default_limit_mps
        self.armed = False          # dead-man held, inputs flowing
        self.tripped = False        # watchdog fired: lift the thumb first
        self.target = (0.0, 0.0)    # speed, steer the stick asks for
        self.speed = 0.0            # what is published, after the rate limits
        self.steer = 0.0
        self.last_input_t: Optional[float] = None
        self.last_seq = -1
        self.zero_until: Optional[float] = None
        self._last_tick: Optional[float] = None
        self._inputs = deque()
        self._events: List[Tuple[str, str, str]] = []
        self.stats = {'published': 0, 'zeros': 0, 'watchdog_trips': 0, 'dropped': 0,
                      'revoked': 0}

    # ---- the lease -----------------------------------------------------------

    def acquire(self, client: str, now: float) -> Tuple[bool, str]:
        with self._lock:
            if self.holder is not None and self.holder != client:
                return False, 'guida occupata da un altro client'
            self.holder = client
            self.armed = self.tripped = False
            self.last_seq = -1
            self._inputs.clear()
            return True, 'ok'

    def release(self, client: str, now: float) -> Optional[Command]:
        with self._lock:
            if self.holder != client:
                return None
            self.holder = None
            return self._zero(now)

    def revoke(self, reason: str, now: float) -> Optional[Command]:
        """Take the lease away (e-stop, mission start, lost precondition)."""
        with self._lock:
            if self.holder is not None:
                self._events.append((self.holder, 'revoked', reason))
                self.stats['revoked'] += 1
            self.holder = None
            moving = self.armed or self.speed != 0.0 or self.zero_until is not None
            return self._zero(now) if moving else None

    # ---- inputs --------------------------------------------------------------

    def on_input(self, client: str, msg: dict, now: float) -> Tuple[str, Optional[Command]]:
        """
        Take one teleop message. Return (status, command to publish now or None).

        status: ok | released | not_holder | tripped | stale | rate | invalid.
        """
        with self._lock:
            if self.holder != client:
                return 'not_holder', None
            deadman = msg.get('deadman', True) is True
            if not deadman:
                # Always honoured, whatever its seq or the input rate: a zero
                # is never the wrong thing to do.
                self.tripped = False
                cmd = self._zero(now) if (self.armed or self.speed or self.zero_until) else None
                self.armed = False
                return 'released', cmd
            seq = msg.get('seq')
            if not isinstance(seq, int) or isinstance(seq, bool) or seq <= self.last_seq:
                self.stats['dropped'] += 1
                return 'stale', None
            thr, steer = _finite(msg.get('thr')), _finite(msg.get('steer'))
            if thr is None or steer is None:
                self.stats['dropped'] += 1
                cmd = self._zero(now) if self.armed else None
                self.armed = False
                return 'invalid', cmd
            while self._inputs and now - self._inputs[0] > 1.0:
                self._inputs.popleft()
            if len(self._inputs) >= self.cfg.max_input_hz:
                self.stats['dropped'] += 1
                return 'rate', None
            self._inputs.append(now)
            self.last_seq = seq
            if self.tripped:
                return 'tripped', None
            limit = _finite(msg.get('limit'))
            if limit is not None:
                self.limit = clamp(limit, MIN_MOVING_SPEED_MPS, self.cfg.max_speed_mps)
            thr = clamp(thr, -1.0, 1.0)
            steer = clamp(steer, -1.0, 1.0)
            self.target = (speed_for(thr, self.limit, self.cfg.deadzone),
                           steer * self.cfg.max_steer_rad)
            self.armed = True
            self.last_input_t = now
            return 'ok', None

    # ---- the publish loop -----------------------------------------------------

    def tick(self, now: float) -> Optional[Command]:
        """Return what to publish now (called at publish_hz); None = publish nothing."""
        with self._lock:
            dt = 0.0 if self._last_tick is None else clamp(now - self._last_tick, 0.0, 0.1)
            self._last_tick = now
            if self.armed:
                if self.last_input_t is None or now - self.last_input_t > self.cfg.watchdog_s:
                    self.stats['watchdog_trips'] += 1
                    self.tripped = True
                    self.armed = False
                    if self.holder is not None:
                        self._events.append((self.holder, 'tripped', 'watchdog'))
                    return self._zero(now)
                return self._drive(dt)
            if self.zero_until is not None:
                if now < self.zero_until:
                    self.stats['zeros'] += 1
                    return Command(0.0, 0.0, 'zero')
                self.zero_until = None
            return None

    def _drive(self, dt: float) -> Command:
        want_v, want_s = self.target
        if want_v <= 0.0:
            self.speed = 0.0
        elif want_v < self.speed:
            self.speed = want_v
        elif self.speed < MIN_MOVING_SPEED_MPS:
            # 0 -> the slowest speed the car moves at, in one step.
            self.speed = min(want_v, MIN_MOVING_SPEED_MPS)
        else:
            self.speed = min(want_v, self.speed + self.cfg.accel_mps2 * dt)
        step = self.cfg.steer_rate_radps * dt
        self.steer += clamp(want_s - self.steer, -step, step)
        self.stats['published'] += 1
        return Command(self.speed, self.steer, 'drive')

    def _zero(self, now: float) -> Command:
        self.armed = False
        self.speed = self.steer = 0.0
        self.target = (0.0, 0.0)
        self.zero_until = now + self.cfg.zero_tail_s
        self.stats['zeros'] += 1
        return Command(0.0, 0.0, 'zero')

    # ---- for the app -----------------------------------------------------------

    def pop_events(self) -> List[Tuple[str, str, str]]:
        """(client, kind, reason) since the last call: revoked / tripped."""
        with self._lock:
            out, self._events = self._events, []
            return out

    def snapshot(self) -> dict:
        with self._lock:
            return {'holder': self.holder, 'armed': self.armed, 'tripped': self.tripped,
                    'limit_mps': round(self.limit, 2), 'speed_cmd': round(self.speed, 3),
                    'steer_cmd': round(self.steer, 4), 'stats': dict(self.stats)}
