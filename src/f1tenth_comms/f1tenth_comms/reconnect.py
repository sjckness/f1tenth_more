"""
Reconnect attempts while the base is unreachable: when to try, and what a try does.

NetworkManager owns the Wi-Fi association. link_manager only asks it, now
and then, to bring the hotspot profile up again. One attempt at a time, in
a one-thread pool: nothing here ever runs on the ROS executor thread.

WHEN (ReconnectPolicy), only while the link is LOST:
    time       retry_period_s after the previous attempt, or after LOST began.
               With retry_backoff_max_s > 0 the interval doubles after each
               attempt of the same outage, up to that cap; 0 keeps it fixed.
    distance   retry_distance_m travelled (odom frame) since the previous
               attempt, or since LOST began.
    new_cell   the car entered a coverage cell that was not mapped yet.
and never sooner than retry_min_interval_s after the previous attempt (or
after LOST began). 0 turns a trigger off. A new cell or the distance seen
during that wait still fires once the wait is over.

WHAT (NmcliReconnector.attempt): read-only nmcli checks, then at most one
`nmcli --wait <nmcli_wait_s> connection up id <profile> ifname <if>`:
    1. the device is connecting (NM state 40..90), unavailable or unmanaged
       -> skipped: NetworkManager is busy, or there is no radio to use.
    2. already on the hotspot profile with a signal above
       skip_if_rssi_above_dbm -> skipped: the radio link is fine and the
       base itself is not answering; re-associating would only cut the car
       off for seconds.
    3. on another network (eduroam) and the hotspot SSID is not in
       NetworkManager's scan list -> skipped: nothing to switch to, and
       leaving eduroam would lose the backup.
    4. otherwise -> connection up. "ok" when nmcli exits 0.
"""

from concurrent.futures import Future, ThreadPoolExecutor
import os
import subprocess
from typing import Callable, List, Optional, Tuple

TRIGGERS = ('new_cell', 'distance', 'time')


class ReconnectPolicy:
    def __init__(self, retry_period_s: float = 2.5, retry_backoff_max_s: float = 0.0,
                 retry_distance_m: float = 3.0, retry_on_new_cell: bool = True,
                 retry_min_interval_s: float = 2.0):
        self.retry_period_s = float(retry_period_s)
        self.retry_backoff_max_s = float(retry_backoff_max_s)
        self.retry_distance_m = float(retry_distance_m)
        self.retry_on_new_cell = bool(retry_on_new_cell)
        self.retry_min_interval_s = float(retry_min_interval_s)
        self.lost_since: Optional[float] = None
        self.last_attempt_t: Optional[float] = None     # in this outage
        self.outage_attempts = 0
        self.distance_m = 0.0
        self.new_cell_pending = False

    @property
    def lost(self) -> bool:
        return self.lost_since is not None

    def update_link(self, now: float, lost: bool) -> None:
        if not lost:
            self.lost_since = self.last_attempt_t = None
            self.outage_attempts = 0
            self.distance_m = 0.0
            self.new_cell_pending = False
        elif self.lost_since is None:
            self.lost_since = now

    def add_distance(self, metres: float) -> None:
        if self.lost:
            self.distance_m += max(0.0, metres)

    def entered_new_cell(self) -> None:
        if self.lost:
            self.new_cell_pending = True

    def interval(self) -> float:
        """Return the time trigger's interval after this outage's attempts so far."""
        if self.retry_backoff_max_s <= 0:
            return self.retry_period_s
        grown = self.retry_period_s * (2 ** self.outage_attempts)
        return min(max(self.retry_backoff_max_s, self.retry_period_s), grown)

    def due(self, now: float, in_flight: bool = False) -> Optional[str]:
        """Return the trigger that fires now ('new_cell', 'distance', 'time') or None."""
        if not self.lost or in_flight:
            return None
        ref = self.last_attempt_t if self.last_attempt_t is not None else self.lost_since
        if now - ref < self.retry_min_interval_s:
            return None
        if self.retry_on_new_cell and self.new_cell_pending:
            return 'new_cell'
        if self.retry_distance_m > 0 and self.distance_m >= self.retry_distance_m:
            return 'distance'
        if self.retry_period_s > 0 and now - ref >= self.interval():
            return 'time'
        return None

    def started(self, now: float) -> None:
        self.last_attempt_t = now
        self.outage_attempts += 1
        self.distance_m = 0.0
        self.new_cell_pending = False


# ---- what one attempt does ----------------------------------------------------------

RunResult = Tuple[int, str, str]


def run_command(cmd: List[str], timeout: float) -> RunResult:
    """Run cmd with the C locale; (returncode, stdout, stderr); 124 on a timeout."""
    env = dict(os.environ, LC_ALL='C', LANG='C')
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env,
                           check=False)
    except subprocess.TimeoutExpired:
        return 124, '', f'timed out after {timeout:g} s'
    except OSError as exc:
        return 127, '', str(exc)
    return p.returncode, p.stdout, p.stderr


def parse_terse(text: str) -> dict:
    """Return {field: value} from `nmcli -t -f A,B ... show` (values unescaped)."""
    out = {}
    for line in text.splitlines():
        key, sep, value = line.partition(':')
        if sep:
            out[key] = value.replace('\\:', ':').replace('\\\\', '\\')
    return out


def parse_ssid_list(text: str) -> List[str]:
    """Return the SSIDs of `nmcli -t -f SSID device wifi list` (one per line)."""
    return [line.replace('\\:', ':').replace('\\\\', '\\') for line in text.splitlines()
            if line]


def _last_line(*texts: str) -> str:
    for text in texts:
        lines = [x.strip() for x in text.strip().splitlines() if x.strip()]
        if lines:
            return lines[-1]
    return 'no output'


class NmcliReconnector:
    def __init__(self, interface: str, connection: str, ssid: str = '',
                 wait_s: float = 15.0, skip_if_rssi_above_dbm: float = -65.0,
                 require_ssid_in_scan: bool = True, rescan: str = 'auto',
                 nmcli: str = 'nmcli', dry_run: bool = False,
                 run: Callable[[List[str], float], RunResult] = run_command):
        self.interface = interface
        self.connection = connection
        self.ssid = ssid
        self.wait_s = float(wait_s)
        self.skip_if_rssi_above_dbm = float(skip_if_rssi_above_dbm)
        self.require_ssid_in_scan = require_ssid_in_scan
        self.rescan = rescan
        self.nmcli = nmcli
        self.dry_run = dry_run
        self.run = run

    def up_command(self) -> List[str]:
        return [self.nmcli, '--wait', f'{self.wait_s:g}', 'connection', 'up', 'id',
                self.connection, 'ifname', self.interface]

    def _hotspot_ssid(self) -> Optional[str]:
        if not self.ssid:
            rc, out, _ = self.run([self.nmcli, '-t', '-f', '802-11-wireless.ssid',
                                   'connection', 'show', 'id', self.connection], 5.0)
            if rc == 0:
                self.ssid = parse_terse(out).get('802-11-wireless.ssid', '')
        return self.ssid or None

    def attempt(self, rssi_dbm: Optional[float] = None) -> str:
        """Try once, in a worker thread; return what happened, in one line."""
        if not self.interface:
            return 'skipped: no Wi-Fi interface'
        rc, out, err = self.run([self.nmcli, '-t', '-f', 'GENERAL.STATE,GENERAL.CONNECTION',
                                 'device', 'show', self.interface], 5.0)
        if rc != 0:
            return f'skipped: nmcli device show: {_last_line(err, out)}'
        dev = parse_terse(out)
        try:
            state = int(dev.get('GENERAL.STATE', '').split()[0])
        except (IndexError, ValueError):
            state = 0
        current = dev.get('GENERAL.CONNECTION', '').strip()
        if 40 <= state <= 90:
            return f'skipped: {self.interface} busy (NM state {state})'
        if state in (10, 20):
            return f'skipped: {self.interface} unmanaged/unavailable (NM state {state})'

        ssid = self._hotspot_ssid()
        if ssid is None:
            return f"skipped: no NetworkManager profile '{self.connection}' with an SSID"
        if current == self.connection:
            if self.skip_if_rssi_above_dbm < 0 and rssi_dbm is not None and \
                    rssi_dbm > self.skip_if_rssi_above_dbm:
                return (f'skipped: on {self.connection} at {rssi_dbm:.0f} dBm: the radio is '
                        'fine, the base does not answer')
        elif current and self.require_ssid_in_scan:
            rc, out, err = self.run([self.nmcli, '-t', '-f', 'SSID', 'device', 'wifi', 'list',
                                     '--rescan', self.rescan, 'ifname', self.interface], 20.0)
            if rc != 0:
                return f'skipped: nmcli wifi list: {_last_line(err, out)}'
            if ssid not in parse_ssid_list(out):
                return f"skipped: on {current}, '{ssid}' not in scan"

        cmd = self.up_command()
        if self.dry_run:
            return 'dry-run: would run ' + ' '.join(cmd)
        rc, out, err = self.run(cmd, self.wait_s + 5.0)
        if rc == 0:
            return 'ok'
        return f'failed: {_last_line(err, out)}'


# ---- policy + one worker thread ---------------------------------------------------------


class ReconnectDriver:
    """
    What link_manager calls once per tick: due? -> submit one attempt; collect the result.

    submit(fn, *args) -> Future runs the attempt elsewhere (a one-thread pool by
    default); the driver never starts a second attempt while one is running.
    """

    def __init__(self, policy: ReconnectPolicy, attempt: Callable[[Optional[float]], str],
                 enabled: bool = True, submit: Optional[Callable[..., Future]] = None):
        self.policy = policy
        self.attempt = attempt
        self.enabled = enabled
        self._pool = None
        if submit is None:
            self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='reconnect')
            submit = self._pool.submit
        self.submit = submit
        self.future: Optional[Future] = None
        self.attempts = 0
        self.completed = 0
        self.last_trigger = ''
        self.last_result = ''
        self.last_attempt_t: Optional[float] = None
        self.results: List[Tuple[float, str, str]] = []     # (t, trigger, result), last 20

    @property
    def in_flight(self) -> bool:
        return self.future is not None and not self.future.done()

    def tick(self, now: float, rssi_dbm: Optional[float] = None) -> Optional[str]:
        """Collect a finished attempt, start a due one; return the trigger started, if any."""
        if self.future is not None and self.future.done():
            try:
                self.last_result = self.future.result()
            except Exception as exc:    # noqa: BLE001 - a bug in an attempt must not stop us
                self.last_result = f'failed: {type(exc).__name__}: {exc}'
            self.results = (self.results + [(now, self.last_trigger, self.last_result)])[-20:]
            self.completed += 1
            self.future = None
        if not self.enabled:
            return None
        trigger = self.policy.due(now, in_flight=self.in_flight)
        if trigger is None:
            return None
        self.policy.started(now)
        self.attempts += 1
        self.last_trigger = trigger
        self.last_attempt_t = now
        self.future = self.submit(self.attempt, rssi_dbm)
        return trigger

    def shutdown(self) -> None:
        if self._pool is not None:
            self._pool.shutdown(wait=False)
