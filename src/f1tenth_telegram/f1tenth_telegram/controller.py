"""
The control panel's state and its actions, independent of Telegram.

The Telegram front end (tg.py) and the terminal one (cli.py) both call
Controller.handle(action) and render the Panel it returns (view.py). Actions:

    refresh | connect | status | estop | estop-reset | killall | help:command
    start:<proc> | stop:<proc>          proc = stack | planner | logger
    confirm:<nonce> | cancel:<nonce>

One action runs at a time; a second one arriving meanwhile is refused, not
queued (a queued command would run late). The exception is estop, which
never waits for anything.
"""

from dataclasses import dataclass, field
import threading
import time
from typing import Optional

from .config import Config
from .guard import Confirmations, NEEDS_CONFIRM
from .transport import Transport

PROCS = ('stack', 'planner', 'logger')
COMMAND_HELP = ('📝 Write the command as a message, or say it in a voice message, e.g. '
                '«vai dritto per due metri» (at most 500 characters). Two steps: 🧭 Plan '
                'shows the planned mission without starting it; ▶️ Start mission, then '
                '✅ Yes, start runs it. The planner must be running (▶️ Planner).')
LABEL = {'start:stack': 'start the stack', 'start:planner': 'start the planner',
         'killall': 'kill ALL ROS 2 processes',
         'estop-reset': 'reset the e-stop (aborts the mission, restarts behavior)'}


@dataclass
class Panel:
    target: str
    reach: Optional[dict] = None
    connected: bool = False
    connect_info: Optional[dict] = None
    status: Optional[dict] = None
    last: Optional[str] = None
    busy: Optional[str] = None
    confirm: Optional[tuple] = None          # (nonce, action)
    updated: Optional[float] = None
    history: list = field(default_factory=list)

    @property
    def online(self) -> Optional[bool]:
        return None if self.reach is None else bool(self.reach.get('online'))


class Controller:
    def __init__(self, cfg: Config, transport: Transport, clock=time.time):
        self.cfg = cfg
        self.t = transport
        self.clock = clock
        self.panel = Panel(cfg.target.name)
        self.confirms = Confirmations(cfg.max_command_age_s, clock)
        self._lock = threading.Lock()

    # ---- entry point -------------------------------------------------------

    def handle(self, action: str) -> Panel:
        if action == 'estop':
            return self._estop()
        if not self._lock.acquire(blocking=False):
            self._note(f'⏳ busy with {self.panel.busy}: "{action}" was not queued')
            return self.panel
        try:
            self.panel.busy = action
            self._dispatch(action)
        finally:
            self.panel.busy = None
            self.panel.updated = self.clock()
            self._lock.release()
        return self.panel

    def _dispatch(self, action: str) -> None:
        if action.startswith('confirm:'):
            act, why = self.confirms.take(action.split(':', 1)[1])
            self.panel.confirm = None
            if act is None:
                self._note(f'✖ {why}')
                return
            self._run(act)
            return
        if action.startswith('cancel:'):
            self.confirms.cancel(action.split(':', 1)[1])
            self.panel.confirm = None
            self._note('✖ cancelled')
            return
        if action in NEEDS_CONFIRM:
            nonce = self.confirms.ask(action)
            self.panel.confirm = (nonce, action)
            self._note(f'❓ really {LABEL[action]}?')
            return
        self._run(action)

    def _run(self, action: str) -> None:
        if action == 'refresh':
            self._refresh()
        elif action == 'connect':
            self._connect()
        elif action == 'status':
            if self.panel.connected:
                self._status()
            else:
                self._refresh()
        elif action == 'killall':
            res = self._ctl(['killall'], 'killall')
            self._note(summary_killall(res))
            self._status()
        elif action == 'estop-reset':
            res = self._ctl(['estop-reset'], 'estop_reset')
            self._note(summary_estop_reset(res))
            self._status()
        elif action == 'help:command':
            self._note(COMMAND_HELP)
        elif action.split(':')[0] in ('start', 'stop') and action.split(':')[-1] in PROCS:
            verb, proc = action.split(':')
            res = self._ctl([verb, proc], verb)
            self._note(summary_startstop(verb, proc, res))
            self._status()
        else:
            self._note(f'✖ unknown action "{action}"')

    # ---- actions -------------------------------------------------------------

    def _refresh(self) -> None:
        self.panel.reach = self.t.reachability(self.cfg.timeout('reach'))
        if not self.panel.online:
            self.panel.connected = False
            self.panel.status = None
            return
        if self.panel.connected:
            self._status()

    def _connect(self) -> None:
        self.panel.reach = self.t.reachability(self.cfg.timeout('reach'))
        if not self.panel.online:
            self.panel.connected = False
            self._note('🔌 not connected: the car is offline')
            return
        res = self._ctl(['connect'], 'connect')
        self.panel.connect_info = res
        self.panel.connected = bool(res.get('ok'))
        self._note(summary_connect(res))
        if self.panel.connected:
            self._status()

    def _status(self) -> None:
        res = self._ctl(['status'], 'status')
        if res.get('transport_error'):
            self.panel.status = None
            self._note(f'🔄 status failed: {res.get("error")}')
            return
        self.panel.status = res

    def _estop(self) -> Panel:
        # Never waits for the lock: it runs even while killall is running.
        res = self.t.ctl(['estop'], self.cfg.timeout('estop'))
        self._note(summary_estop(res))
        if res.get('transport_error'):
            self._recheck(res)
        if self._lock.acquire(blocking=False):
            try:
                if self.panel.connected:
                    self._status()
            finally:
                self._lock.release()
        self.panel.updated = self.clock()
        return self.panel

    # ---- helpers ---------------------------------------------------------------

    def _ctl(self, args, kind) -> dict:
        res = self.t.ctl(args, self.cfg.timeout(kind))
        if res.get('transport_error') in ('ssh', 'timeout'):
            self._recheck(res)
        return res

    def _recheck(self, res) -> None:
        """Check, after an ssh failure or timeout, whether the car is still online."""
        self.panel.reach = self.t.reachability(self.cfg.timeout('reach'))
        if not self.panel.online:
            self.panel.connected = False
            self.panel.status = None

    def note(self, line: str) -> None:
        """Put a line under Last: on the panel (front ends use it too)."""
        self._note(line)

    def _note(self, line: str) -> None:
        self.panel.last = line
        stamp = time.strftime('%H:%M:%S', time.localtime(self.clock()))
        self.panel.history = (self.panel.history + [f'{stamp} {line}'])[-20:]


# ---- one-line summaries of f1tenth_ctl answers --------------------------------

def _err(res: dict) -> str:
    return str(res.get('error') or ', '.join(res.get('failing') or []) or 'failed')


def summary_connect(res: dict) -> str:
    if res.get('ok'):
        disc = res.get('discovery') or {}
        where = disc.get('address') or 'not used'
        return f'🔌 connected: discovery {where}, environment ok, ready to launch'
    return f'🔌 connect failed: {_err(res)}'


def summary_startstop(verb: str, proc: str, res: dict) -> str:
    icon = '▶️' if verb == 'start' else '⏹'
    secs = res.get('elapsed_s') or res.get('_elapsed_s')
    if not res.get('ok'):
        return f'{icon} {proc} NOT {"started" if verb == "start" else "stopped"}: {_err(res)}'
    if verb == 'start':
        warn = f' ({res["warning"]})' if res.get('warning') else ''
        return f'{icon} {proc} started, pid {res.get("pid")}, {secs} s{warn}'
    if res.get('already'):
        return f'{icon} {proc} was already {res["already"]}'
    sigs = '→'.join(res.get('signals') or [])
    return f'{icon} {proc} stopped ({sigs}, {res.get("elapsed_s")} s)'


def summary_killall(res: dict) -> str:
    if res.get('transport_error'):
        return f'☠️ killall: {_err(res)}'
    units = res.get('units') or {}
    stopped = [n for n, u in units.items() if u.get('ok') and not u.get('already')]
    strays = (res.get('strays') or {}).get('count', 0)
    disc = (res.get('discovery') or {}).get('state', '?')
    left = res.get('remaining') or []
    head = '☠️ killall done' if res.get('ok') else '☠️ killall INCOMPLETE'
    out = (f'{head}: units stopped {", ".join(stopped) or "none"}; '
           f'{strays} other ROS process(es) signalled; discovery {disc}')
    if left:
        out += f'; STILL RUNNING: {", ".join(str(p["pid"]) for p in left)}'
    return out


def summary_estop_reset(res: dict) -> str:
    path = {'restart_behavior': 'behavior restarted', 'reset_service': 'reset service',
            'stack_restart': 'stack restarted', 'none': 'nothing to do'}.get(
                res.get('path'), res.get('path') or '?')
    if res.get('ok'):
        if res.get('path') == 'none':
            return '🔓 e-stop was not latched: nothing to do'
        return f'🔓 e-stop reset ({path}), latch clear, checked on /mission/status'
    return f'🔓 e-stop reset FAILED ({path}): {_err(res)}'


ESTOP_STEP_LABEL = (('zero_drive', 'zero drive on /safety_stop'),
                    ('emergency_stop', 'emergency_stop (latch)'),
                    ('abort_mission', 'abort_mission (mission + MPC hold)'))
STEP_ICON = {'ok': '✅', 'failed': '❌', 'skipped': '⏭'}


def summary_estop(res: dict) -> str:
    """One line for the result, then one line per step."""
    if res.get('transport_error'):
        return f'🛑 E-STOP NOT SENT: {_err(res)}'
    steps = res.get('steps') or {}
    head = ('🛑 E-STOP: car held ✅' if res.get('car_held')
            else '🛑 E-STOP: car NOT confirmed held ❌')
    lines = [head]
    for key, label in ESTOP_STEP_LABEL:
        st = steps.get(key) or {}
        icon = STEP_ICON.get(st.get('status'), '❔')
        late = ' LATE' if st.get('late') else ''
        lines.append(f'{icon} {label}: {st.get("message", "no answer")} '
                     f'({st.get("duration_ms", "?")} ms{late})')
    return '\n'.join(lines)
