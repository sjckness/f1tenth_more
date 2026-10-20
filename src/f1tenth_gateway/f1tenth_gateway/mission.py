"""
The mission as the phone shows it, and the abort when the phone goes away.

/mission/status carries only state, json_path and the e-stop latch. The rest
is put together here, without changing the stack:

- the state name: f1tenth_ctl's missions.describe_state (the same one the
  Telegram bot shows), which also turns a LOADED mission inside its start
  countdown into 'starting';
- the moves: read once from the mission file at json_path;
- the active move: one /mission/move_outcome arrives per FINISHED move, so
  the active one is the next (1-based, never past the last);
- the clock: elapsed from the RUNNING edge, countdown left from the start's
  own answer ("starting in 15.0 s", f1tenth_ctl countdown_s).

Auto-abort (decision of 2026-10-20): a mission the app started is aborted
(/mission/abort_mission, not the e-stop) once no client has been heard for
auto_abort_s, during the countdown too. Missions started from Telegram or a
terminal are never touched. One abort per start.
"""

import json
import os
from typing import Callable, Dict, List, Optional

ACTIVE = ('starting', 'running', 'holding')


def read_moves(path: str) -> Dict:
    """Return the mission file's id and move ids; {} when it cannot be read."""
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    moves = data.get('moves') if isinstance(data, dict) else None
    if not isinstance(moves, list):
        return {}
    return {'mission_id': data.get('mission_id'),
            'moves': [str(m.get('id', i + 1)) if isinstance(m, dict) else str(i + 1)
                      for i, m in enumerate(moves)]}


class MissionTracker:
    def __init__(self, describe: Callable, auto_abort_s: float = 2.0,
                 reader: Callable[[str], Dict] = read_moves):
        """describe(ros_state, json_path, now) -> f1tenth_ctl describe_state() dict."""
        self.describe = describe
        self.auto_abort_s = float(auto_abort_s)
        self.reader = reader
        self.ros_state: Optional[str] = None
        self.json_path: Optional[str] = None
        self.estop_latched = False
        self.status_t: Optional[float] = None
        self._files: Dict[str, Dict] = {}
        self._finished: List[str] = []
        self._running_since: Optional[float] = None
        self._ended_at: Optional[float] = None
        # The start this app made: path, wall time, countdown.
        self.app_start: Optional[Dict] = None
        self._aborted_start: Optional[float] = None
        self.last_auto_abort: Optional[Dict] = None

    # ---- inputs ------------------------------------------------------------------

    def on_status(self, state: str, json_path: str, estop: bool, now: float) -> None:
        if state == 'RUNNING' and self.ros_state != 'RUNNING':
            self._running_since = now
            self._ended_at = None
        if state in ('LOADED',) and self.ros_state not in ('LOADED',):
            self._finished = []
            self._running_since = None
        ended = state in ('COMPLETE', 'ABORTED', 'IDLE')
        if ended and self.ros_state in ('RUNNING', 'HOLDING'):
            self._ended_at = now
        if json_path != self.json_path:
            self._finished = []
        self.ros_state, self.json_path = state, json_path
        self.estop_latched = bool(estop)
        self.status_t = now

    def on_move_outcome(self, mission_id: str, move_id: str) -> None:
        info = self._file()
        if info.get('mission_id') and mission_id and mission_id != info['mission_id']:
            return
        self._finished.append(move_id)

    def on_app_start(self, path: str, countdown_s: float, wall_now: float) -> None:
        self.app_start = {'path': path, 't': wall_now, 'countdown_s': float(countdown_s or 0.0)}
        self._aborted_start = None
        self._finished = []

    # ---- the view ------------------------------------------------------------------

    def _file(self) -> Dict:
        path = self.json_path
        if not path:
            return {}
        if path not in self._files:
            self._files[path] = self.reader(path)
        return self._files[path]

    def started_by_app(self) -> bool:
        return bool(self.app_start and self.json_path and
                    os.path.realpath(self.app_start['path']) == os.path.realpath(self.json_path))

    def view(self, now: float, wall_now: float) -> Dict:
        if self.ros_state is None:
            return {'state': 'unknown', 'id': None}
        base = dict(self.describe(self.ros_state, self.json_path, wall_now))
        info = self._file()
        moves = info.get('moves') or []
        n = len(moves)
        done = min(len(self._finished), n)
        state = base.get('state')
        if state in ('finished',) or self.ros_state == 'COMPLETE':
            index = n
        else:
            index = min(done + 1, n) if n else None
        out = {**base, 'mission_id': info.get('mission_id'), 'n_moves': n or None,
               'moves_done': done, 'move_index': index if state in ACTIVE or
               self.ros_state == 'COMPLETE' else None,
               'move_id': moves[index - 1] if index and state in ACTIVE else None,
               'started_by_app': self.started_by_app(), 'estop_latched': self.estop_latched}
        if self._running_since is not None and state in ACTIVE:
            out['elapsed_s'] = round(now - self._running_since, 1)
        elif self._running_since is not None and self._ended_at is not None:
            out['elapsed_s'] = round(self._ended_at - self._running_since, 1)
        if state == 'starting' and self.started_by_app():
            left = self.app_start['countdown_s'] - (wall_now - self.app_start['t'])
            out['countdown_left_s'] = round(max(0.0, left), 1)
        return out

    def active(self, now: float, wall_now: float) -> bool:
        return self.view(now, wall_now).get('state') in ACTIVE

    # ---- the abort when nobody is watching ---------------------------------------------

    def auto_abort_due(self, now: float, wall_now: float, last_client_t: float) -> bool:
        """Tell, once per app start, that it is active, ours and nobody spoke for auto_abort_s."""
        if not self.started_by_app() or self._aborted_start == self.app_start['t']:
            return False
        if self.view(now, wall_now).get('state') not in ACTIVE:
            return False
        return now - last_client_t >= self.auto_abort_s

    def mark_auto_abort(self, result: Dict, wall_now: float) -> None:
        self._aborted_start = self.app_start['t'] if self.app_start else None
        self.last_auto_abort = {'time': wall_now, 'result': result}
