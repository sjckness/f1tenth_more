"""
Who may command the car, and when a command is too old to run.

- Whitelist: only the numeric Telegram user ids in the config. Everyone
  else is ignored (logged, never answered).
- Stale commands: a message older than max_command_age_s (10 s) is dropped.
  Telegram stamps messages but not button presses, so for buttons the bot
  relies on three things instead: pending updates are dropped at start-up,
  a button of an old panel message is refused, and the actions that need a
  confirmation (start stack, start planner, killall, e-stop reset) expire if the
  confirmation comes later than max_command_age_s after the question.
"""

from dataclasses import dataclass
import secrets
import time
from typing import Dict, Iterable, Optional

NEEDS_CONFIRM = ('start:stack', 'start:planner', 'killall', 'estop-reset')


def allowed(user_id: Optional[int], whitelist: Iterable[int]) -> bool:
    return user_id is not None and int(user_id) in {int(x) for x in whitelist}


def is_stale(sent_at: float, now: float, max_age_s: float) -> bool:
    """Say whether a command sent at `sent_at` (epoch s) is too old to run."""
    return now - sent_at > max_age_s


@dataclass
class Pending:
    action: str
    created: float


class Confirmations:
    """Actions waiting for a yes, keyed by a short random nonce."""

    def __init__(self, ttl_s: float, clock=time.time):
        self.ttl_s = ttl_s
        self.clock = clock
        self._pending: Dict[str, Pending] = {}

    def ask(self, action: str) -> str:
        self._pending.clear()   # one question at a time; a new one replaces it
        nonce = secrets.token_hex(4)
        self._pending[nonce] = Pending(action, self.clock())
        return nonce

    def take(self, nonce: str):
        """Return (action, None) if confirmed in time, else (None, reason)."""
        p = self._pending.pop(nonce, None)
        if p is None:
            return None, 'this confirmation is no longer valid'
        if is_stale(p.created, self.clock(), self.ttl_s):
            return None, f'confirmation came after {self.ttl_s:g} s: ask again'
        return p.action, None

    def cancel(self, nonce: str) -> None:
        self._pending.pop(nonce, None)

    def pending(self) -> Optional[tuple]:
        """Return (nonce, action) of the open question, if any."""
        for nonce, p in self._pending.items():
            if not is_stale(p.created, self.clock(), self.ttl_s):
                return nonce, p.action
        return None
