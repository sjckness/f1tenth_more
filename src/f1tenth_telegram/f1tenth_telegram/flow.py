"""
Planner commands in two steps: plan, then start. Typed text and voice alike.

    1. propose(text)    checks, then "Plan: «text»?"          [🧭 Plan] [❌ Cancel]
    2. begin_plan(n)    the planner, plan-only, answers; f1tenth_ctl stores the
       finish_plan(ex)  mission; it becomes an Offer          [▶️ Start] [❌ Discard]
    3. offer_start(t)   "🚗 The car will drive. Start?"        [✅ Yes, start] [❌ No]
    4. start(n)         f1tenth_ctl mission-load <id> + mission-start

There is no other way to a start: every start goes through steps 3 and 4,
and f1tenth_ctl itself refuses mission-start unless READY, the e-stop is
clear and no other mission runs. The questions of steps 1 and 3 expire 10 s
after they are asked (max_command_age_s); a plan older than 5 min cannot be
started. One planning at a time, and none while a mission is starting or
running.

Each method does one blocking step (ssh to the car); the Telegram front end
runs them in a thread, the terminal front end directly.
"""

from dataclasses import dataclass, field
import secrets
import threading
import time
from typing import Dict, Optional

from . import planner
from .config import Config
from .guard import Confirmations

PLAN_MAX_AGE_S = 300.0
ACTIVE = ('starting', 'running')
# f1tenth_ctl's planner-log says plan_only: true only for a planner running
# with --dry-run. An older f1tenth_ctl on the car says nothing: refused too.
NOT_PLAN_ONLY = ('the planner on the car is not plan-only, it would start the mission by '
                 'itself: update src/f1tenth_ctl on the car (telegram-control), then ⏹ and '
                 '▶️ Planner')


def collapse(text: Optional[str]) -> str:
    """Line breaks and runs of spaces become one space (typed text and transcripts)."""
    return ' '.join((text or '').split())


@dataclass
class Offer:
    token: str
    text: str
    id: str  # noqa: A003 - f1tenth_ctl's mission id
    stored_at: float
    entry: dict = field(repr=False)


class Flow:
    def __init__(self, transport, cfg: Config, clock=time.time):
        self.t = transport
        self.cfg = cfg
        self.clock = clock
        self.asks = Confirmations(cfg.max_command_age_s, clock)     # "Plan: «…»?"
        self.starts = Confirmations(cfg.max_command_age_s, clock)   # "🚗 … Start?"
        self.offers: Dict[str, Offer] = {}
        self.planning: Optional[str] = None
        self._lock = threading.Lock()

    def _ctl(self, args, kind='default') -> dict:
        return self.t.ctl(args, self.cfg.timeout(kind))

    def busy(self) -> Optional[str]:
        """Return why no new planning can start now, or None."""
        if self.planning:
            return f'still planning «{self.planning}»'
        st = self._ctl(['mission-status'], 'status')
        if st.get('state') in ACTIVE:
            who = st.get('id') or 'not started by the bot'
            return f"a mission is {st['state']} ({who}): wait for it to end, or 🛑"
        return None

    # ---- 1. the question ---------------------------------------------------------

    def propose(self, raw: str) -> dict:
        text, why = planner.validate(collapse(raw))
        if why:
            return {'kind': 'invalid', 'why': why}
        busy = self.busy()
        if busy:
            return {'kind': 'refused', 'why': busy, 'text': text}
        res = self._ctl(['planner-log'], 'status')
        if res.get('transport_error') or not res.get('ok'):
            return {'kind': 'error', 'why': res.get('error') or 'planner-log failed'}
        if not res.get('running'):
            return {'kind': 'not_running', 'text': text}
        if res.get('plan_only') is not True:
            return {'kind': 'refused', 'text': text, 'why': NOT_PLAN_ONLY}
        return {'kind': 'ask', 'text': text, 'nonce': self.asks.ask(text)}

    def cancel_plan(self, nonce: str) -> None:
        self.asks.cancel(nonce)

    # ---- 2. planning ---------------------------------------------------------------

    def begin_plan(self, nonce: str):
        """Return (Exchange, None) to follow, or (None, why)."""
        text, why = self.asks.take(nonce)
        if text is None:
            return None, why
        busy = self.busy()
        if busy:
            return None, busy
        # Checked again right before the send: a planner that is not
        # plan-only would load and start the mission by itself.
        res = self._ctl(['planner-log'], 'status')
        if res.get('plan_only') is not True:
            return None, NOT_PLAN_ONLY if res.get('running') else 'the planner is not running'
        with self._lock:
            if self.planning:
                return None, f'still planning «{self.planning}»'
            self.planning = text
        ex = planner.Exchange(text)
        if not ex.send(self.t, self.cfg.timeout('default')):
            self.planning = None
        return ex, None

    def finish_plan(self, ex: 'planner.Exchange') -> Optional[Offer]:
        """End the planning; a stored mission becomes the (only) offer."""
        self.planning = None
        entry = ex.final_result()
        if not entry or entry.get('status') != 'ok' or not entry.get('stored'):
            return None
        rec = entry['stored']
        token = secrets.token_hex(4)
        offer = Offer(token=token, text=ex.text, id=rec['id'],
                      stored_at=float(rec.get('stored_at') or self.clock()), entry=entry)
        self.offers = {token: offer}       # a new plan replaces any older offer
        return offer

    # ---- 3. the start question ---------------------------------------------------

    def age_s(self, offer: Offer) -> float:
        return self.clock() - offer.stored_at

    def offer_start(self, token: str) -> dict:
        offer = self.offers.get(token)
        if offer is None:
            return {'kind': 'gone', 'why': 'this plan was discarded or replaced by a newer one'}
        if self.age_s(offer) > PLAN_MAX_AGE_S:
            self.offers.pop(token, None)
            return {'kind': 'expired', 'why': 'the plan is older than 5 min: plan again'}
        return {'kind': 'ask', 'offer': offer, 'nonce': self.starts.ask(token)}

    def discard(self, token: str) -> bool:
        return self.offers.pop(token, None) is not None

    def cancel_start(self, nonce: str) -> None:
        self.starts.cancel(nonce)

    # ---- 4. the start ----------------------------------------------------------------

    def start(self, nonce: str) -> dict:
        token, why = self.starts.take(nonce)
        if token is None:
            return {'ok': False, 'why': why}
        offer = self.offers.get(token)
        if offer is None:
            return {'ok': False, 'why': 'this plan was discarded or replaced by a newer one'}
        if self.age_s(offer) > PLAN_MAX_AGE_S:
            self.offers.pop(token, None)
            return {'ok': False, 'why': 'the plan is older than 5 min: plan again'}
        load = self._ctl(['mission-load', offer.id], 'mission')
        if not load.get('ok'):
            return {'ok': False, 'id': offer.id, 'load': load,
                    'why': load.get('refused') or load.get('error') or 'mission-load failed'}
        start = self._ctl(['mission-start', '--id', offer.id], 'mission')
        if not start.get('ok'):
            return {'ok': False, 'id': offer.id, 'load': load, 'start': start,
                    'why': start.get('refused') or start.get('error') or 'mission-start failed'}
        self.offers.pop(token, None)       # one start per plan
        return {'ok': True, 'id': offer.id, 'load': load, 'start': start,
                'countdown_s': start.get('countdown_s')}
