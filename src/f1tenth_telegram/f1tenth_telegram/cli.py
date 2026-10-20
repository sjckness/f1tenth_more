"""
--cli: the same panel and handlers in a terminal, without Telegram.

Interactive: the panel is printed with numbered buttons; type a number, an
action name (refresh, connect, start:stack, ...) or q. Scripted: each
--do ACTION runs in order and the panel is printed after each one; `confirm`
and `cancel` answer the open confirmation, `wait:S` pauses S seconds and
`until-ready:S` repeats status until READY (at most S seconds).

`say:<text>` (or `say <text>` interactively) is what a plain Telegram message
is: a command for the planner. It asks first; `confirm` sends it and follows
the planner's answer to its RESULT line, `cancel` drops it.
"""

import sys
import time
from typing import Iterable, Optional

from . import planner, view
from .controller import Controller, Panel
from .guard import Confirmations


def show(panel: Panel, out=sys.stdout) -> list:
    text, rows = view.render(panel)
    flat = [b for row in rows for b in row]
    bar = '─' * 60
    out.write(f'{bar}\n{text}\n')
    out.write('  '.join(f'[{i}] {b.label}' for i, b in enumerate(flat, 1)) + '\n')
    out.write(f'{bar}\n')
    out.flush()
    return flat


def resolve(token: str, panel: Panel, flat: list) -> Optional[str]:
    """Map what was typed to an action: a button number, confirm/cancel, a name."""
    token = token.strip()
    if token.isdigit():
        i = int(token) - 1
        return flat[i].action if 0 <= i < len(flat) else None
    if token in ('confirm', 'yes', 'y', 'cancel', 'no', 'n'):
        if not panel.confirm:
            return None
        verb = 'confirm' if token in ('confirm', 'yes', 'y') else 'cancel'
        return f'{verb}:{panel.confirm[0]}'
    return token or None


def _wait(controller: Controller, token: str, out) -> Optional[list]:
    """Scripted only: `wait:S` sleeps; `until-ready:S` runs status until READY."""
    kind, _, secs = token.partition(':')
    deadline = time.monotonic() + float(secs)
    if kind == 'wait':
        out.write(f'> {token}\n')
        time.sleep(float(secs))
        return None
    n = 0
    while True:
        n += 1
        panel = controller.handle('status')
        st = panel.status or {}
        if st.get('ready') or time.monotonic() >= deadline:
            out.write(f'> {token}  ({n} status calls, READY {bool(st.get("ready"))})\n')
            return show(panel, out)
        time.sleep(5.0)


class PlannerChat:
    """The Telegram text flow for the terminal: ask, then send and follow."""

    def __init__(self, controller: Controller, out, sleep=time.sleep):
        self.c = controller
        self.out = out
        self.sleep = sleep
        self.confirms = Confirmations(controller.cfg.max_command_age_s, controller.clock)
        self.nonce = None

    @staticmethod
    def wants(token: str) -> bool:
        return token.startswith(('say:', 'say '))

    def ask(self, token: str) -> None:
        text, why = planner.validate(token[4:])
        if why:
            self.out.write(f'  ❌ not sent to the planner: {why}\n')
            return
        res = self.c.t.ctl(['planner-log'], self.c.cfg.timeout('status'))
        if res.get('transport_error') or not res.get('ok'):
            self.out.write(f'  ❌ cannot reach the planner: {res.get("error")}\n')
            return
        if not res.get('running'):
            panel = self.c.handle('status')
            self.c.note('📝 planner not running: tap ▶️ Planner, then send the command again')
            show(panel, self.out)
            return
        self.nonce = self.confirms.ask(text)
        self.out.write(f'  Send to planner: «{text}»? [confirm / cancel]\n')

    def answer(self, yes: bool) -> None:
        nonce, self.nonce = self.nonce, None
        if not yes:
            self.confirms.cancel(nonce)
            self.out.write('  ❌ not sent to the planner\n')
            return
        text, why = self.confirms.take(nonce)
        if text is None:
            self.out.write(f'  ⌛ not sent: {why}\n')
            return
        ex = planner.Exchange(text)
        if ex.send(self.c.t, self.c.cfg.timeout('default')):
            while not ex.done:
                self.sleep(planner.POLL_S)
                ex.poll(self.c.t, self.c.cfg.timeout('status'))
        self.out.write(ex.render(as_html=False) + '\n')


def run(controller: Controller, actions: Optional[Iterable[str]] = None,
        out=sys.stdout, inp=input) -> int:
    chat = PlannerChat(controller, out)
    if actions:
        flat = show(controller.panel, out)
        for token in actions:
            if token.startswith(('wait:', 'until-ready:')):
                flat = _wait(controller, token, out) or flat
                continue
            if PlannerChat.wants(token):
                out.write(f'> {token}\n')
                chat.ask(token)
                continue
            if chat.nonce and token in ('confirm', 'yes', 'y', 'cancel', 'no', 'n'):
                out.write(f'> {token}\n')
                chat.answer(token in ('confirm', 'yes', 'y'))
                continue
            action = resolve(token, controller.panel, flat)
            out.write(f'> {token}' + (f'  ({action})' if action != token else '') + '\n')
            if action is None:
                out.write('  (nothing to do)\n')
                continue
            flat = show(controller.handle(action), out)
        return 0
    flat = show(controller.handle('refresh'), out)
    while True:
        try:
            token = inp('action> ')
        except (EOFError, KeyboardInterrupt):
            out.write('\n')
            return 0
        if token.strip() in ('q', 'quit', 'exit'):
            return 0
        if PlannerChat.wants(token):
            chat.ask(token)
            continue
        if chat.nonce and token.strip() in ('confirm', 'yes', 'y', 'cancel', 'no', 'n'):
            chat.answer(token.strip() in ('confirm', 'yes', 'y'))
            continue
        action = resolve(token, controller.panel, flat)
        if action is None:
            out.write('  ? type a button number, an action name or q\n')
            continue
        flat = show(controller.handle(action), out)
