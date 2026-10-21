"""
--cli: the same panel and handlers in a terminal, without Telegram.

Interactive: the panel is printed with numbered buttons; type a number, an
action name (refresh, connect, start:stack, ...) or q. Scripted: each
--do ACTION runs in order and the panel is printed after each one; `confirm`
and `cancel` answer the open confirmation, `wait:S` pauses S seconds and
`until-ready:S` repeats status until READY (at most S seconds).

`say:<text>` (or `say <text>`) is a plain Telegram message, `voice:<file>`
a voice message (transcribed here). Then the same two steps as Telegram:
`plan` (or `cancel`) shows the planned mission, `start` (or `discard`) asks
"🚗 The car will drive. Start?", `yes` (or `no`) loads and starts it.
"""

import os
import sys
import time
from typing import Iterable, Optional

from . import planner, view
from .controller import Controller, Panel
from .flow import Flow, when_it_moves


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
    """
    The Telegram planner flow in a terminal (flow.py), same steps and checks.

        say:<text> | voice:<file>   then  plan | cancel
        (a plan)                    then  start | discard
        🚗 The car will drive. Start?  then  yes | no
    """

    WORDS = ('plan', 'cancel', 'start', 'discard', 'yes', 'no')

    def __init__(self, controller: Controller, out, sleep=time.sleep, transcriber=None):
        self.c = controller
        self.out = out
        self.sleep = sleep
        self.flow = Flow(controller.t, controller.cfg, controller.clock)
        self.transcriber = transcriber
        self.ask_nonce = None
        self.offer = None
        self.start_nonce = None

    @staticmethod
    def wants(token: str) -> bool:
        return token.startswith(('say:', 'say ', 'voice:', 'voice '))

    def pending(self, token: str) -> bool:
        word = token.strip()
        return word in self.WORDS and bool(
            {'plan': self.ask_nonce, 'cancel': self.ask_nonce, 'start': self.offer,
             'discard': self.offer, 'yes': self.start_nonce, 'no': self.start_nonce}[word])

    def handle(self, token: str) -> None:
        if token.startswith(('voice:', 'voice ')):
            text = self._transcribe(token[6:].strip())
            if text is None:
                return
        elif self.wants(token):
            text = token[4:]
        else:
            {'plan': self.plan, 'cancel': self.cancel, 'start': self.ask_start,
             'discard': self.discard, 'yes': self.start, 'no': self.no}[token.strip()]()
            return
        r = self.flow.propose(text)
        if r['kind'] == 'ask':
            self.ask_nonce = r['nonce']
            self.out.write(f'  Plan: «{r["text"]}»? [plan / cancel]\n')
        elif r['kind'] == 'not_running':
            panel = self.c.handle('status')
            self.c.note('📝 planner not running: tap ▶️ Planner, then send the command again')
            show(panel, self.out)
        else:
            self.out.write(f'  ❌ not planned: {r.get("why")}\n')

    def _transcribe(self, path: str):
        if self.transcriber is None:
            from .transcribe import Transcriber
            self.transcriber = Transcriber(self.c.cfg.transcription)
            self.transcriber.start(background=False)
        why = self.transcriber.unavailable_message()
        if why:
            self.out.write(f'  🎙 {why}\n')
            return None
        try:
            with open(path, 'rb') as f:
                data = f.read()
        except OSError as exc:
            self.out.write(f'  🎙 cannot read {path}: {exc.strerror}\n')
            return None

        def write(p):
            with open(p, 'wb') as f:
                f.write(data)
        res = self.transcriber.transcribe_bytes(write, os.path.splitext(path)[1] or '.ogg')
        why = self.transcriber.too_long(res['duration_s'])
        if why:
            self.out.write(f'  🎙 {why}\n')
            return None
        self.out.write(f"  🎙 «{res['text']}» ({res['duration_s']} s of audio, "
                       f"transcribed in {res['transcribe_s']} s; wrong? type it)\n")
        return res['text']

    def cancel(self) -> None:
        self.flow.cancel_plan(self.ask_nonce)
        self.ask_nonce = None
        self.out.write('  ❌ not planned\n')

    def plan(self) -> None:
        nonce, self.ask_nonce = self.ask_nonce, None
        ex, why = self.flow.begin_plan(nonce)
        if ex is None:
            self.out.write(f'  ⌛ not planned: {why}\n')
            return
        try:
            while not ex.done:
                self.sleep(planner.POLL_S)
                ex.poll(self.c.t, self.c.cfg.timeout('status'))
        finally:
            self.offer = self.flow.finish_plan(ex)
        if self.offer is None:
            self.out.write(planner.render_failure(ex, as_html=False) + '\n')
            return
        self.out.write(planner.render_offer(self.offer.text, self.offer.entry, as_html=False)
                       + '\n  [start / discard]\n')

    def discard(self) -> None:
        self.flow.discard(self.offer.token)
        self.offer = None
        self.out.write('  ❌ plan discarded\n')

    def ask_start(self) -> None:
        r = self.flow.offer_start(self.offer.token)
        if r['kind'] != 'ask':
            self.offer = None
            self.out.write(f'  ❌ {r["why"]}\n')
            return
        self.start_nonce = r['nonce']
        self.out.write('  🚗 The car will drive. Start? [yes / no]\n')

    def no(self) -> None:
        self.flow.cancel_start(self.start_nonce)
        self.start_nonce = None
        self.out.write('  ❌ not started\n')

    def start(self) -> None:
        nonce, self.start_nonce = self.start_nonce, None
        res = self.flow.start(nonce)
        if res['ok']:
            self.offer = None
            self.out.write(f'  🚗 mission {res["id"]} started: '
                           f'{when_it_moves(res.get("countdown_s"))}\n')
        else:
            self.out.write(f'  ❌ not started: {res["why"]}\n')


def run(controller: Controller, actions: Optional[Iterable[str]] = None,
        out=sys.stdout, inp=input) -> int:
    chat = PlannerChat(controller, out)
    if actions:
        flat = show(controller.panel, out)
        for token in actions:
            if token.startswith(('wait:', 'until-ready:')):
                flat = _wait(controller, token, out) or flat
                continue
            if PlannerChat.wants(token) or chat.pending(token):
                out.write(f'> {token}\n')
                chat.handle(token)
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
        if PlannerChat.wants(token) or chat.pending(token):
            chat.handle(token)
            continue
        action = resolve(token, controller.panel, flat)
        if action is None:
            out.write('  ? type a button number, an action name or q\n')
            continue
        flat = show(controller.handle(action), out)
