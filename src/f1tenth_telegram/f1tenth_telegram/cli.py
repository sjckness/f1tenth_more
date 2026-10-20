"""
--cli: the same panel and handlers in a terminal, without Telegram.

Interactive: the panel is printed with numbered buttons; type a number, an
action name (refresh, connect, start:stack, ...) or q. Scripted: each
--do ACTION runs in order and the panel is printed after each one; `confirm`
and `cancel` answer the open confirmation, `wait:S` pauses S seconds and
`until-ready:S` repeats status until READY (at most S seconds).
"""

import sys
import time
from typing import Iterable, Optional

from . import view
from .controller import Controller, Panel


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


def run(controller: Controller, actions: Optional[Iterable[str]] = None,
        out=sys.stdout, inp=input) -> int:
    if actions:
        flat = show(controller.panel, out)
        for token in actions:
            if token.startswith(('wait:', 'until-ready:')):
                flat = _wait(controller, token, out) or flat
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
        action = resolve(token, controller.panel, flat)
        if action is None:
            out.write('  ? type a button number, an action name or q\n')
            continue
        flat = show(controller.handle(action), out)
