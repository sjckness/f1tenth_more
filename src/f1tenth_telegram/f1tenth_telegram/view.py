"""
The panel as text and buttons, the same for Telegram and the terminal.

Buttons, by state:
- offline (or not checked yet): 🔄 only;
- online, not connected: 🔌 Connect, 🔄;
- connected: ▶️/⏹ Stack, ▶️/⏹ Planner, ▶️/⏹ Logger, 🔄 Status, 📝 Command
  (explains that a plain message is a planner command), ☠️ Kill all ROS 2,
  🛑 EMERGENCY, and 🔓 Reset e-stop while it is latched; 🛑 moves to the
  first row while a mission is starting or running;
- a confirmation open: ✅ yes / ✖ cancel, with 🛑 EMERGENCY still there.
"""

from dataclasses import dataclass
import time
from typing import List, Tuple

from .controller import Panel, PROCS

STATE_ICON = {'running': '🟢', 'stopped': '⚪', 'crashed': '🔴', 'stopping': '🟡',
              'starting': '🟡', 'disabled': '⚪'}


@dataclass
class Button:
    label: str
    action: str


def _proc(name: str, st: dict) -> str:
    state = st.get('state', '?')
    if state == 'running' and st.get('managed') is False:
        return f'{name} 🟠 running (not started by the bot)'
    text = f'{name} {STATE_ICON.get(state, "❔")} {state}'
    if state == 'crashed':
        if 'exit_status' in st:
            text += f' (exit {st["exit_status"]})'
        elif st.get('signal'):
            text += f' ({st["signal"]})'
    return text


def _fmt(value, unit='', nd=1):
    if value is None:
        return '–'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if isinstance(value, float):
        return f'{value:.{nd}f}{unit}'
    return f'{value}{unit}'


def _vars(status: dict) -> str:
    checks = {c['name']: c for c in status.get('checks', [])}
    parts = []
    for name, label, unit, nd in (('odometry', 'odom', ' Hz', 0), ('scan', 'scan', ' Hz', 0),
                                  ('tf_map_base_link', 'TF', ' s', 2),
                                  ('battery', 'battery', ' V', 1)):
        if name in checks:
            c = checks[name]
            mark = '' if c['ok'] or not c['required'] else '❗'
            value = c.get('value')
            if name == 'tf_map_base_link' and isinstance(value, float):
                # Under sim time the clock can trail the newest stamp by a tick.
                value = max(value, 0.0)
            parts.append(f'{label} {_fmt(value, unit, nd)}{mark}')
    est = checks.get('estop_latched')
    if est and est.get('mission_state'):
        parts.append(f'mission {est["mission_state"]}')
    return ' · '.join(parts)


MISSION_ICON = {'loaded': '📦', 'starting': '⏳', 'running': '🚗', 'finished': '🏁',
                'aborted': '⛔'}


def mission_line(m) -> str:
    """Return the current mission's line: id, state, elapsed; nothing when idle."""
    if not m or m.get('state') in (None, 'idle', 'unknown'):
        return ''
    icon = MISSION_ICON.get(m['state'], '🧭')
    who = m.get('id') or 'not started by the bot'
    elapsed = f", {m['elapsed_s']:.0f} s" if m.get('elapsed_s') is not None else ''
    return f"{icon} mission {who}: {m['state']}{elapsed}"


def mission_active(panel: Panel) -> bool:
    m = (panel.status or {}).get('mission') or {}
    return m.get('state') in ('starting', 'running')


def render(panel: Panel) -> Tuple[str, List[List[Button]]]:
    lines = []
    if panel.online is None:
        lines.append(f'🏎 F1Tenth · {panel.target} ❔ not checked yet')
    elif panel.online:
        lines.append(f'🏎 F1Tenth · {panel.target} 🟢 online')
    else:
        why = (panel.reach or {}).get('error') or 'no ssh'
        lines.append(f'🏎 F1Tenth · {panel.target} 🔴 offline ({why})')

    st = panel.status
    if panel.connected:
        disc = ((st or {}).get('processes') or {}).get('discovery') or {}
        lines.append(f'🔌 connected · discovery {_fmt(disc.get("state"))}')
    if panel.connected and st and 'processes' in st:
        failing = st.get('failing') or []
        if st.get('ready'):
            ready = '✅'
        elif 'stack' in failing:
            ready = '❌ stack not running'
        else:
            ready = '❌ ' + ', '.join(failing)
        lines.append(f'READY {ready}')
        procs = st['processes']
        lines.append(' · '.join(_proc(n, procs.get(n, {})) for n in PROCS))
        v = _vars(st)
        if v:
            lines.append(v)
        if st.get('estop_latched'):
            lines.append('🔴 E-STOP LATCHED: no mission will start until 🔓 Reset e-stop')
        ml = mission_line(st.get('mission'))
        if ml:
            lines.append(ml)
        probe = st.get('probe') or {}
        if probe.get('error'):
            lines.append(f'probe: {probe["error"]}')
    if panel.last:
        lines.append(f'Last: {panel.last}')
    if panel.busy:
        lines.append(f'⏳ {panel.busy} …')
    if panel.updated:
        lines.append(time.strftime('updated %H:%M:%S', time.localtime(panel.updated)))
    return '\n'.join(lines), buttons(panel)


def buttons(panel: Panel) -> List[List[Button]]:
    estop = [Button('🛑 EMERGENCY', 'estop')]
    if panel.confirm:
        nonce, action = panel.confirm
        return [[Button('✅ Yes, ' + action.replace(':', ' '), f'confirm:{nonce}'),
                 Button('✖ Cancel', f'cancel:{nonce}')], estop]
    if not panel.online:
        return [[Button('🔄', 'refresh')]]
    if not panel.connected:
        return [[Button('🔌 Connect', 'connect'), Button('🔄', 'refresh')]]
    procs = ((panel.status or {}).get('processes') or {})
    row = []
    for name in PROCS:
        running = procs.get(name, {}).get('state') in ('running', 'stopping')
        row.append(Button(f'⏹ {name.capitalize()}' if running else f'▶️ {name.capitalize()}',
                          f'stop:{name}' if running else f'start:{name}'))
    rows = [row, [Button('🔄 Status', 'status'), Button('📝 Command', 'help:command'),
                  Button('☠️ Kill all ROS 2', 'killall')]]
    if (panel.status or {}).get('estop_latched'):
        rows.append([Button('🔓 Reset e-stop', 'estop-reset')])
    if mission_active(panel):
        # While the car may drive, 🛑 is the first thing under the thumb.
        return [estop] + rows
    return rows + [estop]
