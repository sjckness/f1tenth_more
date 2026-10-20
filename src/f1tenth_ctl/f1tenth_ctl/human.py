"""--human: the JSON result as a few lines of text."""

import json

_ICON = {'running': 'RUNNING', 'stopped': 'stopped', 'crashed': 'CRASHED',
         'stopping': 'stopping', 'starting': 'starting', 'disabled': 'disabled'}


def _proc_line(name: str, st: dict) -> str:
    state = _ICON.get(st.get('state', '?'), st.get('state', '?'))
    bits = []
    if st.get('managed') is False and st.get('state') == 'running':
        bits.append('NOT started by f1tenth_ctl')
    if st.get('pid'):
        bits.append(f"pid {st['pid']}")
    if st.get('address'):
        bits.append(st['address'])
    if st.get('result'):
        bits.append(str(st['result']))
    if 'exit_status' in st:
        bits.append(f"exit {st['exit_status']}")
    if st.get('signal'):
        bits.append(st['signal'])
    seen = st.get('seen') or {}
    if seen.get('nodes'):
        bits.append('nodes ' + ' '.join(seen['nodes']))
    return f'  {name:<10} {state:<9} ' + ', '.join(bits)


def _status(out: dict) -> str:
    lines = [f"READY: {'YES' if out.get('ready') else 'NO'}"
             + ('' if out.get('ready') else f"  (failing: {', '.join(out.get('failing', []))})")]
    if out.get('estop_latched'):
        lines.append('E-STOP LATCHED: missions refused until `f1tenth_ctl estop-reset`')
    lines.append('processes:')
    for name, st in out.get('processes', {}).items():
        lines.append(_proc_line(name, st))
    lines.append('checks:')
    for c in out.get('checks', []):
        if c['kind'] == 'process':
            continue
        mark = 'ok ' if c['ok'] else ('-- ' if not c['required'] else 'NO ')
        val = c.get('value')
        val = '' if val is None else f"{val}{(' ' + c['unit']) if c.get('unit') else ''}"
        why = f"  ({c['why']})" if c.get('why') and not c['ok'] else ''
        lines.append(f"  {mark}{c['name']:<18} {val:<12} want {c.get('want', '')}{why}")
    probe = out.get('probe', {})
    if probe.get('error'):
        lines.append(f"probe: {probe['error']}")
    lines.append(f"({out.get('elapsed_s')} s, {probe.get('nodes')} nodes)")
    return '\n'.join(lines)


def _items(items) -> str:
    lines = []
    for i in items:
        mark = 'ok ' if i['ok'] else ('-- ' if not i.get('required', True) else 'NO ')
        why = f"  ({i['why']})" if i.get('why') and not i['ok'] else ''
        lines.append(f"  {mark}{i['name']:<13} {i.get('value')}{why}")
        for d in i.get('differences', []):
            if 'only_env_file' in d:
                lines.append(f"       {d['key']}: only in env file {d['only_env_file']}, "
                             f"only in shell {d['only_shell']}"
                             + (' (same entries, other order)' if d.get('order_differs') else ''))
            else:
                lines.append(f"       {d['key']}: env file {d['env_file']!r}, "
                             f"shell {d['shell']!r}")
    return '\n'.join(lines)


def _killall(out: dict) -> str:
    lines = []
    for name, u in (out.get('units') or {}).items():
        lines.append(f"  unit {name:<9} {'ok' if u.get('ok') else 'FAILED'} "
                     f"{u.get('already') or ' '.join(u.get('signals') or [])}")
    strays = out.get('strays') or {}
    lines.append(f"  other ROS processes: {strays.get('count', 0)} "
                 f"({' '.join(strays.get('signals') or []) or 'none signalled'})")
    for p in strays.get('processes', [])[:10]:
        lines.append(f"    {p['pid']:>7} {p['cmdline'][:100]}")
    lines.append(f"  ros2 daemon: {out.get('daemon')}")
    lines.append(f"  shm clean: {out.get('shm_clean')}")
    disc = out.get('discovery') or {}
    lines.append(f"  discovery: {disc.get('state')}"
                 + (' (restarted)' if out.get('discovery_restarted') else ''))
    for p in out.get('remaining', []):
        lines.append(f"  STILL RUNNING {p['pid']} {p['cmdline'][:100]}")
    return '\n'.join(lines)


def render(out: dict) -> str:
    head = f"{out.get('cmd')}{' ' + out['target'] if out.get('target') else ''}" \
           f"{' ' + out['action'] if out.get('action') else ''}: " \
           f"{'OK' if out.get('ok') else 'FAILED'}"
    if out.get('error'):
        head += f" - {out['error']}"
    if out.get('cmd') == 'status' and 'processes' in out:
        return head + '\n' + _status(out)
    if out.get('cmd') == 'check' and 'items' in out:
        return head + '\n' + _items(out['items'])
    if out.get('cmd') == 'connect' and 'check' in out:
        disc = out.get('discovery') or {}
        return (head + f"  ready to launch: {'yes' if out.get('ready_to_launch') else 'no'}\n"
                + f"  discovery: {disc.get('state')} {disc.get('address', '')}\n"
                + _items(out['check'].get('items', [])))
    if out.get('cmd') == 'estop-reset' and 'path' in out:
        lines = [f"  path: {out.get('path')}",
                 f"  latched before: {out.get('latched_before')}, "
                 f"after: {out.get('latched_after')}, verified: {out.get('verified')}"]
        for key in ('abort', 'restart', 'reset', 'stop', 'start', 'zero_drive'):
            if key in out:
                lines.append(f'  {key}: {json.dumps(out[key], default=str)}')
        return head + '\n' + '\n'.join(lines)
    if out.get('cmd') == 'killall' and 'units' in out:
        return head + '\n' + _killall(out)
    body = {k: v for k, v in out.items()
            if k not in ('cmd', 'host', 'time', 'env_file', 'ok', 'error', 'target', 'action')}
    if out.get('cmd') == 'logs':
        return head + '\n' + '\n'.join(out.get('lines', []))
    return head + '\n' + json.dumps(body, indent=2, default=str)
