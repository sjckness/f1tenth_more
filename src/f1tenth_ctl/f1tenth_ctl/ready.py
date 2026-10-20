"""
READY: thresholds (config/ready.yaml) applied to what the probe measured.

Pure functions only: the probe (rosprobe.py) measures, this module judges, so
the decision is unit-testable without ROS. Check kinds:

- topic_rate: messages per second over the probe window >= min_hz
- tf_age:     latest parent->child transform no older than max_age_s
- service:    the service is available
- estop_clear: the mission status was received and the e-stop is not latched
- battery:    has_data and voltage >= min_voltage

A check with `required: false` is measured and shown but never blocks READY.
Besides the checks, READY needs the stack running and, when the env file
sets ROS_DISCOVERY_SERVER, the discovery server listening.
"""

from typing import Dict, List, Optional

KINDS = ('topic_rate', 'tf_age', 'service', 'estop_clear', 'battery')


class ReadyConfigError(Exception):
    pass


def load_thresholds(path: str) -> dict:
    import yaml
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except OSError as exc:
        raise ReadyConfigError(f'cannot read {path}: {exc.strerror}') from exc
    return validate(data, path)


def validate(data: dict, origin: str = 'thresholds') -> dict:
    if not isinstance(data, dict):
        raise ReadyConfigError(f'{origin}: expected a mapping')
    checks = data.get('checks') or []
    names = set()
    for i, check in enumerate(checks):
        name = check.get('name')
        kind = check.get('kind')
        if not name:
            raise ReadyConfigError(f'{origin}: check #{i} has no name')
        if name in names:
            raise ReadyConfigError(f'{origin}: duplicate check {name!r}')
        names.add(name)
        if kind not in KINDS:
            raise ReadyConfigError(f'{origin}: check {name!r}: unknown kind {kind!r}')
        need = {'topic_rate': ('topic', 'type'), 'tf_age': ('parent', 'child'),
                'service': ('service', 'type'), 'estop_clear': ('topic', 'type'),
                'battery': ('topic', 'type')}[kind]
        for key in need:
            if not check.get(key):
                raise ReadyConfigError(f'{origin}: check {name!r} needs {key!r}')
    data.setdefault('window_s', 2.0)
    data.setdefault('use_sim_time', False)
    data['checks'] = checks
    return data


def _r(x: Optional[float], nd: int = 2) -> Optional[float]:
    return None if x is None else round(float(x), nd)


def judge(check: dict, m: Optional[dict]) -> dict:
    """Judge one check against its measurement: {name, ok, value, want}."""
    kind = check['kind']
    out = {'name': check['name'], 'kind': kind,
           'required': bool(check.get('required', True))}
    if m is None:
        out.update(ok=False, value=None, want='measured', why='not measured')
        return out
    if 'error' in m and kind != 'tf_age':
        out.update(ok=False, value=None, want='measured', why=m['error'])
        return out
    if kind == 'topic_rate':
        hz = m.get('hz', 0.0) or 0.0
        want = float(check.get('min_hz', 0.0))
        out.update(ok=hz >= want and (want == 0 or m.get('count', 0) >= 2),
                   value=_r(hz, 1), unit='Hz', want=f'>= {want:g} Hz', topic=check['topic'])
        if not out['ok']:
            out['why'] = 'no messages' if not m.get('count') else f'{hz:.1f} Hz'
    elif kind == 'tf_age':
        want = float(check.get('max_age_s', 0.5))
        age = m.get('age_s')
        ok = bool(m.get('found')) and age is not None and age <= want
        out.update(ok=ok, value=_r(age, 3), unit='s',
                   want=f'<= {want:g} s', frames=f"{check['parent']}->{check['child']}")
        if not ok:
            out['why'] = m.get('error') or ('no transform' if not m.get('found')
                                            else f'{age:.2f} s old')
    elif kind == 'service':
        ok = bool(m.get('available'))
        out.update(ok=ok, value=ok, want='available', service=check['service'])
        if not ok:
            out['why'] = 'not available'
    elif kind == 'estop_clear':
        received = bool(m.get('received'))
        latched = bool(m.get('emergency_stop_active'))
        out.update(ok=received and not latched, value=latched, want='not latched',
                   mission_state=m.get('state'))
        if not received:
            out['why'] = f"no {check['topic']}"
        elif latched:
            out['why'] = 'e-stop latched (restart behavior to clear)'
    elif kind == 'battery':
        want = float(check.get('min_voltage', 0.0))
        volts = m.get('voltage')
        has = bool(m.get('received')) and bool(m.get('has_data'))
        ok = has and volts is not None and volts >= want
        out.update(ok=ok, value=_r(volts, 2) if has else None, unit='V',
                   want=f'>= {want:g} V')
        if not ok:
            out['why'] = ('no battery data' if not has else f'{volts:.2f} V')
    return out


def evaluate(thresholds: dict, measured: Optional[Dict[str, dict]],
             stack_running: bool, discovery_ok: Optional[bool]) -> dict:
    """
    Decide READY from the thresholds, the measurements and the processes.

    `measured` is None when the probe did not run. `discovery_ok` is None when
    no discovery server is configured.
    """
    checks: List[dict] = []
    if discovery_ok is not None:
        checks.append({'name': 'discovery', 'kind': 'process', 'required': True,
                       'ok': bool(discovery_ok), 'value': bool(discovery_ok),
                       'want': 'listening',
                       **({} if discovery_ok else {'why': 'discovery server down'})})
    checks.append({'name': 'stack', 'kind': 'process', 'required': True,
                   'ok': bool(stack_running), 'value': bool(stack_running),
                   'want': 'running',
                   **({} if stack_running else {'why': 'stack not running'})})
    for check in thresholds.get('checks', []):
        m = None if measured is None else measured.get(check['name'])
        checks.append(judge(check, m))
    failing = [c['name'] for c in checks if c['required'] and not c['ok']]
    return {'ready': not failing, 'failing': failing, 'checks': checks}


def key_values(checks: List[dict]) -> Dict[str, object]:
    """Return the short {name: value} map the bot shows under READY."""
    out = {}
    for c in checks:
        if c['kind'] in ('process',):
            continue
        out[c['name']] = c.get('value')
    return out
