"""
f1tenth_ctl: start, stop and check the car's ROS 2 processes.

Runs on the car (or on linus against the sim). Every command prints one JSON
object on stdout (--json, the default, for the Telegram bot) or a short text
(--human). Exit status: 0 ok, 1 the command failed, 2 bad usage or config.

    f1tenth_ctl connect                      discovery up + environment check
    f1tenth_ctl discovery up|down|status
    f1tenth_ctl start|stop stack|planner|logger
    f1tenth_ctl status                       processes + READY, in < 5 s
    f1tenth_ctl estop                        e-stop service + zero drive, never kills
    f1tenth_ctl estop-reset [--force]        clear the e-stop latch, and check it
    f1tenth_ctl killall [--reset-discovery] [--dry-run]
    f1tenth_ctl check                        environment check only
    f1tenth_ctl send planner "<command>"     one line to the planner's stdin (<= 500 chars)
    f1tenth_ctl planner-log [--since N]      the planner's output after byte offset N
    f1tenth_ctl missions [-n N]              the stored plans (one per planner RESULT)
    f1tenth_ctl mission-load <id>            validate + load a stored plan
    f1tenth_ctl mission-start                start it: READY, no e-stop, nothing running
    f1tenth_ctl mission-status               idle/loaded/starting/running/finished/aborted
    f1tenth_ctl logs stack|planner|logger|discovery [-n N]

The env file (--env, F1T_CTL_ENV, or config/ctl.env) configures everything;
see config.py.
"""

import argparse
from datetime import datetime
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time

from . import discovery, envcheck, human, missions, monitor, procscan, ready, units
from .config import ConfigError, load, MANAGED, Settings, SOURCE_ROOT

STATUS_BUDGET_S = 4.5      # status must answer in < 5 s, env sourcing included
NODE_CHECK_WINDOW_S = 1.5  # graph look before a start
GRAPH_GRACE_S = 30.0       # a stopped node may linger in the graph this long


# ---- helpers ------------------------------------------------------------

def _probe_env(settings: Settings) -> dict:
    env = dict(settings.env)
    env['PYTHONPATH'] = SOURCE_ROOT + (':' + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    # A plain discovery-server client only learns about endpoints matching its
    # own; listing the nodes needs a super client, whatever the stack uses.
    if settings.discovery_server:
        env['ROS_SUPER_CLIENT'] = 'TRUE'
    return env


def _start_rosprobe(settings: Settings, mode: str, spec: dict) -> subprocess.Popen:
    env = _probe_env(settings)
    env['F1T_PROBE_SPEC'] = json.dumps(spec)
    return subprocess.Popen(
        ['python3', '-m', 'f1tenth_ctl.rosprobe', mode], env=env,
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def _finish_rosprobe(proc: subprocess.Popen, timeout: float) -> dict:
    try:
        out, err = proc.communicate(timeout=max(0.1, timeout))
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()
        return {'error': f'ROS probe timed out after {timeout:.1f} s'}
    for line in reversed(out.strip().splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    tail = ' | '.join(err.strip().splitlines()[-3:])
    return {'error': f'ROS probe gave no answer (exit {proc.returncode}): {tail}'}


def run_rosprobe(settings: Settings, mode: str, spec: dict, timeout: float) -> dict:
    return _finish_rosprobe(_start_rosprobe(settings, mode, spec), timeout)


def _thresholds(settings: Settings) -> dict:
    return ready.load_thresholds(settings.ready_file)


def graph_nodes(settings: Settings, window: float = NODE_CHECK_WINDOW_S):
    """Node names in the ROS graph, or None if the graph cannot be read."""
    if not settings.graph_check:
        return None
    disc = discovery.status(settings)
    if disc['state'] != 'disabled' and not disc.get('listening'):
        return None
    try:
        thr = _thresholds(settings)
    except ready.ReadyConfigError:
        thr = None
    if thr is not None and settings.monitor_enabled:
        # A monitor that has been up a few seconds has the complete list; a
        # fresh participant's may miss nodes on a loaded machine.
        snap = monitor.read(settings, monitor.probe_spec(settings, thr)['spec_id'])
        if snap is not None and not snap['warming_up']:
            return snap['nodes']
    sim = bool(thr and thr.get('use_sim_time'))
    res = run_rosprobe(settings, 'probe',
                       {'checks': [], 'window_s': window, 'use_sim_time': sim, 'nodes': True},
                       timeout=window + 6.0)
    return res.get('nodes') if 'error' not in res else None


def _strays(settings: Settings, name: str, procs=None, excl=None):
    """Return the processes that look like `name` but are outside its unit."""
    spec = settings.procs[name]
    if not spec.match:
        return []
    procs = procscan.all_procs() if procs is None else procs
    excl = procscan.own_exclusions() if excl is None else excl
    excl = excl | set(units.unit_pids(spec.unit))
    return procscan.matching(procs, spec.match, os.getuid(), excl)


# ---- commands -----------------------------------------------------------

def cmd_discovery(settings: Settings, args) -> dict:
    if args.action == 'up':
        return discovery.up(settings)
    if args.action == 'down':
        mon = monitor.stop(settings)
        return {**discovery.down(settings, force=args.force), 'monitor_stopped': mon.get('ok')}
    return {'ok': True, **discovery.status(settings)}


def cmd_check(settings: Settings, args=None) -> dict:
    env = settings.env
    items = []

    def item(name, ok, value, required=True, why=None):
        d = {'name': name, 'ok': bool(ok), 'value': value, 'required': required}
        if why and not ok:
            d['why'] = why
        items.append(d)

    item('env_file', True, settings.env_file)
    item('ros_distro', bool(env.get('ROS_DISTRO')), env.get('ROS_DISTRO'),
         why='ROS_DISTRO not set: the env file did not source ROS')
    ros2 = shutil.which('ros2', path=env.get('PATH', ''))
    item('ros2_cli', bool(ros2), ros2, why='ros2 not on PATH')
    prefixes = [p for p in env.get('AMENT_PREFIX_PATH', '').split(':') if p]
    want = env.get('F1T_CHECK_PACKAGES', 'f1tenth_bringup').split()
    missing = [pkg for pkg in want
               if not any(os.path.isfile(os.path.join(p, 'share', pkg, 'package.xml'))
                          for p in prefixes)]
    item('workspace', not missing, ' '.join(want),
         why=f'packages not found: {" ".join(missing)}')
    item('rmw', True, env.get('RMW_IMPLEMENTATION') or 'default (rmw_fastrtps_cpp)')
    item('domain_id', True, env.get('ROS_DOMAIN_ID') or '0 (default)')
    disc = discovery.status(settings)
    if disc['state'] == 'disabled':
        item('discovery', True, 'not used')
    else:
        item('discovery', disc.get('listening'), disc.get('address'),
             why='discovery server not listening')
        # With a discovery server a plain client sees node names but no
        # topics (output/telegram/discovery.md): the app's own probes are
        # super clients anyway, a shell needs this plus `ros2 ... --no-daemon`.
        super_client = (env.get('ROS_SUPER_CLIENT') or '').upper() == 'TRUE'
        item('super_client', super_client, env.get('ROS_SUPER_CLIENT') or 'unset',
             required=False,
             why='ROS_SUPER_CLIENT is not TRUE: `ros2 topic list --no-daemon` from a '
                 'shell with this env file would list no topics')
    sysd = units.systemctl(['is-system-running'])
    state = sysd.stdout.strip()
    item('systemd_user', state in ('running', 'degraded'), state,
         why='systemd --user not reachable (no user session?)')
    linger = subprocess.run(['loginctl', 'show-user', env.get('USER', ''), '-p', 'Linger'],
                            capture_output=True, text=True, timeout=5).stdout.strip()
    item('linger', linger.endswith('yes'), linger or 'unknown', required=False,
         why='Linger off: units die when the last session closes')
    try:
        thr = _thresholds(settings)
        item('ready_file', True, f'{settings.ready_file} ({len(thr["checks"])} checks)')
    except ready.ReadyConfigError as exc:
        item('ready_file', False, settings.ready_file, why=str(exc))
    try:
        os.makedirs(settings.log_dir, exist_ok=True)
        item('state_dir', os.access(settings.state_dir, os.W_OK), settings.state_dir,
             why='state dir not writable')
    except OSError as exc:
        item('state_dir', False, settings.state_dir, why=exc.strerror)
    shell = envcheck.check(settings)
    if shell is not None:
        items.append(shell)
    failing = [i['name'] for i in items if i['required'] and not i['ok']]
    warnings = [i['name'] for i in items if not i['required'] and not i['ok']]
    return {'ok': not failing, 'failing': failing, 'warnings': warnings, 'items': items}


def cmd_connect(settings: Settings, args) -> dict:
    disc = discovery.up(settings)
    check = cmd_check(settings)
    ok = disc.get('ok', False) and check['ok']
    out = {'ok': ok, 'ready_to_launch': ok, 'discovery': disc, 'check': check,
           'failing': check['failing'] + ([] if disc.get('ok') else ['discovery'])}
    return out


def cmd_start(settings: Settings, args) -> dict:
    name = args.target
    spec = settings.procs[name]
    state = units.unit_states(settings, [name])[name]
    if state['state'] in ('running', 'stopping'):
        return {'ok': False, 'target': name, 'error': f'{name} is already running', **state}
    out = {'target': name}
    disc = discovery.status(settings)
    if disc['state'] != 'disabled' and not disc.get('listening'):
        # Started first so it outlives the stack (the launch reuses it).
        up = discovery.up(settings)
        out['discovery'] = 'started' if up.get('ok') else f'failed: {up.get("error")}'
        if not up.get('ok'):
            return {'ok': False, **out, 'error': 'discovery server did not start'}
    strays = _strays(settings, name)
    if strays:
        return {'ok': False, **out,
                'error': f'{name} is already running outside f1tenth_ctl',
                'pids': [p.pid for p in strays], 'cmdline': strays[0].cmdline[:200]}
    nodes = graph_nodes(settings)
    if nodes is None:
        out['graph_check'] = 'skipped (graph not readable)'
    else:
        present = [n for n in spec.nodes if n in nodes]
        if present:
            out['graph_check'] = f'{len(nodes)} nodes, {" ".join(present)} among them'
            return {'ok': False, **out, 'error': f'{name} already in the graph: '
                    f'{" ".join(present)} (started by hand, or by the stack)', 'nodes': present}
        out['graph_check'] = f'{len(nodes)} nodes, none of {" ".join(spec.nodes)}'
        if name != 'stack' and settings.procs['stack'].nodes and not any(
                n in nodes for n in settings.procs['stack'].nodes):
            out['warning'] = 'the stack is not running'
    result = units.start(settings, spec)
    if name == 'stack' and result.get('ok') and settings.monitor_enabled:
        # It warms up while the stack does: matching takes a few seconds.
        out['monitor'] = monitor.ensure(settings, _thresholds(settings)).get('state')
    return {**out, **result}


def cmd_stop(settings: Settings, args) -> dict:
    name = args.target
    spec = settings.procs[name]
    result = units.stop(settings, spec)
    out = {'target': name, **result}
    strays = _strays(settings, name)
    if strays:
        # Started by hand: the same ladder, on the matching processes.
        ladder = procscan.signal_ladder([p.pid for p in strays], settings.sigint_wait_s,
                                        settings.sigterm_wait_s)
        out['unmanaged'] = {'pids': [p.pid for p in strays], **ladder}
        out['ok'] = result.get('ok', False) and not ladder['survivors']
    if name == 'stack' and settings.monitor_enabled:
        monitor.stop(settings)
    out['state'] = units.unit_states(settings, [name])[name]['state']
    return out


def cmd_status(settings: Settings, args, t_begin: float) -> dict:
    try:
        thr = _thresholds(settings)
    except ready.ReadyConfigError as exc:
        return {'ok': False, 'error': str(exc)}
    disc = discovery.status(settings)
    discovery_ok = None if disc['state'] == 'disabled' else bool(disc.get('listening'))
    states = units.unit_states(settings, list(MANAGED))
    table = procscan.all_procs()
    excl = procscan.own_exclusions()
    strays = {n: _strays(settings, n, table, excl) for n in MANAGED}
    stack_up = states['stack']['state'] == 'running' or bool(strays['stack'])
    if discovery_ok is False:
        res = {'error': 'discovery server down: graph not probed'}
    elif not stack_up:
        # Nothing to measure, and no node is left behind on an idle car: the
        # monitor runs only while there is a stack.
        if settings.monitor_enabled and monitor.state(settings)['state'] == 'running':
            monitor.stop(settings)
        res = {'source': None, 'note': 'stack not running: not measured'}
    else:
        res = _measure(settings, thr, t_begin)
    nodes = res.get('nodes')
    measured = res.get('measured') if 'error' not in res else None
    procs = {'discovery': disc}
    for name in MANAGED:
        st = dict(states[name])
        if st['state'] != 'running':
            seen = unmanaged_evidence(settings.procs[name].nodes, nodes, strays[name],
                                      units.read_desired(settings, name), time.time())
            if seen:
                st = {'state': 'running', 'managed': False, 'seen': seen,
                      'previous': st['state']}
        procs[name] = st
    verdict = ready.evaluate(thr, measured, procs['stack']['state'] == 'running',
                             discovery_ok)
    est = next((measured.get(c['name']) for c in thr['checks']
                if c['kind'] == 'estop' and measured), None) or {}
    mission = (missions.describe_state(settings, est.get('state'), est.get('json_path'),
                                       countdown_s=settings.mission_countdown_s)
               if est.get('received') else None)
    out = {'ok': True, 'ready': verdict['ready'], 'failing': verdict['failing'],
           'estop_latched': ready.estop_latched(verdict['checks']), 'mission': mission,
           'processes': procs, 'vars': ready.key_values(verdict['checks']),
           'checks': verdict['checks'],
           'probe': {'ran': measured is not None, 'source': res.get('source'),
                     'nodes': len(nodes) if nodes is not None else None}}
    for key in ('age_s', 'warming_up', 'listened_s', 'error', 'note'):
        if key in res:
            out['probe'][key] = res[key]
    return out


def unmanaged_evidence(spec_nodes, nodes, strays, desired: dict, now: float) -> dict:
    """
    Say how a process not running under f1tenth_ctl still shows up.

    Matching processes are exact. Node names are not: the graph keeps a node
    for a while after its process is gone (until the discovery server drops
    the participant), so for GRAPH_GRACE_S after a stop they do not count.
    """
    seen = {}
    if strays:
        seen['pids'] = [p.pid for p in strays]
    present = [n for n in spec_nodes if nodes and n in nodes]
    recent = (desired.get('desired') == 'stopped'
              and now - desired.get('time', 0) < GRAPH_GRACE_S)
    if present and not recent:
        seen['nodes'] = present
    return seen


def _measure(settings: Settings, thr: dict, t_begin: float) -> dict:
    """READY measurements: the monitor's snapshot, else a one-shot probe."""
    if not settings.monitor_enabled:
        deadline = time.time() + STATUS_BUDGET_S - 0.4 - (time.monotonic() - t_begin)
        spec = {'checks': thr['checks'], 'window_s': thr['window_s'],
                'use_sim_time': thr['use_sim_time'], 'nodes': True, 'deadline': deadline}
        res = run_rosprobe(settings, 'probe', spec,
                           timeout=STATUS_BUDGET_S - (time.monotonic() - t_begin))
        return {**res, 'source': 'probe'}
    mon = monitor.ensure(settings, thr)
    spec_id = monitor.probe_spec(settings, thr)['spec_id']
    # A monitor that was just (re)started needs ~1-2 s for its first snapshot.
    budget = STATUS_BUDGET_S - 0.3 - (time.monotonic() - t_begin)
    snap = monitor.read(settings, spec_id, wait_s=max(0.0, budget) if mon.get('started')
                        else 0.0)
    if snap is None:
        # Running but no fresh snapshot yet (it was started moments ago, e.g.
        # by connect): it gets what is left of the budget.
        budget = STATUS_BUDGET_S - 0.3 - (time.monotonic() - t_begin)
        snap = monitor.read(settings, spec_id, wait_s=max(0.0, budget))
    if snap is None:
        return {'source': 'monitor', 'error': f'{monitor.why_not(settings)}: no snapshot'}
    return {'source': 'monitor', 'measured': snap['measured'], 'nodes': snap['nodes'],
            'age_s': snap['age_s'], 'warming_up': snap['warming_up']}


ESTOP_STEPS = ('zero_drive', 'emergency_stop', 'abort_mission')


def estop_result(steps: dict, error: str = None) -> dict:
    """
    Build estop's answer from its step records (pure).

    car_held = zero drive published AND (emergency_stop OK OR abort_mission OK).
    A step that never ran (no ROS, no discovery server) is `skipped`.
    """
    full = {}
    for name in ESTOP_STEPS:
        full[name] = steps.get(name) or {'status': 'skipped', 'ok': False,
                                         'message': error or 'not run', 'duration_ms': 0}
    held = full['zero_drive']['ok'] and (full['emergency_stop']['ok']
                                         or full['abort_mission']['ok'])
    out = {'ok': bool(held), 'car_held': bool(held), 'steps': full,
           'note': 'nothing was stopped or killed'}
    if error:
        out['error'] = error
    return out


def cmd_estop(settings: Settings, args) -> dict:
    disc = discovery.status(settings)
    if disc['state'] != 'disabled' and not disc.get('listening'):
        # Nothing on this graph can be reached: answer at once.
        return estop_result({}, 'discovery server down: nothing reachable')
    try:
        sim = bool(_thresholds(settings).get('use_sim_time'))
    except ready.ReadyConfigError:
        sim = False
    spec = {'topic': settings.estop_topic, 'service': settings.estop_service,
            'service_timeout_s': settings.estop_service_timeout_s,
            'abort_service': settings.abort_service,
            'abort_timeout_s': settings.estop_abort_timeout_s,
            'publish_s': settings.estop_publish_s, 'publish_hz': settings.estop_publish_hz,
            'use_sim_time': sim}
    # Match grace 3 s + publish window + 2 s for late answers + start-up.
    res = run_rosprobe(settings, 'estop', spec, timeout=settings.estop_publish_s + 10.0)
    if 'error' in res:
        return estop_result({}, res['error'])
    return estop_result(res.get('steps') or {})


def cmd_estop_reset(settings: Settings, args) -> dict:
    """
    Clear the e-stop latch by the cleanest path available, and check it.

    rosprobe.estop_reset does the ROS part (abort the mission, zero drive,
    reset service or restart of 'behavior', wait for the clear latch). When
    neither exists it answers needs=stack_restart, and this restarts the
    whole stack, if f1tenth_ctl manages it, then waits for the clear latch.
    """
    disc = discovery.status(settings)
    if disc['state'] != 'disabled' and not disc.get('listening'):
        return {'ok': False, 'error': 'discovery server down'}
    stack = units.unit_states(settings, ['stack'])['stack']
    managed = stack['state'] == 'running'
    if not managed and not _strays(settings, 'stack'):
        return {'ok': False, 'error': 'the stack is not running'}
    try:
        sim = bool(_thresholds(settings).get('use_sim_time'))
    except ready.ReadyConfigError:
        sim = False
    spec = {'status_topic': settings.status_topic, 'reset_service': settings.estop_reset_service,
            'restart_service': settings.restart_service,
            'component': settings.behavior_component, 'abort_service': settings.abort_service,
            'safety_topic': settings.estop_topic, 'publish_hz': settings.estop_publish_hz,
            'wait_s': settings.estop_reset_wait_s, 'use_sim_time': sim,
            'force': bool(getattr(args, 'force', False))}
    res = run_rosprobe(settings, 'estop_reset', spec,
                       timeout=settings.estop_reset_wait_s + 60.0)
    if res.get('needs') != 'stack_restart':
        return {'ok': bool(res.get('ok')), **res}
    if not managed:
        return {**res, 'ok': False,
                'error': 'only a full stack restart would clear the latch, and this stack '
                         'was not started by f1tenth_ctl: restart it by hand'}
    # No other way: the whole stack, as stop + start would do it by hand.
    out = {**res, 'path': 'stack_restart'}
    out['stop'] = units.stop(settings, settings.procs['stack'])
    if settings.monitor_enabled:
        monitor.stop(settings)
    start = units.start(settings, settings.procs['stack'])
    out['start'] = {k: start.get(k) for k in ('ok', 'state', 'pid', 'error') if k in start}
    if not start.get('ok'):
        return {**out, 'ok': False, 'error': 'the stack did not start again'}
    if settings.monitor_enabled:
        monitor.ensure(settings, _thresholds(settings))
    wait = run_rosprobe(settings, 'estop_wait',
                        {'status_topic': settings.status_topic, 'use_sim_time': sim,
                         'wait_s': 180.0}, timeout=200.0)
    out.update(latched_after=wait.get('latched_after'), verified=bool(wait.get('verified')),
               waited_s=wait.get('waited_s'))
    out['ok'] = out['verified']
    if out['ok']:
        out.pop('error', None)
    else:
        out['error'] = wait.get('error') or 'the latch did not clear after the restart'
    return out


def cmd_killall(settings: Settings, args) -> dict:
    uid = os.getuid()
    disc_before = discovery.status(settings)
    out = {'reset_discovery': args.reset_discovery}

    excl = procscan.own_exclusions()
    if args.dry_run:
        procs = procscan.ros_processes(procscan.all_procs(), uid, excl,
                                       include_discovery=args.reset_discovery)
        states = units.unit_states(settings, list(MANAGED))
        return {'ok': True, 'dry_run': True,
                'units': {n: s['state'] for n, s in states.items()},
                'would_signal': [{'pid': p.pid, 'cmdline': p.cmdline[:160]} for p in procs]}

    # 1. the managed launches, in parallel
    results = {}

    def stop_one(n):
        if n == monitor.NAME:
            results[n] = monitor.stop(settings)
        else:
            results[n] = units.stop(settings, settings.procs[n])

    threads = [threading.Thread(target=stop_one, args=(n,))
               for n in MANAGED + (monitor.NAME,)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    out['units'] = {n: {'ok': r.get('ok'), 'signals': r.get('signals'),
                        **({'already': r['already']} if 'already' in r else {})}
                    for n, r in results.items()}
    if args.reset_discovery:
        r = units.stop(settings, discovery.spec(settings))
        out['units']['discovery'] = {'ok': r.get('ok'), 'signals': r.get('signals')}

    # 2. every other ROS process of this user (the daemon is step 3's)
    def leftovers():
        procs = procscan.all_procs()
        found = procscan.ros_processes(procs, uid, excl,
                                       include_discovery=args.reset_discovery)
        found = [p for p in found if p.comm != '_ros2_daemon'
                 and 'ros2cli.daemon' not in p.cmdline]
        if args.reset_discovery:
            kids = procscan.discovery_children(
                procs, {p.pid for p in found if procscan.is_discovery(p)})
            found += [p for p in procs if p.pid in kids and p.pid not in excl]
        return found

    strays = leftovers()
    ladder = procscan.signal_ladder([p.pid for p in strays], settings.killall_sigint_wait_s,
                                    settings.killall_sigterm_wait_s)
    out['strays'] = {'count': len(strays),
                     'processes': [{'pid': p.pid, 'cmdline': p.cmdline[:160]} for p in strays],
                     **ladder}

    # 3. the ros2 daemon
    try:
        d = subprocess.run(['ros2', 'daemon', 'stop'], env=settings.env, capture_output=True,
                           text=True, timeout=10)
        out['daemon'] = (d.stdout.strip() or d.stderr.strip())[:200]
    except (subprocess.TimeoutExpired, OSError) as exc:
        out['daemon'] = f'ros2 daemon stop failed: {exc}'
    daemons = [p.pid for p in procscan.all_procs() if p.uid == uid
               and (p.comm == '_ros2_daemon' or 'ros2cli.daemon' in p.cmdline)]
    if daemons:
        procscan.signal_ladder(daemons, 1.0, 1.0)

    # 4. shared-memory segments nobody holds any more
    try:
        # Through bash: Humble's `fastdds` has no shebang (ENOEXEC from exec).
        shm = subprocess.run(['bash', '-c', 'fastdds shm clean'], env=settings.env,
                             capture_output=True, text=True, timeout=10)
        out['shm_clean'] = 'ok' if shm.returncode == 0 else shm.stderr.strip()[:160]
    except (subprocess.TimeoutExpired, OSError) as exc:
        out['shm_clean'] = f'skipped: {exc}'

    # 5. the discovery server stays up unless asked: if it went down with the
    #    stack's launch tree (the stack had started it), start it again.
    disc_after = discovery.status(settings)
    if (not args.reset_discovery and disc_before.get('listening')
            and not disc_after.get('listening')):
        up = discovery.up(settings)
        out['discovery_restarted'] = bool(up.get('ok'))
        disc_after = discovery.status(settings)
    out['discovery'] = disc_after

    remaining = leftovers()
    out['remaining'] = [{'pid': p.pid, 'cmdline': p.cmdline[:160]} for p in remaining]
    units_ok = all(r.get('ok') for r in out['units'].values())
    out['ok'] = units_ok and not remaining
    return out


def cmd_send(settings: Settings, args) -> dict:
    return {'target': args.target, **units.fifo_send(settings, args.target, ' '.join(args.text))}


RESULT_PREFIX = 'RESULT '     # llm_planner_node._emit_result: one line per command, last
PLANNER_PROMPT = 'comando> '  # its interactive prompt, glued to the next printed line


def planner_results(settings: Settings, lines, offsets) -> list:
    """
    Return the RESULT lines among `lines`, each mission stored (missions.py).

    Keyed by the line's byte offset in the planner log, so a RESULT read twice
    (two readers, or a repeated cursor) is stored once.
    """
    out = []
    for line, offset in zip(lines, offsets):
        while line.startswith(PLANNER_PROMPT):
            line = line[len(PLANNER_PROMPT):]
        if not line.startswith(RESULT_PREFIX):
            continue
        try:
            result = json.loads(line[len(RESULT_PREFIX):])
        except ValueError:
            out.append({'offset': offset, 'status': 'unparsed'})
            continue
        if not isinstance(result, dict):
            continue
        keep = ('status', 'stage', 'error_type', 'error_message', 'unsupported', 'notes',
                'mission_id', 'plan_seconds')
        entry = {'offset': offset}
        entry.update({k: result[k] for k in keep if k in result})
        rec = missions.store_result(settings, result, offset)
        if rec:
            entry['stored'] = rec
            entry['mission'] = result.get('mission')
        out.append(entry)
    return out


def cmd_planner_log(settings: Settings, args) -> dict:
    """Return the planner's output since a cursor (a byte offset of its log), ANSI stripped."""
    res = units.read_log_since(settings.log_path('planner'), args.since, args.max_bytes)
    running = units.is_running(settings, 'planner')
    out = {'ok': True, 'target': 'planner', 'since': args.since, 'running': running,
           'plan_only': planner_plan_only(settings) if running else None, **res}
    out['results'] = planner_results(settings, res['lines'], res['offsets'])
    return out


def planner_plan_only(settings: Settings) -> bool:
    """
    Say whether the running planner is plan-only (llm_planner_node --dry-run).

    Read from the command lines of the planner unit's processes, not from the
    env file: a planner started by an older f1tenth_ctl, without --dry-run,
    loads and starts every mission it plans. The bot refuses to plan unless
    this is true.
    """
    for pid in units.unit_pids(settings.procs['planner'].unit):
        p = procscan.read_proc(pid)
        if p and any('llm_planner_node' in a for a in p.argv) and '--dry-run' in p.argv:
            return True
    return False


def _stack_up(settings: Settings) -> bool:
    if units.unit_states(settings, ['stack'])['stack']['state'] == 'running':
        return True
    return bool(_strays(settings, 'stack'))


def _sim(settings: Settings) -> bool:
    try:
        return bool(_thresholds(settings).get('use_sim_time'))
    except ready.ReadyConfigError:
        return False


def _mission_spec(settings: Settings, **kw) -> dict:
    return {'status_topic': settings.status_topic, 'use_sim_time': _sim(settings),
            'discover_s': settings.mission_discover_s, **kw}


def _plan_age(settings: Settings, rec: dict, args):
    """Return a refusal for a plan older than the limit, else None."""
    age = time.time() - float(rec.get('stored_at', 0))
    if age > settings.mission_max_age_s and not getattr(args, 'allow_old', False):
        return (f'the plan is {age:.0f} s old (limit {settings.mission_max_age_s:g} s): '
                'plan it again')
    return None


def cmd_mission_load(settings: Settings, args) -> dict:
    rec = missions.get(settings, args.id)
    if rec is None:
        return {'ok': False, 'error': f'no stored mission {args.id!r} (f1tenth_ctl missions)'}
    out = {'id': rec['id'], 'mission_id': rec.get('mission_id'),
           'age_s': round(time.time() - rec['stored_at'], 1)}
    why = _plan_age(settings, rec, args)
    if why:
        return {**out, 'ok': False, 'refused': why, 'error': why}
    if not _stack_up(settings):
        return {**out, 'ok': False, 'refused': 'the stack is not running',
                'error': 'the stack is not running'}
    cur = missions.current(settings)
    armed = cur.get('countdown_s')
    armed = settings.mission_countdown_s if armed is None else float(armed)
    counting = bool(cur.get('started_at')) and (
        time.time() - cur['started_at'] < armed + 5.0)
    res = run_rosprobe(settings, 'mission',
                       _mission_spec(settings, op='load', path=rec['path'],
                                     counting_down=counting), timeout=25.0)
    out.update(res)
    if res.get('ok'):
        missions.set_current(settings, id=rec['id'], path=rec['path'], loaded_at=time.time(),
                             started_at=None)
    elif 'error' in res and 'refused' not in out:
        out['refused'] = res['error']
    out['ok'] = bool(res.get('ok'))
    return out


def cmd_mission_start(settings: Settings, args, t_begin: float) -> dict:
    """
    Start the mission f1tenth_ctl loaded, after the checks.

    Refused, with the reason, when: nothing was loaded by f1tenth_ctl, the id
    is not the loaded one, the plan is too old, READY is false (which covers a
    latched e-stop), or another mission is running.
    """
    cur = missions.current(settings)
    mid = args.id or cur.get('id')
    rec = missions.get(settings, mid) if mid else None

    def refuse(why, **extra):
        return {'ok': False, 'id': mid, 'refused': why, 'error': why, **extra}

    if rec is None:
        return refuse('no mission loaded by f1tenth_ctl (mission-load <id> first)')
    if cur.get('id') != rec['id']:
        return refuse(f"{rec['id']} is not the loaded mission ({cur.get('id')}): "
                      'mission-load it first')
    why = _plan_age(settings, rec, args)
    if why:
        return refuse(why)
    st = cmd_status(settings, args, t_begin)
    if st.get('estop_latched'):
        return refuse('the e-stop is latched (estop-reset first)', failing=st.get('failing'))
    if not st.get('ready'):
        return refuse('not READY: ' + ', '.join(st.get('failing') or ['unknown']),
                      failing=st.get('failing'))
    res = run_rosprobe(settings, 'mission',
                       _mission_spec(settings, op='start', path=rec['path']), timeout=25.0)
    countdown = missions.armed_countdown(res.get('start'), settings.mission_countdown_s)
    out = {'id': rec['id'], 'mission_id': rec.get('mission_id'), 'ready': True,
           'countdown_s': countdown, **res}
    if res.get('ok'):
        missions.set_current(settings, started_at=time.time(), countdown_s=countdown)
    else:
        out['refused'] = res.get('error') or 'failed'
    out['ok'] = bool(res.get('ok'))
    return out


def cmd_mission_status(settings: Settings, args) -> dict:
    res = run_rosprobe(settings, 'mission', _mission_spec(settings, op='status'), timeout=15.0)
    desc = missions.describe_state(settings, res.get('ros_state'), res.get('json_path'),
                                   countdown_s=settings.mission_countdown_s)
    out = {'ok': bool(res.get('ok')), **desc,
           'emergency_stop_active': res.get('emergency_stop_active')}
    if 'error' in res:
        out['error'] = res['error']
    return out


def cmd_missions(settings: Settings, args) -> dict:
    now = time.time()
    recs = missions.recent(settings, args.n)
    return {'ok': True, 'current': missions.current(settings),
            'missions': [{'id': r['id'], 'mission_id': r.get('mission_id'),
                          'age_s': round(now - r.get('stored_at', now), 1),
                          'startable': now - r.get('stored_at', 0) <= settings.mission_max_age_s,
                          'summary': r.get('summary')} for r in recs]}


def cmd_logs(settings: Settings, args) -> dict:
    lines = units.log_tail(settings, args.target, args.lines)
    return {'ok': True, 'target': args.target, 'log': settings.log_path(args.target),
            'lines': lines}


# ---- entry point --------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    fmt = common.add_mutually_exclusive_group()
    fmt.add_argument('--json', dest='fmt', action='store_const', const='json',
                     default=argparse.SUPPRESS, help='JSON on stdout (default)')
    fmt.add_argument('--human', dest='fmt', action='store_const', const='human',
                     default=argparse.SUPPRESS, help='short text instead of JSON')
    common.add_argument('--env', default=argparse.SUPPRESS,
                        help='env file (default: $F1T_CTL_ENV or config/ctl.env)')

    p = argparse.ArgumentParser(prog='f1tenth_ctl', parents=[common],
                                description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    sub.add_parser('connect', parents=[common], help='discovery up + environment check')
    d = sub.add_parser('discovery', parents=[common], help='the Fast DDS discovery server')
    d.add_argument('action', choices=('up', 'down', 'status'))
    d.add_argument('--force', action='store_true',
                   help='down: also stop a server f1tenth_ctl did not start')
    for verb in ('start', 'stop'):
        s = sub.add_parser(verb, parents=[common], help=f'{verb} a managed process')
        s.add_argument('target', choices=MANAGED)
    sub.add_parser('status', parents=[common], help='processes and READY (< 5 s)')
    sub.add_parser('estop', parents=[common],
                   help='e-stop service + zero drive on the top mux lane; never kills')
    r = sub.add_parser('estop-reset', parents=[common],
                       help='clear the e-stop latch (abort + restart behavior) and verify')
    r.add_argument('--force', action='store_true',
                   help='run the reset even if the latch reads clear')
    k = sub.add_parser('killall', parents=[common], help='stop everything ROS 2')
    k.add_argument('--reset-discovery', action='store_true',
                   help='also stop the discovery server')
    k.add_argument('--dry-run', action='store_true', help='only list what would be signalled')
    sub.add_parser('check', parents=[common], help='environment check')
    s = sub.add_parser('send', parents=[common], help="one line to a process's stdin")
    s.add_argument('target', choices=('planner',))
    s.add_argument('text', nargs='+', help=f'one line, at most {units.MAX_LINE} characters')
    pl = sub.add_parser('planner-log', parents=[common],
                        help="the planner's output since a cursor (byte offset of its log)")
    pl.add_argument('--since', type=int, default=None,
                    help='byte offset from `send` or a previous planner-log; '
                         'none: only the current end')
    pl.add_argument('--max-bytes', type=int, default=65536)
    ml = sub.add_parser('mission-load', parents=[common],
                        help='validate and load a stored mission (from `missions`)')
    ml.add_argument('id')
    ml.add_argument('--allow-old', action='store_true',
                    help='load a plan older than F1T_MISSION_MAX_AGE_S')
    ms = sub.add_parser('mission-start', parents=[common],
                        help='start the loaded mission (READY, no e-stop, nothing running)')
    ms.add_argument('--id', help='refuse unless this is the loaded mission')
    ms.add_argument('--allow-old', action='store_true')
    sub.add_parser('mission-status', parents=[common],
                   help='idle / loaded / starting / running / finished / aborted, id, elapsed')
    mi = sub.add_parser('missions', parents=[common], help='the last stored plans')
    mi.add_argument('-n', type=int, default=10)
    lg = sub.add_parser('logs', parents=[common], help='tail of a process log')
    lg.add_argument('target', choices=MANAGED + ('discovery', 'monitor'))
    lg.add_argument('-n', '--lines', type=int, default=30)
    return p


COMMANDS = {
    'connect': cmd_connect, 'discovery': cmd_discovery, 'start': cmd_start,
    'stop': cmd_stop, 'estop': cmd_estop, 'estop-reset': cmd_estop_reset,
    'killall': cmd_killall, 'check': cmd_check,
    'send': cmd_send, 'logs': cmd_logs, 'planner-log': cmd_planner_log,
    'mission-load': cmd_mission_load, 'mission-status': cmd_mission_status,
    'missions': cmd_missions,
}


def main(argv=None) -> int:
    t_begin = time.monotonic()
    args = build_parser().parse_args(argv)
    fmt = getattr(args, 'fmt', 'json')
    envelope = {'cmd': args.cmd, 'host': socket.gethostname(),
                'time': datetime.now().astimezone().isoformat(timespec='seconds')}
    for key in ('action', 'target'):
        if getattr(args, key, None):
            envelope[key] = getattr(args, key)
    code = 0
    try:
        settings = load(getattr(args, 'env', None))
        envelope['env_file'] = settings.env_file
        if args.cmd == 'status':
            result = cmd_status(settings, args, t_begin)
        elif args.cmd == 'mission-start':
            result = cmd_mission_start(settings, args, t_begin)
        else:
            result = COMMANDS[args.cmd](settings, args)
        if not result.get('ok', False):
            code = 1
    except (ConfigError, ready.ReadyConfigError) as exc:
        result = {'ok': False, 'error': str(exc)}
        code = 2
    except Exception as exc:  # noqa: BLE001 - the bot needs JSON, not a traceback
        result = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
        code = 1
    out = {**envelope, **result, 'elapsed_s': round(time.monotonic() - t_begin, 2)}
    out['ok'] = bool(result.get('ok', False))
    if fmt == 'human':
        sys.stdout.write(human.render(out) + '\n')
    else:
        sys.stdout.write(json.dumps(out, default=str) + '\n')
    sys.stdout.flush()
    return code


if __name__ == '__main__':
    sys.exit(main())
