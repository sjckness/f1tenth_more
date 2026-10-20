"""
The Fast DDS discovery server as its own unit (f1t-discovery.service).

The stack's launch reuses a server that already listens on the port
(ensure_discovery_server.py), so a server started here first outlives every
stack start and stop. A server found listening that is not in the unit (the
stack started it, or someone did by hand) is reported as unmanaged and left
alone by `down` unless --force.
"""

import os

from . import procscan, units
from .config import ProcSpec, Settings

NAME = 'discovery'


def spec(settings: Settings) -> ProcSpec:
    addr = settings.discovery_address
    host, port = addr if addr else ('127.0.0.1', 11811)
    # bash's ENOEXEC fallback runs Humble's shebang-less `fastdds` script.
    cmd = (f'fastdds discovery --server-id {settings.discovery_id} '
           f'--udp-address {host} --udp-port {port}')
    return ProcSpec(name=NAME, cmd=cmd, unit_prefix=settings.unit_prefix)


def status(settings: Settings) -> dict:
    addr = settings.discovery_address
    if addr is None:
        return {'state': 'disabled', 'configured': False,
                'note': 'ROS_DISCOVERY_SERVER is not set in the env file'}
    host, port = addr
    unit = units.unit_states(settings, [NAME])[NAME]
    inodes = procscan.udp_inodes(host, port)
    listening = bool(inodes)
    info = {'configured': True, 'address': f'{host}:{port}', 'listening': listening,
            'unit': unit.get('unit')}
    if unit['state'] == 'running':
        info.update(state='running', managed=True, pid=unit.get('pid'))
        if not listening:
            info['state'] = 'starting'
    elif listening:
        info.update(state='running', managed=False)
        pids = procscan.pids_with_inodes(inodes)
        if pids:
            info['pid'] = pids[0]
            p = procscan.read_proc(pids[0])
            if p:
                info['cmdline'] = p.cmdline[:200]
    else:
        info.update(state=unit['state'], managed=unit['state'] != 'stopped')
        for key in ('result', 'exit_status', 'signal'):
            if key in unit:
                info[key] = unit[key]
    return info


def up(settings: Settings) -> dict:
    before = status(settings)
    if before['state'] == 'disabled':
        return {'ok': True, **before}
    if before['state'] in ('running', 'starting'):
        return {'ok': True, 'already': True, **before}
    result = units.start(settings, spec(settings), settle_s=1.0)
    after = status(settings)
    ok = result.get('ok', False) and after.get('listening', False)
    out = {'ok': ok, **after}
    if not ok:
        out['error'] = result.get('error') or 'the server did not start listening'
        out['log_tail'] = units.log_tail(settings, NAME)
    return out


def down(settings: Settings, force: bool = False) -> dict:
    before = status(settings)
    if before['state'] == 'disabled':
        return {'ok': True, **before}
    stopped = units.stop(settings, spec(settings))
    after = status(settings)
    out = {'ok': stopped.get('ok', False), 'signals': stopped.get('signals', []), **after}
    if after.get('listening') and not after.get('managed', False):
        if force:
            pids = [p.pid for p in procscan.all_procs()
                    if p.uid == os.getuid() and procscan.is_discovery(p)]
            pids += list(procscan.discovery_children(procscan.all_procs(), set(pids)))
            ladder = procscan.signal_ladder(pids, 3.0, 2.0)
            out['forced'] = ladder
            out.update(status(settings))
            out['ok'] = not out.get('listening')
        else:
            out['ok'] = False
            out['error'] = ('a discovery server not started by f1tenth_ctl is listening '
                            '(the stack, or by hand): left running, use --force')
    return out
