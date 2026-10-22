"""link_manager on a private graph: real UDP to a loopback responder, a fake nmcli on disk."""

import importlib.util
import math
import os
import socket
import stat
import threading
import time

import pytest
import yaml

PKG = os.path.join(os.path.dirname(__file__), '..')
RESPONDER = os.path.join(PKG, '..', '..', 'scripts', 'base', 'heartbeat_responder.py')
HEADER = ('Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n'
          ' face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n')
FAKE_NMCLI = r"""#!/usr/bin/env python3
import sys
args = sys.argv[1:]
with open(__file__ + '.log', 'a') as f:
    f.write(' '.join(args) + '\n')
if 'device' in args and 'show' in args:
    print('GENERAL.STATE:30 (disconnected)')
    print('GENERAL.CONNECTION:')
elif 'connection' in args and 'show' in args:
    print('802-11-wireless.ssid:f1tenth-base')
elif 'up' in args:
    print('Connection successfully activated')
"""


def test_every_yaml_key_is_a_declared_parameter_of_the_right_type(ros_context):
    from rclpy.parameter import Parameter
    from f1tenth_comms.link_manager_node import LinkManagerNode
    with open(os.path.join(PKG, 'config', 'comms.yaml')) as f:
        params = yaml.safe_load(f)['link_manager']['ros__parameters']
    node = LinkManagerNode(context=ros_context, parameter_overrides=[
        Parameter(k, value=v) for k, v in params.items()])
    try:
        for k, v in params.items():
            assert node.has_parameter(k), k
            assert node.get_parameter(k).value == v, k
    finally:
        node.destroy_node()


def _spin_until(executor, pred, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        executor.spin_once(timeout_sec=0.05)
        if pred():
            return True
    return False


def test_ok_on_local_then_lost_then_a_rate_limited_reconnect(ros_context, tmp_path):
    if not os.path.exists(RESPONDER):
        pytest.skip('scripts/base/heartbeat_responder.py is not next to this package')
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from f1tenth_comms.link_manager_node import LinkManagerNode
    from f1tenth_messages.msg import LinkStatus

    spec = importlib.util.spec_from_file_location('heartbeat_responder', RESPONDER)
    resp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(resp)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
    silent = {'on': False}
    stop = {'on': False}
    socks = resp.open_sockets(['127.0.0.2'], port)      # only the "hotspot" answers
    th = threading.Thread(target=resp.serve, daemon=True, kwargs={
        'socks': socks, 'log_every': 0, 'stop': lambda: stop['on'],
        'reply': lambda d: None if silent['on'] else resp.reply_for(d)})
    th.start()

    nmcli = tmp_path / 'nmcli'
    nmcli.write_text(FAKE_NMCLI)
    nmcli.chmod(nmcli.stat().st_mode | stat.S_IEXEC)
    proc = tmp_path / 'wireless'
    proc.write_text(HEADER + 'fakewlan0: 0000   50.  -70.  -256   0  0  0  0  0  0\n')
    topic = '/test_comms/link_status'
    node = LinkManagerNode(context=ros_context, parameter_overrides=[
        Parameter('tailscale_host', value='127.0.0.1'),
        Parameter('local_host', value='127.0.0.2'),
        Parameter('port', value=port),
        Parameter('period_s', value=0.1),
        Parameter('timeout_s', value=0.5),
        Parameter('degraded_silence_s', value=0.35),
        Parameter('wifi_interface', value='fakewlan0'),
        Parameter('proc_net_wireless', value=str(proc)),
        Parameter('use_iw_fallback', value=False),
        Parameter('status_topic', value=topic),
        Parameter('coverage_topic', value='/test_comms/coverage_rssi'),
        Parameter('nmcli', value=str(nmcli)),
        Parameter('retry_period_s', value=0.5),
        Parameter('retry_min_interval_s', value=0.5),
        Parameter('retry_distance_m', value=0.0),
    ])
    peer = Node('link_test_peer', context=ros_context)
    got = []
    peer.create_subscription(LinkStatus, topic, got.append, 10)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    executor.add_node(peer)
    try:
        assert _spin_until(executor, lambda: got and got[-1].state_name == 'OK', 5.0)
        m = got[-1]
        assert (m.path, m.address) == ('local', f'127.0.0.2:{port}')
        assert m.path_names == ['tailscale', 'local'] and m.path_up == [False, True]
        assert m.rtt_ms < 50.0 and m.wifi_associated and m.rssi_dbm == -70.0
        assert m.reconnect_attempts == 0 and math.isinf(m.since_last_reconnect_s)

        silent['on'] = True
        assert _spin_until(executor, lambda: got[-1].state == LinkStatus.STATE_LOST, 5.0)
        assert got[-1].path == '' and math.isnan(got[-1].rtt_ms)
        assert _spin_until(executor, lambda: got[-1].last_reconnect_result == 'ok', 10.0)
        last = got[-1]
        assert last.last_reconnect_trigger == 'time' and last.reconnect_attempts >= 1

        silent['on'] = False
        # Back on the local path; DEGRADED until the outage's lost rounds leave the window.
        assert _spin_until(executor, lambda: got[-1].path == 'local', 5.0)
        assert got[-1].state_name in ('OK', 'DEGRADED')
        n_attempts = got[-1].reconnect_attempts
        _spin_until(executor, lambda: False, 1.0)
        assert got[-1].reconnect_attempts == n_attempts      # none while the link is up
    finally:
        executor.shutdown()
        peer.destroy_node()
        node.destroy_node()
        stop['on'] = True
        th.join(timeout=2.0)
        for s in socks:
            s.close()
    ups = [line for line in open(str(nmcli) + '.log').read().splitlines() if ' up ' in line]
    assert ups and ups[0] == '--wait 15 connection up id f1tenth-base ifname fakewlan0'
    assert len(ups) == n_attempts
