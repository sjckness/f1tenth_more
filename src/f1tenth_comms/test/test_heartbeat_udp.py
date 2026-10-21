"""
HeartbeatClient against the real base responder (scripts/base) over loopback UDP.

Tailscale is 127.0.0.1, the hotspot 127.0.0.2, one port: the same shape as
the car's two paths, without leaving this host.
"""

import importlib.util
import os
import socket
import threading
import time

from f1tenth_comms.heartbeat import (encode, HeartbeatClient, LinkMonitor, STATE_LOST,
                                     STATE_OK)
import pytest

RESPONDER = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'scripts', 'base',
                         'heartbeat_responder.py')


@pytest.fixture
def responder_mod():
    if not os.path.exists(RESPONDER):
        pytest.skip('scripts/base/heartbeat_responder.py is not next to this package')
    spec = importlib.util.spec_from_file_location('heartbeat_responder', RESPONDER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Responder:
    """The base's responder on some addresses, in a thread; drop() makes it silent."""

    def __init__(self, mod, addrs, port):
        self.mod = mod
        self.silent = False
        self.stop = False
        self.socks = mod.open_sockets(addrs, port)
        self.thread = threading.Thread(
            target=mod.serve, daemon=True,
            kwargs={'socks': self.socks, 'log_every': 0, 'stop': lambda: self.stop,
                    'reply': self._reply})
        self.thread.start()

    def _reply(self, data):
        return None if self.silent else self.mod.reply_for(data)

    def close(self):
        self.stop = True
        self.thread.join(timeout=2.0)
        for s in self.socks:
            s.close()


def test_responder_echoes_only_heartbeats(responder_mod):
    m = responder_mod
    pkt = encode(0, 1, 2, 3)
    assert m.reply_for(pkt) == pkt
    assert m.reply_for(b'GET / HTTP/1.1\r\n\r\n') is None
    assert m.reply_for(b'F1HB' + b'\x00' * 100) is None     # too long: no reflector
    assert m.reply_for(b'F1HB') is None


def wait_for(pred, timeout=5.0, step=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(step)
    return False


def test_two_paths_fallback_and_back_over_udp(responder_mod):
    port = free_port()
    base_ts = Responder(responder_mod, ['127.0.0.1'], port)
    base_local = Responder(responder_mod, ['127.0.0.2'], port)
    mon = LinkMonitor(['tailscale', 'local'], start_t=time.monotonic(), timeout_s=0.6,
                      window_s=3.0, degraded_silence_s=0.4, failback_s=0.6)
    client = HeartbeatClient([('tailscale', '127.0.0.1', port), ('local', '127.0.0.2', port)],
                             mon)
    client.start()
    stop = threading.Event()

    def sender():
        while not stop.is_set():
            client.send_round()
            stop.wait(0.1)

    t = threading.Thread(target=sender, daemon=True)
    t.start()
    try:
        assert wait_for(lambda: client.evaluate().active == 0 and
                        client.evaluate().state == STATE_OK)
        v = client.evaluate()
        assert v.rtt_ms is not None and v.rtt_ms < 50.0
        assert all(p.up for p in v.paths)

        base_ts.silent = True             # Tailscale gone: the local path takes over
        assert wait_for(lambda: client.evaluate().active == 1)
        assert client.evaluate().state == STATE_OK

        base_local.silent = True          # both gone: LOST
        assert wait_for(lambda: client.evaluate().state == STATE_LOST)

        base_ts.silent = base_local.silent = False
        assert wait_for(lambda: client.evaluate().active == 0 and
                        client.evaluate().state == STATE_OK)
        v = client.evaluate()
        assert v.rounds_lost > 0 and v.rounds_answered > 0
        assert client.bad_packets == 0
    finally:
        stop.set()
        t.join(timeout=1.0)
        client.stop()
        base_ts.close()
        base_local.close()


def test_replies_of_another_run_are_dropped(responder_mod):
    port = free_port()
    base = Responder(responder_mod, ['127.0.0.1'], port)
    mon = LinkMonitor(['tailscale'], start_t=time.monotonic())
    client = HeartbeatClient([('tailscale', '127.0.0.1', port)], mon)
    client.start()
    try:
        # A stale packet of an earlier link_manager (other nonce) echoed to us.
        client.sock.sendto(encode(0, client.nonce ^ 1, 1, 0), ('127.0.0.1', port))
        assert wait_for(lambda: client.bad_packets == 1)
        assert client.evaluate().rounds_answered == 0
    finally:
        client.stop()
        base.close()


def test_unresolvable_name_counts_as_send_error_without_blocking():
    mon = LinkMonitor(['tailscale'], start_t=time.monotonic())
    client = HeartbeatClient([('tailscale', 'no-such-host.invalid', 47800)], mon,
                             resolve_period_s=60.0)
    client.start()
    try:
        t0 = time.monotonic()
        for _ in range(3):
            client.send_round()
        assert time.monotonic() - t0 < 0.1
        assert client.send_errors == [3]
    finally:
        client.stop()
