"""
The gateway end to end on the pretend car (mock.py): REST, WebSocket, the rules.

FastAPI's TestClient runs the real app (its event loop, the background loops,
the mock's 50 Hz publish thread); time is real, so the timing tests have a
margin. Every command the gateway would publish on /teleop and /safety_stop
is in MockWorld.published.
"""

import contextlib
import os
import time

from f1tenth_gateway import config
from f1tenth_gateway.__main__ import build
import pytest

TOKEN = 'k' * 40
H = {'Authorization': f'Bearer {TOKEN}'}
CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'gateway_mock.yaml')


@pytest.fixture
def gw(monkeypatch):
    # Skipped here, not at module level: under launch_testing a module-level
    # importorskip skips the whole directory (CLAUDE.md).
    pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    monkeypatch.setenv('F1T_GATEWAY_TOKEN', TOKEN)
    cfg = config.load(CONFIG, {'client': {'lost_after_s': 1.5}})
    gateway, app = build(cfg, True)
    gateway.world.countdown_s = 1.0
    gateway.world.move_s = 1.0
    with TestClient(app) as client:
        gateway.client = client
        wait(lambda: gateway.status.get('ready'))
        yield gateway


def wait(cond, timeout=5.0, step=0.05):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(step)
    return False


def job(gw, resp):
    assert resp.status_code == 202, resp.text
    jid = resp.json()['id']
    assert wait(lambda: gw.client.get(f'/api/jobs/{jid}', headers=H).json()['state'] != 'running',
                timeout=15)
    return gw.client.get(f'/api/jobs/{jid}', headers=H).json()


def plan(gw, text='vai dritto per due metri'):
    j = job(gw, gw.client.post('/api/mission/plan', headers=H, json={'text': text}))
    assert j['state'] == 'done', j
    return j['result']


@contextlib.contextmanager
def connect(gw):
    """Open an authenticated WebSocket, closed whatever the test does (else teardown hangs)."""
    with gw.client.websocket_connect('/ws') as ws:
        ws.send_json({'type': 'auth', 'token': TOKEN})
        assert ws.receive_json()['type'] == 'hello'
        yield ws


def recv_until(ws, kind, pred=lambda m: True, timeout=8.0):
    """Read until a matching message; ping now and then, as the app does (liveness)."""
    end, last_ping, seen = time.time() + timeout, 0.0, []
    while time.time() < end:
        m = ws.receive_json()
        if m.get('type') == kind and pred(m):
            return m
        seen.append(m.get('type'))
        if time.time() - last_ping > 0.4:
            ws.send_json({'type': 'ping', 'id': 0, 't': 0})
            last_ping = time.time()
    raise AssertionError(f'no {kind} in {timeout} s; saw {seen[-10:]}')


def teleop_published(gw, since):
    return [p for p in gw.world.published if p[1] == '/teleop' and p[0] >= since]


# ---- auth -------------------------------------------------------------------------------------

def test_everything_needs_the_token(gw):
    c = gw.client
    assert c.get('/api/status').status_code == 401
    assert c.get('/api/status', headers={'Authorization': 'Bearer nope'}).status_code == 401
    assert c.get('/api/status', headers=H).status_code == 200
    with c.websocket_connect('/ws') as ws:
        ws.send_json({'type': 'auth', 'token': 'nope'})
        with pytest.raises(Exception):
            ws.receive_json()


def test_hello_says_this_is_the_pretend_car(gw):
    with gw.client.websocket_connect('/ws') as ws:
        ws.send_json({'type': 'auth', 'token': TOKEN})
        hello = ws.receive_json()
        assert hello['type'] == 'hello' and hello['mock'] is True


def ws_refused(gw, send=None, timeout=8.0):
    """Open /ws, optionally send one message, and return the close code."""
    from starlette.websockets import WebSocketDisconnect
    with pytest.raises(WebSocketDisconnect) as exc:
        with gw.client.websocket_connect('/ws') as ws:
            if send is not None:
                ws.send_json(send)
            ws.receive_json()
    return exc.value.code


def test_a_handshake_that_times_out_is_not_an_auth_failure(gw):
    """
    A client that does not authenticate in time gets 4408, never 4401.

    The app stops reconnecting on 4401 and goes back to the pairing screen
    (ws.ts), so answering a TRANSPORT failure with 4401 wedged it offline
    until a reload -- no mission status, EMERGENZA disabled -- with a token
    that was never wrong. iOS holds a backgrounded PWA's timers, so a resume
    easily takes longer than ws_auth_timeout_s (2 s) to send its auth.
    """
    assert ws_refused(gw) == 4408


def test_an_unreadable_auth_message_is_not_an_auth_failure(gw):
    assert ws_refused(gw, send={'type': 'hello'}) == 4400


def test_only_a_rejected_token_gets_4401(gw):
    assert ws_refused(gw, send={'type': 'auth', 'token': 'nope'}) == 4401


def test_too_many_failures_are_refused_for_a_minute(gw):
    for _ in range(5):
        gw.client.get('/api/status', headers={'Authorization': 'Bearer nope'})
    assert gw.client.get('/api/status', headers=H).status_code == 429


def test_a_session_cookie_for_media(gw):
    c = gw.client
    assert c.post('/api/session', headers=H).status_code == 200
    rid = '2026-10-19T17-02-00_mission-wall_turn'
    assert c.get(f'/api/runs/{rid}/video').status_code == 200        # cookie only


def test_security_headers(gw):
    r = gw.client.get('/api/status', headers=H)
    assert "frame-ancestors 'none'" in r.headers['content-security-policy']
    assert r.headers['cache-control'] == 'no-store'


# ---- the e-stop's reach -----------------------------------------------------------------------

def test_status_says_whether_the_estop_can_be_reached(gw):
    """
    /api/status carries the e-stop's reach, so the app can say so explicitly.

    Without it the app had nothing to go on and gated EMERGENZA on its
    WebSocket instead -- which the e-stop does not use (/api/estop is REST),
    so the button went away exactly when the link was shaky and stayed
    offered when the mux was not listening at all.
    """
    c = gw.client
    assert c.get('/api/status', headers=H).json()['estop'] == {
        'available': True, 'why': None,
        'zero_subscribers': 1, 'service_ready': True, 'abort_ready': True}

    c.post('/api/_mock/set', headers=H, json={'mux_listens': False})
    e = c.get('/api/status', headers=H).json()['estop']
    assert e['available'] is False and '/safety_stop' in e['why']

    c.post('/api/_mock/set', headers=H, json={'mux_listens': True,
                                              'estop_service_ready': False})
    e = c.get('/api/status', headers=H).json()['estop']
    assert e['available'] is False and '/mission/emergency_stop' in e['why']


def test_the_estop_reach_fails_open_when_ros_cannot_answer(gw, monkeypatch):
    def boom():
        raise RuntimeError('no graph')
    monkeypatch.setattr(gw.ros, 'estop_graph', boom)
    e = gw.estop_reach()
    assert e['available'] is True and 'RuntimeError' in e['unknown']


# ---- status and processes ---------------------------------------------------------------------

def test_status_has_ready_checks_and_processes(gw):
    s = gw.client.get('/api/status', headers=H).json()
    assert s['ready'] and s['checks_ok'] == s['checks_total'] == 6
    assert s['processes']['stack']['state'] == 'running'
    assert s['processes']['stack']['nodes'] == 63 and s['processes']['stack']['uptime_s'] > 0
    gw.client.post('/api/_mock/set', headers=H, json={'failing': ['scan']})
    s = gw.client.get('/api/status', headers=H).json()
    assert not s['ready'] and s['failing'] == ['scan']


def test_a_process_switch_shows_its_transition_and_never_retries(gw):
    r = gw.client.post('/api/process/logger', headers=H, json={'action': 'start'})
    assert wait(lambda: gw.client.get('/api/status', headers=H).json()['processes']['logger']
                .get('pending') == 'starting', timeout=2)
    assert gw.client.post('/api/process/logger', headers=H,
                          json={'action': 'start'}).status_code == 409
    j = job(gw, r)
    assert j['state'] == 'done'
    assert gw.client.get('/api/status', headers=H).json()['processes']['logger']['state'] == \
        'running'


def test_the_stack_cannot_be_stopped_while_the_car_moves(gw):
    gw.client.post('/api/_mock/set', headers=H, json={'v': 0.5})
    gw.world.cmd = (0.5, 0.0, 'app')
    gw.world.teleop_t = time.monotonic() + 100
    r = gw.client.post('/api/process/stack', headers=H, json={'action': 'stop'})
    assert r.status_code == 409 and 'ferma' in r.json()['detail']


# ---- mission ----------------------------------------------------------------------------------

def test_plan_then_start_then_moves_then_complete(gw):
    p = plan(gw)
    assert p['valid'] and p['summary']['n_moves'] == 3 and len(p['steps']) == 3
    assert p['json']['moves'][0]['id'] == 'straight_1'
    with connect(gw) as ws:
        r = gw.client.post('/api/mission/start', headers=H, json={'plan_id': p['plan_id']})
        assert r.status_code == 200 and r.json()['countdown_s'] == 1.0
        m = recv_until(ws, 'telemetry', lambda m: m['mission']['state'] == 'starting')
        assert m['mission']['countdown_left_s'] <= 1.0 and m['mission']['started_by_app']
        recv_until(ws, 'telemetry', lambda m: m['mission'].get('move_index') == 2)
        done = recv_until(ws, 'telemetry', lambda m: m['mission']['state'] == 'finished')
        assert done['mission']['move_index'] == 3


def test_a_failed_plan_has_its_reason(gw):
    j = job(gw, gw.client.post('/api/mission/plan', headers=H, json={'text': 'errore'}))
    assert j['state'] == 'failed' and j['result']['valid'] is False
    assert 'EmptyPlanError' in j['result']['errors'][0]


@pytest.mark.parametrize('case', ['unknown', 'old', 'latched', 'guida'])
def test_start_refusals(gw, case):
    p = plan(gw)
    pid = p['plan_id']
    if case == 'unknown':
        pid = 'nope'
    elif case == 'old':
        gw.plans[pid]['created'] -= 301
    elif case == 'latched':
        gw.client.post('/api/_mock/set', headers=H, json={'estop_latched': True})
    elif case == 'guida':
        gw.teleop.acquire('someone', time.monotonic())
    r = gw.client.post('/api/mission/start', headers=H, json={'plan_id': pid})
    assert r.status_code in (404, 409)
    assert gw.world.ros_state == 'IDLE', 'nothing was loaded or started'


def test_abort_is_not_the_estop(gw):
    p = plan(gw)
    gw.client.post('/api/mission/start', headers=H, json={'plan_id': p['plan_id']})
    r = gw.client.post('/api/mission/abort', headers=H)
    assert r.status_code == 200 and gw.world.ros_state == 'ABORTED'
    assert not gw.world.estop_latched


def test_lost_clients_abort_an_app_mission_after_2_s(gw):
    gw.world.move_s = 30.0
    p = plan(gw)
    with connect(gw) as ws:
        gw.client.post('/api/mission/start', headers=H, json={'plan_id': p['plan_id']})
        recv_until(ws, 'telemetry', lambda m: m['mission']['state'] == 'running')
    t0 = time.time()
    assert wait(lambda: gw.world.ros_state == 'ABORTED', timeout=5)
    assert 1.8 <= time.time() - t0 <= 3.5
    assert not gw.world.estop_latched
    assert any('nessun client' in e['text'] for e in gw.events)


# ---- e-stop -----------------------------------------------------------------------------------

def test_estop_latches_and_publishes_on_the_safety_lane(gw):
    t0 = time.monotonic()
    r = gw.client.post('/api/estop', headers=H)
    assert r.status_code == 200 and r.json()['car_held'] and r.json()['path'] == 'gateway'
    assert gw.world.estop_latched
    assert any(p[1] == '/safety_stop' and p[0] >= t0 for p in gw.world.published)


def test_ready_follows_the_latch_before_the_checks_refresh(gw, monkeypatch):
    # f1tenth_ctl's checks are only refreshed every status_period_s: hold them stale.
    monkeypatch.setattr(gw, 'refresh_status', lambda: None)
    assert gw.status_payload()['ready']
    gw.client.post('/api/estop', headers=H)
    assert wait(lambda: gw.estop_latched())
    st = gw.status_payload()
    assert not st['ready'] and 'estop_latched' in st['failing']
    check = next(c for c in st['checks'] if c['kind'] == 'estop')
    assert check['ok'] is False and check['value'] is True and check['why'] == 'e-stop latched'
    stale = next(c for c in gw.status['checks'] if c['kind'] == 'estop')
    assert stale['ok'] is True                              # the stale one, untouched


def test_a_latch_change_is_pushed_at_once(gw, monkeypatch):
    monkeypatch.setattr(gw, 'refresh_status', lambda: None)
    gw.cfg['client']['status_period_s'] = 60.0
    time.sleep(1.3)                             # the status loop is now in its long sleep
    with connect(gw) as ws:
        recv_until(ws, 'status', lambda m: m['ready'])
        gw.client.post('/api/estop', headers=H)
        t0 = time.time()
        m = recv_until(ws, 'status', lambda m: not m['ready'], timeout=2.0)
        assert m['estop_latched'] and time.time() - t0 < 1.0


def test_reset_only_with_the_car_still(gw):
    gw.client.post('/api/estop', headers=H)
    gw.world.v = 0.3
    gw.world.cmd = (0.3, 0.0, 'x')
    gw.world.teleop_t = time.monotonic() + 100
    time.sleep(0.3)
    r = gw.client.post('/api/estop/reset', headers=H)
    assert r.status_code == 409
    gw.world.teleop_t = 0.0
    gw.world.v = 0.0
    assert wait(lambda: gw.reset_allowed()['ok'], timeout=4)
    j = job(gw, gw.client.post('/api/estop/reset', headers=H))
    assert j['state'] == 'done' and not gw.world.estop_latched


def test_no_reset_without_a_latch(gw):
    assert gw.client.post('/api/estop/reset', headers=H).status_code == 409


# ---- transcription ----------------------------------------------------------------------------

def test_transcription_and_its_refusals(gw):
    files = {'audio': ('a.m4a', b'\x00' * 1000, 'audio/mp4')}
    r = gw.client.post('/api/mission/transcribe', headers=H, files=files)
    assert r.status_code == 200 and r.json()['text'].startswith('vai dritto')
    big = {'audio': ('a.m4a', b'\x00' * 2_000_001, 'audio/mp4')}
    assert gw.client.post('/api/mission/transcribe', headers=H, files=big).status_code == 413
    gw.teleop.acquire('x', time.monotonic())
    assert gw.client.post('/api/mission/transcribe', headers=H,
                          files=files).status_code == 409
    gw.teleop.release('x', time.monotonic())
    p = plan(gw)
    gw.client.post('/api/mission/start', headers=H, json={'plan_id': p['plan_id']})
    assert gw.client.post('/api/mission/transcribe', headers=H,
                          files=files).status_code == 409


# ---- runs -------------------------------------------------------------------------------------

def test_runs_report_and_a_ranged_video(gw):
    runs = gw.client.get('/api/runs?limit=3', headers=H).json()['runs']
    assert len(runs) == 3
    rid = '2026-10-19T17-02-00_mission-wall_turn'
    assert 'straight_1' in gw.client.get(f'/api/runs/{rid}/report', headers=H).text
    r = gw.client.get(f'/api/runs/{rid}/video', headers={**H, 'Range': 'bytes=0-99'})
    assert r.status_code == 206 and len(r.content) == 100
    assert r.headers['content-range'].startswith('bytes 0-99/')
    assert gw.client.get('/api/runs/../x/report', headers=H).status_code == 404


# ---- Guida: lease, mapping, watchdog ----------------------------------------------------------

def test_guida_lease_teleop_release_and_close(gw):
    with connect(gw) as ws:
        ws.send_json({'type': 'drive_acquire'})
        assert recv_until(ws, 'drive')['state'] == 'granted'
        with connect(gw) as other:
            other.send_json({'type': 'drive_acquire'})
            assert recv_until(other, 'drive')['state'] == 'busy'
        t0 = time.monotonic()
        for seq in range(1, 31):
            ws.send_json({'type': 'teleop', 'seq': seq, 'thr': 1.0, 'steer': 0.5,
                          'limit': 5.0, 'deadman': True})
            time.sleep(0.05)
        pubs = [p for p in teleop_published(gw, t0) if p[2] > 0]
        assert pubs and max(p[2] for p in pubs) <= 1.0 and min(p[2] for p in pubs) >= 0.4
        assert max(abs(p[3]) for p in pubs) <= 0.30
        t1 = time.monotonic()
        ws.send_json({'type': 'teleop', 'deadman': False})
        assert wait(lambda: any(p[2] == 0.0 for p in teleop_published(gw, t1)), timeout=0.5)
    assert wait(lambda: gw.teleop.holder is None, timeout=2)


def test_the_watchdog_on_the_wire(gw):
    with connect(gw) as ws:
        ws.send_json({'type': 'drive_acquire'})
        recv_until(ws, 'drive')
        for seq in range(1, 6):
            ws.send_json({'type': 'teleop', 'seq': seq, 'thr': 1.0, 'steer': 0.0,
                          'deadman': True})
            time.sleep(0.05)
        last_input = time.monotonic()
        # Silence: no teleop message any more (a stalled link), the socket stays up.
        assert wait(lambda: any(p[2] == 0.0 for p in teleop_published(gw, last_input)),
                    timeout=1.0)
        zero_t = next(p[0] for p in teleop_published(gw, last_input) if p[2] == 0.0)
        assert zero_t - last_input <= 0.25 + 0.06, 'zero within the 250 ms watchdog'
        time.sleep(0.8)
        late = teleop_published(gw, zero_t + 0.6)
        assert late == [], 'after the zero tail nothing more: the mux lane times out'


def test_reverse_on_the_wire_goes_through_0_until_the_car_stops(gw):
    """Forward, then the stick back: zeros until the pretend car is still, then -0.4 at most."""
    with gw.client.websocket_connect('/ws') as ws:
        ws.send_json({'type': 'auth', 'token': TOKEN})
        hello = ws.receive_json()
        assert hello['teleop']['max_reverse_speed_mps'] == 0.4
        ws.send_json({'type': 'drive_acquire'})
        assert recv_until(ws, 'drive')['state'] == 'granted'
        t0 = time.monotonic()
        seq = 0
        for thr in [1.0] * 20 + [-1.0] * 40:
            seq += 1
            ws.send_json({'type': 'teleop', 'seq': seq, 'thr': thr, 'steer': 0.0,
                          'limit': 1.0, 'deadman': True})
            time.sleep(0.05)
        pubs = teleop_published(gw, t0)
        ws.send_json({'type': 'teleop', 'deadman': False})
    last_fwd = max(p[0] for p in pubs if p[2] > 0)
    first_rev = min(p[0] for p in pubs if p[2] < 0)
    assert min(p[2] for p in pubs) >= -0.4 - 1e-9
    assert all(p[2] == 0.0 for p in pubs if last_fwd < p[0] < first_rev)
    assert first_rev - last_fwd >= 0.3, 'at least reverse_hold_s at 0'
    # The pretend car is at ~1.0 m/s after 1 s (1 m/s^2) and slows at 2 m/s^2: 0.47 s to
    # 0.05 m/s, longer than the hold, so the zero lasted until it was still.
    assert first_rev - last_fwd >= 0.4


def test_guida_is_refused_with_a_joystick_or_a_latch_or_a_mission(gw):
    with connect(gw) as ws:
        gw.world.teleop_others = 1
        ws.send_json({'type': 'drive_acquire'})
        assert 'joystick' in recv_until(ws, 'drive')['reason']
        gw.world.teleop_others = 0
        gw.client.post('/api/_mock/set', headers=H, json={'estop_latched': True})
        ws.send_json({'type': 'drive_acquire'})
        assert 'E-STOP' in recv_until(ws, 'drive')['reason']


def test_estop_revokes_the_lease(gw):
    with connect(gw) as ws:
        ws.send_json({'type': 'drive_acquire'})
        recv_until(ws, 'drive')
        ws.send_json({'type': 'estop'})
        m = recv_until(ws, 'drive', lambda m: m['state'] == 'revoked')
        assert m['reason'] == 'E-STOP' and gw.teleop.holder is None
