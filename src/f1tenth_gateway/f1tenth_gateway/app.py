"""
The gateway's web side: REST + one WebSocket + the PWA's files.

Gateway holds the state and the rules; create_app() wires it to FastAPI.
ROS and f1tenth_ctl are two ports (ros_bridge / ctl_port, or mock.py's
stand-ins): nothing in here imports rclpy.

Threads: uvicorn's event loop runs the handlers; anything that blocks
(f1tenth_ctl, a ROS service call, the e-stop's 2 s window, a transcription)
runs in the thread pool; the teleop publish loop runs in the ROS port's own
thread, so a busy event loop never delays a zero (teleop.py).

The rules that make the car safe are in teleop.py (watchdog, clamps, lease)
and mission.py (auto-abort); this file enforces the preconditions around
them:
- Guida only with the stack running, the mux listening on /teleop, no other
  /teleop publisher, no e-stop latched and no mission active, re-checked at
  2 Hz while the lease is held;
- a mission start only for a plan this gateway made, valid, not older than
  the plan age limit, with no mission active, no latch and Guida closed;
- an e-stop reset only with the measured speed exactly 0 for reset_still_s;
- a transcription never during a mission or with Guida open;
- no automatic retry of a start, an abort or a process stop.
"""

import asyncio
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import hmac
import json
import logging
import os
import re
import secrets
import threading
import time
from typing import Dict, List, Optional

from fastapi import Depends, FastAPI, File, HTTPException, Request, UploadFile, WebSocket
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from starlette.websockets import WebSocketDisconnect, WebSocketState

from . import mapimg
from .mission import ACTIVE, MissionTracker
from .runs import Archive
from .teleop import Teleop

log = logging.getLogger('f1tenth_gateway')

PROCS = ('stack', 'planner', 'logger')
AUDIO_SUFFIX = {'audio/mp4': '.m4a', 'audio/x-m4a': '.m4a', 'audio/aac': '.aac',
                'audio/webm': '.webm', 'audio/ogg': '.ogg', 'audio/wav': '.wav',
                'audio/x-wav': '.wav', 'audio/mpeg': '.mp3'}
SESSION_S = 12 * 3600
SESSION_COOKIE = 'f1t_session'


class Client:
    def __init__(self, ws: WebSocket, user: str, now: float):
        self.id = secrets.token_hex(6)
        self.ws = ws
        self.user = user
        self.last_rx = now
        self.connected = now


class Gateway:
    def __init__(self, cfg: Dict, make_ros, ctl, transcriber, token: str, world=None,
                 clock=time.monotonic, wall=time.time):
        self.cfg = cfg
        self.ctl = ctl
        self.transcriber = transcriber
        self.token = token
        self.world = world                 # MockWorld in mock mode, else None
        self.clock, self.wall = clock, wall
        self.teleop = Teleop(cfg['teleop_cfg'])
        self.tlock = threading.Lock()      # the tracker is fed from ROS threads
        self.tracker = MissionTracker(ctl.describe_state, cfg['mission']['auto_abort_s'])
        self.ros = make_ros(self.teleop, ctl.estop_spec(), self._on_status, self._on_outcome)
        arch = cfg['archive']
        self.archive = Archive(arch['root'], arch['workspace'])
        self.pool = ThreadPoolExecutor(max_workers=8, thread_name_prefix='gw')
        self.clients: Dict[str, Client] = {}
        self.last_client_t = clock()
        self.status: Dict = {}
        self.status_t: Optional[float] = None
        self._status_busy = False
        self.jobs: Dict[str, Dict] = {}
        self.plans: Dict[str, Dict] = {}
        self.planning: Optional[str] = None
        self.proc_pending: Dict[str, str] = {}
        self.events: deque = deque(maxlen=50)
        self.failures: deque = deque()
        self.sessions: Dict[str, float] = {}
        self.estop_future = None
        self.still_since: Optional[float] = None
        self.frozen_until = 0.0
        self._sent = {'bt': None, 'path': None, 'map': None}
        self._map_png: Dict = {}
        self._next_guard = 0.0
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self.tasks: List[asyncio.Task] = []

    # ---- ROS callbacks (ROS threads) -------------------------------------------------------

    def _on_status(self, state, json_path, estop, now):
        with self.tlock:
            was = self.tracker.estop_latched if self.tracker.ros_state is not None else None
            self.tracker.on_status(state, json_path, estop, now)
        if was is not None and bool(estop) != bool(was) and self.clients and self._loop:
            # READY changes with the latch: tell the phones now, not at the next status tick.
            # The payload is built on the event loop, as status_loop does, not on ROS's thread.
            asyncio.run_coroutine_threadsafe(self._push_status(), self._loop)

    async def _push_status(self) -> None:
        await self.broadcast(self.status_payload())

    def _on_outcome(self, mission_id, move_id):
        with self.tlock:
            self.tracker.on_move_outcome(mission_id, move_id)

    def mission_view(self) -> Dict:
        with self.tlock:
            return self.tracker.view(self.clock(), self.wall())

    def mission_active(self) -> bool:
        return self.mission_view().get('state') in ACTIVE

    def estop_latched(self) -> bool:
        with self.tlock:
            if self.tracker.ros_state is not None:
                return bool(self.tracker.estop_latched)
        return bool(self.status.get('estop_latched'))

    # ---- auth ---------------------------------------------------------------------------------

    def _limited(self) -> bool:
        now = self.clock()
        while self.failures and now - self.failures[0] > 60.0:
            self.failures.popleft()
        return len(self.failures) >= int(self.cfg['auth']['max_failures_per_min'])

    def check_token(self, value: Optional[str]) -> bool:
        if self._limited():
            raise HTTPException(429, 'troppi tentativi, riprova tra un minuto')
        if value and hmac.compare_digest(value.encode(), self.token.encode()):
            return True
        self.failures.append(self.clock())
        return False

    def check_session(self, sid: Optional[str]) -> bool:
        exp = self.sessions.get(sid or '')
        return exp is not None and exp > self.wall()

    # ---- events and broadcast ------------------------------------------------------------------

    def event(self, level: str, text: str) -> None:
        ev = {'type': 'event', 'level': level, 'text': text, 't': self.wall()}
        self.events.append(ev)
        log.log(logging.WARNING if level != 'info' else logging.INFO, 'event: %s', text)
        self._send_threadsafe(ev)

    def _send_threadsafe(self, msg: Dict) -> None:
        loop = self._loop
        if loop is None:
            return
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            loop.create_task(self.broadcast(msg))
        else:
            asyncio.run_coroutine_threadsafe(self.broadcast(msg), loop)

    async def send(self, client: Client, msg: Dict) -> None:
        if self.frozen_until > self.clock():
            return
        try:
            if client.ws.application_state == WebSocketState.CONNECTED:
                await asyncio.wait_for(client.ws.send_text(json.dumps(msg)), timeout=1.0)
        except Exception:  # noqa: BLE001 - a dead or slow client is closed by the guard
            pass

    async def broadcast(self, msg: Dict) -> None:
        for client in list(self.clients.values()):
            await self.send(client, msg)

    def publish_now(self, cmd) -> None:
        if cmd is not None:
            self.ros.publish_teleop(cmd)

    # ---- jobs ----------------------------------------------------------------------------------

    def job(self, kind: str, fn, *args) -> Dict:
        jid = secrets.token_hex(6)
        job = {'id': jid, 'kind': kind, 'state': 'running', 'started': self.wall()}
        self.jobs[jid] = job

        def run():
            try:
                job['result'] = fn(*args)
                result = job['result']
                ok = bool(result.get('ok', True)) if isinstance(result, dict) else True
                job['state'] = 'done' if ok else 'failed'
            except Exception as exc:  # noqa: BLE001 - reported in the job
                job['state'] = 'failed'
                job['result'] = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
            job['ended'] = self.wall()
        self.pool.submit(run)
        if len(self.jobs) > 200:
            for old in sorted(self.jobs.values(), key=lambda j: j['started'])[:50]:
                self.jobs.pop(old['id'], None)
        return job

    # ---- status --------------------------------------------------------------------------------

    def refresh_status(self) -> None:
        try:
            st = self.ctl.status()
        except Exception as exc:  # noqa: BLE001
            st = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
        self.status, self.status_t = st, self.clock()

    def status_payload(self) -> Dict:
        st = self.status or {}
        latest = self.ros.latest()
        procs = {}
        for name in PROCS:
            p = dict((st.get('processes') or {}).get(name) or {'state': 'unknown'})
            if name in self.proc_pending:
                p['pending'] = self.proc_pending[name]
            since = p.get('since')
            if isinstance(since, (int, float)):
                p['uptime_s'] = round(self.wall() - since, 0)
            procs[name] = p
        procs['stack']['nodes'] = (st.get('probe') or {}).get('nodes')
        latched = self.estop_latched()
        checks = [self._live_estop(c, latched) for c in st.get('checks') or []]
        required = [c for c in checks if c.get('required', True)]
        failing = [c['name'] for c in required if not c.get('ok')]
        if not checks:
            failing = st.get('failing') or []
        system = latest.get('system') or {}
        return {
            'type': 'status', 'ok': st.get('ok', False), 'error': st.get('error'),
            'ready': bool(st.get('ready')) and not failing, 'failing': failing,
            'checks': checks, 'checks_ok': sum(1 for c in required if c.get('ok')),
            'checks_total': len(required), 'estop_latched': latched,
            'processes': procs, 'mission': self.mission_view(),
            'battery': latest.get('battery'), 'vesc_temp_c': latest.get('vesc_temp_c'),
            'cpu_percent': system.get('cpu_percent'), 'cpu_temp_c': system.get('cpu_temp_c'),
            'drive': {'busy': self.teleop.holder is not None, 'blockers': self.drive_blockers()},
            'reset_allowed': self.reset_allowed(), 'transcription': self.transcriber.status(),
            'age_s': None if self.status_t is None else round(self.clock() - self.status_t, 1),
            'server_time': self.wall()}

    def _live_estop(self, check: Dict, latched: bool) -> Dict:
        """
        Return the e-stop check as /mission/status has it now.

        f1tenth_ctl's checks are refreshed every status_period_s; the latch comes from
        /mission/status as it changes, so READY does not lag the E-STOP banner.
        """
        with self.tlock:
            known = self.tracker.ros_state is not None
        if check.get('kind') != 'estop' or not known or check.get('value') is latched:
            return check
        out = dict(check, ok=not latched, value=latched)
        out.pop('why', None)
        if latched:
            out['why'] = 'e-stop latched'
        return out

    # ---- preconditions -------------------------------------------------------------------------

    def drive_blockers(self) -> List[str]:
        why = []
        stack = ((self.status.get('processes') or {}).get('stack') or {}).get('state')
        if stack != 'running':
            why.append('stack non in esecuzione')
        try:
            graph = self.ros.teleop_graph()
        except Exception:  # noqa: BLE001 - ROS not up yet
            graph = {'subscribers': 0, 'other_publishers': 0}
        if graph.get('subscribers', 0) < 1:
            why.append('ackermann_mux non ascolta /teleop')
        if graph.get('other_publishers', 0) > 0:
            why.append('un altro nodo pubblica su /teleop (joystick?)')
        if self.estop_latched():
            why.append('E-STOP attivo')
        if self.mission_active():
            why.append('missione in corso')
        return why

    def reset_allowed(self) -> Dict:
        if not self.estop_latched():
            return {'ok': False, 'why': 'nessun E-STOP attivo'}
        if self.teleop.holder is not None:
            return {'ok': False, 'why': 'chiudi la Guida'}
        if self.still_since is None:
            return {'ok': False, 'why': 'velocità misurata non nulla o sconosciuta'}
        held = self.clock() - self.still_since
        need = float(self.cfg['estop']['reset_still_s'])
        if held < need:
            return {'ok': False, 'why': f'ferma da {held:.1f} s, servono {need:.1f} s'}
        return {'ok': True, 'why': None}

    def _update_still(self, latest: Dict) -> None:
        v = latest.get('v_meas')
        age = (latest.get('ages') or {}).get('v_meas')
        fresh = v is not None and age is not None and age <= float(
            self.cfg['estop']['reset_odom_max_age_s'])
        if fresh and abs(float(v['v'])) < 1e-3:
            if self.still_since is None:
                self.still_since = self.clock()
        else:
            self.still_since = None

    # ---- actions -------------------------------------------------------------------------------

    async def do_estop(self, who: str) -> Dict:
        self.publish_now(self.teleop.revoke('E-STOP', self.clock()))
        loop = asyncio.get_running_loop()
        if self.estop_future is None or self.estop_future.done():
            self.estop_future = loop.run_in_executor(self.pool, self._estop_blocking)
            self.event('error', f'E-STOP da {who}')
        res = await self.estop_future
        return res

    def _estop_blocking(self) -> Dict:
        try:
            res = self.ros.estop()
            res['path'] = 'gateway'
        except Exception as exc:  # noqa: BLE001 - the CLI's own e-stop is the fallback
            log.error('in-process e-stop failed (%s): running f1tenth_ctl estop', exc)
            res = self.ctl.estop()
            res['path'] = 'f1tenth_ctl'
            res['gateway_error'] = f'{type(exc).__name__}: {exc}'
        return {'ok': bool(res.get('car_held')), **res}

    def plan_job(self, text: str) -> Dict:
        plan_lock = self.ctl.plan_lock
        if not plan_lock.acquire(blocking=False):
            return {'ok': False, 'error': 'un altro piano è in corso'}
        try:
            self.planning = text
            st = self.ctl.planner_log()
            if not st.get('running'):
                return {'ok': False, 'error': 'il planner non è in esecuzione'}
            if st.get('plan_only') is not True:
                return {'ok': False, 'error': "il planner sull'auto non è plan-only: "
                                              'riavvialo con f1tenth_ctl aggiornato'}
            sent = self.ctl.send_planner(text)
            if not sent.get('ok', True) or sent.get('cursor') is None:
                return {'ok': False, 'error': sent.get('error') or 'invio al planner fallito'}
            since = int(sent['cursor'])
            deadline = time.monotonic() + float(self.cfg['mission']['plan_timeout_s'])
            lines, entry = [], None
            while time.monotonic() < deadline:
                res = self.ctl.planner_log(since)
                lines = res.get('lines') or lines
                results = res.get('results') or []
                if results:
                    entry = results[0]
                    break
                if not res.get('running'):
                    return {'ok': False, 'error': 'il planner si è fermato',
                            'log_tail': lines[-20:]}
                time.sleep(float(self.cfg['mission']['plan_poll_s']))
            if entry is None:
                return {'ok': False, 'error': 'nessuna risposta dal planner entro '
                                              f'{self.cfg["mission"]["plan_timeout_s"]:.0f} s',
                        'log_tail': lines[-20:]}
            return self._plan_from(text, entry, lines)
        finally:
            self.planning = None
            plan_lock.release()

    def _plan_from(self, text: str, entry: Dict, lines: List[str]) -> Dict:
        base = {'text': text, 'status': entry.get('status'), 'log_tail': lines[-20:],
                'unsupported': entry.get('unsupported') or [], 'notes': entry.get('notes') or [],
                'plan_seconds': entry.get('plan_seconds')}
        stored = entry.get('stored')
        if entry.get('status') != 'ok' or not stored:
            err = ': '.join(x for x in (entry.get('error_type'), entry.get('error_message'))
                            if x) or entry.get('status') or 'piano non valido'
            return {'ok': False, **base, 'valid': False, 'errors': [err]}
        mission = entry.get('mission') or {}
        pid = stored['id']
        path = stored.get('path') or self.ctl.mission_path(pid)
        check = self.ctl.validate(path)
        summary = self.ctl.summarize(mission)
        plan = {'ok': True, **base, 'plan_id': pid, 'path': path, 'json': mission,
                'mission_id': mission.get('mission_id'), 'created': self.wall(),
                'summary': {k: summary.get(k) for k in ('n_moves', 'max_speed_mps',
                                                        'distance_m', 'max_duration_s',
                                                        'targets', 'reverse', 'speed_note')},
                'steps': summary.get('moves') or [], 'valid': bool(check.get('ok')),
                'errors': [] if check.get('ok') else [check.get('message')]}
        self.plans[pid] = plan
        return plan

    def start_mission(self, pid: str) -> Dict:
        plan = self.plans.get(pid)
        if plan is None:
            return {'ok': False, 'code': 404, 'why': 'piano sconosciuto a questo gateway'}
        age = self.wall() - plan['created']
        max_age = float(self.ctl.plan_max_age_s)
        if age > max_age:
            return {'ok': False, 'code': 409,
                    'why': f'piano vecchio di {age / 60:.0f} min (max {max_age / 60:.0f}): '
                           'ripianifica'}
        if not plan['valid']:
            return {'ok': False, 'code': 409, 'why': 'piano non valido'}
        if self.estop_latched():
            return {'ok': False, 'code': 409, 'why': 'E-STOP attivo'}
        if self.mission_active():
            return {'ok': False, 'code': 409, 'why': 'una missione è già in corso'}
        if self.teleop.holder is not None:
            return {'ok': False, 'code': 409, 'why': 'chiudi la Guida prima di avviare'}
        load = self.ctl.mission_load(pid)
        if not load.get('ok'):
            return {'ok': False, 'code': 409, 'load': load,
                    'why': load.get('refused') or load.get('error') or 'caricamento fallito'}
        start = self.ctl.mission_start(pid)
        if not start.get('ok'):
            return {'ok': False, 'code': 409, 'load': load, 'start': start,
                    'why': start.get('refused') or start.get('error') or 'avvio fallito'}
        countdown = float(start.get('countdown_s') or 0.0)
        with self.tlock:
            self.tracker.on_app_start(plan['path'], countdown, self.wall())
        self.ros.reset_path()
        self.event('warn', f"missione {pid} avviata dall'app (countdown {countdown:.0f} s)")
        return {'ok': True, 'plan_id': pid, 'countdown_s': countdown, 'load': load,
                'start': start}

    def proc_op(self, name: str, action: str) -> Dict:
        lock = self.ctl.proc_locks[name]
        if not lock.acquire(blocking=False):
            return {'ok': False, 'error': 'operazione già in corso'}
        try:
            self.proc_pending[name] = 'starting' if action == 'start' else 'stopping'
            res = (self.ctl.start if action == 'start' else self.ctl.stop)(name)
            self.refresh_status()
            return res
        finally:
            self.proc_pending.pop(name, None)
            lock.release()

    # ---- the periodic work ---------------------------------------------------------------------

    async def status_loop(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            if not self._status_busy:
                self._status_busy = True
                try:
                    await loop.run_in_executor(self.pool, self.refresh_status)
                finally:
                    self._status_busy = False
                if self.clients:
                    await self.broadcast(self.status_payload())
            await asyncio.sleep(float(self.cfg['client']['status_period_s']))

    async def telemetry_loop(self) -> None:
        period = 1.0 / float(self.cfg['client']['telemetry_hz'])
        n = 0
        while True:
            try:
                await self._telemetry_tick(n)
            except Exception:  # noqa: BLE001 - never let the loop die
                log.exception('telemetry tick failed')
            n += 1
            await asyncio.sleep(period)

    async def _telemetry_tick(self, n: int) -> None:
        now = self.clock()
        self.ros.set_viewers(len(self.clients))
        latest = self.ros.latest()
        self._update_still(latest)
        # Lost clients: no message for lost_after_s -> closed, lease released.
        lost_after = float(self.cfg['client']['lost_after_s'])
        for client in list(self.clients.values()):
            if now - client.last_rx > lost_after:
                await self.drop(client, 4408)
        # The lease's preconditions, at 2 Hz.
        if self.teleop.holder is not None and now >= self._next_guard:
            self._next_guard = now + 0.5
            blockers = self.drive_blockers()
            if blockers:
                self.publish_now(self.teleop.revoke(blockers[0], now))
        for cid, kind, reason in self.teleop.pop_events():
            client = self.clients.get(cid)
            if client is not None:
                await self.send(client, {'type': 'drive', 'state': kind, 'reason': reason})
            if kind == 'revoked':
                self.event('warn', f'guida revocata: {reason}')
        await self._auto_abort(now)
        if not self.clients:
            return
        view = self.mission_view()
        cmd = latest.get('cmd') or {}
        tel = {'type': 'telemetry', 't': self.wall(), 'pose': latest.get('pose'),
               'v_meas': (latest.get('v_meas') or {}).get('v'), 'v_cmd': cmd.get('v'),
               'steer_cmd': cmd.get('steer'), 'cmd_source': cmd.get('source'),
               'safety': latest.get('safety'), 'mission': view,
               'battery': latest.get('battery'), 'estop_latched': self.estop_latched(),
               'drive': self.teleop.snapshot(), 'reset_allowed': self.reset_allowed()}
        await self.broadcast(tel)
        if n % 2 == 0:
            bt = self.ros.bt()
            if bt['version'] != self._sent['bt']:
                self._sent['bt'] = bt['version']
                await self.broadcast({'type': 'bt', 'nodes': bt['nodes']})
        if n % 10 == 0:
            path = self.ros.path()
            if path['version'] != self._sent['path']:
                self._sent['path'] = path['version']
                pts = path['points']
                step = max(1, len(pts) // 600)
                await self.broadcast({'type': 'path', 'points': pts[::step],
                                      'horizon': latest.get('horizon') or []})
            grid = self.ros.map_grid()
            if grid is not None and grid['version'] != self._sent['map']:
                self._sent['map'] = grid['version']
                await self.broadcast({'type': 'map', 'version': grid['version']})
        if n % 100 == 0:
            self.transcriber.reap_idle()

    async def _auto_abort(self, now: float) -> None:
        if self.clients:
            last = max(c.last_rx for c in self.clients.values())
        else:
            last = self.last_client_t
        with self.tlock:
            due = self.tracker.auto_abort_due(now, self.wall(), last)
        if not due:
            return
        loop = asyncio.get_running_loop()
        res = await loop.run_in_executor(self.pool, self.ros.abort)
        with self.tlock:
            self.tracker.mark_auto_abort(res, self.wall())
        self.event('error', 'missione interrotta (abort): nessun client da '
                            f'{now - last:.1f} s ({res.get("message")})')

    async def drop(self, client: Client, code: int) -> None:
        self.clients.pop(client.id, None)
        self.last_client_t = self.clock()
        self.publish_now(self.teleop.release(client.id, self.clock()))
        try:
            if client.ws.application_state == WebSocketState.CONNECTED:
                await client.ws.close(code=code)
        except Exception:  # noqa: BLE001
            pass

    def map_png(self) -> Optional[Dict]:
        grid = self.ros.map_grid()
        if grid is None:
            return None
        if self._map_png.get('version') != grid['version']:
            img = mapimg.grid_to_png(grid['width'], grid['height'], grid['data'])
            res = grid['resolution'] * img['factor']
            self._map_png = {'version': grid['version'], 'png': img['png'],
                             'meta': {'version': grid['version'], 'width': img['width'],
                                      'height': img['height'], 'resolution': res,
                                      'origin': grid['origin']}}
        return self._map_png


# ==============================================================================================
# FastAPI
# ==============================================================================================

def create_app(gw: Gateway) -> FastAPI:
    app = FastAPI(title='F1Tenth Lab Control gateway', docs_url=None, redoc_url=None,
                  openapi_url=None)
    cfg = gw.cfg

    @app.on_event('startup')
    async def _startup():
        gw._loop = asyncio.get_running_loop()
        gw.ros.start()
        gw.tasks = [asyncio.create_task(gw.status_loop()),
                    asyncio.create_task(gw.telemetry_loop())]

    @app.on_event('shutdown')
    async def _shutdown():
        for t in gw.tasks:
            t.cancel()
        gw.publish_now(gw.teleop.revoke('shutdown', gw.clock()))
        gw.transcriber.stop()
        gw.ros.stop()
        gw.pool.shutdown(wait=False)

    @app.middleware('http')
    async def headers(request: Request, call_next):
        response = await call_next(request)
        path = request.url.path
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Referrer-Policy'] = 'no-referrer'
        response.headers['Content-Security-Policy'] = (
            "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
            "img-src 'self' data: blob:; media-src 'self' blob:; font-src 'self'; "
            "connect-src 'self' ws: wss:; frame-ancestors 'none'; base-uri 'none'")
        no_store = ('/', '/index.html', '/sw.js', '/manifest.webmanifest')
        if path.startswith('/api/') or path in no_store:
            response.headers['Cache-Control'] = 'no-store'
        elif path.startswith('/assets/'):
            response.headers['Cache-Control'] = 'public, max-age=31536000, immutable'
        return response

    def auth(request: Request):
        header = request.headers.get('authorization', '')
        if header.lower().startswith('bearer ') and gw.check_token(header[7:].strip()):
            return True
        if gw.check_session(request.cookies.get(SESSION_COOKIE)):
            return True
        if not header:
            gw.failures.append(gw.clock())
        raise HTTPException(401, 'token mancante o errato')

    def who(request: Request) -> str:
        return request.headers.get('tailscale-user-login') or 'app'

    # ---- session -------------------------------------------------------------------------------

    @app.post('/api/session')
    async def session(request: Request, _=Depends(auth)):
        sid = secrets.token_urlsafe(24)
        gw.sessions[sid] = gw.wall() + SESSION_S
        resp = JSONResponse({'ok': True, 'expires_s': SESSION_S})
        resp.set_cookie(SESSION_COOKIE, sid, max_age=SESSION_S, httponly=True,
                        secure=bool(cfg['auth'].get('cookie_secure', True)),
                        samesite='strict', path='/api')
        return resp

    # ---- status, processes, jobs ---------------------------------------------------------------

    @app.get('/api/status')
    async def status(_=Depends(auth)):
        return gw.status_payload()

    @app.post('/api/process/{name}')
    async def process(name: str, request: Request, _=Depends(auth)):
        if name not in PROCS:
            raise HTTPException(404, 'processo sconosciuto')
        body = await request.json()
        action = body.get('action')
        if action not in ('start', 'stop'):
            raise HTTPException(422, 'action: start | stop')
        if name in gw.proc_pending:
            raise HTTPException(409, 'operazione già in corso')
        if name == 'stack' and action == 'stop':
            latest = gw.ros.latest()
            v = (latest.get('v_meas') or {}).get('v')
            if gw.mission_active() or (v is not None and abs(v) > 0.05):
                raise HTTPException(409, 'la macchina si muove o ha una missione: ferma prima '
                                         '(ABORT o EMERGENZA)')
        gw.event('info', f'{name}: {action} ({who(request)})')
        job = gw.job(f'process:{name}:{action}', gw.proc_op, name, action)
        return JSONResponse(job, status_code=202)

    @app.get('/api/jobs/{jid}')
    async def job(jid: str, _=Depends(auth)):
        job = gw.jobs.get(jid)
        if job is None:
            raise HTTPException(404, 'job sconosciuto')
        return job

    # ---- e-stop --------------------------------------------------------------------------------

    @app.post('/api/estop')
    async def estop(request: Request, _=Depends(auth)):
        return await gw.do_estop(who(request))

    @app.post('/api/estop/reset')
    async def estop_reset(request: Request, _=Depends(auth)):
        allowed = gw.reset_allowed()
        if not allowed['ok']:
            raise HTTPException(409, allowed['why'])
        gw.event('warn', f'sblocco E-STOP richiesto ({who(request)})')
        return JSONResponse(gw.job('estop_reset', gw.ctl.estop_reset), status_code=202)

    # ---- mission -------------------------------------------------------------------------------

    @app.post('/api/mission/transcribe')
    async def transcribe(audio: UploadFile = File(...), _=Depends(auth)):
        if gw.mission_active():
            raise HTTPException(409, 'trascrizione non disponibile durante una missione')
        if gw.teleop.holder is not None:
            raise HTTPException(409, 'trascrizione non disponibile con la Guida aperta')
        data = await audio.read(int(cfg['transcription']['max_bytes']) + 1)
        if len(data) > int(cfg['transcription']['max_bytes']):
            raise HTTPException(413, 'audio troppo grande')
        ctype = (audio.content_type or '').split(';')[0].strip().lower()
        suffix = AUDIO_SUFFIX.get(ctype, '.m4a')
        loop = asyncio.get_running_loop()
        try:
            res = await loop.run_in_executor(gw.pool, gw.transcriber.transcribe, data, suffix)
        except Exception as exc:  # noqa: BLE001 - TranscriptionError and decode errors
            raise HTTPException(422, str(exc)) from exc
        text = ' '.join(str(res.get('text') or '').split())[:int(cfg['mission']['max_text_len'])]
        return {**res, 'text': text}

    @app.post('/api/mission/plan')
    async def plan(request: Request, _=Depends(auth)):
        body = await request.json()
        text = ' '.join(str(body.get('text') or '').split())
        if not text:
            raise HTTPException(422, 'comando vuoto')
        if len(text) > int(cfg['mission']['max_text_len']):
            raise HTTPException(422, f'al massimo {cfg["mission"]["max_text_len"]} caratteri')
        if gw.mission_active():
            raise HTTPException(409, 'una missione è in corso')
        if gw.planning:
            raise HTTPException(409, 'un altro piano è in corso')
        gw.event('info', f'piano richiesto: «{text}» ({who(request)})')
        return JSONResponse(gw.job('plan', gw.plan_job, text), status_code=202)

    @app.post('/api/mission/start')
    async def start(request: Request, _=Depends(auth)):
        body = await request.json()
        pid = str(body.get('plan_id') or '')
        loop = asyncio.get_running_loop()
        res = await loop.run_in_executor(gw.pool, gw.start_mission, pid)
        if not res['ok']:
            raise HTTPException(res.get('code', 409), res['why'])
        return res

    @app.post('/api/mission/abort')
    async def abort(request: Request, _=Depends(auth)):
        loop = asyncio.get_running_loop()
        res = await loop.run_in_executor(gw.pool, gw.ros.abort)
        gw.event('warn', f'ABORT ({who(request)}): {res.get("message")}')
        if not res.get('ok'):
            raise HTTPException(502, res.get('message') or 'abort fallito')
        return res

    # ---- runs, map -----------------------------------------------------------------------------

    @app.get('/api/runs')
    async def runs(limit: int = 20, _=Depends(auth)):
        limit = max(1, min(int(limit), int(cfg['archive']['max_runs'])))
        loop = asyncio.get_running_loop()
        return {'runs': await loop.run_in_executor(gw.pool, gw.archive.list_runs, limit)}

    @app.get('/api/runs/{run_id}/report')
    async def report(run_id: str, _=Depends(auth)):
        page = gw.archive.report_html(run_id)
        if page is None:
            raise HTTPException(404, 'run sconosciuta')
        return HTMLResponse(page)

    @app.get('/api/runs/{run_id}/snapshots/{n}')
    async def snapshot(run_id: str, n: int, _=Depends(auth)):
        path = gw.archive.snapshot_path(run_id, n)
        if path is None:
            raise HTTPException(404, 'snapshot assente')
        with open(path, 'rb') as f:
            return Response(f.read(), media_type='image/jpeg')

    @app.get('/api/runs/{run_id}/video')
    async def video(run_id: str, request: Request, _=Depends(auth)):
        path = gw.archive.video_path(run_id)
        if path is None:
            raise HTTPException(404, 'nessun video per questa run')
        return ranged_file(path, request.headers.get('range'), 'video/mp4')

    @app.get('/api/map')
    async def map_meta(_=Depends(auth)):
        loop = asyncio.get_running_loop()
        img = await loop.run_in_executor(gw.pool, gw.map_png)
        if img is None:
            raise HTTPException(404, 'nessuna mappa (/slam/map)')
        return img['meta']

    @app.get('/api/map.png')
    async def map_png(_=Depends(auth)):
        loop = asyncio.get_running_loop()
        img = await loop.run_in_executor(gw.pool, gw.map_png)
        if img is None:
            raise HTTPException(404, 'nessuna mappa (/slam/map)')
        return Response(img['png'], media_type='image/png')

    # ---- mock-only hooks (tests and screenshots) -----------------------------------------------

    if gw.world is not None:
        @app.post('/api/_mock/{op}')
        async def mock_op(op: str, request: Request, _=Depends(auth)):
            w = gw.world
            body = await request.json() if (await request.body()) else {}
            if op == 'set':
                with w._lock:
                    for key in ('failing', 'battery_v', 'teleop_others', 'mux_listens',
                                'countdown_s', 'move_s'):
                        if key in body:
                            setattr(w, key, body[key])
                    if 'estop_latched' in body:
                        w.estop_latched = bool(body['estop_latched'])
                        w.notify()
                    if 'v' in body:
                        w.v = float(body['v'])
                gw.refresh_status()
                return {'ok': True}
            if op == 'reset':
                w.reset()
                gw.publish_now(gw.teleop.revoke('reset', gw.clock()))
                gw.teleop.pop_events()
                gw.plans.clear()
                gw.events.clear()
                with gw.tlock:
                    gw.tracker = MissionTracker(gw.ctl.describe_state,
                                                cfg['mission']['auto_abort_s'])
                    gw.tracker.on_status('IDLE', '', False, gw.clock())
                gw.frozen_until = 0.0
                gw.refresh_status()
                return {'ok': True}
            if op == 'freeze':
                gw.frozen_until = gw.clock() + float(body.get('seconds', 3.0))
                return {'ok': True}
            if op == 'drop':
                for client in list(gw.clients.values()):
                    await gw.drop(client, 4000)
                return {'ok': True}
            if op == 'state':
                with w._lock:
                    return {'published': w.published[-400:], 'ros_state': w.ros_state,
                            'estop_latched': w.estop_latched, 'v': w.v,
                            'teleop': gw.teleop.snapshot(), 'clients': len(gw.clients),
                            'events': list(gw.events)[-20:]}
            raise HTTPException(404, 'op sconosciuta')

    # ---- the WebSocket -------------------------------------------------------------------------

    @app.websocket('/ws')
    async def ws(websocket: WebSocket):
        await websocket.accept()
        try:
            first = await asyncio.wait_for(websocket.receive_json(),
                                           timeout=float(cfg['auth']['ws_auth_timeout_s']))
            ok = first.get('type') == 'auth' and gw.check_token(str(first.get('token') or ''))
        except HTTPException:
            await websocket.close(code=4429)
            return
        except Exception:  # noqa: BLE001 - timeout, bad JSON, disconnect
            ok = False
        if not ok:
            await websocket.close(code=4401)
            return
        user = websocket.headers.get('tailscale-user-login') or 'app'
        client = Client(websocket, user, gw.clock())
        gw.clients[client.id] = client
        gw.last_client_t = gw.clock()
        t = cfg['teleop_cfg']
        await gw.send(client, {
            'type': 'hello', 'client_id': client.id,
            'mock': gw.world is not None,          # the pretend car: the app shows MOCK
            'teleop': {'min_speed_mps': 0.4, 'max_speed_mps': t.max_speed_mps,
                       'default_limit_mps': t.default_limit_mps,
                       'limit_step_mps': t.limit_step_mps, 'deadzone': t.deadzone,
                       'max_steer_rad': t.max_steer_rad, 'watchdog_s': t.watchdog_s},
            'mission': {'auto_abort_s': cfg['mission']['auto_abort_s']},
            'client': {'lost_after_s': cfg['client']['lost_after_s'],
                       'ping_s': cfg['client']['ping_s']}})
        await gw.send(client, gw.status_payload())
        gw._sent = {'bt': None, 'path': None, 'map': None}
        for ev in list(gw.events)[-5:]:
            await gw.send(client, ev)
        try:
            while True:
                msg = await websocket.receive_json()
                now = gw.clock()
                client.last_rx = now
                gw.last_client_t = now
                await handle(client, msg, now)
        except (WebSocketDisconnect, RuntimeError):
            pass
        except Exception:  # noqa: BLE001
            log.exception('websocket handler')
        finally:
            await gw.drop(client, 1000)

    async def handle(client: Client, msg: Dict, now: float) -> None:
        kind = msg.get('type')
        if kind == 'ping':
            await gw.send(client, {'type': 'pong', 'id': msg.get('id'), 't': msg.get('t'),
                                   'server_time': gw.wall()})
        elif kind == 'teleop':
            status, cmd = gw.teleop.on_input(client.id, msg, now)
            gw.publish_now(cmd)
            if status in ('not_holder', 'tripped', 'invalid'):
                await gw.send(client, {'type': 'drive', 'state': status,
                                       'reason': {'not_holder': 'non hai la guida',
                                                  'tripped': 'watchdog: rilascia e ripremi',
                                                  'invalid': 'messaggio non valido'}[status]})
        elif kind == 'drive_acquire':
            blockers = gw.drive_blockers()
            if blockers:
                await gw.send(client, {'type': 'drive', 'state': 'refused',
                                       'reason': ', '.join(blockers)})
                return
            ok, why = gw.teleop.acquire(client.id, now)
            await gw.send(client, {'type': 'drive', 'state': 'granted' if ok else 'busy',
                                   'reason': None if ok else why})
            if ok:
                gw.event('info', f'guida aperta ({client.user})')
        elif kind == 'drive_release':
            gw.publish_now(gw.teleop.release(client.id, now))
            await gw.send(client, {'type': 'drive', 'state': 'released', 'reason': None})
        elif kind == 'estop':
            asyncio.create_task(gw.do_estop(client.user))

    # ---- the PWA -------------------------------------------------------------------------------

    www = cfg['www_dir']
    if os.path.isdir(www):
        app.mount('/', StaticFiles(directory=www, html=True), name='www')
    else:
        @app.get('/')
        async def no_www():
            return HTMLResponse('<!doctype html><meta charset="utf-8"><title>gateway</title>'
                                f'<p>PWA non installata in {www}: npm run deploy.</p>',
                                status_code=503)
    return app


RANGE = re.compile(r'bytes=(\d*)-(\d*)$')


def ranged_file(path: str, header: Optional[str], media_type: str):
    """Serve a file with HTTP Range support (Safari's <video> needs 206 answers)."""
    size = os.path.getsize(path)
    start, end = 0, size - 1
    status = 200
    if header:
        m = RANGE.match(header.strip())
        if not m or (not m.group(1) and not m.group(2)):
            return Response(status_code=416, headers={'Content-Range': f'bytes */{size}'})
        if m.group(1):
            start = int(m.group(1))
            end = int(m.group(2)) if m.group(2) else size - 1
        else:
            start = max(0, size - int(m.group(2)))
        end = min(end, size - 1)
        if start > end:
            return Response(status_code=416, headers={'Content-Range': f'bytes */{size}'})
        status = 206

    def chunks():
        with open(path, 'rb') as f:
            f.seek(start)
            left = end - start + 1
            while left > 0:
                data = f.read(min(65536, left))
                if not data:
                    break
                left -= len(data)
                yield data
    headers = {'Accept-Ranges': 'bytes', 'Content-Length': str(end - start + 1)}
    if status == 206:
        headers['Content-Range'] = f'bytes {start}-{end}/{size}'
    return StreamingResponse(chunks(), status_code=status, media_type=media_type,
                             headers=headers)
