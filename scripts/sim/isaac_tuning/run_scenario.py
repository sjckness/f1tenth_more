#!/usr/bin/env python3
"""Run one tuning scenario N times in Isaac with the car's stack and a parameter set; score each run.

    scripts/sim/isaac_tuning/run_scenario.py --scenario S1 --params tuning/baseline.yaml --repeat 3

Headless Isaac (scripts/sim/sim_isaac.sh isaac, layout 'tuning') and the stack
(sim_isaac.sh stack, sim:=isaac, the car's supervisor bringup) on this host, on a private ROS domain
and Discovery Server port. The parameter set reaches mpc_corr through two generated launch overlays
(common.write_overlays), never through stack_params.yaml, and is read back from the live nodes
before the first run (a mismatch makes the run 'invalid'). Every repetition: /sim/scenario (car
start + jitter, obstacles), /reset_all, a straight-drive mission, live recording (recorder.py), then
metrics.py -> run.json. No bags.

Output: <out>/<scenario>_r<k>/ (CSV streams, meta.json, run.json), <out>/session.json (wall times),
<out>/aggregate.json. Everything started here is stopped here, and the teardown checks that no
process of this workspace's install is left (session.json 'orphans', must be empty).
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import common  # noqa: E402
import metrics  # noqa: E402

ISAAC_CONTAINER = 'f1t_isaac'
SIM_ISAAC = os.path.join(common.REPO, 'scripts', 'sim', 'sim_isaac.sh')
ENV_SH = os.path.join(common.HERE, 'env.sh')
RECORDER = os.path.join(common.HERE, 'recorder.py')
STACK_ARGS = ('enable_intelligence:=false', 'enable_mission_logger:=false')


def log(msg: str) -> None:
    print(f'[isaac_tuning {time.strftime("%H:%M:%S")}] {msg}', flush=True)


def _bash(cmd: str, **kw) -> subprocess.Popen:
    """A bash command in the harness's ROS environment, in its own process group."""
    return subprocess.Popen(['bash', '-c', f'source "{ENV_SH}" && {cmd}'], start_new_session=True, **kw)


def install_processes(exclude_pgids=()) -> list[dict]:
    """Processes running anything from THIS workspace's install space (the stack), by /proc scan."""
    marker = (common.REPO + '/install/').encode()
    out = []
    me = os.getpid()
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as fh:
                cmd = fh.read()
            pgid = os.getpgid(int(d))
        except (OSError, ProcessLookupError):
            continue
        if marker in cmd and pgid not in exclude_pgids:
            out.append({'pid': int(d), 'pgid': pgid,
                        'cmd': cmd.replace(b'\0', b' ').decode(errors='replace')[:200]})
    return out


def pgid_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def stop_group(proc: subprocess.Popen, name: str, first=signal.SIGINT, grace=45.0) -> None:
    """Stop a process group: `first` to the group, up to `grace` for its LEADER to finish its own
    cleanup (sim_isaac.sh stops the launch and the container itself), then SIGTERM and SIGKILL
    to whatever is left in the group (5 s each)."""
    if proc is None:
        return
    pgid = proc.pid
    if pgid_alive(pgid):
        log(f'stopping {name}: {first.name} to process group {pgid}')
        try:
            os.killpg(pgid, first)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            log(f'{name}: leader still running after {grace:.0f} s')
    for sig in (signal.SIGTERM, signal.SIGKILL):
        if not pgid_alive(pgid):
            break
        log(f'stopping {name}: {sig.name} to what is left of process group {pgid}')
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            break
        t_end = time.time() + 5.0
        while time.time() < t_end and pgid_alive(pgid):
            time.sleep(0.25)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def container_running(name: str = ISAAC_CONTAINER) -> bool:
    r = subprocess.run(['docker', 'ps', '-q', '-f', f'name=^{name}$'], capture_output=True, text=True)
    return bool(r.stdout.strip())


def free_vram_mib() -> int | None:
    try:
        r = subprocess.run(['nvidia-smi', '--query-gpu=memory.total,memory.used',
                            '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=10)
        total, used = (int(v) for v in r.stdout.strip().splitlines()[0].split(','))
        return total - used
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        return None


class Session:
    """The Discovery Server, Isaac and the stack this harness started (and only those)."""

    def __init__(self, out: str, domain: int, ds_port: int, min_vram_mib: int = 3500):
        self.out = out
        self.domain, self.ds_port = domain, ds_port
        self.min_vram_mib = min_vram_mib
        self.ds = self.isaac = self.stack = None
        self.stack_params = None
        self.info = {'started': datetime.datetime.now().isoformat(timespec='seconds'),
                     'domain': domain, 'ds_port': ds_port, 'events': [], 'wall': {}, 'orphans': []}
        os.environ['ISAAC_TUNING_DOMAIN'] = str(domain)
        os.environ['ISAAC_TUNING_DS_PORT'] = str(ds_port)

    def event(self, what: str, **kw):
        self.info['events'].append({'t': round(time.time(), 1), 'what': what, **kw})
        log(what + (f' {kw}' if kw else ''))

    def save(self):
        with open(os.path.join(self.out, 'session.json'), 'w', encoding='utf-8') as fh:
            json.dump(self.info, fh, indent=1, default=str)

    # ------------------------------------------------------------ preflight
    def preflight(self, attach: bool) -> None:
        problems = []
        if os.path.exists('/tmp/component_supervisor.lock') and install_processes():
            problems.append('a stack of this workspace is already running')
        others = [p for p in install_processes()]
        if others and not attach:
            problems.append(f'{len(others)} processes of this install already running')
        if container_running() and not attach:
            problems.append(f'container {ISAAC_CONTAINER} already running (another session?)')
        st = os.statvfs('/')
        free_gb = st.f_bavail * st.f_frsize / 1e9
        if free_gb < 8:
            problems.append(f'only {free_gb:.1f} GB free on /')
        vram = free_vram_mib()
        self.info['preflight'] = {'free_disk_gb': round(free_gb, 1), 'free_vram_mib': vram}
        if not attach and vram is not None and vram < self.min_vram_mib:
            problems.append(f'only {vram} MiB of GPU memory free (< {self.min_vram_mib}): '
                            'another GPU user (freeparty-llm?) would make Isaac crash')
        if problems:
            raise SystemExit('preflight failed: ' + '; '.join(problems))
        self.event('preflight ok', **self.info['preflight'])

    # ------------------------------------------------------------ discovery server
    def start_ds(self) -> None:
        script = 'ros2 pkg prefix f1tenth_bringup'
        logf = open(os.path.join(self.out, 'discovery_server.log'), 'w')
        self.ds = _bash(f'exec python3 "$({script})/share/f1tenth_bringup/scripts/'
                        f'ensure_discovery_server.py" 127.0.0.1 {self.ds_port}',
                        stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        time.sleep(2.0)
        if self.ds.poll() is not None:
            raise SystemExit('Discovery Server did not start (see discovery_server.log)')
        self.event('discovery server', port=self.ds_port, pgid=self.ds.pid)

    # ------------------------------------------------------------ isaac
    def start_isaac(self, layout: str = 'tuning', scene: str = 'base_room') -> None:
        t0 = time.time()
        logf = open(os.path.join(self.out, 'isaac.log'), 'w')
        env = dict(os.environ, F1T_SIM_MIN_FREE_GB='10', F1TENTH_SIM_DISCOVERY_PORT=str(self.ds_port))
        self.isaac = subprocess.Popen(
            [SIM_ISAAC, 'isaac', '--domain', str(self.domain), '--scene', scene, '--layout', layout,
             '--set', 'sensors.stereo.right_camera=false'],
            stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, env=env,
            start_new_session=True)
        if not self.wait_clock(300.0):
            raise SystemExit('Isaac: no /clock within 300 s (see isaac.log)')
        self.info['wall']['isaac_start_s'] = round(time.time() - t0, 1)
        self.event('isaac up', wall_s=self.info['wall']['isaac_start_s'])

    def wait_clock(self, timeout: float) -> bool:
        p = _bash(f'timeout {int(timeout)} ros2 topic echo --once --no-daemon /clock '
                  'rosgraph_msgs/msg/Clock', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return p.wait() == 0

    def isaac_ok(self) -> bool:
        return container_running() and (self.isaac is None or self.isaac.poll() is None)

    # ------------------------------------------------------------ stack
    def start_stack(self, sim_overlay: str, sim_extra: str, tag: str) -> None:
        logf = open(os.path.join(self.out, f'stack_{tag}.log'), 'w')
        env = dict(os.environ, F1T_SIM_HOST_OVERLAY=sim_extra, F1T_SIM_CLOCK_WAIT='60',
                   F1TENTH_SIM_DISCOVERY_PORT=str(self.ds_port))
        self.stack_t0 = time.time()
        self.stack = subprocess.Popen(
            [SIM_ISAAC, 'stack', '--domain', str(self.domain), f'sim_overlay:={sim_overlay}',
             *STACK_ARGS], stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            env=env, start_new_session=True)
        self.event('stack starting', pgid=self.stack.pid, log=f'stack_{tag}.log')

    def stop_stack(self) -> list[dict]:
        if self.stack is None:
            return []
        t0 = time.time()
        # sim_isaac.sh's own cleanup: SIGINT to the launch (supervisor stops its components),
        # 30 s, then SIGTERM; SIGTERM to the script triggers it.
        stop_group(self.stack, 'stack', first=signal.SIGTERM, grace=60.0)
        self.stack = None
        left = []
        for _ in range(20):   # components may take a moment to exit after the launch
            left = install_processes(exclude_pgids=self._own_pgids())
            if not left:
                break
            time.sleep(1.0)
        if left:
            self.event('teardown: leftovers, killing', n=len(left))
            for p in left:
                try:
                    os.kill(p['pid'], signal.SIGKILL)
                except ProcessLookupError:
                    pass
        wall = round(time.time() - t0, 1)
        self.info['wall'].setdefault('stack_stop_s', []).append(wall)
        self.info['orphans'].extend(left)
        self.event('stack stopped', wall_s=wall, leftovers=len(left))
        return left

    def _own_pgids(self):
        return {p.pid for p in (self.ds, self.isaac) if p is not None}

    def close(self, keep_isaac: bool = False) -> None:
        self.stop_stack()
        if not keep_isaac:
            if container_running():
                subprocess.run(['docker', 'stop', '-t', '40', ISAAC_CONTAINER], capture_output=True)
            stop_group(self.isaac, 'isaac', first=signal.SIGTERM, grace=60.0)
            stop_group(self.ds, 'discovery server', first=signal.SIGINT, grace=5.0)
            left = install_processes()
            self.info['orphans'].extend(left)
            self.info['final_check'] = {'container_running': container_running(),
                                        'install_processes': left}
            self.event('session closed', container=container_running(), leftovers=len(left))
        self.info['ended'] = datetime.datetime.now().isoformat(timespec='seconds')
        self.save()


def run_reps(session: Session, sid: str, params: dict, space: dict, reps, set_dir: str,
             seq0: int = 1, fresh_stack_per_run: bool = False) -> list[dict]:
    """Run repetitions `reps` of scenario `sid`; returns the run.json dicts."""
    scn = common.load_scenario(sid)
    sim_overlay, sim_extra = common.write_overlays(params, space, set_dir)
    expect = common.expected_node_params(params, space)
    with open(os.path.join(set_dir, 'expect.json'), 'w', encoding='utf-8') as fh:
        json.dump(expect, fh, indent=1)
    results = []
    for k, rep in enumerate(reps):
        run_dir = os.path.join(set_dir, f'{sid}_r{rep}')
        os.makedirs(run_dir, exist_ok=True)
        inst = common.scenario_instance(scn, rep)
        with open(os.path.join(run_dir, 'instance.json'), 'w', encoding='utf-8') as fh:
            json.dump(inst, fh, indent=1)
        mission_path = os.path.join(run_dir, 'mission.json')
        with open(mission_path, 'w', encoding='utf-8') as fh:
            json.dump(common.mission_json(inst), fh, indent=1)
        for attempt in (1, 2):
            if not session.isaac_ok():
                session.event('isaac down: restarting', run=run_dir)
                session.stop_stack()
                session.start_isaac()
            if session.stack is None:
                session.start_stack(sim_overlay, sim_extra, tag=f'{sid}_r{rep}')
            t0 = time.time()
            rec = _bash(f'exec python3 "{RECORDER}" --out "{run_dir}" '
                        f'--instance "{run_dir}/instance.json" --mission "{mission_path}" '
                        f'--expect "{set_dir}/expect.json" --seq {seq0 + k}',
                        stdout=open(os.path.join(run_dir, 'recorder.log'), 'w'),
                        stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            try:
                code = rec.wait(timeout=600)
            except subprocess.TimeoutExpired:
                stop_group(rec, 'recorder', first=signal.SIGINT, grace=10.0)
                code = -1
            wall = round(time.time() - t0, 1)
            if code == 2 and attempt == 1:
                # stack never got ready: restart it (and Isaac if it died) once
                session.event('stack not ready: restarting it', run=run_dir)
                session.stop_stack()
                continue
            break
        run = metrics.compute(run_dir) if os.path.exists(os.path.join(run_dir, 'meta.json')) else {
            'scenario': sid, 'rep': rep, 'outcome': 'invalid', 'valid': False, 'score': None}
        run['recorder_code'] = code
        run['wall_recorder_s'] = wall
        run['isaac_alive_after'] = session.isaac_ok()
        if fresh_stack_per_run:
            run['teardown_leftovers'] = session.stop_stack()
        with open(os.path.join(run_dir, 'run.json'), 'w', encoding='utf-8') as fh:
            json.dump(run, fh, indent=1, default=str)
        log(f'{sid} r{rep}: {run.get("outcome")} score={run.get("score")} '
            f'margin={run.get("min_margin_m")} wall={wall}s')
        results.append(run)
        session.save()
    return results


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--scenario', required=True, choices=common.SCENARIO_IDS)
    ap.add_argument('--params', required=True, help='parameter set YAML ({name: value}, see space.yaml)')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--first-rep', type=int, default=0)
    ap.add_argument('--out', default=None, help='default output/isaac_tuning/<timestamp>_<scenario>')
    ap.add_argument('--domain', type=int, default=78)
    ap.add_argument('--ds-port', type=int, default=11878)
    ap.add_argument('--fresh-stack-per-run', action='store_true')
    ap.add_argument('--keep-isaac', action='store_true', help='leave Isaac + DS running at the end')
    args = ap.parse_args(argv)
    space = common.load_space()
    params = common.load_yaml(args.params) or {}
    problems = common.check_params(params, space)
    if problems:
        print('parameter set outside the space:\n  ' + '\n  '.join(problems), file=sys.stderr)
        return 2
    out = args.out or os.path.join(common.OUT_ROOT, time.strftime('%Y%m%d_%H%M%S') + f'_{args.scenario}')
    os.makedirs(out, exist_ok=True)
    common.dump_yaml(common.full_params(params, space), os.path.join(out, 'params.yaml'))
    session = Session(out, args.domain, args.ds_port)
    t0 = time.time()
    try:
        session.preflight(attach=False)
        session.start_ds()
        session.start_isaac()
        runs = run_reps(session, args.scenario, params, space,
                        range(args.first_rep, args.first_rep + args.repeat), out,
                        fresh_stack_per_run=args.fresh_stack_per_run)
    finally:
        session.close(keep_isaac=args.keep_isaac)
    agg = metrics.aggregate(runs)
    agg['wall_total_s'] = round(time.time() - t0, 1)
    agg['session'] = session.info.get('wall')
    agg['orphans'] = session.info['orphans']
    with open(os.path.join(out, 'aggregate.json'), 'w', encoding='utf-8') as fh:
        json.dump(agg, fh, indent=1, default=str)
    log(f'done: {agg["success"]}/{agg["n"]} success, score {agg["score_mean"]:.1f} '
        f'+- {agg["score_std"]:.1f}, orphans {len(session.info["orphans"])}, out {out}')
    return 0 if not session.info['orphans'] else 1


if __name__ == '__main__':
    sys.exit(main())
