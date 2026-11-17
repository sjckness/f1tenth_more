#!/usr/bin/env python3
"""The night loop: run a parameter set in Isaac, score it, have Claude propose the next one, repeat.

    setsid nohup scripts/sim/isaac_tuning/tuning/night_loop.py --max-iters 10 \
        > output/isaac_tuning/night.log 2>&1 &

Run it detached (setsid nohup): it must not die with the Claude Code session that started it.
Stop it early with `touch output/isaac_tuning/night_<date>/STOP` (it finishes the current set and
cleans up) or SIGTERM to its process group.

Each iteration (budget --max-iters):
  1. screening: run_set.run_set on S1-S4 x --repeat (default 3) and S5 x 1, one stack per set;
     a row in tuning/history.csv (phase 'screen');
  2. if every run of the set is good (success, car-centre clearance >= R_safe_ref - tolerance;
     S5 success): confirmation, S1-S4 x --confirm-repeat (default 10), repetitions not used in
     screening; if all good again: STOP, the set is written to tuning/best.yaml;
  3. otherwise the proposer writes tuning/next_params.yaml: `claude -p` (no tools) gets
     space.yaml, history.csv and the last set's per-scenario numbers and answers with a YAML
     block; tuning/check_params.py's rules reject anything outside the space. If Claude fails
     or proposes something invalid, a random perturbation of the best set so far (inside the
     space) is used instead, and the fallback is logged.
"Good enough", not millimetric: the bar is the tolerance, not the score; the score only ranks.

Isaac crash: the run is retried once by run_scenario.run_reps after restarting Isaac; three sets
in a row with invalid runs stop the loop. GPU: freeparty-llm.service (5.4 GB) is stopped for the
night and started again at the end (Andreas's call, 2026-11-18; --gpu-service '' to skip).
Status: <night>/status.md after every set, <night>/loop.json, history.csv.
"""
from __future__ import annotations

import argparse
import json
import os
import random
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import common  # noqa: E402
import metrics  # noqa: E402
from run_scenario import Session, log  # noqa: E402
from run_set import append_history, params_hash, run_set  # noqa: E402

NEXT = os.path.join(common.TUNING_DIR, 'next_params.yaml')
BEST = os.path.join(common.TUNING_DIR, 'best.yaml')
HISTORY = os.path.join(common.TUNING_DIR, 'history.csv')
PROMPT_HEAD = """You are tuning the corridor MPC (mpc_corr) of a 1:10 car so that it passes an obstacle
ahead CONSISTENTLY in Isaac Sim (same outcome on every repetition and small start-pose jitter).
The MPC's target is always the centerline lookahead point; avoidance is the solver's soft obstacle
cost (w_obs, on probe points ahead of the nose) traded against the terminal pull (w_term), the
cross-track pull (w_corr) and the steering-rate cost (w_du_delta). Scenarios: S1 cylinder D0.5 on
the line 1.5 m ahead; S2 D1.0 at 1.5 m; S3 D0.5 at 1.5 m, +-0.15 m off the line; S4 D0.5 at 1.0 m;
S5 no obstacle. Mission: 4 m straight at 0.5 m/s, 20 s timeout. A run is GOOD when it succeeds and
the car centre stays >= R_safe_ref - {tol} m from the obstacle (R_safe_ref = r + 0.20 + 0.12).
Collision / e-stop are hard failures. Score 0..100 ranks sets (mean - std per scenario).

Propose the NEXT parameter set to try. Rules:
- only keys of the space below, each within [min, max]; omitted keys take their default;
- change few things at a time and say why in at most 8 short lines, then give the set as ONE
  fenced ```yaml block (a flat mapping name: number) and nothing after it;
- never repeat a set already in the history (same values).

space.yaml:
{space}

history.csv (latest rows last):
{history}

Last set, per scenario:
{last}
"""


def sh(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def gpu_service(action: str, name: str) -> str:
    if not name:
        return 'skipped'
    r = sh(['systemctl', '--user', action, name], timeout=60)
    state = sh(['systemctl', '--user', 'is-active', name]).stdout.strip()
    return f'{action} rc={r.returncode} now {state}'


def propose_claude(space_text: str, last: dict, tol: float, timeout: float = 600.0) -> tuple[dict, str]:
    hist = ''
    if os.path.exists(HISTORY):
        with open(HISTORY, encoding='utf-8') as fh:
            lines = fh.read().splitlines()
        hist = '\n'.join(lines[:1] + lines[-20:])
    summary = {sid: {k: a[k] for k in ('n', 'success', 'good', 'collision', 'estop', 'stuck',
                                       'timeout', 'score_mean', 'score_std', 'min_margin_m',
                                       'max_lateral_m', 'max_sign_changes')}
               for sid, a in last['per_scenario'].items()}
    prompt = PROMPT_HEAD.format(tol=tol, space=space_text, history=hist or '(empty)',
                                last=json.dumps({'params': last['params'], 'per_scenario': summary},
                                                indent=1, default=str))
    r = subprocess.run(['claude', '-p', '--tools', '', '--output-format', 'text',
                        '--no-session-persistence', prompt],
                       capture_output=True, text=True, timeout=timeout, cwd=common.TUNING_DIR)
    text = r.stdout
    m = re.findall(r'```yaml\s*\n(.*?)```', text, flags=re.S)
    if r.returncode != 0 or not m:
        raise RuntimeError(f'claude rc={r.returncode}, no yaml block: {text[-400:]!r} {r.stderr[-300:]!r}')
    import yaml
    params = yaml.safe_load(m[-1]) or {}
    if not isinstance(params, dict):
        raise RuntimeError(f'not a mapping: {m[-1]!r}')
    return {k: float(v) for k, v in params.items()}, text


def propose_random(best: dict, space: dict, rng: random.Random) -> dict:
    """Perturb the best set so far: 1-2 parameters, 15 % of their range, clipped to the space."""
    entries = space['parameters']
    out = dict(best)
    for name in rng.sample(sorted(entries), k=rng.choice((1, 2))):
        e = entries[name]
        span = float(e['max']) - float(e['min'])
        v = float(out.get(name, e['default'])) + rng.gauss(0.0, 0.15 * span)
        out[name] = round(min(float(e['max']), max(float(e['min']), v)), 4)
    return out


def write_status(night: str, state: dict) -> None:
    lines = [f'# Night loop {os.path.basename(night)}', '',
             f'- updated {time.strftime("%Y-%m-%d %H:%M:%S")}',
             f'- iteration {state["iteration"]} / {state["max_iters"]}, phase {state["phase"]}',
             f'- best so far: score {state.get("best_score")} params {state.get("best_params")}',
             f'- stop reason: {state.get("stop_reason") or "(running)"}', '',
             '| iter | phase | params hash | set score | all good | wall s | proposer |',
             '|---|---|---|---|---|---|---|']
    for row in state['sets']:
        lines.append(f'| {row["iteration"]} | {row["phase"]} | {row["hash"]} | {row["score"]:.1f} | '
                     f'{row["all_good"]} | {row["wall_s"]} | {row.get("proposer", "")} |')
    with open(os.path.join(night, 'status.md'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    with open(os.path.join(night, 'loop.json'), 'w', encoding='utf-8') as fh:
        json.dump(state, fh, indent=1, default=str)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--max-iters', type=int, default=None, help='default space.yaml stop.max_iterations')
    ap.add_argument('--start', default=os.path.join(common.TUNING_DIR, 'baseline.yaml'))
    ap.add_argument('--repeat', type=int, default=None)
    ap.add_argument('--confirm-repeat', type=int, default=None)
    ap.add_argument('--proposer', choices=('claude', 'random'), default='claude')
    ap.add_argument('--gpu-service', default='freeparty-llm.service')
    ap.add_argument('--domain', type=int, default=78)
    ap.add_argument('--ds-port', type=int, default=11878)
    ap.add_argument('--seed', type=int, default=17)
    ap.add_argument('--spectator', action='store_true',
                    help='third-person frames of every run (<run>/frames), for the figures')
    args = ap.parse_args(argv)
    space = common.load_space()
    stop_cfg = space.get('stop') or {}
    max_iters = args.max_iters or int(stop_cfg.get('max_iterations', 10))
    repeat = args.repeat or int(stop_cfg.get('screening_repeats', 3))
    confirm = args.confirm_repeat or int(stop_cfg.get('confirm_repeats', 10))
    tol = float(stop_cfg.get('margin_tolerance_m', 0.03))
    with open(common.SPACE_PATH, encoding='utf-8') as fh:
        space_text = fh.read()
    night = os.path.join(common.OUT_ROOT, 'night_' + time.strftime('%Y%m%d_%H%M'))
    os.makedirs(night, exist_ok=True)
    rng = random.Random(args.seed)
    state = {'iteration': 0, 'max_iters': max_iters, 'phase': 'start', 'sets': [], 'best_score': None,
             'best_params': None, 'stop_reason': None, 'gpu': {}}
    stop_flag = {'stop': False}
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.update(stop=True))
    state['gpu']['before'] = gpu_service('stop', args.gpu_service)
    log(f'night loop in {night}: GPU service {state["gpu"]["before"]}')
    params = common.load_yaml(args.start) or {}
    session = Session(night, args.domain, args.ds_port, spectator=args.spectator)
    invalid_streak = 0
    tried = set()
    try:
        session.preflight(attach=False)
        session.start_ds()
        session.start_isaac()
        for it in range(1, max_iters + 1):
            if stop_flag['stop'] or os.path.exists(os.path.join(night, 'STOP')):
                state['stop_reason'] = 'stopped by request'
                break
            state['iteration'], state['phase'] = it, 'screen'
            problems = common.check_params(params, space)
            if problems:
                state['stop_reason'] = f'invalid set reached the loop: {problems}'
                break
            full = common.full_params(params, space)
            h = params_hash(full)
            tried.add(h)
            set_dir = os.path.join(night, f'it{it:02d}_{h}')
            os.makedirs(set_dir, exist_ok=True)
            common.dump_yaml(full, os.path.join(set_dir, 'params.yaml'))
            res = run_set(session, params, space, set_dir, repeat, 1)
            append_history(res, f'it{it:02d}', 'screen')
            row = {'iteration': it, 'phase': 'screen', 'hash': h, 'score': res['set_score'],
                   'all_good': res['all_good'], 'wall_s': res['wall_s'],
                   'proposer': state.get('last_proposer', 'start')}
            state['sets'].append(row)
            invalid_streak = invalid_streak + 1 if res['invalid'] else 0
            if state['best_score'] is None or res['set_score'] > state['best_score']:
                state['best_score'], state['best_params'] = res['set_score'], full
            write_status(night, state)
            if invalid_streak >= 3:
                state['stop_reason'] = 'three sets in a row with invalid runs (sim or stack trouble)'
                break
            if res['all_good']:
                state['phase'] = 'confirm'
                write_status(night, state)
                conf_dir = os.path.join(night, f'it{it:02d}_{h}_confirm')
                os.makedirs(conf_dir, exist_ok=True)
                conf = run_set(session, params, space, conf_dir, confirm, 1,
                               scenarios=('S1', 'S2', 'S3', 'S4', 'S5'), first_rep=repeat)
                append_history(conf, f'it{it:02d}', 'confirm')
                state['sets'].append({'iteration': it, 'phase': 'confirm', 'hash': h,
                                      'score': conf['set_score'], 'all_good': conf['all_good'],
                                      'wall_s': conf['wall_s'], 'proposer': ''})
                if conf['all_good']:
                    common.dump_yaml(full, BEST)
                    state['stop_reason'] = f'set {h} confirmed: every run good on S1-S4 x {confirm}'
                    write_status(night, state)
                    break
            # propose the next set
            state['phase'] = 'propose'
            write_status(night, state)
            nxt, how = None, None
            if args.proposer == 'claude':
                try:
                    cand, text = propose_claude(space_text, res, tol)
                    with open(os.path.join(set_dir, 'proposal.md'), 'w', encoding='utf-8') as fh:
                        fh.write(text)
                    probs = common.check_params(cand, space)
                    if probs:
                        raise RuntimeError(f'outside the space: {probs}')
                    if params_hash(common.full_params(cand, space)) in tried:
                        raise RuntimeError('repeats a set already tried')
                    nxt, how = cand, 'claude'
                except Exception as exc:  # noqa: BLE001 - any proposer failure falls back
                    log(f'proposer failed ({exc}); random fallback')
                    with open(os.path.join(set_dir, 'proposal_error.txt'), 'w', encoding='utf-8') as fh:
                        fh.write(str(exc))
            if nxt is None:
                for _ in range(20):
                    cand = propose_random(state['best_params'], space, rng)
                    if params_hash(common.full_params(cand, space)) not in tried:
                        break
                nxt, how = cand, 'random'
            common.dump_yaml(nxt, NEXT)
            state['last_proposer'] = how
            params = nxt
        else:
            state['stop_reason'] = f'budget: {max_iters} iterations'
    except SystemExit as exc:
        state['stop_reason'] = f'stopped: {exc}'
        raise
    finally:
        session.close()
        state['gpu']['after'] = gpu_service('start', args.gpu_service)
        state['phase'] = 'done'
        write_status(night, state)
        log(f'night loop done: {state["stop_reason"]}; best {state["best_score"]} '
            f'{state["best_params"]}; GPU service {state["gpu"]["after"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
