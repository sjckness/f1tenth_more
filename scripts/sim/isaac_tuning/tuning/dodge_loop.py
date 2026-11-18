#!/usr/bin/env python3
"""The mpc-dodge tuning loop (phase 3): raise d_fail of the dodge ramp without regressions.

    setsid nohup python3 scripts/sim/isaac_tuning/tuning/dodge_loop.py --start <set.yaml> \
        --d-fail <d> --base-ab <ab dir>/<variant>/summary.json --hours 8 \
        > output/isaac_tuning/dodge/loop.log 2>&1 < /dev/null &

Extends night_loop.py (its Claude proposer, random fallback, GPU-service handling, the same
space.yaml contract) with the dodge family. One iteration = ONE candidate set:
  1. proposal: 1-2 parameters changed from the best set, with a one-line reason each (`claude -p`
     gets space.yaml, the loop's history and a DIAGNOSIS of the best set's failed runs: the level,
     the clearance, the time, steering saturation, solver time over the 100 ms deadline);
     invalid / repeated / failed proposals fall back to night_loop.propose_random;
  2. screening x3: the dodge levels d_fail-2, d_fail-1, d_fail (3 seeds each, the ramp's); the
     lower two must stay all-good (no regression on the ramp), and if d_fail passes the climb goes
     on up to the next failing level: the candidate's d_fail;
  3. only if d_fail went UP: confirmation x10, the highest newly passed level on seeds 3..12 (all
     good), then the regression set M1-M4 x 3 and S1-S4 x 3 + S5: no fewer good runs than the best
     set's (base-ab summary, then each accepted set's own);
  4. accepted -> the new best (tuning/dodge_best.yaml); otherwise the best set stays.
Stops: --hours, --max-iters (40), d_max reached, or --patience (5) iterations without improvement.
Status: <loop dir>/status.md and loop.json after every iteration; history in <loop dir>/history.csv.
"""
from __future__ import annotations

import argparse
import csv
import datetime
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
import dodge  # noqa: E402
import evaluate  # noqa: E402
import metrics  # noqa: E402
import ramp  # noqa: E402
from night_loop import gpu_service, propose_random  # noqa: E402
from run_scenario import Session, log, run_reps  # noqa: E402
from run_set import params_hash  # noqa: E402

BEST = os.path.join(common.TUNING_DIR, 'dodge_best.yaml')
TUNABLE = ('mpc_obs_body_probes', 'mpc_w_obs', 'mpc_w_corr', 'mpc_w_psi', 'mpc_w_term', 'mpc_w_du_delta', 'mpc_w_v',
           'class_margin_unknown_m', 'mpc_prediction_dt_s', 'mission_speed_mps',
           'mpc_obs_vel_trust', 'mpc_obs_wake_weight', 'mpc_obs_wake_horizon_s',
           'mpc_obs_vel_lead_s', 'mpc_return_gain', 'mpc_return_lookahead_ratio',
           'mpc_return_psi_scale', 'mpc_return_max_heading_deg', 'mpc_return_overshoot_max_m')

PROMPT = """You are tuning the corridor MPC (mpc_corr) of a 1:10 car in Isaac Sim so that it dodges
MOVING cylinders. Scenario family D(d): the car drives straight at 0.5 m/s; rows ("levels") of
cylinders (D0.3, sometimes D0.5) 1.5, 2.5, ... m ahead slide wall to wall (some diagonally); level d
sets the cylinder speed 0.2 + 0.1 d m/s, the share of diagonal levels min(0.5, 0.1 d) and the number
of levels 2 + d // 3. Every scenario is feasible for a clairvoyant (checked). A run is GOOD when the
mission succeeds and the car centre stays >= R_safe - 0.03 m from every cylinder (R_safe = r + 0.20
+ 0.12). d passes when its 3 seeds are good; d_fail = the first level that does not pass. The MPC
predicts each cylinder's disc along its estimated velocity (mpc_obs_vel_trust), may add a wake cost
ahead of its motion (mpc_obs_wake_weight), and returns to the move's line after a swerve
(mpc_return_*). Weights: w_obs obstacle soft cost, w_term terminal pull to the centreline target,
w_corr cross-track, w_psi terminal heading, w_du_delta steering rate, w_v speed tracking. The
static regressions S1-S5 and moving M1-M4 must not lose good runs.

Propose the NEXT candidate: change ONE or TWO parameters of the best set (only these: {tunable}),
inside [min, max] of the space below, each with a one-line reason. Answer with the reasons, then
ONE fenced ```yaml block holding the WHOLE set (flat mapping name: value, the best set with your
change applied), nothing after it. Never repeat a set already tried.

space.yaml:
{space}

Best set so far (d_fail {d_fail}):
{best}

Diagnosis of the best set's failed runs (level, margin to R_safe, body clearance, time after start,
steering saturation in the 1.5 s before, MPC CPU per tick, solver failures):
{diag}

Loop history (latest last):
{history}
"""


def diagnose(runs: list) -> str:
    """One line per failed run: where and how it failed."""
    lines = []
    for r in runs:
        if r is None or metrics.good_enough(r):
            continue
        f = r.get('failure') or {}
        sat = f.get('steer_sat_before') or {}
        lines.append(
            f"- {r.get('scenario')}: {r.get('outcome')}, level {f.get('level')}, margin "
            f"{_fmt(f.get('margin_m'))} m, body {_fmt(f.get('body_clearance_m'))} m, t "
            f"{_fmt(f.get('t_s'))} s, steer at limit {_fmt(sat.get('delta_frac'))}, rate at limit "
            f"{_fmt(sat.get('rate_frac'))}, mpc CPU {_fmt(r.get('mpc_cpu_ms_per_tick'))} ms/tick "
            f"(sim time stops during a solve, so CPU is the deadline proxy; budget 100 ms), "
            f"solve failures {r.get('solve_failures')}, max lateral "
            f"{_fmt(r.get('max_lateral_error_m'))} m")
    return '\n'.join(lines[:24]) or '(none)'


def _fmt(v, nd=2):
    return '-' if v is None else (f'{v:.{nd}f}' if isinstance(v, float) else str(v))


def load_run_dirs(set_dir: str, prefix: str) -> list:
    return evaluate._load_runs(set_dir, prefix) if os.path.isdir(set_dir) else []


def propose_claude(space_text: str, best: dict, d_fail, diag: str, history: str,
                   timeout: float = 240.0) -> tuple[dict, str]:
    prompt = PROMPT.format(tunable=', '.join(TUNABLE), space=space_text,
                           best=json.dumps(best, indent=1, sort_keys=True), d_fail=d_fail,
                           diag=diag, history=history or '(empty)')
    r = subprocess.run(['claude', '-p', '--tools', '', '--output-format', 'text',
                        '--no-session-persistence', prompt],
                       capture_output=True, text=True, timeout=timeout, cwd=common.TUNING_DIR)
    m = re.findall(r'```yaml\s*\n(.*?)```', r.stdout, flags=re.S)
    if r.returncode != 0 or not m:
        raise RuntimeError(f'claude rc={r.returncode}, no yaml block: {r.stdout[-400:]!r} '
                           f'{r.stderr[-300:]!r}')
    import yaml
    cand = yaml.safe_load(m[-1]) or {}
    if not isinstance(cand, dict):
        raise RuntimeError(f'not a mapping: {m[-1]!r}')
    return cand, r.stdout


def changed(a: dict, b: dict) -> dict:
    return {k: v for k, v in b.items() if a.get(k) != v}


def random_candidate(best: dict, space: dict, rng: random.Random) -> dict:
    """night_loop.propose_random restricted to TUNABLE (1-2 numeric parameters, 15 % of range)."""
    sub = {'parameters': {k: v for k, v in space['parameters'].items()
                          if k in TUNABLE and 'choices' not in v}}
    numeric = {k: v for k, v in best.items() if k in sub['parameters']}
    out = dict(best)
    out.update(propose_random(numeric, sub, rng))
    return out


def regression_ok(res: dict, ref: dict, tol: int = 1) -> tuple[bool, str]:
    """No fewer good runs on M and S than the reference summary, minus `tol` (run-to-run noise:
    the same set gave 7-9 good S runs of 13 and 3-4 good M runs of 12 in the phase 2 A/B)."""
    why = []
    for fam in ('M', 'S'):
        a, b = (res.get(fam) or {}), (ref.get(fam) or {})
        if b and a.get('good', 0) < b.get('good', 0) - tol:
            why.append(f'{fam} good {a.get("good")}/{a.get("n")} < {b.get("good")}/{b.get("n")}')
    return (not why), '; '.join(why)


def write_status(loop_dir: str, state: dict) -> None:
    lines = [f'# Dodge loop {os.path.basename(loop_dir)}', '',
             f'- updated {time.strftime("%Y-%m-%d %H:%M:%S")}',
             f'- iteration {state["iteration"]} / {state["max_iters"]}, phase {state["phase"]}',
             f'- best: d_fail {state["best_d_fail"]}, params {json.dumps(state["best"], sort_keys=True)}',
             f'- stop reason: {state.get("stop_reason") or "(running)"}', '',
             '| iter | change | why | d_fail before | d_fail after | confirm | regressions | '
             'accepted | proposer | wall s |', '|---|---|---|---|---|---|---|---|---|---|']
    for row in state['rows']:
        lines.append(f'| {row["iteration"]} | `{json.dumps(row["change"], sort_keys=True)}` | '
                     f'{row.get("why", "")} | {row["d_fail_before"]} | {row.get("d_fail_after")} | '
                     f'{row.get("confirm", "")} | {row.get("regression", "")} | '
                     f'{row["accepted"]} | {row["proposer"]} | {row.get("wall_s")} |')
    with open(os.path.join(loop_dir, 'status.md'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    with open(os.path.join(loop_dir, 'loop.json'), 'w', encoding='utf-8') as fh:
        json.dump(state, fh, indent=1, default=str)


def append_history(loop_dir: str, row: dict) -> None:
    path = os.path.join(loop_dir, 'history.csv')
    flat = {'iteration': row['iteration'], 'time': datetime.datetime.now().isoformat(timespec='seconds'),
            'change': json.dumps(row['change'], sort_keys=True), 'why': row.get('why', ''),
            'd_fail_before': row['d_fail_before'], 'd_fail_after': row.get('d_fail_after'),
            'confirm': row.get('confirm', ''), 'regression': row.get('regression', ''),
            'accepted': row['accepted'], 'proposer': row['proposer'], 'out': row.get('out', '')}
    new = not os.path.exists(path)
    with open(path, 'a', newline='', encoding='utf-8') as fh:
        w = csv.DictWriter(fh, fieldnames=list(flat))
        if new:
            w.writeheader()
        w.writerow(flat)


def history_text(loop_dir: str) -> str:
    path = os.path.join(loop_dir, 'history.csv')
    if not os.path.exists(path):
        return ''
    with open(path, encoding='utf-8') as fh:
        lines = fh.read().splitlines()
    return '\n'.join(lines[:1] + lines[-20:])


def screen(session: Session, params: dict, space: dict, it_dir: str, d_fail: int, d_max: int,
           seeds) -> dict:
    """Levels d_fail-2 .. d_fail, then climb while passing; returns {'d_fail', 'lower_ok', levels}."""
    lower = [d for d in (d_fail - 2, d_fail - 1) if d >= 0]
    res = {'levels': [], 'lower_ok': True}
    for d in lower:
        lv = ramp.run_level(session, params, space, it_dir, d, seeds)
        res['levels'].append(lv)
        if not lv['pass']:
            res['lower_ok'] = False
            res['d_fail'] = d
            return res
    d = d_fail
    while d <= d_max:
        lv = ramp.run_level(session, params, space, it_dir, d, seeds)
        res['levels'].append(lv)
        if not lv['pass']:
            break
        d += 1
    res['d_fail'] = d if d <= d_max else None
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--start', required=True, help='the starting set (YAML, space keys)')
    ap.add_argument('--d-fail', type=int, required=True, help="the starting set's d_fail")
    ap.add_argument('--base-ab', required=True, help="the starting set's evaluate.py summary.json")
    ap.add_argument('--hours', type=float, default=8.0)
    ap.add_argument('--max-iters', type=int, default=40)
    ap.add_argument('--patience', type=int, default=5)
    ap.add_argument('--d-max', type=int, default=20)
    ap.add_argument('--seeds', default='0,1,2')
    ap.add_argument('--confirm', type=int, default=10)
    ap.add_argument('--regression-tol', type=int, default=1,
                    help='good runs a candidate may lose per family (M, S) to the noise')
    ap.add_argument('--proposer', choices=('claude', 'random'), default='claude')
    ap.add_argument('--gpu-service', default='freeparty-llm.service')
    ap.add_argument('--domain', type=int, default=78)
    ap.add_argument('--ds-port', type=int, default=11878)
    ap.add_argument('--seed', type=int, default=23)
    args = ap.parse_args(argv)
    space = common.load_space()
    with open(common.SPACE_PATH, encoding='utf-8') as fh:
        space_text = fh.read()
    best = common.full_params(common.load_yaml(args.start) or {}, space)
    with open(args.base_ab, encoding='utf-8') as fh:
        ref = json.load(fh)
    seeds = tuple(int(s) for s in args.seeds.split(','))
    loop_dir = os.path.join(common.OUT_ROOT, 'dodge', 'loop_' + time.strftime('%Y%m%d_%H%M'))
    os.makedirs(loop_dir, exist_ok=True)
    common.dump_yaml(best, os.path.join(loop_dir, 'start.yaml'))
    state = {'iteration': 0, 'max_iters': args.max_iters, 'phase': 'start', 'best': best,
             'best_d_fail': args.d_fail, 'rows': [], 'stop_reason': None, 'gpu': {},
             'ref': args.base_ab}
    stop = {'flag': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    rng = random.Random(args.seed)
    tried = {params_hash(best)}
    t_end = time.time() + args.hours * 3600.0
    no_gain = 0
    best_dir = os.path.dirname(args.base_ab)
    state['gpu']['before'] = gpu_service('stop', args.gpu_service)
    session = Session(loop_dir, args.domain, args.ds_port)
    try:
        session.preflight(attach=False)
        session.start_ds()
        for it in range(1, args.max_iters + 1):
            if stop['flag'] or os.path.exists(os.path.join(loop_dir, 'STOP')):
                state['stop_reason'] = 'stopped by request'
                break
            if time.time() > t_end:
                state['stop_reason'] = f'time budget {args.hours} h'
                break
            if state['best_d_fail'] is None:
                state['stop_reason'] = f'd_max {args.d_max} passed'
                break
            state['iteration'], state['phase'] = it, 'propose'
            write_status(loop_dir, state)
            t0 = time.time()
            diag = diagnose(load_run_dirs(best_dir, 'D') + load_run_dirs(best_dir, 'M')
                            + load_run_dirs(best_dir, 'S'))
            cand, how, why = None, 'random', ''
            if args.proposer == 'claude':
                try:
                    c, text = propose_claude(space_text, best, state['best_d_fail'], diag,
                                             history_text(loop_dir))
                    full = common.full_params(c, space)
                    ch = changed(best, full)
                    bad = common.check_params(c, space)
                    if bad:
                        raise RuntimeError(f'outside the space: {bad}')
                    if not 1 <= len(ch) <= 2 or any(k not in TUNABLE for k in ch):
                        raise RuntimeError(f'changes {sorted(ch)}: need 1-2 of the tunable keys')
                    if params_hash(full) in tried:
                        raise RuntimeError('repeats a tried set')
                    cand, how = full, 'claude'
                    why = ' / '.join(ln.strip('-* ').strip() for ln in text.split('```')[0]
                                     .strip().splitlines() if ln.strip())[:300]
                    with open(os.path.join(loop_dir, f'it{it:02d}_proposal.md'), 'w',
                              encoding='utf-8') as fh:
                        fh.write(text)
                except Exception as exc:  # noqa: BLE001 - any proposer failure falls back
                    log(f'proposer failed ({exc}); random fallback')
            if cand is None:
                for _ in range(30):
                    cand = common.full_params(random_candidate(best, space, rng), space)
                    if params_hash(cand) not in tried and changed(best, cand):
                        break
                why = 'random perturbation of the best set'
            tried.add(params_hash(cand))
            ch = changed(best, cand)
            it_dir = os.path.join(loop_dir, f'it{it:02d}_{params_hash(cand)}')
            os.makedirs(it_dir, exist_ok=True)
            common.dump_yaml(cand, os.path.join(it_dir, 'params.yaml'))
            row = {'iteration': it, 'change': ch, 'why': why, 'd_fail_before': state['best_d_fail'],
                   'proposer': how, 'accepted': False, 'out': os.path.relpath(it_dir, common.REPO)}
            log(f'it{it:02d}: {how} {ch} -- {why[:120]}')
            # 2. screening
            state['phase'] = 'screen'
            write_status(loop_dir, state)
            sc = screen(session, cand, space, it_dir, state['best_d_fail'], args.d_max, seeds)
            row['d_fail_after'] = sc['d_fail']
            improved = sc['lower_ok'] and (sc['d_fail'] is None or sc['d_fail'] > state['best_d_fail'])
            if not sc['lower_ok']:
                row['regression'] = f'ramp level {sc["d_fail"]} fails'
            if improved:
                # 3. confirmation x10 on the highest newly passed level, then M/S regressions
                state['phase'] = 'confirm'
                write_status(loop_dir, state)
                top = (sc['d_fail'] - 1) if sc['d_fail'] is not None else args.d_max
                conf = ramp.run_level(session, cand, space, os.path.join(it_dir, 'confirm'), top,
                                      tuple(range(3, 3 + args.confirm)))
                row['confirm'] = f'd={top}: {conf["good"]}/{conf["n"]}'
                if conf['pass']:
                    state['phase'] = 'regression'
                    write_status(loop_dir, state)
                    res = evaluate.evaluate(session, f'it{it:02d}_reg', cand, space, it_dir, 0, 0,
                                            3, 3, seeds, families=('M', 'S'))
                    ok, why_reg = regression_ok(res, ref, args.regression_tol)
                    row['regression'] = 'none' if ok else why_reg
                    if ok:
                        row['accepted'] = True
                        # the reference stays the BETTER of the two, so the tolerance cannot
                        # ratchet the regressions down one run at a time
                        ref = {fam: max((res.get(fam) or {}), (ref.get(fam) or {}),
                                        key=lambda x: x.get('good', 0)) for fam in ('M', 'S')}
                        best = cand
                        best_dir = it_dir
                        state['best'], state['best_d_fail'] = best, sc['d_fail']
                        common.dump_yaml(best, BEST)
            no_gain = 0 if row['accepted'] else no_gain + 1
            row['wall_s'] = round(time.time() - t0, 1)
            state['rows'].append(row)
            append_history(loop_dir, row)
            write_status(loop_dir, state)
            if no_gain >= args.patience:
                state['stop_reason'] = f'{args.patience} iterations without improvement'
                break
        else:
            state['stop_reason'] = f'budget: {args.max_iters} iterations'
    except SystemExit as exc:
        state['stop_reason'] = f'stopped: {exc}'
        raise
    finally:
        session.close()
        state['gpu']['after'] = gpu_service('start', args.gpu_service)
        state['phase'] = 'done'
        write_status(loop_dir, state)
        log(f'dodge loop done: {state["stop_reason"]}; best d_fail {state["best_d_fail"]} '
            f'{json.dumps(state["best"], sort_keys=True)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
