#!/usr/bin/env python3
"""The dodge ramp: climb d = 0, 1, 2, ... of the D(d, seed) family until the first level that fails.

    ramp.py --params tuning/baseline.yaml --label baseline [--d-start 0 --d-max 20] [--spectator]

A level d PASSES when every one of its seeds (default 0, 1, 2; one run each, repetition 0) is a good
run (metrics.good_enough: success, car-centre clearance >= R_safe_ref - 3 cm). The climb stops at the
first level that does not pass: d_fail (None if every level up to --d-max passed). One Isaac (layout
`dodge`) and one stack for the whole climb (the parameters do not change). Output: <out>/ramp.json
(per level: the runs' outcomes, margins, failure points) and <out>/ramp.md.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import common  # noqa: E402
import dodge  # noqa: E402
import metrics  # noqa: E402
from run_scenario import Session, log, run_reps  # noqa: E402

DEFAULT_SEEDS = (0, 1, 2)


def run_level(session: Session, params: dict, space: dict, out: str, d: int, seeds=DEFAULT_SEEDS,
              seq0: int = 1) -> dict:
    """Every seed of level d once; {'d', 'pass', 'good', 'n', 'runs': [...]}"""
    runs = []
    for i, seed in enumerate(seeds):
        sid = dodge.scenario_id(d, seed)
        runs.extend(run_reps(session, sid, params, space, [0], out, seq0=seq0 + i))
    good = sum(1 for r in runs if metrics.good_enough(r))
    invalid = sum(1 for r in runs if not r.get('valid'))
    return {'d': d, 'ramp': dodge.ramp(d), 'n': len(runs), 'good': good, 'invalid': invalid,
            'pass': good == len(runs) and invalid == 0,
            'runs': [summarise(r) for r in runs]}


def summarise(r: dict) -> dict:
    """The fields of a run the ramp report needs."""
    keys = ('scenario', 'rep', 'outcome', 'valid', 'score', 'min_margin_m', 'min_body_clearance_m',
            'max_lateral_error_m', 'steer_sign_changes', 'steer_sat', 'failure', 'solve_time_ms_p95',
            'solve_over_deadline', 'solve_failures', 'progress_m', 'time_to_goal_s', 'return')
    return {k: r.get(k) for k in keys}


def climb(session: Session, params: dict, space: dict, out: str, d_start: int = 0, d_max: int = 20,
          seeds=DEFAULT_SEEDS, levels=None, stop_on_fail: bool = True, patience: int = 0) -> dict:
    """Run levels d_start.. (or exactly `levels`) and return {'d_fail', 'd_pass_max', 'levels'}.

    d_fail is the first level run that did not pass. stop_on_fail: the climb ends there, or, with
    patience k, after k more failed levels (the A/B looks past the first failure to see the trend;
    d_fail stays the FIRST failed level)."""
    todo = list(levels) if levels is not None else list(range(d_start, d_max + 1))
    res = {'levels': [], 'd_fail': None, 'd_pass_max': None, 'seeds': list(seeds)}
    seq = int(time.time()) % 100000 * 100
    for d in todo:
        lv = run_level(session, params, space, out, d, seeds, seq0=seq)
        seq += len(seeds)
        res['levels'].append(lv)
        log(f'ramp d={d}: {lv["good"]}/{lv["n"]} good ' + ('PASS' if lv['pass'] else 'FAIL'))
        if lv['pass']:
            res['d_pass_max'] = d if res['d_pass_max'] is None else max(res['d_pass_max'], d)
        else:
            if res['d_fail'] is None:
                res['d_fail'] = d
            res['n_failed_levels'] = res.get('n_failed_levels', 0) + 1
            if stop_on_fail and res['n_failed_levels'] > patience:
                break
        _save(res, out)
    _save(res, out)
    return res


def _fmt(v, nd=2):
    return '' if v is None else (f'{v:.{nd}f}' if isinstance(v, float) else str(v))


def _save(res: dict, out: str) -> None:
    with open(os.path.join(out, 'ramp.json'), 'w', encoding='utf-8') as fh:
        json.dump(res, fh, indent=1, default=str)
    lines = [f'# Dodge ramp {res.get("label", "")}', '',
             f'- d_fail: **{res["d_fail"]}**, highest level passed: {res["d_pass_max"]}',
             f'- params: `{json.dumps(res.get("params", {}), sort_keys=True)}`', '',
             '| d | speed | levels | diag | run | outcome | margin m | body m | fail level | '
             'fail margin m | delta sat | rate sat |', '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for lv in res['levels']:
        rp = lv['ramp']
        for r in lv['runs']:
            f = r.get('failure') or {}
            sat = r.get('steer_sat') or {}
            lines.append(f'| {lv["d"]} | {rp["speed"]} | {rp["n_levels"]} | {rp["diag_frac"]} | '
                         f'{r["scenario"]} | {r["outcome"]} | {_fmt(r.get("min_margin_m"))} | '
                         f'{_fmt(r.get("min_body_clearance_m"))} | {_fmt(f.get("level"))} | '
                         f'{_fmt(f.get("margin_m"))} | {_fmt(sat.get("delta_frac"))} | '
                         f'{_fmt(sat.get("rate_frac"))} |')
    with open(os.path.join(out, 'ramp.md'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')


def _sigterm(*_):
    raise SystemExit('SIGTERM')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--params', required=True)
    ap.add_argument('--label', default='ramp')
    ap.add_argument('--d-start', type=int, default=0)
    ap.add_argument('--d-max', type=int, default=20)
    ap.add_argument('--levels', default=None, help='comma list of d to run (no stop at the first fail)')
    ap.add_argument('--seeds', default='0,1,2')
    ap.add_argument('--out', default=None)
    ap.add_argument('--domain', type=int, default=78)
    ap.add_argument('--ds-port', type=int, default=11878)
    ap.add_argument('--spectator', action='store_true')
    args = ap.parse_args(argv)
    # started detached (setsid nohup ... &) SIGINT is ignored: SIGTERM must still run the cleanup
    signal.signal(signal.SIGTERM, _sigterm)
    space = common.load_space()
    params = common.load_yaml(args.params) or {}
    problems = common.check_params(params, space)
    if problems:
        print('parameter set outside the space:\n  ' + '\n  '.join(problems), file=sys.stderr)
        return 2
    out = args.out or os.path.join(common.OUT_ROOT, 'dodge', f'ramp_{args.label}_' + time.strftime('%Y%m%d_%H%M'))
    os.makedirs(out, exist_ok=True)
    full = common.full_params(params, space)
    common.dump_yaml(full, os.path.join(out, 'params.yaml'))
    seeds = tuple(int(s) for s in args.seeds.split(','))
    session = Session(out, args.domain, args.ds_port, spectator=args.spectator)
    t0 = time.time()
    try:
        session.preflight(attach=False)
        session.start_ds()
        levels = [int(v) for v in args.levels.split(',')] if args.levels else None
        res = climb(session, params, space, out, args.d_start, args.d_max, seeds, levels=levels,
                    stop_on_fail=levels is None)
    finally:
        session.close()
    res.update(label=args.label, params=full, wall_s=round(time.time() - t0, 1),
               orphans=session.info['orphans'])
    _save(res, out)
    log(f'ramp {args.label}: d_fail {res["d_fail"]}, passed up to {res["d_pass_max"]}, '
        f'{res["wall_s"]} s, out {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
