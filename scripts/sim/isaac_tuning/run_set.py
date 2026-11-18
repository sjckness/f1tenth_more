#!/usr/bin/env python3
"""Run one parameter set on every scenario and append its row to tuning/history.csv.

    scripts/sim/isaac_tuning/run_set.py --params tuning/next_params.yaml --repeat 3

One Isaac session and ONE stack for the whole set (the parameters only change between sets, and
/sim/scenario + /reset_all put each repetition back at its start), S1-S4 x --repeat and S5 x
--s5-repeat. Used on its own or by night_loop.py, which passes its own Session.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import common  # noqa: E402
import metrics  # noqa: E402
from run_scenario import Session, log, run_reps  # noqa: E402

HISTORY = os.path.join(common.TUNING_DIR, 'history.csv')
OBSTACLE_SCENARIOS = ('S1', 'S2', 'S3', 'S4')


def params_hash(full: dict) -> str:
    return hashlib.sha1(json.dumps(full, sort_keys=True).encode()).hexdigest()[:10]


def git_commit() -> str:
    r = subprocess.run(['git', '-C', common.REPO, 'rev-parse', '--short', 'HEAD'],
                       capture_output=True, text=True)
    return r.stdout.strip()


def run_set(session: Session, params: dict, space: dict, out: str, repeat: int, s5_repeat: int,
            scenarios=common.SCENARIO_IDS, first_rep: int = 0) -> dict:
    """All scenarios of one set; returns {'per_scenario': {...}, 'runs': [...], ...}."""
    t0 = time.time()
    runs, per = [], {}
    seq = int(time.time()) % 100000 * 100
    try:
        for sid in scenarios:
            n = s5_repeat if sid == 'S5' else repeat
            r = run_reps(session, sid, params, space, range(first_rep, first_rep + n), out,
                         seq0=seq)
            seq += n
            runs.extend(r)
            per[sid] = metrics.aggregate(r)
    finally:
        session.stop_stack()
    obstacle_aggs = [per[s] for s in OBSTACLE_SCENARIOS if s in per]
    all_good = (all(a['good'] == a['n'] and a['n'] > 0 and a['invalid'] == 0 for a in obstacle_aggs)
                and ('S5' not in per or per['S5']['success'] == per['S5']['n'] > 0))
    result = {
        'params': common.full_params(params, space),
        'per_scenario': per,
        'set_score': (sum(a['score_agg'] for a in per.values()) / len(per)) if per else 0.0,
        'all_good': all_good,
        'n_runs': len(runs),
        'invalid': sum(1 for r in runs if not r.get('valid')),
        'wall_s': round(time.time() - t0, 1),
        'out': out,
    }
    with open(os.path.join(out, 'set.json'), 'w', encoding='utf-8') as fh:
        json.dump(result, fh, indent=1, default=str)
    return result


def append_history(result: dict, set_id: str, phase: str, path: str = HISTORY) -> None:
    row = {
        'set_id': set_id, 'phase': phase,
        'time': datetime.datetime.now().isoformat(timespec='seconds'),
        'commit': git_commit(), 'params_hash': params_hash(result['params']),
        'params': json.dumps(result['params'], sort_keys=True),
        'set_score': round(result['set_score'], 2), 'all_good': int(result['all_good']),
        'n_runs': result['n_runs'], 'invalid': result['invalid'], 'wall_s': result['wall_s'],
    }
    for sid in common.SCENARIO_IDS:
        a = result['per_scenario'].get(sid)
        for k in ('n', 'success', 'good', 'collision', 'estop', 'stuck', 'timeout'):
            row[f'{sid}_{k}'] = '' if a is None else a[k]
        for k in ('score_mean', 'score_std', 'min_margin_m', 'max_lateral_m', 'max_sign_changes'):
            v = None if a is None else a[k]
            row[f'{sid}_{k}'] = '' if v is None else round(v, 3)
    row['out'] = os.path.relpath(result['out'], common.REPO)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='', encoding='utf-8') as fh:
        w = csv.DictWriter(fh, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--params', required=True)
    ap.add_argument('--repeat', type=int, default=3)
    ap.add_argument('--s5-repeat', type=int, default=1)
    ap.add_argument('--scenarios', default=','.join(common.SCENARIO_IDS))
    ap.add_argument('--set-id', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--domain', type=int, default=78)
    ap.add_argument('--ds-port', type=int, default=11878)
    ap.add_argument('--no-history', action='store_true')
    ap.add_argument('--spectator', action='store_true',
                    help='third-person screenshots (Isaac scene.spectator) into <run>/frames/')
    args = ap.parse_args(argv)
    space = common.load_space()
    params = common.load_yaml(args.params) or {}
    problems = common.check_params(params, space)
    if problems:
        print('parameter set outside the space:\n  ' + '\n  '.join(problems), file=sys.stderr)
        return 2
    set_id = args.set_id or time.strftime('%Y%m%d_%H%M%S')
    out = args.out or os.path.join(common.OUT_ROOT, f'set_{set_id}')
    os.makedirs(out, exist_ok=True)
    common.dump_yaml(common.full_params(params, space), os.path.join(out, 'params.yaml'))
    session = Session(out, args.domain, args.ds_port, spectator=args.spectator)
    try:
        session.preflight(attach=False)
        session.start_ds()
        result = run_set(session, params, space, out, args.repeat, args.s5_repeat,
                         scenarios=tuple(args.scenarios.split(',')))
    finally:
        session.close()
    if not args.no_history:
        append_history(result, set_id, 'manual')
    log(f'set {set_id}: score {result["set_score"]:.1f}, all_good {result["all_good"]}, '
        f'{result["n_runs"]} runs in {result["wall_s"]} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
