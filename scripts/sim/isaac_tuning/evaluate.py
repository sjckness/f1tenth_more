#!/usr/bin/env python3
"""Evaluate parameter sets (variants) on the dodge ramp, M1-M4 and S1-S5: the A/B of mpc-dodge.

    evaluate.py --variants tuning/variants_phase2.yaml [--only baseline,alpha] [--m-repeat 3]

variants file: {name: {param: value, ...}} (space.yaml keys; {} = the shipped defaults). For each
variant, with one Isaac session for the whole file (the layout switches between `dodge` and
`tuning` as needed, and a new parameter set restarts the stack):
  * the dodge ramp from --d-start up (ramp.climb), stopping at the first level that fails: d_fail;
  * M1-M4 x --m-repeat and S1-S4 x --s-repeat + S5 x 1 (regression on the static ones).
Output: <out>/<variant>/... run directories, <out>/<variant>/summary.json, and <out>/ab.json /
<out>/ab.md with one row per variant (d_fail, good runs per family, margins, lateral, return).
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import common  # noqa: E402
import metrics  # noqa: E402
import ramp  # noqa: E402
from run_scenario import Session, log, run_reps  # noqa: E402


def family_summary(runs: list) -> dict:
    """Good / n, outcomes, worst margin, lateral and return numbers over a list of run.json."""
    valid = [r for r in runs if r.get('valid')]
    rets = [r.get('return') or {} for r in valid]
    back = [x['return_time_s'] for x in rets if x.get('passed') and x.get('return_time_s') is not None]
    passed = [x for x in rets if x.get('passed')]
    margins = [r['min_margin_m'] for r in valid if r.get('min_margin_m') is not None]
    return {
        'n': len(runs), 'valid': len(valid),
        'good': sum(1 for r in valid if metrics.good_enough(r)),
        'outcomes': {o: sum(1 for r in valid if r.get('outcome') == o)
                     for o in ('success', 'collision', 'estop', 'stuck', 'timeout')},
        'min_margin_m': min(margins) if margins else None,
        'mean_margin_m': statistics.fmean(margins) if margins else None,
        'max_lateral_m': max((r['max_lateral_error_m'] for r in valid), default=None),
        'mean_max_lateral_m': (statistics.fmean([r['max_lateral_error_m'] for r in valid])
                               if valid else None),
        'returned': f'{len(back)}/{len(passed)}',
        'mean_return_time_s': statistics.fmean(back) if back else None,
        'max_overshoot_m': max((x.get('overshoot_m', 0.0) for x in passed), default=None),
        'mean_final_lateral_m': (statistics.fmean([abs(x['final_lateral_m']) for x in passed])
                                 if passed else None),
        'max_sign_changes': max((r.get('steer_sign_changes', 0) for r in valid), default=None),
        'runs': [{k: r.get(k) for k in ('scenario', 'rep', 'outcome', 'min_margin_m',
                                         'max_lateral_error_m', 'steer_sign_changes', 'return',
                                         'failure')} for r in runs],
    }


def evaluate(session: Session, name: str, params: dict, space: dict, out: str, d_start: int,
             d_max: int, m_repeat: int, s_repeat: int, seeds, families=('D', 'M', 'S'),
             d_levels=None, patience: int = 0) -> dict:
    vdir = os.path.join(out, name)
    os.makedirs(vdir, exist_ok=True)
    full = common.full_params(params, space)
    common.dump_yaml(full, os.path.join(vdir, 'params.yaml'))
    t0 = time.time()
    res = {'variant': name, 'params': params}
    seq = int(time.time()) % 100000 * 100
    try:
        if 'D' in families:
            climb = ramp.climb(session, params, space, vdir, d_start, d_max, seeds, levels=d_levels,
                               stop_on_fail=d_levels is None, patience=patience)
            res['ramp'] = {k: climb[k] for k in ('d_fail', 'd_pass_max', 'seeds')}
            res['ramp']['levels'] = [{k: lv[k] for k in ('d', 'good', 'n', 'pass', 'invalid')}
                                     | {'runs': lv['runs']} for lv in climb['levels']]
            res['D'] = family_summary(_load_runs(vdir, 'D'))
        if 'M' in families:
            runs = []
            for sid in common.MOVING_IDS:
                runs.extend(run_reps(session, sid, params, space, range(m_repeat), vdir, seq0=seq))
                seq += m_repeat
            res['M'] = family_summary(runs)
        if 'S' in families:
            runs = []
            for sid in common.SCENARIO_IDS:
                n = 1 if sid == 'S5' else s_repeat
                runs.extend(run_reps(session, sid, params, space, range(n), vdir, seq0=seq))
                seq += n
            res['S'] = family_summary(runs)
    finally:
        session.stop_stack()
    res['wall_s'] = round(time.time() - t0, 1)
    with open(os.path.join(vdir, 'summary.json'), 'w', encoding='utf-8') as fh:
        json.dump(res, fh, indent=1, default=str)
    return res


def _load_runs(vdir: str, prefix: str) -> list:
    out = []
    for name in sorted(os.listdir(vdir)):
        p = os.path.join(vdir, name, 'run.json')
        if name.startswith(prefix) and os.path.exists(p):
            with open(p, encoding='utf-8') as fh:
                out.append(json.load(fh))
    return out


def _f(v, nd=2):
    return '-' if v is None else (f'{v:.{nd}f}' if isinstance(v, float) else str(v))


def write_table(results: list, out: str) -> None:
    lines = ['# mpc-dodge A/B', '',
             '| variant | d_fail | D good (levels run) | M good | S good | M min margin | S min margin | '
             'S mean max lat | S returned | S mean final lat | M/S max overshoot | wall s |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        D, M, S = r.get('D') or {}, r.get('M') or {}, r.get('S') or {}
        ov = max([x for x in (M.get('max_overshoot_m'), S.get('max_overshoot_m')) if x is not None],
                 default=None)
        lines.append(
            f'| {r["variant"]} | {_f((r.get("ramp") or {}).get("d_fail"))} | '
            f'{D.get("good", "-")}/{D.get("n", "-")} | {M.get("good", "-")}/{M.get("n", "-")} | '
            f'{S.get("good", "-")}/{S.get("n", "-")} | {_f(M.get("min_margin_m"))} | '
            f'{_f(S.get("min_margin_m"))} | {_f(S.get("mean_max_lateral_m"))} | '
            f'{S.get("returned", "-")} | {_f(S.get("mean_final_lateral_m"))} | {_f(ov)} | '
            f'{r.get("wall_s")} |')
    with open(os.path.join(out, 'ab.md'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    with open(os.path.join(out, 'ab.json'), 'w', encoding='utf-8') as fh:
        json.dump(results, fh, indent=1, default=str)


def _sigterm(*_):
    raise SystemExit('SIGTERM')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--variants', required=True)
    ap.add_argument('--only', default=None, help='comma list of variant names')
    ap.add_argument('--families', default='D,M,S')
    ap.add_argument('--d-start', type=int, default=0)
    ap.add_argument('--d-max', type=int, default=20)
    ap.add_argument('--d-levels', default=None, help='comma list: exactly these levels, no stop')
    ap.add_argument('--seeds', default='0,1,2')
    ap.add_argument('--patience', type=int, default=2,
                    help='levels to keep climbing after the first failed one (d_fail unchanged)')
    ap.add_argument('--m-repeat', type=int, default=3)
    ap.add_argument('--s-repeat', type=int, default=3)
    ap.add_argument('--out', default=None)
    ap.add_argument('--domain', type=int, default=78)
    ap.add_argument('--ds-port', type=int, default=11878)
    ap.add_argument('--spectator', action='store_true')
    args = ap.parse_args(argv)
    # started detached (setsid nohup ... &) SIGINT is ignored: SIGTERM must still run the cleanup
    signal.signal(signal.SIGTERM, _sigterm)
    space = common.load_space()
    variants = common.load_yaml(args.variants) or {}
    names = args.only.split(',') if args.only else list(variants)
    for n in names:
        problems = common.check_params(variants[n] or {}, space)
        if problems:
            print(f'{n}: outside the space:\n  ' + '\n  '.join(problems), file=sys.stderr)
            return 2
    out = args.out or os.path.join(common.OUT_ROOT, 'dodge', 'ab_' + time.strftime('%Y%m%d_%H%M'))
    os.makedirs(out, exist_ok=True)
    seeds = tuple(int(s) for s in args.seeds.split(','))
    levels = [int(v) for v in args.d_levels.split(',')] if args.d_levels else None
    session = Session(out, args.domain, args.ds_port, spectator=args.spectator)
    results = []
    try:
        session.preflight(attach=False)
        session.start_ds()
        for i, n in enumerate(names):
            fam = tuple(args.families.split(','))
            if i % 2 == 1:               # alternate the order: one layout switch per variant
                fam = tuple(reversed(fam))
            log(f'variant {n}: {variants[n]}')
            results.append(evaluate(session, n, variants[n] or {}, space, out, args.d_start,
                                    args.d_max, args.m_repeat, args.s_repeat, seeds, fam, levels,
                                    args.patience))
            write_table(results, out)
    finally:
        session.close()
    write_table(results, out)
    log(f'A/B done: {out}/ab.md')
    return 0


if __name__ == '__main__':
    sys.exit(main())
