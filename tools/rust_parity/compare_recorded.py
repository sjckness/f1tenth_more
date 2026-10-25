#!/usr/bin/env python3
"""
Compare a parity re-run with the recorded results (R0b, ROS snapshot move).

    python3 tools/rust_parity/compare_recorded.py <new dir> [--only a,b] [--json OUT]

<new dir> holds the JSON files the harnesses wrote in the re-run (names as in
output/rust_main/r0b_parity/). Each is checked against the recorded run of the
same harness:

  proximity   output/rust_main/parity_proximity.json (R0): seeded synthetic
              group identical, 0 trip / presence / unexplained stop
              differences, min_front bit-identical everywhere, the same
              fail-safe reasons; bags present in both runs: the same case count.
  costmap     output/rust_main/parity_costmap_boundary.json (R0): synthetic and
              malformed groups identical, 0 differences, the same Python
              exceptions and intended difference, front clearance 0 ulp, the
              largest constraint difference not above the recorded one; common
              bags: the same maps and poses.
  hard_stop   output/rust/parity_hard_stop.json (R4): 0 differences, gate PASS,
              every group with the same op / stop / engage / scan counts.
  mpc         output/rust/parity_mpc_solver.json (R2): every gate entry true and
              every L1 / L2 / L3 count equal.
  mpc_loop    output/rust_main/r2_closed_loop_parity.json: runs, pass,
              bit-identical runs and the golden counts equal.
  corridor    output/rust/parity_corridor.json (R1): rebuild / sweep / replay
              counts and gate equal.
  rigs        output/rust_main/r1_closed_loop_rigs.json: per implementation runs
              equal to the golden.
  wall        output/rust/wall_tracker/{numerics,fusion_cases,parity_*}.json (R6):
              0 mismatches with the same case counts; per bag the same cycles,
              messages and tracks compared, 0 differences of any kind.

Paths that name the worktree or scratch directory of a run (the golden file,
the capture directory) are compared by name. Exit 1 when any check fails; a
missing file is reported as a failed check, not skipped.
"""

import argparse
import json
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def load(path):
    """Read one JSON file."""
    with open(path, encoding='utf-8') as fh:
        return json.load(fh)


class Report:
    """Collects (section, check, ok, detail) rows."""

    def __init__(self):
        """Start with no rows."""
        self.rows = []

    def check(self, section, name, ok, detail=''):
        """Record one check."""
        self.rows.append({'section': section, 'check': name, 'ok': bool(ok),
                          'detail': str(detail)})

    @property
    def ok(self):
        """Return True when every check passed."""
        return all(r['ok'] for r in self.rows)


def rec(*parts):
    """Path of a recorded result under output/."""
    return os.path.join(REPO, 'output', *parts)


def common_bags(new, old, key, fields):
    """Count the bags in both lists and those equal in `fields`: (common, equal, first diff)."""
    o = {b[key]: b for b in old}
    common = equal = 0
    first = None
    for b in new:
        if b[key] in o:
            common += 1
            if all(b.get(f) == o[b[key]].get(f) for f in fields):
                equal += 1
            elif first is None:
                first = (b[key], {f: (b.get(f), o[b[key]].get(f)) for f in fields})
    return common, equal, first


def bag_detail(n, o, c, e, first):
    """Describe a common-bag comparison."""
    return (f"{len(n['bags'])} bags now, {len(o['bags'])} recorded, {c} common, {e} equal"
            f" {first or ''}")


def proximity(r, new_dir):
    """Front-proximity guard against R0's run."""
    s = 'proximity'
    n = load(os.path.join(new_dir, 'parity_proximity.json'))
    o = load(rec('rust_main', 'parity_proximity.json'))
    r.check(s, 'synthetic group identical',
            n['groups']['synthetic'] == o['groups']['synthetic'], n['groups']['synthetic'])
    for g in ('bag', 'bag+obstacle'):
        gg = n['groups'][g]
        ok = (gg['py_trips'] == gg['rs_trips'] and gg['trip_diff'] == 0
              and gg['stop_ne_py'] == 0)
        r.check(s, f'{g}: py trips == rust trips, 0 trip diffs, 0 stop != py', ok,
                f"{gg} (recorded {o['groups'][g]})")
    r.check(s, '0 trip / presence / unexplained stop differences',
            not n['trip_diffs'] and not n['min_presence_diffs']
            and not n['unexplained_stop_diffs'])
    r.check(s, 'min_front bit-identical on every compared case',
            n['min_bitexact'] == n['min_compared'] and n['max_min_abs_diff'] == 0.0,
            f"{n['min_bitexact']} / {n['min_compared']} "
            f"(recorded {o['min_bitexact']} / {o['min_compared']})")
    r.check(s, 'same fail-safe reasons as recorded',
            n['fail_safe_reasons'] == o['fail_safe_reasons'], n['fail_safe_reasons'])
    c, e, first = common_bags(n['bags'], o['bags'], 'bag', ('cases',))
    r.check(s, 'bags in both runs: same case count', c == len(o['bags']) and c == e,
            bag_detail(n, o, c, e, first))


def costmap(r, new_dir):
    """costmap_boundary_node against R0's run."""
    s = 'costmap_boundary'
    n = load(os.path.join(new_dir, 'parity_costmap_boundary.json'))
    o = load(rec('rust_main', 'parity_costmap_boundary.json'))
    for g in ('synthetic', 'malformed'):
        r.check(s, f'{g} group identical', n['groups'][g] == o['groups'][g], n['groups'][g])
    nb = n['groups']['bag']
    r.check(s, 'bag group: 0 differences, 0 raised', nb['diffs'] == 0 and nb['py_raised'] == 0,
            f"{nb} (recorded {o['groups']['bag']})")
    r.check(s, '0 differences beyond the tolerances', not n['diffs'])
    r.check(s, 'same Python exceptions and intended difference',
            n['py_raised'] == o['py_raised']
            and json.dumps(n['intended']) == json.dumps(o['intended']))
    r.check(s, 'front clearance 0 ulp', n['max_clearance_ulps'] == 0)
    r.check(s, 'largest normal / offset difference not above the recorded',
            n['max_normal_diff'] <= o['max_normal_diff']
            and n['max_offset_diff'] <= o['max_offset_diff'],
            f"{n['max_normal_diff']:.3g} / {n['max_offset_diff']:.3g}")
    share = n['bitexact_constraints'] / n['compared_constraints']
    oshare = o['bitexact_constraints'] / o['compared_constraints']
    r.check(s, 'bit-identical constraint share', abs(share - oshare) < 0.005,
            f"{n['bitexact_constraints']} / {n['compared_constraints']} = {share:.4%} "
            f'(recorded {oshare:.4%})')
    c, e, first = common_bags(n['bags'], o['bags'], 'bag', ('maps', 'poses'))
    r.check(s, 'bags in both runs: same maps and poses', c == len(o['bags']) and c == e,
            bag_detail(n, o, c, e, first))


def hard_stop(r, new_dir):
    """hard_stop parity against R4's run."""
    s = 'hard_stop'
    n = load(os.path.join(new_dir, 'parity_hard_stop.json'))
    o = load(rec('rust', 'parity_hard_stop.json'))
    r.check(s, 'gate PASS, 0 differences', n['gate'] == 'PASS' and n['total_diffs'] == 0,
            f"{n['total_ops']} ops (recorded {o['total_ops']})")
    keys = ('ops', 'diffs', 'stops', 'engages', 'scans', 'by_op')
    for g, og in o['groups'].items():
        ng = n['groups'].get(g)
        same = ng is not None and all(ng.get(k) == og.get(k) for k in keys)
        r.check(s, f'group {g}: same ops / stops / engages / scans', same,
                f"ops {ng and ng['ops']} stops {ng and ng['stops']} "
                f"(recorded {og['ops']} / {og['stops']})")


def mpc(r, new_dir):
    """MPC solver core layers L1-L3 against R2's run."""
    s = 'mpc_solver'
    n = load(os.path.join(new_dir, 'parity_mpc_solver.json'))
    o = load(rec('rust', 'parity_mpc_solver.json'))
    r.check(s, 'every gate entry true, same entries', n['pass'] and n['gate'] == o['gate'],
            f"{sum(n['gate'].values())}/{len(n['gate'])}")
    for layer in ('L1', 'L2', 'L3'):
        r.check(s, f'{layer} counts equal to recorded', n[layer] == o[layer],
                json.dumps(n[layer])[:300])

    def dirs(c):
        return dict(c, dirs=sorted(os.path.realpath(d) for d in c.get('dirs', [])))

    r.check(s, 'same fixture, sweep and capture (paths normalised)',
            n['fixture'] == o['fixture'] and n['sweep'] == o['sweep']
            and dirs(n['captures']) == dirs(o['captures']),
            {'fixture_ticks': n['fixture']['ticks'], 'sweep': n['sweep'],
             'capture_ticks': n['captures']['ticks']})


def mpc_loop(r, new_dir):
    """MPC closed-loop parity (L4) against R2's run."""
    s = 'mpc_closed_loop'
    n = load(os.path.join(new_dir, 'closed_loop_parity.json'))
    o = load(rec('rust_main', 'r2_closed_loop_parity.json'))
    keys = ('n_runs', 'pass_runs', 'bit_identical_runs', 'pass')

    def gold(g):
        # The golden file is read from the worktree that ran it: compare its name only.
        return dict(g, path=os.path.basename(g['path']))

    r.check(s, 'runs / pass / bit-identical / golden counts equal',
            all(n[k] == o[k] for k in keys) and gold(n['golden']) == gold(o['golden']),
            dict({k: n[k] for k in keys}, golden=gold(n['golden'])))
    diff = [k for k in o['runs'] if json.dumps(n['runs'].get(k), sort_keys=True)
            != json.dumps(o['runs'][k], sort_keys=True)]
    r.check(s, 'every run record equal to recorded', not diff, diff[:5])


def corridor(r, new_dir):
    """Corridor geometry parity against R1's run."""
    s = 'corridor'
    n = load(os.path.join(new_dir, 'parity_corridor.json'))
    o = load(rec('rust', 'parity_corridor.json'))
    r.check(s, 'gate pass', n['gate_pass'] and o['gate_pass'])
    for k in ('rebuilds', 'sweep'):
        r.check(s, f'{k} equal to recorded', n[k] == o[k], json.dumps(n[k])[:200])

    def by_corpus(replays):
        out = {}
        for x in replays:
            name = os.path.basename(os.path.dirname(x.get('corpus', '')))
            out[name] = {k: v for k, v in x.items() if k != 'corpus'}
        return out

    nr, orr = by_corpus(n.get('replay', [])), by_corpus(o.get('replay', []))
    r.check(s, 'corpus replays equal to recorded', nr == orr, f'{sorted(nr)}')


def rigs(r, new_dir):
    """Corridor closed-loop rigs against R1's run."""
    s = 'corridor_rigs'
    n = load(os.path.join(new_dir, 'closed_loop_rigs.json'))
    o = load(rec('rust_main', 'r1_closed_loop_rigs.json'))
    # The golden file is read from the worktree that ran it: compare its name only.
    r.check(s, 'same golden file, same run count, pass',
            os.path.basename(n['golden']) == os.path.basename(o['golden'])
            and n['runs'] == o['runs'] and n['pass'] and o['pass'], f"{n['runs']} runs")
    for impl, oi in o['impl'].items():
        ni = {k: v for k, v in n['impl'].get(impl, {}).items() if k != 'seconds'}
        oi = {k: v for k, v in oi.items() if k != 'seconds'}
        r.check(s, f'{impl}: solves, runs equal to the golden, corridor calls as recorded',
                ni == oi, ni)


WALL_FIELDS = ('cycles', 'stepped', 'cycles_with_diff', 'state_diffs', 'message_diffs',
               'debug_diffs', 'max_abs_diff', 'messages_compared', 'tracks_compared',
               'max_tracks', 'confirmed_track_steps', 'front_steps', 'rect_cycles',
               'door_cycles')


def wall(r, new_dir):
    """Wall tracker numerics, fusion cases and bags against R6's runs."""
    s = 'wall_tracker'
    base = rec('rust', 'wall_tracker')
    n = load(os.path.join(new_dir, 'wt_numerics.json'))
    o = load(os.path.join(base, 'numerics.json'))
    same = all(n[k] == o[k] for k in o if isinstance(o[k], dict))
    r.check(s, 'numerics: 0 mismatches, same cases per primitive',
            n['total_mismatches'] == 0 and same,
            {k: n[k]['cases'] for k in n if isinstance(n[k], dict)})
    n = load(os.path.join(new_dir, 'wt_fusion_cases.json'))
    o = load(os.path.join(base, 'fusion_cases.json'))
    r.check(s, 'fusion cases: 0 mismatches, same results and coverage',
            n['total_mismatches'] == 0 and n['results'] == o['results']
            and n['coverage'] == o['coverage'],
            {k: v['cases'] for k, v in n['results'].items()})
    for tag in ('car', 'every', 'full', 'doors'):
        n = load(os.path.join(new_dir, f'wt_parity_{tag}.json'))['runs']
        o = load(os.path.join(base, f'parity_{tag}.json'))['runs']
        okey = {x['bag']: x for x in o}
        bad = []
        for x in n:
            y = okey.get(x['bag'])
            if (y is None or any(x.get(f) != y.get(f) for f in WALL_FIELDS)
                    or x['cycles_with_diff'] != 0):
                bad.append(os.path.basename(x['bag']))
        steps = sum(x['stepped'] for x in n)
        msgs = sum(x['messages_compared'] for x in n)
        r.check(s, f'bags {tag}: every run equal to recorded, 0 differences',
                len(n) == len(o) and not bad,
                f'{len(n)} runs, {steps} steps, {msgs} messages compared; bad {bad}')


SECTIONS = {'proximity': proximity, 'costmap': costmap, 'hard_stop': hard_stop, 'mpc': mpc,
            'mpc_loop': mpc_loop, 'corridor': corridor, 'rigs': rigs, 'wall': wall}


def main(argv=None):
    """Run the selected comparisons; exit 1 on any difference."""
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('new_dir')
    ap.add_argument('--only', default=','.join(SECTIONS))
    ap.add_argument('--json')
    a = ap.parse_args(argv)
    r = Report()
    for name in a.only.split(','):
        try:
            SECTIONS[name](r, a.new_dir)
        except (OSError, KeyError, TypeError, ValueError) as e:
            r.check(name, 'comparison ran', False, f'{type(e).__name__}: {e}')
    for row in r.rows:
        detail = f"  [{row['detail']}]" if row['detail'] else ''
        print(f"{'PASS' if row['ok'] else 'FAIL'}  {row['section']:16s} {row['check']}{detail}")
    print('ALL EQUAL TO RECORDED' if r.ok else 'DIFFERENCES FROM RECORDED')
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump({'ok': r.ok, 'rows': r.rows}, fh, indent=1)
    return 0 if r.ok else 1


if __name__ == '__main__':
    sys.exit(main())
