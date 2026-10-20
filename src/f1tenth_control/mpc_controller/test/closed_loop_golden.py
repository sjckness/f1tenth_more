"""
Digests of the existing closed-loop scenarios, for the allow_reverse pin.

Not a test module (no test_ prefix): test_reverse_flag_off_identity.py imports
it, and running it as a script writes the golden file it compares against.

WHAT IS RECORDED. Every solve the existing rigs make -- the object approach
rows (test_object_approach_closed_loop.run_approach) and the person-pass rows
(test_person_pass_closed_loop.run_pass) -- plus a few direct solves that the
rigs do not reach (live soft boundaries, the 0.2 s prediction step with a
0.1 s control period, the SLSQP rollback). Each solve contributes the exact
bytes of its inputs (x0, last_u, pref_nom) and outputs (u0, zopt, x_pred, the
reported cost) to one SHA-256 per run. Equal digests mean every tick of every
run is bit-for-bit the same, inputs included, so a change anywhere upstream
(corridor, target, warm start) shows up as well as a change in the solver.

The golden file was written on 868ec8a, before the reverse work touched
mpc_solver.py, with:

    python3 test/closed_loop_golden.py --write test/data/closed_loop_golden.json
"""

import argparse
import contextlib
import hashlib
import json
import math
import os
import sys

from mpc_controller import mpc_solver
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_object_approach_closed_loop as approach_rig  # noqa: E402
import test_person_pass_closed_loop as person_rig  # noqa: E402

GOLDEN_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'data', 'closed_loop_golden.json')

APPROACH_RUNS = tuple(
    (f'approach/{name}/{label}', dict(kwargs, obstacle_r=obstacle_r, standoff=1.0))
    for name, kwargs in approach_rig.SCENARIOS.items()
    for label, obstacle_r in (('none', None), ('footprint', 0.25)))

PERSON_RUNS = tuple(
    (f'person/{label}/{offset:.1f}', {'obstacle_r': r, 'offset': offset})
    for label, r in (('legacy', 0.875), ('footprint', 0.25))
    for offset in (0.0, 0.3, 0.6))


class _Digest:
    """Running SHA-256 over the exact float64 bytes of each solve."""

    def __init__(self):
        self.h = hashlib.sha256()
        self.n = 0

    def add(self, *arrays):
        for arr in arrays:
            self.h.update(np.ascontiguousarray(np.asarray(arr, dtype=float)).tobytes())

    def record(self, kwargs, u0, info):
        self.n += 1
        self.add(kwargs['x0'], kwargs['last_u'], kwargs['pref_nom'], u0,
                 info.get('zopt', []), info.get('x_pred', []),
                 [info.get('cost', math.nan)])

    def hexdigest(self):
        return self.h.hexdigest()


@contextlib.contextmanager
def _patched(module, solve):
    original = module.solve_mpc_step
    module.solve_mpc_step = solve
    try:
        yield
    finally:
        module.solve_mpc_step = original


def _recording(solve, digest):
    def wrapper(**kwargs):
        u0, info = solve(**kwargs)
        digest.record(kwargs, u0, info)
        return u0, info
    return wrapper


def _direct_cases():
    """Solves the rigs never make: boundaries, ts_pred 0.2, SLSQP."""
    corridor_fake = approach_rig._ObjectMPC(standoff=1.0, speed=0.5)
    corridor_fake.goal_distance = 4.0
    corridor_fake.goal_start_xy = (0.0, 0.0)
    corridor_fake.goal_anchor_odom = (0.0, 0.0, 0.0)
    corridor_fake.psi_init_corridor = 0.0
    from mpc_controller.MPC_corr import MPCController
    out = []
    for i, (x0, boundaries, ts, solver) in enumerate((
            ((0.0, 0.0, 0.0, 0.4), [(0.0, 1.0, 0.7), (0.0, -1.0, 0.7)], 0.1, 'rti'),
            ((0.3, 0.1, 0.05, 0.5), [(0.0, 1.0, 0.45), (1.0, 0.0, 2.0)], 0.1, 'rti'),
            ((0.0, 0.0, 0.0, 0.4), [], 0.2, 'rti'),
            ((0.0, 0.0, 0.0, 0.4), [], 0.1, 'slsqp'))):
        x = np.asarray(x0, dtype=float)
        corridor = MPCController.build_straight_corridor(corridor_fake, x)
        corridor['obstacles_world'] = [(1.5, 0.2, 0.25)]
        corridor['d_safe'] = approach_rig.DMIN
        corridor['car_radius'] = approach_rig.CAR_RADIUS
        corridor['avoidance_margin'] = approach_rig.AVOIDANCE_MARGIN
        pref_nom = MPCController.compute_local_target(corridor_fake, x, corridor)
        out.append((f'direct/{i}', {
            'x0': x, 'last_u': np.array([0.02, 0.1]), 'pref_nom': pref_nom,
            'corridor': corridor, 'horizon': approach_rig.HORIZON, 'ts': ts,
            'params': approach_rig.PARAMS, 'limits': approach_rig.LIMITS,
            'weights': dict(approach_rig.WEIGHTS),
            'obstacles': corridor['obstacles_world'], 'dmin': approach_rig.DMIN,
            'vdes': 0.5, 'solver': solver, 'boundaries': boundaries,
            'control_period': 0.1}))
    return out


def digests(solve=None, runs=None):
    """
    {run name: (n_solves, sha256)} with every rig solve routed through `solve`.

    `solve` defaults to mpc_solver.solve_mpc_step as it is NOW (looked up at
    call time, so a monkeypatched module is honoured). `runs` filters by name
    prefix.
    """
    solve = solve or (lambda **kw: mpc_solver.solve_mpc_step(**kw))
    out = {}

    def wanted(name):
        return runs is None or any(name.startswith(r) for r in runs)

    for name, kwargs in APPROACH_RUNS:
        if not wanted(name):
            continue
        d = _Digest()
        with _patched(approach_rig, _recording(solve, d)):
            approach_rig.run_approach(**kwargs)
        out[name] = (d.n, d.hexdigest())
    for name, kwargs in PERSON_RUNS:
        if not wanted(name):
            continue
        d = _Digest()
        with _patched(person_rig, _recording(solve, d)):
            person_rig.run_pass(kwargs['obstacle_r'], kwargs['offset'])
        out[name] = (d.n, d.hexdigest())
    for name, kwargs in _direct_cases():
        if not wanted(name):
            continue
        d = _Digest()
        u0, info = solve(**kwargs)
        d.record(kwargs, u0, info)
        out[name] = (d.n, d.hexdigest())
    return out


def load_golden(path=GOLDEN_PATH):
    with open(path, encoding='utf-8') as fh:
        data = json.load(fh)
    return {k: (v['n_solves'], v['sha256']) for k, v in data['runs'].items()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--write', help='write the golden JSON here')
    args = parser.parse_args(argv)
    result = digests()
    for name, (n, h) in result.items():
        print(f'{name:45s} {n:5d} {h[:16]}')
    if args.write:
        os.makedirs(os.path.dirname(os.path.abspath(args.write)), exist_ok=True)
        rev = os.popen('git rev-parse --short HEAD').read().strip()
        with open(args.write, 'w', encoding='utf-8') as fh:
            json.dump({'recorded_at': rev,
                       'runs': {k: {'n_solves': n, 'sha256': h}
                                for k, (n, h) in result.items()}},
                      fh, indent=1, sort_keys=True)
            fh.write('\n')


if __name__ == '__main__':
    main()
