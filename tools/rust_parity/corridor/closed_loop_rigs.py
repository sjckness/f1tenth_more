#!/usr/bin/env python3
"""The closed-loop rigs on the Rust corridor: same bytes, solve for solve.

    python3 tools/rust_parity/corridor/closed_loop_rigs.py [--json out.json]

closed_loop_golden.py (mpc_controller/test) runs the object-approach and
person-pass rigs plus a few direct solves and hashes the exact float64 bytes
of every solve's inputs and outputs into one SHA-256 per run; its golden file
was written on the Python corridor. This runs those digests once per
corridor implementation (corridor_backend.DEFAULT_IMPL = 'py', then 'rust'):
every rig builds its corridors through MPC_corr.build_straight_corridor on a
stand-in, i.e. through DEFAULT_IMPL. Equal digests mean every tick of every
run -- corridor, target, warm start, solve -- is bit for bit the same.
"""

import argparse
import json
import os
from pathlib import Path
import sys
import time

REPO = Path(__file__).resolve().parents[3]
TEST_DIR = REPO / 'src' / 'f1tenth_control' / 'mpc_controller' / 'test'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--json')
    args = ap.parse_args(argv)
    sys.path.insert(0, str(TEST_DIR))
    import closed_loop_golden as golden  # noqa: E402
    from mpc_controller import corridor_backend, mpc_native  # noqa: E402

    if not mpc_native.available():
        raise SystemExit(f'Rust library not loadable: {mpc_native.load_error()}')
    calls = {'py': 0, 'rust': 0}
    real = {'py': (corridor_backend.corridor_geometry.corridor_curves,
                   corridor_backend.corridor_geometry.corridor_curves_to_pose),
            'rust': (mpc_native.corridor_curves, mpc_native.corridor_curves_to_pose)}

    def counted(impl, fn):
        def wrapped(*a, **k):
            calls[impl] += 1
            return fn(*a, **k)
        return wrapped

    corridor_backend.corridor_geometry.corridor_curves = counted('py', real['py'][0])
    corridor_backend.corridor_geometry.corridor_curves_to_pose = counted('py', real['py'][1])
    mpc_native.corridor_curves = counted('rust', real['rust'][0])
    mpc_native.corridor_curves_to_pose = counted('rust', real['rust'][1])

    expected = golden.load_golden()
    out = {'golden': str(golden.GOLDEN_PATH), 'runs': len(expected), 'impl': {}}
    for impl in ('py', 'rust'):
        corridor_backend.DEFAULT_IMPL = impl
        before = dict(calls)
        t0 = time.time()
        got = golden.digests()
        same = {k: got.get(k) == v for k, v in expected.items()}
        out['impl'][impl] = {
            'seconds': time.time() - t0,
            'solves': sum(n for n, _h in got.values()),
            'runs_equal_to_golden': sum(same.values()),
            'differing_runs': sorted(k for k, ok in same.items() if not ok),
            'corridor_calls_py': calls['py'] - before['py'],
            'corridor_calls_rust': calls['rust'] - before['rust'],
        }
        print(f'{impl}: {out["impl"][impl]["runs_equal_to_golden"]}/{len(expected)} runs '
              f'equal to the golden, {out["impl"][impl]["solves"]} solves, corridor calls '
              f'py {out["impl"][impl]["corridor_calls_py"]} rust '
              f'{out["impl"][impl]["corridor_calls_rust"]} '
              f'({out["impl"][impl]["seconds"]:.1f} s)')
    ok = all(not v['differing_runs'] for v in out['impl'].values()) and \
        out['impl']['rust']['corridor_calls_rust'] > 0 and \
        out['impl']['rust']['corridor_calls_py'] == 0
    out['pass'] = ok
    if args.json:
        Path(args.json).write_text(json.dumps(out, indent=1) + '\n')
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    os.environ.pop('F1TENTH_CORRIDOR_IMPL', None)
    sys.exit(main())
