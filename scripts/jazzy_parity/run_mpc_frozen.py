#!/usr/bin/env python3
"""Phase 3: re-solve every frozen MPC tick with plain Python + NumPy + SciPy +
osqp. No ROS, no rclpy, no rosbag2.

Input: mpc_frozen_inputs.npz (freeze_mpc_inputs.py) -- for each control tick
of the Thor capture run, the exact keyword arguments mpc_corr passed to
mpc_solver.solve_mpc_step() (state, last_u, warm start, corridor, weights,
obstacles, hard boundaries, ...). Each tick is solved on its own from its
own frozen warm start, so nothing carries over between ticks and no timer,
cache or message timing is involved.

mpc_solver.py (and the vehicle_model.py / f1tenth_params/object_geometry.py
it imports) are loaded from the given repo checkout. Their sha256s go into
the output, so the comparison can confirm both platforms ran the same source.

Recorded per tick, in addition to solve_mpc_step's own return values:
  - the QP handed to OSQP (P, q, A, l, u as CSC arrays) and the settings
    passed to setup(), so a difference can be placed either in the QP
    construction (NumPy/SciPy) or in the solve (osqp);
  - OSQP's raw primal x and dual y, iteration count, status and run time;
  - wall-clock time of the whole solve_mpc_step call (time.perf_counter).

Usage:
  python3 run_mpc_frozen.py --repo ~/dev_ws/f1tenth_more \\
      --inputs mpc_frozen_inputs.npz --out mpc_orin_outputs.npz --label orin
  [--osqp-target DIR]   import osqp from DIR first (a `pip install --target`
                        directory) -- for the second-osqp-version run.

Python 3.6+ / NumPy 1.17+ compatible on purpose (runs on the Orin's 3.10).
"""
import argparse
import hashlib
import os
import platform
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mpc_frozen_io  # noqa: E402

SOLVER_FILES = (
    os.path.join('src', 'f1tenth_control', 'mpc_controller', 'mpc_controller', 'mpc_solver.py'),
    os.path.join('src', 'f1tenth_control', 'mpc_controller', 'mpc_controller', 'vehicle_model.py'),
    os.path.join('src', 'f1tenth_params', 'f1tenth_params', 'object_geometry.py'),
)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        h.update(f.read())
    return h.hexdigest()


def csc_parts(M):
    M = M.tocsc()
    return {'shape': np.array(M.shape, dtype=np.int64), 'data': np.array(M.data, dtype=np.float64),
            'indices': np.array(M.indices, dtype=np.int64), 'indptr': np.array(M.indptr, dtype=np.int64)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--repo', required=True)
    ap.add_argument('--inputs', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', default=platform.node())
    ap.add_argument('--osqp-target', default=None)
    args = ap.parse_args()

    if args.osqp_target:
        sys.path.insert(0, os.path.abspath(os.path.expanduser(args.osqp_target)))
    repo = os.path.abspath(os.path.expanduser(args.repo))
    sys.path.insert(0, os.path.join(repo, 'src', 'f1tenth_params'))
    sys.path.insert(0, os.path.join(repo, 'src', 'f1tenth_control', 'mpc_controller'))

    import osqp
    import scipy
    from mpc_controller import mpc_solver

    if not mpc_solver.OSQP_AVAILABLE:
        sys.exit('mpc_solver could not import osqp')

    record = {}
    Base = osqp.OSQP

    class RecordingOSQP(Base):
        def setup(self, P, q, A, l, u, **settings):
            record['P'] = csc_parts(P)
            record['A'] = csc_parts(A)
            record['q'] = np.array(q, dtype=np.float64)
            record['l'] = np.array(l, dtype=np.float64)
            record['u'] = np.array(u, dtype=np.float64)
            record['settings'] = dict(settings)
            return Base.setup(self, P, q, A, l, u, **settings)

        def warm_start(self, *a, **kw):
            record['warm_x'] = np.array(kw.get('x'), dtype=np.float64) if kw.get('x') is not None else None
            return Base.warm_start(self, *a, **kw)

        def solve(self, *a, **kw):
            res = Base.solve(self, *a, **kw)
            record['x'] = None if res.x is None else np.array(res.x, dtype=np.float64)
            record['y'] = None if res.y is None else np.array(res.y, dtype=np.float64)
            info = res.info
            record['iter'] = int(info.iter) if info is not None else -1
            record['status_val'] = int(info.status_val) if info is not None else -99
            record['obj_val'] = float(info.obj_val) if info is not None else float('nan')
            record['run_time'] = float(info.run_time) if info is not None else float('nan')
            record['solve_time'] = float(getattr(info, 'solve_time', float('nan')))
            record['polish_time'] = float(getattr(info, 'polish_time', float('nan')))
            record['status_polish'] = int(getattr(info, 'status_polish', -99))
            return res

    osqp.OSQP = RecordingOSQP  # mpc_solver calls osqp.OSQP() by module attribute

    ticks, meta_in, _ = mpc_frozen_io.load_ticks(args.inputs, ('inputs',))
    out_ticks = []
    for tick in ticks:
        record.clear()
        t0 = time.perf_counter()
        u0, info = mpc_solver.solve_mpc_step(**tick['inputs'])
        wall = time.perf_counter() - t0
        rec = dict(record)
        out_ticks.append({'u0': np.array(u0, dtype=np.float64), 'info': info, 'qp': rec,
                          'wall_dt': float(wall)})

    meta = {
        'label': args.label,
        'python': sys.version.replace('\n', ' '),
        'numpy': np.__version__,
        'scipy': scipy.__version__,
        'osqp': osqp.__version__,
        'osqp_file': osqp.__file__,
        'platform': platform.platform(),
        'machine': platform.machine(),
        'repo': repo,
        'inputs_sha256': sha256_file(args.inputs),
    }
    for rel in SOLVER_FILES:
        meta['sha256_' + os.path.basename(rel)] = sha256_file(os.path.join(repo, rel))
    mpc_frozen_io.save_ticks(args.out, out_ticks, meta)

    walls = np.array([t['wall_dt'] for t in out_ticks]) * 1e3
    print('ticks solved:     %d' % len(out_ticks))
    print('solve_mpc_step wall ms: mean %.2f p50 %.2f p95 %.2f p99 %.2f max %.2f' % (
        walls.mean(), np.percentile(walls, 50), np.percentile(walls, 95),
        np.percentile(walls, 99), walls.max()))
    for k in sorted(meta):
        print('%-26s %s' % (k + ':', meta[k]))
    print('wrote %s' % args.out)


if __name__ == '__main__':
    main()
