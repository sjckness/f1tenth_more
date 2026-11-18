#!/usr/bin/env python3
r"""
Run the real mpc_corr node and time every solve_mpc_step() call.

Runs on: the Orin host (native Humble), the harness container, linus.
Started by replay_localization.sh's mpc_capture layer; mpc_timing.py reads
the capture (kind `capture:`): the pure solve time of each tick, unaffected
by the executor or the clock, to compare native vs container (P9.4).

No production code is changed. MPC_corr.py imports solve_mpc_step into its
own module namespace (`from mpc_controller.mpc_solver import ...
solve_mpc_step`) and calls it by that name (via _mpc_solve when
allow_reverse is off), with keyword arguments only; this script swaps that
one name for a wrapper that times the original call. Everything else --
node class, callbacks, timers, parameters -- is the production code. With
allow_reverse on, the reverse planner calls the solver itself and those
solves are not seen here.

Run it like the production node, with the parameters dumped from a
production-launched replay (`ros2 param dump /mpc_corr`):

  mpc_capture_node.py --capture-out capture.npz [--full] \\
      --ros-args --params-file params.yaml -p use_sim_time:=true

Written on SIGINT/shutdown. Per tick: the node's (sim) time at the call and
the solve's wall-clock duration (time.perf_counter around the original call
only); with --full also deep copies of the kwargs, u0 and the info dict (for
a function-level comparison; large).
"""
import copy
from pathlib import Path
import sys
import time

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mpc_frozen_io  # noqa: E402, I100


def main():
    argv = list(sys.argv[1:])
    if '--capture-out' not in argv or '-h' in argv or '--help' in argv:
        print(__doc__)
        return 0 if ('-h' in argv or '--help' in argv) else 2
    i = argv.index('--capture-out')
    out_path = argv[i + 1]
    del argv[i:i + 2]
    full = '--full' in argv
    if full:
        argv.remove('--full')

    import mpc_controller.MPC_corr as mpc_corr_module
    original = mpc_corr_module.solve_mpc_step
    ticks = []
    holder = {}

    def capturing_solve_mpc_step(**kwargs):
        inputs = copy.deepcopy(kwargs) if full else None
        node = holder.get('node')
        sim_ns = node.get_clock().now().nanoseconds if node is not None else -1
        t0 = time.perf_counter()
        u0, info = original(**kwargs)
        wall_dt = time.perf_counter() - t0
        tick = {'sim_ns': int(sim_ns), 'wall_dt': float(wall_dt)}
        if isinstance(info, dict) and 'success' in info:
            tick['success'] = bool(info['success'])
        if full:
            tick.update({'inputs': inputs, 'u0': copy.deepcopy(u0), 'info': copy.deepcopy(info)})
        ticks.append(tick)
        return u0, info

    mpc_corr_module.solve_mpc_step = capturing_solve_mpc_step

    rclpy.init(args=[sys.argv[0]] + argv)
    node = mpc_corr_module.MPCController()
    holder['node'] = node
    try:
        rclpy.spin(node)
    except BaseException:  # noqa: B036 - KeyboardInterrupt / ExternalShutdownException
        pass
    finally:
        import numpy
        try:
            import osqp
            osqp_v, osqp_f = osqp.__version__, osqp.__file__
        except ImportError:
            osqp_v = osqp_f = 'not importable'
        meta = {'python': sys.version, 'numpy': numpy.__version__, 'osqp': osqp_v,
                'osqp_file': osqp_f, 'mpc_corr_file': mpc_corr_module.__file__,
                'full': str(full)}
        mpc_frozen_io.save_ticks(out_path, ticks, meta)
        print('captured %d solve_mpc_step calls -> %s' % (len(ticks), out_path), flush=True)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
