#!/usr/bin/env python3
"""Phase 3: run the real mpc_corr node, recording the exact arguments and
results of every solve_mpc_step() call, for the function-level (timing
removed) parity test.

No production code is changed. MPC_corr.py imports solve_mpc_step into its
own module namespace (`from mpc_controller.mpc_solver import ...
solve_mpc_step`) and calls it by that name from control_loop, always with
keyword arguments only; this script swaps that one name for a wrapper that
deep-copies the arguments, calls the original, and keeps both. Everything
else -- node class, callbacks, timers, parameters -- is the production code.

Run it exactly like the production node, with the parameters dumped from a
production-launched replay (`ros2 param dump /mpc_corr`), so the capture run
sees the same configuration:

  mpc_capture_node.py --capture-out capture.npz \\
      --ros-args --params-file params.yaml -p use_sim_time:=true

Written on SIGINT/shutdown. Per tick it stores: the solve_mpc_step kwargs,
u0, the info dict, the node's sim time at the call, and the solve's wall-clock
duration (time.perf_counter around the original call only, so it is a pure
solve time, unaffected by the deep copy).
"""
import copy
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mpc_frozen_io  # noqa: E402

import mpc_controller.MPC_corr as mpc_corr_module  # noqa: E402


def main():
    argv = list(sys.argv[1:])
    if '--capture-out' not in argv:
        sys.exit('usage: mpc_capture_node.py --capture-out FILE.npz [--ros-args ...]')
    i = argv.index('--capture-out')
    out_path = argv[i + 1]
    del argv[i:i + 2]

    original = mpc_corr_module.solve_mpc_step
    ticks = []
    holder = {}

    def capturing_solve_mpc_step(**kwargs):
        inputs = copy.deepcopy(kwargs)
        node = holder.get('node')
        sim_ns = node.get_clock().now().nanoseconds if node is not None else -1
        t0 = time.perf_counter()
        u0, info = original(**kwargs)
        wall_dt = time.perf_counter() - t0
        ticks.append({
            'inputs': inputs,
            'u0': copy.deepcopy(u0),
            'info': copy.deepcopy(info),
            'sim_ns': int(sim_ns),
            'wall_dt': float(wall_dt),
        })
        return u0, info

    mpc_corr_module.solve_mpc_step = capturing_solve_mpc_step

    rclpy.init(args=[sys.argv[0]] + argv)
    node = mpc_corr_module.MPCController()
    holder['node'] = node
    try:
        rclpy.spin(node)
    except BaseException:  # KeyboardInterrupt / ExternalShutdownException
        pass
    finally:
        import numpy
        import osqp
        meta = {
            'python': sys.version,
            'numpy': numpy.__version__,
            'osqp': osqp.__version__,
            'osqp_file': osqp.__file__,
            'mpc_solver_file': mpc_corr_module.__file__,
        }
        mpc_frozen_io.save_ticks(out_path, ticks, meta)
        print('captured %d solve_mpc_step calls -> %s' % (len(ticks), out_path), flush=True)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
