#!/usr/bin/env python3
"""Phase 3: split an mpc_capture run (mpc_capture_node.py's capture.npz) into

  mpc_frozen_inputs.npz   -- per tick: the exact solve_mpc_step kwargs and the
                             node's sim time at the call. This is what
                             run_mpc_frozen.py re-solves, on Thor and on the
                             Orin.
  mpc_live_outputs.npz    -- per tick: what the live node got back (u0, info)
                             and the solve's wall time. A Thor re-solve of
                             the frozen inputs must reproduce this bit for bit;
                             that is the check that the capture is lossless and
                             the solve deterministic once timing is removed.

All ticks are kept (every control tick of the run, ~420), well above the 50
the Phase 3 plan asks for, and spread over the whole run by construction.

Usage: freeze_mpc_inputs.py CAPTURE_NPZ OUT_DIR
Plain Python + NumPy.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mpc_frozen_io  # noqa: E402


def main():
    capture, out_dir = sys.argv[1], sys.argv[2]
    ticks, meta, _ = mpc_frozen_io.load_ticks(
        capture, ('inputs', 'u0', 'info', 'sim_ns', 'wall_dt'))
    inputs = [{'inputs': t['inputs'], 'sim_ns': t['sim_ns']} for t in ticks]
    live = [{'u0': t['u0'], 'info': t['info'], 'sim_ns': t['sim_ns'], 'wall_dt': t['wall_dt']}
            for t in ticks]
    meta = dict(meta, source=os.path.abspath(capture))
    mpc_frozen_io.save_ticks(os.path.join(out_dir, 'mpc_frozen_inputs.npz'), inputs, meta)
    mpc_frozen_io.save_ticks(os.path.join(out_dir, 'mpc_live_outputs.npz'), live,
                             dict(meta, label='thor-live'))
    print('froze %d ticks -> %s' % (len(ticks), out_dir))


if __name__ == '__main__':
    main()
