# MPC RTI tick fixture (`rti_ticks.jsonl.gz`)

Captured inputs and outputs of `mpc_solver.solve_mpc_step` on the RTI path, one solve per line,
for the R2 solver parity (Python `_solve_rti` against the Rust core). Written by
`tools/rust_parity/mpc/profile_rti.py`. The full format is specified in the docstring of
`tools/rust_parity/mpc/rti_fixture.py`, which also has the reader.

## What is in it

| group | ticks | source |
|---|---|---|
| `matrix` | every 2nd tick of 12 closed loops | `test/speed_channel_harness.run`: a 4 m goal_distance move at 0.5 m/s with stack_params weights and limits, N = 20; speed model `legacy` / `offset_free` x 0 / 1 / 8 obstacles x 0 / 4 live wall rows (`boundary_max_sources` 4, soft rows, `wall_row_slack_weight`) |
| `approach` | every tick of 3 closed loops | `test/test_object_approach_closed_loop.run_approach`: real corridor builder (curved object corridors, deflection), legacy speed model, target as a 0.25 m disk |
| `edge` | 40 | a matrix tick with one input changed: no, short or non-finite warm start; hard rows; infeasible hard rows; a saturated soft row; 8 slots; `psiRefTurn`; a target line; ts 0.2 with a 0.1 s control period (stack_params runs both at 0.1) |

Statuses covered: 1 solved, 2 solved inaccurate, 3 primal infeasible, 4 primal infeasible
inaccurate, 7 max iter. The closed loops reach status 3 on their own (legacy model: measured
speed above the hard `vMax` box). Both failure branches (`success` False, warm start held) are
in the file.

## Format in one paragraph

gzip (mtime 0, so equal inputs give equal bytes) of UTF-8 JSON Lines. Line 1 is a header
(format name, version 1, source hashes, the stack_params the scenarios read, the effective OSQP
settings, library versions, counts). Every other line is one tick:
`{"id", "group", "config", "tick", "inputs", "outputs", "osqp"}`. `inputs` holds every keyword
argument of `solve_mpc_step`, defaults filled in, with the corridor cut down to the keys the
solver reads. `outputs` holds u0, success, status, status_message, message, cost, iterations,
zopt, x_pred (N x 4), boundary_slack, and, with a speed model, v_cmd_pred and speed_slack.
`osqp` holds the QP size, iterations, rho updates, polish status and a sha256 of the QP exactly
as OSQP C received it. Floats are Python `repr`, so they round-trip exactly (serde_json needs
`float_roundtrip`). Non-finite floats are written `{"$float": "inf"}`, `"-inf"` or `"nan"`.

## Use

```python
import sys; sys.path.insert(0, 'tools/rust_parity/mpc')
import rti_fixture
from mpc_controller.mpc_solver import solve_mpc_step

header, ticks = rti_fixture.load('tools/rust_parity/mpc/data/rti_ticks.jsonl.gz')
for t in ticks:
    u0, info = solve_mpc_step(**rti_fixture.solve_kwargs(t))
    assert rti_fixture.compare_outputs(t['outputs'], rti_fixture.outputs_of(u0, info))['identical']
```

To regenerate it, run `tools/rust_parity/mpc/profile_rti.py` with the workspace sourced. The
run checks that every tick read back from the file solves bit-identical to the recorded output,
and records the file's sha256 in its JSON. Two runs on linus produced the same bytes.
