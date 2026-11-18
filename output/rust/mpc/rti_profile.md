# MPC RTI solve: per-part profile (R2.0)

Branch `main-jazzy-rust` at 34288544, linus (i7-9700K), 2026-10-23. Tool:
`tools/rust_parity/mpc/profile_rti.py`. Raw numbers: `output/rust/mpc/rti_profile_run{1,2}.json`.
Parity fixture: `tools/rust_parity/mpc/data/rti_ticks.jsonl.gz` (723 ticks; format in
`tools/rust_parity/mpc/data/README.md` and the `rti_fixture.py` docstring). All numbers below
come from run 1. The tables at the end are generated from both runs.

**Result.** One RTI tick (`solve_mpc_step`, `solver='rti'`) costs **8.0 ms** on average over the
1037 profiled closed-loop ticks (p50 7.8, p95 10.4 ms). In the live configuration (offset_free,
8 obstacles, 4 live wall rows: 145 variables x 325 rows) it costs **10.4 ms** (p95 10.6 ms).
Timed inside the closed loop, as a node sees it, that is p95 11.6 ms and max 15.8 ms. **OSQP C's
own work (setup, ADMM iterations, polish) is 1.0 ms of the 8.0 ms (12.5 %)**, and 1.4 ms of the
10.4 ms in the live configuration. The other ~87 % is Python around the solver: building the QP
row by row in dense numpy arrays, converting it to CSC, osqp-python's wrapper, the true-cost
evaluation and the numeric obstacle linearisation. A Rust core that keeps only OSQP C should
therefore run a tick in roughly 1-1.5 ms on linus. That is an estimate: the C time is measured,
the Rust arithmetic around it is not yet. `bench_solve.py` measures it in R2.

## Where the time goes (all 1037 matrix ticks, mean per tick)

| part | ms | share | what it is | in Rust |
|---|---|---|---|---|
| row assembly | 2.54 | 32 % | `np.zeros(n_z)` per row and `+=` on dense `P` / `q` blocks: dynamics, control and rate rows, then the stage loop (velocity, effort, rate, corridor bound and centring, heading, target line, speed, wall rows), slack and terminal terms, `np.array(rows)`, `0.5 (P + P^T)` | qp.rs, straight to CSC |
| OSQP setup | 1.36 | 17 % | `osqp.OSQP()` **0.78** (backend probing, see below), then `setup()` 0.58: conversion and `triu` in Python 0.30, OSQP C 0.18, binding ~0.1 | C setup only: 0.18 |
| planner_cost_corridor | 0.93 | 12 % | the true cost reported as `info['cost']`: a second nonlinear rollout with a Python double loop over probe points and obstacles, corridor queries per stage | cost.rs |
| OSQP solve + polish | 0.82 | 10 % | OSQP C solve 0.77 + polish 0.05, binding ~0.01 | the same C code |
| obstacle linearisation | 0.82 | 10 % | `_linearize_obstacle_cost_rti`: 7 distance evaluations per stage as small numpy arrays (1.07 ms with an obstacle, 0.03 without) | qp.rs |
| CSC conversion | 0.67 | 8 % | `scipy.sparse.csc_matrix` of the dense 145 x 145 `P` and 325 x 145 `A` | gone (built as CSC) |
| corridor queries | 0.41 | 5 % | `_nearest_corridor_frame` / `corridor_heading_at` from the stage loop (argmin over 120 centreline points, 20-40 times) | R1's query.rs |
| model + Jacobians | 0.27 | 3 % | reference rollout and analytic Jacobians, 20 stages | model.rs |
| rollout | 0.06 | 1 % | `_rollout_true_dynamics` for `x_pred` | model.rs |
| other | 0.03 | 0 % | weight scaling, argument checks, result dict | |

The sum of the parts is within 1.6 % of the uninstrumented time in every config. What each
feature costs (from the per-config table):

- **the first obstacle +1.4 ms**: obstacle linearisation +1.07 ms, true cost +0.09 ms. Going from
  1 to 8 obstacles adds another +0.6 ms, nearly all in planner_cost_corridor's Python double loop
  (+0.47 ms); the batched linearisation adds only +0.12 ms.
- **4 live wall rows +0.6 to +0.8 ms**, almost all OSQP: the median iteration count goes from
  100-125 to 175-200, and C setup grows by 0.09 ms.
- **offset_free over legacy +2.0 to +2.3 ms**, 1.4 ms of it in the stage loop (2.57 ms against
  1.23 ms). The main cause is `np.pad` of the 4x4 obstacle model to 5x5: 38 us a pair, 20 pairs
  per tick, 0.77 ms.
- BLAS does not matter for the time. The only BLAS calls on this path are 4x4 /
  5x5 products (`A_k @ x_ref`, `B_k @ u_ref`, `Wdu @ last_u`, `E.T @ E`), microseconds in total.
  Reference BLAS matters for bit parity (summation order), not for speed.

## Recommendation for R2.1

1. **Take the QP build first, as one piece.** That means the rows and `P`/`q`, including the
   stage loop's obstacle linearisation and corridor queries, plus model + Jacobians, written
   directly as CSC (`P`'s upper triangle). This is 4.7 of the 7.9 ms (59 %). None of it is
   numerically heavy (4x4 / 5x5 blocks, 20 stages). It is Python call overhead and per-row
   allocation, and Rust removes nearly all of it. Do not go through dense matrices.
2. **Then the OSQP call** (solve.rs, `osqp = "=1.0.1"`, the same OSQP C 1.0.0): today 2.2 ms per
   tick, of which 1.0 ms is OSQP C. Through the C library only the 1.0 ms stays.
3. **Then the true cost and the rollout** (cost.rs on model.rs): 1.0 ms (12 %); 1.3-1.4 ms with
   8 obstacles.

The stateless call in design.md (R2 item 2) returns u0, zopt, x_pred and the cost, so it needs all
three anyway. The order above is the order to implement and check them in. L1 checks qp.rs against
the QP of each fixture tick (`qp_sha256` is in the fixture; the arrays come back by replaying the
Python with the tick's inputs). L2 checks solve.rs on Python's QP. L3 checks the whole call on the
fixture.

**Persistent OSQP workspace: not needed. Keep a fresh one per tick, as designed, and do not
build `rust_persistent` now.** The comparisons below are against the fresh solve's OSQP C time,
which is what the Rust port will pay.

- In Rust a fresh workspace costs only OSQP C's setup: 0.10-0.25 ms per tick (Ruiz scaling, AMD
  ordering, symbolic and numeric factorisation of the KKT system). The 0.78 ms constructor and
  the 0.30 ms conversion are osqp-python costs, and the port loses them anyway.
- **Persistent with fresh numerics** (`update(Px, Ax, q, l, u)`, rho reset, y = 0, which is how a
  fresh solver starts): the same iterations and statuses as the fresh solve on every tick, x
  within 2.6e-6. That is still not bit for bit, because the fixed sparsity pattern keeps explicit
  zeros, which change the factorisation's rounding. It saves 0.00-0.12 ms per tick (update
  0.06-0.11 ms plus a rho reset of 0.01-0.03 ms, against a 0.10-0.25 ms setup).
- **Persistent the natural way** (rho and the dual iterate y carried over from the previous tick)
  is a different solver. Iteration counts move by -175 to +175, the status changes on up to 62 %
  of a run's ticks, and the solution moves by up to 6 cm at p50 (max abs over x, worst 0.47).
  Most solves stop at the iteration limit rather than at convergence, so where the iteration
  starts decides where it ends. Resetting rho alone does not fix this (status still changes on
  up to 60 %). The carried dual does save iterations in offset_free: up to 0.43 ms per tick, and
  -0.43 to +0.11 ms over all configs. That is a tuning change to judge on the closed loop, not a
  free speed-up.
- Neither is needed for the budget. The projected Rust tick is about 1-1.5 ms in a 100 ms control
  period, and the R2 gate (p95 no worse than Python) is met by the fresh workspace with a wide
  margin. If `rust_persistent` is ever built: a fixed structural pattern, and the closed loop
  judges it, as design.md says.

## What the port has to reproduce (found while measuring)

- **The QP must be bit-identical, not just within 1e-14.** 47 % of the matrix ticks (489 of 1037)
  end solved-inaccurate (350) or at max_iter (139). Iteration counts sit on the check_termination
  grid (25, 50, ..., 200). A non-converged ADMM iterate follows every rounding difference, so L1
  should target 0 ulp on P, q, A, l, u. That means the same `0.5 * (P + P.T)` on the dense sum and
  the same exact zeros dropped: `csc_matrix` drops +-0.0, and the `P` and `A` patterns change
  from tick to tick (`A` 596-693 nonzeros in the legacy wall configs). Jacobian entries are exactly
  zero at v = 0 or psi + beta = 0, and the corridor normal has one zero component on a corridor
  along x. Then `P`'s upper triangle, and `l`/`u` clamped to +-1e30. A different pattern means a
  different factorisation.
- **Settings:** set every field in the settings table below explicitly. In particular,
  `adaptive_rho` 1 is the iteration-based rule with interval 50, which is deterministic (not the
  time-based one). Also `check_dualgap` 1, `scaled_termination` 0, `polish_refine_iter` 3,
  `delta` 1e-6, `sigma` 1e-6, `rho` 0.1, `alpha` 1.6, `scaling` 10, eps 1e-3 / 1e-4, and QDLDL. The
  warm start is x only; y starts at 0.
- **Failure path:** statuses 3, 4, 5 and 6 mean `success` False, and `z_guess` is returned (the
  warm start is held). The closed loops reach status 3 on their own: 42 matrix ticks, all legacy
  with live walls, where the measured speed exceeds the hard `vMax` box and the acceleration rate
  limit cannot bring it back in one step. The fixture's edge group adds status 4 and infeasible
  hard rows.
- `x_pred` is **N x 4** (x_1..x_N), not (N+1) x 4 as design.md says (logged in decisions.md).
  `v_cmd_pred` has N entries.
- stack_params now runs `mpc_prediction_dt_s` 0.1 = control period, so the rate-weight factor is
  1 on every closed-loop tick. The fixture's `ts0.2_cp0.1` edge ticks cover the other branch.

## Python-side findings (not changed: R2.0 edits nothing under src/)

- **`osqp.OSQP()` costs 0.74 ms per tick.** `default_algebra()` tries the cuda and mkl backends
  first, and each failed import walks all 70 sys.path entries of the ROS environment.
  `osqp.OSQP(algebra='builtin')` costs 1.3 us, gives the same backend and identical numerics, and
  would save about 9 % of every tick in Python today. It is a one-line change to `_solve_rti`;
  logged in decisions.md for the coordinator. The car's cost depends on its own sys.path.
- **`np.pad` in the offset_free stage loop costs 0.77 ms per tick** (see above). Zero-filled
  5x5 / 5-vector blocks written by slice would give the same numbers.
- osqp-python's `update(Px=...)` rebinds `.data` of the matrix object `setup()` cached. When
  setup did not make its own `triu` copy, that object is the caller's. This does not affect
  production, which never calls update. The persistent benchmark sets up on copies.

## Method

- **Ticks** (closed loops, not synthetic calls):
  - matrix: `test/speed_channel_harness.run`, a 4 m goal_distance move at 0.5 m/s with the
    stack_params weights and limits, N = 20, the real SpeedChannel and stop ramp, and a plant that
    is not the solver's model.
  - The matrix crosses the speed model (legacy, offset_free), 0 / 1 / 8 obstacles
    (test_obstacles_v2's timing set) and 0 / 4 live wall rows: four odom-frame half-planes, with
    the shipped `boundary_max_sources` 4, soft rows, `wall_row_slack_weight` 2e6 and
    `boundary_slack_max` 0.5. "walls0" still has the 4 padded all-zero rows per stage, as live.
  - approach: three runs of `test_object_approach_closed_loop.run_approach` (real corridor
    builder, curved corridors, deflection; legacy, ts 0.1, no control period), the target as a
    0.25 m disk.
  - 1197 ticks are profiled. The 40 edge ticks are only in the fixture.
- **Timing:** each tick is replayed from its decoded fixture encoding, 7 times, after a warm-up.
  Each repeat runs the uninstrumented `solve_mpc_step` over every tick, then the instrumented copy.
  Per tick the minimum is kept, which is robust against other load on the machine. Configs are
  summarised as the mean / p50 / p95 over their ticks. "closed loop" is the single call timed
  inside the rig.
- **Breakdown:** `solve_mpc_step` and `_solve_rti` are recompiled from their own source into a
  copy of `mpc_solver`'s globals, with a timestamp probe before 23 anchor statements (a changed
  anchor stops the run). The stage loop's calls to `_linearize_obstacle_cost_rti`,
  `_nearest_corridor_frame` and `corridor_heading_at` are wrapped in that copy only. The real
  module is never patched. All 1237 instrumented solves are bit-identical to the real ones.
- **OSQP alone:** on each tick's captured QP, `OSQP()`, `setup()`, `warm_start()` and `solve()`
  are timed separately, together with OSQP C's own `setup_time`, `solve_time` and `polish_time`
  and the Python-side conversion (`_infer_mnpqalu`). The solution is bit-identical to the real
  solve on every tick. For the persistent workspace, one setup per run is followed by
  `update` + `warm_start` + `solve` per tick, in closed-loop order. Three variants: rho carried,
  rho reset, and rho reset with y = 0. The pattern is the run's union pattern, explicit zeros
  kept.
- **Checks:** all 1237 ticks re-solve bit-identically from the fixture encoding, and still after
  the timing replays. Every fixture tick read back from the file solves bit-identically. The
  effective OSQP settings are the same on every tick.
- **Gate:** two complete runs, compared with `--compare` on 174 metrics: every config's mean and
  p95, every part above 5 % of its config, OSQP-alone setup and solve, and every aggregate part.
  Values under 0.05 ms are shown but not gated. Worst deviation: **7.33 % (tolerance 10 %,
  PASS)**. The fixture's sha256 is the same in both runs. The R1 agent's pytest ran on another
  core through both runs (load average 1.1-2.2), which costs the Python-heavy parts 5-8 %. A pair
  run earlier on a quiet machine with the same timing code differed by at most 2.80 %.
- **Caveats:** linus only (x86, intel_pstate `powersave` with turbo). The Thor (aarch64) and the
  Orin container will have other absolute times and another Python/C ratio; the planning session
  measured 27 / 54 ms p50 / p95 on the Orin. Means of per-tick minima describe what the code
  costs, not the jitter a live node sees.

## Reproduce

```bash
source output/rust_main/env.sh            # bash; the workspace's install
tools/rust_parity/mpc/profile_rti.py --out output/rust/mpc/rti_profile_run1.json   # ~3.5 min
tools/rust_parity/mpc/profile_rti.py --out output/rust/mpc/rti_profile_run2.json
tools/rust_parity/mpc/profile_rti.py --compare output/rust/mpc/rti_profile_run1.json \
    output/rust/mpc/rti_profile_run2.json                                          # exit 0 = gate
tools/rust_parity/mpc/profile_rti.py --report output/rust/mpc/rti_profile_run1.json \
    output/rust/mpc/rti_profile_run2.json      # rewrites only the generated block below
```

Every run also rewrites the fixture, with the same bytes when nothing upstream changed.

## Generated tables

<!-- BEGIN GENERATED (profile_rti.py --report) -->
_Generated from `output/rust/mpc/rti_profile_run1.json` and `output/rust/mpc/rti_profile_run2.json` (tables: the first). Times in ms; per tick the minimum of 7 replays, then mean / p95 over the ticks of a config._

### Machine and libraries

|  |  |
|---|---|
| CPU | Intel(R) Core(TM) i7-9700K CPU @ 3.60GHz, 8 CPUs, max 4900 MHz, governor `powersave`, affinity [0, 1, 2, 3, 4, 5, 6, 7] |
| Load average (start / end) | 1.10, 1.55, 1.44 / 2.22, 1.98, 1.65 |
| Python | 3.12.3 (Linux-7.0.0-38-generic-x86_64-with-glibc2.39), 70 sys.path entries |
| numpy / scipy | 1.26.4 / 1.11.4 (Ubuntu packages) |
| BLAS | numpy built against `blas` (system); loaded `/usr/lib/x86_64-linux-gnu/blas/libblas.so.3.12.0`, `/usr/lib/x86_64-linux-gnu/lapack/liblapack.so.3.12.0`: reference BLAS/LAPACK, libblas3 3.12.0-3build1.1 |
| OSQP | osqp-python 1.1.3 (pip), OSQP C 1.0.0 (float64 True, int64 indices False), QDLDL direct solver |
| Source | git 342885443694c29e4c33f62c03fcfa843b5f3674; mpc_solver.py sha256 `b201649dedaccb11` |

### Per config: totals and QP size

| config | ticks | n | m | nnz P (upper) | nnz A | iter p50 / max | status (count) | closed loop p50 / p95 | solve_mpc_step mean / p50 / p95 |
|---|---|---|---|---|---|---|---|---|---|
| legacy/obs0/walls0 | 87 | 120 | 280 | 100-100 | 412-412 | 100 / 200 | 1: 62, 2: 23, 7: 2 | 5.66 / 5.96 | 5.49 / 5.46 / 5.81 |
| legacy/obs0/walls4 | 87 | 124 | 284 | 104-143 | 596-693 | 175 / 200 | 1: 45, 2: 13, 3: 14, 7: 15 | 6.26 / 6.64 | 6.09 / 6.16 / 6.35 |
| legacy/obs1/walls0 | 91 | 120 | 280 | 179-179 | 412-509 | 100 / 200 | 1: 70, 2: 19, 7: 2 | 6.97 / 7.52 | 6.93 / 6.91 / 7.28 |
| legacy/obs1/walls4 | 88 | 124 | 284 | 183-183 | 596-693 | 200 / 200 | 1: 27, 2: 30, 3: 14, 7: 17 | 7.60 / 8.28 | 7.45 / 7.52 / 7.68 |
| legacy/obs8/walls0 | 88 | 120 | 280 | 179-179 | 412-509 | 100 / 200 | 1: 67, 2: 18, 7: 3 | 7.59 / 8.22 | 7.54 / 7.53 / 7.90 |
| legacy/obs8/walls4 | 88 | 124 | 284 | 183-183 | 596-693 | 200 / 200 | 1: 29, 2: 29, 3: 14, 7: 16 | 8.14 / 8.57 | 8.05 / 8.11 / 8.35 |
| offset_free/obs0/walls0 | 83 | 141 | 321 | 101-101 | 531-531 | 125 / 200 | 1: 58, 2: 25 | 7.54 / 7.92 | 7.48 / 7.46 / 7.74 |
| offset_free/obs0/walls4 | 83 | 145 | 325 | 105-144 | 715-812 | 200 / 200 | 1: 9, 2: 42, 7: 32 | 8.34 / 9.00 | 8.24 / 8.25 / 8.46 |
| offset_free/obs1/walls0 | 87 | 141 | 321 | 180-180 | 531-628 | 125 / 200 | 1: 61, 2: 26 | 9.24 / 9.87 | 9.16 / 9.19 / 9.43 |
| offset_free/obs1/walls4 | 85 | 145 | 325 | 184-184 | 715-812 | 200 / 200 | 1: 9, 2: 50, 7: 26 | 9.88 / 10.48 | 9.73 / 9.74 / 9.92 |
| offset_free/obs8/walls0 | 85 | 141 | 321 | 180-180 | 531-628 | 125 / 200 | 1: 60, 2: 25 | 9.74 / 10.28 | 9.77 / 9.78 / 10.07 |
| offset_free/obs8/walls4 | 85 | 145 | 325 | 184-184 | 715-812 | 200 / 200 | 1: 9, 2: 50, 7: 26 | 10.55 / 11.60 | 10.39 / 10.38 / 10.63 |
| approach/45deg_4m | 63 | 120 | 260 | 179-179 | 373-509 | 25 / 150 | 1: 63 | 6.86 / 7.21 | 6.57 / 6.57 / 6.81 |
| approach/70deg_2.5m | 38 | 120 | 260 | 179-179 | 373-509 | 25 / 200 | 1: 34, 2: 4 | 6.69 / 7.38 | 6.68 / 6.60 / 7.13 |
| approach/jitter_4m | 59 | 120 | 260 | 179-179 | 373-509 | 25 / 50 | 1: 59 | 6.74 / 7.17 | 6.53 / 6.53 / 6.67 |
| **all matrix** | 1037 |  |  |  |  |  |  |  | 8.01 / 7.78 / 10.35 |
| **all approach** | 160 |  |  |  |  |  |  |  | 6.58 / 6.56 / 6.96 |

OSQP status: 1 solved, 2 solved_inaccurate, 3 primal_infeasible, 4 primal_infeasible_inaccurate, 5 dual_infeasible, 6 dual_infeasible_inaccurate, 7 max_iter_reached. "closed loop" is the single timed call inside the rig; "solve_mpc_step" the uninstrumented replay of the same tick.

### Per config: where the time goes (mean ms per tick, share of the sum)

| config | row assembly | csc conversion | osqp setup | osqp solve polish | obstacle linearisation | model jacobians | rollout | planner cost corridor | corridor queries | other | sum of parts | uninstrumented |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| legacy/obs0/walls0 | 1.748 (32 %) | 0.590 (11 %) | 1.261 (23 %) | 0.442 (8 %) | 0.026 (0 %) | 0.211 (4 %) | 0.048 (1 %) | 0.697 (13 %) | 0.374 (7 %) | 0.030 (1 %) | 5.427 | 5.493 |
| legacy/obs0/walls4 | 1.778 (30 %) | 0.615 (10 %) | 1.356 (23 %) | 0.863 (14 %) | 0.027 (0 %) | 0.213 (4 %) | 0.049 (1 %) | 0.699 (12 %) | 0.365 (6 %) | 0.034 (1 %) | 5.999 | 6.093 |
| legacy/obs1/walls0 | 1.833 (27 %) | 0.599 (9 %) | 1.300 (19 %) | 0.513 (7 %) | 1.096 (16 %) | 0.213 (3 %) | 0.049 (1 %) | 0.791 (12 %) | 0.411 (6 %) | 0.030 (0 %) | 6.835 | 6.927 |
| legacy/obs1/walls4 | 1.870 (25 %) | 0.620 (8 %) | 1.382 (19 %) | 0.914 (12 %) | 1.095 (15 %) | 0.213 (3 %) | 0.050 (1 %) | 0.787 (11 %) | 0.398 (5 %) | 0.035 (0 %) | 7.364 | 7.455 |
| legacy/obs8/walls0 | 1.849 (25 %) | 0.600 (8 %) | 1.309 (18 %) | 0.509 (7 %) | 1.221 (16 %) | 0.214 (3 %) | 0.049 (1 %) | 1.258 (17 %) | 0.416 (6 %) | 0.031 (0 %) | 7.456 | 7.539 |
| legacy/obs8/walls4 | 1.889 (24 %) | 0.624 (8 %) | 1.389 (17 %) | 0.898 (11 %) | 1.222 (15 %) | 0.216 (3 %) | 0.050 (1 %) | 1.258 (16 %) | 0.403 (5 %) | 0.035 (0 %) | 7.985 | 8.048 |
| offset_free/obs0/walls0 | 3.169 (43 %) | 0.716 (10 %) | 1.319 (18 %) | 0.616 (8 %) | 0.031 (0 %) | 0.317 (4 %) | 0.065 (1 %) | 0.727 (10 %) | 0.414 (6 %) | 0.034 (0 %) | 7.409 | 7.483 |
| offset_free/obs0/walls4 | 3.205 (39 %) | 0.743 (9 %) | 1.408 (17 %) | 1.202 (15 %) | 0.031 (0 %) | 0.321 (4 %) | 0.065 (1 %) | 0.730 (9 %) | 0.417 (5 %) | 0.038 (0 %) | 8.160 | 8.245 |
| offset_free/obs1/walls0 | 3.310 (37 %) | 0.723 (8 %) | 1.365 (15 %) | 0.765 (8 %) | 1.192 (13 %) | 0.320 (4 %) | 0.065 (1 %) | 0.820 (9 %) | 0.436 (5 %) | 0.034 (0 %) | 9.031 | 9.158 |
| offset_free/obs1/walls4 | 3.340 (35 %) | 0.750 (8 %) | 1.430 (15 %) | 1.212 (13 %) | 1.199 (12 %) | 0.322 (3 %) | 0.066 (1 %) | 0.819 (9 %) | 0.437 (5 %) | 0.038 (0 %) | 9.613 | 9.727 |
| offset_free/obs8/walls0 | 3.330 (35 %) | 0.726 (8 %) | 1.373 (14 %) | 0.760 (8 %) | 1.316 (14 %) | 0.321 (3 %) | 0.066 (1 %) | 1.286 (13 %) | 0.439 (5 %) | 0.035 (0 %) | 9.653 | 9.768 |
| offset_free/obs8/walls4 | 3.357 (33 %) | 0.750 (7 %) | 1.431 (14 %) | 1.219 (12 %) | 1.313 (13 %) | 0.322 (3 %) | 0.066 (1 %) | 1.284 (13 %) | 0.440 (4 %) | 0.038 (0 %) | 10.221 | 10.389 |
| approach/45deg_4m | 1.808 (28 %) | 0.584 (9 %) | 1.308 (20 %) | 0.234 (4 %) | 1.101 (17 %) | 0.215 (3 %) | 0.050 (1 %) | 0.799 (12 %) | 0.409 (6 %) | 0.031 (0 %) | 6.539 | 6.570 |
| approach/70deg_2.5m | 1.809 (27 %) | 0.582 (9 %) | 1.306 (20 %) | 0.333 (5 %) | 1.098 (17 %) | 0.214 (3 %) | 0.050 (1 %) | 0.794 (12 %) | 0.409 (6 %) | 0.031 (0 %) | 6.626 | 6.683 |
| approach/jitter_4m | 1.804 (28 %) | 0.586 (9 %) | 1.303 (20 %) | 0.206 (3 %) | 1.096 (17 %) | 0.213 (3 %) | 0.050 (1 %) | 0.789 (12 %) | 0.408 (6 %) | 0.031 (0 %) | 6.486 | 6.532 |
| **all matrix** | 2.543 (32 %) | 0.670 (8 %) | 1.360 (17 %) | 0.823 (10 %) | 0.820 (10 %) | 0.266 (3 %) | 0.057 (1 %) | 0.930 (12 %) | 0.412 (5 %) | 0.034 (0 %) | 7.914 | 8.012 |
| **all approach** | 1.807 (28 %) | 0.584 (9 %) | 1.306 (20 %) | 0.247 (4 %) | 1.099 (17 %) | 0.214 (3 %) | 0.050 (1 %) | 0.794 (12 %) | 0.409 (6 %) | 0.031 (0 %) | 6.540 | 6.583 |

Parts, as segments of the instrumented copy: **row assembly** = layout + rows_dynamics + rows_control + stage_terms + slack_terminal + rows_to_array + p_symmetrize + z_guess; **csc conversion** = csc; **osqp setup** = osqp_ctor + osqp_setup_call; **osqp solve polish** = osqp_warm_start + osqp_solve; **obstacle linearisation** = obstacle_lin; **model jacobians** = model_ref + model_jac; **rollout** = rollout; **planner cost corridor** = true_cost; **corridor queries** = corridor_query; **other** = step_prep + prep + result_check + result_split + info_dict. obstacle_lin and corridor_query are the calls made from the stage loop, taken out of stage_terms; planner_cost_corridor and the rollout include their own model steps and corridor queries.

### By segment (all matrix ticks)

| segment | mean | p95 |
|---|---|---|
| layout | 0.018 | 0.027 |
| rows_dynamics | 0.170 | 0.182 |
| rows_control | 0.209 | 0.216 |
| stage_terms | 1.975 | 2.760 |
| slack_terminal | 0.021 | 0.025 |
| rows_to_array | 0.102 | 0.118 |
| p_symmetrize | 0.032 | 0.040 |
| z_guess | 0.014 | 0.019 |
| model_ref | 0.044 | 0.053 |
| model_jac | 0.221 | 0.272 |
| obstacle_lin | 0.820 | 1.319 |
| corridor_query | 0.412 | 0.442 |
| rollout | 0.057 | 0.066 |
| true_cost | 0.930 | 1.288 |
| csc | 0.670 | 0.754 |
| osqp_ctor | 0.776 | 0.806 |
| osqp_setup_call | 0.583 | 0.648 |
| osqp_warm_start | 0.004 | 0.004 |
| osqp_solve | 0.819 | 1.230 |

### OSQP alone, on each tick's captured QP (mean ms per tick)

| config | OSQP() | OSQP(algebra='builtin') | setup() | of which py conversion | C setup | warm_start | solve() | C solve | C polish | x bit-identical |
|---|---|---|---|---|---|---|---|---|---|---|
| legacy/obs0/walls0 | 0.756 | 0.004 | 0.497 | 0.289 | 0.105 | 0.003 | 0.448 | 0.376 | 0.042 | 87/87 |
| legacy/obs0/walls4 | 0.756 | 0.004 | 0.582 | 0.292 | 0.191 | 0.003 | 0.871 | 0.798 | 0.041 | 87/87 |
| legacy/obs1/walls0 | 0.755 | 0.004 | 0.526 | 0.298 | 0.130 | 0.003 | 0.519 | 0.421 | 0.067 | 91/91 |
| legacy/obs1/walls4 | 0.752 | 0.004 | 0.601 | 0.297 | 0.202 | 0.003 | 0.918 | 0.861 | 0.027 | 88/88 |
| legacy/obs8/walls0 | 0.801 | 0.004 | 0.558 | 0.314 | 0.138 | 0.003 | 0.548 | 0.446 | 0.071 | 88/88 |
| legacy/obs8/walls4 | 0.782 | 0.004 | 0.626 | 0.311 | 0.210 | 0.003 | 0.936 | 0.875 | 0.030 | 88/88 |
| offset_free/obs0/walls0 | 0.762 | 0.004 | 0.541 | 0.296 | 0.144 | 0.003 | 0.626 | 0.534 | 0.063 | 83/83 |
| offset_free/obs0/walls4 | 0.772 | 0.004 | 0.639 | 0.302 | 0.233 | 0.003 | 1.223 | 1.181 | 0.012 | 83/83 |
| offset_free/obs1/walls0 | 0.792 | 0.004 | 0.607 | 0.314 | 0.188 | 0.003 | 0.807 | 0.680 | 0.096 | 87/87 |
| offset_free/obs1/walls4 | 0.790 | 0.004 | 0.666 | 0.312 | 0.246 | 0.004 | 1.259 | 1.214 | 0.013 | 85/85 |
| offset_free/obs8/walls0 | 0.788 | 0.004 | 0.603 | 0.311 | 0.187 | 0.003 | 0.796 | 0.669 | 0.095 | 85/85 |
| offset_free/obs8/walls4 | 0.769 | 0.004 | 0.646 | 0.303 | 0.238 | 0.003 | 1.234 | 1.191 | 0.013 | 85/85 |
| approach/45deg_4m | 0.831 | 0.004 | 0.576 | 0.327 | 0.142 | 0.003 | 0.261 | 0.135 | 0.095 | 63/63 |
| approach/70deg_2.5m | 0.810 | 0.004 | 0.563 | 0.319 | 0.139 | 0.003 | 0.362 | 0.244 | 0.088 | 38/38 |
| approach/jitter_4m | 0.763 | 0.004 | 0.533 | 0.302 | 0.130 | 0.003 | 0.211 | 0.099 | 0.083 | 59/59 |
| **all matrix** | 0.773 | 0.004 | 0.591 | 0.303 | 0.184 | 0.003 | 0.845 | 0.767 | 0.048 |  |
| **all approach** | 0.801 | 0.004 | 0.557 | 0.316 | 0.137 | 0.003 | 0.266 | 0.147 | 0.089 |  |

Micro: `osqp.OSQP()` 739 us against 1.3 us with `algebra='builtin'` (the default constructor tries to import the cuda and mkl backends first, and a failed import walks all 70 sys.path entries, every tick); one `np.pad` pair (4x4 -> 5x5 obstacle model, offset_free only) 38.4 us, 20 per tick.

### Persistent OSQP workspace (setup once per run, then update + warm_start + solve)

Pattern: the union of every tick's sparsity in the run, explicit zeros kept. "fresh" is today's OSQP() + setup + warm_start + solve. dx is the largest absolute difference of the full OSQP solution x from the fresh solve.

| config | fresh | update() | rho carried: update + solve | iter change p50 / min / max | status changed | max abs dx p50 / max | rho reset: status changed | max abs dx p50 | rho reset + y = 0: update + solve | iter change min / max | status changed | max abs dx max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| legacy/obs0/walls0 | 1.721 | 0.061 | 0.633 | +25 / -75 / +175 | 38/87 | 2.5e-02 / 4.5e-01 | 23/87 | 1.9e-02 | 0.505 | +0 / +0 | 0/87 | 2.7e-10 |
| legacy/obs0/walls4 | 2.230 | 0.089 | 0.913 | +0 / -125 / +75 | 31/87 | 2.6e-02 / 1.4e-01 | 27/87 | 1.6e-02 | 0.968 | +0 / +0 | 0/87 | 6.5e-08 |
| legacy/obs1/walls0 | 1.819 | 0.072 | 0.701 | +25 / -150 / +175 | 42/91 | 2.6e-02 / 4.7e-01 | 21/91 | 1.3e-02 | 0.593 | +0 / +0 | 0/91 | 4.6e-10 |
| legacy/obs1/walls4 | 2.291 | 0.095 | 0.960 | +0 / -175 / +100 | 32/88 | 4.1e-02 / 2.7e-01 | 35/88 | 4.4e-02 | 1.016 | +0 / +0 | 0/88 | 1.3e-06 |
| legacy/obs8/walls0 | 1.914 | 0.073 | 0.727 | +50 / -150 / +175 | 44/88 | 2.5e-02 / 2.9e-01 | 19/88 | 1.1e-02 | 0.608 | +0 / +0 | 0/88 | 4.6e-10 |
| legacy/obs8/walls4 | 2.353 | 0.099 | 0.978 | +0 / -175 / +125 | 32/88 | 4.2e-02 / 2.7e-01 | 36/88 | 4.5e-02 | 0.996 | +0 / +0 | 0/88 | 2.0e-07 |
| offset_free/obs0/walls0 | 1.944 | 0.076 | 0.433 | -75 / -175 / +25 | 17/83 | 4.5e-02 / 1.7e-01 | 17/83 | 4.5e-02 | 0.705 | +0 / +0 | 0/83 | 8.0e-11 |
| offset_free/obs0/walls4 | 2.640 | 0.105 | 1.080 | +0 / -175 / +75 | 46/83 | 2.0e-02 / 1.3e-01 | 50/83 | 2.2e-02 | 1.434 | +0 / +0 | 0/83 | 6.8e-08 |
| offset_free/obs1/walls0 | 2.214 | 0.095 | 0.530 | -75 / -175 / +75 | 22/87 | 4.5e-02 / 1.5e-01 | 22/87 | 4.5e-02 | 0.914 | +0 / +0 | 0/87 | 9.3e-11 |
| offset_free/obs1/walls4 | 2.721 | 0.115 | 1.247 | +0 / -175 / +150 | 53/85 | 3.8e-02 / 3.4e-01 | 44/85 | 4.1e-02 | 1.350 | +0 / +0 | 0/85 | 2.4e-07 |
| offset_free/obs8/walls0 | 2.196 | 0.094 | 0.531 | -75 / -175 / +50 | 22/85 | 4.4e-02 / 1.2e-01 | 22/85 | 4.4e-02 | 0.864 | +0 / +0 | 0/85 | 6.8e-11 |
| offset_free/obs8/walls4 | 2.658 | 0.109 | 1.195 | +0 / -175 / +75 | 46/85 | 4.0e-02 / 3.3e-01 | 51/85 | 3.9e-02 | 1.342 | +0 / +0 | 0/85 | 2.6e-06 |
| approach/45deg_4m | 1.675 | 0.078 | 0.294 | +0 / -75 / +0 | 0/63 | 6.4e-02 / 1.1e-01 | 0/63 | 4.4e-04 | 0.328 | +0 / +0 | 0/63 | 2.2e-09 |
| approach/70deg_2.5m | 1.741 | 0.076 | 0.393 | +0 / -50 / +100 | 3/38 | 4.1e-02 / 2.2e-01 | 3/38 | 1.1e-02 | 0.417 | +0 / +0 | 0/38 | 1.1e-08 |
| approach/jitter_4m | 1.517 | 0.075 | 0.267 | +0 / +0 / +0 | 0/59 | 1.4e-04 / 1.2e-01 | 0/59 | 1.4e-04 | 0.286 | +0 / +0 | 0/59 | 2.7e-10 |

### OSQP settings the Python path runs with (`prob.settings` after setup)

`_solve_rti` calls `osqp.OSQP().setup(P, q, A, l, u, warm_starting=True, verbose=False, polishing=True, max_iter=200)`, then `warm_start(x=z_guess)` (x only, no y) and `solve(raise_error=False)`; everything else is OSQP's default. Identical on every tick: True.

| setting | value |
|---|---|
| `adaptive_rho` | `1` |
| `adaptive_rho_fraction` | `0.4` |
| `adaptive_rho_interval` | `50` |
| `adaptive_rho_tolerance` | `5.0` |
| `alpha` | `1.6` |
| `cg_max_iter` | `20` |
| `cg_precond` | `1` |
| `cg_tol_fraction` | `0.15` |
| `cg_tol_reduction` | `10` |
| `check_dualgap` | `1` |
| `check_termination` | `25` |
| `delta` | `1e-06` |
| `device` | `0` |
| `eps_abs` | `0.001` |
| `eps_dual_inf` | `0.0001` |
| `eps_prim_inf` | `0.0001` |
| `eps_rel` | `0.001` |
| `linsys_solver` | `1` |
| `max_iter` | `200` |
| `polish_refine_iter` | `3` |
| `polishing` | `1` |
| `rho` | `0.1` |
| `rho_is_vec` | `1` |
| `scaled_termination` | `0` |
| `scaling` | `10` |
| `sigma` | `1e-06` |
| `time_limit` | `10000000000.0` |
| `verbose` | `0` |
| `warm_starting` | `1` |

Before OSQP C: osqp-python keeps only the upper triangle of P (`scipy.sparse.triu` of the symmetric CSC `_solve_rti` passes; zeros already dropped by `csc_matrix`, indices sorted) and clamps l/u to +-OSQP_INFTY = 1e+30. linsys_solver enum: {'OSQP_DIRECT_SOLVER': 1, 'OSQP_INDIRECT_SOLVER': 2}. Statuses `_solve_rti` treats as an infeasibility certificate (success False, warm start held): [3, 4, 5, 6].

### Checks

- replay of all 1237 ticks from the fixture encoding bit-identical to the closed-loop solve: 1237/1237; instrumented copy bit-identical: 1237/1237; still identical after the timing replays: True.
- fixture `tools/rust_parity/mpc/data/rti_ticks.jsonl.gz`: 723 ticks ({'matrix': 523, 'approach': 160, 'edge': 40}), 2.33 MB, sha256 `231ea11963a2d136cd30acd0fb10d47a97604a0045dbd5b194ae4bedfb898aff`; read back and solved from the file, bit-identical: True.
- reproducibility (`--compare`): 174 gated metrics, worst deviation 7.33 % (tolerance 10 %): **PASS**; fixture sha256 identical across runs: True.

| metric | run 1 | run 2 | deviation |
|---|---|---|---|
| ALL approach: corridor_queries | 0.409 | 0.430 | 5.10 % |
| ALL approach: csc_conversion | 0.584 | 0.596 | 1.93 % |
| ALL approach: model_jacobians | 0.214 | 0.219 | 2.22 % |
| ALL approach: obstacle_linearisation | 1.099 | 1.122 | 2.17 % |
| ALL approach: osqp_setup | 1.306 | 1.324 | 1.36 % |
| ALL approach: osqp_solve_polish | 0.247 | 0.250 | 1.03 % |
| ALL approach: planner_cost_corridor | 0.794 | 0.833 | 4.89 % |
| ALL approach: rollout | 0.050 | 0.051 | 2.21 % |
| ALL approach: row_assembly | 1.807 | 1.830 | 1.27 % |
| ALL approach: solve_mpc_step mean | 6.583 | 6.700 | 1.78 % |
| ALL matrix: corridor_queries | 0.412 | 0.434 | 5.30 % |
| ALL matrix: csc_conversion | 0.670 | 0.680 | 1.45 % |
| ALL matrix: model_jacobians | 0.266 | 0.271 | 2.08 % |
| ALL matrix: obstacle_linearisation | 0.820 | 0.837 | 2.05 % |
| ALL matrix: osqp_setup | 1.360 | 1.379 | 1.40 % |
| ALL matrix: osqp_solve_polish | 0.823 | 0.824 | 0.12 % |
| ALL matrix: planner_cost_corridor | 0.930 | 0.967 | 3.95 % |
| ALL matrix: rollout | 0.057 | 0.058 | 2.21 % |
| ALL matrix: row_assembly | 2.543 | 2.585 | 1.69 % |
| ALL matrix: solve_mpc_step mean | 8.012 | 8.122 | 1.37 % |
| approach/45deg_4m: solve_mpc_step mean | 6.570 | 6.695 | 1.91 % |
| approach/70deg_2.5m: solve_mpc_step mean | 6.683 | 6.767 | 1.26 % |
| approach/jitter_4m: solve_mpc_step mean | 6.532 | 6.662 | 1.99 % |
| legacy/obs0/walls0: solve_mpc_step mean | 5.493 | 5.590 | 1.76 % |
| legacy/obs0/walls4: solve_mpc_step mean | 6.093 | 6.205 | 1.83 % |
| legacy/obs1/walls0: solve_mpc_step mean | 6.927 | 7.062 | 1.96 % |
| legacy/obs1/walls4: solve_mpc_step mean | 7.455 | 7.564 | 1.47 % |
| legacy/obs8/walls0: solve_mpc_step mean | 7.539 | 7.619 | 1.05 % |
| legacy/obs8/walls4: solve_mpc_step mean | 8.048 | 8.123 | 0.93 % |
| offset_free/obs0/walls0: solve_mpc_step mean | 7.483 | 7.577 | 1.26 % |
| offset_free/obs0/walls4: solve_mpc_step mean | 8.245 | 8.348 | 1.25 % |
| offset_free/obs1/walls0: solve_mpc_step mean | 9.158 | 9.284 | 1.38 % |
| offset_free/obs1/walls4: solve_mpc_step mean | 9.727 | 9.852 | 1.29 % |
| offset_free/obs8/walls0: solve_mpc_step mean | 9.768 | 9.925 | 1.61 % |
| offset_free/obs8/walls4: solve_mpc_step mean | 10.389 | 10.490 | 0.97 % |

Largest deviations: approach/45deg_4m: osqp alone solve_wall 0.261 -> 0.242 (7.3 %); offset_free/obs0/walls0: osqp alone solve_wall 0.626 -> 0.670 (6.9 %); legacy/obs0/walls4: corridor_queries 0.365 -> 0.388 (6.5 %); offset_free/obs0/walls0: osqp alone setup_wall 1.314 -> 1.400 (6.5 %); approach/45deg_4m: osqp alone setup_wall 1.411 -> 1.324 (6.2 %).
<!-- END GENERATED -->
