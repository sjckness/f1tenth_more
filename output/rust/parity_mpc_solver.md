# MPC RTI solver core in Rust: parity (R2)

Branch `main-jazzy-rust`, phase start b6909dc1, linus (i7-9700K, Ubuntu 24.04, Python 3.12.3,
numpy 1.26.4 with reference BLAS, osqp-python 1.1.3 = OSQP C 1.0.0, rustc 1.99.1), 2026-10-23.
Tools: `tools/rust_parity/mpc/parity_solver.py` (L1-L3), `src/f1tenth_control/mpc_controller/test/closed_loop_parity.py`
(L4), `tools/rust_parity/mpc/bench_solve.py` (timing). Numbers: `output/rust/parity_mpc_solver.json`,
`output/rust_main/r2_closed_loop_parity.json`, `output/rust_main/r2_bench_*.json`.

**Result: bit-identical.** The Rust core builds the same QP to the last bit (0 ulp, the same
sparsity pattern) on every one of 42,909 ticks, its OSQP returns the same iterate, iteration count,
status, polish status and residuals on the Python's QP, and the whole tick (u0, zopt, x_pred,
v_cmd_pred, cost, slacks, status, message, iterations, success) is bit for bit the Python's on all
42,909 ticks: the 723-tick fixture, 2,599 perturbed sweep ticks (401 more raise, with the same
exception type and message on both) and 39,587 live ticks captured from the whole mpc_controller
test suite. Nothing needed the 1e-14 (L1) or 1e-9 (L3) tolerance. In closed loop, all 32 runs of
the package's rigs are identical solve for solve, and the closed-loop golden SHA file (recorded
with the Python) is reproduced 20/20 by the Rust core. One tick costs **1.2 ms mean / 2.0 ms p95**
against 7.8 / 10.4 ms in Python (p95 5.2x faster; live configuration p95 2.2 against 11.1 ms).

## Scope

`mpc_solver._solve_rti` for `stage_spec None`, both speed models (legacy 4 states, offset_free 5
states with the soft speed-cap slack), everything the tick does:

| Python | Rust (`src/rust/f1tenth_mpc_core/src`) |
|---|---|
| `vehicle_model.f1tenth_state_fcn_dt_beta[_lag][_jacobian]` | `model.rs` |
| reference rollout, linearisation, every row and cost block of `_solve_rti`, `0.5 (P + P^T)`, `csc_matrix`, osqp-python's `triu` and +-1e30 clamp, `z_guess` | `qp.rs` |
| `_linearize_obstacle_cost_rti`, `_d_front_batch`, `_softplus_sq_derivs`, heading targets, `rate_weight_factor`, `planner_cost_corridor` (the reported cost) | `cost.rs` (corridor queries: R1's `query.rs`) |
| `osqp.OSQP().setup(...)`, `warm_start(x=z_guess)`, `solve()`, the accept / hold decision, `_rollout_true_dynamics`, the info dict | `solve.rs` + `ffi/osqp.rs` (OSQP C 1.0.0 through osqp-sys 1.0.1) |
| the call | `ffi/solver.rs` (C ABI) + `mpc_controller/mpc_native.py` (`solve_rti`) |

Stays Python: `scale_stage_weights` (done before the dispatch, for both), SLSQP, and every
`stage_spec` call (reverse_mpc's direction candidates, reverse_manoeuvre.py). The dispatch is
`solve_mpc_step(..., impl=None)`: rust only for `solver='rti'` and `stage_spec is None`.

## Layers

| Layer | Input | Result |
|---|---|---|
| **L1** QP assembly: `mpc_native.build_qp` vs the arrays osqp-python handed OSQP C (spied `osqp.OSQP.setup`, `prob._derivative_cache`) | every tick of L3 (42,909 QPs) | sparsity identical 42,909/42,909; values bit-identical 42,909/42,909 (max abs 0, max 0 ulp over P, q, A, l, u); warm start `z_guess` bit-identical 42,909/42,909; the fixture's recorded `qp_sha256` reproduced 723/723 |
| **L2** OSQP alone: `mpc_native.osqp_solve_csc` (the core's OSQP, settings, fresh workspace) on the Python's QP and warm start | the same 42,909 QPs | x bit-identical 42,909/42,909; iterations, status_val, status string, status_polish, rho_updates, rho_estimate, obj_val, prim_res, dual_res identical 42,909/42,909; effective settings (read back from the solver) equal to `prob.settings` on every field, 42,909/42,909 |
| **L3** the whole tick, py vs rust | fixture: 523 matrix + 160 approach + 40 edge ticks | 723/723 bit-identical, and 723/723 equal to the outputs recorded at capture |
| | sweep: 3,000 perturbed fixture ticks (seed 20261023: states, weights x0.3-3, 1-8 obstacles, 1-5 random walls hard/soft, 1-8 slots, warm-start noise, ts 0.05-0.2 / control period, psiRefTurn, a target line, vdes, footprint, horizon 1-30, 1e3-1e6 m offsets, tiny / huge w_obs, NaN speed, speed-model alpha/d/delay/no cap, slack weight / max) | 2,599 solved, all bit-identical; 401 raise, all with the same exception type and message (math domain error on an infinite heading, ZeroDivisionError on a zero wheelbase, KeyError for a missing obstacles_world, the x0 size ValueError, numpy's empty-argmin ValueError, osqp.OSQPException from OSQP C's non-convex KKT factorisation on NaN data) |
| | live: 39,587 ticks captured (F1TENTH_MPC_CAPTURE) while the whole mpc_controller suite ran: closed-loop rigs, node-level tests, boundary / stage-tracking / speed-channel / reverse-flag-off suites | 39,587/39,587 bit-identical; the 39,401 captured from the Python also equal to their recorded outputs |
| **L4** closed loop (`closed_loop_parity.py`) | 32 runs: object approach (5 scenarios x no obstacle / footprint), person pass (6), speed channel (legacy / offset_free x 0 / 1 / 8 obstacles x 0 / 4 live walls), 4 direct solves | 32/32 pass: same outcome, same end tick, final pose difference 0.0 m / 0.0 rad; 32/32 bit-identical (per-run SHA-256 of every solve); golden SHA (Python reference, `test/data/closed_loop_golden.json`, read only): py 20/20, rust 20/20 |

Statuses covered (the 42,909 L3 ticks): 1 solved 34,069, 2 solved inaccurate 7,136, 3 primal
infeasible 654, 4 primal infeasible inaccurate 134, 7 max iterations 916: both failure branches
(success False, warm start held) and the non-converged ones, where any rounding difference in the
data would have shown up in the iterate.

Gates (design R2 item 4): L1 sparsity identical and values within 1e-14 (target 0 ulp): **0 ulp**.
L2 bit-identical x, iterations, status: **yes**. L3 >= 99 % within 1e-9 with identical status,
outliers <= 1e-3: **100 % bit-identical, no outlier**. L4 same outcome and end tick, <= 1 cm,
<= 0.01 rad: **identical**.

## How bit identity is reached

The QP has to be the same bits: about half of the closed-loop solves stop at the iteration limit
or "solved inaccurate" (rti_profile.md), and a non-converged ADMM iterate follows every rounding
difference. So every operation is the one the Python performs, in its order:

- **Python floats vs numpy scalars.** `vehicle_model.py` computes on Python floats (`math.tan`,
  `math.cos`, ...: libm, the same glibc in the same process), so its failures are CPython's
  (`math.cos(inf)` raises, `math.cos(nan)` does not; a division by a zero wheelbase raises).
  `np.cos`/`np.exp`/`np.log1p`/`np.hypot` on this CPU are libm as well (no AVX-512, so no SVML
  loops; checked), but never raise.
- **`x ** 2` is `pow`, not `x * x`.** glibc's `pow(x, 2)` differs from the product in the last bit
  for about 0.1 % of arguments (measured). Every scalar square in the cost (`pen ** 2`,
  `(v - vdes) ** 2`, `du ** 2`, `d_lat ** 2`, `(k t) ** 2` in the Jacobian, the rate factor) is
  libm `pow`; on Python floats with CPython's `float_pow` special cases and its OverflowError, on
  numpy float64 without. LLVM would turn `pow(x, 2.0)` into `x * x`, so the exponent goes through
  `black_box`. Array squares (`d2 = dx**2 + dy**2` in the corridor query) are `np.square`, i.e.
  `x * x`, as in R1.
- **Small matrix products in reference-BLAS order.** `A_k @ x`, `B_k @ u`, `(-2 Wdu) @ last_u`,
  `E.T @ pref_nom` are numpy's `DOUBLE_gemv` -> `cblas_dgemv` -> netlib `dgemv`: a sequential sum
  from +0.0 per row. `m @ c` and `n @ pc` are `DOUBLE_dot` -> `cblas_ddot` (sequential for n < 5,
  then groups of five), plus DOUBLE_dot's own `sum = 0.`.
- **`P` accumulated dense in the Python's order.** Each term adds its whole numpy block
  (`(2 w) * np.outer(e, e)` incl. its zeros, so a NaN / inf weight spreads as in numpy), stage by
  stage in the same sequence, so every entry sees the same `+=` chain. Then `0.5 * (P + P.T)`,
  `csc_matrix` (drops exact zeros of either sign, keeps NaN, sorted rows), osqp-python's upper
  triangle; `A` row by row from fresh zero rows; `l`/`u` through `np.maximum(l, -1e30)` /
  `np.minimum(u, 1e30)`.
- **NaN and min/max rules.** Python's builtin `min`/`max` (keep the first argument unless the
  second compares) for the planner's `d_front` and the speed-cap slack guess; `np.minimum.reduce`
  (NaN propagates) for `_d_front_batch`; numpy's first-NaN argmin for the corridor query (R1).
- **OSQP.** Rust links the same OSQP C 1.0.0 (osqp-sys 1.0.1 vendors it; QDLDL v0.1.8 fetched by
  OSQP's CMake), built at -O2 without -march / FMA / fast-math, like the Python wheel's x86-64
  baseline build. A fresh solver per tick; every one of the 31 `OSQPSettings` fields is written
  explicitly from the Python's effective values (`warm_starting=True, verbose=False,
  polishing=True, max_iter=200` on top of `osqp_set_default_settings`; `allocate_solution` 1 and
  `profiler_level` 0, which osqp-python does not expose, are C's defaults) and read back after
  setup; `osqp_warm_start(x, NULL)`; the return codes of warm_start and solve are ignored, as
  osqp-python ignores them; no `l <= u` / finiteness check beyond OSQP C's own (NaN bounds reach
  OSQP C, as from Python). Unit test: C defaults + the four Python overrides == the core's settings.

## Exceptions and differences

The same exception TYPE for every input the Python fails on (and the same message where the
Python's is fixed): `x0` of the wrong size, `math.sin/cos/tan` of an infinity, a zero wheelbase or
prediction step, an overflowing Python-float square, an empty corridor, a missing
`corridor['obstacles_world']` (or `['psiRef']` with `w_psi` on), an OSQP setup error
(`osqp.OSQPException(code)`). Documented differences (mpc_native.py's docstring), none reachable
from MPC_corr:

- refused up front (ValueError) where the Python fails later or builds a degenerate QP: horizon
  < 1, a negative `boundary_max_sources`, `last_u` / `pref_nom` / a target-line point that is not
  two numbers, obstacle / boundary rows that are not three, corridor arrays of different lengths;
  speed-model arrays shorter than the horizon raise IndexError, as in the Python;
- inputs pass through `float()` / `np.asarray(dtype=float64)` first (a float32 corridor would be
  widened before the arithmetic instead of inside it); an n-D `x0` or warm start is raveled;
- the Rust path does not need the Python osqp package (OSQP C is linked into the library);
- numpy's RuntimeWarnings are not emitted; OSQP C's own error prints (stdout) are the same.

## Timing (`bench_solve.py`, linus, CPU 5 pinned, 7 repeats x 723 ticks = 5,061 calls each)

The committed runs, on the final library (`output/rust_main/r2_bench_{py,rust}_run{1,2}.json`,
load average 1.9-2.2, governor powersave):

| | Python mean / p50 / p95 / max | Rust mean / p50 / p95 / max | p95 speed-up |
|---|---|---|---|
| all fixture ticks, run 1 | 7.82 / 7.51 / 10.44 / 11.82 ms | 1.24 / 1.27 / 2.01 / 2.40 ms | 5.2x |
| all fixture ticks, run 2 | 7.85 / 7.60 / 10.40 / 12.29 ms | 1.23 / 1.26 / 1.99 / 2.51 ms | 5.2x |
| live config offset_free / 8 obstacles / 4 walls, run 1 | p50 10.44, p95 11.10, max 11.76 ms | p50 1.99, p95 2.15, max 2.35 ms | 5.2x |
| live config, run 2 | p50 10.58, p95 11.32, max 12.29 ms | p50 1.98, p95 2.32, max 2.51 ms | 4.9x |
| object approach, run 1 | p50 6.64, p95 7.27 ms | p50 0.52, p95 1.01 ms | 7.2x |

Two earlier pairs gave the same picture: under load 4-8 (other agents' builds) all ticks py p95
11.59 / 10.81 ms against rust 2.07 / 2.10 ms; on a quiet machine (load 1.2-1.6) py 10.17 / 10.43
against rust 1.95 / 1.92 ms.

Gate p95 rust <= p95 py, whole fixture and live config: **PASS** in every run
(`r2_bench_compare.log`). Mean ~1.2 ms against ~7.8 ms (6.3x). OSQP C's own work was 1.0 ms of
the Python's 8.0 ms mean (rti_profile.md); the Rust tick is now essentially that OSQP C time plus
~0.2 ms (QP build, rollout, true cost, ctypes both ways).
A persistent OSQP workspace (`rust_persistent`) is not built: it could save at most the
0.10-0.25 ms of setup and changes the numerics (rti_profile.md).

## Reproduce

```bash
source output/rust_main/env.sh
colcon build --symlink-install --packages-select f1tenth_mpc_core mpc_controller
tools/rust_parity/mpc/parity_solver.py --sweep 3000 [--capture-dir DIR] --json output/rust/parity_mpc_solver.json
# live captures: any run with F1TENTH_MPC_CAPTURE=DIR (e.g. the mpc_controller suite) writes DIR/rti_capture_<pid>.jsonl.gz
python3 src/f1tenth_control/mpc_controller/test/closed_loop_parity.py --json output/rust_main/r2_closed_loop_parity.json
for impl in py rust; do tools/rust_parity/mpc/bench_solve.py --impl $impl --cpu 5 --out /tmp/bench_$impl.json; done
tools/rust_parity/mpc/bench_solve.py --compare /tmp/bench_py.json /tmp/bench_rust.json
F1TENTH_MPC_SOLVER_IMPL=rust colcon test --packages-select mpc_controller ...   # every solver test on Rust
```

## Pending

- aarch64 (Thor, the Orin container): bit identity is not promised there (another libm build,
  possibly FMA in OSQP C or numpy); the tolerance gates apply (R7.2). `profile_rti.py` there with
  `--no-fixture` (R2.0 manager note).
- A launched mpc_corr node with mpc_solver_impl:=rust in the Isaac / Gazebo loop (R7.1).
