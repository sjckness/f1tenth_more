# Corridor parity: Python vs Rust (main-jazzy-rust R1)

MPC_corr's corridor geometry ported to Rust (`src/rust/f1tenth_mpc_core`, a ROS-free cdylib loaded
with ctypes by `mpc_controller/mpc_native.py`, selected by `corridor_impl:=py|rust`, default py).
linus (i7-9700K, x86_64, Ubuntu 24.04, Python 3.12.3, numpy 1.26.4 apt with netlib BLAS, glibc
libm, rustc 1.99.1), 2026-10-23. Library under test: the installed
`install/f1tenth_mpc_core/lib/f1tenth_mpc_core/libf1tenth_mpc_core.so` (opt-level 3).
Numbers: `output/rust/parity_corridor.json`, `output/rust_main/r1_*.json`.

**Result: bit-identical.** Every finite value, flag, index, key, type and exception is the same on
both sides, in every part below; nothing needed the 1e-12 tolerance (max |difference| 0.0). The
only bits that differ are the sign of some NaNs, and only where an input was already NaN or
infinite (sweep): 1,543 of 14.1 M geometry values and 578 of 396 k query values.

| Part | Geometry cases | Bit-identical | Within 1e-12 | Same exception | Queries | Bit-identical |
|---|---:|---:|---:|---:|---:|---:|
| 24 archived rebuilds x 7 shapes x 2 walls x 4 n | 1,344 | **1,344** | 1,344 | - | 22,848 | **22,848** |
| 10k edge-case sweep (seed 20261023) | 8,937 built + 1,063 raised | **8,907** (30: NaN sign only) | 8,937 | **1,063 / 1,063** | 43,987 | **43,409** (578: NaN sign only) |
| In-process capture, mpc_controller test suite | 2,432 calls + 9 raised | **2,432** | 2,432 | **9 / 9** | 9,235,864 | **9,235,864** |
| In-process capture, f1tenth_params tests | 226 + 2 raised | **226** | 226 | **2 / 2** | 0 | - |
| In-process capture, f1tenth_logger tests | 127 | **127** | 127 | - | 0 | - |
| Corpus replay (the three captures, offline) | 1,546 distinct + 11 raised | **1,546** | 1,546 | **11 / 11** | 30,446 sampled | **30,446** |
| Closed-loop rigs (golden SHA-256 of every solve) | 138 corridors, 20 runs, 1,218 solves | **20 / 20 runs identical to the golden** | | | | |
| Test suites on Rust (4 modes x 3 packages, 1,962 tests) | | **identical outcomes in every mode** | | | | |

## How to run it

```bash
source output/rust_main/env.sh; cd $WS
# 1. the test suites in four modes + the in-process capture (writes corpus.pkl), ~33 min
tools/rust_parity/corridor/run_corridor_tests.sh /tmp/corridor_suites
# 2. rebuilds, 10k sweep, the captured corpora replayed, timing, ~20 s
python3 tools/rust_parity/corridor/parity_corridor.py --sweep 10000 --timing 50 \
    --replay /tmp/corridor_suites/capture/mpc_controller/corpus.pkl \
    --replay /tmp/corridor_suites/capture/f1tenth_params/corpus.pkl \
    --replay /tmp/corridor_suites/capture/f1tenth_logger/corpus.pkl --json out.json
# 3. the closed-loop rigs' golden digests on both implementations, ~18 s
python3 tools/rust_parity/corridor/closed_loop_rigs.py --json rigs.json
```

`F1TENTH_MPC_CORE_LIB=<path to .so>` tests another build of the library (e.g. a fresh
`cargo build` in `src/rust/target/debug/`).

## What is compared

| Python (reference) | Rust (`src/rust/f1tenth_mpc_core`) |
|---|---|
| `f1tenth_params.corridor_geometry.corridor_curves` | `corridor::corridor_curves` (`mpc_corridor_curves`) |
| `f1tenth_params.corridor_geometry.corridor_curves_to_pose` | `corridor::corridor_curves_to_pose` (`mpc_corridor_curves_to_pose`) |
| `mpc_solver.nearest_centerline_index` | `query::nearest_centerline_index` (`mpc_nearest_centerline_index`) |
| `mpc_solver._nearest_corridor_frame`, `corridor_lateral_coordinates`, `corridor_heading_at` | `query::corridor_frame` (`mpc_corridor_frame`) |

The Rust side is always called the way MPC_corr calls it: through `mpc_native.py` (ctypes; ABI
version and the size of all five structs checked at load), which returns the same dict as the
Python: same keys in the same order, same value types, `ny` the same array object as `tx`.

**Comparator** (`tools/rust_parity/corridor/corridor_compare.py`): every key and its order, every
value's type (ndarray / float / bool / list / None), every flag (`feasible`, `ramp_clamped`), every
index (`argmin`), `ny is tx`, and every float (array elements, scalars, control points). A float
passes the gate within 1e-12 absolute, NaN == NaN; it is counted **bit-identical** when its 64 bits
are equal. An exception must have the same type and the same message on both sides.

## The parts

**Rebuilds** (`parity_corridor.py`, `src/f1tenth_params/test/data/object_rebuilds.json`). Each of
the 24 go_to_object rebuilds of the first test campaign as the planner's shapes: the straight
corridor on the target line; the ramp arc from the car unclamped, with the r_min clamp (0.923 m
left / 0.955 m right), and with r_min + the width rate (length_ref 3.0); the pose arc to the goal
unclamped, with r_min, with the width rate. Each with Bezier and trapezoid walls, at n = 2, 30, 120,
500: 1,344 corridors (1,256 feasible, 112 ramp-clamped), 2.43 M values, all bit-identical. The four
queries at the car, the goal, the lead-in origin, 12 points scattered around each corridor, a point
exactly on a sample and one past the end: 22,848 queries, all bit-identical.

**Edge-case sweep**, 10,000 seeded cases (seed 20261023): 5,451 ramp, 4,549 pose. A case can be in
several families:

| Family | Cases | What it stresses |
|---|---:|---|
| ramp, L ~ 0 (0, -0, 5e-324, 1e-300, 1e-15 .. 1e-3) | 2,161 | `ds` = 0, `linspace(0, L)` with a zero / subnormal step, the r_min clamp's `length > 0` guard |
| ramp, dpsi = +-pi | 369 | the wrap branch; also +-2pi, 3pi, -2.5pi, 1e3 among the angles |
| ramp, u_end == u_start (0, 0.4, 0.5, 1) | 1,099 | the `max(u_end - u_start, 1e-6)` floor of the smoothstep |
| ramp, u_start > u_end / outside [0, 1] | 1,115 | clip and the clamp on an inverted ramp |
| pose, tiny chord (0, 1e-300, 1e-15, 1e-9, 1e-6, 1e-3) | 1,120 | the 1e-6 handle floor, `interp` over a degenerate arclength |
| pose arriving backwards along the chord | 721 | the out-and-back curve, peak curvature |
| n = 2..5 / 6..400 / 1000, 4096 | 706 built at n = 2 | the gradient's one-sided ends only; the dense count 512 |
| non-finite inputs (NaN, +-inf in a coordinate, an angle or the length) | 490 | exception parity and NaN propagation |
| clamps at their limits (r_min 1e-9, 100, -1, 0; length_ref 0, -1, 1e-9, 1e9) | throughout | |
| huge coordinates (sigma 1e6, +-1e15) | throughout | |

8,937 cases built a corridor on both sides; 1,063 raised the same exception on both
(584 ZeroDivisionError from `1.0 / r_min` with r_min = 0 on the pose corridor, 283 ValueError
"a corridor needs at least 2 samples", 100 ValueError "math domain error" for an infinite heading
or an overflowing `psi_end - psi_start`, 96 ValueError for walls 'spline'). 8,907 of the 8,937
corridors are bit-identical; the other 30 differ only in the sign bit of NaNs, all 30 with a
non-finite input. Queries on each corridor (3 scattered points, plus a NaN point in 5 %): 43,987,
of which 578 differ only in a NaN's sign, all with a NaN or infinite input.

**In-process capture** (`corridor_capture_plugin.py`, mode capture, `pytest -p`). The plugin wraps
corridor_geometry's two functions and mpc_solver's four queries before any test module is imported,
lets the Python run and return its result unchanged, then runs the same inputs through Rust and
compares on the spot. Every call of the whole mpc_controller, f1tenth_params and f1tenth_logger
suites was compared, the closed-loop rigs included (object approach, person pass, stop ramp, gap,
speed-channel matrix, reverse scenarios, go_to_object / LLM end to end):

| Package | Geometry calls (distinct) | Call sites | Same exception | Queries compared |
|---|---:|---:|---:|---:|
| mpc_controller | 2,441 (1,392) | 11 | 9 | 9,235,864 (3,708,897 nearest, 2,693,698 frame, 1,818,070 heading, 1,015,199 lateral) |
| f1tenth_params | 228 (132) | 17 | 2 | 0 |
| f1tenth_logger | 127 (33) | 5 | 0 | 0 |

All 2,785 built corridors (7.57 M values) and all 9.24 M queries (21.0 M values) are bit-identical; every query
corridor was float64. Every geometry call is kept in `corpus.pkl` (deduplicated) and a sample of
30,446 queries; the offline replay of the three corpora reproduces the same result (table above).

**Closed-loop rigs** (`closed_loop_rigs.py`). `mpc_controller/test/closed_loop_golden.py` hashes the
exact float64 bytes of every solve's inputs and outputs, run by run, and its golden file was written
on the Python corridor. With `corridor_backend.DEFAULT_IMPL` = 'rust' all 20 runs (1,218 solves,
138 corridors, every one built by the Rust library and none by the Python) hash to the golden.

**The test suites on Rust** (`run_corridor_tests.sh` + `junit_compare.py`), 1,962 tests:

| Mode | What runs on Rust | mpc_controller | f1tenth_params | f1tenth_logger |
|---|---|---|---|---|
| py | nothing (reference) | 1,530 pass, 13 fail, 3 skip | 80 pass | 331 pass, 5 fail |
| rust (`F1TENTH_CORRIDOR_IMPL=rust`) | MPC_corr's two geometry call sites, real nodes and stand-ins | same | same | same |
| substitute (plugin) | corridor_geometry's two functions and the four solver queries, for every caller (test_corridor_geometry.py, reverse_manoeuvre, corridor_def, the solver) | same | same | same |
| capture (plugin) | the in-process comparison above | same | same | same |

Every test ID has the same outcome in all four modes (`output/rust_main/r1_corridor_suites_outcomes.json`).
The 13 + 5 failures are the baseline's (`output/jazzy_main/test_failing_ids.txt`: the 10 stale-default
mpc tests, model_log's header, flake8 and pep257 of mpc_controller; f1tenth_logger's copyright, flake8,
pep257 and the two robot_logger workspace-search tests).

## How the bit identity is reached

The port is operation for operation (`src/rust/f1tenth_mpc_core/src/npcompat.rs`), against what
numpy 1.26.4 actually executes on linus:

- `np.linspace` as `function_base.linspace` writes it (`i * step + start`, the `step == 0` branch,
  the exact endpoint); `np.cumsum` strictly sequential (add.accumulate, not pairwise);
  `np.gradient` with unit spacing and edge_order 1 (interior `/ 2.0`, ends `/ 1.0`);
  `np.interp` = `arr_interp` + `binary_search_with_guess` from compiled_base.c verbatim (the
  guess, the cache window, the NaN retry from the right end); `np.argmin` first minimum / first NaN;
  `np.clip` and `np.maximum` with numpy's NaN rules; Python's `max`/`min` (first argument on a tie
  or a NaN); CPython's `math.atan2` special cases; `math.sin/cos` of an infinite angle refused
  (ValueError "math domain error") as CPython does; `1.0 / r_min` with r_min = 0 a ZeroDivisionError,
  in the Python's order of checks.
- `x ** 2` is `np.square` (`x * x`); `x ** 3` is `np.power` = libm `pow(x, 3.0)`, not `x*x*x`
  (`pow3`, the exponent hidden from LLVM). `np.sin/cos/hypot/power` are libm's on this machine (no
  AVX-512, so no SVML loops; checked over 34k values: 0 differences), and Rust's
  `f64::sin/cos/hypot/powf` call the same glibc. `n @ (p - pc)` runs through the netlib reference
  BLAS `ddot` (`(0 + a0 b0) + a1 b1`, then numpy's own `0. +`), reproduced by `dot2`.
- Every product and sum in the Python's left-to-right order (Bezier terms, `d1`, `d2`, the
  curvature). Rust never contracts `a * b + c` into an FMA nor reassociates, at any opt-level.

## Representation differences (none reaches a caller)

- `xL`/`yL`/`xR`/`yR` are contiguous arrays, not column views of one `(n, 2)` array: same values,
  nothing reads the layout. `ny` is the `tx` array object, as in the Python.
- Inputs pass through `float()`/`int()` before the call, so a numpy float32 scalar would be widened
  before the arithmetic instead of inside it (no caller passes one).
- numpy's RuntimeWarnings (invalid value, overflow) are not emitted.
- `n > 1,000,000` is refused with ValueError (the Python would try; a failed Rust allocation would
  abort the process). The planner uses n = 120.
- With numpy-scalar `u_start`/`u_end` the Python returns `kappa_max` as `numpy.float64`, the Rust
  dict always as a Python `float` (the same value; R1 manager check: 1520 of 6000 such cases, no
  other key differs). No caller passes numpy scalars there. (Added in R2, as the R1 review asked.)
- NaN bit patterns: the gate compares NaN == NaN. The sign of a NaN produced from a NaN or
  infinite input follows numpy's operand order inside its loops (`0xfff8...` vs `0x7ff8...`);
  it carries no information and appears in no finite-input case.

## Timing (linus, median per call, 50 repeats x 24 rebuilds, n = 120, Bezier walls)

| | Python | Rust via mpc_native | Speed-up |
|---|---:|---:|---:|
| straight corridor | 155 us | 49 us | 3.1x |
| ramp arc + r_min | 154 us | 50 us | 3.1x |
| pose arc + r_min (960 dense samples) | 375 us | 125 us | 3.0x |
| one corridor query (lateral coordinates / frame) | 7.9 us | 16.9 us | 0.5x |

Of the 49 us, the library call itself (ctypes + Rust) is 14.6 us (pose: 85 us); the rest is the
binding: ten `np.empty` output arrays, the structs and the dict. MPC_corr rebuilds its corridor at
`corridor_update_period` (1 s), so this saves about 0.1 ms a second: R1's value is the
library, not the time. The queries stay on the Python solver's own numpy code: through ctypes their
per-call array conversion costs more than numpy's 120-element arithmetic. R2's Rust solver core
calls `query.rs` natively.

## Not covered here

- aarch64 (Thor, Orin): bit identity is not promised (another libm, numpy build and SIMD paths);
  the 1e-12 gate applies. R7.2 runs this harness on the Thor.
- numpy 2.x or an AVX-512 machine changes numpy's `sin/cos/power` loops (SVML / Highway); the
  harness then measures the gap instead of assuming it.
- `wall_turn.py`, `object_approach.py` and `reverse_manoeuvre.py` keep calling the Python geometry
  directly (design: microseconds of scalar math or one-off builds); the substitute mode ran their
  tests on the Rust functions anyway.
