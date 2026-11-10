# scripts/perf: the performance baseline tools

Measure only. Nothing here is imported by the stack and nothing here edits it.
A run's results live in `output/perf/<YYYYMMDD_HHMM>/` (start with its `REPORT.md`).

## Build (once per worktree)

```bash
git submodule update --init src/f1tenth_external/ackermann_mux src/f1tenth_hardware/vesc
source ~/dev_ws/jc_env.sh          # linus only: osqp / torch / ultralytics on PYTHONPATH
colcon build --symlink-install \
  --packages-ignore vesc vesc_driver vesc_ackermann f1tenth_hydra \
  --packages-up-to f1tenth_bringup f1tenth_logger corridor_perception_c
```

`--packages-ignore`, not `--packages-skip`: a skipped package is still required by its
dependents (`f1tenth_hardware` then fails with "check that vesc_driver has been built"),
an ignored one is invisible. `vesc_driver` does not build on linus (SUDO_TODO #4); only
`vesc_msgs` is needed. Level A alone needs no submodule.

Every tool sources `scripts/perf/perf_env.sh` or expects you to have done so.

## Level A: micro-benchmarks

```bash
source scripts/perf/perf_env.sh
python3 scripts/perf/bench_micro.py --out output/perf/<ts>                 # ~2 min
python3 scripts/perf/bench_micro.py --check-golden output/perf/<ts>/golden # equivalence
python3 scripts/perf/bench_micro.py --list
python3 scripts/perf/bench_micro.py --out /tmp/x --only 'mpc.*' --core 7
```

- `micro.csv`: one row per case (p50 / p95 / max ms, tracemalloc peak, calls per 100 ms
  control cycle and their product `ms_per_cycle`, whether the default stack runs it).
- `profiles/*.txt`: cProfile of the five costliest cases.
- `golden/*.npz`: inputs and outputs of each case; `--check-golden` re-runs the
  functions and compares (rtol 1e-9, atol 1e-12 by default). This is the phase 2
  equivalence test: loosen the tolerances on purpose, per case, when a change is
  allowed to move the numbers, and say so.
- The module docstring of `bench_micro.py` describes the method; `perf_scene.py` the
  synthetic scene. To add a case: subclass `Case`, decorate with `@register`.

For a before/after comparison use the same `--core` and take two runs of each side:
on linus two identical runs differ by up to ~10 % while the machine is shared.

`bench_yolo.py` times YOLO inference alone; `bag_reference.py <bag>` reads the stage
times the car recorded about itself (solver, tracker, scene mask) from a mission bag.

## Level B: the whole stack against a bag

```bash
scripts/perf/run_replay.sh ~/f1tenth_archive/mission_row/<run>/bag output/perf/<ts>/replay
```

One warm-up run and two measured 60 s runs: the car's own `supervisor_bringup.launch.py`
in `sim:=isaac use_sim_time:=true` (drivers off, logger and LLM off) on a private
loopback DDS (domain 77, Discovery Server 11911), `ros2 bag play --clock`, the camera
feeder (`replay_feeder.py`) and one probe (`probe.py`). Output: `nodes.csv` (CPU %, RSS
per node, difference between the runs), `topics.csv` (rates and stamp periods, `/tf` per
edge), `summary.json`. `--help` lists the options; read the header of `run_replay.sh`
and of `replay_feeder.py` for what the replay does not reproduce (IMU, raw camera,
missions).

It needs a bag with `/scan`, `/odom`, a compressed left image and a depth topic (the
2026-10-22/23 mission bags have them; older ones do not). It refuses to start under
3 GB of free memory and stops itself if the machine runs out: linus is shared.

It never calls `pkill` or `scripts/kill_ros2.py`: it stops only the process tree it
started. The supervisor's lock (`/tmp/component_supervisor.lock`) means no other
supervisor can run on the machine during a run, and a run does not start if one is.
