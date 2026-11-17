# Isaac MPC tuning: how the night loop runs

Goal: a set of mpc_corr weights with which the car gets past an obstacle ahead **consistently**
(same outcome on every repetition and under small start-pose jitter), on the new MPC (target
always on the centreline, branch `mpc-centerline-target-linus`, merged here). "Good enough", not
millimetric: a run is good when the mission succeeds and the car centre stays within 3 cm of
R_safe_ref; the score only ranks sets.

## Pieces (all in `scripts/sim/isaac_tuning/`)

| file | what |
|---|---|
| `scenarios/S1..S5.yaml` | base_room layout `tuning` (6 x 6 m), car at (0.5, 3.0, 0), 4 m at 0.5 m/s, 20 s; S1 D0.5 on the line at 1.5 m, S2 D1.0, S3 D0.5 +-0.15 m (sign alternates), S4 D0.5 at 1.0 m, S5 none; seeded jitter +-5 cm / +-3 deg |
| `run_scenario.py` | one scenario x N (Isaac + stack + recorder + metrics, full teardown with an orphan check) |
| `run_set.py` | one parameter set on S1-S5 with one stack; appends `tuning/history.csv` |
| `recorder.py` | per run: ready, parameters read back from the live nodes, `/sim/scenario`, `/reset_all`, mission, CSV streams (no bags) |
| `metrics.py` | outcome, clearances, lateral error, time, oscillation, solver; score; aggregate |
| `tuning/space.yaml` | the only parameters the loop may touch, with min/max and how they reach the node |
| `tuning/check_params.py` | rejects anything outside the space |
| `tuning/night_loop.py` | the loop |
| `~/isaac_ws` branch `mpc-tuning` | cylinders, the `tuning` layout, the runtime `/sim/scenario` topic |

## The loop (`tuning/night_loop.py`)

```bash
cd ~/dev_ws/f1tenth_isaac_tuning
setsid nohup python3 scripts/sim/isaac_tuning/tuning/night_loop.py --max-iters 10 \
    > output/isaac_tuning/night.log 2>&1 < /dev/null &
# watch: output/isaac_tuning/night_<date>/status.md ; stop: touch .../STOP (or SIGTERM)
```

Detached on purpose: background work dies with the Claude Code session that started it.

1. **GPU**: stops `freeparty-llm.service` (its llama-server holds 5.4 of the 8 GB and made
   Isaac crash with ERROR_OUT_OF_DEVICE_MEMORY on 2026-11-18) and starts it again at the end.
   `--gpu-service ''` skips this. The preflight refuses to start Isaac with < 3.5 GB free.
2. **Session**: one Discovery Server (127.0.0.1:11878, domain 78) and one Isaac for the night.
3. **Screening** of the current set: S1-S4 x 3 + S5 x 1, one stack (MPC weights are only read at
   start, so a new set = a new stack; within a set `/sim/scenario` + `/reset_all` put each
   repetition back at its start). Row in `history.csv` (phase `screen`).
4. **Confirmation**, only if every screening run was good: S1-S4 x 10 + S5, repetitions 3..12
   (new jitters). All good -> stop, the set goes to `tuning/best.yaml`.
5. **Proposal**: `claude -p --tools ''` gets `space.yaml`, the last 20 rows of `history.csv` and
   the last set per scenario, answers with a YAML block. Out of the space, unparsable, or a
   repeat of a tried set -> a random perturbation of the best set so far (1-2 parameters, 15 %
   of their range), logged as such. The answer is kept as `it<NN>_<hash>/proposal.md`.
6. Budget: `--max-iters` (default 10 from `space.yaml`).

Crash handling: before each run the runner checks the container; if Isaac died it is restarted
and the run retried once; a stack that never gets ready is restarted once. Three sets in a row
with invalid runs stop the loop. Everything the loop started is stopped in `finally`.

## Measured times (linus, 2026-11-18, warm shader cache)

| step | wall |
|---|---|
| Isaac start to /clock | 19-20 s (cold cache: a few minutes, first start compiles shaders) |
| stack start to ready (recorder) | 12.5 s |
| per run: scenario + `/reset_all` (5 s, slam restart) + settle 2 s | 7 s |
| per run: mission 4 m + 2 s tail | 11-13 s (S5: 8.8 s to COMPLETE) |
| stack stop | 3-5 s, no leftovers |
| one fresh `run_scenario.py` run (Isaac + stack + run + teardown) | ~60 s |

Estimate per iteration: screening 13 runs x ~21 s + stack ~20 s = ~5 min, plus ~1-2 min for the
proposal; a confirmation 41 runs = ~15 min. 10 iterations + one confirmation: ~1.5-2 h, so a
night (8 h) holds ~50-60 iterations if the budget is raised.

## What the night needs (and what is still missing)

- Done: the new MPC merged; harness, metrics, loop contract, proof runs S5 and S1 (see the report).
- Missing: a full screening set has not been run yet (only single runs); the loop itself has not
  run a full iteration; the Claude proposer has not been exercised live (it falls back to random
  if it fails).
- The stack's obstacle list comes from the LiDAR (`unknown_objects_mode: active`, set for every
  run in `space.yaml` `fixed_node_params`): the plain cylinders are not YOLO classes.

## Open questions (not blocking)

1. **Sim vs car**: the car runs `unknown_objects_mode: off`, so on the car only YOLO-recognised
   objects reach the MPC; here only the LiDAR does. Tuned weights transfer only if the obstacle
   disc the MPC gets is similar (r from the LiDAR footprint vs footprint + class margin).
2. **Obstacle distance**: scenarios measure base_link -> obstacle centre (like the MPC tests);
   the bumper-to-surface gap is 1.5 - 0.405 - r (S1: 0.845 m). Is that the real test's "1.5 m"?
3. **Score weights** (margin 0.4, time 0.2, oscillation 0.2, tracking 0.2) and the 3 cm
   tolerance are a first guess.
4. **No lateral homing / no adaptive widening** on this line: after the swerve the car keeps
   driving parallel ~1 m off the original line. Fine in a 6 x 6 room, matters in a corridor.
5. **Exact symmetry** (obstacle exactly on the line, car exactly aligned) leaves the QP no side
   to prefer; the jitter breaks it in Isaac, the car might not. Needs a decision (a side rule).
6. **w_term attenuation** and **adaptive corridor width** are not in the space: the first was
   tried and dropped (no gain, stops the car), the second does not exist on this branch.
