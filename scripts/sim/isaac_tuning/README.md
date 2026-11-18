# scripts/sim/isaac_tuning: Isaac harness for tuning mpc_corr on "obstacle ahead"

Headless Isaac Sim scenarios on linus with the car's stack (`scripts/sim/sim_isaac.sh`,
`sim:=isaac`, the same supervisor bringup), MPC parameters passed as launch overlays (never by
editing `stack_params.yaml`), every run scored from live topics (no bags). The night loop and
its numbers: [`tuning/PLAN.md`](tuning/PLAN.md).

```bash
cd ~/dev_ws/f1tenth_isaac_tuning          # branch isaac-mpc-tuning, built (colcon, --symlink-install)
python3 scripts/sim/isaac_tuning/run_scenario.py --scenario S1 \
    --params scripts/sim/isaac_tuning/tuning/baseline.yaml --repeat 3
python3 scripts/sim/isaac_tuning/run_set.py --params scripts/sim/isaac_tuning/tuning/next_params.yaml
python3 scripts/sim/isaac_tuning/tuning/check_params.py scripts/sim/isaac_tuning/tuning/next_params.yaml
python3 -m pytest scripts/sim/isaac_tuning/test -q                  # pure parts, no ROS
```

**Moving obstacles (mpc-dodge, 2026-11-18):** the dodge family D(d, seed) (`dodge.py`, Isaac layout
`dodge`), `ramp.py` (climb to d_fail), `evaluate.py` (A/B of parameter sets on D, M1-M4, S1-S5),
`tuning/dodge_loop.py` (the tuning loop on d_fail), persons P1-P4 (layout `people`), the analysis
scripts in `analysis/`. Results and what is open: [`docs/mpc_dodge/REPORT.md`](../../../docs/mpc_dodge/REPORT.md).

Output under `output/isaac_tuning/<timestamp>_<scenario>/`: one directory per run (CSV streams,
`meta.json`, `run.json` with outcome, clearances and score), `session.json` (wall times, the
orphan check), `aggregate.json`.

Needs:
- the Isaac image `isaac_ws_devcontainer-isaac-sim:latest` (`docker/Dockerfile`, ~23 GB) and
  `~/isaac_ws` on branch `mpc-tuning` (cylinders, layout `tuning`, `/sim/scenario`);
- >= 3.5 GB of free GPU memory: stop `freeparty-llm.service` first (`night_loop.py` does it);
- no other ROS stack of this workspace and no `f1t_isaac` container running (preflight).

Private ROS domain 78 and Discovery Server port 11878 (never the car's 42 / 11811).
`env.sh` gives a shell the same environment (`source scripts/sim/isaac_tuning/env.sh`).
