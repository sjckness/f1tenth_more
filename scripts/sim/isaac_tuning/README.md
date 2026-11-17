# scripts/sim/isaac_tuning: Isaac harness for tuning mpc_corr on "obstacle ahead"

Work in progress (branch `isaac-mpc-tuning`). Runs short headless Isaac scenarios on linus with
the car's stack (`scripts/sim/sim_isaac.sh`, `sim:=isaac`), MPC parameters passed as an override
YAML (never by editing `stack_params.yaml`), and scores each run from live topics (no bags).

- `scenarios/S1..S5.yaml`: the scenarios (base_room, layout `tuning`, 4 m at 0.5 m/s, seeded
  start-pose jitter of +-5 cm / +-3 deg).
- `docker/Dockerfile`: minimal Isaac Sim 5.1 image for `~/isaac_ws` (tag
  `isaac_ws_devcontainer-isaac-sim:latest`).

The Isaac side (cylinder obstacles, the `tuning` layout, the runtime `/sim/scenario` topic) lives
in `~/isaac_ws`, branch `mpc-tuning`.
