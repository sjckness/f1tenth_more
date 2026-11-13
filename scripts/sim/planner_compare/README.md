# planner_compare: the three planner-controller modes on one command, in Isaac

One natural-language command ("go straight 3m" by default) goes through the **full stack**
once per `planner_controller_mode`: LLM planner → mission → behaviour tree → controller →
mux → Isaac car. The output is one report that compares paths, trajectories and CPU cost.

```bash
# on the sim host (Isaac, stack and LLM all on this machine), workspace built:
scripts/sim/planner_compare/run.sh
# variants
scripts/sim/planner_compare/run.sh --modes corridor_rrt,p2p_geometric
scripts/sim/planner_compare/run.sh --command "go ahead 2 meters" --target 2
scripts/sim/planner_compare/run.sh --isaac external          # Isaac already up (same --net/--domain)
scripts/sim/planner_compare/run.sh --analyze-only output/planner_compare/<session>
```

Expect about 3–5 minutes per mode, most of it the stack bringup. Results go to
`output/planner_compare/<YYYYmmdd_HHMMSS>/`:

| file | what |
|---|---|
| `report.md` | the comparison table and the four plots |
| `trajectories.png` | ground-truth trajectories of all modes, start frame, plus a zoomed lateral view |
| `paths.png` | per mode: corridor(s), planned paths or MPC predicted horizons, the RRT* tree, and the GT and EKF trajectories |
| `timeseries.png` | speed, steering command and lateral offset over time |
| `cpu.png` | controller-process CPU over time |
| `summary.json`, `summary.csv` | the table as data (`*.csv` is gitignored: `git add -f`) |
| `<mode>/` | raw recordings (`gt.csv`, `odom.csv`, `drive.csv`, `paths.jsonl`, `cpu.csv`, …), `mission.json`, `llm.log`, `run.json`, `meta.json` |
| `logs/` | Isaac, every stack bringup, ready, restart and recorder logs |

## What a run does

Each mode gets a fresh bringup, because the mode is fixed at launch:

1. `sim_isaac.sh stack planner_controller_mode:=<mode> enable_intelligence:=false`.
2. `pc_record.py ready`: the mode's controller node is up, no other controller is, and
   `/mission/status`, `/odometry/filtered` and `/sim/ground_truth/odom` are arriving.
3. `restart.sh`, i.e. `/reset_all`: the car goes back to the scene start pose and the EKFs and
   SLAM are zeroed, then checked. Doing it right before the command also undoes Isaac's
   parked-car creep (about 6.5 mm/s backwards, finding D20).
4. `pc_record.py record` runs in the background until the mission leaves RUNNING, plus 4 s
   so the stop is captured.
5. `llm.sh "<command>"`: llama-server translates the command, then the planner loads and
   starts the mission.
6. The stack is stopped, and the runner waits out the 5 s DDS lease before the next mode.

The runner holds three things for the whole session:

- **Discovery Server:** it starts one, so the per-mode stacks reuse it instead of killing
  it under Isaac.
- **llama-server:** started once through `llm.launch.py`, unless one already answers on
  127.0.0.1:8083. The binary is `$LLAMA_SERVER_PATH`, else stack_params
  `llama_server_path`, else `llama-server` on PATH, else `~/projects/llm/llama-*/llama-server`.
- **Isaac:** started once, scene `base_room`, layout `campaign`. The car starts with the
  wall 4.8 m ahead and the plant 1.3 m to the right at 2.6 m.

Ctrl+C stops everything that the runner started.

## How it measures

- **Frame:** the car's pose at the moment the mission went RUNNING is the origin, with x
  forward and y to the left.
  - Ground truth is moved into this frame using its own pose at that instant.
  - Paths and corridors are in odom, so they are moved using the EKF's pose at that
    instant.
  - The EKF-vs-GT gap at the end is reported in the table.
- **Reference tracked:**
  - corridor_rrt and p2p_geometric: the planned path (`/planner/path`).
  - mpc_corr: the corridor centreline (`/corridor`). The MPC's predicted horizon is also
    plotted.
- **CPU:**
  - What is counted: the controller process only, from `/proc/<pid>/stat` (utime + stime,
    all threads), sampled at 5 Hz over the mission's wall-clock window. The method is the
    same for all three controllers.
  - "CPU per sim second" removes Isaac's real-time factor.
  - "CPU per /drive msg" is the cost of one control tick.
  - "idle before" is the controller holding zero before the command arrives.
  - mpc_corr's own `solve_dt_sec` is on the ROS clock, so it reads 0 in sim time. The
    report says so when that happens.
  - The new nodes' `plan_ms` is wall time.
- **LLM:** the command is sent once per mode. The report checks that the three missions
  are identical (it hashes the moves). Temperature 0 makes this the expected case. On linus
  "go straight 3m" became one `drive straight`, speed 0.5, with stop condition
  `distance_reached 3.0`. That means the controllers see an open-ended drive and the
  behaviour tree ends it at 3 m: "stopped minus commanded" includes the stopping distance.

## Checked so far

- **Checked on linus without Isaac:** a kinematic fake car standing in for Isaac, with the
  real behaviour tree, the real LLM and each controller, recorded and analysed by these
  scripts. All three modes completed, so the recorder, report and plots work.
- **Also checked:** the runner's startup, its prechecks and its Ctrl+C cleanup.
- **Not yet run in Isaac:** the full-stack Isaac run is the first real use.
