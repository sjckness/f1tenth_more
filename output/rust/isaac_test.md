# Isaac test checklist: Rust guard + Rust costmap_boundary (linus only)

For: Andreas, 2026-10-19. Isaac Sim 5.1 + the full stack + the Rust nodes, all on **linus**
(`--net local`, Fast DDS, Discovery Server on 127.0.0.1:11811). Worktree
`~/dev_ws/f1tenth_more_rust`, branch `jazzy-rust`. Nothing here touches the Thor or the car.

## 0. Before you start (once)

```bash
cd ~/dev_ws/f1tenth_more_rust
git log --oneline -1                       # the jazzy-rust tip you test
docker ps                                  # empty: no f1t_isaac running
pgrep -af component_supervisor_node        # empty: no stack running (the main checkout's included)
# Only if src/ changed since the last build (the install is --symlink-install):
source ~/dev_ws/jc_env.sh && source ~/dev_ws/rust_underlay/install/setup.bash
colcon build --symlink-install --packages-ignore $(cat output/jazzy_port/linus_skip.txt)
```

Run every command below from `~/dev_ws/f1tenth_more_rust` (sim_isaac.sh uses the workspace
it lives in). The main checkout `~/dev_ws/f1tenth_more` is a different workspace and branch;
do not start its stack at the same time (same domain 42, same Discovery Server port).

## 1. The two configurations

| Run | Command (Isaac + stack, one terminal; Ctrl+C stops both) |
|---|---|
| **PY** | `scripts/sim/sim_isaac.sh all guard:=shadow impl:=py` |
| **RUST** | `scripts/sim/sim_isaac.sh all guard:=shadow impl:=rust` |

- `guard:=shadow`: the Rust guard runs next to the BT and publishes decisions on
  `/proximity_guard/decision`, **no command**. The BT's emergency lane is the one that stops.
- `impl:=py|rust`: which costmap_boundary_node runs. Same node name, same topics; never both.
- Add `--layout obstacle` for scenario B2, `--gui` for the Isaac window.
- On linus (8 CPUs) sim_isaac.sh adds `config/sim/hosts/linus.yaml` automatically (mpc_corr on
  6,7, yolo on 4,5: the car's 10,11 and 8,9 do not exist here). llama-server is not on linus,
  so `enable_intelligence` is false and missions are started with e2e_checks.py (below), not
  with llm.sh.
- Switch back: start without the two arguments (defaults `guard:=off impl:=py`), or with
  `guard:=off impl:=py` explicitly. Nothing is persisted; no parameter file was changed.
- Alternative, fully scripted (what was run on 2026-10-18, section 5):
  `tools/e2e_test.sh --out <dir> guard:=shadow impl:=rust enable_mission_logger:=true`
  (`--keep-running` leaves sim + stack up afterwards; stop with SIGTERM to `sim_isaac.sh all`).

A second terminal for every check below, in the stack's environment (super client):

```bash
scripts/sim/sim_isaac.sh shell             # prompt [sim local d42]
```

## 2. Ready checks (both runs, after "Started 'behavior'" in the stack log)

```bash
ros2 node list | grep -E 'proximity_guard_node|costmap_boundary_node|behavior_executor'
# RUST: the costmap node must be the Rust binary:
ps -eo pid,args | grep -E 'costmap_boundary_node' | grep -v grep
#   impl:=py   -> .../f1tenth_costmap/costmap_boundary_node (python3)
#   impl:=rust -> .../f1tenth_costmap_boundary_rs/lib/f1tenth_costmap_boundary_rs/costmap_boundary_node
ros2 topic echo --once /proximity_guard/decision     # message: clear, mode: shadow
ros2 topic hz /scan /proximity_guard/decision        # both ~40 Hz (Isaac /scan rate x RTF)
ros2 topic hz /costmap/boundaries /costmap/front_clearance   # ~10 Hz each (sim time x RTF)
ros2 topic echo --once /costmap/boundaries           # frame_id base_link, 0-3 constraints
```

Pass: the guard says `clear` with `tf_fallback: false` (the laser TF arrived); no
`no_scan_yet` / `stale`; boundaries at 10 Hz; `ros2 topic info /costmap/boundaries -v` shows
exactly ONE publisher.

## 3. Scenarios

Record the guard against the BT for each scenario (third terminal; stop it with
`touch /tmp/stop_rec`; writes the summary JSON):

```bash
rm -f /tmp/stop_rec
scripts/sim/sim_isaac.sh run -- python3 tools/rust_parity/proximity/guard_vs_bt.py \
    --out /tmp/guard_vs_bt_<run>_<scenario>.json --stop-file /tmp/stop_rec
```

### A: normal mission

```bash
scripts/sim/sim_isaac.sh run -- python3 tools/sim/e2e_checks.py mission \
    --mission $(ros2 pkg prefix f1tenth_behavior)/share/f1tenth_behavior/missions/drive_test_straight_2m.json \
    --timeout 120
scripts/sim/restart.sh --net local         # back to the start pose between missions (default --net eth)
# then: go_to_person.json (perception + semantic tracks + MPC), same command
```

Check:
- the mission reaches COMPLETE (the JSON's `final_state`); PY and RUST give the same outcome and
  a similar ground-truth distance;
- recorder: `front_disagree` = 0 (the guard's front decision equals the BT's front decision on every
  scan), `rs_stop_decisions` = 0 or only `tripped`, no `stale`/`no_scan_yet`;
- `/costmap/boundaries` keeps 10 Hz while driving (`ros2 topic hz`), front_clearance plausible
  (`ros2 topic echo /costmap/front_clearance`);
- control path rates: `ros2 topic hz /drive /ackermann_drive /odometry/filtered /scan` (MPC
  10 Hz, EKF 20 Hz decided rate after cpu-cleanup, scan ~40 Hz x RTF);
- CPU: `top -o %CPU` or the e2e perf sampler (`tools/sim/perf_sampler.py`); in RUST the
  costmap_boundary_node process should be ~1 % instead of ~8 %.

### B1: obstacle right in front of the parked car (the guard must trip)

The box placed 0.10 m in front of the bumper (inside the 0.15 m threshold), car parked:

```bash
scripts/sim/sim_isaac.sh all guard:=shadow impl:=rust --layout obstacle \
    --set "scene.obstacle.position=[1.955, 2.5]"
```

(obstacle layout: car base_link at x 1.2, bumper at 1.605; the box is 0.5 m deep, so its near
face is at 1.955 - 0.25 = 1.705.)

**The parked Isaac car creeps backwards** (about -6.5 mm/s, jazzy-campaign finding D20): by the
time the stack is up it is ~0.4 m behind its start pose and nothing trips. Start the recorder,
then put the car back and look within ~8 s (0.10 m -> 0.15 m at 6.5 mm/s):

```bash
scripts/sim/sim_isaac.sh reset             # /sim/reset: car (and person) at their start poses
```

Check:
- `ros2 topic echo /proximity_guard/decision`: `tripped: true`, `min_front_m` about 0.10, `stop: true`
  (shadow: no command), every scan;
- `ros2 topic echo /behavior/tree_status`: `active_lane: emergency`, `emergency_trip:
  IsProximityTooClose`; `/safety_stop` carries `base_link/emergency` only (shadow);
- recorder: `front_disagree` = 0, `py_front_trips` = `rs_trips`.

### B2: obstacle on the path of a mission

`drive_straight_to_wall_3m.json` is no use here: it stops on `front_clearance < 3.0 m`, and the box
is 3.0 m ahead, so it ends at once. Use the 2.9 m straight drive into the box instead
(`tools/rust_parity/isaac/b2_straight_into_box_2p9m.json`: the bumper starts 2.60 m from the box).
One command does sim + recorder + reset + mission + stop (results in the OUT directory):

```bash
tools/rust_parity/isaac/scenario.sh /tmp/b2_shadow \
    tools/rust_parity/isaac/b2_straight_into_box_2p9m.json --layout obstacle guard:=shadow impl:=rust
```

Expected (2026-10-18): the mission ends ABORTED (timeout) because the BT's IsProximityTooClose
stops the car in front of the box (after ~2.47 m, MPC avoidance turns it ~75 deg on the way); the
guard trips on exactly the same scans (`front_disagree` 0); `/safety_stop` only
`base_link/emergency` in shadow. The same script runs scenario A:
`tools/rust_parity/isaac/scenario.sh /tmp/a src/f1tenth_behavior/missions/drive_test_straight_2m.json guard:=shadow impl:=rust`.

### Optional: guard:=active

Only after A and B1 agree (all of this was run on 2026-10-18, section 5). `guard:=active` adds the guard's own zero-speed stops on the
`safety_stop` lane (frame_id `base_link/proximity_guard`). In B1 both the BT and the guard then
stop; on `/ackermann_drive` the frame_id says which stop the mux forwarded. Note that in active
mode the guard also stops while there is no /scan (at start-up until the first scan, and on any
gap over 200 ms), by design.

## 4. Switch back

Ctrl+C in the sim_isaac.sh terminal (stops the stack, Isaac, the Discovery Server); if anything is
left: `scripts/kill_ros2.py -y` and `docker stop -t 40 f1t_isaac`. Next start without
`guard:=` / `impl:=` = the stack as before. The main checkout (`~/dev_ws/f1tenth_more`,
jazzy-campaign) is untouched and has neither switch.

## 5. What was verified on linus on 2026-10-18

All on linus, `--net local`, from `~/dev_ws/f1tenth_more_rust` at the commits of this branch.
Raw data: `output/rust/isaac_smoke/`.

| Run | What | Result |
|---|---|---|
| e2e, `impl:=py guard:=shadow` | `tools/e2e_test.sh` (9 steps) | up, hz, perception, 2 m mission, reset, go-to-person, perf: PASS. graph and tf FAILED on two stale expectations in the e2e script, fixed afterwards (wall_distance off since cpu-cleanup; laser z 0.190 since the merge) |
| e2e, `impl:=rust guard:=shadow` | same, with the two fixes | **9/9 PASS** |
| mission outcomes py / rust | 2 m straight (distance error), go_to_person (gap car front -> person) | 0.015 / 0.025 m; 0.905 / 0.904 m; both COMPLETE, same durations (20.3 s; 18.4 / 18.6 s) |
| guard vs BT, e2e py | 35,120 scans matched by stamp | **0 front disagreements**; all clear; guard `stale` 6 decisions (sim stalls > 200 ms), `no_scan_yet` 11 at start, tf_fallback 8 scans at start |
| guard vs BT, e2e rust | 9,965 scans | **0 disagreements**; `stale` 3; tf_fallback 58 scans at start (before /tf_static) |
| B1, `guard:=shadow` | box 0.10 m from the bumper, after `/sim/reset`, 856 scans | Python front trips 589 = Rust trips 589, **0 disagreements**; BT emergency episode `IsProximityTooClose`; `/safety_stop` 738 x `base_link/emergency`, **0 from the guard** |
| B1, `guard:=active` | same, 20 s | `/safety_stop`: 583 x `base_link/proximity_guard` + 731 x `base_link/emergency`; `/ackermann_drive` (mux output): 519 guard + 590 BT stops + 14 MPC commands (once the car had crept out of the band) |
| A, `guard:=active impl:=rust` | 2 m straight, campaign layout | COMPLETE 20.3 s; guard clear on 2,942/2,942 scans, **0 guard stops**; `/ackermann_drive` MPC only |
| A, `guard:=active impl:=py` | same (tools/rust_parity/isaac/scenario.sh) | COMPLETE 20.3 s; 0 trips, 0 disagreements; **1 guard stop** from one `stale` (sim stall), forwarded once by the mux; no effect on the outcome |
| B2, `guard:=shadow` | 2.9 m straight into the box | BT e-stop in front of the box (ABORTED by timeout as intended), travel 2.47 m; trips Python 746 = Rust 746, 0 disagreements; `/safety_stop` BT only |
| B2, `guard:=active` | same | trips 744 = 744, 0 disagreements; `/safety_stop` 744 guard + 936 BT; mux forwarded 635 guard + 711 BT; car stops at the same place as in shadow (x 3.641) |
| costmap node in Isaac | census | Python 9.0 % / 89 MB, Rust 1.7 % / 38 MB; the Rust node came back after `/reset_all` restarted the slam component |
| resources | VRAM, RAM, CPU | VRAM max 3.6 of 8.1 GB (Isaac 1.7, YOLO 0.34), RAM max 20.2 of 32 GB, host CPU mean ~50 %: no YOLO change needed |

Latency under sim time is not meaningful (stamp and now come from /clock); the wall-clock latency
is in `parity_proximity.md` section 5 (0.63 ms p50 scan -> stop).

**Watch for tomorrow:**
- `stale` decisions in shadow are sim stalls (wall-clock gaps > 200 ms in /scan), 3-6 per e2e
  session. In `guard:=active` each is a stop for about 50-250 ms. If that disturbs a run, raise the
  timeout for sim only in `config/sim/isaac.yaml` (the overlay every sim component gets as global
  parameters): `proximity_guard_node: {ros__parameters: {scan_timeout_ms: 500.0}}`. Not done and
  not tested: the default is the car's value, and shadow mode is unaffected.
- In `guard:=active` the guard also stops until the first /scan arrives (~4 s after start).
