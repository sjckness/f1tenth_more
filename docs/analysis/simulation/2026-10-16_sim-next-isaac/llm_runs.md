# LLM planner end to end in the Isaac sim

Two machines: Isaac Sim on linus (`sim_isaac.sh isaac --net eth --scene base_room --layout campaign`, or `--scene office`), the jazzy-campaign stack on the Thor (`sim_isaac.sh stack --net eth enable_intelligence:=true`). The runs were driven on the Thor by `tools/sim/llm_runs.py` inside `sim_isaac.sh run --net eth`.

Each command ran exactly as on the car:
1. `/reset_all`, then 5 s settle.
2. `ros2 run llm llm_planner_node "<command>"`: translate, write the mission, then abort, load and start it.
3. Wait for `/mission/status` COMPLETE or ABORTED.

Ground truth comes only from the sim: `/sim/ground_truth/odom`, `/objects` and `/front_range`.

- **Base room:** run 1 ran before the wheel-torque limit (decisions S20); run 2 ran on the final sim. The verdicts are the same in both runs.
- **Office:** ran on the final sim.

The raw data for each run is in `llm_runs_run1/`, `llm_runs_run2/` and `llm_runs_office/`:
- `<id>.mission.json`
- `<id>.planner.log` (the RESULT line holds the LLM's raw answer)
- `<id>.trajectory.csv`
- `results.jsonl`
- `isaac.log`

All rows: `llm_runs.csv`.

## How a run is scored (decisions S17)

- **Translation OK:** the mission has the shape the command asks for.
  - go ahead N: one `distance_reached` N move.
  - approach the wall: one `front_clearance` move.
  - go to X: one `go_to_object` move with target X.
  - turn left at the wall: a `front_clearance` move, then a `wall_turn` left 90°.
  - mixed: `distance_reached` 2.0, then go_to person.
- **Success:** translation OK, final state COMPLETE, no contact (footprint within 1 cm of an object), and the campaign criterion from `first_test_campaign/prompts.yaml` is met:

  | command | criterion |
  |---|---|
  | go ahead 3 m | 3.0 ± 0.15 m, lateral < 0.2 m |
  | approach the wall | 0.5 ± 0.2 m from the wall |
  | go to the person | 1.0 ± 0.3 m |
  | go to the potted plant | ≤ 0.5 m and facing it, bearing < 30° (M03's criterion) |
  | turn left at the wall | turn starts 1.0 ± 0.2 m from the wall, then 90 ± 15° |
  | mixed | first leg 2.0 ± 0.15 m, then person 1.0 ± 0.3 m |

- **Distances:** from the car footprint (rear -0.082 m to bumper +0.405 m, ±0.136 m) to the object's ground-truth box. "Stop error vs commanded" is the measured value minus the mission's own value (wall 2.0 m, go_to gap 0.6 m, distance).
- **Durations:**
  - `running_s`: from RUNNING to the end.
  - `command_to_end_s`: from the planner call to the end. It includes about 10 s for the planner process and the mission's 15 s countdown.

## Results

| run | command | translation_ok | final_state | success | result_vs_intent | stop_error_vs_commanded_m | min_clear_person_m | min_clear_plant_m | min_clear_walls_m | running_s | command_to_end_s | stop_reasons |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| base_room run 1 | go ahead 3 meters | ✅ | COMPLETE | ✅ | along 3.038 m, lateral 0.03 m | 0.038 | 1.794 | 0.882 | 1.007 | 10.6 | 41.8 | move_0_straight: stop_condition:distance_reached |
| base_room run 1 | approach the wall | ✅ | COMPLETE | ❌ | wall gap 1.606 m | -0.394 | 1.901 | 0.881 | 1.018 | 9.5 | 42.3 | move_0_straight: stop_condition:front_clearance |
| base_room run 1 | go to the person | ✅ | COMPLETE | ✅ | gap 0.837 m, bearing -0.3 deg | 0.237 | 0.835 | 1.731 | 1.016 | 11.4 | 44.1 | move_0_go_to: stop_condition:object_reached |
| base_room run 1 | go to the potted plant | ✅ | ABORTED | ❌ | gap 1.123 m, bearing 7.3 deg | 0.523 | 3.615 | 1.119 | 1.003 | 5.1 | 35.6 | move_0_go_to: go_to_object:target_lost |
| base_room run 1 | turn left at the wall | ❌ | COMPLETE | ❌ | turn starts 2.114 m from wall, heading 95.6 deg | None | 0.063 | 0.902 | 0.837 | 17.7 | 48.2 | move_0_turn: stop_condition:orientation_delta; move_1_straight: stop_condition:front_clearance |
| base_room run 1 | go ahead 2 meters then go to the person | ✅ | COMPLETE | ✅ | leg 1 1.998 m, person gap 0.753 m | 0.153 | 0.753 | 0.931 | 1.017 | 13.7 | 46.2 | move_0_straight: stop_condition:distance_reached; move_1_go_to: stop_condition:object_reached |
| base_room run 2 | go ahead 3 meters | ✅ | COMPLETE | ✅ | along 3.084 m, lateral 0.03 m | 0.084 | 1.777 | 0.883 | 1.005 | 9.6 | 40.6 | move_0_straight: stop_condition:distance_reached |
| base_room run 2 | approach the wall | ✅ | COMPLETE | ❌ | wall gap 1.583 m | -0.417 | 1.888 | 0.882 | 1.006 | 8.6 | 38.9 | move_0_straight: stop_condition:front_clearance |
| base_room run 2 | go to the person | ✅ | COMPLETE | ✅ | gap 0.823 m, bearing 0.8 deg | 0.223 | 0.82 | 1.753 | 1.001 | 10.5 | 40.7 | move_0_go_to: stop_condition:object_reached |
| base_room run 2 | go to the potted plant | ✅ | ABORTED | ❌ | gap 1.186 m, bearing 9.7 deg | 0.586 | 3.62 | 1.181 | 1.005 | 4.9 | 35.1 | move_0_go_to: go_to_object:target_lost |
| base_room run 2 | turn left at the wall | ❌ | COMPLETE | ❌ | turn starts 2.075 m from wall, heading 98.8 deg | None | 0.044 | 0.869 | 0.755 | 15.3 | 44.6 | move_0_turn: stop_condition:orientation_delta; move_1_straight: stop_condition:front_clearance |
| base_room run 2 | go ahead 2 meters then go to the person | ✅ | COMPLETE | ✅ | leg 1 1.971 m, person gap 0.73 m | 0.13 | 0.728 | 0.946 | 1.006 | 11.9 | 41.3 | move_0_straight: stop_condition:distance_reached; move_1_go_to: stop_condition:object_reached |
| office | approach the wall | ✅ | COMPLETE | ❌ | wall gap 1.548 m | -0.452 | 0.457 | None | None | 4.3 | 35.0 | move_0_straight: stop_condition:front_clearance |
| office | go to the person | ✅ | COMPLETE | ✅ | gap 0.843 m, bearing -0.9 deg | 0.243 | 0.84 | None | None | 3.1 | 33.7 | move_0_go_to: stop_condition:object_reached |

## Findings (nothing in the stack was tuned)

| finding | evidence | likely cause |
|---|---|---|
| **approach the wall: the planner asks for 2.0 m; campaign criterion is 0.5 m** | mission `front_clearance 2.0` in all 3 runs | Planner default: the wall default is 2.0 m in plan_translate (3.0 before a turn). The prompt names no distance. This is the campaign's own note on M02 ("a different wall thresh is a translation finding") |
| **the wall stop overshoots the commanded distance by 0.39-0.45 m** | stop error -0.394, -0.417 (base room), -0.452 (office); car at 0.5 m/s | `front_clearance` with `debounce_ticks 3` at a 10 Hz control tick, plus the EMA on the camera front distance, plus braking. About 0.8 s × 0.5 m/s. Same in both scenes |
| **go to the potted plant: ABORTED, `go_to_object:target_lost` after ~5 s** in both runs | At the start pose YOLO sees the plant in 199/199 frames (score 0.83). The tracker keeps spawning new tracks between (2.5, -1.4) and (2.7, -1.5), and once at (3.93, -2.33), which is the wall behind the plant. mpc_corr's goal used that far point: `target_map=(+3.994,-2.366) standoff=1.42` (`plant_tracks_evidence.log`) | Depth through the sparse leaves hits the wall behind, so the plant's 3D position jumps more than 1 m. The tracks churn, the go_to goal sits on the wall, and the target counts as lost. A real thin plant can do the same with the real ZED; it is a perception and tracking finding, not a sim artefact |
| **turn left at the wall: wrong translation, and the car passed 4-6 cm from the person** | LLM answer `[{"mode":"turn","dir":"left"},{"mode":"straight","guard":"wall"}]`: order inverted, no wall guard before the turn. The `wall_turn` move turned at 2.08-2.11 m from the wall (90° reached), and the swept path came within 0.063 m and 0.044 m of the person's box. No e-stop, no contact | The model (Qwen2.5-3B, Italian prompt with Italian examples) maps the English "turn left at the wall" to turn-first. The near miss is the person 2.2 m to the left of the wall approach (campaign layout, decisions S9) inside the turn's arc. This needs a look at the turn's obstacle handling |
| go to the person: stops 0.22-0.24 m further away than the commanded 0.6 m gap, still inside the M04 criterion (1.0 ± 0.3) | gap 0.837 / 0.823 (base room), 0.843 (office); `MoveOutcome.actual` 0.756 by the stack's own track gap | The stack's gap is measured to its own person estimate; the sim's box is the skinned standing mesh. Arms down now (D26 fixed): the T-pose had given 1.49 m |
| go ahead 3 m and the mixed prompt: within criteria | 3.038 / 3.084 m; first leg 1.998 / 1.971 m | — |
| Isaac RTF 0.72-0.77 during all runs | `isaac.log` RTF lines, 1.00 before the stack connected | Raw images published synchronously over the link (decisions S19) |
