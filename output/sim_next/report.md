# sim_next: Isaac render fixes, campaign room, LLM planner end to end, office: report

**Morning summary**
- 🟢 **P1.1 DLSS**: the cameras render natively with DLAA. The 320x180 warning is gone, and the edges are sharper (`isaac_ws/output/sim_next/dlss_before_after_crop.png`).
- 🟢 **P1.2 person**: stands with arms down (a pose authored on the skeleton). **0.577 x 0.420 x 1.788 m**; the shipped T-pose was 1.872 wide. Depth stays 0.42 m: that is the asset's real depth, shoes to coat.
- 🟢 **P1.3 RTF**: GUI 0.57 → **1.00** (unthrottled capacity 1.48; headless 1.00 / 1.54). The GUI had been rendering both cameras at 60 Hz. Host settings: `isaac_ws/output/todo_for_andreas.md` §4.
- 🟢 **P1.4 noise**: lark silenced; `omni.isaac.*` deprecations ignored (Isaac's own start-up); first-frame camera warning moved to debug.
- 🟢 **P2**: four layout presets (`campaign` is the default), all ASSUMED (no measured layout exists). Potted plant 0.52 x 0.55 x 0.86 m with label `potted plant`, optional box, ground truth on `/sim/ground_truth/objects`.
- 🟢 **P3 plumbing**: `sim_isaac.sh shell`/`run` as a super client (128 topics). `enable_intelligence:=true` starts llama-server exactly as on the car. The server is warmed at component start (ready 1.5 s, primed 0.9 s), and the planner now waits out HTTP 503 while the model loads.
- 🟡 **P3 runs** (2 runs in the base room, same verdicts both times):
  - ✅ go ahead 3 m (3.04 / 3.08 m)
  - ✅ go to the person (gap 0.84 / 0.82 m)
  - ✅ 2 m then the person (leg 1.998 / 1.971 m)
  - ❌ approach the wall: the planner commands 2.0 m and the stop overshoots by 0.4 m
  - ❌ potted plant: `target_lost` (depth through the leaves)
  - ❌ **turn left at the wall: the LLM inverts the order, and the turn passed 4-6 cm from the person**
- 🟢 **P4 colliders**: every wall, door, glass, column, partition and furniture mesh of the office already collides, and /scan sees the walls (70.3 % of beams < 10 m). There was nothing to add (a rule is in place, adds 0).
- 🟢 **P4 NaN**: 5 min of driving gave **1543 NaN samples** (stalled against a wall, unbounded wheel torque). A 2 N m torque limit brought that to **0**.
- 🟡 **P4 office runs**: ❌ approach the wall (same 2.0 m and overshoot), ✅ go to the person (0.84 m). Lighting along a 10 m route: mean brightness 10-17 → 58-83 /255 with 5 fill lights. VRAM 5.6 GB / 8.
- 🔴 **Open: Isaac RTF drops to 0.72-0.77 whenever the Thor stack is connected** (1.00 before). The raw images are published synchronously over the link (decisions S19).
- Nothing pushed. isaac_ws `f1tenth`: da2c96c, b911095, 2777dd9. f1tenth_more `jazzy-campaign`: 5ef58d1, f1b9c1e, plus this report. The Thor got them from git bundles (no push); `llm` was rebuilt there with `--symlink-install`.

**Reproduce**
```bash
# linus (Isaac Sim)
scripts/sim/sim_isaac.sh isaac --net eth --scene base_room [--layout campaign|obstacle|gazebo_6x6|e2e_legacy] [--gui]
scripts/sim/sim_isaac.sh isaac --net eth --scene office
# Thor (stack + llama-server)
scripts/sim/sim_isaac.sh stack --net eth enable_intelligence:=true
# Thor (the planner needs llama-server on 127.0.0.1)
scripts/sim/sim_isaac.sh run --net eth ros2 run llm llm_planner_node "go to the person"
scripts/sim/sim_isaac.sh shell --net eth                      # interactive, whole graph visible
scripts/sim/sim_isaac.sh run --net eth python3 tools/sim/llm_runs.py --out output/sim_next/llm_runs_new   # all 6 commands
```
Decisions S1-S24: `decisions.md`. Run tables: `llm_runs.md` / `llm_runs.csv`. Isaac evidence: `~/isaac_ws/output/sim_next/`.

---

## Part 1: Isaac fixes (isaac_ws da2c96c)

### 1.1 DLSS → DLAA (S1)
- **Cause:** `SimulationApp(anti_aliasing=0)` did not stick; `/rtx/post/aa/op` read back **3** (DLSS, Performance) at the first frame. The Kit 107 real-time renderer accepts only 3 (DLSS) and 4 (DLAA). FXAA (2) produced the same upscaled frame as DLSS (mean abs diff 0.28/255).
- **Fix** (`sim/app.py`): `aa/op = 4` (DLAA, render scale 1.0), set in the app config and forced again after the scene is built.
- **Measured** on the same frame (static car, t = 8 s):

  | | before (DLSS) | after (DLAA) |
  |---|---|---|
  | Kit warning `DLSS increasing input dimensions ... (320, 180)` | present | gone |
  | mean gradient magnitude | 2.436 | 2.587 (+6 %) |

- **Evidence:** `left_before_dlss.png`, `left_after_dlaa.png`, and `dlss_before_after_crop.png` (crop x60-260 y0-120, 4x nearest; seams and texture are visible only after).

### 1.2 Person (S2-S4)
- **Why the old footprint was 1.63 x 1.94 m:** the world AABB of the T-pose (2.05 m arm span) rotated by the person's yaw. All `/Isaac/People/Characters` ship T-posed. The animation clips use another skeleton (0 joints in common), so they would need retargeting.
- **Fix:** `sim/person_pose.py` authors a one-frame UsdSkel pose on the character's own skeleton: the rest pose with the upper arms hanging 4° off vertical. The footprint is measured on the skinned points.

| | width | depth | height |
|---|---|---|---|
| as shipped (T-pose) | 1.872 | 0.420 | 1.788 |
| **standing (4°)** | **0.577** | 0.420 | 1.788 |
| 0° / 8° | 0.514 / 0.651 | 0.420 | 1.788 |

- Label `person` and the capsule collider (r 0.175) are unchanged.
- Images: `person_before_tpose.png`, `person_after_standing.png`.

### 1.3 RTF (S5)
base_room, 60 s sim, 900 camera frames in every run:

| mode | before | after |
|---|---|---|
| headless, paced | 1.000 | 1.000 |
| `--gui` | **0.569** | **1.000** |
| `--gui --viewport off` | – | 1.000 |
| headless unthrottled (capacity) | – | 1.536 |
| `--gui` unthrottled (capacity) | – | 1.478 |

- **Cause:** GUI mode called `world.step(render=True)` on every physics step. A render draws every render product, so both cameras rendered at 60 Hz instead of 15, and GUI mode had no real-time pacing.
- **Fix:**
  - The GUI renders with the cameras (15 Hz).
  - Pacing applies in GUI mode too.
  - `--viewport-res` (default 640x360) and `--viewport off` were added.
  - Sensor rates and resolutions are unchanged.
- **CPU governor:** `intel_pstate` reports "powersave", but the cores ran at 4.56 GHz and `powerprofilesctl` already says performance. The command for `performance` is in the todo (§4).

### 1.4 Log noise (S6)
| message | decision |
|---|---|
| `No module named 'lark'` | silenced. It is a DEBUG traceback from the bridge's code generator, which message classes do not need at run time |
| `omni.isaac.*` deprecations (83 lines) | ignored. They come from the compatibility extensions enabled by Isaac's experience file at start-up, before our code runs |
| `camera rgb not ready` | debug level for the first 5 renders (the pipeline is a few frames deep); a warning after that |

## Part 2: the campaign room (isaac_ws b911095, S8-S12)
- **Layouts** (`config/scenes/base_room.yaml`, `--layout NAME` or `--set scene.layout=NAME`; `--set` overrides win). Coordinates are in the room frame: interior 0..6 m, corner at the origin.

| preset | car start | person | plant | box |
|---|---|---|---|---|
| **campaign** (default) | (1.2, 2.5, 0°): wall 4.5 m ahead of the lidar (M02 R009/R011) | (5.4, 4.7): 4.2 m ahead, 2.2 m left (M04 2026-09-24) | (3.8, 1.2): 2.6 m ahead, 1.3 m right (M03 mirrored) | off |
| obstacle | as campaign | as campaign | as campaign | 1.0 m across, 0.5 m deep, 0.8 m high, near face 3.0 m ahead |
| gazebo_6x6 | (0.5, 3.0) | (5.0, 3.0) | (3.0, 3.0) | off |
| e2e_legacy | (1.0, 1.5) | (3.0, 3.0) | off | off |

- **Why ASSUMED:** no document gives the real room's measurements (searched docs/, output/, and the campaign data on campaign-final). The numbers are relative positions from the campaign logs. The plant is mirrored to the right so that the left turn has room.
- **Plant:**
  - Asset: `/Isaac/Environments/Office/Props/SM_Plant02.usd` x0.55 → 0.516 x 0.554 x 0.855 m. It is the only NVIDIA plant that reaches 0.8-1.2 m tall and 0.4-0.6 m wide at a uniform scale.
  - Label: `potted plant` (the COCO name YOLO and the go_to enum use).
  - Collider: static triangle mesh; the lidar sees the pot and the leaves.
- **Ground truth:** `/sim/ground_truth/objects` (vision_msgs/Detection3DArray, map_sim, 1 Hz, transient-local), one entry each for person, plant, obstacle and wall_0..3, with class, id, box pose and size. Checked with `ros2 topic echo`.
- Evidence: `layout_obstacle_left.png`. `tools/e2e_test.sh` now pins `--layout e2e_legacy`; I did not re-run it.

## Part 3: LLM planner in the sim (f1tenth_more 5ef58d1, f1b9c1e; S13-S19)

### Plumbing
- **`sim_isaac.sh shell [--net eth]` / `run [--net eth] CMD...`**
  - Environment: the stack's own `ros_env` plus `ROS_SUPER_CLIENT=TRUE`.
  - Daemon: the ros2 daemon is restarted first. An old plain-client daemon answered `ros2 topic list` with 1 topic; after the restart, 128.
  - Exit code: `run` returns the command's.
  - Where to use it: the planner itself must run on the Thor, because llama-server listens on 127.0.0.1.
- **`enable_intelligence:=true`:** passed through, and an error if llama-server is missing; it is no longer silently turned off. On the Thor the intelligence component runs the car's exact command line: `/scratch/fabiocar/llama.cpp/build/bin/llama-server -m ~/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf --port 8083 -ngl 999`.
- **Cold start (S15):**
  - **Code state:** `LLAMA_TIMEOUT` was already 90 s, and the planner warmed the server with an 8-token request but retried only refused connections.
  - **Measured on the Thor** with a second server:

    | stage | result |
    |---|---|
    | while the model loads (~1 s) | `/health` and `/completion` both answer **503**, so the planner failed at once |
    | first planner prompt | 2.9 s (0.9 s prefill of 2443 tokens) |
    | next prompt | 2.0 s (prefix cached) |

  - **58 s first completion:** not reproduced, also not with an empty CUDA JIT cache. A cold page cache right after boot needs sudo to test (todo).
  - **Fix**, valid on the car:
    - `llm_warmup`, started by `llm.launch.py`, waits for `/health` and then primes the planner prompt.
    - The planner's own wait now also retries on 503.
  - **Result in all runs:** "ready after 1.5-2.0 s, warm-up 0.87-0.90 s", and the planner found the server ready in 0.2 s.
  - Tests: `test_llm_warmup.py` (3) and a 503 retry test; 15/15 pass. The 14 failures in `test_planner_path.py` were already there (same count before my change, listed in `jazzy_port/test_failing_ids.txt`).
- **Between runs:** `/reset_all` (reset_manager: stop, `/sim/reset`, EKF local, SLAM restart, EKF global) and 5 s settle. 14/14 resets succeeded, 4.1-7.6 s.

### Results (details and the full table: `llm_runs.md`)
| command | run 1 | run 2 | office |
|---|---|---|---|
| go ahead 3 meters | ✅ 3.038 m, lat 0.03 | ✅ 3.084 m | – |
| approach the wall | ❌ commanded 2.0 m, stopped 1.606 m | ❌ 1.583 m | ❌ 1.548 m |
| go to the person | ✅ gap 0.837 m | ✅ 0.823 m | ✅ 0.843 m |
| go to the potted plant | ❌ ABORTED target_lost at gap 1.12 m | ❌ same, 1.19 m | – |
| turn left at the wall | ❌ translated turn-first; person clearance **0.063 m** | ❌ same; **0.044 m** | – |
| go ahead 2 meters then go to the person | ✅ leg 1.998 m, gap 0.753 m | ✅ 1.971 m, 0.73 m | – |

- **Overall:** translation OK in 5/6 commands; success 3/6 in each base-room run. The stack itself was not changed (S18).
- **Durations:** RUNNING lasted 3-18 s. Command to end took 34-48 s, of which ~10 s is the planner process and 15 s the mission countdown.
- **Causes**, in short:
  - **Wall:** planner default 2.0 m (the campaign criterion is 0.5); the overshoot of ~0.4 m is the 3-tick debounce plus the EMA at 0.5 m/s.
  - **Plant:** YOLO sees it every frame (0.83). Depth through the leaves puts tracks on the wall behind it, (3.93, -2.33), and the go_to goal follows them.
  - **Turn:** Qwen answers `[turn left, straight to wall]` to the English command. The turn's arc passes the person's box at a few cm.

## Part 4: office (isaac_ws 2777dd9, S19-S24)
1. **Colliders before:** whole-building audit (`office_colliders_all.json`):

   | category | meshes with a collider |
   |---|---|
   | wall | 236/236 |
   | door | 111/111 |
   | glass | 5/5 |
   | column | 48/48 |
   | partition | 8/8 |
   | furniture | 238/238 |

   All of them are instanced triangle meshes from the asset. The 140 non-instanced meshes without a collider were:
   - 134 floors/carpets: they have colliders since D20.
   - 5 skinned elevator doors with a degenerate bound: left alone, the frame around them collides.
   - The GroundPlane.

   The scan at the start shows the walls as lines (`office_scan_before.png`: 760/1081 beams < 10 m, 70.3 %).

   Scene-query raycasts are single-sided (172 beams cross back faces of `SM_WallsElevator`, `office_passthrough_before.png`). The PhysX lidar still sees 120 of those beams as back-face hits.
2. **Collision rule:**
   - **The rule:** `environment.vertical_colliders`, enabled for the office.
     - Shapes: boundingCube for wall, partition and glass; convexHull for column, door and furniture.
     - Skipped: meshes over 40 m, under 0.3 m tall, or degenerate.
     - Result: it adds 0 today.
   - **NaN check:** 5 min of circles, `tools/drive_nan_check.py`.

     | | before | after (torque limit) |
     |---|---|---|
     | NaN samples (/odom, IMU, ground truth) | **1543** each | **0** of 14994 |
     | what happened | the car pinned itself against a structure at x ≈ -8; at t = 270 s z jumped 0.2 m and PhysX reported invalid transforms on every car body | still pinned in the same place, no NaN |

   - **Fix:** wheel drive max torque 2.0 N m (3.6x the traction limit; the velocity drives had no force limit).
3. **After** (`office_scan_after.png`): 760/1081 beams < 10 m (70.3 %); 0 beams differ by more than 5 cm from before.
4. **Lighting**, 10 m route (-12,-3) → (-12,-10.6) → (-9,-11), 7 waypoints (`office_route_before.png` / `office_route_after.png`):

   | | mean brightness /255 | dark pixels (< 20) |
   |---|---|---|
   | before | 55, 39, 16, 10, 17, 14, 14 | 48-92 % |
   | after (5 fill lights) | 83, 81, 62, 58, 76, 68, 62 | 23-48 % |

   Light positions: (-12,-5.5), (-12,-8), (-11.5,-10.5), (-9.5,-11.2), (-7.5,-11.2), all at 2.4 m.
5. **Stack in the office** (person 1.8 m ahead, 0.8 m left):
   - approach the wall ❌: stopped 1.548 m from the wall, against a commanded 2.0 m. The person was passed at 0.46 m.
   - go to the person ✅: gap 0.843 m.
   - VRAM on linus: 5.6 GB of 8 (YOLO runs on the Thor). RTF: mean 0.75, min 0.72.
   - New ground truth for wall distances here: `/sim/ground_truth/front_range` (bumper raycast).

## Open issues
1. **RTF with the remote stack: 0.72-0.77** (S19). The likely cost is the synchronous raw-image publishing from the physics loop (~55 MB/s). Next step: a worker thread or compressed images, then measure.
2. **Turn left at the wall:**
   - The LLM inverts the order (the prompt examples are Italian).
   - Separately, the `wall_turn` arc came within 4-6 cm of a person standing 2.2 m to the side, and nothing stopped it. Worth checking the turn's obstacle handling on the real car before the campaign repeats M05 with people around.
3. **Plant go_to:** depth through foliage gives jumping 3D tracks and `target_lost`. Real thin plants can do the same.
4. **Wall stop overshoot ~0.4 m at 0.5 m/s** (debounce + EMA), in both scenes.
5. Parked creep (D25) still there: ~5 cm in 30 s.
6. The 58 s cold start is not reproduced. It needs a test right after a Thor reboot (or `drop_caches` with sudo).
7. `tools/e2e_test.sh` was not re-run after these changes (it now pins `--layout e2e_legacy`).

## Todos for Andreas
- CPU governor (optional, sudo): `sudo cpupower frequency-set -g performance` (`isaac_ws/output/todo_for_andreas.md` §4).
- After the next Thor reboot: `scripts/sim/sim_isaac.sh stack --net eth enable_intelligence:=true`, then read the `llm_warmup` line in `~/.ros/log/component_supervisor/llm_llm.launch.py.log`, which is the true cold-start number.
- Decide on the planner's wall default (2.0 m) against M02's 0.5 m, and on English examples for "turn left at the wall". Both are planner and prompt changes, so I left them alone.
- The Thor's working copy is at f1b9c1e from a bundle (nothing pushed). Push both branches when happy.
