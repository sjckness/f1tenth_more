# office_hil_campaign — 3 commands × 3 repetitions, Orin in the loop

Three natural-language commands through the **full** pipeline — LLM → mission → behaviour tree
→ `mpc_corr` → `/drive` → Isaac — in the HIL configuration: Isaac renders on the Windows
workstation, the **unchanged car stack** runs in the Orin container. 9 runs, `corridor_mpc`
only, recorded and judged in the shape of [first_test_campaign/](../first_test_campaign/).

Every command to type, in order, is in [RUNBOOK.md](RUNBOOK.md). The protocol is the one in
[TEST_CAMPAIGN.md](../src/f1tenth_logger/TEST_CAMPAIGN.md); this
file covers only what is different here. The geometry of all three missions was verified
before any of them was run: [output/office_missions/](../output/office_missions/).

| | command | start | Isaac preset |
|---|---|---|---|
| M01 | `go straight, enter the door on the left and then approach the chair` | (9.33, −15.87) yaw 90° | `campaign_ma` |
| M02 | `go straight at the wall, turn left and then approach the wall` | (16.83, −9.87) yaw 180° | `campaign_mb` |
| M03 | `at the wall turn left, then go to the person, then enter the door on the right` | (16.83, −9.87) yaw 180° | `campaign_md` |

Run order is **M02 → M01 → M03**: M02 needs no perception at all, so it is the smoke case.

## Three Isaac sessions, not one

The scene is fixed at Isaac launch and the three missions need three scenes. M01 starts
elsewhere; M03 adds a person at (11.63, −13.30) which stands **on M02's second leg** and leaves
0.45 m to its left and 0.80 m to its right, against the 1.18 m the car needs to pass a person.
So M02 cannot be driven in M03's scene, and the campaign restarts Isaac — and therefore the
stack — between missions. `hil_campaign.sh all` stops at each boundary and prints what to type.

## A session

Three terminals. The recorder is started **once** and survives the stack restarts: it owns the
campaign's repetition counter, and the test folder must already be open when the planner
publishes `/test/plan_result`.

```bash
# 1. Orin, stack terminal  (re-run this after every scene change)
scripts/sim/hil_orin stack enable_intelligence:=true enable_mission_logger:=true \
      planner_controller_mode:=corridor_mpc

# 2. Orin, recorder terminal  (ONCE per session)
scripts/sim/hil_orin shell
ros2 launch f1tenth_logger test_campaign_logger.launch.py \
    root:=/home/fabiocar/dev_ws/f1tenth_hil \
    config:=$(ros2 pkg prefix f1tenth_logger)/share/f1tenth_logger/config/office_hil_campaign_logger.yaml

# 3. Orin, driver terminal
scripts/sim/campaign/hil_campaign.sh preflight
scripts/sim/campaign/hil_campaign.sh translate        # no motion; retires the biggest LLM risk
scripts/sim/campaign/hil_campaign.sh all --reps 3
```

Isaac, on the workstation, once per mission:

```bat
scripts\sim\win\hil_sim_link.bat --layout campaign_mb
```

**Prerequisite, one-time:** the Orin container needs the repo and the bag archive bind-mounted
(mounts are fixed at container creation) — see
[output/hil/run_windows.md](../output/hil/run_windows.md). `hil_campaign.sh preflight` checks
it and says exactly what is missing.

## The judging gate

`first_test_campaign/analysis/report.txt` reads **`evaluated by hand: 0`**: 75 runs of data and
not one verdict, because `success` is a manual column and the judging was left until after the
sessions. Nothing computed can stand in for it — `test_campaign_analyze` counts no other
verdict, so every k/N in that campaign is `0/0`.

So here the driver judges **per mission**: after each mission's 3 runs it exports and
**refuses to start the next mission** while any of those rows has an empty `success`. Three
five-minute stops instead of one that never happens. `--no-gate` overrides it, and then nobody
judges them.

The evidence is prepared for you, per run, so judging is reading not re-deriving:

| column | read it from |
|---|---|
| `success` | `aux/<id>/<ts>/verdict_sheet.txt` — the criterion, the final ground-truth pose in **map** coordinates against the target, and the `/mission/status` trail |
| `transl_ok` | `plan.json` / `llm_calls.jsonl` in the run folder, against the `expected_sequence` comment in [prompts.yaml](prompts.yaml) |
| `notes` | `corridor_plot.png`, and the bag's `/camera/debug_image/compressed` for "did YOLO actually see the chair" |

## The bags

Nothing extra runs: `mission_logger_node` is part of the normal bringup and under `sim:=isaac`
writes to `~/f1tenth_sim_archive/complete/<UTC>_mission-<plan_id>/` on the Orin — bag, manifest,
the resolved `stack_params.yaml`, a parquet extract and up to 10 snapshots.

The driver ties each bag to its run in [bag_index.csv](bag_index.csv) and in a `bag_ref.json`
inside the test folder. The manifest's `mission_id` is a sha1 of the **intent**, so it is
identical across the 3 repetitions of one command: time separates them, not the id.

Two settings that matter, both handled by the driver:

- `mission_countdown_sec` is set to **15 s** after every bringup. It has been 0.0 since
  2026-10-21 and the recorder needs 2.4–14 s of discovery before its first message, so at 0 a
  short mission's bag starts mid-drive or empty. It is read live at each start and lost on
  restart.
- the recording profile stays **`default`**, not `campaign`. The default allowlist already
  carries `/camera/detections`, `/camera/debug_image/compressed` at 5 Hz,
  `/mission/move_outcome`, `/perception/*` and `/slam/map` — the evidence this campaign needs.
  `campaign` adds 650–800 MB/min of scene mask and raw image/depth that nothing here reads.

Known gap worth writing in the notes: `/costmap/freespace_doors/debug` is **not** in the
allowlist, so "why was there no strong door" has to be read from `/mission/move_outcome`
(`door_not_found` / `no_strong_door`) plus `/slam/map`.

## What to expect to go wrong

In the order it is likely to, with the check that settles each one. The full list is in the
plan; these are the three that decide whether a mission produces data at all.

1. **YOLO never sees the chair or the person.** Headless `RaytracedLighting` has no indirect
   diffuse, and `office_missions.py` reports M01's waypoints 6.2–6.4 m from the nearest
   existing fill light. The `campaign_ma` / `campaign_md` presets add lights for exactly this,
   and they are a starting point, not a measurement. Before burning three runs: with the scene
   up and **no mission**, check `/camera/detections` and `/costmap/semantic_tracks` from the
   start pose. If no `chair` track appears within the 5 s acquire timeout, raise those lights
   (30000 → 60000) and retry. This is a per-mission go/no-go.
2. **An ordinal in the command.** "the **first** door on the left" sends the request to
   `unsupported`, and a non-empty `unsupported` makes the planner ask an interactive `[s/N]`
   that nothing can answer. `hil_campaign.sh translate` catches it in a minute, and the texts
   in `prompts.yaml` already avoid it — `side: left|right` means the nearest on that side,
   which in all three scenes is the first one.
3. **`enter_door` never gets a STRONG door.** It needs two sightings from poses ≥ 1.0 m apart
   within a 5 s acquire timeout. M01 has only ~2.6 m of run-up and M03 arrives at a standstill
   3.87 m from the opening. The mitigation is a parameter, not a redesign — and if one is
   changed, it goes in the notes, because it changes what the campaign measured.
