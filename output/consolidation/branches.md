# Branch inventory for the consolidation (2026-10-22)

Every branch on linus, on GitHub (`origin`) and on the Orin (`orin`,
`fabiocar@jetson-orin:dev_ws/f1tenth_more`), plus the submodule forks that
have their own branches.

**How it was made**
- `git fetch origin` and `git fetch orin` on 2026-10-22, 16:1x–16:3x. No
  prune, so nothing was deleted.
- The Orin was **reachable**. Its refs are current as of 16:31.
- Ahead/behind is `git rev-list --left-right --count campaign-final...<tip>`,
  where campaign-final is **fa5471f**. "Ahead" = commits the branch has that
  campaign-final does not.
- "In by patch" means `git cherry campaign-final <tip>` shows every commit as
  already present under another id.

Where each branch lives:
- **L** = local branch on linus;
- **O** = GitHub;
- **R** = the Orin.

Verdicts:
- **ALREADY IN**: everything is already in campaign-final, by ancestry or by
  patch.
- **MERGE**: merged into `main-consolidation`.
- **KEEP OFF**: not merged; the reason is given.
- **OBSOLETE**: superseded, or a branch made only to save work. The archive
  tag keeps it (report.md §5).

## 1. Car line (Humble)

| Branch | Where (tip) | Last commit | Ahead / behind CF | Content | Verdict |
|---|---|---|---|---|---|
| `campaign-final` | L O fa5471f; **R 1ee82c86** | 10-22 | 0 / 0 (R: 0 / 68) | The test-campaign base: doors, multi-wall tracker, offset-free speed law, front-wall lock, wall confirmation level, cpu_cleanup, reverse MPC (2-QP), obstacles v2, the app. On the Orin the branch name still points at 1ee82c86 (10-21), an ancestor of fa5471f. | **BASE** (frozen, untouched) |
| `main` | L O R fa5471f | 10-22 | 0 / 0 | Already fast-forwarded to campaign-final on 10-22. | Target of the final fast-forward |
| `main-consolidation` | L 9bed256+ | 10-22 | 26+ / 0 | This work | **NEW** |
| `campaign-final-yolo-gpu` | **L only** 5792e2b | 10-22 | 4 / 0 | YOLO CPU cost: per-stage timing, thread limits, CUDA blocking sync, 15 Hz frame drop, TensorRT engine backend with `.pt` fallback, GPU pre/post pipeline (default on), smoke test | **MERGE** (static checks pass; pure-Python tests 11 pass / 2 skip) |
| `two-llm-arch` | L O R 175da11 | 10-22 | 11 / 0 | LLM2: a second llama-server + `llm2_mpc_param_node` proposing MPC weights, mpc_corr whitelist consumer, gateway and PWA pause/resume toggle. All off by default. The Orin's app gateway runs from this worktree. | **MERGE** |
| `comms-base` | L O 02b245e | 10-21 | 7 / 51 | New package `f1tenth_comms`: link_manager (heartbeat, OK/DEGRADED/LOST, reconnect), Wi-Fi coverage layer, base hotspot scripts, `/comms/link_status` in the gateway payload. `enable_comms` false. | **MERGE**. Not on your list, but off by default and clean: 4 trivial conflicts. Revert with `git revert -m 1 e7c257c` if unwanted. |
| `pwa-control` | L O R 87e6f69 | 10-22 | 0 / 21 | **The latest app branch**: gateway, f1tenth_ctl and PWA "lamacchinina", including reverse, QR pairing and the CPU/GPU view. Contains `telegram-control`. | **ALREADY IN** (merge 4370897) |
| `telegram-control` | L O b131b9b | 10-20 | 0 / 117 | Telegram bot: planner in two steps, voice | **ALREADY IN** (through pwa-control) |
| `mpc_corr_v2_reverse` | O R 4f53ab8; L **a14ec2f (stale, 9 behind O)** | 10-20 | 0 / 123 | **The 2-QP reverse manoeuvre** (forward + rear solver), object memory, BT integration | **ALREADY IN** (merge 247264e) |
| `night-reverse` | **L only** 3b8b0b8 | 10-22 | 0 / 63 | The reverse counts the distance backed up in odom (A1 fix) | **ALREADY IN** (ec288e7) |
| `night-walls` | **L only** 120d0f4 | 10-22 | 0 / 61 | One wall confirmation level (`wall_confirm_*`); MPC ramp on the locked front wall | **ALREADY IN** |
| `night-reports` | **L only** 2ed5216 | 10-22 | 0 / 61 | Run-report stills and video (mission_overlay) | **ALREADY IN** (7bf99f9) |
| `door-fixes` | L O R 486a157 | 10-20 | 0 / 108 | Doors addendum: hosts, perpendicular gap tests, lifecycle, role `unknown` | **ALREADY IN** (2ff621e) |
| `door-detection` | L O c873247 | 10-19 | 0 / 136 | Door detection v1 + replay tool | **ALREADY IN** |
| `obstacles-v2` | L O d0028f2 | 10-19 | 0 / 132 | One semantic layer, unknown objects, one obstacle path | **ALREADY IN** (e05816b) |
| `cf-next` | L O 3feae47 | 10-20 | 0 / 74 | Semantic merge replay | **ALREADY IN** |
| `cpu-cleanup` | L O e41512a | 10-20 | 1 / 162 | Replay v2 notes | **ALREADY IN** (by patch: cd180d8) |
| `tmp/door-fixes-{core,node,replay,replay2,tracker,unknown}` | **L only** 9b73a63, dd5c565, 35b4d8e, d8ece07, ea4beb4, dee1687 | 10-19/20 | 1-11 / 136 | Work branches for door-fixes | **OBSOLETE** (all in by patch through door-fixes) |
| `campaign-final-doors` | **R only** 6c693ac | 10-20 | 0 / 125 | The 10-20 car HEAD: doors + stop ramp | **ALREADY IN** |
| `stack-campaign-final-2026-10-22` | **R only** fa5471f | 10-22 | 0 / 0 | **The Orin's HEAD now.** Same commit as campaign-final, but the checkout has 32 staged files at 1ee82c86 (conflicts.md §2.2) | **ALREADY IN** (commit). The running state is in no commit. |
| `orin-wip-2026-10-22` | **R only** 02cd0f2 | 10-22 | 1 / 68 | "WIP snapshot: campaign-final uncommitted work before 2026-10-22 Orin pull/build", on 1ee82c86. **Additions only** (830 files): M02 runs P002-R012..R022 of **10-20** (136 files), the misspelled `first_test_campaing/` (M02 R001-R008 of 10-13/14, 442), 245 `output/` files (cpu tests, replays), wall-v2 variant experiments (`test_conditional_birth.py`, `tools/wall_variant_videos.py`, `tools/wall_variants/*.yaml`), `pr.json`. **Not on GitHub.** | **OBSOLETE** as a branch (made to save work). Not merged: data and experiments, and campaign data belongs to campaign-final, which is frozen. **Archive it.** It is the only copy of the 10-20 M02 runs outside the Orin. |
| `docs/branch-audit` | **R only** 953c897 | 10-20 | 1 / 123 | `output/branches/migration_plan.md`, a branch audit by the Orin session of 10-20. **Not on GitHub.** | **OBSOLETE** (superseded by the 10-20 inventory and this file); archive |
| `orin-snapshot-20261013` | **R only** 8c9e29b | 10-13 | 31 / 263 | Orin working tree + `orin_check_report.md` | **OBSOLETE**; already tag `archive/orin-snapshot-20261013` (on GitHub) |
| `orin-verify-campaign-dirty` | **R only** 3ae2108 | 10-13 | 4 / 255 | P1-P4 verification state | **OBSOLETE**; already tag `archive/orin-verify-campaign-dirty` |
| `refactor/mpc-corridor-split` | **R only** a3ea50d (Orin worktree `~/dev_ws/f1tenth_mpc_refactor`) | 10-09 | 4 / 263 | Corridor split out of MPC_corr, `andre_mpc_node` dropped | **KEEP OFF**: another MPC line, and it predates every reverse and wall change. Already tag `archive/mpc-corridor-split`. |
| `test-campaign` | **R only** 6ebbc8a | 10-12 | 25 / 263 | M02 Part A fixes on the wrong base | **OBSOLETE**; in `archive/test-campaign` |
| `test-logging` | **R only** 573f313 | 09-30 | 0 / 263 | M01 data, the base of all campaign work | **ALREADY IN** |
| `llm`, `scene-graph`, `sick` | **R only** e47e646 | 09-30 | 0 / 264 | Old feature branches = the old main | **OBSOLETE** (ALREADY IN) |

**Orin-only content that is not on GitHub** (now also in linus's
remote-tracking refs, **not pushed**):

| Item | What it holds |
|---|---|
| `orin-wip-2026-10-22` 02cd0f2 | See above |
| `docs/branch-audit` 953c897 | See above |
| tag `orin-before-pull-2026-10-22` | Annotated, by "audit", points at 1ee82c86. The commit itself is on GitHub; the tag is not. |
| Orin stash `stash@{0}` | "results.csv + settings.json from mpc_corr_v2_reverse (claude, pre-switch)". New since the 10-20 archive, which tagged the 5 older stashes as `archive/orin-stash-{0..4}-20261020`. Not archived. |
| The Orin's staged 32-file state | No commit holds it (conflicts.md §2.2) |

The Orin was on, so no Orin-only branch is missing from this list.

## 2. Jazzy line (Ubuntu 24.04): not merged, listed separately

None of these goes into main. They are a separate port with its own
submodule pins: zed_ros2_wrapper cce25d3/86171bc, ackermann_mux b3c0b08,
vesc 2848a9f.

| Branch | Where (tip) | Last commit | Ahead / behind CF | Content | Verdict |
|---|---|---|---|---|---|
| `jazzy` | L **eb78d6b**, O **a596962**, R af1f167 | 10-21 | 161-163 / 263 | The old Jazzy port: sim, Thor and Orin Jazzy docker. **L and O have diverged.** L has 2 commits O lacks: the inventories, sim_net.sh. O has 4 by James (llm@thor) L lacks: LLM output and reverse parsing, and **`mpc_corr: added negative speeds if reverse command` (a596962), a different reverse MPC**. | **KEEP OFF** (Jazzy; a596962 is an "other MPC") |
| `jazzy-campaign` | L O 60f9af7 | 10-19 | 39 / 236 | The Jazzy re-port of campaign-final 10-14. 60f9af7's M02 data is in CF as f2d36a2. | **KEEP OFF** (Jazzy) |
| `jazzy-next` | L O 2893d41 | 10-19 | 40 / 144 | jazzy-campaign ⊕ campaign-final 868ec8a + build fixes | **KEEP OFF** (Jazzy) |
| `jazzy-rust` | L O aa721fd | 10-18 | 46 / 151 | Rust nodes (front-proximity guard), zenoh, Isaac parity | **KEEP OFF** (Jazzy) |
| `orin/jazzy` | R af1f167 (Orin worktree `~/dev_ws/f1tenth_jazzy`) | 10-12 | 118 / 264 | ⊂ origin/jazzy | **OBSOLETE** (contained) |

## 3. Submodules with their own branches

Every branch of the car line pins the same submodules:
- vesc **0e74a60**;
- ackermann_mux 42b7cd7;
- teleop_tools 4337558;
- transport_drivers d3f510c;
- zed_ros2_wrapper b5844a8;
- zed-ros2-interfaces e970087.

None of the three merges changes a pin, so main-consolidation has the same.

Only the two forks under sjckness have branches of their own.

**vesc** (`sjckness/vesc`, fetched 10-22):

| Branch | Tip | Last commit | Content | Verdict |
|---|---|---|---|---|
| `campaign-final` (= `scene-graph`) | 0e74a60 | 10-20 | `vesc_to_odom_backup`: dead deadbands dropped, reverse sign pinned | **PINNED** by main |
| `jazzy` | 2848a9f | 09-03 | Parent of 0e74a60 | ALREADY IN |
| `odom-test` | a33a30b | 07-18 | "Fix erpm sign convention" | ALREADY IN (ancestor of 0e74a60) |
| `humble-jetson` | a84869c | 07-10 | 3 commits: an older `vesc_tuning` package + calibration gains | **OBSOLETE**: superseded by f0cd7a9 "vesc_tuning: new package" on the pinned line. a84869c's gains may be worth a look. |
| `vesc-tuner-test` | 2324c85 | 06-15 | ⊂ humble-jetson | **OBSOLETE** |
| `main` | e3f3084 | 2023 | Upstream | n/a |

**ackermann_mux** (`sjckness/ackermann_mux`): `humble-jetson` 42b7cd7 is
pinned by main. `jazzy` and `foxy-devel` are b3c0b08 (upstream). No extra
work.

## 4. `git log --oneline --graph`, car line

Command: `git log --oneline --graph --simplify-by-decoration --decorate=short --date-order` on every car-line tip in §1. backup/night tags are filtered out, and nothing below 573f313 is shown.

```
* 9bed2561 (HEAD -> main-consolidation) main-consolidation: LLM planner off by default (enable_intelligence false)
*   e7c257cc Merge comms-base: f1tenth_comms (link_manager, Wi-Fi coverage), off by default
|\  
* \   514fa56d Merge campaign-final-yolo-gpu: YOLO CPU-cost work and GPU pipeline
|\ \  
| * | 5792e2ba (campaign-final-yolo-gpu) yolo_detector_node: GPU pipeline (gpu_pipeline, default on) + smoke test
| | | * 02cd0f20 (orin/orin-wip-2026-10-22) WIP snapshot: campaign-final uncommitted work before 2026-10-22 Orin pull/build
* | | | 175da113 (orin/two-llm-arch, origin/two-llm-arch, two-llm-arch) f1tenth_gateway: review fixes - LLM2 status parse cannot stop the ROS executor
|/ / /  
* | | fa5471f3 (orin/stack-campaign-final-2026-10-22, orin/main, origin/main, origin/campaign-final, origin/HEAD, main, campaign-final) output/night_2026-10-21: the night
* | |   4370897e Merge pwa-control: the F1Tenth Lab Control app (gateway, f1tenth_ctl, PWA) with reverse, lamacchinina, CPU/GPU view
|\ \ \  
| * | | 87e6f69b (orin/pwa-control, origin/pwa-control, pwa-control) apps/pwa: review fixes - CPU view colours, chip names, 320 px tiles and tab
* | | |   ec288e78 Merge night-reverse: the reverse manoeuvre counts the distance backed up in odom
|\ \ \ \  
* \ \ \ \   7bf99f90 Merge night-reports: run-report stills and video with the scene mask and the LiDAR
|\ \ \ \ \  
* | | | | | 120d0f4d (night-walls) output/front_wall_lock: review fixes - measured confirmation latency, replay wording
| * | | | | 2ed52169 (night-reports) mission_overlay: review fixes - an unstamped message does not end the search
|/ / / / /  
| * / / / 3b8b0b83 (night-reverse) output/mpc_v2/STATUS.md: review fixes - first reverses, end-tick distances, bag /tf
|/ / / /  
| * | |   23e11e07 Merge campaign-final (6002cb0, night-start/campaign-final) into pwa-control
| |\ \ \  
| |/ / /  
|/| | |   
| * | |   876fe121 Merge origin/pwa-control (the Orin's 7 commits) into pwa-control
| |\ \ \  
* | | | | 6002cb08 front_clearance stop: lock onto the wall the move approaches (wall_lock_enabled)
| |_|_|/  
|/| | |   
* | | | 1ee82c86 (tag: orin-before-pull-2026-10-22, orin/campaign-final) output/speed_gap: offset-free speed law -- design, identification, matrix, risks
| | | * 02b245e6 (origin/comms-base, comms-base) comms: linus base housekeeping, 2.4 GHz retry, the USB adapter's drops
| | |/  
| |/|   
| | * adb91439 f1tenth_ctl: give /mission/status discovery the time it needs (3 s -> 15 s)
| * | 9c5371d4 PWA: a permanent MOCK badge when the gateway runs the pretend car
| |/  
| * a45d87e6 Merge telegram-control (b131b9b) into the app branch
|/| 
* | 3feae478 (origin/cf-next, cf-next) tools/semantic_merge_replay.py + output/semantic_merge_replay.md: the merged semantic layer on 71 local bags
* |   e05816b1 Merge obstacles-v2 (d0028f2) into campaign-final: one semantic layer, both lifecycles
|\ \  
* \ \   2ff621ef Merge door-fixes (486a157) into campaign-final
|\ \ \  
* \ \ \   247264e6 Merge mpc_corr_v2_reverse (4f53ab8) into campaign-final
|\ \ \ \  
| | | | | * 953c8970 (orin/docs/branch-audit) output/branches: branch audit and migration plan (2026-10-20)
| |_|_|_|/  
|/| | | |   
| * | | | 4f53ab8f (orin/mpc_corr_v2_reverse, origin/mpc_corr_v2_reverse) f1tenth_ctl: two-step plan/start (+ planner plan-only) from telegram-control
| | | | * b131b9bf (origin/telegram-control, telegram-control) telegram: planner commands in two steps (plan, then start) and voice commands
| |_|_|/  
|/| | |   
| | * | 486a1577 (orin/door-fixes, origin/door-fixes, door-fixes) output/perception/doors.md: finish the addendum report; doors_phase1 part 2
| |/ /  
|/| |   
| | | * e41512a1 (origin/cpu-cleanup, cpu-cleanup) output/cpu_cleanup: replay v2 of corridor10m_20261016T144537
| * | | a14ec2f5 (mpc_corr_v2_reverse) Mission start countdown only while the mission logger records
| | | | * dd5c565b (tmp/door-fixes-node) doors: the node, markers, params and launch on the addendum's core -- confirmed-only /perception/doors, guarded cycle
* | | | | 6c693ac2 (orin/campaign-final-doors) stop ramp: arrive at a stopping point instead of crossing it
* | | | |   904d86ed merge door-detection step 3: frame jambs, square-on, panes
|\ \ \ \ \  
| | |_|_|/  
| |/| | |   
| * | | | c8732474 (origin/door-detection, door-detection) tools/door_replay.py: door detection replayed offline on a bag (rosbags, no rclpy)
| |/ / /  
| | * / d0028f2e (origin/obstacles-v2, obstacles-v2) B7: the v2 replay on today's bags, and the report
| |/ /  
|/| |   
* | | 29dd3aa0 output/avoidance: log for A1, A2, A5, and the A3 premise check
* | | 69830ecf output/avoidance: the wall-boundaries report, audit first
|/ /  
* | 0c1cd417 post-merge: a no-transform scan publishes its NO_DATA
* | c6d8f990 V1-V4: the decisions log, and the real-DDS verification
|/  
* 9c46addc H5: one teardown for both paths, which is where the orphans came from
* 573f313b (orin/test-logging) test: campaing: m01 with obstacle done
```

Jazzy line, by hand (the generated graph does not fit on a page):

```
e47e646 main@09-30 ─┬─ old Jazzy port ── af1f167 orin/jazzy ── … ── 3dbe37f ─┬─ eb78d6b jazzy (L, +2)
                    │                                                         └─ a596962 origin/jazzy (+4, James)
                    └─ 573f313 ── campaign-final 10-14 (7b17e2f) ── jazzy-campaign 883a0f0 ─┬─ 60f9af7 jazzy-campaign
                                                                                         ├─ 2893d41 jazzy-next  (⊕ CF 868ec8a)
                                                                                         └─ aa721fd jazzy-rust  (⊕ CF 0c1cd41)
```
