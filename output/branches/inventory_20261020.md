# Branch inventory: f1tenth_more, 2026-10-20

Read-only. Nothing was checked out, committed, stashed, rebased, deleted or
pushed. The only file written is this one, and it is not committed. Section 6
is a plan for you to approve. None of it has been run.

**How the facts were gathered**
- `git fetch origin`: nothing new.
- `git fetch orin`: `campaign-final-doors` moved `904d86e → 6c693ac`.
- `git fetch orin refs/stash`: fetched the Orin's stash commit only, into
  FETCH_HEAD, so it could be read. No ref was created.
- Read-only `ssh fabiocar@jetson-orin` for the Orin checkout. Commands:
  `git --no-optional-locks status / worktree list / stash list / submodule status`,
  and `git hash-object` on the untracked M02 files.
- Ancestry: `merge-base --is-ancestor`, `rev-list --left-right --count`.
- Duplicate commits: `git cherry` and `git patch-id --stable` over every
  commit since `7b17e2f` on every live branch.
- Conflict predictions: `git merge-tree --write-tree`. Per-commit rebase
  simulations ran in a throw-away `git clone --bare --shared` in the session
  scratchpad. No ref in this repository was touched.

---

## 0. Short version

- **Recommended car base:** `campaign-final`, fast-forwarded to
  `orin/campaign-final-doors` **6c693ac**.
  - 6c693ac is what the Orin has checked out and drives today.
  - It contains every other Humble base-like ref.
  - On top of it go the M02 data commit `60f9af7` (cherry-picked off
    `jazzy-campaign`) and the cpu-cleanup worktree's uncommitted notes.
  - Both remotes move by fast-forward only. No force-push of the base.
- **door-fixes:** on that base it is exactly its own **15** commits. Rebasing
  it onto the tip is **predicted conflict-free**.
- **mpc_corr_v2_reverse:** **12** own commits.
  - Rebasing onto 6c693ac is predicted to stop in **5 commits**, all in
    `MPC_corr.py`. One of them also stops in `f1tenth_messages/CMakeLists.txt`,
    a trivial conflict.
  - Behind the conflict there is a real semantic merge. The stop ramp and the
    wall-rows obstacles meet reverse's `_planning_obstacles()` and the
    reverse-manoeuvre early return (§5.3).
- **Work that exists only on linus or only on the car:**
  - linus only: `jazzy-campaign` `60f9af7`, `door-fixes`, `obstacles-v2`,
    `jazzy-next`, `jazzy-rust`, and 4 local tags.
  - car only: 11 Orin commits (`orin/campaign-final` +
    `campaign-final-doors`). GitHub has none of them.
  - Uploading all of it is ~34 MB, with no blob over 20 MB.
- **Blockers before cleanup:**
  - The cpu-cleanup worktree has an uncommitted report addendum and 20 MB of
    replay output.
  - `orin-verify-campaign-dirty` has commits that no tag holds.
  - The Orin's untracked M02 run folders collide with `60f9af7`. The content
    is byte-identical, so this is solved by moving the folders aside.
- **Safe to delete after a backup tag:**
  - local: the six `tmp/door-fixes-*` branches, `cpu-cleanup`, `door-detection`;
  - worktrees: the two detached ones (`_jcbase`, `_jzref`) and the `/tmp` one;
  - remote: 10 stale branches on the Orin, and `cpu-cleanup` + `door-detection`
    on origin.

---

## 1. Inventory

### 1.1 Local branches

"On a remote" means the tip is contained in some `origin/*` or `orin/*` ref.

| Branch | Tip | Date | Subject | Upstream | Ahead/behind | On a remote? | Worktree | Dirty |
|---|---|---|---|---|---|---|---|---|
| `campaign-final` | `868ec8a` | 10-18 | wall_v2: log and decisions for Q0-Q2 | origin/campaign-final | 0/0 (orin/campaign-final is 8 ahead) | yes | none | – |
| `cpu-cleanup` | `82c0afa` | 10-16 | cpu-cleanup R3 block 6: diagnostics group no longer "crashes" with the logger off | none (= origin/cpu-cleanup) | – | yes | `~/dev_ws/f1tenth_more_cpu_cleanup` | `M output/cpu_cleanup/log.md`, `M output/cpu_cleanup/report.md`, `?? output/run_corridor10m_20261016T144537/`, `?? …_v2/` (see §4c) |
| `door-detection` | `c873247` | 10-19 | tools/door_replay.py: door detection replayed offline on a bag | origin/door-detection | 0/0 | yes | `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/1d54b94c-…/scratchpad/ws/src_wt` | clean |
| `door-fixes` | `cde3111` | 10-20 | output/perception/doors.md: finish the addendum report; doors_phase1 part 2 | none | – | **local only** | `~/wt/doors` | `M .claude/settings.local.json` (one allowlist line) |
| `jazzy` | `3dbe37f` | 10-13 | LOG: jazzy test parity run (corridor funnel, base_front, pan off, grey boxes) | origin/jazzy | 0/0 | yes | none (`_jzref` is detached at the same commit) | – |
| `jazzy-campaign` | `60f9af7` | 10-19 | campaign: M02_approach_wall P002-R012..R020 (2026-10-19) + tools/test_campaign_video.py | origin/jazzy-campaign | **ahead 1** | 60f9af7 **local only** | main checkout `~/dev_ws/f1tenth_more` | `m` zed_ros2_wrapper, `m` zed-ros2-interfaces, `?? docs/branches.md`, `?? src/husarion_gz_worlds/` |
| `jazzy-next` | `2893d41` | 10-19 | Jazzy build/test fixes for the campaign-final merge (from jazzy-rust e025898, non-Rust part) | none | – | **local only** | `~/dev_ws/f1tenth_more_jazzy` | `?? output/jazzy_align/` (another session's analysis) |
| `jazzy-rust` | `aa721fd` | 10-18 | Guard: stale /scan judged on ROS time under use_sim_time; timeout and Isaac parity | none | – | **local only** | `~/dev_ws/f1tenth_more_rust` | `?? src/husarion_gz_worlds/` |
| `main` | `e47e646` | 09-30 | repo root: move the loose docs under docs/, drop the stale ones | origin/main | 0/0 | yes | none | – |
| `mpc_corr_v2_reverse` | `a14ec2f` | 10-20 | Mission start countdown only while the mission logger records | origin/mpc_corr_v2_reverse | 0/0 | yes | `~/dev_ws/f1tenth_more_reverse` | clean |
| `obstacles-v2` | `d0028f2` | 10-19 | B7: the v2 replay on today's bags, and the report | **orin/campaign-final** (wrong: see §3.4) | ahead 5, behind 1 | **local only** | `~/dev_ws/f1tenth_more_obst` | clean |
| `tmp/door-fixes-core` | `9b73a63` | 10-20 | doors: the addendum's door core -- hosts, perpendicular gap tests, … | none | – | local only | `~/wt/doors-core` | clean |
| `tmp/door-fixes-node` | `dd5c565` | 10-20 | doors: the node, markers, params and launch on the addendum's core … | none | – | local only | `~/wt/doors-node` | clean |
| `tmp/door-fixes-replay` | `35b4d8e` | 10-19 | tools/door_replay.py: car cadence and params, --src, --walls retrack, … | none | – | local only | `~/wt/doors-replay` | clean |
| `tmp/door-fixes-replay2` | `d8ece07` | 10-20 | tools/door_replay.py: a candidate door drawn faded and dashed, its posts lime | none | – | local only | `~/wt/doors-replay2` | clean |
| `tmp/door-fixes-tracker` | `ea4beb4` | 10-19 | test_extraction_backends: the stack's merge gap (0.70 m) at its boundary, … | none | – | local only | `~/wt/doors-tracker` | clean |
| `tmp/door-fixes-unknown` | `dee1687` | 10-19 | multi_wall_tracker: role 'unknown' replaces 'other'; tentative tracks reported unknown | none | – | local only | `~/wt/doors-unknown` | clean |
| *(detached, `_jcbase`)* | `883a0f0` | 10-16 | restart.sh: /slam/pose = latest since the reset | – | – | yes (= origin/jazzy-campaign) | `~/dev_ws/f1tenth_more_jcbase` | clean (ignored `src/f1tenth_behavior/mission_reports/`; submodules initialised) |
| *(detached, `_jzref`)* | `3dbe37f` | 10-13 | LOG: jazzy test parity run … | – | – | yes (= origin/jazzy) | `~/dev_ws/f1tenth_more_jzref` | clean (submodules not initialised) |

There are no local stashes.

### 1.2 Remote branches

**origin** (GitHub, `HEAD → main`)

| Branch | Tip | Date | Note |
|---|---|---|---|
| `campaign-final` | `868ec8a` | 10-18 | = local `campaign-final`; 19 behind `orin/campaign-final-doors` |
| `cpu-cleanup` | `82c0afa` | 10-16 | merged into campaign-final (`e9ab0a0`) |
| `door-detection` | `c873247` | 10-19 | contained in door-fixes and orin/campaign-final-doors |
| `jazzy` | `3dbe37f` | 10-13 | old Jazzy port line |
| `jazzy-campaign` | `883a0f0` | 10-16 | local is 1 ahead (`60f9af7`) |
| `main` | `e47e646` | 09-30 | |
| `mpc_corr_v2_reverse` | `a14ec2f` | 10-20 | = local |

**orin** (`fabiocar@jetson-orin:dev_ws/f1tenth_more`: Ubuntu 22.04, ROS Humble, git 2.34.1)

| Branch | Tip | Date | Subject | Note |
|---|---|---|---|---|
| `campaign-final-doors` | `6c693ac` | 10-20 | stop ramp: arrive at a stopping point instead of crossing it | **checked out on the Orin (HEAD)**. 19 ahead of origin/campaign-final, 11 of them on no GitHub ref |
| `campaign-final` | `b08675e` | 10-19 | replay: --param-file/--variant, side_coverage, undecodable-topic tolerance (WIP from the Orin) | 8 ahead of origin/campaign-final, none on GitHub |
| `jazzy` | `af1f167` | 10-12 | Phase 5D docker: WIP | ⊂ origin/jazzy (42 behind). Orin worktree `~/dev_ws/f1tenth_jazzy`, clean |
| `llm`, `scene-graph`, `sick`, `main` | `e47e646` | 09-30 | repo root: move the loose docs … | = main |
| `orin-snapshot-20261013` | `8c9e29b` | 10-13 | output: orin_check_report.md, dummy run count | = tag `archive/orin-snapshot-20261013` (on origin) |
| `orin-verify-campaign-dirty` | `3ae2108` | 10-13 | campaign state: missions, MATLAB export, corridor_perception Step 1, scripts (offline) | **no tag, not on GitHub** (§4c) |
| `refactor/mpc-corridor-split` | `a3ea50d` | 10-09 | mpc_controller: split the corridor out of MPC_corr into mpc_controller/corridor | = tag `archive/mpc-corridor-split` (on origin). Orin worktree `~/dev_ws/f1tenth_mpc_refactor`, clean |
| `test-campaign` | `6ebbc8a` | 10-12 | wall_turn: no front distance = no wall decision, BLOCKED no_wall_distance after 2 s (B8) | ⊂ tag `archive/test-campaign` (349c881) and ⊂ origin/jazzy |
| `test-logging` | `573f313` | 09-30 | test: campaing: m01 with obstacle done | ⊂ every campaign branch |

**The Orin checkout** (`~/dev_ws/f1tenth_more` on `campaign-final-doors`), read over ssh:
- Tracked changes:
  - `M .claude/settings.json`;
  - `M first_test_campaign/results.csv` (+9 rows);
  - vesc submodule `?`: untracked `__pycache__/` and `vesc_tuning/speed_sweep_results/`.
- Untracked M02 runs: the nine `first_test_campaign/M02_approach_wall/P002-R012…R020-20261019T*` folders.
  - These, and `results.csv`, are **byte-identical** to what `60f9af7` commits (`git hash-object` on all 122 files).
  - Exception: the nine `corridor_plot.png` files exist only in `60f9af7`. They were generated on linus.
- Other untracked items (outside this plan's scope):
  - `build_base573/`, `install_base573/`;
  - `first_test_campaing/` (the old misspelled folder);
  - at least 20 `output/…` entries (replays, videos, cpu tests). The status
    listing was cut at 40 lines.
- Stashes: 5.
  - `stash@{0}` (2026-10-08, on scene-graph) is a 3-line change to
    `llm_planner_node.py`: legacy SYSTEM_PROMPT wall threshold 3.0→2.0,
    inactive with `planner_path=v2`.
  - `@{1..4}` are older (vesc-tuner-test, stereocamera-test).
  - Left alone.
- The Orin's own view of GitHub is stale: its `origin/campaign-final` is `3ffe245`. It also keeps `refs/remotes/gh/*` and a self-referencing `orin` remote. Noted, out of scope.

### 1.3 Worktrees (linus)

| Path | Checkout | State | Processes in it now |
|---|---|---|---|
| `~/dev_ws/f1tenth_more` | jazzy-campaign | zed submodules empty (§4 submodules), 2 untracked | 2 claude, VS Code |
| `~/dev_ws/f1tenth_more_cpu_cleanup` | cpu-cleanup | **uncommitted report addendum + 20 MB replay output** | none |
| `~/dev_ws/f1tenth_more_jazzy` | jazzy-next | untracked `output/jazzy_align/` (7 files, ~800 KB) | none |
| `~/dev_ws/f1tenth_more_jcbase` | detached 883a0f0 | clean, 430 MB (build/install) | none |
| `~/dev_ws/f1tenth_more_jzref` | detached 3dbe37f | clean, 419 MB | none |
| `~/dev_ws/f1tenth_more_obst` | obstacles-v2 | clean | none |
| `~/dev_ws/f1tenth_more_reverse` | mpc_corr_v2_reverse | clean | none |
| `~/dev_ws/f1tenth_more_rust` | jazzy-rust | untracked `src/husarion_gz_worlds/` | none |
| `~/wt/doors` | door-fixes | 1 line in `.claude/settings.local.json` | none |
| `~/wt/doors-{core,node,replay,replay2,tracker,unknown}` | tmp/door-fixes-* | all clean, 250–310 MB each | none |
| `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/1d54b94c-bc85-4ffd-8443-44aacfa3c6e1/scratchpad/ws/src_wt` | door-detection | clean, 257 MB; **inside another Claude session's scratchpad** | none |

`src/husarion_gz_worlds/` (main checkout and `_rust`) is the leftover
checkout of the old jazzy branch's submodule (`87271ef`). No current branch
has that submodule.

### 1.4 Tags

- **On origin:**
  - `archive/{mpc-corridor-split, orin-snapshot-20261013, pre-campaign-final-20261012, test-campaign}`;
  - `backup/{campaign-final-9c46add, campaign-final-c6d8f99, pre-wall-v2-0c1cd417}`;
  - `evidence/campaign-dirty-p004-r012-r019`;
  - `jazzy-p0, p1, p3, p4, p5`.
- **Local only (not on origin):**

  | Tag | Points at | Status |
  |---|---|---|
  | `backup/jazzy-pre-align-3dbe37f` | `3dbe37f` | = origin/jazzy |
  | `backup/jazzy-rust-pre-sync-ced4d90` | `ced4d90` | patch-identical to `a9b9618` on jazzy-rust |
  | `backup/pre-avoidance-69830ecf` | Orin-only commit | ⊂ orin/campaign-final |
  | `backup/pre-obstacles-v2-29dd3aa` | Orin-only commit | ⊂ orin/campaign-final |

---

## 2. Lineage of the base-like branches

```
main e47e646 (09-30, Humble)  = origin/main = orin/main = orin/llm = orin/scene-graph = orin/sick
├─ old Jazzy port (118) ─ af1f167 orin/jazzy ─ (42) ─⊕ merge test-campaign 6af1ddf ─ … ─ 3dbe37f jazzy = origin/jazzy  [_jzref]
└─ 573f313 orin/test-logging (M01 data)
   ├─ 1d00853 "7/10 tests" ─(24)─ 6ebbc8a orin/test-campaign ─┬─ 349c881 tag archive/test-campaign
   │     │                                                     ├─ 8c9e29b orin/orin-snapshot-20261013 (= tag)
   │     │                                                     └─ merged into jazzy (6af1ddf)
   │     └─ a3ea50d orin/refactor/mpc-corridor-split (+3, = tag archive/mpc-corridor-split)
   └─ campaign-final line (rebuilt on 573f313 without the test-campaign history)
      ├─ 5c25ade (10-13) ── 3ae2108 orin/orin-verify-campaign-dirty (+4)
      ├─ 7b17e2f (10-14) ── jazzy-campaign line: Jazzy re-port, 38 commits to 883a0f0
      │                       883a0f0 = origin/jazzy-campaign  [_jcbase]
      │                       ├─ 60f9af7  jazzy-campaign            (+1, local only)
      │                       ├─ 2893d41  jazzy-next  = 883a0f0 ⊕ 868ec8a (84f264b) + 1 fix   (local only)
      │                       └─ aa721fd  jazzy-rust  = 883a0f0 + 1 ⊕ 0c1cd41 (90b367c) + 6   (local only)
      ├─ 9c46add (10-16) ── cpu-cleanup 82c0afa (+13) ──⊕ merged by e9ab0a0
      ├─ 0c1cd41 (10-18, tag backup/pre-wall-v2-0c1cd417) ── merged into jazzy-rust
      └─ 868ec8a campaign-final = origin/campaign-final (10-18) ── merged into jazzy-next
         ├─ a14ec2f mpc_corr_v2_reverse (+12) = origin
         ├─ c873247 door-detection (+8) = origin
         │    └─ cde3111 door-fixes (+15, local only)  ⇐ tmp/door-fixes-* x6 (patch-identical copies)
         └─ dbb2da9 … 29dd3aa  A1/A2 avoidance + wall rows (+7, Orin only)
              ├─ d0028f2 obstacles-v2 (+5, local only)
              └─ b08675e orin/campaign-final (+1, replay WIP)
                   └─ 6c693ac orin/campaign-final-doors
                        = b08675e ⊕ door-detection (e727203, 904d86e) + stop ramp      ← Orin HEAD
```

**Which contains which** ("A ⊂ B" means A is an ancestor of B):

- **Humble car line:**
  `main ⊂ 573f313 ⊂ cpu-cleanup ⊂ 868ec8a (campaign-final) ⊂ orin/campaign-final ⊂ orin/campaign-final-doors`.
  - Also `door-detection ⊂ orin/campaign-final-doors`.
  - `orin/campaign-final-doors` therefore contains **every** Humble base-like
    ref: 868ec8a, b08675e, cpu-cleanup, door-detection, 0c1cd41 and 29dd3aa.
- **Feature forks:**
  - `868ec8a ⊂ mpc_corr_v2_reverse`;
  - `868ec8a ⊂ door-detection ⊂ door-fixes`;
  - `29dd3aa (= orin/campaign-final~1) ⊂ obstacles-v2`. obstacles-v2 lacks `b08675e`.
- **The base of door-fixes** is `c873247` (door-detection's tip), which itself
  forks at `868ec8a`.
- **Jazzy-campaign line:**
  - `jazzy-campaign` contains `7b17e2f` (campaign-final of 10-14), but **not**
    `868ec8a` and **not** `jazzy`.
  - `jazzy-next ⊃ 883a0f0 + 868ec8a (+ cpu-cleanup)`. It lacks `60f9af7`.
  - `jazzy-rust ⊃ 883a0f0 + 0c1cd41 (+ cpu-cleanup)`. It lacks `868ec8a`
    (the 7 wall_v2 commits P0–Q2).
  - `jazzy-next` and `jazzy-rust` share `0c1cd41` and diverge: 9 / 8 commits.
- **Old Jazzy line:**
  - `jazzy ⊃ orin/jazzy`.
  - `jazzy ⊃ test-campaign ⊃ 573f313`, through the merge `6af1ddf`.
  - jazzy shares nothing with the campaign-final line after 573f313:
    159 / 159 commits against jazzy-next.

---

## 3. Feature branches

"Own" means a commit no other branch holds by patch-id. Duplicates were
searched across all branches since `7b17e2f`. The only duplicates found are
the `tmp/door-fixes-*` ↔ `door-fixes` pairs.

### 3.1 mpc_corr_v2_reverse

- **Fork point:** `868ec8a` (= origin/campaign-final).
- **Carries:** nothing from other features.
- **Own:** 12 commits, `1f7e9e4..a14ec2f`.

| Commit | Subject | Topic |
|---|---|---|
| `1f7e9e4` | output/reverse: recon | reverse |
| `780fd09` | reverse as the MPC's own choice: direction candidates, allow_reverse flag | reverse (MPC) |
| `9f78004` | wall-aware terminal cost-to-go | reverse (MPC) |
| `353bc07` | closed-loop scenarios, flag-off pin, report | reverse (MPC) |
| `b5f46ea` | Part A: reverse manoeuvre, automatic arbitration | reverse manoeuvre |
| `c33f969` | Part B: semantic object memory with visibility-aware decay | object memory |
| `9c24024` | llm_planner_node: start_mission on the load confirmation | **mission flow, not reverse** |
| `ba8779e` | Review of A/B/C | reverse |
| `6f48d19` | Part B fixes: the move's target is never an obstacle | object memory |
| `b7f9596` | Part D: BT integration of the reverse manoeuvre | reverse BT |
| `f884400` | refuse to start with both reverse mechanisms on | reverse |
| `a14ec2f` | Mission start countdown only while the mission logger records | **mission flow, not reverse** |

Files: 52 outside `output/` and 22 under `output/`. The main ones:
- **mpc_controller:**
  - `MPC_corr.py`, `mpc_solver.py`;
  - new `reverse_mpc.py`, `cost_to_go.py`, `reverse_manoeuvre.py`, `move_target.py`;
  - tests and harnesses.
- **f1tenth_costmap:** new `object_memory.py`; `semantic_layer.py`, `semantic_layer_node.py`.
- **f1tenth_messages:** `CMakeLists.txt`, `ObjectGoal.msg`, new `SemanticObject*.msg`, `ReverseStatus.msg`, `RequestReverse.srv`.
- **f1tenth_behavior:**
  - `behavior_executor_node.py`, `is_obstacle_detected.py`, `is_proximity_too_close.py`;
  - new `reverse_integration.py`;
  - `mission/loader.py`, `behavior_bringup.launch.py`.
- **f1tenth_bringup:** `component_supervisor_node.py`.
- **llm:** `llm_planner_node.py`.
- **f1tenth_logger:** `mission_logger_node.py`, `TEST_CAMPAIGN.md`.
- **f1tenth_params:** `stack_params.yaml`, new `semantic_objects.py`, `object_geometry.py`.

`9c24024` and `a14ec2f` are its own commits, but they do not belong to the
reverse topic.
- `9c24024` is the fix for the kind of failure `60f9af7` records: R018 and
  R019 "failed on load_mission timing out".
- Cherry-picked alone onto 6c693ac, each conflicts only in the reverse report
  `output/mpc_v2/reverse_and_memory.md`. They can be split off (§6, optional
  step 5d).

### 3.2 door-fixes

- **Fork point:** `c873247` (door-detection tip).
- **Carries:** door-detection's 8 commits `3034c94..c873247`. They are on
  origin and already merged into `orin/campaign-final-doors`.
- **Own:** 15 commits.
  - 11 are exact patch-id copies of the `tmp/door-fixes-*` work branches.
  - 4 were written directly on door-fixes.

| door-fixes | from tmp branch |
|---|---|
| `1b2155d` narrow wall behind a person is born again | tracker `9a9668b` |
| `a74a628` pin DOOR_WIDTH_MIN_M | tracker `244d736` |
| `1a6c7cc` end-wall doorway off the 0.70 m boundary | tracker `f2f7b7d` |
| `d1fc7cc` merge gap at its boundary, both backends | tracker `ea4beb4` |
| `0ee22d9` door_replay car cadence, --src, CSV | replay `35b4d8e` |
| `1148636` stack_params: span gates read the wall's span | (direct) |
| `c07e7d8` doors_phase1.md item 5 | (direct) |
| `d811e58` role 'unknown' replaces 'other' | unknown `dee1687` |
| `ed6899a` the addendum's door core | core `9b73a63` |
| `95d3a77` Door.jamb_height_unknown, STATE_CANDIDATE | node `e8fed2e` |
| `a49adca` node, markers, params, launch | node `dd5c565` |
| `9df1976` door_replay: every DoorCandidate a row | replay2 `0de5720` |
| `affd305` door_replay: candidate door drawn faded | replay2 `d8ece07` |
| `148d58e` optional front-facing hosts (default off) | (direct) |
| `cde3111` doors.md addendum report | (direct) |

Files (23 outside `output/`):
- **f1tenth_perception:** `door_detection.py`, `door_markers.py`,
  `lidar_front_wall_node.py`, `multi_wall_tracker.py`, `wall_markers.py`,
  `lidar_front_wall.launch.py`, and tests.
- **f1tenth_messages:** `Door.msg`, `DoorArray.msg`, `MultiWallTrack.msg`.
- **f1tenth_costmap:** `wall_raster.py` and its test.
- **corridor_perception:** `test_extraction_backends.py`.
- **f1tenth_params:** `stack_params.yaml`.
- **tools:** `tools/door_replay.py`, `tools/test_door_replay.py`.

### 3.3 jazzy-rust (Jazzy line)

- **Fork point:** `883a0f0` (origin/jazzy-campaign).
- **Carries:**
  - jazzy-campaign up to 883a0f0;
  - campaign-final up to `0c1cd41`, through the merge `90b367c`. That
    includes cpu-cleanup. The merge has its own resolutions in 4 files:
    `preflight.py`, `component_supervisor_node.py`, `reset_manager.launch.py`,
    `localization.launch.py`.
- **Own:**
  - `a9b9618`: zenoh recon. Patch-identical to `ced4d90`, the commit the tag
    `backup/jazzy-rust-pre-sync` points at.
  - `e025898`: sync and toolchain fixes. Its non-Rust half was re-applied on
    jazzy-next as `2893d41`. The patch-ids differ because it is a partial copy.
  - Rust and Isaac: `1af629a`, `2c6d4fc`, `1956a7b`, `a589089`, `aa721fd`.
- **Files:**
  - 20 under `src/rust/`, 21 under `tools/rust_parity/`, 162 under `output/`;
  - plus `.gitignore`, `scripts/sim/sim_isaac.sh`, `scripts/sim/sim_stack.sh`, `tools/e2e_test.sh`;
  - `corridor_perception_c/CMakeLists.txt`;
  - `f1tenth_bringup`: `component_supervisor_node.py`, `stack_bringup.launch.py`,
    `supervisor_bringup.launch.py`, `test_reset_manager_component.py`, `test_rust_switches.py`;
  - `f1tenth_costmap`: `costmap.launch.py` and a test;
  - `f1tenth_behavior`: `behavior_bringup.launch.py` and a test.
- **Thematic overlap with the car line:** its Rust front-proximity guard
  covers ground close to A2's Python `hard_stop_node` (the last-resort LiDAR
  stop on the base).

### 3.4 obstacles-v2

- **Fork point:** `29dd3aa`, i.e. `orin/campaign-final` without its last
  commit `b08675e`.
- **Carries:** the 7 Orin commits `dbb2da9..29dd3aa`: A1/A2 avoidance, wall
  rows, hard stop. They are on the Orin only.
- **Own:** 5 commits.
  - `88cfc1c` B1/B2: unknown-obstacle cells;
  - `6ece508` B3: one tracker;
  - `79562b1` B4–B6: one obstacle path to the MPC, speed cap;
  - `67beb26` B7 findings;
  - `d0028f2` B7 replay + report.
- **Files:**
  - `f1tenth_costmap`: `unknown_objects.py` (new), `semantic_layer.py`,
    `semantic_layer_node.py`, and tests;
  - `mpc_controller`: `MPC_corr.py`, `mpc_solver.py`, `obstacle_select.py` (new),
    `test_obstacles_v2.py`, `test_abort_holds_the_car.py`;
  - `stack_params.yaml`, `tools/obstacles_v2_replay.py`.
- **Upstream misconfigured:** it tracks `orin/campaign-final`. Fixed by the
  `-u` push in §6 step 2.

### 3.5 cpu-cleanup

- **Fork point:** `9c46add` (campaign-final, 10-16).
- **Own:** 13 commits, `69d2e1c..82c0afa`, all merged into campaign-final by
  `e9ab0a0`. Through that merge they are in every Humble feature, in
  jazzy-next and in jazzy-rust.
- **Nothing left on the branch.** What is left is in the worktree: §4c.

### 3.6 Files touched by more than one feature

Feature groups:
- `REV` = mpc_corr_v2_reverse;
- `DOORDET` = door-detection (carried by door-fixes; already in the proposed base);
- `DOORFIX` = door-fixes' own commits;
- `OBST` = obstacles-v2's own commits;
- `AVOID` = the A1/A2 Orin commits (already in the proposed base);
- `STOP` = the stop-ramp commit 6c693ac (already in the proposed base);
- `REPLAYWIP` = b08675e (already in the proposed base);
- `RUST` = jazzy-rust's own commits;
- `JNEXT` = jazzy-next's own commits.

| File | Touched by | Predicted |
|---|---|---|
| `mpc_controller/MPC_corr.py` | REV, OBST, AVOID, STOP | **conflict:** REV onto base (5 commits), OBST onto base (1 commit), REV×OBST (4 hunks) |
| `mpc_controller/mpc_solver.py` | REV, OBST | **conflict:** REV×OBST (1 hunk) |
| `f1tenth_costmap/semantic_layer.py` | REV, OBST | **conflict:** REV×OBST (4 hunks) |
| `f1tenth_costmap/semantic_layer_node.py` | REV, OBST | **conflict:** REV×OBST (5 hunks) |
| `f1tenth_messages/CMakeLists.txt` | REV, DOORDET | **conflict:** adjacent msg list entries, keep both |
| `f1tenth_params/config/stack_params.yaml` | REV, DOORDET, DOORFIX, OBST, AVOID, STOP | clean in every simulation (each adds its own keys) |
| `f1tenth_behavior/behavior_executor_node.py` | REV, STOP | clean textually; **semantic check** (stop target vs reverse BT) |
| `f1tenth_logger/mission_logger_node.py` | REV, DOORDET | clean |
| `f1tenth_behavior/launch/behavior_bringup.launch.py` | REV, RUST | clean on the car line; matters only when the Jazzy line takes REV |
| `f1tenth_bringup/component_supervisor_node.py` | REV, RUST | same as above |
| `f1tenth_costmap/launch/costmap.launch.py` | AVOID, RUST | Jazzy line only |
| `corridor_perception_c/CMakeLists.txt`, `f1tenth_bringup/test/test_reset_manager_component.py` | RUST, JNEXT | same fix, applied twice (2893d41 is e025898's non-Rust half) |
| `tools/corridor_wall_replay.py` | DOORDET, DOORFIX, REPLAYWIP | already resolved in the Orin merge `e727203`; DOORFIX onto base is clean |
| door files (`door_detection.py`, `Door*.msg`, `multi_wall_tracker.py`, `wall_markers.py`, `lidar_front_wall_node.py`, tests, `tools/door_replay.py`, `output/perception/doors.md`) | DOORDET, DOORFIX | same line of work; clean |

The prediction tool:
- REV, DOORFIX and OBST were replayed one commit at a time onto 6c693ac in the
  scratch clone, with `merge-tree --merge-base`.
- After the first conflict in a file, that file's markers stay in the
  simulated tree, so later per-commit verdicts for the same file are
  approximate.

**Semantic touch point not visible as a conflict:**
- door-fixes `d811e58` renames the wall role `'other'` → `'unknown'` (in
  `MultiWallTrack.msg` and `wall_raster.py`).
- The base's A1 code feeds "other" walls to the MPC as soft obstacles:
  `wall_boundaries.other_wall_obstacles`, used in `MPC_corr.py`.
- It selects them by `role not in ROW_ROLES` (front/left/right), so
  `'unknown'` still qualifies.
- Looks safe. Confirm with `test_wall_boundaries.py` after the rebase.

---

## 4. Classification

### (a) Candidate bases

| Ref | For | Verdict |
|---|---|---|
| `orin/campaign-final-doors` 6c693ac | car (Humble) | **recommended** (§5) |
| `orin/campaign-final` b08675e | car | subset of the above; no doors, no stop ramp |
| `campaign-final` = `origin/campaign-final` 868ec8a | car | subset; 19 behind what the car runs |
| `main` e47e646 | car | stale since 09-30; fast-forward it later |
| `jazzy-next` 2893d41 | Jazzy | the most current Jazzy integration (883a0f0 + campaign-final 868ec8a); separate line |
| `jazzy-campaign` 60f9af7 | Jazzy | the Jazzy re-port; 60f9af7 is car data that belongs on the car line too |
| `jazzy` 3dbe37f | Jazzy (old) | old port. `_jazzy/output/jazzy_align/old_jazzy_only.md` (uncommitted, another session) lists groups to BRING BACK, so keep it |

### (b) Live features

- Car line: `mpc_corr_v2_reverse`, `door-fixes`, `obstacles-v2`.
- Jazzy line: `jazzy-rust`.

### (c) tmp / save refs: anything not already in a live branch?

| Ref | Unique content? |
|---|---|
| `tmp/door-fixes-core / node / replay / replay2 / tracker / unknown` | **No.** `git cherry door-fixes <tmp>` gives `-` for all 11 commits. |
| detached `_jcbase` 883a0f0 | **No.** = origin/jazzy-campaign. |
| detached `_jzref` 3dbe37f | **No.** = origin/jazzy and tag `backup/jazzy-pre-align-3dbe37f`. |
| `/tmp/…/src_wt` (door-detection c873247) | **No.** = origin/door-detection. |
| `orin/orin-snapshot-20261013` 8c9e29b | **No.** = tag `archive/orin-snapshot-20261013`. Its `output/orin_check_report.md` is on no live branch, only in the tag. |
| `orin/refactor/mpc-corridor-split` a3ea50d | 3 commits on no live branch (`1e23f6a`, `ddd0235`, `a3ea50d`), but the tag `archive/mpc-corridor-split` holds them. The keep/port decision from `docs/branches.md` (10-14) is still open. |
| `orin/orin-verify-campaign-dirty` 3ae2108 | **Yes, partly. No tag holds it.** Details below this table. |
| `orin/test-campaign`, `orin/test-logging`, `orin/llm / scene-graph / sick`, `orin/jazzy` | **No.** Each is contained in origin/jazzy, in the archive tags or in main. |
| `cpu-cleanup` (branch) | **No.** Merged. |
| **cpu-cleanup worktree** | **Yes:** uncommitted notes and replay output. Details below this table. |
| `_jazzy` worktree `output/jazzy_align/` | **Yes**, untracked: the old-jazzy-vs-jazzy-next inventory (`old_jazzy_only.md`, 111 KB, plus test logs). Another session's work. Leave it to that session to commit on jazzy-next. |
| main checkout `docs/branches.md` | Untracked inventory of 10-14, superseded by this file. Your call whether to delete it. |
| Orin untracked M02 runs | Byte-identical to `60f9af7` (§1.2). Moved aside in §6 step 4. |

**orin-verify-campaign-dirty in detail.** This is the 10-13 verification
branch for the cherry-pick proposal P1–P4 in
`archive/orin-snapshot-20261013:output/orin_check_report.md`.
- `703ad07` (P1) is patch-identical to a commit on campaign-final.
- `70d90d1`, `6505420` and `3ae2108` (P2–P4) have no patch equivalent anywhere.
- By content:
  - Most P2/P3 files are byte-identical on campaign-final.
  - The P4 files (MATLAB export, `corridor_perception/tracker.py`,
    `tools/prune_runs.py`, `scripts/drive_3m.py`, `missions/test.json`) are
    byte-identical on jazzy and in the archive tags.
  - About 30 files (`MPC_corr.py`, `front_clearance_node.py`, `stack_params.yaml`, …)
    exist in this exact version nowhere else, because they are the merged
    verification state.
- **Tag it before deleting.**

**cpu-cleanup worktree in detail.**
- `output/cpu_cleanup/{log,report}.md` have +19 uncommitted lines: "Replay v2
  of corridor10m_20261016T144537". They report the regression that the FRONT
  confidence gate rejects the end wall, which campaign-final later addressed
  in P0 `6cbda12`.
- `output/run_corridor10m_20261016T144537{,_v2}/` hold 20 MB, including two
  mp4s.
- None of this is on any branch.
- Both `.md` files are unchanged between `82c0afa` and 6c693ac (same blobs
  `19e3dd5` / `abb1ce8`), so the diff applies as is.

### (d) Fully merged: safe to delete after a backup tag

- **Local:**
  - `tmp/door-fixes-*` (6): patch-identical copies, not ancestors, so this
    needs `-D`;
  - `cpu-cleanup`: ⊂ campaign-final;
  - `door-detection`: ⊂ door-fixes and ⊂ the new base.
- **origin:**
  - `cpu-cleanup`;
  - `door-detection`, once door-fixes and the new campaign-final are pushed.
- **orin:**
  - `llm`, `scene-graph`, `sick`: = main;
  - `test-logging`, `test-campaign`, `jazzy`, `orin-snapshot-20261013`,
    `refactor/mpc-corridor-split`: each tagged on origin;
  - `orin-verify-campaign-dirty`: after its tag;
  - `campaign-final-doors`: after the base fast-forward and after the Orin has
    switched to `campaign-final`.
- **Keep:**
  - `main`, `campaign-final`;
  - `jazzy`, `jazzy-campaign`, `jazzy-next`, `jazzy-rust`;
  - the three car features;
  - the Orin's `main` and `campaign-final`.

### Submodules

| Line | vesc | ackermann_mux | teleop_tools | transport_drivers | zed_ros2_wrapper | zed-ros2-interfaces | husarion_gz_worlds |
|---|---|---|---|---|---|---|---|
| Humble: main, test-logging, every campaign-final ref, cpu-cleanup, door-*, tmp/*, reverse, obstacles-v2, every stale orin/* | 2848a9f | 42b7cd7 (humble-jetson) | 4337558 | d3f510c (1.2.0) | b5844a8 (humble-v4.2.5) | e970087 (5.3.0) | – |
| jazzy-campaign (incl. 883a0f0), jazzy-next, jazzy-rust | 2848a9f | 42b7cd7 | 4337558 | – (apt) | **cce25d3 (v5.5.0-8)** | e970087 | – |
| jazzy, orin/jazzy, `_jzref` | 2848a9f | b3c0b08 (upstream jazzy) | 4337558 | – | 86171bc (v5.4.1) | e970087 | 87271ef |

- **The car line pins identical submodules everywhere,** so the plan causes
  no submodule churn.
- `.gitmodules` on the car line has no `branch=` lines. The Jazzy lines set
  `vesc branch = jazzy`.
- **cce25d3 was a mistake.** The jazzy-campaign line's `cce25d3` is the
  accidental bump in `883a0f0`: v5.4.1 → v5.5.0-8, committed with `commit -a`.
  Step 14 (`bddd014`) had pinned v5.4.1. It is still on jazzy-campaign,
  jazzy-next and jazzy-rust. `old_jazzy_only.md` records the same finding.
- **Dirty submodules:**
  - main checkout: the zed_ros2_wrapper (102 files) and zed-ros2-interfaces
    (37 files) checkouts have an empty working tree **and** an empty index.
    The directory mtimes are 2026-09-30. zed is on
    `output/jazzy_port/linus_skip.txt`, so this looks deliberate and
    long-standing, but `git status` reports both as `m`.
  - Orin: vesc has untracked `__pycache__/` and `speed_sweep_results/`.
  - Every other worktree's submodules are clean or not initialised.

---

## 5. Recommendation

### 5.1 One base for the car: `campaign-final` at 6c693ac (+ the M02 data)

Fast-forward `campaign-final`, on origin and on the Orin, to
`orin/campaign-final-doors` **6c693ac**. Then add two commits:
1. a cherry-pick of `60f9af7`: the car's M02 data, plus `tools/test_campaign_video.py`;
2. the cpu-cleanup worktree's uncommitted replay-v2 notes.

The Orin then switches from `campaign-final-doors` to `campaign-final`, and
`campaign-final-doors` goes away.

Reasons:
1. **It is what the car runs.**
   - 6c693ac is HEAD on the Orin.
   - Its defaults are the live configuration: `enable_stop_ramp: true`,
     `boundary_source: walls`, `lidar_front_wall_door_detection_enabled: true`.
2. **It contains every Humble base-like ref:** 868ec8a, b08675e, cpu-cleanup,
   door-detection, and the fork points of all three car features. No base
   commit is left out on any side.
3. **Fast-forward only.**
   - `origin/campaign-final 868ec8a → 6c693ac` and `orin campaign-final b08675e → 6c693ac`
     are both ancestors-to-descendant.
   - No force-push of a shared branch. That keeps the "never force-push
     campaign-final" rule.
4. **Each feature becomes exactly its own commits:**
   - `campaign-final..door-fixes` = 15;
   - `..mpc_corr_v2_reverse` = 12;
   - `..obstacles-v2` = 5.
5. **It puts the 11 Orin-only commits on GitHub.** Today they exist only on
   the car and in linus' remote-tracking refs.
6. **Same submodule pins as every Humble ref.**
7. **60f9af7 belongs there.**
   - It is Humble car data: the M02 session of 10-19 on the car.
   - It applies cleanly: `results.csv` has the same base blob `31f8dd5` on
     both lines.
   - Its tool's imports (`f1tenth_logger.test_campaign.corridor_def`,
     `horizon_log`) exist on 6c693ac.
   - jazzy-campaign keeps its own copy. A later jazzy-next ← campaign-final
     merge sees the same change on both sides.

Costs:
- It carries `b08675e` "WIP from the Orin". That commit touches tools only:
  `tools/corridor_wall_replay.py`, `tools/scene_labels_replay.py`.
- The base now includes door detection and the stop ramp. Both already run on
  the car, but neither has been on GitHub before.

The Jazzy line stays separate. `jazzy-next` is its natural base. Pushing
`jazzy-campaign`, `jazzy-next` and `jazzy-rust` (§6 step 2) is only a backup.

### 5.2 door-fixes on the base

- **Already only its own changes.** door-detection is inside the base, so
  `campaign-final..door-fixes` will list exactly its 15 commits with no
  rebase at all.
- **To sit on the tip** (and be tested with the stop ramp and the wall rows),
  run `rebase --onto campaign-final c873247`.
  - All 15 commits are predicted clean.
  - It needs `--autostash` for the one-line `.claude/settings.local.json`
    change in `~/wt/doors`.
- **Push:** a force-with-lease against the pre-rebase tip that step 2 pushed.
  The backup tag is taken first.
- **Tests:**
  - f1tenth_perception (door, tracker, marker and lidar_front_wall tests);
  - f1tenth_costmap `test_wall_raster.py`;
  - corridor_perception `test_extraction_backends.py`;
  - `tools/test_door_replay.py`;
  - mpc_controller `test_wall_boundaries.py`, for the role rename.

### 5.3 mpc_corr_v2_reverse on the base

`git rebase campaign-final` replays its 12 commits from 868ec8a.

Predicted stops: `780fd09`, `9f78004`, `b5f46ea`, `c33f969`, `6f48d19`. Each
is in `MPC_corr.py`. `c33f969` also stops in `CMakeLists.txt`: keep `Door*`
and `SemanticObject*` / `ReverseStatus` / `RequestReverse`.

The final-state merge of 6c693ac with the reverse tip has three
`MPC_corr.py` hunks:

1. **Imports.** Take the union of the two lists.
2. **The obstacle list.** Combine both sides.
   - The base builds `obstacles_global` from the live list, plus "other"-wall
     disks when `boundary_source == 'walls'`.
   - Reverse replaces it with `self._planning_obstacles()`: live plus
     remembered objects, without the move's target.
   - The resolution must start from `_planning_obstacles()` and then append
     the wall disks. Taking either side alone drops a feature.
3. **The `BOUNDARY/live` log line and reverse's block after it.** Keep both:
   the base's `source=` field, and reverse's `_update_cost_to_go`, the
   manoeuvre early return and `_publish_reverse_status`.

Semantic checks:
- The base passes `limits=self._stop_ramp_limits(vdes)` into the forward
  solve, a hard row on the speed box.
  - Check that reverse's direction-candidate `_mpc_solve` forwards it, or
    ignores it on purpose, for a reverse candidate.
  - The manoeuvre path bypasses the forward solver entirely, so the stop
    ramp never acts during a manoeuvre.
- `behavior_executor_node.py` now gets both PublishMoveGoal's
  `/mpc/stop_target` and the reverse BT integration.

Tests:
- **mpc_controller:**
  - `test_stop_profile.py`, `test_stop_ramp_closed_loop.py`;
  - `test_wall_boundaries.py`, `test_boundary_constraints.py`;
  - `test_reverse_candidates.py`, `test_cost_to_go.py`, `test_reverse_scenarios.py`;
  - `test_reverse_flag_off_identity.py`, `test_reverse_manoeuvre.py`;
  - `test_semantic_memory_consumer.py`, `test_go_to_target_not_an_obstacle.py`.
- **f1tenth_costmap:** `test_object_memory*.py`, `test_semantic_layer.py`.
- **llm:** `test_load_then_start.py`.
- **f1tenth_behavior:** `test_stop_ramp_arrival.py`,
  `test_bt_reverse_integration.py`, `test_mission_countdown.py`.
  - On linus this suite cannot be collected (no py_trees_ros), so run it on
    the Orin.

Push with force-with-lease against `a14ec2f`.

Optional: split the two mission-flow commits `9c24024` and `a14ec2f` onto
their own small branch off the base. Then reverse holds only the reverse
topic, and the load-confirmation fix can reach the car sooner. Step 5d.

### 5.4 obstacles-v2

- `rebase --onto campaign-final 29dd3aa` replays its 5 commits.
- `79562b1` is predicted to stop once in `MPC_corr.py` (1 hunk).
- REV and OBST overlap heavily: `MPC_corr.py`, `mpc_solver.py`,
  `semantic_layer.py`, `semantic_layer_node.py`. Whichever of the two lands
  on campaign-final second rebases onto the first.
- door-fixes is independent of both.

---

## 6. Plan (NOT executed)

Run everything under `bash`. This machine's shell is zsh: there, `$var:s…`
is a history modifier and unquoted variables are not word-split.

Do the linus part from `~/dev_ws/f1tenth_more`. Nothing in the plan checks
anything out there.

Before starting:
- Make sure no session is working in `~/wt/doors`, `_reverse`, `_obst`,
  `_cpu_cleanup` or the `/tmp/…/1d54b94c-…` worktree.
- Make sure the car is not in a run.

### Step 0: re-check that nothing moved since this inventory

```bash
cd ~/dev_ws/f1tenth_more
git fetch origin && git fetch orin
while read r h; do [ "$(git rev-parse --short=7 "$r")" = "$h" ] || echo "MOVED: $r (now $(git rev-parse --short=7 "$r"))"; done <<'EOF'
orin/campaign-final-doors 6c693ac
orin/campaign-final b08675e
origin/campaign-final 868ec8a
origin/jazzy-campaign 883a0f0
origin/mpc_corr_v2_reverse a14ec2f
origin/door-detection c873247
jazzy-campaign 60f9af7
door-fixes cde3111
obstacles-v2 d0028f2
jazzy-next 2893d41
jazzy-rust aa721fd
EOF
```

It must print nothing.

### Step 1: backup tags for every branch that will be rewritten or deleted, pushed to GitHub

Names follow `backup/<branch>-20261020`. Branches that exist only on the Orin
get `backup/orin/<branch>-20261020`.

```bash
while read name sha; do
  git tag -a "backup/${name}-20261020" "$sha" -m "backup before the 2026-10-20 branch cleanup: ${name} @ ${sha:0:7}"
done <<'EOF'
door-fixes                        cde3111e7a32d5e4f0cc0dbfc78443da6144a1f3
mpc_corr_v2_reverse               a14ec2f5780f8467e26eb9eb24cc0ed4eaf1ce43
obstacles-v2                      d0028f2efbc8d021b74d732875e3027159ddd72c
cpu-cleanup                       82c0afa3d34e6f7f34f32c53e7a48c668ce2c162
door-detection                    c873247400c81296342129a54a4fb5b76647276a
tmp/door-fixes-core               9b73a63b86cbfdb4e1d8bffe18067fdcb959de27
tmp/door-fixes-node               dd5c565b2eca9715c216acbb0d1363d63849395d
tmp/door-fixes-replay             35b4d8e425912958e4b3f8a7c3f4f17b14622b9f
tmp/door-fixes-replay2            d8ece076f919d44de2d4547d9e0e39af1b4649a7
tmp/door-fixes-tracker            ea4beb468036398e107d9a77bd8c6d6e7207f164
tmp/door-fixes-unknown            dee1687bd62a1a273e4e380daca51c146d8113d9
orin/campaign-final-doors         6c693ac281f2b238b74f18a9f1bcb154e91048df
orin/orin-verify-campaign-dirty   3ae21086f82c3a0c7f4a56d9df7b98514f764d17
orin/orin-snapshot-20261013       8c9e29b372625a75aace7a9fbb82d2e4998be34c
orin/refactor/mpc-corridor-split  a3ea50d051ad46e17f69c5b8f4754420ced73787
orin/test-campaign                6ebbc8aa58830c088c4acdfecc0ac06f99c2d615
orin/test-logging                 573f313b1ed502a34584ea9cf89b09baa17837e4
orin/jazzy                        af1f167b17f51c93546b5873aaf180c01bc24554
orin/llm                          e47e64683fb61e63a9c393c3df91cf3df89cb190
orin/scene-graph                  e47e64683fb61e63a9c393c3df91cf3df89cb190
orin/sick                         e47e64683fb61e63a9c393c3df91cf3df89cb190
EOF
git push origin 'refs/tags/backup/*:refs/tags/backup/*'
```

The push also uploads the 4 older local-only `backup/*` tags (§1.4).

### Step 2: push the unpushed work as it is (new branches or fast-forwards only)

```bash
git push origin jazzy-campaign            # 883a0f0 -> 60f9af7, fast-forward
git push -u origin door-fixes             # new (23 over 868ec8a, incl. door-detection)
git push -u origin obstacles-v2           # new; -u also replaces the wrong upstream orin/campaign-final
git push -u origin jazzy-next             # new
git push -u origin jazzy-rust             # new
```

### Step 3: the base, by fast-forward

```bash
# 3a. GitHub campaign-final 868ec8a -> 6c693ac (no force), then local campaign-final follows (it has no worktree)
git push origin 6c693ac281f2b238b74f18a9f1bcb154e91048df:refs/heads/campaign-final
git fetch origin campaign-final:campaign-final

# 3b. a worktree for the base
git worktree add ~/wt/cf campaign-final

# 3c. the car's M02 data (simulated: applies clean)
git -C ~/wt/cf cherry-pick -x 60f9af70052f6c8da364914ccf72530e2824de0a

# 3d. the cpu-cleanup worktree's uncommitted replay-v2 notes; the replay output is moved out of git
git -C ~/dev_ws/f1tenth_more_cpu_cleanup diff -- output/cpu_cleanup > ~/wt/cpu_cleanup_replay_v2.patch
git -C ~/wt/cf apply --index ~/wt/cpu_cleanup_replay_v2.patch
git -C ~/wt/cf commit -m "output/cpu_cleanup: replay v2 of corridor10m_20261016T144537 (notes left uncommitted on 2026-10-16)"
mkdir -p ~/bags/replays
mv ~/dev_ws/f1tenth_more_cpu_cleanup/output/run_corridor10m_20261016T144537 \
   ~/dev_ws/f1tenth_more_cpu_cleanup/output/run_corridor10m_20261016T144537_v2 ~/bags/replays/
git -C ~/dev_ws/f1tenth_more_cpu_cleanup restore output/cpu_cleanup

# 3e. publish (both fast-forward; the Orin's campaign-final is not checked out there, so the push is accepted)
git -C ~/wt/cf push origin campaign-final
git -C ~/wt/cf push orin campaign-final
```

### Step 4: the Orin moves from campaign-final-doors to campaign-final (on the car, not during a run)

```bash
ssh fabiocar@jetson-orin
cd ~/dev_ws/f1tenth_more
mkdir -p ~/m02_untracked_20261020
mv first_test_campaign/M02_approach_wall/P002-R01[2-9]-20261019T* \
   first_test_campaign/M02_approach_wall/P002-R020-20261019T* ~/m02_untracked_20261020/
cp first_test_campaign/results.csv ~/m02_untracked_20261020/
git restore first_test_campaign/results.csv
git switch campaign-final      # = 6c693ac + data/output commits: no rebuild needed
git fetch origin               # optional: refreshes the Orin's stale view of GitHub (its origin/campaign-final is 3ffe245)
for d in ~/m02_untracked_20261020/P002-*; do diff -r "$d" "first_test_campaign/M02_approach_wall/$(basename "$d")"; done
diff ~/m02_untracked_20261020/results.csv first_test_campaign/results.csv
```

- The `diff -r` loop must print only "Only in …: corridor_plot.png" lines.
- The `results.csv` diff must print nothing.
- `.claude/settings.json` carries over unchanged (same blob on both branches).

### Step 5: put the features on the base

```bash
# 5a. door-fixes: 15 own commits, predicted clean
git -C ~/wt/doors rebase --autostash --onto campaign-final c873247400c81296342129a54a4fb5b76647276a door-fixes
git log --oneline campaign-final..door-fixes | wc -l          # 15
#     tests from §5.2, then:
git -C ~/wt/doors push --force-with-lease=door-fixes:cde3111e7a32d5e4f0cc0dbfc78443da6144a1f3 origin door-fixes

# 5b. mpc_corr_v2_reverse: 12 own commits; expect 5 stops in MPC_corr.py (resolution notes in §5.3)
git -C ~/dev_ws/f1tenth_more_reverse rebase campaign-final
git log --oneline campaign-final..mpc_corr_v2_reverse | wc -l   # 12
#     tests from §5.3 (f1tenth_behavior on the Orin), then:
git -C ~/dev_ws/f1tenth_more_reverse push --force-with-lease=mpc_corr_v2_reverse:a14ec2f5780f8467e26eb9eb24cc0ed4eaf1ce43 origin mpc_corr_v2_reverse

# 5c. obstacles-v2: 5 own commits; expect 1 stop (79562b1, MPC_corr.py)
git -C ~/dev_ws/f1tenth_more_obst rebase --onto campaign-final 29dd3aa03f6e0f495c75f6b4ccd5a2a03f03d921 obstacles-v2
git log --oneline campaign-final..obstacles-v2 | wc -l          # 5
git -C ~/dev_ws/f1tenth_more_obst push --force-with-lease=obstacles-v2:d0028f2efbc8d021b74d732875e3027159ddd72c origin obstacles-v2
```

**5d (optional): take the two mission-flow commits out of reverse.**
- Do it instead of 5b.
- The interactive rebase is yours to run.
- Both cherry-picks are predicted to conflict only in
  `output/mpc_v2/reverse_and_memory.md`. Drop those hunks.
- The interactive rebase also carries the 5b conflicts. Later reverse commits
  may also stop in that report file where they build on the dropped commits'
  sections.

```bash
git worktree add -b mission-start-fixes ~/wt/mission-fixes campaign-final
git -C ~/wt/mission-fixes cherry-pick -x 9c24024 a14ec2f
git -C ~/dev_ws/f1tenth_more_reverse rebase -i campaign-final   # mark 9c24024 and a14ec2f as "drop"
git log --oneline campaign-final..mpc_corr_v2_reverse | wc -l     # 10
git -C ~/wt/mission-fixes push -u origin mission-start-fixes
git -C ~/dev_ws/f1tenth_more_reverse push --force-with-lease=mpc_corr_v2_reverse:a14ec2f5780f8467e26eb9eb24cc0ed4eaf1ce43 origin mpc_corr_v2_reverse
```

### Step 6: remove worktrees

Each is clean and its content is on a live branch.

```bash
for w in ~/wt/doors-core ~/wt/doors-node ~/wt/doors-replay ~/wt/doors-replay2 ~/wt/doors-tracker ~/wt/doors-unknown \
         ~/dev_ws/f1tenth_more_jzref ~/dev_ws/f1tenth_more_cpu_cleanup; do git worktree remove "$w"; done
git worktree remove --force ~/dev_ws/f1tenth_more_jcbase   # --force only because its submodules are initialised; verified clean
# another Claude session's scratchpad: confirm that session is finished first
git worktree remove /tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/1d54b94c-bc85-4ffd-8443-44aacfa3c6e1/scratchpad/ws/src_wt \
  || git worktree prune
# on the Orin: two stale, clean worktrees (submodules not initialised; add --force only if git says it found submodules)
ssh fabiocar@jetson-orin 'cd ~/dev_ws/f1tenth_more && git worktree remove ../f1tenth_jazzy && git worktree remove ../f1tenth_mpc_refactor'
```

- `_cpu_cleanup` is clean only after step 3d.
- These worktrees stay:
  - `_jazzy`: its `output/jazzy_align/` belongs to another session;
  - `_rust`, `_obst`, `_reverse`;
  - `~/wt/doors`, `~/wt/cf` and the main checkout.

### Step 7: delete branches

The tags from step 1 are already on GitHub.

```bash
# local
git branch -D tmp/door-fixes-core tmp/door-fixes-node tmp/door-fixes-replay tmp/door-fixes-replay2 tmp/door-fixes-tracker tmp/door-fixes-unknown
git merge-base --is-ancestor cpu-cleanup campaign-final && git branch -D cpu-cleanup
git merge-base --is-ancestor door-detection campaign-final && git branch -D door-detection
# GitHub
git push origin --delete cpu-cleanup door-detection
# Orin (campaign-final-doors only after step 4)
git push orin --delete campaign-final-doors orin-verify-campaign-dirty orin-snapshot-20261013 \
  refactor/mpc-corridor-split test-campaign test-logging jazzy llm scene-graph sick
git fetch --prune origin && git fetch --prune orin
```

- `tmp/door-fixes-*` need `-D`: they are patch-identical copies, not
  ancestors, so `-d` refuses.
- `cpu-cleanup` and `door-detection` need `-D` because they are not in HEAD
  (jazzy-campaign). The `merge-base` guard in front of each delete is the
  safety check.

### Step 8: verify

```bash
git branch -a --no-merged campaign-final   # car line: only door-fixes, mpc_corr_v2_reverse, obstacles-v2 (+ mission-start-fixes); plus the Jazzy refs
git worktree list
git ls-remote --refs origin | grep -c 'refs/tags/backup/.*-20261020$'   # 21
ssh fabiocar@jetson-orin 'cd ~/dev_ws/f1tenth_more && git status --short --branch | head -3'   # ## campaign-final
```

### Not in this plan (follow-ups)

- **`main`** (09-30) can later fast-forward to `campaign-final`.
- **Jazzy line:**
  - `jazzy-next` still needs `60f9af7` and the new base. Merge the new
    `campaign-final` into it.
  - `jazzy-rust` should then move onto `jazzy-next`. Its own 7 commits
    replace its older `0c1cd41` merge.
  - The `cce25d3` zed pin needs a decision (back to v5.4.1, or keep).
- **Open decisions:**
  - `archive/mpc-corridor-split`: port or drop;
  - `output/orin_check_report.md`: copy it onto campaign-final, or leave it in
    the tag.
- **Orin clutter:**
  - 5 old stashes;
  - `build_base573/`, `install_base573/`;
  - ~25 untracked `output/` folders;
  - the stale `gh/*` refs and the self-referencing `orin` remote.

---

## 7. Execution log

### Phase 1: safety (2026-10-20, done)

**Pre-check**
- Branch tips: all 36 refs matched the inventory after a fresh `git fetch origin`
  and `git fetch orin`. Worktree states matched too.
- Other sessions: one other Claude Code session, PID 2071275, cwd main
  checkout, started 11:36. It had no child processes, held no git lock, and
  had written no file in any worktree in the last 60 minutes.

**What was done**
- **cpu-cleanup** (§4c):
  - Moved the two run folders out of git to
    `~/f1tenth_data/replays/cpu_cleanup/` (`run_corridor10m_20261016T144537/`
    0.9 MB, `…_v2/` 20 MB; 16 files, byte counts checked).
  - Recorded the new location in `output/cpu_cleanup/report.md` and `log.md`.
  - Committed both files as **`e41512a`** on cpu-cleanup and pushed
    (`82c0afa..e41512a`). Upstream set to origin/cpu-cleanup.
  - This replaces plan step 3d's patch: phase 2 cherry-picks `e41512a`
    instead. cpu-cleanup is now 1 commit outside campaign-final until then.
- **Old-jazzy inventory:** moved `_jazzy/output/jazzy_align/` (9 files,
  0.8 MB, last written 10-19) to `~/f1tenth_data/archive/jazzy_align/`.
  Not committed, not deleted.
- **Tags:** created the 21 `backup/<branch>-20261020` tags.
  - `backup/cpu-cleanup-20261020` is at `e41512a`, the branch after the
    addendum commit, instead of the planned `82c0afa`.
  - Also created `archive/orin-verify-campaign-dirty` → `3ae2108`, alongside
    the other `archive/*` tags.
  - Pushed all of them, plus the 4 older local-only `backup/*` tags.
- **Pushes:**
  - fast-forwards: `jazzy-campaign` (`883a0f0..60f9af7`), cpu-cleanup;
  - new branches with `-u`: `door-fixes`, `obstacles-v2`, `jazzy-next`,
    `jazzy-rust`. obstacles-v2 now tracks origin/obstacles-v2.
  - The 11 Orin-only commits are on GitHub through
    `backup/orin/campaign-final-doors-20261020`. No new branch was created
    for them; phase 2 makes them `campaign-final`.

**Verification** (`git ls-remote origin`)
- All 11 local branches named on origin match exactly.
- All 12 `orin/*` tips, both detached worktree HEADs and the 6 `tmp/*` tips
  are reachable from origin refs.
- 39/39 tags are on origin with the same object.

**Still local-only**
- Branch names: the six `tmp/door-fixes-*`. Their commits are on origin
  through the backup tags.
- Uncommitted files:
  - main checkout: `docs/branches.md`, `output/branches/`,
    `src/husarion_gz_worlds/`, and the emptied zed submodule checkouts;
  - `~/wt/doors`: 1 line in `.claude/settings.local.json`;
  - `_rust`: `src/husarion_gz_worlds/`.
- On the Orin:
  - 5 stashes;
  - the untracked M02 folders (their content is `60f9af7`, now on origin);
  - untracked `output/…`, `build_base573/`, `install_base573/`.

### Before phase 2: Orin stashes and Orin output/ (2026-10-20, done)

**Stashes**
- Read-only listing (`git stash list`, `git stash show --stat`):

  | n | stash commit | date | on | files |
  |---|---|---|---|---|
  | 0 | `a46d10d` | 10-08 | scene-graph | `llm_planner_node.py` (3/3 lines) |
  | 1 | `1c6e59b` | 07-07 | vesc-tuner-test | `.claude/settings.json`, `track_mpc_opt_node.py` (has an untracked-files parent) |
  | 2 | `8fd79cb` | 06-15 | vesc-tuner-test | `stack_bringup_launch.py` |
  | 3 | `3bf62e0` | 06-15 | vesc-tuner-test | `vesc.yaml`, `stack_bringup_launch.py`, `andre_mpc_opt_node.py`, `track_mpc_opt_node.py` |
  | 4 | `4a2c2e0` | 06-12 | stereocamera-test | `stack_bringup_launch.py`, `llm_mpc_tuner_node.py`, `andre_mpc_opt_node.py` |

- On the Orin: `git tag archive/orin-stash-<n>-20261020 stash@{n}` for n = 0..4.
  Lightweight tags. No stash was dropped: still 5.
- Relayed to GitHub through linus: `git fetch orin 'refs/tags/archive/orin-stash-*:…'`,
  then a push to origin. `ls-remote origin` shows all 5 at the stash commits.

**Orin output/**
- Every untracked file (243) and every git-ignored file (62) under the Orin's
  `output/` was hashed with `git hash-object --no-filters`.
- Each hash was compared against all 12,194 blobs reachable from any ref on
  linus (all origin and orin branches and tags).
- **Not in git: 287 files, 161.6 MB** (225 untracked + 62 ignored). They were
  copied with rsync to `~/f1tenth_data/archive/orin_output_20261020/`
  (repo-relative paths), and all 287 hashes were re-checked on linus.
  - The bulk is `output/wall_v2/videos/` (101 MB of mp4) and
    `run_corridor10m_20261016T144537/` + `…20261019T131815/` (38 MB).
  - The rest is `output/cpu_test/` (108 files), `replay_*`, `merge_cleanup/`,
    `self_mask_proposal/`, `live/`, `nightly/`, `glass_lidar/`, and a few loose files.
- **Already in git: 18 untracked files** (small `*.info.txt` / `*.replay.md`).
  Not copied.
- The folder holds `MANIFEST.tsv` (all 305 files: status, kind, blob, bytes,
  path) and a README.
- Nothing on the Orin was changed. `build_base573/` and `install_base573/`
  were left alone.

### Phase 2, linus/GitHub side (2026-10-20, done; the Orin side waits for "car idle")

**Pre-check**
- Tips unchanged after a fresh fetch.
- PID 2071275 still idle: no children, no lock, no file written.

**What was done**
- **Fast-forward:** `git push origin 6c693ac:refs/heads/campaign-final`
  (`868ec8a..6c693ac`, no force), then `git fetch origin campaign-final:campaign-final`.
- **Worktree:** `~/wt/cf` on campaign-final.
- **Cherry-picks** with `-x`:
  - `60f9af7` → **`f2d36a2`** (M02 P002-R012..R020 + `tools/test_campaign_video.py`, 129 files);
  - `e41512a` → **`cd180d8`** (`output/cpu_cleanup` replay v2 notes, 2 files).
  - Both applied without conflict.
- **Push:** `campaign-final` to origin (`6c693ac..cd180d8`). **Not pushed to the Orin.**

**Verification**
- `ls-remote origin refs/heads/campaign-final` = `cd180d8`.
- The base contains 868ec8a, b08675e, 6c693ac, c873247 (door-detection) and
  82c0afa (cpu-cleanup).
- `6c693ac..cd180d8` changes exactly 131 files: the 129 + 2 of the two
  sources, blob-identical.
- `git cherry` shows both sources as present.
- Against the new base:
  - campaign-final is 2 ahead of `orin/campaign-final-doors` and 13 ahead of
    `orin/campaign-final`, so both Orin branches can fast-forward.
  - own commits: door-fixes 15, mpc_corr_v2_reverse 12, obstacles-v2 5;
    door-detection 0;
  - cpu-cleanup 1, patch-identical to `cd180d8`.

**Still to do (Orin side, waits for "car idle")**
- `git push orin campaign-final`.
- Then plan step 4: move the colliding untracked M02 folders aside, switch
  to campaign-final, diff-verify.

### Phase 2, Orin side (2026-10-20 13:2x, done on "car idle")

The instruction "the Orin will pull from GitHub itself, skip all Orin steps" arrived after
these steps had already run.

**Pre-check**
- HEAD was `campaign-final-doors` 6c693ac. No ROS or colcon processes were
  running, there was no git lock, and no file had been written in the
  checkout since 10:00.
- The untracked files outside `output/` (`pr.json`, `test_conditional_birth.py`,
  `tools/wall_variant_videos.py`, `tools/wall_variants/`) date from 10-12 and
  10-19. They had been hidden by the 40-line cut in the inventory's listing;
  nothing moved.

**What was done**
- **Push:** `git push orin cd180d8:refs/heads/campaign-final` (`b08675e..cd180d8`,
  fast-forward; the branch was not checked out there).
- **On the Orin:**
  - the 9 M02 folders P002-R012..R020 → `~/m02_untracked_20261020/`;
  - `results.csv` copied there (`cp -p`), then `git restore first_test_campaign/results.csv`;
  - `git switch campaign-final`.

**Verification**
- `diff -r` of every moved folder against the checkout shows only
  "Only in …: corridor_plot.png", 9 times. Those plots were generated on linus.
- `results.csv` is identical.
- Orin HEAD = `campaign-final` cd180d8.
  - Status: `M .claude/settings.json`, vesc `?`, the same untracked files as before.
  - It reports "ahead 116" against its stale view of origin.
- **No rebuild needed.** `6c693ac..cd180d8` touches only `first_test_campaign/`
  (128 files), `tools/test_campaign_video.py` and `output/cpu_cleanup/*.md`.
  None of these is in a colcon package. No `setup.py`, `CMakeLists.txt` or
  launch file installs or launches them; `test_campaign_logger.launch.py`
  only names the folder in a comment.

**Found on the Orin, not in git, not copied yet** (outside `output/`)
- `first_test_campaing/` (old misspelled folder, 59 MB): M02 P002-R001..R008
  of 2026-10-13/14.
  - 426 of its 442 untracked files are on no ref, plus 43 ignored files.
  - linus holds only the raw rows, in `~/f1tenth_archive/mission_row/2026-10-1[34]*`.
- `src/f1tenth_perception/test/test_conditional_birth.py`,
  `tools/wall_variant_videos.py` and `tools/wall_variants/*.yaml` (4): wall v2
  variant work of 10-19.

### Before phase 3

- **husarion_gz_worlds:**
  `git --no-optional-locks -C src/husarion_gz_worlds diff > ~/f1tenth_data/archive/husarion_gz_worlds_gz-sim_20261020.patch`
  (4 files, 101 lines).
  - `git apply --check` passes on the clean 87271ef copy in `_rust`.
  - `apply --check -R` passes on the edited copy.
  - Base commit 87271ef83672d9c9c96cb014ac64d746cacfd2c4 is recorded in
    `~/f1tenth_data/archive/README.md`.
  - Not committed.

### Phase 3a: door-fixes onto the new base (2026-10-20, done)

**Pre-check**
- PID 2071275 is `…/anthropic.claude-code-2.1.301-linux-x64/resources/native-binary/claude --output-format stream-json …`,
  started Tue Oct 20 11:36:04, cwd the main checkout. No children, no open
  file under `~/wt/doors`; no process at all has its cwd or a file there.
- `~/wt/doors` was clean apart from the `.claude/settings.local.json` line,
  with no file written in the last 2 h.
- `backup/door-fixes-20261020` on origin peels to `cde3111`.

**Rebase**
- `git -C ~/wt/doors rebase --autostash --onto campaign-final c873247 door-fixes`:
  15/15 commits applied without conflict, and the autostash was re-applied.
- `git range-diff` gives `=` for all 15. The net diff equals the old one
  apart from hunk line numbers.

**Tests** (pytest in the `~/wt/doors` Jazzy `--symlink-install` env,
`ROS_DOMAIN_ID=147`, LOCALHOST)

| Suite | Before (`cde3111`) | After (`486a157`) |
|---|---|---|
| f1tenth_perception | 1228 pass | 1228 pass |
| f1tenth_costmap | 233 pass | 265 pass (+32 = the base's `test_costmap_component_filter.py`) |
| corridor_perception | 55 pass, 40 skip (no C lib on linus) | same |
| f1tenth_params | 54 pass | 54 pass |
| `tools/test_door_replay.py` + `test_corridor_wall_replay_helpers.py` | 27 pass, 5 skip | same |
| mpc_controller `test_wall_boundaries.py` + `test_boundary_constraints.py` | – | 92 pass (base tests; covers the role rename) |

The test-id sets are identical before and after, except for the 32 added
costmap tests. No status changed.

**Push**
- `campaign-final`: "Everything up-to-date" (`cd180d8`).
- `git push --force-with-lease=door-fixes:cde3111 origin door-fixes`: `cde3111...486a157`.
- `ls-remote`: campaign-final `cd180d821869c38aefa271cf7da01bf4de473639`,
  door-fixes `486a15773f5d1f0aeb3b328eef7ad93ff394a5ac`.
