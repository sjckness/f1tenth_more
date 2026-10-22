# Consolidation report: one `main` (2026-10-22)

Worktree `~/dev_ws/f1tenth_more_consolidation` on linus, branch
`main-consolidation`, starting from `campaign-final` fa5471f.

Nothing was pushed. No branch was deleted or moved, other than creating
`main-consolidation`. `campaign-final` and `main` are untouched. Nothing was
built and no node was run.

Companion files in this folder:

| File | What it holds |
|---|---|
| `branches.md` | The inventory: every branch and its verdict |
| `conflicts.md` | Conflicts and the decisions you need to make |
| `known_bugs.md` | The debug backlog |
| `static_checks.py` | The checks that ran after each merge |
| `archive_tags.sh` | §5 |
| `orin_main.sh` | §8 |

## 1. What is in main-consolidation

```
(top)    output/consolidation: inventory, conflicts, bug backlog, report, scripts
9bed256  main-consolidation: LLM planner off by default (enable_intelligence false)
e7c257c  Merge comms-base           (4 trivial conflicts, resolved)
514fa56  Merge campaign-final-yolo-gpu
75fa55d  Merge two-llm-arch
fa5471f  campaign-final  (= main = origin/main today)
```

**From campaign-final, unchanged.** Everything already on fa5471f:
- doors (off by default);
- multi-wall tracker;
- offset-free speed law;
- front-wall lock, with the MPC ramp on the locked wall;
- wall confirmation level (`wall_confirm_*`);
- cpu_cleanup;
- obstacles v2;
- **the 2-QP reverse manoeuvre** (forward + rear solver, `reverse_manoeuvre.py`), which counts the reverse in odom;
- the app: pwa-control, which includes telegram-control.

**Merged:**

| Branch | Content | Default state |
|---|---|---|
| `two-llm-arch` | LLM2 MPC-parameter node, gateway/PWA toggle | off |
| `campaign-final-yolo-gpu` | YOLO GPU pipeline and CPU-cost work | **on** (see §3) |
| `comms-base` | `f1tenth_comms` | off |

**One requested change:** the LLM planner's server is off by default (§3, §4).

Net change against campaign-final: 103 files, +12756 / −108. The
documentation commit is on top of that.

## 2. What stayed out, and why

| Branch(es) | Why |
|---|---|
| `jazzy` (L eb78d6b / O a596962), `jazzy-campaign`, `jazzy-next`, `jazzy-rust`, `orin/jazzy` | The Jazzy line, by your rule. `origin/jazzy` also carries James's a596962, a different reverse MPC (negative `vdes` through the forward solver) without the v ≥ 0 guard. |
| `orin/refactor/mpc-corridor-split` | Another MPC line, older than every reverse and wall change. Already tag `archive/mpc-corridor-split`. |
| `orin/orin-wip-2026-10-22` (Orin only) | A save-work snapshot of untracked files: 10-20 M02 runs, old misspelled M02 runs, output/, wall-v2 variant experiments. Data, not stack code; campaign data belongs to the frozen campaign-final. **Archive it**: it is the only copy of the 10-20 M02 runs outside the Orin. |
| `orin/docs/branch-audit` (Orin only) | A 10-20 migration plan, superseded |
| `tmp/door-fixes-*` ×6, `cpu-cleanup` | Already in by patch |
| `orin/orin-snapshot-20261013`, `orin/orin-verify-campaign-dirty`, `orin/test-campaign`, `orin/llm`/`scene-graph`/`sick`/`test-logging` | Obsolete. Already archived or contained. |
| vesc `humble-jetson`, `vesc-tuner-test` | An older `vesc_tuning`, superseded. The pinned vesc stays 0e74a60. |

**Reverse MPC.** The only MPC that drives on main is mpc_corr with the 2-QP
reverse manoeuvre (`reverse_auto_enabled: true`). Two other implementations
were already in campaign-final, and they stay in the code but inactive:
- `reverse_mpc.py`, the direction-candidate reverse: `allow_reverse: false`, and a start guard refuses both on;
- `andre_mpc_node`: never launched.

Removing them is a refactor, so it waits for your decision (conflicts.md §2.1).

## 3. Defaults that differ from campaign-final

Complete list, from a key-by-key diff of `stack_params.yaml`. No key was
removed.

| Key | campaign-final | main-consolidation | Why | Revert line (stack_params.yaml) |
|---|---|---|---|---|
| `enable_intelligence` | `true` | **`false`** | **Your instruction**: the LLM planner off by default | `default: true` |
| `yolo_gpu_pipeline` | (classic CPU path) | **`true`** | YOLO GPU branch, as is | `default: false` |
| `yolo_max_rate_hz` | (every frame, about 30 Hz) | **`15.0`** | YOLO GPU branch, as is | `default: 0.0` |
| `yolo_cpu_threads` | (library default, one per core) | **`2`** | YOLO GPU branch, as is | `default: 0` |
| `yolo_cuda_blocking_sync` | (CUDA spin) | **`true`** | YOLO GPU branch, as is | `default: false` |
| `foxglove_topic_whitelist` | 16 entries | + `/comms/{coverage_rssi,coverage_rtt,coverage_cells,link_status}` | comms-base; those topics exist only with `enable_comms` | drop the 4 entries |

The other new keys add a feature that is off:
- `yolo_backend: pt` (same behaviour as before) and `yolo_engine_path: ''`;
- `enable_llm2`, `llm2_params_enabled`, `llm2_model`, `llm2_port`, `llm2_param_*`;
- `enable_comms`.

Outside stack_params:
- `component_supervisor_node`'s fallback literal for a bare `ros2 run`
  follows `enable_intelligence` (True → False).
- `f1tenth_comms/config/comms.yaml` ships `reconnect_enabled: true` and
  `reconnect_dry_run: false`. This only acts when comms is enabled (H3 in
  known_bugs.md).

**The YOLO defaults are a behaviour change, merged as the branch has them.**
They halve the detection rate and move pre- and post-processing to the GPU.
The branch validated this on linus only. Keeping or reverting them is your
call (conflicts.md §2.3).

## 4. Switches: LLM planner, LLM2, doors, comms

Each feature is now behind the same pattern:
- a key in `src/f1tenth_params/config/stack_params.yaml` that defaults to `false`;
- read when the stack launches;
- overridable by a launch argument.

The workspace is built with `--symlink-install`, so an edit to the yaml takes
effect at the next launch of that component without a rebuild.

| Feature | Switch | Turn it on |
|---|---|---|
| **LLM planner (LLM1)**: llama-server for `llm_planner_node`, which the CLI, f1tenth_ctl, Telegram and the PWA "plan" use | `enable_intelligence` | `ros2 launch f1tenth_bringup supervisor_bringup.launch.py enable_intelligence:=true`, or on a running stack `./scripts/stackctl.py start intelligence`, or `enable_intelligence: default: true` in the yaml. The planner node itself is still started by hand or by the app. |
| **LLM2**: MPC-parameter LLM, a second llama-server on 8084 | `enable_llm2` (producer) **and** `llm2_params_enabled` (mpc_corr consumer) | `ros2 launch f1tenth_bringup stack_bringup.launch.py enable_llm2:=true llm2_params_enabled:=true`. **`supervisor_bringup` has no `enable_llm2`.** On the supervisor path: `ros2 launch llm llm2.launch.py`, plus `llm2_params_enabled: 'true'` in the `navigation` entry of `components.yaml`, then restart `navigation` (known_bugs.md F3). |
| **Door detection** | `lidar_front_wall_door_detection_enabled` (already false on campaign-final since fb913d1) | `lidar_front_wall_door_detection_enabled: default: true` in the yaml, then `./scripts/stackctl.py restart lidar_front_wall`. Standalone: `ros2 launch f1tenth_perception lidar_front_wall.launch.py lidar_front_wall_door_detection_enabled:=true`. The supervisor does not forward CLI arguments to this component, so on the car the yaml is the switch. |
| comms (link_manager, coverage) | `enable_comms` | `supervisor_bringup.launch.py enable_comms:=true`, or `stackctl.py start comms`. Needs the polkit rule (§7) for reconnects. |

## 5. Archive tags (prepared, not run)

`output/consolidation/archive_tags.sh`. Run it on linus under bash, from
`~/dev_ws/f1tenth_more`.

What it does:
- Tags 27 branches as `archive/<name>`:
  - the 3 merged branches;
  - the 11 feature branches already in campaign-final;
  - the 6 `tmp/door-fixes-*` branches;
  - the Orin-only `campaign-final-doors`, `orin-wip-2026-10-22`,
    `docs/branch-audit`, `test-logging`, `llm`, `scene-graph` and `sick`.
- Each tag is annotated and pinned to the SHA recorded today. A branch that
  has moved since is skipped and reported, never tagged at its new tip.
- `PUSH=1` also pushes the tags. Note that `archive/orin-wip-2026-10-22` alone
  uploads about 180 MB; the script shows how to leave it out.
- Commands for archive tags on the vesc fork's two old branches are included
  as comments.

These already exist on GitHub and are not repeated:
- `archive/orin-snapshot-20261013`;
- `archive/orin-verify-campaign-dirty`;
- `archive/mpc-corridor-split`;
- `archive/test-campaign`.

The script does not delete any branch.

## 6. Static checks

`python3 -I output/consolidation/static_checks.py` (no ROS, no build). It ran
on campaign-final and after every merge.

What it checks:
- **conflict markers**;
- **Python syntax** of every tracked `.py`:
  - compiled and parsed with `feature_version=(3, 10)`;
  - **plus the PEP 701 f-string forms**. Python 3.12 on linus accepts them, but
    Humble's 3.10 on the Orin rejects them;
- `generate_launch_description()` present in every `*.launch.py`;
- **YAML** parses, with duplicate keys reported;
- JSON parses;
- **XML** parses (`package.xml` and the rest; this also catches `--` inside a comment);
- every `get_default/get_value('<key>')` names a key in `stack_params.yaml`;
- every `console_scripts` target exists;
- every rosidl-listed `.msg`/`.srv` exists.

| State | Result |
|---|---|
| campaign-final fa5471f | 1 FAIL, pre-existing: `model_log` console script has no `main` (known_bugs.md B8) |
| + two-llm-arch | same 1 FAIL, nothing new |
| + campaign-final-yolo-gpu | same 1 FAIL, nothing new |
| + comms-base (after resolution) | same 1 FAIL, nothing new |
| + enable_intelligence default | same 1 FAIL, nothing new |
| final | see the end of this file |

Tests run without ROS:
- **YOLO GPU** (`test_yolo_gpu.py`, `test_stage_timing.py`): **11 passed, 2
  skipped**. The skips are for torch and ultralytics, which linus does not
  have.
- `test_yolo_detector_node.py` imports rclpy, cv_bridge and vision_msgs, so it
  **was not run**.
- Gateway smoke (`test_app_mock.py`, `test_llm2.py`): 2 passed, 36 skipped (no
  fastapi stack on linus). Indicative only.

## 7. Needs sudo (none of it was done)

Only needed when you enable the feature in question.

1. **Orin, only if comms is used**: the polkit rule that lets the stack's user
   run `nmcli connection up` over ssh. Without it every reconnect fails with
   "Not authorized" (`output/comms/setup.md`).
   `sudo bash scripts/orin/comms_network.sh polkit`
2. **linus, only if linus becomes the comms base**: the hotspot and heartbeat
   setup, `scripts/base/hotspot.sh` / `install_heartbeat.sh`, as described in
   `output/comms/setup.md`. Some steps need sudo.
3. **Nothing for the build.** The merges add no apt dependency: `f1tenth_comms`
   needs only `rclpy`, `std_msgs`, `std_srvs`, `nav_msgs` and `tf2_ros`; `llm`
   adds `std_srvs` and `rcl_interfaces`. All are already on the Orin.

## 8. Commands

### 8.1 Fast-forward `main` and push it (after your approval; on linus, under bash)

```bash
cd ~/dev_ws/f1tenth_more            # nothing is checked out on main anywhere on linus
git fetch origin
MC=$(git rev-parse main-consolidation); echo "approving $MC"
# STOP unless both print OK:
[ "$(git rev-parse origin/main)" = fa5471f3ea4b2e62a8cadc0819f6b280e872338b ] && echo OK-origin-main-unmoved
git merge-base --is-ancestor origin/main main-consolidation && echo OK-fast-forward
git fetch . main-consolidation:main      # local main, fast-forward only
git push origin main                     # fast-forward on GitHub, no --force
git push -u origin main-consolidation    # optional: the branch itself as a backup
```

The first check makes sure nobody has moved `origin/main` since today.

### 8.2 Build `main` on the Orin in its own worktree

Use `output/consolidation/orin_main.sh`. It keeps the campaign checkout
`~/dev_ws/f1tenth_more` and its `build/ install/ log/` untouched, and checks
that by fingerprint before and after each step. The car keeps running the
campaign code.

```bash
scp output/consolidation/orin_main.sh fabiocar@jetson-orin:~/orin_main.sh
ssh fabiocar@jetson-orin

# git only; safe while the car runs. <sha> = the main you pushed in 8.1
bash ~/orin_main.sh setup <full sha of main>

# when the car is idle: refuses while a stack runs (the build takes every core)
bash ~/orin_main.sh build
```

What each step does:
- **`setup`**:
  - fetches origin;
  - checks that `origin/main` is the approved sha;
  - fast-forwards the Orin's local `main`, which is not checked out there;
  - `git worktree add ~/dev_ws/f1tenth_more_main main`;
  - `submodule update --init --recursive` inside the new worktree. That needs
    GitHub access. Its submodule repos go under
    `.git/worktrees/f1tenth_more_main/`, not the campaign's.
- **`build`**:
  - runs from an **empty environment** (`env -i`), because `~/.bashrc` line
    119 sources the campaign install;
  - sources only `/opt/ros/humble/setup.bash`, and refuses if a campaign
    path leaked in;
  - runs `colcon --log-base log build --symlink-install --build-base build --install-base install`
    inside `~/dev_ws/f1tenth_more_main`;
  - then runs CLAUDE.md's `ldd -r` undefined-symbol scan on main's install.

### 8.3 Sourcing and launching each, without mixing them up

```bash
# CAMPAIGN (what the car runs today). A plain login shell already is campaign
# (~/.bashrc), but this one says so:
bash ~/orin_main.sh shell campaign     # green [CAMPAIGN …] prompt
ros2 launch f1tenth_bringup supervisor_bringup.launch.py

# MAIN
bash ~/orin_main.sh shell main         # magenta [MAIN …] prompt
ros2 launch f1tenth_bringup supervisor_bringup.launch.py
#   + enable_intelligence:=true   if you want the LLM planner (off on main)

# which one is running right now?
bash ~/orin_main.sh which              # MAIN / CAMPAIGN per stack process
```

What the `shell` command does:
- starts a clean interactive shell with **only** that workspace sourced;
- sets the Orin's runtime variables from `~/.bashrc`:
  `ROS_DISCOVERY_SERVER=127.0.0.1:11811`, `ROS_SUPER_CLIENT=TRUE`, and the pip
  CUDA `LD_LIBRARY_PATH` that torch needs;
- `cd`s into the workspace;
- prints `ros2 pkg prefix f1tenth_bringup`, with `!! WRONG WORKSPACE SOURCED` if
  that path is not under the workspace;
- warns if a stack is already up.

Rules that keep them apart:
- **Never run both stacks at once.** They share the Discovery Server port
  11811, the VESC, the LiDAR and the camera. Check with `which` and stop one
  before starting the other.
- **The app does not follow you.** f1tenth_ctl, and through it Telegram and the
  PWA gateway, always sources `~/dev_ws/f1tenth_more/install/setup.bash`: it is
  hard-coded in `ctl.env` (known_bugs.md E6). So anything started from the app
  is the **campaign** stack. To drive main through the app, give f1tenth_ctl a
  copy of `ctl.env` that sources the main install, through `F1T_CTL_ENV`.
- The gateway unit runs from `~/dev_ws/f1tenth_more_two_llm`
  (`f1tenth-gateway.service.d/two-llm.conf`); nothing here changes it.
- **Shared on purpose:**
  - `~/f1tenth_archive`, the mission logger, so runs from both land in one
    archive;
  - `~/.ros/log`.
- **Not shared:** the test-campaign logger writes into the workspace it runs
  from, `<workspace>/first_test_campaign/`.

## 9. Open conflicts

**No open merge conflict.** Three decisions are waiting for you
(conflicts.md §2):
1. Remove the two inactive MPC implementations or keep them (§2.1).
2. **The Orin does not run clean campaign-final.** It runs fa5471f with 32
   files staged back to 1ee82c86. That means the front-wall lock, the wall
   confirmation level, the ramp on the locked wall and the reverse odom count
   are **not** on the car. That state is in no commit (§2.2).
3. Keep the YOLO GPU defaults or revert them on main (§2.3).

## Final static check

```
static_checks: HEAD 9bed2561, 2239 tracked files (590 .py, 35 .yaml/.yml), base campaign-final
note: [params] src/f1tenth_control/mpc_controller/test/test_corridor_width.py:96: 'corr_half_width_end_m' is not a key of stack_params.yaml (test file)
FAIL [entry] src/f1tenth_control/mpc_controller/setup.py: model_log -> mpc_controller.model_log:main: main not defined in src/f1tenth_control/mpc_controller/mpc_controller/model_log.py
result: 1 FAIL
```
