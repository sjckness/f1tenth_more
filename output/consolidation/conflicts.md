# Consolidation conflicts

Branch `main-consolidation`, from `campaign-final` fa5471f, 2026-10-22.

**Open non-trivial merge conflicts: none.** No merge stopped. The four textual
conflicts were trivial: in each, both sides added an adjacent entry. They are
logged below.

Section 2 lists the places where two of your requirements, or a requirement
and the code, point different ways. They are not merge conflicts, and nothing
was changed for them. Each one needs a decision from you.

---

## 1. Resolved (trivial)

### 1.1 `comms-base` into main-consolidation (merge `e7c257c`)

`comms-base` forked from `pwa-control` 9c5371d (2026-10-20). That is 51
commits behind `campaign-final`. Both sides added entries at the same place.
In every case both were kept.

| File | Ours (main-consolidation) | Theirs (comms-base) | Resolution |
|---|---|---|---|
| `src/f1tenth_messages/CMakeLists.txt` | `"msg/MpcParamUpdate.msg"` (two-llm-arch) | `"msg/LinkStatus.msg"` | Both, adjacent in the `rosidl_generate_interfaces` list |
| `src/f1tenth_gateway/f1tenth_gateway/config.py` | `topics['llm2_status']`, and a new `'llm2'` block after `topics` | `topics['link_status']` | Both topic entries, then the `'llm2'` block unchanged |
| `src/f1tenth_gateway/f1tenth_gateway/app.py` (status payload) | campaign-final's `'cpu_percent': cpu if cpu is not None else …`, `'cpu_temp_c'`, `'system': own` | the base's old `cpu_percent`/`cpu_temp_c` line, plus `'link': latest.get('link')` | campaign-final's cpu/system fields, plus comms' `'link'` field. Theirs had the cpu line unchanged from the base, so taking ours loses nothing. |
| `src/f1tenth_gateway/test/test_ros_bridge.py` | an assertion on `teleop._measured`, plus `test_llm2_pause_through_a_fake_set_active_and_the_status_after_it` | `test_link_status_reaches_the_status_payload` | Ours unchanged, then theirs |

Auto-merged without conflict, and checked by reading the merged result:
- `stack_bringup.launch.py`: both branches added the same `IfCondition` import. The `enable_comms` block and the `enable_llm2` block are each present once.
- `component_supervisor_node.py`, `supervisor_bringup.launch.py`, `components.yaml`, `stack_params.yaml`, `ros_bridge.py`.

`git rerere` recorded the four resolutions.

### 1.2 `two-llm-arch` and `campaign-final-yolo-gpu`

Both fork at the campaign-final tip fa5471f, so neither had a conflict.

---

## 2. Decisions needed (not merge conflicts; nothing was changed)

### 2.1 "Only the 2-QP reverse MPC": campaign-final also carries two other MPC implementations

What you asked for: MPC = ONLY the reverse manoeuvre with two QPs (forward
solver + rear solver, `reverse_manoeuvre.py`, mpc_corr_v2_reverse lineage),
with every other MPC implementation left out.

What the base already contains: campaign-final itself holds two other
implementations. No branch merge brings them in.

| Code | What it is | State on campaign-final / main-consolidation |
|---|---|---|
| `mpc_controller/reverse_mpc.py` (`ReversePlanner`, `DirectionSelector`) | Reverse as the MPC's own choice: per-tick direction candidates (all-forward / all-reverse / reverse-then-forward QPs), scored | **Off**: `allow_reverse: false`. `MPC_corr.py` refuses to start with both `allow_reverse` and `reverse_auto_enabled` on (f884400). |
| `mpc_controller/andre_mpc_node.py` + `f1tenth_control/launch/mpc.launch.py` | The older rollout MPC node (console script `andre_mpc_node`) | **Not launched** by any bringup. `navigation.launch.py` includes only `mpc_corr.launch.py`. |
| `mpc_controller/reverse_manoeuvre.py` | **The 2-QP reverse manoeuvre** you want | **On**: `reverse_auto_enabled: true`, `bt_reverse_integration: true` |

Kept out (other MPC lines, not merged):
- `origin/jazzy` a596962 (James, 10-21) "mpc_corr: added negative speeds if reverse command". Jazzy line.
- `orin/refactor/mpc-corridor-split`. Tag `archive/mpc-corridor-split`.

**Proposal:** leave both in the code during the freeze. Removing them is a
refactor, and `reverse_mpc.py` is woven into `MPC_corr.py` and its tests
(`test_reverse_flag_off_identity.py` pins the flag-off path). As the first
debug-phase item on main, delete `andre_mpc_node`/`mpc.launch.py` and
`reverse_mpc.py` together with the `allow_reverse` keys. Until then, the only
thing that turns the second reverse mechanism on is setting
`allow_reverse: true`, and the start guard refuses that combination.

### 2.2 The Orin is not running clean campaign-final

Read over ssh at 16:2x today, read-only. The supervisor stack was up, started
16:28.

- **HEAD:** `stack-campaign-final-2026-10-22` = fa5471f, i.e. campaign-final.
- **The checkout is not fa5471f.** 32 files under `src/` are staged with the
  content of the Orin's old `campaign-final` **1ee82c86**, which is also the
  base of `orin-wip-2026-10-22`. Compared file by file, 0 of the 32 differ
  from 1ee82c86. Among them:
  - deleted: `mission/wall_lock.py` and its two tests, `mpc_controller/ramp_wall_source.py`, `f1tenth_logger/mission_overlay.py`, `test_wall_confirm_level.py`;
  - reverted to the older version: `MPC_corr.py`, `reverse_manoeuvre.py`, `behavior_executor_node.py`, `front_clearance_node.py`, `lidar_front_wall_node.py`, `multi_wall_tracker.py`, `yolo_detector_node.py`, `stack_params.yaml`.
- **What the car drives** (symlink-install, built 13:54 and 16:17) is
  therefore fa5471f with the 2026-10-21/22 night work on those files
  reverted: front-wall lock, wall confirmation level, ramp wall source and
  mission overlay.
- That state is in no commit.
- Nothing was touched.

Consequences for this consolidation:
- main-consolidation is built on fa5471f as asked. So it contains the front-wall
  lock and the wall confirmation level, which are both on your must-have list,
  **although the car is not running them today**.
- "What runs on the Orin" and "campaign-final" are two different trees.
- Before the Orin builds main, decide whether the reverted 32 files are a
  deliberate campaign configuration that main should take, or a temporary
  local state.

**Proposal:** capture the Orin state in a commit on its own branch on the Orin,
when the car is idle, so that it exists somewhere:
```bash
git switch -c orin-running-20261022
git commit -m 'Orin running state 2026-10-22: fa5471f with 32 src files at 1ee82c86'
```
Leave campaign-final untouched. Only after that, decide per feature.

### 2.3 Defaults that change behaviour vs campaign-final

| Change | Source | Why it is here |
|---|---|---|
| `enable_intelligence` true → false | your instruction | You asked for it explicitly; see report.md §3 |
| `yolo_gpu_pipeline` (new) = true | the YOLO GPU branch, as is | The branch's own default. Not changed. |
| `yolo_max_rate_hz` (new) = 15.0 | the YOLO GPU branch, as is | The branch's own default. Not changed. |
| `yolo_cpu_threads` (new) = 2 | the YOLO GPU branch, as is | The branch's own default. Not changed. |
| `yolo_cuda_blocking_sync` (new) = true | the YOLO GPU branch, as is | The branch's own default. Not changed. |

The YOLO ones are the branch's defaults, merged "as is". Leaving the YOLO
behaviour as it was on campaign-final would have meant flipping four keys on
main, which would be choosing a behaviour. **Decide whether main keeps them.**
The one-line revert of each is in report.md §3.
