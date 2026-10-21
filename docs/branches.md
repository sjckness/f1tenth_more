# Branches

State as of 2026-10-14 (after `git fetch --prune`). It lists every branch on
`origin` and on this machine (linus), what each one is for, and what should
happen to it during the cleanup. Nothing has been deleted yet: the
**Cleanup** column is a proposal.

## How they relate

Every branch starts at `main` (`e47e646`, 2026-09-30, ROS 2 Humble).
`test-logging` adds a single commit to it, `573f313` ("m01 with obstacle
done"). That commit is the common base of all the campaign work.

```
main e47e646  (Humble)            = llm = scene-graph = sick
 └─ test-logging 573f313          (M01 campaign data)
     ├─ campaign-final            Humble, campaign rebuilt cleanly on 573f313  ← current car work
     ├─ campaign-dirty-evidence   573f313 + the car's dirty stack_params.yaml
     ├─ refactor/mpc-corridor-split   1d00853 + 3 unmerged mpc_controller commits
     ├─ test-campaign             M02 campaign fixes (Part A), Humble
     │   └─ orin-snapshot-20261013    Orin working tree + orin_check_report.md
     └─ jazzy                     ROS 2 Jazzy port, test-campaign merged in (6af1ddf)
         └─ snapshot/pre-campaign-final-20261012   jazzy 49de45d + 1 snapshot commit
```

## Summary

| Branch | Tip (origin) | Last commit | Ahead of main | What it is | Cleanup |
|---|---|---|---|---|---|
| `main` | `e47e646` | 2026-09-30 | – | Humble baseline, last merged state | **Keep.** It is stale; campaign-final should land here |
| `campaign-final` | `3ffe245` | 2026-10-13 | 25 | Campaign rebuilt on 573f313 (Parts B–F, calibration, measured sensor frames, scene labels). What the Orin builds | **Keep.** Active |
| `jazzy` | `3dbe37f` | 2026-10-13 | 160 | ROS 2 Jazzy port (Ubuntu 24.04, sim, Thor and Orin Jazzy docker) | **Keep.** Active, long-lived |
| `orin-snapshot-20261013` | `8c9e29b` | 2026-10-13 | 32 | The Orin's working tree on 2026-10-13 plus `output/orin_check_report.md` | Keep until the report has been copied onto campaign-final, then delete |
| `test-campaign` | `349c881` | 2026-10-12 | 29 | M02 campaign fixes (Part A, review items, B8). Superseded by campaign-final and merged into jazzy | Delete (after the `_tc` worktree goes) |
| `test-logging` | `573f313` | 2026-09-30 | 1 | M01 campaign data. The base of all campaign work | Merge into main (fast-forward), then delete |
| `snapshot/pre-campaign-final-20261012` | `8f345dd` | 2026-10-12 | 153 | Safety snapshot of jazzy's dirty tree taken before campaign-final started | Keep as a **tag**, delete the branch |
| `campaign-dirty-evidence` | `7e5910e` | 2026-10-12 | 2 | Evidence: `stack_params.yaml` as it was on the car during P004-R012..R019 | Keep as a **tag**, delete the branch |
| `refactor/mpc-corridor-split` | `a3ea50d` | 2026-10-09 | 5 | mpc_controller refactor: corridor moved into its own module, `andre_mpc_node` dropped, golden-replay test | **Decide.** 3 commits that are not on any other branch |
| `llm` | `e47e646` | 2026-09-30 | 0 | Old feature branch, identical to main | Delete from origin |
| `scene-graph` | `e47e646` | 2026-09-30 | 0 | Old feature branch, identical to main | Delete from origin |
| `sick` | `e47e646` | 2026-09-30 | 0 | Old feature branch, identical to main | Delete from origin |
| `sync/m02-into-jazzy` (local only) | `49de45d` | 2026-10-12 | – | Temporary branch for merging M02 into jazzy (`_sync` worktree) | Delete with its worktree. Its work is in jazzy |

## Each branch

### `main`
The Humble workspace for the physical car, as described in the README. It
has not moved since 2026-09-30 and every other branch is ahead of it. There
is no local `main` on linus. **Purpose:** the last stable integration point.
**Open point:** once the campaign finishes, fast-forward or merge
`campaign-final` into it so main reflects what actually drives the car.

### `campaign-final`
The campaign done again cleanly on top of `573f313`, without the
`test-campaign` history:
- Part D: trapezoid corridor;
- Part B: record from LOADED with a 15 s pre-roll;
- Part E: `base_front` frame;
- Part C: debug image and MP4 renderer;
- Part F: MPC step 0.2 s, N = 20.

On 2026-10-13 the Orin added:
- the campaign calibration, LLM wall-guard default and bring-up;
- measured sensor positions (wheelbase 0.305) and the beside-the-car clearance fix;
- the ZED 2i launch;
- scene labelling and the virtual wall.

**Used for:** what the Orin (Humble) builds and drives. The rule is to push
after every part and never force-push.
**Note:** local `campaign-final` is 16 commits behind origin. Pull before
working on it.

### `jazzy`
The ROS 2 Jazzy port: `output/jazzy_migration_analysis.md`, gz sim
(`test_room_6x6`, camera pan), and the Phase 5D Orin Jazzy docker and Thor
images. On 2026-10-12, `test-campaign` was merged in (`6af1ddf`) so the
M02 fixes run in sim. On 2026-10-12/13 it gained a test-parity run with
campaign-final's ideas (corridor funnel, `base_front`, camera pan off).
**Used for:** sim on linus and the Jazzy targets.
**Note:** local `jazzy` is 8 commits behind origin. Parts of campaign-final
now exist on both branches as separate implementations, so expect
conflicts when they are reconciled.

### `orin-snapshot-20261013`
The Orin's working tree on `test-campaign 6ebbc8a`, committed on 2026-10-13,
plus `output/orin_check_report.md`. The report contains:
- rosbag2 0.15.16, which confirms Part B works as written;
- measured geometry;
- a CONDITIONAL GO, pending the LiDAR checks.

**Used for:** delivering the Orin report. That report was the input Part B
was waiting for.

### `test-campaign`
The M02 campaign fixes on Humble: the Part A fixes A1–A5, review items 1–7,
B8, and the backlog B9/B10. It is the "wrong base" that `campaign-final`
deliberately does not build on. All of its commits are already in `jazzy`.
The last one, `349c881`, went in as `da79f37` (same subject, adapted
patch). Its worktree is `~/dev_ws/f1tenth_more_tc`.
**Used for:** nothing new. It is history.

### `test-logging`
`main` plus `573f313` (the M01 campaign data). Every campaign branch
contains it. Once it is merged into main, the branch is redundant.

### `snapshot/pre-campaign-final-20261012`
`jazzy 49de45d` plus one commit that captured the dirty tree on linus
(25 files, sim and controller edits) before the campaign-final work
started. It has no successors. It exists only so that work can be
recovered. A tag is the right container for that.

### `campaign-dirty-evidence`
`573f313` plus `7e5910e`. It records the `stack_params.yaml` that was
actually on the car during P004-R012..R019. That file is evidence for the
run analysis; see `params-snapshot-not-launch-proof` in memory for why it
matters. It has no successors, so it should be a tag.

### `refactor/mpc-corridor-split`
Based on `1d00853` ("7/10 tests"). It adds three commits that exist on no
other branch:
- `1e23f6a`: map→odom age guard, golden-replay of the control loop;
- `ddd0235`: drops `andre_mpc_node`;
- `a3ea50d`: splits the corridor out of `MPC_corr`.

campaign-final and jazzy have both reworked `mpc_corr` since then, so
rebasing it will not be trivial. **Decision needed:** port it onto
campaign-final or jazzy, or archive it as a tag and drop it.

### `llm`, `scene-graph`, `sick`
Old feature branches. All three point at exactly `main` and contain
nothing of their own, so they are safe to delete.

## Worktrees on linus

| Path | Checkout | State | Cleanup |
|---|---|---|---|
| `~/dev_ws/f1tenth_more` | `jazzy` | Submodules out of sync with the branch | Main checkout |
| `~/dev_ws/f1tenth_more_tc` | `test-campaign` | clean, has build/install | Remove with `test-campaign` |
| `~/dev_ws/f1tenth_more_sync` | `sync/m02-into-jazzy` | Uninitialised submodules only, has build/install | Remove |
| `~/dev_ws/f1tenth_more_base` | detached `5c82958` (jazzy, 2026-10-12) | Uninitialised submodules only, has build/install | Remove; it looks like a scratch baseline build |

## Proposed cleanup order

1. Copy `output/orin_check_report.md` onto `campaign-final`, if it should
   live there.
2. Turn `snapshot/pre-campaign-final-20261012` and
   `campaign-dirty-evidence` into tags and push the tags. Then delete the
   branches locally and on origin.
3. Delete `llm`, `scene-graph` and `sick` on origin.
4. Remove the `_tc`, `_sync` and `_base` worktrees. Then delete
   `test-campaign` and `sync/m02-into-jazzy`.
5. Fast-forward `main` to `test-logging` (or later to `campaign-final`).
   Then delete `test-logging`.
6. Decide on `refactor/mpc-corridor-split`.
7. Delete `orin-snapshot-20261013` once step 1 is done.

The branches left are `main`, `campaign-final` and `jazzy`.
