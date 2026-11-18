---
id: 2026-10-16_git-integration-orin
title: "Log integrazione git sull'Orin (campaign-final)"
area: _unsorted
date: 2026-10-16
tags: [git, merge, orin, campaign-final, dati-campagna]
modules: {f1tenth_logger: "0.0.1"}
branch: "orin/orin-wip-2026-10-22"
commit: "02cd0f2084da"
status: done
hypothesis: "n/d"
result: "Divergenza: 3 commit solo remoti (da linus) e 64 solo locali (3ffe245..9c46add); 1 file toccato da entrambi (TEST_CAMPAIGN.md); merge senza collisioni, e67aded locale non pushato."
decision: "Backup tag backup/campaign-final-9c46add; push non eseguito (comando indicato); 59 MB di run M02 in first_test_campaing/ lasciati all'operatore."
raw_data: "nessun dato grezzo su git. Esterni: Orin first_test_campaing/ (485 file, 59 MB, 8 run M02 2026-10-13/14: P002-R001..R008)"
migrated_from: "orin/orin-wip-2026-10-22:output/git_integration/log.md"
migrated_on: 2026-11-18
---
# Git integration log — Orin, campaign-final

Date: 2026-10-16. Operator: Claude (Opus 5), on the Orin, `~/dev_ws/f1tenth_more`.
Rules in force: no force-push, no reset --hard, no rebase, no branch -D, no clean -fd,
no stash drop. The local dirty state was not touched.

## 0. Backup

    git tag backup/campaign-final-9c46add 9c46add      # local only, not pushed

`9c46add` is reachable from the tag and from the first parent of the merge commit, so
the history `cpu-cleanup` is based on is intact.

## 1. Divergence (read-only)

    R=git@github.com:sjckness/f1tenth_more.git
    git fetch $R campaign-final:refs/remotes/gh/campaign-final
    git fetch $R 'refs/heads/*:refs/remotes/gh/*'

Remote heads: campaign-final, jazzy, jazzy-campaign, main. **No `cpu-cleanup`.**

merge-base: `3ffe245` "tools: scene_labels_replay, and the replay report".

Only on the remote (3 commits, all `andreas-linus <andreas21steffens@gmail.com>`,
2026-10-14, i.e. pushed from **linus**, based on `3ffe245`):

| hash | date | subject | files |
|---|---|---|---|
| `cdfd736` | 2026-10-14 13:27 | first_test_campaing: the full test-campaign data from archive/orin-snapshot-20261013 | 898 A |
| `a8a61db` | 2026-10-14 13:42 | campaign: fix folder name | 918 R100 + 14 M |
| `7b17e2f` | 2026-10-14 13:50 | docs: repo reorganisation plan (logs, output/, tools/) | 1 A |

Only local: **64 commits** (`3ffe245..9c46add`, the H1-H5 series from the last two days).

Net remote diff vs merge-base: 899 A, 20 R100, 14 M — touching `docs/` (1),
`first_test_campaign/` (918) and `src/` (14: f1tenth_logger 10, f1tenth_params 2,
plus the two already counted).

Files touched by **both** sides: exactly one — `src/f1tenth_logger/TEST_CAMPAIGN.md`.

Collision check against the local dirty state: of the 899 paths the remote adds,
**0** exist locally as untracked or modified files. `build_base573/`,
`install_base573/`, `output/`, `.claude/settings.json` and the vesc submodule are
outside the remote diff entirely.

## 2. Merge

    git switch campaign-final
    git merge --no-ff gh/campaign-final -m "Merge remote campaign-final into Orin campaign-final"

Result: `e67aded`. **No conflicts, trivial or otherwise.** The one shared file,
`TEST_CAMPAIGN.md`, merged by hunk: the remote rewrote the spelling
`first_test_campaing` → `first_test_campaign` at original lines 77-235, the local side
added the `enable_mission_logger:=true` block at original lines 56-65. Disjoint regions,
both survive (7 occurrences of the corrected name, 2 of the new launch argument).

    git submodule update --init --recursive        # exit 0, no checkouts

The vesc submodule pointer is unchanged (`2848a9f`, heads/scene-graph) and its four
untracked directories (`vesc_tuning/**/__pycache__`, `vesc_tuning/speed_sweep_results/`)
are untouched. The earlier ` M src/f1tenth_hardware/vesc` in `git status -uall` was
untracked *content* inside the submodule, not a pointer change.

## 3. cpu-cleanup

**Not integrated — the branch does not exist on GitHub.** `git ls-remote --heads`
returns only campaign-final, jazzy, jazzy-campaign, main. Per the work order, stopping
here and not reaching for linus.

Consequence for the point-4 greps: `wall_distance_node` and `ekf_cost_observer_node`
are both still present and still launched, which is the correct state for a tree that
does not contain cpu-cleanup.

- `wall_distance_node`: 11 files; launched by
  `src/f1tenth_perception/launch/wall_distance.launch.py:201`.
- `ekf_cost_observer_node`: 6 files; launched by
  `src/f1tenth_localization/launch/localization.launch.py:102`, condition
  `enable_ekf_cost_observer`, **default_value='true'**.

Re-run both greps after cpu-cleanup is merged.

## 4. Verification

### Build

    colcon build --symlink-install --packages-above f1tenth_logger f1tenth_params

`Summary: 17 packages finished [46.9s]`, zero warnings, zero errors. The 17 are
f1tenth_logger, f1tenth_params and every dependent: f1tenth_behavior, f1tenth_bringup,
f1tenth_control, f1tenth_costmap, f1tenth_description, f1tenth_diagnostics,
f1tenth_hardware, f1tenth_localization, f1tenth_more, f1tenth_navigation,
f1tenth_perception, f1tenth_sim, llm, mpc_controller, vesc_tuning.

Both touched packages declare the `test` extra (f1tenth_logger/setup.py:39,
f1tenth_params/setup.py), so `colcon test` really executes their suites rather than
reporting an empty green.

### Tests

    colcon test --packages-select f1tenth_logger f1tenth_params \
                --test-result-base <scratchpad>/tr_after

(An isolated `--test-result-base` on purpose: `colcon test-result` otherwise reads
stale xml from earlier runs.)

| package | tests | errors | failures | skipped |
|---|---|---|---|---|
| f1tenth_logger | 319 | 0 | **2** | 0 |
| f1tenth_params | 54 | 0 | 0 | 0 |
| **total** | **373** | **0** | **2** | **0** |

The two failures are `test_copyright` and `test_pep257` — ament lint, not logic.
Proven pre-existing by linting a detached worktree at `9c46add` and comparing:

| check | at 9c46add (before) | at e67aded (after) |
|---|---|---|
| `ament_copyright` | 35 errors, 39 files checked | 35 errors, 39 files checked |
| `ament_pep257` | 414 errors | 414 errors |

Byte-identical counts, so the merge introduced no new lint error and no new test
failure. 317 of f1tenth_logger's 319 and all 54 of f1tenth_params pass. This matches
what `docs/repo_reorg_plan.md` reports from linus (test_campaign 203 passed,
test_corridor_geometry.py 38 passed).

Merged source spot-checks: `robot_logger.py:105 DEFAULT_CAMPAIGN = "first_test_campaign"`,
`config/test_campaign_logger.yaml:14 campaign: first_test_campaign`. The only remaining
`first_test_campaing` spellings in src/tools/scripts/docs are the 3 narrative mentions
inside `docs/repo_reorg_plan.md` itself.

### git log --oneline --graph -15

```
*   e67aded Merge remote campaign-final into Orin campaign-final
|\
| * 7b17e2f docs: repo reorganisation plan (logs, output/, tools/)
| * a8a61db campaign: fix folder name
| * cdfd736 first_test_campaing: the full test-campaign data from archive/orin-snapshot-20261013
* | 9c46add H5: one teardown for both paths, which is where the orphans came from
* | 1d0334e H5: cleanup kills process GROUPS, not just the launch pid
* | e2518ad H5: the CPU test, and the three bugs its own dry run found
* | 6cd1609 H4: one archive run carries the front_distance bias
* | eaffd16 H3: the publish cap is a budget spent on roles, not on a score
* | e41f606 H2: the wall tracker on one scan in four
* | c421578 H1: extract_lines in C, 24.6 ms a scan down to 1.1
* | 70307ae nightly: log the staging mistake in 05c5898
* | 9e1c8a8 nightly: the morning report
* | fdd65e0 G: intensity is there, the glass examples are not
* | edfd9f8 WIP: G1 green and the glass/intensity probe
```

## 5. Push — NOT DONE

Point 4 is green, but the work order says that when `cpu-cleanup` is absent from
GitHub, stop after point 4. `e67aded` is therefore committed locally and **not pushed**.
One command away, no force:

    git push git@github.com:sjckness/f1tenth_more.git campaign-final

## Open item for the operator: 59 MB of campaign data stranded by the rename

Not touched, per the rules — the decision is the operator's.

`first_test_campaing/` (misspelled) still exists on the Orin as **untracked** data:
485 files, 59 MB, holding 8 M02 runs recorded 2026-10-13/14 plus `results.csv`,
`campaign.json` and `M02_approach_wall/mission.json`:

    P002-R001-20261013T131046   P002-R005-20261013T133321
    P002-R002-20261013T131735   P002-R006-20261013T170541
    P002-R003-20261013T133146   P002-R007-20261013T171355
    P002-R004-20261013T133225   P002-R008-20261014T151520

None of these 8 run ids exists in the merged, tracked `first_test_campaign/M02_approach_wall/`
(16 runs, all from 2026-09-22 / 09-23 / 10-08) — they are genuinely new data, not duplicates.

The logger now writes to `first_test_campaign/` (`DEFAULT_CAMPAIGN`), so the next
campaign run will not see them and `test_campaign_export` / `test_campaign_analyze`
will silently omit them. `docs/repo_reorg_plan.md:57-59` anticipates exactly this:
"Unpushed runs still sitting in a local `first_test_campaing/` on the Orin must be
moved by hand."

The merge did not collide with any of it (0 of the 899 added paths exist locally),
so nothing was lost; it is simply in the old folder.
