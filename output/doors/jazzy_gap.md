# Which Jazzy base the door work lands on

Branch `jazzy-doors`, worktree `~/dev_ws/f1tenth_more_jazzy_doors`.

**Base: `origin/main-jazzy` (`74c08897`), not `origin/jazzy`.** The port needs
no adaptation beyond one launch-file conflict.

## Why not `origin/jazzy`

`origin/jazzy` is the stale Jazzy line. It and `origin/campaign-final`
diverged at `573f313b`: **163 commits** one side, **274** the other, and the
perception and costmap stacks there are not versions of each other any more.

| what `semantic_doors.py` needs | `origin/jazzy` | `origin/main-jazzy` |
|---|---|---|
| `MultiWallTrack(Array).msg` + confirmation levels | **absent** | present |
| `/perception/walls` | **the string does not occur in the tree** | present |
| `multi_wall_tracker.py`, `wall_fusion.py` | **absent** | present |
| `semantic_layer_node` walls in map (`_confirmed_walls_map`) | **absent** | present |
| `wall_raster.py`, `unknown_objects.py`, `object_memory.py` | **absent** | present |
| `/scan` + `_laser_pose_for()` in the semantic layer | **absent** | present |
| `SemanticObject(Array).msg` | **absent** | present |
| `Door(Array).msg`, `door_detection.py` (the profile detector) | **absent** | present |
| mission schema, `ObjectSpec`, pose-goal behaviours | present (5.0) | present (5.0) |

| file | `jazzy` | `main-jazzy` | `campaign-final` |
|---|---|---|---|
| `lidar_front_wall_node.py` | 412 lines | 2311 | 2311 |
| `semantic_layer_node.py` | 617 | 1426 | 1426 |
| `stack_params.yaml` | 2582 | 4019 | 3851 |

On `origin/jazzy` the detector's *entire geometric input* is missing, and
`door_source` would have no second detector to choose between. Closing that
gap is a merge of ~274 commits, not a cherry-pick — the separate job the
brief anticipated.

`origin/main-jazzy` is **11 commits behind campaign-final and 101 ahead**, and
holds every foundation. It is also the line that becomes `main`.

## What landed, and what was skipped

| commit | onto main-jazzy |
|---|---|
| `eb78d6b9` docs/branches.md + output/branches/inventory_20261020.md | **clean** (`cc4c266e`) — both files absent here |
| `098c295d` scripts/sim: sim_net.sh sourceable from zsh | **SKIPPED** — see below |
| `a475f80b` the semantic door detector | one conflict, resolved (`22a5da6b`) |
| `91e313d0` the `enter_door` mission goal | **clean** (`494cc292`) |
| `2ba1497b` semantic doors on by default | **clean** (`c862a14f`) |

**`098c295d` was skipped deliberately**, not lost. It patches
`scripts/sim/sim_net.sh` and `scripts/sim/README.md`; neither exists on
main-jazzy, whose sim scripts are a different set (`sim_isaac.sh`,
`sim_stack.sh`, `llm.sh`, `restart.sh`). Cherry-picking it would invent a file
into a layout that does not use it. The fix is preserved on
`origin/jazzy-linus-backup-20261028` and belongs with whoever ports the
`jazzy` line's sim scripts.

**The one conflict** was `src/f1tenth_costmap/launch/costmap.launch.py`:
main-jazzy wraps `semantic_layer_node` in an `OpaqueFunction` for its CPU
thread env (R3, `semantic_layer_cpu_threads`), where campaign-final has a
plain `Node`. Resolved by keeping main-jazzy's `OpaqueFunction` and passing
`semantic_doors_enabled` / `door_source` as two more parameters inside it —
they need no launch context, so nothing about the thread-env machinery
changes.

## The mission the brief asks to fix

`missions/straight_until_potted_plant.json` — the one with
`stop_condition.type = " "` — **does not exist on main-jazzy**, and every
mission that does exist here parses with a valid `stop_condition.type`. It is
a campaign-final-only mission. Nothing to fix on this base; the fix belongs in
a campaign-final-derived branch.

## Sim, for trying `enter_door` next

- `husarion_gz_worlds/models/Door/` exists but is a **static closed door
  leaf** (one STL, `<static>1</static>`). To a LiDAR a closed leaf is wall,
  and the detector reports open openings only — so the model as it stands is
  the opposite of a test case.
- It is **not placed in `husarion_office.sdf`** anyway: that world is 265
  furniture and floor models (`OpenRobotics/AdjTable`, `Drawer`, `DeskChair`,
  `floor_*`) with no wall or door models at all. Open plan, nothing to hold a
  doorway.
- `f1tenth_sim/worlds/` has `test_room_6x6.sdf` (the default), `empty_room.sdf`
  and `husarion_office_2x.sdf`.

**The cheapest sim test is a new world, not an existing one**: two collinear
box walls with a 0.90 m gap and a room behind, in the `test_room_6x6` style.
That is exactly the scene `test_semantic_doors.py` already encodes
synthetically, so the unit tests and the sim run would check the same
geometry.
