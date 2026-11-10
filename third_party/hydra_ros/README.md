# third_party/hydra_ros -- Hydra (MIT-SPARK) as pinned submodules

Hydra builds the 3D scene graph for the HIL setup (`docs/hydra_hil.md`). Its 11 upstream
repositories live here as **separate git submodules**, each pinned to the exact commit named in
`src/f1tenth_hydra/config/hydra/hydra_jazzy.repos`. Nothing is forked and nothing is merged into
this repository: every submodule is upstream's own history at an upstream commit, and each stays
its own colcon package (22 packages in total, see the table).

## Why they are here, and why not under `src/`

- **Reproducibility.** The version is recorded in this repository, so a checkout of a given
  commit of the stack carries the Hydra it was tested with. The old flow (`vcs import` into
  `~/hydra_ws`) left the version in a scratch directory outside version control.
- **Not under `src/`.** `third_party/` carries a `COLCON_IGNORE`, so the stack's own
  `colcon build` at the workspace root never sees these packages: **the default build does not
  build Hydra or GTSAM**, and the stack still builds and runs on a machine where the submodules
  were never initialized. Hydra is built on purpose, into its own build/install space, by
  `scripts/hydra/build_hydra_ws.sh` (`colcon --base-paths third_party/hydra_ros`).
- **One source of truth for the version.** The `.repos` file. `scripts/hydra/check_hydra_pins.py`
  fails if a submodule SHA, or a `.gitmodules` URL, differs from it.

## Pinned versions

Branch names are upstream's; the SHA is what is checked out (`git submodule status`).

| submodule | upstream (https) | branch | commit | colcon packages |
|---|---|---|---|---|
| `hydra_ros` | MIT-SPARK/Hydra-ROS | main | `8f3b7e36` | hydra_ros, hydra_msgs, hydra_visualizer |
| `hydra` | MIT-SPARK/Hydra | main | `2e58a35b` | hydra |
| `config_utilities` | MIT-SPARK/config_utilities | main | `1df5f6b5` | config_utilities, _msgs, _ros |
| `ianvs` | MIT-SPARK/Ianvs | main | `c204dec5` | ianvs |
| `kimera_pgmo` | MIT-SPARK/Kimera-PGMO | ros2 | `382b1f35` | kimera_pgmo, _msgs, _ros, _rviz |
| `kimera_rpgo` | MIT-SPARK/Kimera-RPGO | develop | `222ad5e8` | kimera_rpgo |
| `pose_graph_tools` | MIT-SPARK/pose_graph_tools | ros2 | `965dfe86` | pose_graph_tools, _msgs, _ros |
| `semantic_inference` | MIT-SPARK/semantic_inference | main | `14b7eaf7` | semantic_inference, _msgs, _ros |
| `spark_dsg` | MIT-SPARK/Spark-DSG | main | `3c40997e` | spark_dsg (+ the Python bindings) |
| `spatial_hash` | MIT-SPARK/Spatial-Hash | main | `8045892f` | spatial_hash |
| `teaser_plusplus` | MIT-SPARK/TEASER-plusplus | master | `52a9c52e` | teaserpp |

About 81 MB checked out. Each submodule keeps its own upstream licence file (`kimera_rpgo` has
none in-tree; it is BSD-2 per its `package.xml`).

## Use

```bash
git submodule update --init third_party/hydra_ros      # all 11, non-recursive on purpose
scripts/hydra/check_hydra_pins.py                      # SHAs == .repos ?
scripts/hydra/build_hydra_ws.sh --models               # builds into ~/hydra_ws (--ws DIR)
```

A fresh clone **without** `--recurse-submodules` leaves these directories empty; that is a
working state for everything except Hydra itself. `git clone --recurse-submodules` also pulls
the stack's other submodules (`vesc`, `zed_ros2_wrapper`, ...).

**Do not recurse into these submodules.** `semantic_inference` carries a nested submodule
(`facebookresearch/Mask2Former`, for its open-set model zoo) that the closed-set MobileNetV2 path
used here does not need, so `--init` without `--recursive` is deliberate.

## Bumping the version

All 11 move together (upstream develops them as one tree):

1. edit the SHAs in `src/f1tenth_hydra/config/hydra/hydra_jazzy.repos`;
2. `git -C third_party/hydra_ros/<name> fetch origin && git -C ... checkout --detach <sha>` for
   each one;
3. `scripts/hydra/check_hydra_pins.py` must pass, then `git add third_party/hydra_ros` and commit
   the `.repos` change and the submodule bumps **together**;
4. rebuild: `scripts/hydra/build_hydra_ws.sh` (a bump can change the config keys in
   `src/f1tenth_hydra/config/hydra/*.yaml` -- diff upstream's configs).

Never commit a local change inside a submodule: it would make the pin unreachable from upstream.
Fix anything that needs fixing in `src/f1tenth_hydra` (the launch, the configs) instead.
