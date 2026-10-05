# CLAUDE.md

Working notes for this workspace. Not a full conventions document yet — seeded
with the gotchas that have already cost real debugging time. Add to it when
something bites.

## Testing

### `colcon test` runs nothing unless `setup.py` declares a `test` extra

colcon's `ament_python` test step only invokes pytest for a package whose
`setup.py` declares:

```python
extras_require={
    'test': ['pytest'],
},
```

Without it, colcon falls back to `setup.py test`, whose unittest discovery
finds no pytest-style tests and prints:

```
Ran 0 tests in 0.000s

OK
```

That is a **green result that executed nothing**. Until 2026-09-03 only
`mpc_controller` and `llm` had the extra, so every other package's test suite
had never run — 628 tests across 17 packages were dead weight. Fixed workspace-wide
in `aaae377`.

`tests_require=['pytest']` is **not** the same thing and does nothing: modern
setuptools does not recognize it, warns `UserWarning: Unknown distribution
option: 'tests_require'` on every build, and ignores it. Removed in `626fb7b`.

**When adding a new ament_python package, declare the `test` extra.** If a
package reports suspiciously few tests, check its `setup.py` before believing it.

### A test that never runs will contradict the code and nobody notices

`f1tenth_perception/test/test_detection_launch_config.py` asserted that
`yolo_model` defaulted to `yolo26s.engine`. The same commit (`96c6bbc`) changed
that default to `yolo26s-seg.pt`. The test was wrong from the moment it was
written and survived two days of live runs on the new default, because colcon
never executed the file (see above). It surfaced the instant real test discovery
was switched on.

Two habits that would have caught it:

- **Name a test after what it asserts.** The stale one was called
  `test_default_launch_stays_on_tensorrt_box_detector_unchanged` while the
  deployed default had moved to segmentation — the name actively argued for
  the wrong behaviour.
- **When a test encodes a config default, it is a coupling.** Changing the
  default in `stack_params.yaml` means updating the test in the same commit.

## Packaging

### Never put a literal `--` inside a `package.xml` XML comment

`--` is illegal inside an XML comment body. colcon does not fail loudly; it
silently downgrades the package type to plain `python`, which then shows up as
a runtime-only "package not found" crash long after the build reported success.
Use a single hyphen in comment prose.

## DDS / discovery

### Every participant loads `config/fastdds_profile.xml` (5 s lease)

`src/f1tenth_bringup/config/fastdds_profile.xml` sets a 5 s participant lease
(Fast DDS default 20 s). A participant that dies without disposing is dropped
from the Discovery Server within 5 s; with 20 s, a stack started inside that
window intermittently never matched some endpoints (backlog H1: 8 of 30
bringups, slam_toolbox never activating; 0 of 90 with the profile, fix batches
4 and 5).

The lease is announced by **each participant itself** — the server and the
other side cannot impose it (measured: profile on the server only, a killed
node still stays 19.4 s). So it must reach every process:

- the stack: `supervisor_bringup.launch.py` / `stack_bringup.launch.py` set
  `FASTRTPS_DEFAULT_PROFILES_FILE` (launch argument `fastdds_profile`). A new
  top-level launch entry point must set it the same way, next to
  `ROS_DISCOVERY_SERVER`;
- everything started by hand, the Phase S sim host included:
  `source scripts/env/jazzy.sh` (bash or zsh). After changing it, `ros2 daemon
  stop`;
- the car's Docker image runs the launch files, so it is covered.

Never add a second profile file or an `is_default_profile` elsewhere: Fast DDS
loads one default profile.

### Launch parameters: never `on` / `off` / `yes` / `no` as string values

Launch writes parameters through a YAML file, and YAML turns `on`/`off`
into booleans: a string parameter declared with default `'on'` received
`True` and the node refused to start (fix batch 5, `health_watchdog`). Use
words YAML leaves alone (`enforce`, `alert`, `disabled`).

## Harness gotchas

- `phase5_bringup.sh` refuses to start a run while **any** ROS process is up
  (its precheck). A probe or talker started "on the side" during a batch
  aborts the batch and contaminates its traffic numbers. Run nothing ROS while
  a batch runs.
- `pkill -f PATTERN` / `pgrep -f` inside a Bash tool call match the calling
  shell's own command line, which contains PATTERN: `pkill -f` then kills the
  call itself. Kill by PID, or by exact process name (`pkill -x`).

## Rebuild policy

### After any `ros-jazzy-*` apt upgrade, clean-rebuild every CMake package

An apt upgrade can change a library's ABI inside a distro. `diagnostic_updater`
4.2.6 → 4.2.7 added a parameter to `Updater`'s constructor; the workspace's
`ackermann_mux` (and `zed_components`, `zed_debug`), built the day before the
2026-10-01 upgrade, then died at startup with `symbol lookup error: undefined
symbol: _ZN18diagnostic_updater7UpdaterC1E...` and the supervisor's `control`
component crash-looped. Nothing failed at build time, and no unit test runs the
binary. Found in Phase 5, only by running the full stack.

A plain rebuild is **not** enough: dpkg installs headers with their original
mtimes (months old), so make considers the old object files up to date and the
link fails again. After every `ros-jazzy-*` upgrade:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-clean-first \
  --packages-select $(colcon list -t | awk '$3=="(ros.ament_cmake)"{print $1}')
```

then scan the install space for unresolved symbols —
it must print nothing:

```bash
source install/setup.bash
find install -path '*/lib/*' \( -type f -o -type l \) | while read f; do
  file -L "$f" | grep -q ELF || continue
  ldd -r "$f" 2>&1 | grep 'undefined symbol' | grep -v '__rosidl_\|__msg__\|__srv__\|__action__' \
    | sed "s|^|$f: |"
done
```

(The message-type symbols filtered out are resolved at load time through the
generated typesupport libraries and are reported by `ldd -r` on those libraries
alone.)

The workspace is built with `--symlink-install`; keep it that way when
rebuilding single packages.

**Docker:** the image for the car must build the workspace against the same apt
snapshot it runs. Never copy an `install/` built against a different set of
`ros-jazzy-*` packages into it.
