# Rust toolchain for the F1Tenth workspace (rclrs, Jazzy)

State on **linus** (Ubuntu 24.04.4, x86_64, ROS 2 Jazzy from apt). First written 2026-10-18 (branch
`jazzy-rust`, apt as of 2026-04-12, rclrs 0.7.0); **sections 1 to 7 updated 2026-10-24 for R0b**:
linus's `ros-jazzy-*` now come from the ROS snapshot **2026-09-11** (`tools/linus/ros_snapshot_move.sh`,
the same set as the Thor and the Orin image), and the rclrs packages use **rclrs 0.8.0 + ros-env
0.3.0**. Everything Rust is user-space (rustup, pip `--user`); only the snapshot move needed sudo
(the user ran it). What R0b changed, with every gate: `output/rust_main/r0b.md`.

## 0. Base of the branch

`jazzy` (3dbe37f, 2026-10-13) contains **none** of the campaign-final work: no cpu-cleanup, no
lidar_front_wall / virtual wall, no scene labels. The Isaac sim mode exists only on
`jazzy-campaign`, and cpu-cleanup only on `origin/campaign-final` (Humble, merged there
2026-10-18 13:39). Decision (Andreas, this session): jazzy-rust =
`jazzy-campaign` + merge of `origin/campaign-final` 0c1cd41.

| Step | Result |
|---|---|
| `git rebase --onto jazzy-campaign jazzy jazzy-rust` | the one jazzy-rust commit (zenoh recon report) now on jazzy-campaign 883a0f0; old tip tagged `backup/jazzy-rust-pre-sync-ced4d90` |
| `git merge origin/campaign-final` | 90b367c; 4 conflicts, each "keep both sides" (preflight actuation node + wall requirement, supervisor sim rewrite + mission-logger opt-in, reset_manager params_overlay + skip_components, ekf_cost_observer removed) |
| Submodules | `git submodule update --init` in the worktree (its checkouts were the old jazzy pins) |
| Build fix | `corridor_perception_c` (new in campaign-final H1) failed on Ubuntu 24.04: `M_PI undeclared` under `-std=c11`. Added `add_compile_definitions(_DEFAULT_SOURCE)`: widens the headers only, ISO C mode (FP contraction off) unchanged |
| Test fix | `test_reset_manager_component.py` read the launch file's last entity as a `Node`; the sim side builds it in an `OpaqueFunction`. The helper now evaluates it (test only) |

## 1. Workspace build on linus

```bash
source output/rust_main/env.sh       # /opt/ros/jazzy, jc_venv on PYTHONPATH, src/rust/build_env.sh,
                                     # WS = this worktree, SKIP = output/jazzy_port/linus_skip.txt
colcon build --symlink-install --packages-ignore $SKIP
```

- **No user-space underlay any more.** Since the snapshot move, slam_toolbox, py_trees_ros(+interfaces),
  topic_tools, test_msgs and serial_driver are apt packages. `~/dev_ws/jc_deps_ws` and
  `~/dev_ws/rust_underlay` were built against the old apt set: never source them again (plan P2.4
  retires them by `mv`). `jc_venv` stays on PYTHONPATH for the pip-only deps (osqp, pyarrow,
  ultralytics, torch).
- After the move, every CMake package must be rebuilt clean (CLAUDE.md "Rebuild policy"); a
  fresh worktree is clean by construction.
- Clean build of a fresh worktree (R0b, `output/rust_main/r0b_build.log`): **37 packages, 3 min
  45 s**, `-j3`, with other agents loading the machine. The three rclrs packages take 2 min to
  2 min 25 s each (ros-env compiles every message package, section 2), f1tenth_mpc_core 13 s
  (OSQP C included), f1tenth_wall_tracker_core 7 s.
- `--packages-select` alone fails on linus (`zed_wrapper` package.sh missing: it is in the skip
  list); always combine it with `--packages-ignore $SKIP`.
- Tests: `colcon test` with isolated DDS (`unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
  FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_DOMAIN_ID=9x ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`).
  The failing-ID baseline is `output/jazzy_main/test_failing_ids.txt`
  (`python3 tools/r3_failing_ids.py <test-result-base> output/jazzy_main/test_failing_ids.txt`).

## 2. Versions

| Component | Version | Source |
|---|---|---|
| rustc / cargo | 1.99.1 (stable, x86_64-unknown-linux-gnu) | rustup (`~/.cargo`) |
| **rclrs** | **=0.8.0** (pinned exactly) | crates.io |
| **ros-env** | **=0.3.0** (pinned exactly; rclrs 0.8.0 depends on `^0.3`) | crates.io |
| rosidl_runtime_rs | 0.7.0 (through rclrs and ros-env) | crates.io |
| ament_rs | 0.3.0 (rclrs and ros-env build scripts) | crates.io |
| rosidl_generator_rs | **0.5.0** (`ros-jazzy-rosidl-generator-rs 0.5.0-1noble.20260813`, snapshot 2026-09-11) | apt |
| colcon-cargo / colcon-ros-cargo | 0.2.0 / 0.2.0 | pip (user) |
| cargo-ament-build | 0.1.11 | `cargo install cargo-ament-build` |
| osqp-sys (f1tenth_mpc_core) | =1.0.1 (OSQP C 1.0.0), QDLDL v0.1.8 vendored (section 5) | crates.io + `src/rust/third_party` |
| libclang | not needed (rclrs ships pre-generated rcl bindings per distro) | |

**Why rclrs 0.8.0 now.** rclrs 0.7.0 was the newest release on rosidl_runtime_rs 0.6, which the
old apt generator (0.4.11) targeted. The snapshot ships generator 0.5.0, the Thor has the same,
and the Orin image is built from the same snapshot, so all three machines now take rclrs 0.8.0,
the release on rosidl_runtime_rs 0.7. Its changelog items that touch these nodes: messages come
from ros-env (below), the vendored test crates are gone (section 3), and on Jazzy every node now
also creates its `/rosout` publisher (log lines reach `/rosout` as rclpy's and rclcpp's do; 0.7
did not). The API the three nodes use (Context, executor, declare_parameter builders, publishers,
subscriptions with QoS options, timers on the node clock, `log_*!` with throttling) compiled
unchanged; only the message imports changed.

**How the messages are compiled (ros-env 0.3.0).** The generator still writes a crate per
interface package into `share/<pkg>/rust` (Cargo.toml + `src/{msg,srv,action}.rs` +
`.../rmw.rs`). Its Cargo.toml says `rosidl_runtime_rs = "0.6"` (sic, the generator 0.5.0
template) and `[package.metadata.ros-env] include = true`. Nothing resolves that manifest with
cargo: ros-env's build script walks `AMENT_PREFIX_PATH`, keeps every crate marked `include = true`
(plus the `*` dependencies they name, minus any whose dependencies are missing), and writes one
`interfaces.rs` that `include!()`s their source files into modules
`ros_env::<pkg>::{msg,srv,action}[::rmw]`. That code is compiled inside ros-env, against ros-env's
own rosidl_runtime_rs 0.7, which is rclrs 0.8's; the typesupport libraries come in through the
`#[link(name = "<pkg>__rosidl_typesupport_c")]` attributes in the generated `rmw.rs`, and rustc's
`--as-needed` keeps only the ones a binary uses (section 4). Consequences:

- The members declare no message crates: `rclrs = "=0.8.0"`, `ros-env = "=0.3.0"`, then
  `use ros_env::sensor_msgs::msg::LaserScan;` (and `use ros_env::builtin_interfaces;` where the
  code names `builtin_interfaces::msg::Time`).
- An interface package is visible to a member only if its prefix is on AMENT_PREFIX_PATH when the
  member builds. colcon puts the prefixes of a package's dependencies there, so each package.xml
  keeps its `<depend>`s: they are now the only thing that orders the build.
- ros-env compiles **every** included message package (56 on linus's /opt/ros plus the
  workspace's f1tenth_messages, zed_msgs, vesc_msgs, teleop_tools_msgs), once per target dir. That
  is why a first build of an rclrs package takes about 2 minutes, and why one generated crate
  that does not compile breaks every rclrs package (plan "Risks").
- ros-env's build script re-runs when AMENT_PREFIX_PATH or an included crate changes, not when a
  new package appears in an already listed prefix (merged install): see section 6.

**pip, not pipx, for the colcon plugins** (unchanged). colcon is installed from apt and finds its
extensions through entry points in its own interpreter's site-packages, which pipx's isolated
venvs are not. `pip install --user --break-system-packages colcon-cargo colcon-ros-cargo`.

## 3. ros2-rust/ros2_rust#557 (obsolete since R0b)

rclrs 0.6 / 0.7 vendored copies of `example_interfaces` and `test_msgs` (for their doctests) and
linked their typesupport libraries, so every rclrs package had to declare both and linus needed
a hand-built `test_msgs` underlay (`~/dev_ws/rust_underlay`). rclrs 0.8 vendors nothing (its own
tests take them from ros-env, `cfg(test)` only), so R0b removed both dependencies from Cargo.toml
and package.xml. On the snapshot, test_msgs is an apt package anyway. Checked on the installed
binaries: no Rust artefact has `test_msgs` or `example_interfaces` in its `DT_NEEDED`
(`output/rust_main/r0b_install_checks.log`), even though ros-env compiles both crates (they are in
/opt/ros/jazzy).

## 4. Layout and build of the Rust packages

```
src/rust/
  Cargo.toml                     virtual workspace: members, [profile.dev] opt-level = 3
  Cargo.lock                     committed (crates.io entries only, section 4.1)
  clippy.toml rustfmt.toml       tests may unwrap; edition 2021
  build_env.sh                   source before building (QDLDL offline, section 5)
  third_party/                   COLCON_IGNORE; qdldl/ (QDLDL v0.1.8), osqp_offline.cmake, README.md
  f1tenth_proximity_guard/       ament_cargo: proximity_guard_node (rclrs)
  f1tenth_costmap_boundary_rs/   ament_cargo: costmap_boundary_node (rclrs)
  f1tenth_hard_stop_rs/          ament_cargo: hard_stop_node (rclrs), R4
  f1tenth_mpc_core/              ament_cargo: cdylib + rlib, no ROS (corridor R1, solver R2, OSQP C)
  f1tenth_wall_tracker_core/     ament_cargo: cdylib + rlib, no ROS (R6)
```

- colcon-cargo identifies the virtual workspace and enumerates its members; each member is an
  `ament_cargo` package (colcon-ros-cargo), built with `cargo ament-build -- --target-dir
  build/<pkg>` (no `--release`: the dev profile, at opt-level 3 since R0).
- **Members do not inherit from the workspace** (`x.workspace = true`): cargo-ament-build installs
  each lib crate's manifest into `install/<pkg>/share/<pkg>/rust`, where there is no workspace
  root. Versions and lints are repeated per crate.
- **colcon-ros-cargo still writes `.cargo/config.toml`** in the colcon workspace root: a
  `[patch.crates-io]` entry per crate registered under
  `share/ament_index/resource_index/rust_packages` in the prefixes of the package's
  dependencies (and of the calling shell's AMENT_PREFIX_PATH). Generator 0.5.0 registers nothing
  there, so only the workspace's own five ament_cargo crates do: a first build in a fresh
  worktree writes an empty `[patch.crates-io]`, a rebuild with the workspace sourced patches all
  five, and in a merged install the ones installed earlier in the run (section 6). No member
  uses a patch any more. `.cargo/` stays `.gitignore`d, and never run two colcon builds of Rust
  packages at once (both rewrite it).
- **Each rclrs crate's `build.rs` emits `-L <prefix>/lib` for every AMENT_PREFIX_PATH entry and
  re-runs when AMENT_PREFIX_PATH changes** (R0b). rclrs 0.8.0's own build script emits those
  paths but re-runs only on ROS_DISTRO, while ros-env re-runs on AMENT_PREFIX_PATH: without our
  copy, a second build in the same target dir with more prefixes on the path (workspace sourced,
  `colcon test`) included more message crates than rclrs's stale search paths covered and failed
  to link (`unable to find library -lzed_msgs__rosidl_typesupport_c`, `output/rust_main/r0b.md`
  section 1.1).
- **Building one Rust package alone** works with `--packages-select <pkg> --packages-ignore
  $SKIP`. The old trap (cargo resolving every member needed the patched `f1tenth_messages` crate,
  hence the `<build_depend>f1tenth_messages</build_depend>` in every member) is gone with the
  "*" message crates; the build_depends stay, for build order only.
- **Gates** (from `src/rust`, `output/rust_main/env.sh` sourced, after a colcon build so the
  workspace's f1tenth_messages is on AMENT_PREFIX_PATH for ros-env):

  ```bash
  cargo fmt --all --check
  cargo clippy --workspace --all-targets -j 3 -- -D warnings  # unsafe_code forbid/deny; unwrap/expect/panic deny
  cargo test --workspace -j 3                                 # R0b: 182 passed
  colcon test --packages-select f1tenth_proximity_guard f1tenth_costmap_boundary_rs \
    f1tenth_hard_stop_rs f1tenth_mpc_core f1tenth_wall_tracker_core --packages-ignore $SKIP
  ```

- `serde_json` is used with `float_roundtrip`: its default float parser is not correctly
  rounded (1 ulp), which the parity harnesses would report as differences.

### 4.1 Cargo.lock

Committed with crates.io entries and the five members only. Every cargo run under a
`.cargo/config.toml` with patches nobody uses appends `[[patch.unused]]` blocks; they are
machine-specific (absolute install paths behind them) and are **not committed**. After R0b
only the members' own installed crates are ever patched (section 4 above): a fresh build leaves
the file untouched, a rebuild with the workspace sourced appends five blocks (f1tenth_*): drop
them before a commit (the tomllib comparison of section 6 tells whether anything else changed).
Updates are precise
(`cargo update -p <crate> --precise <version>`), reviewed with a package/version diff; R0b's is in
`output/rust_main/r0b.md`. Do not use `cargo build --locked` (or colcon `--cargo-args --locked`)
in a build where colcon-ros-cargo wrote patches: cargo then wants to add the `[[patch.unused]]`
blocks and refuses (checked, section 6).

Install result (lib/<pkg>/): `proximity_guard_node`, `proximity_guard_replay`,
`costmap_boundary_node`, `costmap_boundary_replay`, `hard_stop_node`, `hard_stop_replay`,
`libf1tenth_mpc_core.so`, `libf1tenth_wall_tracker_core.so`. The nodes are about 62 MB each
(debug info of the whole ros-env message set; the dev profile keeps debuginfo), the replays 9 to
12 MB.

## 5. OSQP's QDLDL without network (`src/rust/third_party`)

f1tenth_mpc_core links OSQP C 1.0.0 through `osqp-sys =1.0.1`, whose `build.rs` builds OSQP with
CMake (the `cmake` crate). OSQP's `qdldl.cmake` declares QDLDL with FetchContent
(`https://github.com/osqp/qdldl.git`, tag `v0.1.8`), so until R0b every clean build cloned it from
GitHub. Now:

- `src/rust/third_party/qdldl/` is the tag's tree (commit `138fdac58b9cd1c4137ff1b99152c8108a6cff5b`,
  git tree `7bbce655e64e74c476ca4694c06d7bfb08a5cf3d`, Apache-2.0), unmodified; the earlier online
  builds fetched exactly this tree. `third_party/README.md` says how to verify and update it.
- `src/rust/third_party/osqp_offline.cmake` is a CMake toolchain file that sets
  `FETCHCONTENT_SOURCE_DIR_QDLDL` to that directory and `FETCHCONTENT_FULLY_DISCONNECTED` on.
- `src/rust/build_env.sh` (bash or zsh, sourced) exports
  `CMAKE_TOOLCHAIN_FILE_<host triple with _>` (`CMAKE_TOOLCHAIN_FILE_x86_64_unknown_linux_gnu` on
  linus, `..._aarch64_unknown_linux_gnu` on the Thor / Orin). The `cmake` crate reads that
  target-specific name; CMake itself only reads plain `CMAKE_TOOLCHAIN_FILE` (>= 3.21), so the
  colcon build's CMake packages never see the file. `output/rust_main/env.sh` sources it.
- Effect, checked in every R0b build (colcon, cargo, the image mirror): the OSQP build tree has
  `_deps/qdldl-build` and no `_deps/qdldl-subbuild` / `_deps/qdldl-src`; `CMakeCache.txt` shows
  the toolchain file, `FETCHCONTENT_SOURCE_DIR_QDLDL` = the vendored path, FULLY_DISCONNECTED on;
  all builds ran with `GIT_ALLOW_PROTOCOL=file` (git refuses https). Compiler (`/usr/bin/cc`),
  `CMAKE_C_FLAGS`, `CMAKE_BUILD_TYPE` (RelWithDebInfo) and every target's `C_FLAGS` /
  `C_DEFINES` are those of the online build; only the QDLDL include path moved. MPC parity is
  bit-identical as before (`output/rust_main/r0b.md`).
- One side effect of a toolchain file: the `cmake` crate then does not pass `CMAKE_C_COMPILER`
  and CMake finds the compiler itself (`cc` on PATH, or `CC`); the same `/usr/bin/cc` here.
- cargo does not re-run osqp-sys's build script when the variable changes: it matters for a clean
  build (the image's case). Without `build_env.sh` a build still works online (the old path).
- Crates still come from crates.io (or the image's registry cache mount); only QDLDL left the
  network.

## 6. The image build (docker/orin, RUST=build)

Proven on linus with a mirror of the image's ws stage (`output/rust_main/r0b.md` section 4,
scratch workspace `/tmp/claude-1000/r0b_mirror`):

1. **Phase A**: `source /opt/ros/jazzy/setup.bash`; `colcon build --merge-install` of everything
   except the ament_cargo packages (`--packages-ignore $(colcon list -t | awk '$3 ~
   /^\((ros\.)?ament_cargo\)$/ {print $1}')`), Release.
2. **Phase B**: `source install/setup.bash` (the merged prefix: f1tenth_messages, zed_msgs,
   vesc_msgs, teleop_tools_msgs now visible to ros-env), `source src/rust/build_env.sh`, then
   `colcon build --merge-install --packages-select <the ament_cargo packages>`.

Why two phases: in a single merged build, ros-env's build script would scan the one shared
prefix while other packages are still being installed into it (it sees whatever is there at that
moment, possibly half-installed), and it does not re-run when a package appears later. In
phase B the prefix is complete and static. In phase B colcon-ros-cargo patches the workspace's
own lib crates installed earlier in the phase, so `Cargo.lock` gains two `[[patch.unused]]`
blocks (f1tenth_mpc_core, f1tenth_wall_tracker_core): a "Cargo.lock unchanged" check must
compare with the `patch` table ignored, e.g.

```bash
python3 -c 'import sys,tomllib; a,b=(tomllib.load(open(p,"rb")) for p in sys.argv[1:3]); [d.pop("patch",None) for d in (a,b)]; sys.exit(a!=b)' committed.lock src/rust/Cargo.lock
```

(`cargo metadata --locked` fails there for the same reason; run it, if at all, before phase B,
when no `.cargo/config.toml` exists.)

## 7. aarch64 notes (Thor, Orin image)

Both are on the same ROS snapshot as linus (2026-09-11, generator 0.5.0), so the crates build
unchanged: no rclrs choice to make any more, no test_msgs underlay.

1. `curl https://sh.rustup.rs -sSf | sh -s -- -y` (the image pins rustup-init 1.29.1 and toolchain
   1.99.1); rust-version 1.85 or newer.
2. `pip install --user --break-system-packages colcon-cargo colcon-ros-cargo` (the image: plus the
   cargo-ament-build wheel), `cargo install --locked cargo-ament-build --version 0.1.11`.
3. Check the generator: `dpkg -l ros-jazzy-rosidl-generator-rs` must say 0.5.0 and
   `grep -A1 metadata.ros-env /opt/ros/jazzy/share/sensor_msgs/rust/Cargo.toml` must show
   `include = true`. A different snapshot is a different generator: do not mix snapshots between
   machines.
4. `source src/rust/build_env.sh` before the build (QDLDL offline).
5. The Thor's install is a copy install: build with `--symlink-install` or rebuild after a pull
   (memory note).
6. The interface packages on the Thor / in the image are not exactly linus's: six of the image's
   (ffmpeg_image_transport_msgs, nmea_msgs, point_cloud_interfaces, rosapi_msgs, rosbridge_msgs,
   urg_node_msgs) are not installed on linus, so ros-env has never compiled them; the first
   aarch64 build is their first compile (`output/rust_main/r0b.md`, open issues).
7. Native aarch64 builds only (the message libraries are the target's). The parity results
   (bit-exact on linus) assume glibc's libm on both sides; re-run `tools/rust_parity/*/` there
   (plan P9.6: tolerance gates, bit identity not promised).
