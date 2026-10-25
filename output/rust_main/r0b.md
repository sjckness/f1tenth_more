# R0b: rclrs 0.8 on the shared ROS snapshot, offline QDLDL, image-like build, parity re-run

Plan `~/.claude/plans/fuzzy-enchanting-teapot.md` P2.5/P2.6 + P3 (R0b). Branch `r0b-rclrs08`, worktree
`~/dev_ws/f1tenth_more_r0b`, package start be849452 (main-jazzy-rust). linus (i7-9700K, 8 cores,
Ubuntu 24.04, rustc 1.99.1), 2026-10-24, shared with other agents (load 3 to 15 during the run).
linus's `ros-jazzy-*` were moved by the user to `snapshots.ros.org/jazzy/2026-09-11` at 23:09
(`/etc/apt/preferences.d/ros2-snapshot.pref`; `ros-jazzy-rosidl-generator-rs 0.5.0-1noble.20260813`);
the worktree was created after the move, so every CMake package in it was built clean against the
snapshot. Environment: `output/rust_main/env_r0b.sh` (no user-space underlay) plus
`src/rust/build_env.sh`. Every ROS process ran on an isolated localhost domain without discovery
server (97, 98; the live harnesses pick 150-220). Nothing committed: the coordinator commits.
Decisions: `~/dev_ws/f1tenth_more_main_jazzy/output/orin_container/decisions.md`, "R0b implementer".

| Item | State | Section |
|---|---|---|
| 1. rclrs =0.8.0 + ros-env =0.3.0 in the three rclrs crates; "*" message crates and the #557 workaround gone | done | 1 |
| 1a. Found by colcon test and fixed: rclrs 0.8.0's stale link-search paths when AMENT_PREFIX_PATH changes | fixed in the three crates' build.rs, proven old vs new | 1.1 |
| 2. Cargo.lock: precise update, crates.io entries only | done, reviewed | 2 |
| 3. QDLDL v0.1.8 vendored, OSQP built offline | done, proven (incl. a negative control) | 3 |
| 4. Full colcon build + image-like two-phase mirror build, install checks | pass | 4 |
| 5. cargo fmt / clippy -D warnings / test | pass (182 tests) | 5 |
| 6. Parity re-runs (offline + live) | **every result equal to the recorded one** (bit-exact where recorded bit-exact) | 6 |
| 7. colcon test of the whole worktree | pass: 4 failing IDs outside the baselines, all explained, none from R0b | 7 |
| 8. Docs (setup.md, this file; ros_packages.txt: nothing to change) | done | 8 |

**f1tenth_mpc_core and f1tenth_wall_tracker_core are unchanged**: `git diff be849452 --
src/rust/f1tenth_mpc_core src/rust/f1tenth_wall_tracker_core` is empty.

## 1. The port

**How ros-env consumes the generated crates.** The snapshot's generator 0.5.0 writes each
interface package's Rust code into `share/<pkg>/rust` with a Cargo.toml that says
`rosidl_runtime_rs = "0.6"` and `[package.metadata.ros-env] include = true` (all 56 crates in
/opt/ros/jazzy, and the workspace's own). rclrs 0.8.0 and ros-env 0.3.0 use rosidl_runtime_rs 0.7.
That mismatch does not matter: ros-env's `build.rs` (`~/.cargo/registry/src/*/ros-env-0.3.0/build.rs`)
never hands those manifests to cargo. It walks AMENT_PREFIX_PATH, reads only the `include` flag and
the names of the `*` dependencies, keeps the marked crates plus their `*` dependencies (dropping any
whose dependencies are missing), and writes an `interfaces.rs` that `include!()`s their `msg.rs` /
`msg/rmw.rs` / `srv.rs` / ... files into `ros_env::<pkg>::{msg,srv,action}[::rmw]`. The code is
compiled inside ros-env, against ros-env's rosidl_runtime_rs 0.7. The typesupport libraries come
from the `#[link(name = "<pkg>__rosidl_typesupport_c")]` attributes in the generated rmw.rs; rustc
links with `--as-needed`, so a binary keeps only the ones it uses. rclrs 0.8 takes its own messages
(rcl_interfaces, builtin_interfaces, rosgraph_msgs, action_msgs) from ros-env the same way, so one
ros-env build serves both. No workaround is needed.

Changes (diff against be849452):

| File | Change |
|---|---|
| `src/rust/{f1tenth_proximity_guard,f1tenth_costmap_boundary_rs,f1tenth_hard_stop_rs}/Cargo.toml` | `rclrs = "=0.8.0"`, `ros-env = "=0.3.0"`; the `"*"` message crates (builtin_interfaces, std_msgs, sensor_msgs, tf2_msgs, geometry_msgs, nav_msgs, ackermann_msgs, diagnostic_msgs, f1tenth_messages) and `example_interfaces` / `test_msgs` (#557) removed |
| `.../src/main.rs` (the three) | imports only: `use ros_env::sensor_msgs::msg::LaserScan;` etc., `use ros_env::builtin_interfaces;` (and `ros_env::std_msgs` in hard_stop) where the code names `builtin_interfaces::msg::Time`; one comment "rclrs 0.7 and 0.8 parameter.rs". No logic changed |
| `.../package.xml` (the three) | `example_interfaces` / `test_msgs` and the #557 comment removed; every message `<depend>` kept, with a comment that they now only order the build and put the prefixes on AMENT_PREFIX_PATH; the `<build_depend>f1tenth_messages</build_depend>` of guard and hard_stop kept, its comment rewritten (build order only; the old reason, cargo resolving the workspace through the patches, is gone) |
| `.../build.rs` (the three) | also emits `cargo:rustc-link-search=native=<prefix>/lib` for every AMENT_PREFIX_PATH entry and `rerun-if-env-changed=AMENT_PREFIX_PATH` (section 1.1); the build-info part unchanged |
| `src/rust/Cargo.toml` | the comment block: why =0.8.0 / =0.3.0, how ros-env works and what follows (build order, ~60 crates compiled, one broken crate breaks all, no #557, `[[patch.unused]]` not committed), QDLDL via build_env.sh |
| `src/rust/Cargo.lock` | section 2 |
| `tools/rust_parity/heavy/ingest_probe_rs/{Cargo.toml,Cargo.lock,src/main.rs}` | the bench probe (COLCON_IGNORE, built by hand) ported the same way; it took its message crates from colcon-ros-cargo's patches, which no longer exist, so it could not build on the snapshot otherwise. `cargo check --locked` passes; no bench run (not required) |
| `output/rust_main/env.sh` | no underlay (jc_env.sh / rust_underlay were built against the old apt set), WS = the worktree the file is in, sources `src/rust/build_env.sh`. `run_corridor_tests.sh` and `run_gate_tests.sh` source it |
| `tools/rust_parity/compare_recorded.py` | new: checks a parity re-run against the recorded JSONs (section 6) |

rclrs 0.8 behaviour visible at run time: every node now has a `/rosout` publisher (its
`NodeOptions::enable_rosout` is now honoured on Jazzy; 0.7 never created it), as rclpy and rclcpp
nodes do. Checked with `ros2 node info` on the three installed nodes (`r0b_node_startup.log`):
publishers = the node's topics + `/rosout: rcl_interfaces/msg/Log`. The nodes still end by the
signal on SIGINT (exit -2, as with 0.7; timeout_parity_hard_stop's shutdown record is unchanged).

### 1.1 Found by colcon test: rclrs 0.8.0's link-search paths go stale

The first full `colcon test` failed the colcon-cargo `unit` test of all three rclrs packages
(`build/<pkg>/cargo_test.xml`); `cargo test --target-dir build/f1tenth_proximity_guard` reproduced
it: `rust-lld: error: unable to find library -lzed_msgs__rosidl_typesupport_c` (and
`..._generator_c`). Cause: the package was built by `colcon build` with AMENT_PREFIX_PATH =
/opt/ros/jazzy + its dependencies, and tested with the whole workspace sourced. ros-env's build
script declares `rerun-if-env-changed=AMENT_PREFIX_PATH`, so it re-ran and now included zed_msgs
(with its `#[link]` attributes); rclrs 0.8.0's build script, which emits `-L <prefix>/lib` for
every AMENT_PREFIX_PATH entry, declares only `rerun-if-env-changed=ROS_DISTRO`, so its cached
search paths (2 entries, no `install/zed_msgs/lib`) were used for the link (checked in
`build/f1tenth_proximity_guard/debug/build/{rclrs,ros-env}-*/output`). The same happens to any
rebuild in a shell where the workspace was sourced after the first build, so it would have hit
developers on linus and the Thor, not only the test. A fresh build with one fixed
AMENT_PREFIX_PATH (the image's phase B) is not affected.

Fix (in our code, no patched rclrs): each rclrs crate's `build.rs` emits the same search paths
and re-runs on AMENT_PREFIX_PATH, so they always match ros-env's set. mpc_core / wall_tracker_core
(no rclrs) keep their build.rs. Proof (`r0b_linkfix_proof.log`, two scratch copies of src/rust):
build `proximity_guard_node` with `AMENT_PREFIX_PATH=/opt/ros/jazzy`, then again in the same target
dir with the workspace sourced (38 prefixes): old build.rs **exit 101** (10x each of the two zed_msgs
libraries), new build.rs **exit 0**. In the worktree: `colcon build --packages-select` of the three
with the workspace sourced (`r0b_build_linkfix.log`, the failing scenario): 3 packages, exit 0; the
re-run colcon test passes unit + fmt for all five Rust packages (`r0b_test_rust_rerun.log`). The
replay binaries' `.text` and `.rodata` are byte-identical before and after this rebuild, so the
offline parity results below hold for the final binaries; the node binaries differ (ros-env now
compiled the whole workspace's messages, as the image does) and the live harnesses were re-run
on them. Worth reporting upstream (rclrs's build.rs should declare
`rerun-if-env-changed=AMENT_PREFIX_PATH`).

## 2. Cargo.lock (precise update, reviewed)

`cd src/rust && cargo update -p rclrs --precise 0.8.0`, run before any colcon build wrote
`.cargo/config.toml` (so no patch was in play). Package/version diff against be849452:

| | Packages |
|---|---|
| updated | rclrs 0.7.0 -> **0.8.0**; rosidl_runtime_rs 0.6.1 -> **0.7.0**; ament_rs 0.2.1 -> **0.3.0** (rclrs 0.8 and ros-env need 0.3); itertools 0.8.2 -> 0.14.0 (ament_rs 0.3's) |
| added | **ros-env 0.3.0**; its build dependency cargo_toml 0.22.3 with toml 0.9.12, toml_datetime 0.7.5, toml_parser 1.1.5, toml_writer 1.1.3, serde_spanned 1.1.2, winnow 0.7.15 and 1.0.4, indexmap 2.14.2 / hashbrown 0.17.1 / equivalent 1.0.3 (toml's optional preserve_order, locked by cargo, not compiled) |
| removed | the 14 path entries of the patched message crates (ackermann_msgs, action_msgs, builtin_interfaces, diagnostic_msgs, example_interfaces, f1tenth_messages, geometry_msgs, nav_msgs, sensor_msgs, service_msgs, std_msgs, test_msgs, tf2_msgs, unique_identifier_msgs) and all 42 machine-specific `[[patch.unused]]` blocks |
| unchanged | everything else (tokio 1.53.3, uuid 1.28.0, serde, osqp-sys 1.0.1, cmake 0.1.58, cc 1.10.0, ...); `cargo update --dry-run` would move tokio and uuid, deliberately not done |

`cargo metadata --locked` passes on the committed file (sha256 `02efb461fca4...`). The first build,
the cargo gates and the tests left it untouched (`cmp` with the saved copy after each): the
generator 0.5.0 packages register no `rust_packages` ament resource, so colcon-ros-cargo's patch
list was empty (`.cargo/config.toml` = `[patch.crates-io]` only). Only the five members register
one: the `--packages-select` rebuild with the workspace sourced (section 1.1) patched all five and
cargo appended five `[[patch.unused]]` blocks (nothing else changed, tomllib comparison); the file
was restored to the committed content afterwards. In a merged install the same happens for the
crates installed earlier in phase B (section 4).
The probe's own lock (`tools/rust_parity/heavy/ingest_probe_rs/Cargo.lock`) got the same update
plus serde 1.0.229 / serde_core / serde_derive (cargo_toml needs serde; the probe had none);
`[[patch.unused]]` removed there too.

## 3. QDLDL without network

- `src/rust/third_party/qdldl/`: QDLDL tag `v0.1.8` = commit
  `138fdac58b9cd1c4137ff1b99152c8108a6cff5b` (lightweight tag; `git ls-remote` and the clone agree),
  the whole tree minus `.git` (28 files, Apache-2.0 `LICENSE` included). Its git tree hash is
  upstream's `7bbce655e64e74c476ca4694c06d7bfb08a5cf3d` (checked by `git write-tree` in a throwaway
  repo), so after the commit `git rev-parse HEAD:src/rust/third_party/qdldl` must print that. The
  pre-R0b online builds fetched exactly this tree (`diff -r` of their `_deps/qdldl-src`: identical,
  same commit). `git check-ignore` matches none of the 31 files (none silently dropped by
  `.gitignore`).
- `src/rust/third_party/COLCON_IGNORE` (QDLDL has a CMakeLists.txt), `third_party/README.md`
  (what, why, sha, how to verify and update).
- `src/rust/third_party/osqp_offline.cmake`: toolchain file, `FETCHCONTENT_SOURCE_DIR_QDLDL` =
  the vendored dir, `FETCHCONTENT_FULLY_DISCONNECTED` ON (cache, FORCE), FATAL_ERROR if the dir is
  missing.
- `src/rust/build_env.sh` (bash and zsh; `bash -n` / `zsh -n` clean, shellcheck is not installed on
  linus): exports `CMAKE_TOOLCHAIN_FILE_<rustc host triple, - -> _>` =
  that file. The `cmake` crate (0.1.58) reads `CMAKE_TOOLCHAIN_FILE_<target>`; CMake reads only
  plain `CMAKE_TOOLCHAIN_FILE`, so no colcon CMake package sees it.

Proof (`r0b_offline_qdldl.log`): every OSQP build of the session (colcon worktree build, the
cargo gates' target dir, the mirror's phase B) ran with `GIT_ALLOW_PROTOCOL=file` and has
`_deps/qdldl-build` only (no `qdldl-subbuild`, no `qdldl-src`), `CMAKE_TOOLCHAIN_FILE` = the file,
`FETCHCONTENT_SOURCE_DIR_QDLDL` / `qdldl_SOURCE_DIR` = the vendored dir, FULLY_DISCONNECTED ON.
**Negative control**: the same `cargo build -p f1tenth_mpc_core` without `build_env.sh`, same
`GIT_ALLOW_PROTOCOL=file`, fresh target dir: `_deps/qdldl-subbuild` appears, `git clone` fails with
`fatal: transport 'https' not allowed`, exit 101. Build equivalence: `CMAKE_C_COMPILER` /usr/bin/cc,
`CMAKE_C_FLAGS`, `CMAKE_BUILD_TYPE` RelWithDebInfo equal to the online build; the generated
`flags.make` of OSQPLIB, osqpstatic and qdldlobject differ only in the QDLDL include path
(`_deps/qdldl-src/include` -> `third_party/qdldl/include`); `qdldl_types.h` byte-identical. The
`cmake` crate no longer passes `-DCMAKE_C_COMPILER` when a toolchain file is set (CMake finds the
same /usr/bin/cc). Result-level: `parity_solver.py --sweep 3000` with the R2 live capture is
bit-identical to the R2 record on every layer (section 6).

## 4. Builds

**Worktree** (`r0b_build.log`): `colcon build --symlink-install --parallel-workers 3
--packages-ignore $SKIP` (env_r0b.sh + build_env.sh, `GIT_ALLOW_PROTOCOL=file`, fresh worktree):
**37 packages finished, exit 0, 3 min 45 s**. f1tenth_messages 1 min 14 s; the rclrs packages
2 min 1 s (guard), 2 min 23 s (costmap), 2 min 22 s (hard_stop), built in parallel with load up
to 14 (ros-env compiles the whole message set once per package target dir); f1tenth_mpc_core
12.8 s, f1tenth_wall_tracker_core 6.6 s. After the build.rs fix, the three rclrs packages
rebuilt with the workspace sourced (`r0b_build_linkfix.log`): 3 packages, exit 0, 52 s.

**Image-like mirror** (`r0b_mirror/`, scratch workspace `/tmp/claude-1000/r0b_mirror`, the
worktree's `src/` and `scripts/env` copied, then built from nothing). Run twice, both passing:
before the build.rs fix (`r0b_mirror/pre_linkfix/`: phase A 124 s, phase B 170 s) and with the
final sources (numbers below):
- Phase A: `source /opt/ros/jazzy/setup.bash`; `colcon build --merge-install --parallel-workers 3
  --packages-skip f1tenth_sim f1tenth_more husarion_gz_worlds --packages-ignore $SKIP <the 5
  ament_cargo packages> --cmake-args -DCMAKE_BUILD_TYPE=Release` (the Dockerfile's flags minus
  ccache, which linus lacks; `$SKIP` = linus's zed_*/vesc*): **31 packages, exit 0, 97 s**.
- Phase B: `source install/setup.bash` (AMENT_PREFIX_PATH = the merged install + /opt/ros/jazzy;
  4 workspace message crates in `install/share/*/rust`), `source src/rust/build_env.sh`,
  `GIT_ALLOW_PROTOCOL=file`, `colcon build --merge-install --packages-select <the 5>
  --packages-ignore $SKIP`: **5 packages, exit 0, 129 s** (rclrs packages 1 min 51 s to 2 min
  4 s each, in parallel).
- In phase B colcon-ros-cargo wrote two patches (the merged prefix holds the `rust_packages`
  resources of f1tenth_mpc_core and f1tenth_wall_tracker_core, installed earlier in the phase) and
  cargo rewrote the `[[patch.unused]]` blocks of `Cargo.lock` accordingly; nothing else changed:
  `Cargo.lock.before` (copied while the worktree's file carried five such blocks, section 2) and
  `.after` (two blocks) both equal `Cargo.lock.committed` with the `patch` table ignored. In the
  first run (copied from the clean file) the build added exactly two blocks.
  `cargo metadata --locked --offline` on the committed lock with those patches present: **exit
  101** ("cannot update the lock file ... --locked"). The tomllib comparison with the `patch`
  table ignored: exit 0; against the pre-R0b lock: exit 1.

**Install checks** (`r0b_install_checks.log` isolated, `r0b_mirror/mirror_install_checks.log`
merged; same script):

| Check | Worktree (symlink, isolated) | Mirror (merged, Release) |
|---|---|---|
| CLAUDE.md `ldd -r` scan of every ELF under `*/lib/*` (message symbols filtered) | **empty** (58 ELF files) | **empty** |
| Rust artefacts with `test_msgs` / `example_interfaces` in DT_NEEDED or ldd | **none** (8 artefacts) | **none** |
| `ldd` "not found" in a Rust artefact | none | none |
| `*_replay` and the two cdylibs (C ABI build_info) report | `cargo profile debug, opt-level 3, debug_assertions on` (5/5) | the same (5/5) |
| node binaries log at startup | `... build: cargo profile debug, opt-level 3, debug_assertions on` (3/3, `r0b_node_startup.log`) | |

DT_NEEDED of the nodes: rcl, rcl_action, rcl_yaml_param_parser, rcutils, rmw, rosidl_runtime_c,
the typesupport/generator libs of the messages they use, rosgraph_msgs (clock) and rcl_interfaces
(parameters), libc/libm/libgcc_s. Sizes: nodes 62-64 MB (debuginfo of the included message code),
replays 9-12 MB, libf1tenth_mpc_core.so 6.8 MB, libf1tenth_wall_tracker_core.so 11.8 MB.

### 4.1 For the Dockerfile (W2 / the coordinator; not edited here, W2 owns it)

The ws stage with `RUST=build` should do exactly what the mirror did:

```bash
source /opt/ros/jazzy/setup.bash
rust_pkgs=$(colcon list -t | awk '$3 ~ /^\((ros\.)?ament_cargo\)$/ {printf "%s ", $1}')
colcon build --merge-install --packages-skip f1tenth_sim f1tenth_more husarion_gz_worlds \
  --packages-ignore ${rust_pkgs} --cmake-args -DCMAKE_BUILD_TYPE=Release ...      # phase A (RUST=skip: the same)
if [ "${RUST}" = build ]; then
  cp src/rust/Cargo.lock /tmp/Cargo.lock.committed
  source install/setup.bash && source src/rust/build_env.sh                        # complete prefix; QDLDL offline
  GIT_ALLOW_PROTOCOL=file colcon build --merge-install --packages-select ${rust_pkgs}   # phase B
  python3 -c 'import sys,tomllib; a,b=(tomllib.load(open(p,"rb")) for p in sys.argv[1:3]); [d.pop("patch",None) for d in (a,b)]; sys.exit(a!=b)' \
    /tmp/Cargo.lock.committed src/rust/Cargo.lock                                  # lock unchanged
fi
rm -rf build log .cargo
```

Comments in the current Dockerfile that R0b makes obsolete (rust-build stage header): "Both crates
pin rclrs =0.7.0 ... will not resolve them here until they are ported to rclrs 0.8" (done: three
crates on 0.8.0); "ros2-rust#557: rclrs links test_msgs' and example_interfaces' typesupport
libraries" (no longer: rclrs 0.8 links neither; they stay in ros_packages.txt because the Thor has
them); "--merge-install with ament_cargo packages has not been tried" (tried: two phases as above);
"OSQP's CMake clones QDLDL v0.1.8 from github.com at build time ... needs network" (no longer, with
build_env.sh; crates.io still needs network or the registry cache). `docker/orin/ros_packages.txt`
has no comments (it is read with `$(cat ...)` into apt-get, so it cannot hold any): nothing to change.

## 5. cargo gates (`r0b_cargo.log`, from src/rust, env_r0b.sh + build_env.sh)

| Command | Result |
|---|---|
| `cargo fmt --all --check` | **exit 0** |
| `cargo clippy --workspace --all-targets -j 3 -- -D warnings` | **exit 0** (27 s) |
| `cargo test --workspace -j 3` | **exit 0: 182 passed** (costmap 18, hard_stop 57 + 1 (test_cases), mpc_core 47, guard 21, wall_tracker_core 38) |

## 6. Parity re-runs on the snapshot

Outputs in `r0b_parity/` (JSON + log per harness, `runs.log`). Offline harnesses on domain 97
(isolated, localhost); the live ones pick a random localhost domain 150-220. Checked against the
recorded JSONs with `tools/rust_parity/compare_recorded.py output/rust_main/r0b_parity` (new,
`ament_flake8` / `ament_pep257` clean): **47 of 47 checks "equal to recorded"**
(`r0b_parity/compare_recorded.txt`). Timing benches not run (not required; the machine was shared).
The corridor corpora and the R2 live capture are the recorded runs' own inputs (this session's
scratchpad, `r1/suites2/capture`, `r2/capture_suite`), so those layers re-ran on identical inputs.

| Harness | Recorded | R0b on the snapshot |
|---|---|---|
| `proximity/parity_proximity.py` (seed 20261018, every bag) | R0: 3,184 synthetic / 145,368 bag / 101,813 bag+obstacle cases (238 bags), trips 444 / 9,841 / 30,858 on both sides, 0 trip diffs, min_front 244,462 / 244,462 bit-identical, 12 named fail-safe differences | **identical, number for number** (the same 238 bags, same per-bag counts) |
| `proximity/timeout_parity.py` | 20 cases x 4 implementations (2026-10-18) | 80/80 cells agree on behaviour (stop or not; total stops within one at the window edge; first Rust stale stop 200-260 ms). First run on the rebuilt nodes: harness process aborted at interpreter exit after writing complete results ("terminate called without an active exception", rclpy teardown; results 80/80 too, kept as `timeout_parity_run1_abort.json`); re-run exit 0 |
| `proximity/live_guard_test.py --phase-a-s 30` | R0: 238 trips, 238/238 stops both sides, shadow 0, B 36 stale stops, C 0, D 41 (40 invalid_beams + 1 window-edge stale) | 237 trips (the 30 s loop sent 1,189 scans, R0 1,190), **237/237 stops on both sides, 0 missing**, shadow 0; B 36/36 stale stops, first after 206-218 ms (R0 228), max gap 50.1 ms; D 40 `invalid_beams` stops, + 1 window-edge `stale` in 3 of 4 runs (as R0); C: 1 stop counted "after 100 ms" in 4 of 4 runs on the final nodes (`live_guard.json`, `live_guard_run2/3.json`, the instrumented run). Instrumented copy (`scratchpad`, not committed): that stop is the guard's `stale` decision 214 ms after the **last** clear scan of phase C (scan_age 213.9 ms, received 2,208 ms into the phase, scans ended at 1,994 ms, largest publish gap 25.4 ms): the harness's 200 ms tail window, the edge effect r0.md section 2 describes for phase D. No stop while scans flowed |
| `costmap_boundary/parity_costmap_boundary.py` | R0: 804 synthetic / 7 malformed / 155,353 bag poses (970 maps, 235 bags), 0 diffs, 463,062 of 466,209 constraints bit-identical, max 2.2e-16 / 8.9e-16, clearance 0 ulp | **identical** (same bags, maps, poses, bit-identical share 99.3250 %, same 5 Python exceptions + 1 intended) |
| `costmap_boundary/live_costmap_test.py` | R0: phases 1-3 identical py/rust | phases 1-3 identical py vs rust **and equal to R0's values** (clearance 3.4081037 / 1.4317597 / 6.0, same constraints); CPU phase run at the default 30 s (R0 60 s), not gated |
| `hard_stop/parity_hard_stop.py` + `make_test_cases.py --check` | R4: 1,130,585 ops, 0 diffs | **1,130,585 ops, 0 diffs, every group's ops / stops / engages / scans equal**; committed test cases current |
| `hard_stop/live_hard_stop_test.py` | R4: 26 checks ok | 26/26 ok (same engage / release scans, byte-identical diagnostics) |
| `hard_stop/timeout_parity.py` | R4: 23 rows agree, pass | 23/23 rows agree, pass, same shutdown record |
| `corridor/parity_corridor.py --sweep 10000` + 3 corpora | R1 GATE PASS | **identical output lines** (rebuilds 1,344/1,344, sweep 8,907 + 30 NaN-sign + 1,063 matched exceptions, replays 1,383 / 130 / 33, 30,446 queries) |
| `corridor/closed_loop_rigs.py` | R1: 20/20 golden per impl, 1,218 solves | 20/20 py, 20/20 rust, 1,218 solves |
| `corridor/run_corridor_tests.sh` (py / rust / substitute / capture) | R1: identical outcomes (mpc_controller then 1,546 tests) | **IDENTICAL OUTCOMES** for every test ID in all four modes: mpc_controller 1,687 (1,669 pass, 15 fail, 3 skipped in each), f1tenth_logger 336 (331 / 5), f1tenth_params 80 (80); capture mode compared every call in process: 0 mismatches over 3,932 geometry calls and 9,374,314 corridor queries. First pass: f1tenth_logger's `test_logger_node_roundtrip.py` failed in substitute (1 test) and capture (module fixture "the logger node never subscribed", 24 errors); that module then failed and passed in **every** mode, plain py included (2 rounds, `roundtrip_rate.sh`): its conftest pins every run to ROS domain 91, which other agents' logger tests use too. Re-run of the two plugin modes with `F1TENTH_TEST_DOMAIN=86` (the test's documented override): identical (`r0b_corridor_suites_logger_rerun.log`; first pass kept in `r0b_corridor_suites_run1.log`) |
| `mpc/parity_solver.py --sweep 3000 --capture-dir <R2 capture>` | R2: 26/26 gates; L1 42,909 QPs 0 ulp, sha 723/723; L2 42,909 bit-identical; L3 fixture 723, sweep 2,599 + 401 exceptions, capture 39,587 (39,401 = recorded) | **26/26, every L1/L2/L3 count equal** (offline-QDLDL build) |
| `mpc_controller/test/closed_loop_parity.py` | R2: 32/32 bit-identical, golden py 20/20, rust 20/20 | **32/32, golden 20/20 + 20/20**, every run record equal |
| `wall_tracker/{numerics,fusion_cases}.py` | R6: 1,120,028 / 3,000+3,000+600 cases, 0 mismatches | identical counts, 0 mismatches, same coverage |
| `wall_tracker/parity_wall_tracker.py` car / every / full / doors | R6: 8 / 1 / 4 / 3 runs, 0 differences | 2,859 / 2,292 / 913 / 1,277 steps, 19,754 / 14,336 / 6,386 / 10,221 messages: **every per-run field equal, 0 differences** |

The live runs listed are those on the final (rebuilt) nodes; the runs on the first build are in
`r0b_parity/pre_linkfix/` with the same outcomes (guard phase C there: 1 stop, inside 100 ms;
the 20 s default phase-A run: 158/158 trips stopped, 2 stops arrived after the next trip scan
under load, i.e. > 25 ms).

## 7. colcon test of the whole worktree

`colcon test --packages-ignore $SKIP --parallel-workers 3 --test-result-base
output/rust_main/r0b_test_results` (env_r0b.sh + build_env.sh, `ROS_DOMAIN_ID=97`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, no discovery server; `r0b_test.log`): exit 0, 9 min 46 s.
`colcon test-result --all`: **5,669 tests, 0 errors, 49 failures, 101 skipped**
(`r0b_test_result_all.txt`; jazzy_main baseline 5,373 / 1 / 45 / 101). CTest packages as in the
baseline: ackermann_mux 3, vesc_ackermann 3, zed_msgs 1 (upstream lint). pytest packages
(`tools/r3_failing_ids.py`, `r0b_test_failing_ids.txt`): 42 failures + errors.

| Compared with | New failing IDs | Fixed |
|---|---|---|
| `output/jazzy_main/test_failing_ids.txt` (all packages, 2026-10-23) | 4 (`r0b_test_ids_vs_jazzy_main.log`) | 1: `error joy_teleop::joy_teleop::` (collection) |
| `output/rust_main/merged/test_failing_ids.txt` (14 packages) | 26, of which 22 are in the jazzy_main list (lint of diagnostics/logger/llm, the llm planner stubs, the logger's checkout-name tests); the other 4 are the same 4 | 1: f1tenth_bringup `test_rust_switches::test_stack_bringup_forwards_both_mpc_switches_to_navigation` |

The 4 IDs in neither baseline, each explained; none comes from R0b (R0b changes no Python and no
launch file):

| ID | Cause | Kind |
|---|---|---|
| `joy_teleop ... test_flake8::test_flake8` | the baseline could not collect joy_teleop at all: `ModuleNotFoundError: No module named 'test_msgs'` (its pytest.xml); test_msgs is an apt package since the snapshot move, so the 18 tests now run, 16 pass. The submodule's own lint | snapshot / environment |
| `joy_teleop ... test_parameter_failures::test_parameter_failures` | same: now collected; the submodule's own failure (`test_no_buttons_or_axes`: joy_teleop does not shut down within 10 s). Both IDs were in `output/rust/test_baseline_failing_ids.txt` (2026-10-18, the run that had the test_msgs underlay) | snapshot / environment |
| `mpc_controller ... test_wall_turn_increment.TestCommitLead::test_the_lead_moves_the_trigger_out_by_exactly_the_lead` | the class arrived with the car-sync merge 81461f04 (the car's wall_turn commits), after both baselines; `_step(...).committed` is False where the test expects the commit. Same file and symptom as the three baseline failures listed in `output/mpc_known_failures.md` #10-12 | real, pre-existing (car-sync), not R0b |
| `mpc_controller ... TestChatter::test_without_the_ratchet_the_same_jitter_would_retreat` | passed at d58c0f8e; asserts `d_avail > 0`, which the car's 6137165f ("wall_turn no longer freezes once the car is inside the margin") turned into 0 = "distance cap lifted"; came in with 81461f04 | real, pre-existing (car-sync), not R0b |

The colcon-cargo results of the five Rust packages are written to `build/<pkg>/cargo_test.xml`, not
to the result base. In the first run `unit` failed for the three rclrs packages (section 1.1); after
the fix, `colcon test --packages-select <the 5>`: **unit + fmt pass for all five**
(`r0b_test_rust_rerun.log`).

## 8. Gate summary

All from `bash -c 'source output/rust_main/env_r0b.sh; source src/rust/build_env.sh; cd $WS; ...'`
unless noted; scripts in this session's scratchpad (`r0b/*.sh`), logs here.

| Gate | Command | Result |
|---|---|---|
| port builds | `GIT_ALLOW_PROTOCOL=file colcon build --symlink-install --parallel-workers 3 --packages-ignore $SKIP` | **37 packages, exit 0** (`r0b_build.log`) |
| rebuild with the workspace sourced (after the fix) | same, `--packages-select` the three rclrs packages | **3 packages, exit 0** (`r0b_build_linkfix.log`) |
| image-like two-phase build | mirror, phase A merge-install without the ament_cargo packages, phase B only them (section 4) | **31 + 5 packages, exit 0**, twice (`r0b_mirror/`) |
| ldd -r scan (CLAUDE.md filter) | `install_checks.sh` on the worktree and the mirror | **empty** on both (58 ELF in the worktree) |
| no test_msgs | DT_NEEDED + ldd of every Rust artefact | **none** links test_msgs / example_interfaces, no "not found" |
| opt-level 3 | replay binaries, cdylib build_info, node startup log | **3 of 3 replays, 2 of 2 cdylibs, 3 of 3 nodes** |
| QDLDL offline | `_deps` listing + CMakeCache of every OSQP build; build under `GIT_ALLOW_PROTOCOL=file`; negative control | no `qdldl-subbuild` anywhere; all builds pass; without build_env.sh the clone is refused, exit 101 (`r0b_offline_qdldl.log`) |
| vendored tree | `git write-tree` of the copy | `7bbce655e64e74c476ca4694c06d7bfb08a5cf3d` = upstream v0.1.8 |
| Cargo.lock | `cargo update -p rclrs --precise 0.8.0`, review, `cargo metadata --locked` | crates.io entries + members only, 0 `[[patch.unused]]`; locked OK (section 2) |
| cargo fmt | `cd src/rust && cargo fmt --all --check` | **exit 0** |
| clippy | `cargo clippy --workspace --all-targets -j 3 -- -D warnings` | **exit 0** |
| cargo test | `cargo test --workspace -j 3` | **exit 0, 182 passed** |
| colcon test (Rust) | `colcon test --packages-select <the 5>` -> `build/<pkg>/cargo_test.xml` | **unit + fmt pass, 5 of 5** (after the fix) |
| colcon test (all) | section 7 | 5,669 tests; 4 failing IDs outside the baselines, explained, none from R0b |
| parity | section 6 + `compare_recorded.py` | **47 of 47 equal to recorded**; live harnesses equal in decisions |
| MPC parity on the offline build | `parity_solver.py --sweep 3000 --capture-dir <R2 capture>` | **26/26**, L1/L2/L3 counts = R2 |
| bench probe | `cd tools/rust_parity/heavy/ingest_probe_rs && cargo check --locked` | exit 0 |
| new Python | `ament_flake8` / `ament_pep257 tools/rust_parity/compare_recorded.py` | no problems |
| new shell | `bash -n` / `zsh -n src/rust/build_env.sh`, sourced in bash and zsh | OK (shellcheck not installed on linus) |
| git visibility | `git check-ignore --no-index` on every new file | nothing ignored (31 files under third_party) |

## 9. Open issues

- **Six interface packages of the image's `ros_packages.txt` have never been compiled by ros-env**:
  ffmpeg_image_transport_msgs, nmea_msgs, point_cloud_interfaces, rosapi_msgs, rosbridge_msgs,
  urg_node_msgs are not installed on linus (not fetched: network scope, no apt). ros-env compiles
  every crate on the prefix, so the first RUST=build image is their first compile; a failure there
  breaks all three rclrs packages (fallback RUST=skip, plan "Risks"). Conversely, 15 crates linus
  has are not in the image (actuator_msgs, moveit_msgs, ros_gz_interfaces, ...): fewer to compile.
- **Dockerfile** (W2 / coordinator): the phase-B commands, the lock check and the obsolete comments
  of section 4.1. `--cargo-args --locked` must not be used.
- **`Cargo.lock` before the commit**: any rebuild with the workspace sourced appends five
  `[[patch.unused]]` blocks (section 2); the file is restored now (sha256 `02efb461fca4...`), but
  check it before committing (the tomllib one-liner, or `grep -c patch.unused` = 0).
- `src/rust/f1tenth_mpc_core/package.xml`'s comment on its `<build_depend>f1tenth_messages` still
  gives the pre-R0b reason (cargo resolving the workspace through the patches); left byte-identical
  because the crate is to stay unchanged. The build_depend itself is harmless (build order).
- Upstream: rclrs 0.8.0's build.rs should declare `rerun-if-env-changed=AMENT_PREFIX_PATH`
  (section 1.1); our build.rs works around it.
- f1tenth_logger's `test_campaign/conftest.py` pins every run to ROS domain 91; parallel runs from
  other worktrees collide (section 6, corridor suites). `F1TENTH_TEST_DOMAIN` avoids it.
- `docker/orin/build_checks.sh` could also assert that no Rust artefact needs test_msgs (cheap,
  `readelf -d`), now that nothing should.
- Not run: timing benches (not required), Isaac e2e per switch (optional in the plan).

## Files

Changed: `src/rust/Cargo.toml`, `src/rust/Cargo.lock`,
`src/rust/{f1tenth_proximity_guard,f1tenth_costmap_boundary_rs,f1tenth_hard_stop_rs}/{Cargo.toml,package.xml,build.rs,src/main.rs}`,
`tools/rust_parity/heavy/ingest_probe_rs/{Cargo.toml,Cargo.lock,src/main.rs}`, `output/rust_main/env.sh`,
`output/rust/setup.md`.
New: `src/rust/build_env.sh`, `src/rust/third_party/{COLCON_IGNORE,README.md,osqp_offline.cmake,qdldl/**}`
(31 files), `tools/rust_parity/compare_recorded.py`, this file, and the outputs in `output/rust_main/`:
`r0b_build.log`, `r0b_build_linkfix.log`, `r0b_linkfix_proof.log`, `r0b_cargo.log`,
`r0b_install_checks.log`, `r0b_node_startup.log`, `r0b_offline_qdldl.log`, `r0b_mirror/` (final run,
`pre_linkfix/` the first), `r0b_parity/` (JSON + logs, `compare_recorded.{txt,json}`, `runs.log`,
`pre_linkfix/`), `r0b_test.log`, `r0b_test_result_all.txt`, `r0b_test_failing_ids.txt`,
`r0b_test_ids_vs_{jazzy_main,merged}.log`, `r0b_test_rust_rerun.log`,
`r0b_corridor_suites_run1.log` + `_run1_outcomes.json` (first pass),
`r0b_corridor_suites_logger_rerun.log` + `r0b_corridor_suites_outcomes.json` (final comparison).
Under 2 MB in all. Not to commit: `output/rust_main/r0b_test_results/` (test XML),
`output/rust_main/env_r0b.sh` (the coordinator's).
