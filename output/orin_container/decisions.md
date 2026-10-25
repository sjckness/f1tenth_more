# Decisions during the Orin container run (append: question, decision, reason)


## W2 (image + host run model), 2026-10-24
- Q: Which mount points does the car container get (W1 preflight should check exactly these)? D: compose.yaml (service car,
  container f1tenth_car) bind-mounts, host state root `/scratch/fabiocar/f1tenth_container` = S:
  `/dev`; S/ros -> ~/.ros; S/archive (or ~/f1tenth_archive with `--archive shared`) -> ~/f1tenth_archive;
  S/ctl -> ~/.local/state/f1tenth_ctl; S/gateway -> ~/.local/state/f1tenth_gateway;
  S/archive_trigger -> ~/.local/state/f1tenth_archive_trigger; S/zed_settings -> /usr/local/zed/settings;
  S/hf_cache -> ~/.cache/huggingface; S/missions/{mission_reports,mission_videos,llm_generated,corridors_jsons} ->
  /opt/f1tenth_ws/src/f1tenth_behavior/mission_reports, /opt/f1tenth_ws/mission_videos,
  /opt/f1tenth_ws/install/share/f1tenth_behavior/missions/llm_generated, /opt/f1tenth_ws/src/f1tenth_control/corridors_jsons;
  read-only: ~/projects/llm/models, ~/.config/f1tenth_gateway/env (file), ~/.local/share/f1tenth_gateway/www,
  ~/.config/f1tenth_car (dir, optional stack_args), /etc/localtime; /run/jtop.sock; named volumes
  f1tenth_car_zed_resources -> /usr/local/zed/resources, f1tenth_car_cuda -> /var/cache/f1tenth/cuda,
  f1tenth_car_trt -> /var/cache/f1tenth/trt. Env set by compose: F1T_CONTAINER_ROLE, F1T_AUTOSTART (default none),
  F1T_CTL_ENV=/opt/f1tenth_ws/src/f1tenth_ctl/config/ctl_container.env, F1T_GATEWAY_VENV=/opt/f1tenth_gateway/venv,
  F1T_GATEWAY_CONFIG=/opt/f1tenth_ws/docker/orin/container/gateway.yaml,
  F1T_GATEWAY_ENV_FILE=/home/fabiocar/.config/f1tenth_gateway/env, F1TENTH_IMAGE_REF, F1TENTH_IMAGE_ID.
  Command: `python3 /opt/f1tenth_ws/docker/orin/container/f1tenth_container_main.py` (entrypoint.sh first, tini PID 1).
  Image paths: docker/orin/container/* -> /opt/f1tenth_ws/docker/orin/container/ (healthcheck.sh there too,
  Dockerfile HEALTHCHECK), scripts/export_yolo_engine.sh -> /opt/f1tenth_ws/scripts/, CAR_SYNC.json and SOURCES at
  /opt/f1tenth_ws/. R: the contract names the paths repo-relative with the repo at /opt/f1tenth_ws; one list for both
  packages.
- Q: HF cache read-only (plan P6.1)? D: read-write container state dir S/hf_cache. R: faster-whisper downloads its model
  on first use; the Orin host has no whisper cache (known_bugs G6: no faster-whisper there), so a read-only mount
  could never be filled.

## R0b implementer (worktree f1tenth_more_r0b, branch r0b-rclrs08, start be849452), 2026-10-24
- Q: How does ros-env 0.3.0 consume the generated crates whose Cargo.toml says rosidl_runtime_rs = "0.6"? D: no
  workaround needed. R: ros-env's build.rs never resolves those manifests with cargo: it reads only
  `[package.metadata.ros-env] include` and the `*` dependency names, then include!()s share/<pkg>/rust/src/*.rs
  into its own crate, compiled against ros-env's rosidl_runtime_rs 0.7 (= rclrs 0.8's); the link names come from
  `#[link(name = ...)]` in the generated rmw.rs.
- Q: Keep the `<build_depend>f1tenth_messages</build_depend>` in guard / hard_stop (its old reason, cargo resolving
  the workspace through colcon-ros-cargo patches, is gone)? D: keep, comment rewritten to "build order only".
  R: task says keep package.xml depends for build order; ros-env compiles whatever is on AMENT_PREFIX_PATH, so
  waiting for the workspace's f1tenth_messages avoids compiling it half-installed in a rebuild.
- Q: f1tenth_mpc_core/package.xml carries the same now-outdated comment on its build_depend. D: left byte-identical.
  R: the task requires f1tenth_mpc_core and f1tenth_wall_tracker_core unchanged; flagged as a follow-up (comment only).
- Q: Edit docker/orin/Dockerfile (obsolete comments: rclrs 0.7 blocker, #557, QDLDL network fetch; phase-B build)?
  D: no; exact contract and replacement text in output/rust_main/r0b.md for W2 / the coordinator. R: W2 owns the
  Dockerfile in another worktree; a parallel edit would conflict.
- Q: docker/orin/ros_packages.txt "#557 note"? D: unchanged. R: the file has no comments and cannot hold any (read with
  `$(cat ...)` into apt-get); ros-jazzy-test-msgs / example-interfaces stay because the list is the Thor's set.
- Q: output/rust_main/env.sh (sourced by run_corridor_tests.sh, run_gate_tests.sh) sources jc_env.sh +
  rust_underlay and a fixed WS. D: rewritten: no underlay, WS = the worktree the file is in, sources
  src/rust/build_env.sh. R: the underlays were built against the old apt set (never source them again) and the
  harnesses must run against this worktree.
- Q: Vendor all of QDLDL v0.1.8 or only what OSQP compiles? D: the whole tag tree minus .git (28 files).
  R: the copy then has upstream's git tree hash 7bbce655e64e (checked in a throwaway repo), so
  `git rev-parse HEAD:src/rust/third_party/qdldl` verifies it after the commit.
- Q: tools/rust_parity/heavy/ingest_probe_rs (bench probe, COLCON_IGNORE) also pinned rclrs 0.7 and took its message
  crates from the colcon patches, which no longer exist. D: ported the same way (rclrs =0.8.0, ros-env =0.3.0,
  precise lock update). R: otherwise it cannot build on the snapshot at all; no bench run (not required).
- Q: How should the image (W2) check "Cargo.lock unchanged" after the Rust build? D: compare the committed lock and
  the post-build lock with the `patch` table removed (tomllib one-liner in output/rust/setup.md section 6); do not
  use `--locked` / `colcon --cargo-args --locked`. R: tested in the merge-install mirror: phase B makes
  colcon-ros-cargo patch the workspace's own lib crates (f1tenth_mpc_core, f1tenth_wall_tracker_core installed
  earlier in the phase), cargo appends two [[patch.unused]] blocks, and `cargo metadata --locked` then exits 101;
  the tomllib comparison passes on that file and fails (exit 1) on the pre-R0b lock (negative control).
- Q: The live guard harness's default phase A is 20 s, R0 recorded 30 s. D: re-ran with `--phase-a-s 30` and
  compared that run (the 20 s run is kept as live_guard_20s.json). R: like-for-like counts.
- Q: Live and timeout harnesses are timing-sensitive (watchdog 50 ms, machine shared with other agents): what is
  "equal to the recorded"? D: decisions and outcomes must match (every trip stopped on both sides, shadow 0 commands,
  stale/no-stale per case, identical costmap outputs, identical hard_stop engage/release scans and diagnostics,
  the harnesses' own agree flags); stop counts in a timeout window may differ by one stop at the window edge, and
  latencies/CPU are reported, not gated. R: the task excludes timing benches; the recorded runs show the same
  window-edge effect (r0.md section 2).
- Q: Six interface packages of the image's ros_packages.txt (ffmpeg_image_transport_msgs, nmea_msgs,
  point_cloud_interfaces, rosapi_msgs, rosbridge_msgs, urg_node_msgs) are not installed on linus, so ros-env has
  never compiled their generated crates. Fetch them to test? D: no (network scope is crates.io + the QDLDL tag;
  no apt); recorded as an open risk for the first RUST=build image (fallback RUST=skip). R: rules.
- Found (real, fixed): `colcon test` of the three rclrs packages failed to link ("unable to find library
  -lzed_msgs__rosidl_typesupport_c"). Cause: rclrs 0.8.0's build.rs emits `-L <prefix>/lib` for every
  AMENT_PREFIX_PATH entry but declares only `rerun-if-env-changed=ROS_DISTRO`, while ros-env re-runs on
  AMENT_PREFIX_PATH: a second cargo run in the same target dir with more prefixes (workspace sourced, colcon test,
  a rebuild after sourcing) includes more message crates whose libraries rclrs's stale search paths miss.
  Q: fix where? D: in the three rclrs crates' own build.rs (emit the same `-L` paths, `rerun-if-env-changed=
  AMENT_PREFIX_PATH`); mpc_core / wall_tracker_core build.rs untouched. R: no patched copy of rclrs needed; the
  crate's build script is the one place that re-runs with ros-env. Proven old-vs-new in scratch copies
  (r0b_linkfix_proof.log). The fresh image build (one AMENT_PREFIX_PATH) is not affected either way; worth an
  upstream report to ros2-rust (rclrs build.rs should declare rerun-if-env-changed=AMENT_PREFIX_PATH).
- Q: Four failing IDs are in neither baseline. D: explained, none from R0b: joy_teleop test_flake8 and
  test_parameter_failures (the jazzy_main baseline could not collect joy_teleop: "No module named 'test_msgs'";
  test_msgs is apt now, both are the submodule's own failures, already in output/rust/test_baseline_failing_ids.txt
  of 2026-10-18 which had test_msgs); mpc_controller test_wall_turn_increment TestCommitLead (added by the car-sync
  merge 81461f04) and TestChatter::test_without_the_ratchet_the_same_jitter_would_retreat (passes at d58c0f8e;
  asserts d_avail > 0, which the car's 6137165f made 0 = "distance cap lifted"): car-sync, after both baselines,
  pure Python. R: R0b changes no Python.

## W1 (main-jazzy: P4.1-P4.6, P1.5, P1.6), 2026-10-24/25
- Q: linus's ROS apt packages moved to the 2026-09-11 snapshot at 23:08-23:12 (pref present), the worktree's
  install is from 2026-10-23: rebuild what? D: `--cmake-clean-first` of the CMake packages in the closure of the
  tested packages (f1tenth_messages, zed_msgs; zed_components/zed_wrapper are on linus_skip.txt), then the three
  Python packages; the full rebuild stays P2.5's (rebuild_after_apt.sh). R: trusted tests for W1 without a long
  build on a machine loaded by the R0b build.
- Q: delete docker/orin/set_llama_paths.py now? D: no: set_installed_params.py is added beside it; main-jazzy's
  Dockerfile still COPYs set_llama_paths.py, W2's Dockerfile switch (main-jazzy-rust) drops the COPY and the file.
  R: no broken Dockerfile on main-jazzy, no merge conflict on the Dockerfile lines.
- Q: where is F1T_PROCESS_BACKEND read? D: from the env file only (never the caller's shell), default auto,
  anything but auto|systemd|proc is a ConfigError (exit 2). auto = proc in a container (/.dockerenv,
  /run/.containerenv) or when `systemctl --user is-system-running` gives no manager state, decided once per process.
  R: ssh, the bot and the gateway must manage the same processes the same way.
- Q: units.unit_pids(unit) has no settings, the proc backend needs the state dir. D: optional `settings` argument
  (old calls keep the systemd behaviour); cli.py's two callers pass it. R: facade signature kept compatible.
- Q: a process that exits at once showed 'stopping' (the runner reaps within 0.1 s, then stops the leftovers). D:
  units.start waits through 'stopping' (at most TimeoutStopSec + 1 s past the settle time), both backends. R: the
  start answer must say crashed + exit status, as test_a_process_that_exits_right_away_fails_the_start wants.
- Q: what does `check` report without systemd? D: items `process_backend` (info) and `workdir` (required) replace
  `systemd_user` and `linger`; the systemd backend's items are unchanged. R: systemd_user would be a required failure
  in the container, so `connect` would never be ready.
- Fact correction: design.md says main-jazzy has enable_mission_logger false; HEAD has true since the car-sync pick
  6137165f (the car's own e491643a flipped it). Q: drop it from the profile? D: no, the profile passes
  enable_mission_logger:=true explicitly anyway. R: a later default flip on main must not stop the car's recording.
- Q: the main-jazzy defaults the car never had (yolo_max_rate_hz 15, yolo_cpu_threads 2, yolo_gpu_pipeline,
  yolo_cuda_blocking_sync, enable_swept_clearance false, the two *_tf_static_only) are not supervisor launch
  arguments. D: kept as the image ships them, each one explained in params_delta.md (class `reviewed`, the ones that
  change behaviour marked "NOT the car's behaviour") for the [U-dec] review; params_delta.py --check fails on any new
  unexplained difference. R: the contract makes the profile launch arguments; reproducing these needs more
  installed-param edits, i.e. a user decision (P9/P10 measure them against native).
- Q: stack_args validation. D: one line of `name:=value` words (regex), anything else ignores the whole file and
  says so in the stack's log (stderr of the stack's sourcing; the stack log's first line is the full command);
  never makes the env file fail. R: estop sources the same file. Note: the extra arguments reach only what
  supervisor_bringup.launch.py declares (the supervisor forwards no other launch configuration).
- Q: container main preflight: strict or warn, and what is a "mount"? D: strict by default (exit 3, nothing
  started); a required path must not be on the container's root filesystem (longest mountinfo prefix != /):
  ~/f1tenth_archive, ~/.ros, ~/.local/state/{f1tenth_ctl,f1tenth_gateway,f1tenth_archive_trigger},
  /usr/local/zed/settings (F1T_REQUIRED_MOUNTS overrides, colon-separated; W2's compose can set it). Also refuses:
  UDP 11811 bound, TCP 8770 listening, a live Fast DDS segment (flock probe of /dev/shm/fastrtps_*_el, the test
  `fastdds shm clean` makes), no gateway venv/launcher/config, token missing or < 32 chars. Warnings only: no
  SN*.conf, no *.gguf. F1T_PREFLIGHT=warn downgrades. R: state in the image layer is lost at `down`; two stacks.
- Q: ordered stop beyond the plan's list. D: planner, logger, monitor, stack (f1tenth_ctl stop), gateway (SIGINT,
  SIGKILL after 10 s), discovery (down --force), then a sweep of every runner left, then (inside a container only)
  leftover ROS processes of the user (killall's ladder) and `fastdds shm clean`. Any exception after the reset also
  runs the ordered stop. R: on the host (linus mock run) other ROS processes of the user are not ours.
- Q: what makes the healthcheck healthy? D: UDP 11811 and TCP 8770 (LISTEN) bound AND the socket inode an open fd
  of a process the container sees (find /proc/[0-9]*/fd -lname). R: with host networking /proc/net shows the
  native stack's and the native gateway's sockets too.
- Interface notes for W2: (1) main finds f1tenth_ctl through F1T_CTL_ENV's package, else its own checkout
  (docker/orin/container/../../../src/f1tenth_ctl), else /opt/f1tenth_ws/src/f1tenth_ctl; F1T_CTL_ENV should be
  the SOURCE path /opt/f1tenth_ws/src/f1tenth_ctl/config/ctl_container.env (ctl_container.env also works from the
  install share: it falls back to /opt/f1tenth_ws). (2) main, the gateway and their venv must not live under a path
  containing /install/ or /opt/ros/: procscan.is_ros would count them as ROS and killall would kill them.
  (3) the archive trigger is replaced by rename (IN_MOVED_TO of `request`): the .path unit should watch the file
  with PathChanged=. (4) healthcheck.sh needs bash, awk, find (Ubuntu base has them).
- Q: shellcheck is not installed on linus. D: `pipx run --spec shellcheck-py shellcheck` (0.11.0, a cached pipx
  venv, nothing on PATH). R: no apt/sudo.
- Found, not changed: mission_logger_node._find_git_root wants a .git DIRECTORY, so in a git worktree (.git is a
  file) the manifest's git block is all null (the new test that needs a checkout skips here). R: native behaviour,
  outside W1; open issue.
- Q: Ubuntu archive snapshot for an arm64 image (plan P5.1 "verify ports.ubuntu.com snapshot support")? D: checked
  from linus 2026-10-24 (read-only curl + a non-root `apt-get update` with its own state dir, APT::Architecture=arm64):
  https://snapshot.ubuntu.com/ubuntu-ports/<id>/ answers 401, but https://snapshot.ubuntu.com/ubuntu/<id>/ serves
  binary-arm64 indexes and the arm64 pool files (curl_8.5.0-2ubuntu10_arm64.deb: 200), InRelease verified with
  ubuntu-archive-keyring. Dockerfile ARG UBUNTU_SNAPSHOT=20261024T000000Z (deb822 URIs on that URL, noble
  noble-updates noble-backports noble-security); https only, so the first update + ca-certificates run with TLS peer
  verification off (GPG still verifies everything). Documented fallback ORIN_UBUNTU_SNAPSHOT=none (live archive);
  every image records DPKG_VERSIONS.txt and the label f1tenth.ubuntu_snapshot. build.sh probes the URL and refuses
  if it does not answer (no silent fallback). R: the pin the plan asked for, verified without an Orin container.
- Q: Gateway venv pins (plan P5.1 "exact av/ctranslate2/tokenizers")? D: gateway-requirements.txt is the COMPLETE
  list the venv installs (installed --no-deps --only-binary=:all:, then pip check): the gateway's 5 pins, faster-whisper
  1.2.1, av 16.1.0, ctranslate2 4.8.3, tokenizers 0.23.3, huggingface_hub 2.2.0, hf-xet 1.7.0 (the bot venv's tested
  set on linus) + 12 resolved transitive pins (pydantic 2.14.1, starlette 0.41.3, ...; httpx2/httpcore2 2.3.0 since
  2.15.1 needs idna>=3.18 and the venv keeps apt's idna 3.6). Resolved with pip --platform manylinux aarch64 cp312
  against the image's system site; verified in a scratch venv on linus (pip check clean, imports, Silero VAD loads,
  the gateway's mock tests 104 passed). R: reproducible and fail-closed (a missing dependency fails pip check).
- Q: onnx/onnxslim/onnxruntime versions (engine export)? D: the Orin Humble host's 1.22.0 / 0.1.94 / 1.23.2 in the
  system site, every transitive pinned (ml_dtypes 0.5.4, protobuf 7.36.2, coloredlogs 15.0.1, flatbuffers 25.12.19,
  humanfriendly 10.0, colorama 0.4.6); faster-whisper reuses this onnxruntime. R: the toolchain that exports the
  native car's engine.
- Q: osqp_offline.cmake path (P3 not done yet)? D: the Dockerfile does not hard-code it: phase B sources
  src/rust/build_env.sh (P3.3) and requires that it exports CMAKE_TOOLCHAIN_FILE_<host triple, - -> _> naming an
  existing file whose basename is osqp_offline.cmake, and that src/rust/third_party/qdldl exists; then builds with
  GIT_ALLOW_PROTOCOL=file and fails if any _deps/qdldl-subbuild appears. Contract for P3: build_env.sh must resolve
  its paths from its own location (the image's workspace is /opt/f1tenth_ws). R: one place owns the path.
- Q: Cargo.lock "unchanged" check when colcon-ros-cargo's [patch] makes cargo rewrite [[patch.unused]] (the committed
  lock lists linus's message versions)? D: docker/orin/cargo_lock_check.py compares the [[package]] entries only (name,
  version, source, checksum, dependencies), reports a different [[patch.unused]] as a note, and the committed lock is
  restored afterwards. No --locked. R: --locked would fail on the machine-specific unused-patch list alone.
- Q: input group GID (host value unknown on linus)? D: build.sh passes every host GID (video render dialout input zed
  jtop) as build args; the base stage moves render/input to them (chgrp of files with the old GID; a GID taken by
  another image group fails the build); labels f1tenth.gid.*; docker_orin.sh up refuses on any host/label mismatch.
  R: F2 fixed at the source, strict check as the D2 order asked.
- Q: Wrapper f1tenth_ctl_car when f1tenth_car runs but is not healthy? D: container side whenever f1tenth_car runs
  with role car (any health); refuse (JSON, exit 1) while it runs as the harness or its state cannot be read; native
  only when it is absent/created/exited/dead or the daemon is down. R: the plan's "otherwise native" would let the
  bot start the Humble stack next to a running (unhealthy) container: never two stacks.
- Q: docker_orin.sh down for role car? D: `docker stop -t 120` only (SIGTERM -> tini -> W1's ordered stop, which
  also kills leftover ROS processes); the exec-started-process sweep runs only for the harness role. R: W1's runners
  are tini's children, not the main's: a "not under the main" sweep would SIGINT the managed stack out of order.
- Q: telegram ctl_command change vs the running bot? D: committed in src/f1tenth_telegram/config/telegram.yaml
  (+ test renamed test_shipped_configs_load_with_their_ctl_command, README); deploy only after install.sh on the
  Orin (P8). The bot on linus runs from its own checkout, untouched. R: plan P6.7.
- Q: Image tag for RUST=skip? D: f1tenth:jazzy-orin-<sha>-norust (RUST=build keeps f1tenth:jazzy-orin-<sha>). R: both
  can exist side by side; the pin file names the one `up` runs.
- Q: docker_orin.sh image selection? D: `docker_orin.sh pin IMAGE` writes ~/.config/f1tenth/car_image (tag, id,
  pinned, revision); refuses an image without the build.sh labels; `up` refuses if the tag's ID moved. compose runs the
  tag (lib_guard's "f1tenth container" check greps the image name). R: plan P6.3.
- Q: set_llama_paths.py? D: deleted on main-jazzy-rust (the Dockerfile runs W1's set_installed_params.py
  <installed stack_params.yaml>). R: superseded per the contract.
- Q: /run/jtop.sock missing at `up`? D: refuse (bind mounts use create_host_path: false; Docker would otherwise create
  a root-owned directory at /run/jtop.sock on the host and break the jtop service). R: the car runs jtop natively.

## W1 manager review (round 1), 2026-10-25
- Q: keep `enable_mission_logger:=true` in the car profile although HEAD's default is already true? D: keep it explicit.
  R: the car records every mission; a later default flip on main must not silently stop the recording.
- Q: the main-jazzy defaults the car never had (yolo_max_rate_hz 15, yolo_cpu_threads 2, yolo_gpu_pipeline,
  yolo_cuda_blocking_sync, enable_swept_clearance false, the two *_tf_static_only) are not launch arguments: reproduce them
  another way (set_installed_params.py)? D: no; kept as the image ships them, documented in params_delta.md for the user's
  [U-dec]; P9 should add the detection-topic rate and per-node CPU to its tables so their effect is measured.
  R: an installed-params edit would make the image deviate from main-jazzy's tested defaults without the user's review;
  the minimum the task names (enable_intelligence, the mission logger as the car runs it) is reproduced.
- Q: delete set_llama_paths.py on main-jazzy now? D: no, W2 deletes it with the Dockerfile switch on main-jazzy-rust.
  R: main-jazzy's Dockerfile still COPYs it; a deletion on one side only merges cleanly.
- Q: F1T_PROCESS_BACKEND from the env file only, auto semantics? D: accepted as built (ctl_container.env sets proc
  explicitly). Follow-up, not blocking: auto should take only a definitive absence (no user bus, "offline") as "no
  manager", not a 5 s timeout, and not cache a fallback that came from a timeout. R: the container never uses auto; on a
  native systemd host a transient timeout must not switch a long-lived in-process caller (the gateway) to proc.
- Q: strict preflight and its mount list? D: accepted; the six default paths equal the bind targets of W2's
  compose.yaml (checked in f1tenth_more_rust), so compose needs no F1T_REQUIRED_MOUNTS. R: state in the image layer is
  lost at `down`.
- Q: healthcheck semantics? D: accepted (port bound AND its socket an fd of a process in the container's PID namespace).
  R: with host networking /proc/net shows the native stack's and the native gateway's sockets too.
- Q: rebuild scope after linus's snapshot move? D: accepted for W1's gate (clean f1tenth_messages + zed_msgs, the three
  Python packages are symlink-installed so the source runs); the full rebuild stays P2.5. R: the manager's own colcon run
  on that install gave NEW 0.
- Q: shellcheck without apt? D: accepted, `pipx run --spec shellcheck-py shellcheck` (0.11.0). R: no sudo/apt.
- Q (manager): units.start now waits through 'stopping' on the systemd backend too: allowed under "native path
  unchanged"? D: accepted. R: only the error path of a process that dies at once leaving children, bounded by
  TimeoutStopSec + 1 s, and it reports crashed + exit status instead of a transient 'stopping'; the native car
  (campaign-final, Humble) does not run this code.
- Q (manager): ordered stop stops the gateway after the stack, so its last zero on /teleop reaches no driver? D: keep the
  plan's order. R: plan P4.2; the VESC driver goes down with the stack; P10 exercises the e-stop paths with a lease live.
- Note for the coordinator: output/orin_container/test_results_w1/ (implementer's run and manager_r1/) is not gitignored;
  do not commit it.

## W3 (D3 harness, worktree f1tenth_more_d3h, branch d3-harness)
- Q: port all of jazzy:scripts/jazzy_parity? D: ported and adapted what a P8/P9 check or its dependency needs (25 of the 46
  + lease_probe from fix_batch_5 + the 5 spike gates); not ported: make_plots*, the Thor frozen-input studies
  (freeze/run/compare_boundary_frozen, boundary_decision_margins, verify_costmap_boundary_logic, freeze/run/compare_mpc_frozen),
  the BT replay (bt_*, needs the Thor reference bag), graph_calls_live / diag_metrics (ekf_cost_observer_node is gone on
  main-jazzy), loop_bag (restamp_play loops), publish_logger_topics (its fix-batch-1 topics are in neither main-jazzy's nor
  the car's logger list), the rest of fix_batch_5 (main-jazzy has no topic_watchdog). R: P9 compares native vs container
  on the same Orin; the Thor-reference studies and removed nodes have nothing to compare there (P9.6 Rust parity uses
  tools/rust_parity on main-jazzy-rust).
- Q: "watchdog 0 false triggers" without fix batch 5's health watchdog? D: the supervisor's process watchdog: 0
  "crashed (exit N) -- auto-respawning" / budget lines in the hold and cycles runs (parity_gates.py watchdog).
  R: it is the watchdog main-jazzy and the car have.
- Q: stop processes / clear run dirs the jazzy way (pkill -f, rm -rf "$OUT_DIR")? D: every process in its own process
  group (setsid exec wrapper), stopped by group; run directories must be new or empty, nothing is ever deleted.
  R: CLAUDE.md harness gotchas; the jazzy versions match their own command lines and delete variable paths.
- Q: the Discovery Server outlives each launch (sh fastdds does not forward SIGINT) and the mission logger leaves a
  dead-pid lock at shutdown (it dies with exit -2 on the second SIGINT): fail the smoke? D: no, both reported as notes;
  a lock held by a LIVE pid after shutdown fails; the harness sweeps the server after the audit so the next cycle starts
  clean. R: both are reclaimed by design at the next start; observed identically in the linus smoke.
- Q: how to time the MPC in a replay (P9.4)? D: wall-clock receive period of /mpc/solver_status (stack_probe.py stamps,
  run beside the replay) for the gate, plus perf_counter solve time via mpc_capture_node; the replay runs the production
  mpc_corr.launch.py under use_sim_time with a goal injected by make_mpc_goal.py (default mission
  drive_test_straight_2m: a drive move mpc_corr never reports reached). R: solve_dt_sec is node-clock time (0.0 under
  sim time) and main-jazzy has no "SOLVE/in" log line; a drive goal keeps the solver busy for the whole bag.
- Q: replay layers with hand-kept topic lists and ros2 run node defaults (jazzy)? D: each layer is the side's production
  launch file under use_sim_time (layer_replay.launch.py); what is played / recorded is derived from the graph
  (stack_probe.py io). Layers: ekf_global (+relay), slam, costmap, mpc, mpc_capture. R: each side then replays its own
  parameters, and the lists cannot drift from the code on either side.
- Q: logger_check refuses when f1tenth-archive.service exists (true on the Orin)? D: capture the sync request instead:
  a `systemctl` PATH shim plus F1TENTH_ARCHIVE_TRIGGER in the work dir; the check passes on either mechanism.
  R: works for the native (systemctl) and container (trigger file) logger, and never syncs from a check.
- Q: ds_probe's "plain client sees nothing"? D: gate on topics: a plain DS client sees node names (ros_discovery_info)
  but not the talker's topic; the super client sees both. R: measured on linus Jazzy; matches the memory note.
- Q: colcon test in the harness container while the image has no build/ and a root-owned src? D: colcon_test_ids.sh
  copies the sources into a work dir and builds/tests there exactly as linus (--symlink-install, isolated domain 96).
  R: the same procedure on both sides gives comparable IDs.
- Q: where does the harness see the checkout? D: assume compose.harness.yaml mounts it at the SAME absolute path,
  writable for results; d3_side.sh has D3_PATH_MAP for anything else. R: every path a harness writes is then valid on
  both sides (open issue for W2 to confirm).
- Q: "engine vs .pt the same" for an FP16 engine? D: same gates as torch-vs-torch (IoU 0.99, |dconf| 2e-3); a failure
  is a D3 finding for the user's review, not a relaxed gate. R: the plan states no other tolerance.
- Q: the linus smoke needs knobs the Orin must never use? D: CPU_REMAP=1 (taskset shim: mpc_corr is pinned to cores
  10,11), PARITY_IGNORE=<regex> (other agents' isolated runs on linus), and a scratch overlay build of ackermann_mux.
  R: the main-jazzy install's ackermann_mux has an undefined diagnostic_updater symbol since the 23:12 snapshot move
  (ldd -r: the only broken binary); W3 did not rebuild main-jazzy (not its worktree; P2.5).

## W2 manager review, round 1 (2026-10-25)
- Note: the W2 entries from "Q: Ubuntu archive snapshot for an arm64 image" through "Q: /run/jtop.sock missing at `up`"
  above were appended after W1 started its section, so they sit under the W1 heading. They are W2's. (The file is
  append-only, so they stay where they are.)
- Rulings on the implementer's 12 questions, all accepted:
  (1) Snapshot URL https://snapshot.ubuntu.com/ubuntu/20261024T000000Z/: accepted. The manager re-checked from linus:
      noble and noble-security InRelease, noble/main/binary-arm64/Release and an arm64 pool .deb all answer 200.
      Bootstrap with TLS peer verification off is acceptable because apt still checks InRelease against
      ubuntu-archive-keyring.
  (2) Gateway lock: accepted. Independent check: all 23 pins have an aarch64 cp312/abi3/py3 wheel. Every pin's
      requires_dist is met by the other pins. The rest (filelock, fsspec, idna, numpy, onnxruntime, packaging,
      pyyaml, typing-extensions) comes from the image's system site, and the build's pip check is fatal.
  (3) onnx 1.22.0 / onnxslim 0.1.94 / onnxruntime 1.23.2: accepted (the Orin Humble export toolchain);
      onnxslim.__version__ exists.
  (4) osqp_offline.cmake comes through src/rust/build_env.sh (CMAKE_TOOLCHAIN_FILE_<triple>): accepted. This is the
      plan's P3.3 mechanism, and P3 owns the path.
  (5) cargo_lock_check.py compares [[package]] only and reports [[patch.unused]] as a note: accepted. It is equivalent
      to the R0b rule (compare without the patch table).
  (6) Host GIDs passed as build args, recorded as f1tenth.gid.* labels, `up` refuses on a mismatch: accepted.
  (7) The wrapper always uses the container side while f1tenth_car runs as the car, whatever its health: accepted.
      This is stricter than the plan and never allows two stacks.
  (8) `down` for the car role is `docker stop -t 120` only: accepted.
  (9) HF cache read-write: accepted.
  (10) The -norust tag and the pin file: accepted.
  (11) A missing /run/jtop.sock refuses `up`: accepted. Before P12, the boot-time start needs a wait for dockerd and
       jtop (user units cannot order after the system docker.service).
  (12) set_llama_paths.py deleted: accepted.
- Q: How should the wrapper and docker_orin.sh find out whether f1tenth_car exists? D: from exit codes and
  `docker ps -a --filter name=^f1tenth_car$ --format {{.State}}` (rc 0 and empty = absent; rc != 0 = daemon unreachable
  or no permission), never by matching message text. For "daemon down -> native", match case-insensitively both the old
  and the new texts ("cannot connect to the docker daemon", "failed to connect to the docker api"). R: measured on
  linus with Docker 29.5.3. A missing container prints "error: no such object: f1tenth_car", which the wrapper's
  case-sensitive "No such object" misses, so it refuses instead of running the native ctl. `docker inspect -f`
  prints an empty line on stdout before failing, so `|| echo absent` yields "\nabsent". The Orin runs Docker 29.6.
- Q: stack_mode container while f1tenth-car-container.service is still "active (exited)" but the container is gone?
  D: stop the unit first (its ExecStop is a no-op `down` once fixed), then start it. After the start, require ctr_state
  running with role car, else FAILED. R: `systemctl start` on an active oneshot RemainAfterExit unit does nothing, and
  stack_mode currently reports "container mode in N s" after killing the native side.
- Q: Gateway venv install `--no-deps` without `--ignore-installed`? D: add `--ignore-installed`. R: pip skips a pin
  that the system site already has at the same version, and build_checks' exact equality would then fail late in the
  1-hour build. No such overlap exists today (checked against the D1 image's pip list), but the system site's
  unpinned transitive packages are resolved at build time.
- Q (W3, smoke): the graph-derived play list is empty for ekf_global and slam (robot_localization logs "Waiting for
  clock to start..." and creates its subscriptions only then)? D: fixed play / record lists for those two layers (the
  jazzy lists), the graph adds what it shows; /tf is always played (one multiplexed topic: the input bag drops the
  layer's own edges); PLAY_TOPICS overrides. R: found in the linus smoke (first ekf_global replay recorded nothing).
- Q: The live guard's phase C counted one Rust stop "after 100 ms" on the rebuilt nodes in 4 of 4 runs (R0: 0).
  D: not a behaviour change. R: an instrumented copy of the harness showed it is the guard's `stale` decision
  214 ms after the LAST clear scan of phase C (scans ended at 1,994 ms, stop at 2,208 ms, largest publish gap
  25.4 ms, no stop while scans flowed): the harness's 200 ms tail window catches the first stale stop, the same
  edge effect r0.md documents for phase D.
- Q: run_corridor_tests.sh's first pass differed in f1tenth_logger (substitute 1 test, capture 24 setup errors
  "the logger node never subscribed"). D: re-ran those two mode/package runs with F1TENTH_TEST_DOMAIN=86 (the
  test's documented override); all four modes then identical for every test ID. R: test_logger_node_roundtrip.py
  failed and passed in every mode, plain py included, across two quick rounds; its conftest pins all runs to ROS
  domain 91, which other agents' logger tests use concurrently. First pass kept (r0b_corridor_suites_run1.log).
- Q: The image-like mirror ran before the build.rs fix. D: ran it again from nothing with the final sources (both
  runs pass; first kept in r0b_mirror/pre_linkfix/). R: the proof must cover what ships.
- Note for the coordinator: src/rust/Cargo.lock is the committed content again (sha256 02efb461fca4...); the
  worktree's .cargo/config.toml now patches the five members (rebuild with the workspace sourced), so any further
  cargo/colcon run there appends [[patch.unused]] blocks: drop them before committing.

## W2 fix round 1 (implementer, 2026-10-25)
- Q: The wrapper's order: `docker ps -a` first (the review's sketch) or `docker inspect` first? D: inspect first
  (`--type container`; one call gives state + role). Only when it fails, `docker ps -a --filter name=f1tenth_car
  --format {{.Names}}` with an exact name match decides: rc 0 without the name = absent -> native; with the name =
  refused; rc != 0 = Docker did not answer, and only then the text decides (case-insensitive "cannot connect to the
  docker daemon" / "failed to connect to the docker api" -> native, anything else -> refused). R: the container side
  stays at 2 Docker calls (inspect + exec), which matters for P10's e-stop gate. Absence is never read from text, per
  ruling (a). Measured on linus with the real CLI: a missing socket gives the new text, a socket with no listener
  still gives the old one, and "permission denied ... docker API" matches neither.
- Q: `--filter name=^f1tenth_car$` (anchored)? D: an unanchored `name=f1tenth_car` filter plus an exact match on
  `{{.Names}}` (awk `$1 == n` / `grep -xF`). R: linus has no container to verify how Docker anchors the name regex
  (the leading slash). The exact match is correct either way, and a test covers `f1tenth_car_old`.
- Q: docker_orin.sh's state when Docker does not answer? D: an error ("cannot ask Docker for f1tenth_car's state:
  <its last line>", exit 1), never "absent". stack_mode reads it as "unknown": `container` fails before it touches
  the native side; `native` goes on, and the guard's /proc scan judges. R: "absent" would print "does not exist"
  and exit 0 for a container that may still run. Going native must stay possible while Docker is broken.
- Q: How should the unit wait for dockerd and jtop at boot (minor issue 5)? D: a `docker_orin wait-host [SECONDS]`
  subcommand (F1T_WAIT_HOST_S, 120 s) as `ExecStartPre=`, not an inline shell loop in the unit. R: it is testable,
  needs no systemd `$$`/`%%` escaping, and returns at once on a running system. TimeoutStartSec=420 covers 120 s +
  up's 150 s.
- Q: The stale unit's stop returns non-zero (ExecStop `docker_orin down` exits 3 on stale Fast DDS SHM of the
  dead container)? D: judge by the unit's state: inactive -> warn and start; still active -> FAILED, no start.
  R: exit 3 only lists leftovers; the unit is stopped, `start` accepts a failed unit, and `up`'s guard judges
  what is live.
- Q: `--type container` on the other `docker inspect f1tenth_car` calls (guard, banner, uninstall)? D: yes,
  everywhere. R: an image, volume or network named f1tenth_car would otherwise answer `docker inspect`.
- Q: A test against the real Docker CLI? D: `test_wrapper_against_the_real_docker_cli_without_the_container`
  (`--which` and `status --json` give native). It is skipped when there is no docker CLI, when Docker does not
  answer, or when f1tenth_car exists. R: the round-1 bug passed its tests only because the fake imitated older
  Docker. The fakes now print Docker 29's texts (FAKE_DOCKER_STYLE=28 keeps the old ones as parametrized cases).
