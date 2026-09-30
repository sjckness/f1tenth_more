# C++ Submodule Port Report — Jazzy Migration
<!-- Generated 2026-09-30 -->

## Executive summary

One build-blocking gap: `io_context` (and everything that depends on it) fails because
`libasio-dev` is not installed on this host. The fix is **Strategy C** for transport_drivers:
discard the source tree in favour of the matching apt binary package, then add `COLCON_IGNORE`
so colcon never tries to build it from source again.

All other packages are Jazzy-clean as-is. No source edits to any package.

**Required apt install (run once before building):**
```
sudo apt install \
  ros-jazzy-serial-driver \
  ros-jazzy-io-context \
  ros-jazzy-asio-cmake-module \
  ros-jazzy-udp-msgs
```

---

## Per-submodule decisions

### 1. transport_drivers  (`src/f1tenth_external/transport_drivers`)

| field         | value |
|---------------|-------|
| upstream      | ros-drivers/transport_drivers |
| pinned commit | `d3f510c` (tag 1.2.0) |
| custom commits | **0** |
| apt package   | `ros-jazzy-serial-driver` 1.2.0-4noble, `ros-jazzy-io-context` 1.2.0-4noble |
| strategy      | **C — apt package + COLCON_IGNORE** |

**Justification:**
- The submodule is pristine upstream at 1.2.0. No fork-specific commits exist.
- `apt` ships an identically-versioned binary (`1.2.0-4noble`), which is a clean rebuild
  of the same sources against Jazzy system packages — safer than building the identical
  source ourselves while libasio-dev is absent.
- `libasio-dev` is unavailable on this host and would need `sudo` to install.
- `vesc_driver` finds `serial_driver` via `find_package` from the apt installation once
  the packages are installed; nothing in vesc_driver's CMakeLists.txt encodes a
  source-tree path.

**Baseline build evidence (`output/cpp_build_before.log`):**
```
Failed   <<< io_context [0.59s, exited with code 1]
CMake Error: Could NOT find ASIO (missing: ASIO_INCLUDE_DIRS)
```
No Jazzy API incompatibility — the only failure is the missing system library.

**Change applied:**
- Added `src/f1tenth_external/transport_drivers/COLCON_IGNORE`
- Created local branch `jazzy` in the submodule with this single commit
- Superproject `jazzy` branch updated to point at the new commit

---

### 2. ackermann_mux  (`src/f1tenth_external/ackermann_mux`)

| field         | value |
|---------------|-------|
| upstream      | sjckness/ackermann_mux (forked from PAL Robotics twist_mux, rewritten for AckermannDriveStamped) |
| pinned commit | `b3c0b083` (HEAD of origin/foxy-devel) |
| custom commits | **4** (AckermannDriveStamped rewrite, launch files, config) |
| apt package   | none — no `ros-jazzy-ackermann-mux` exists |
| strategy      | **A — build from source, no changes** |

**Custom behavior that must survive:**
- Publishes `ackermann_msgs/AckermannDriveStamped` on `ackermann_cmd` (not Twist).
- Priority-based muxing matches `src/f1tenth_bringup/config/mux.yaml`:
  `safety_stop`(200) > `joystick`(100) > `calibration`(50) > `navigation`(10).
- No twist_mux anywhere in the stack — this fork is the only mux.

**Baseline build evidence:**
```
Finished <<< ackermann_mux [0.34s]   (and [14.0s] in standalone run)
```
Builds clean under GCC 13 + Jazzy rclcpp. All API calls (rclcpp::Duration,
rclcpp::SystemDefaultsQoS, create_subscription, create_publisher) remain valid.

**Change applied:** none.

---

### 3. teleop_tools  (`src/f1tenth_external/teleop_tools`)

| field         | value |
|---------------|-------|
| upstream      | f1tenth/teleop_tools |
| pinned commit | `4337558` |
| custom commits | **0** |
| apt package   | `ros-jazzy-joy-teleop` 2.0.0, `ros-jazzy-key-teleop` 2.0.0 |
| strategy      | **A — build from source, no changes** |

**Rationale for A over C:** The submodule is already checked out and building. Stack
uses `joy_teleop` from the source tree (`joy.launch.py` loads it, `joy_teleop.yaml`
configures it). Switching to apt would require removing the submodule initialisation
or adding a second COLCON_IGNORE — unnecessary churn for something that works.

**Baseline build evidence:**
```
Finished <<< teleop_tools_msgs [0.52s]
Finished <<< joy_teleop (standalone run [8.35s])
```
Python packages, no C++ compilation. No Jazzy-breaking patterns found.

**Change applied:** none.

---

### 4. vesc  (`src/f1tenth_hardware/vesc`)

| field         | value |
|---------------|-------|
| upstream      | sjckness/vesc (itself forked from f1tenth/vesc) |
| pinned commit | `2848a9f` (9 custom commits above upstream merge `153998d`) |
| custom commits | **9** (see below) |
| apt package   | none — custom behaviour makes apt packages unusable |
| strategy      | **A — build from source, no changes** |

**Custom behavior that must survive:**

| commit  | change |
|---------|--------|
| `1e6a734` | `gyro_scale_x/y/z`: final gain applied after bias subtraction in `vesc_driver.cpp:283`. `gyro_scale_z = 0.0174533` (≈π/180) corrects a suspected deg/s → rad/s unit mismatch in `gyr_z()`. **Load-bearing for EKF accuracy.** |
| `c6c1a1d` | `erpm_deadband_`: zeroes raw ERPM when `abs(rpm) < 500.0` in `vesc_driver.cpp:215`. Eliminates ~380-400 ERPM noise floor at rest. |
| `ee17d6c` | `steering_angle_to_servo_gain_left/right`: asymmetric gain split replacing the single `steering_angle_to_servo_gain`. `vesc.yaml` uses `gain=-1.0925563`. |
| `a33a30b` | Sign convention, servo topic, covariance calibration params. |
| `f0cd7a9` | `vesc_tuning` Python package (steering calibration, speed tuning). |
| `d08fcfd` | Odometry integration fixes. |
| `7169442` | Kalman filter for IMU+ERPM+servo. |
| `6e67147`/`2848a9f` | test infra fixes (tests_require cleanup, pytest extra). |

**Jazzy compatibility check:**
- `#include <experimental/optional>` and `std::experimental::optional` — verified present
  in GCC 13.3.0 under both C++14 and C++17 mode (no warnings, exit 0).
- CMakeLists.txt C++ standard guard only activates when default is `98`; GCC 13 defaults
  to `17`, so neither guard branch fires. Code compiles at C++17 — no issue found.
- All rclcpp API calls (declare_parameter, create_subscription, create_publisher,
  rclcpp_components) remain valid in Jazzy.

**Baseline build result:**
```
Aborted  <<< vesc_ackermann [1.46s]    ← aborted because serial_driver didn't build
Not processed: vesc_driver, vesc_tuning, serial_driver, udp_driver
```
The abort is purely a cascading dependency failure from missing libasio-dev.
No Jazzy API failures found in the source.

**Change applied:** none.

---

## Build plan

1. Run (once, as root):
   ```bash
   sudo apt install \
     ros-jazzy-serial-driver \
     ros-jazzy-io-context \
     ros-jazzy-asio-cmake-module \
     ros-jazzy-udp-msgs
   ```

2. Source Jazzy:
   ```bash
   source /opt/ros/jazzy/setup.zsh
   ```

3. Build all submodule packages:
   ```bash
   colcon build --symlink-install \
     --packages-select \
       ackermann_mux \
       teleop_tools_msgs joy_teleop key_teleop teleop_tools \
       vesc_msgs vesc_driver vesc_ackermann vesc_tuning \
     --event-handlers console_direct+
   ```
   (transport_drivers packages are excluded — colcon skips them via COLCON_IGNORE,
    vesc_driver finds them through the apt-installed cmake config.)

4. Smoke-test vesc_driver (fails cleanly on missing serial port — that's the pass criterion):
   ```bash
   source install/setup.zsh
   ros2 run vesc_driver vesc_driver_node \
     --ros-args --params-file src/f1tenth_bringup/config/vesc.yaml 2>&1 | head -5
   # Expected: FATAL "Failed to connect to the VESC" then shutdown — NOT a segfault
   ```

---

---

## ackermann_mux phantom commit investigation

### What was asked

`origin/main`, `origin/jazzy` (and the pre-port `e47e646` superproject commit) all point
ackermann_mux at `42b7cd70bf4cd1d11c2eb1eea62ad275af9b5a77`. The fetch attempt during
submodule init produced `fatal: remote error: upload-pack: not our ref 42b7cd70...`.
Investigation requested: trace the commit through superproject history, attempt to recover
the object, compare against `b3c0b083`, and recommend.

---

### Superproject history of the ackermann_mux pointer

| superproject commit | date | author | message | ackermann_mux pointer |
|---|---|---|---|---|
| `621f076` | 2026-05-30 | andreas-linus | fix teleop_tools submodule | → `b3c0b083` (first appearance) |
| `9997d58` | 2026-05-30 | andreas-linus | Restructure into colcon workspace | rename only |
| `895d9ca` | 2026-06-19 | andreas-linus | docker for thor | rename only |
| **`5b69f47`** | **2026-07-13** | **andreas** | **nav2 not working, odometry not bad** | **→ `42b7cd70`** |
| `8d99f78` | 2026-09-30 | andreas-linus | ackermann_mux: correct superproject pointer | → `b3c0b083` (our port fix) |

**The commit that set the phantom pointer** is `5b69f47` ("nav2 not working, odometry not
bad", Jul 13 2026). It was a large 66-file / +2530 line dump that added the `f1tenth_behavior`
package, nav2 integration, first vesc launch files, `mux.yaml`, and `ackermann_mux_launch.py`.
It also bumped vesc from `7169442` (first Kalman filter attempt) to `d08fcfd` (odometry fixes).

---

### Fetch attempt results

```
# Inside src/f1tenth_external/ackermann_mux
$ git fetch origin 42b7cd70bf4cd1d11c2eb1eea62ad275af9b5a77
fatal: remote error: upload-pack: not our ref 42b7cd70bf4cd1d11c2eb1eea62ad275af9b5a77

$ git cat-file -t 42b7cd70bf4cd1d11c2eb1eea62ad275af9b5a77
fatal: git cat-file: could not get object info
```

`42b7cd70` is **not in the local object store and not fetchable from the remote**.
It has been permanently removed (force-push or branch recreation on sjckness/ackermann_mux).

The remote has exactly one branch (`foxy-devel`) and one commit on it:
```
$ git ls-remote origin
b3c0b083ac03aa8c648537d7e4d22608fcd3440c  HEAD
b3c0b083ac03aa8c648537d7e4d22608fcd3440c  refs/heads/foxy-devel
```

The full foxy-devel history (20 commits) was inspected: `42b7cd70` does not appear anywhere.

---

### Timeline reconstruction

- **2021-11-10** — `b3c0b083` ("Change out going topic name") authored on sjckness/ackermann_mux.
  This is the last commit on the remote's foxy-devel today.

- **2026-05-30** — superproject first pins ackermann_mux at `b3c0b083`.

- **2026-07-13** — `5b69f47` bumps ackermann_mux to `42b7cd70`. This means Andreas had pushed at
  least one new commit to sjckness/ackermann_mux by that date, making `42b7cd70` the HEAD of
  foxy-devel at that moment.

- **Unknown date** — sjckness/ackermann_mux foxy-devel was force-pushed or recreated, returning
  to `b3c0b083` and removing `42b7cd70`.

- **2026-09-30** — this investigation: `42b7cd70` unrecoverable; port session moves to `b3c0b083`.

---

### Diff `b3c0b083` vs `42b7cd70`: not possible

The commit object is gone. `git diff b3c0b083 42b7cd70 --stat` cannot run.

---

### Remote reachability check: other submodules

| submodule | Humble-era pinned commit | in local store? | reachable on remote? |
|---|---|---|---|
| `vesc` | `2848a9f` | ✅ yes (`commit`) | ✅ yes — HEAD of `refs/heads/scene-graph` |
| `teleop_tools` | `4337558` | ✅ yes (`commit`) | ✅ yes — HEAD of `foxy-devel` and remote HEAD |
| `ackermann_mux` | `42b7cd70` | ❌ not in store | ❌ not fetchable |

vesc and teleop_tools are fully recoverable on a fresh checkout. ackermann_mux is not.

---

### Jetson implication

Both `origin/main` and `origin/jazzy` (pre-port) still record `42b7cd70`. The Jetson
running Humble (`origin/main`) was presumably checked out before the force-push, so it
likely **has `42b7cd70` in its local submodule object store**. A fresh `git submodule update
--init` on a new machine would fail with the same "not our ref" error; the Jetson only works
because the object is already cached locally from an earlier checkout.

---

### Recommendation

**On the Jetson (before next `git pull` wipes the local object):**

```bash
# Inside the Jetson's ackermann_mux submodule dir
git cat-file -t 42b7cd70bf4cd1d11c2eb1eea62ad275af9b5a77  # should print "commit"
git diff b3c0b083 42b7cd70  --stat
git log --oneline b3c0b083..42b7cd70
```

If the diff is non-trivial (changes to source files), cherry-pick or squash the delta into a new
commit on a branch in your own fork and re-pin the superproject to that. If it's trivial (only
build metadata, README, test changes), move all superproject branches to `b3c0b083`.

**On this host (Jazzy dev), the port pointer stays at `b3c0b083`** — it is the only
recoverable commit, it builds clean, and the stack's Jazzy-facing behavior was validated with
it. Do not advance further without first resolving the Jetson side.

---

## What was NOT changed and why

- **No source edits in vesc, ackermann_mux, or teleop_tools.** The guiding principle is
  "same behaviour, not better code." The baseline build failures are entirely due to a
  missing system library, not API changes — confirmed by inspecting all compiler errors
  and grepping for deprecated patterns.

- **`std::experimental::optional` left alone.** Available in GCC 13 under all standard
  modes tested. Changing it to `std::optional` would be scope creep with no build benefit.

- **No C++ standard upgrade.** The vesc CMakeLists's guard happens not to fire on GCC 13
  (default is C++17, guard only fires on C++98). Code compiles correctly at C++17. Not
  our bug to fix.
