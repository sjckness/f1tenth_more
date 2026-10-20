# Telegram control: test on linus (2026-10-20)

**Result.**

- The whole sequence passed through `f1tenth_telegram --cli` against the
  Gazebo sim on linus:
  connect → start stack → READY → planner on/off → logger on/off → estop →
  killall → everything stopped.
- 117 unit tests pass under `colcon test` (7 Telegram tests are skipped
  without python-telegram-bot); all 49 bot tests pass in a venv with
  python-telegram-bot 22.8.
- Telegram itself, the ssh path to a deployed f1tenth_ctl, and the car were
  **not** tested (§5).

## 1. Setup

### The sim on one host

- **Branch.** The car's branch has no sim mode in its supervisor, so the sim
  test ran a `jazzy` checkout (`3dbe37f`): Gazebo plus the stack with
  `sim:=true`. It was built in this session's scratch directory.
- **Missing packages.** linus lacks some packages that stack needs, and there
  is no apt access. They were built from source in a scratch underlay:

  | Package | Version |
  |---|---|
  | slam_toolbox | 2.8.5 |
  | py_trees_ros | 2.6.0 |
  | py_trees_ros_interfaces | 2.1.2 |
  | topic_tools | 1.3.4 |

  Two **empty** stand-in packages, `vesc` and `vesc_driver`, let
  f1tenth_hardware build without serial_driver/asio. The sim never runs the
  VESC driver.
- **Env file.**
  - f1tenth_ctl ran with `src/f1tenth_ctl/config/sim_linus.env`, through a
    3-line wrapper that sets the two workspace paths.
  - The wrapper also adds `components_config:=` with a components.yaml
    without `mission_logger.launch.py`. jazzy's supervisor has no
    `enable_mission_logger` switch, so its stack always ran the logger.
  - Without that change, `start logger` was refused, correctly:
    "logger already in the graph: /mission_logger_node".
- **Gazebo.** It plays the car's hardware and is not a managed process. It
  was started first, in its own user unit:

  ```bash
  ros2 launch f1tenth_sim sim_bringup.launch.py world:=test_room_6x6 gui:=false \
      camera:=false clock_rate:=50.0
  ```

- **Network.** Domain 42, the discovery server on `127.0.0.1:11811` started
  by `connect`, `rmw_fastrtps_cpp`.
- **Bot.** `f1tenth_telegram --cli` with `host: local` (f1tenth_ctl runs
  directly, no ssh), as `config/telegram_sim.yaml`.

### Load: what had to change for one host

linus has 8 cores and 15 GB. The first runs used the defaults: `/clock` at
250 Hz and every process a discovery-server super client, like the Orin's
`~/.bashrc`. Load reached 37, swap was in use, and the discovery server alone
used 50 % of a core.

Two changes made it usable, at load 17–20:

- `clock_rate:=50.0` for Gazebo;
- no global `ROS_SUPER_CLIENT` in the sim profile, as jazzy's own sim scripts
  do. f1tenth_ctl's probe and monitor set it for themselves.

## 2. The sequence

The log is `output/telegram/test_cli_run.log`, 15:00:53–15:02:10. The
command:

```bash
bin/f1tenth_telegram --cli --config <session telegram_sim.yaml> \
  --do refresh --do connect --do wait:15 --do status \
  --do start:stack --do confirm --do until-ready:240 \
  --do start:planner --do confirm --do wait:5 --do status --do stop:planner \
  --do start:logger --do wait:5 --do status --do stop:logger \
  --do estop --do killall --do confirm --do refresh --do status
```

| Step | Panel after it | Result |
|---|---|---|
| refresh | 🟢 online, buttons 🔌 Connect 🔄 | ✅ |
| connect | "connected: discovery 127.0.0.1:11811, environment ok, ready to launch"; READY ❌ stack not running | ✅ |
| start:stack | "❓ really start the stack?", buttons ✅ yes / ✖ cancel / 🛑 | ✅ asks first |
| confirm | stack 🟢 running, pid 2369380, 4.35 s | ✅ |
| until-ready | READY ✅ at the 2nd status (about 14 s after the start): odom 44 Hz, scan 40 Hz, TF 0.00 s, mission IDLE | ✅ |
| start:planner, confirm | planner 🟢 running, 2.63 s | ✅ |
| status | READY ✅, planner 🟢 | ✅ |
| stop:planner | planner ⚪ stopped (SIGINT, 1.2 s) | ✅ |
| start:logger | logger 🟢 running, 2.67 s (no confirmation, as specified) | ✅ |
| status | READY ✅, logger 🟢 | ✅ |
| stop:logger | logger ⚪ stopped (SIGINT, 2.3 s) | ✅ |
| estop | service ✅, answer in 0.003 s ("emergency stop engaged"); zero drive ✅, 100 msgs at 50 Hz to 2 subscribers; READY ❌ estop_clear, "e-stop LATCHED" | ✅ (the stack was not touched) |
| killall, confirm | units stopped: stack, monitor; 9 other ROS processes signalled (the Gazebo launch); discovery running | ✅ |
| refresh, status | stack ⚪ · planner ⚪ · logger ⚪; READY ❌ stack not running | ✅ |

**Afterwards:**

- `ps` showed only the discovery server's three processes (`sh fastdds`,
  `fastdds.py`, `fast-discovery-server`);
- `systemctl --user list-units 'f1t-*'` showed only `f1t-discovery`.

That is the specified end state: killall leaves the discovery server up
unless `--reset-discovery`.

## 3. Other checks run on linus

| Check | How | Result |
|---|---|---|
| status time | 12 consecutive `f1tenth_ctl status`, stack running, monitor warm | 1.13–1.37 s, READY ✅ in all 12 |
| status time, one-shot probe (before the monitor existed) | 10 runs at load 17 | 3.5–4.5 s; READY ✅ only 2 of 10: the probe often matched nothing in time. This is why the monitor was added |
| crash detection | planner started with no llama-server: it waits 90 s, then exits 1 | `planner CRASHED exit-code, exit 1`; `stop planner` cleared it to `stopped` |
| double start, managed | `start stack` twice, with a dummy stack (and in a unit test) | refused: "stack is already running" |
| double start, by hand | jazzy stack running its own `mission_logger_node`, then `start logger` | refused: node already in the graph |
| stop ladder, SIGINT ignored | unit test with a dummy that ignores SIGINT | SIGINT → SIGTERM after the wait, then stopped |
| stop of a `ros2 run` | first run: SIGINT to the main pid only | `ros2 run` did not forward it: SIGINT→SIGTERM→SIGKILL, 16.3 s. Fixed: SIGINT to the process group (as Ctrl-C); now SIGINT only, 1.2 s |
| estop under load | first run | service answered after its 0.5 s timeout (the latch engaged anyway); only 33 msgs in 2 s, and matching took 1.1 s. Fixed: publishing is timed from the mux match, and a late answer is reported as late |
| offline | bot `--cli` against 192.0.2.1 (TEST-NET) | 🔴 offline "Connection timed out", only 🔄; connect → "not connected: the car is offline"; estop → "E-STOP NOT SENT" |
| real Orin | bot `--cli` with `config/telegram.yaml`: refresh, connect | 🟢 online (BatchMode ssh ok). connect → "f1tenth_ctl not found" (exit 127), because it is not deployed there. Reported, no crash |
| Humble compatibility | every f1tenth_ctl and f1tenth_telegram module compiled in memory with the Orin's Python 3.10.12, plus the rclpy/tf2 APIs used (`raw=`, `spin_thread=`, `can_transform`, message types incl. `f1tenth_messages`) checked under Humble. Nothing written on the Orin | all ok |
| Orin facts | read-only ssh | systemd 249, `systemctl --user` running, Linger=yes, sourcing ROS + workspace 0.82 s, 12 cores |

## 4. Unit tests

| Package | Tests | Result |
|---|---|---|
| f1tenth_ctl | 68 | pass |
| f1tenth_telegram | 49 | 42 pass, 7 skipped (Telegram glue, needs python-telegram-bot) |
| f1tenth_telegram in a venv with python-telegram-bot 22.8 | 49 | pass |

`colcon test --packages-select f1tenth_ctl f1tenth_telegram` on linus gave
117 tests, 0 errors, 0 failures, 7 skipped. It ran with `ROS_DOMAIN_ID=147`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

**f1tenth_ctl**
- `test_ready`: READY logic, thresholds, and coupling tests: the battery
  threshold equals stack_params `min_battery_voltage`; the estop topic is the
  mux's top lane and its rate beats the lane timeout.
- `test_config`: env file sourcing on a minimal environment.
- `test_units`: systemd states, the stop ladder, and that ExecStart carries
  no `${…}` or `$$`.
- `test_procscan`: `/proc` matching, the signal ladder on real processes, the
  UDP port owner.
- `test_cli_logic`: the graph grace after a stop.
- `test_process_manager`: real `systemd --user` units with dummy processes:
  - start/status/stop;
  - SIGINT escalation;
  - FIFO stdin;
  - crash and early-exit detection;
  - refusing a copy started by hand;
  - the command reaching bash unexpanded by systemd;
  - the same environment in units and the CLI;
  - SIGINT reaching the child of a `ros2 run`-like parent;
  - killall `--dry-run`;
  - a bad env file giving exit 2 and JSON.

  Units are named `f1t-test-<pid>-*` and never touch real ones.

**f1tenth_telegram**
- `test_transport`: JSON parsing, ssh command line and quoting, ssh timeout
  (fake `ssh` that sleeps), ssh failure (exit 255), reachability, local mode.
- `test_guard`: whitelist, the 10 s stale filter, confirmations (expiry,
  single use, replacement, cancel).
- `test_controller`: the state machine:
  - offline → 🔄 only; online → Connect; connected → all buttons;
  - confirmations; cancel; no confirmation for stop and logger;
  - estop while another action holds the lock;
  - refusing instead of queuing;
  - an ssh failure → offline.
- `test_view_cli_config`: rendering, scripted and interactive `--cli`,
  config, and token file permissions.
- `test_tg`, with python-telegram-bot, never contacting Telegram:
  - polling setup: drop pending updates, concurrent updates;
  - whitelist; stale message dropped;
  - one panel edited in place; an old panel's button refused;
  - a stranger's button ignored; `/estop`.

## 5. Not tested

- **Telegram itself.** There is no token yet. The glue ran against the real
  library with stand-in updates only.
- **The ssh path to a deployed f1tenth_ctl.** Unit tests use a fake `ssh`.
  The real Orin was used only for reachability and the "not found" path.
  linus does not authorize its own key, so `host: localhost` over ssh was not
  possible without editing `~/.ssh/authorized_keys`.
- **The car.** Not tested:
  - the stack's real start and stop times on the Orin;
  - status timing on its CPU under a campaign load;
  - the estop path to the real mux and VESC;
  - the battery check with real data.

  The e-stop latch was exercised in sim only.
- **Planner commands.** No llama-server on linus: the planner was started and
  stopped while waiting for it. `f1tenth_ctl send planner` was tested with
  `cat` in the unit tests.

## 6. Reproducing

The scratch workspaces of this session disappear with it. To run the sim
test again:

1. Check out `jazzy` somewhere with its submodules: vesc, ackermann_mux,
   teleop_tools, husarion_gz_worlds.
2. Build the underlay of §1 (or install the debs), then build the workspace
   with `--packages-skip vesc_driver vesc`.
3. Set `F1T_SIM_WS` / `F1T_SIM_DEPS_WS` in a wrapper env file that sources
   `config/sim_linus.env`.
4. Start Gazebo as in §1, then run the `--cli` sequence of §2.
