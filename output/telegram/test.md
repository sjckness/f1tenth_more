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

## 7. Follow-up: e-stop reset and discovery setup

Also on 2026-10-20, with the changes of the follow-up:
- `f1tenth_ctl estop-reset`;
- the READY check renamed `estop_clear` → `estop_latched`, and status's
  `estop_latched` field;
- the bot's 🔓 button;
- `sim_linus.env` taking its ROS environment from `sim_net.sh`;
- `check`'s `shell_env` warning.

The reset design is in `src/f1tenth_ctl/README.md`, estop-reset. The
discovery answers are in `output/telegram/discovery.md`.

### Through the bot

The log is `output/telegram/test_cli_reset_run.log`. Sim profile: the new
`sim_linus.env`.

| Step | Panel | Result |
|---|---|---|
| connect, start stack (confirmed), until-ready | READY ✅ at the 2nd status | ✅ |
| 🛑 estop | READY ❌ `estop_latched`; `🔴 E-STOP LATCHED: no mission will start until 🔓 Reset e-stop`; buttons now include 🔓 Reset e-stop | ✅ |
| status | still latched, still READY ❌ | ✅ |
| 🔓 estop-reset | "❓ really reset the e-stop (aborts the mission, restarts behavior)?" | ✅ asks first |
| confirm | "🔓 e-stop reset (behavior restarted), latch clear, checked on /mission/status"; READY ✅; the 🔓 button is gone | ✅ |
| killall (confirmed), status | all stopped, discovery left up | ✅ |

### f1tenth_ctl estop-reset, JSON detail (stack running, after an estop)

- **path:** `restart_behavior`;
- **latched_before:** true; **latched_after:** false; **verified:** true;
- **abort:** "no mission to abort (state=IDLE)";
- **restart:** answered in 0.38 s, "'behavior' restarted -- behavior_bringup.launch.py: clean SIGINT exit in 0.4s";
- **zero drive:** 198 messages after the mux matched (0.25 s), over 4.2 s;
- **total:** 7.5 s, then status READY ✅, `estop_latched: false`.

### The stack-restart path

The same, with `F1T_SUPERVISOR_RESTART_SERVICE` pointed at a service that
does not exist, as if there were no supervisor:
- `path: stack_restart`;
- stop with SIGINT in 2.6 s, start again;
- verified; 18.1 s in all;
- then READY ✅.

### Does the car move while `behavior` restarts?

Set-up:
- `/teleop` (mux priority 100) asks for 0.3 m/s the whole time. It stands in
  for any lower lane that still commands motion; on the car that is a RUNNING
  mission's mpc_corr, which the e-stop leaves alone.
- Motion is measured on `/sim/ground_truth/car`.

| Run | Window | Car moved |
|---|---|---|
| latched, teleop on, 3 s | — | 0.000 m / 0.008 m |
| plain `/restart_component behavior` | 6.1 s, until the new node published a clear latch | **1.533 m** |
| `estop-reset`, first version | 12.3 s | 0.047 m |
| `estop-reset` with the match gate | 10.8 s | 0.042 m |
| `estop-reset` with the match gate, mux output logged | 9.6 s | 0.000 m |

**What the numbers show.**
- **The hazard is real.** The behaviour tree's zero on `/safety_stop` is the
  only thing holding the car while latched, and it goes away with the
  restart.
- **First version.** The reset could start before the mux had matched
  f1tenth_ctl's own zero drive: the abort answers in 3 ms, and matching took
  0.25 s.
- **Fix.** Nothing is reset until the mux has matched the zero drive and five
  messages have gone out.
- **What is left** is the stack's own behaviour at load 17–28: the mux lets a
  single lower-lane message through when the top lane is late. The latched
  tree leaks the same way: 0.008 m in 3 s, one sample in the logged run.

### Discovery profile checks

- **`f1tenth_ctl check`** with the new `sim_linus.env`: `shell_env` ok, "same
  ROS environment as F1T_SHELL_REF snippet".
- **With the first-round `sim_linus.env`:** a warning, "2 difference(s):
  ROS_AUTOMATIC_DISCOVERY_RANGE, FASTRTPS_DEFAULT_PROFILES_FILE". `check` stays
  ok, and the warning is listed under `warnings`.
- **Plain vs super client, one participant, 5 s, sim graph:**

  | | Nodes | Topics |
  |---|---|---|
  | plain | 67 | 2 |
  | super | 67 | 137 |

- **Car `ctl.env` vs the Orin's `~/.bashrc`:** identical on all 13 variables
  (`discovery.md` §3). Read-only.

### Unit tests

| Package | Tests |
|---|---|
| f1tenth_ctl | 82 |
| f1tenth_telegram (python-telegram-bot venv) | 51 |

**New:**
- READY false while latched, with `estop_latched` failing and named;
- READY again once clear;
- an unknown latch is not READY and not reported as clear;
- the reset-path choice;
- the env comparison: defaults, super-client spellings, graph-changing keys,
  path entries, a snippet reference, `bash -i` reading a `~/.bashrc`;
- the bot's latch line, the 🔓 button and its confirmation.

**Not tested:**
- the reset on the car;
- a reset during a RUNNING mission. Preflight in this sim always fails on
  `ackermann_to_vesc_node`, so no mission could be started; the
  abort → `/mpc/hold` step ran only as "no mission to abort".

## 8. Round 3: e-stop holds the mission, bot networking (2026-10-20, NOT sim-tested)

What changed. **None of it ran against the sim or the car.**

1. **`f1tenth_ctl estop`** runs three steps at once, each best effort:
   - zero drive on `/safety_stop`;
   - `/mission/emergency_stop` (0.5 s);
   - `/mission/abort_mission` (1 s; "no mission to abort" is ok).

   It answers one record per step (`status` ok/failed/skipped, `message`,
   `duration_ms`) and
   `car_held = zero drive AND (latch OR abort)`.
   - `estop-reset` keeps its sequence. It now uses the same step records and
     names for `zero_drive` and `abort_mission`.
   - The bot's 🛑 reply shows the three steps, ✅/❌, and the result.
2. **Bot networking:**
   - `telegram.force_ipv4` (default true): an httpx transport bound to
     `0.0.0.0`;
   - `telegram.timeout_s` (default 20) for both the bot request and the
     `get_updates` request;
   - `run_polling(bootstrap_retries=-1, drop_pending_updates=True)`;
   - one INFO line at start (no ids, no token);
   - httpx's request log is held at WARNING, because Bot API URLs carry the
     token;
   - the unit: `Restart=always`, `RestartSec=5`.
3. **Whitelist** `[857433485]` in `config/telegram.yaml`. It was already
   committed in `f4a12c9`.
4. **`output/telegram/estop_proposal.md`**: the e-stop aborts and holds
   itself, plus `/mission/reset_estop`. A proposal only; no change in
   `f1tenth_behavior`.
5. **Docs:**
   - the bot README: setup on linus, troubleshooting, security;
   - the f1tenth_ctl README: what estop guarantees and what it does not.

**Checks run:**

| Check | Result |
|---|---|
| f1tenth_ctl unit tests | 88 passed |
| f1tenth_telegram unit tests, venv with python-telegram-bot 22.8 | 56 passed |
| f1tenth_telegram unit tests, without it | 48 passed, 8 skipped |
| New tests | estop JSON shape and `car_held` with mocked steps; "no mission to abort" is ok; `force_ipv4`/`timeout_s` parsing and validation; the startup line has no ids; the requests use the configured timeout and an IPv4-bound transport |
| getMe with the real token through the bot's own IPv4 request | ok, 0.19 s. The token was not printed |
| `f1tenth_ctl estop --json` on linus, sim profile, no discovery server, no graph | 0.5 s; all three steps `skipped` ("discovery server down"); `car_held: false`; exit 1 |
| The same with ROS sourced, no discovery server, isolated domain, nothing on the graph | 5.8 s; zero drive "no subscriber matched", both services "service not available"; `car_held: false`; exit 1 |

**Not tested:**
- the new estop against a running stack (sim or car);
- the abort step during a RUNNING mission;
- the bot under systemd with the new networking.

## 9. Round 4: planner commands from Telegram (2026-10-20, NOT sim-tested)

**What changed:**
- **f1tenth_ctl:**
  - `send planner` takes one line of at most 500 characters and answers the
    log's byte offset (`cursor`);
  - new `planner-log [--since N]` returns the planner's complete output lines
    after it, ANSI stripped.
- **Bot:**
  - a plain text message is a planner command: one line, ≤ 500 chars, sent
    < 10 s ago;
  - planner not running → a panel with ▶️ Planner;
  - planner running → `Send to planner: «…»?` with ✅/❌ (expires after
    10 s);
  - after ✅ the message is edited every 2 s with the planner's output;
  - the mission JSON is shown as a code block;
  - it stops at the planner's `RESULT {…}` line, at 60 s, or when the
    planner stops;
  - the panel has a 📝 Command button (an explanation only);
  - commands sent are logged with the user id;
  - `--cli` gets `say:<text>`.

**Checks:**

| Package | Tests |
|---|---|
| f1tenth_ctl | 92 passed |
| f1tenth_telegram, venv with python-telegram-bot | 73 passed |
| f1tenth_telegram, without it | 60 passed, 13 skipped |

**New tests:**
- line validation (break, length, empty);
- the log cursor:
  - complete lines only;
  - a partial line arriving whole later;
  - ANSI stripped;
  - reset and cap;
- `send` + `planner-log` against a real systemd unit (`cat` as the planner);
- output parsing: the prompt removed, the mission JSON, RESULT;
- trimming to 3500 chars and HTML escaping;
- the exchange: result, 60 s timeout, planner gone, send refused;
- the Telegram flow:
  - confirmation and send;
  - an expired confirmation;
  - cancel;
  - planner not running;
  - bad, foreign and stale text;
- `--cli say:`.

**Not tested:** the real `llm_planner_node` over this path. No llama-server
on linus; the parsing follows `llm_planner_node.py`'s prints, `JSON (…):` and
`RESULT {…}`.

## 10. Round 5: two-step planner commands and voice (2026-10-20, NOT sim-tested)

**Recon.**
- `llm_planner_node` already has a plan-only mode: `--dry-run`. It
  translates, prints and writes the mission file, but never calls the
  mission services, and it applies to every command in interactive mode.
- **The planner package was not changed.** f1tenth_ctl starts it with
  `--dry-run`.
- The RESULT line of an `ok` plan already carries the full mission, and
  f1tenth_ctl stores it from there.
- **Interlock.** `planner-log` reports `plan_only`, read from the planner
  process's argv. The bot plans only when it is `true`:
  - a planner started by the old f1tenth_ctl (no `--dry-run`) would load and
    start every plan by itself;
  - so would an Orin that still has the old f1tenth_ctl.

**What changed.**
- **f1tenth_ctl:**
  - the planner starts plan-only;
  - `planner-log` stores each RESULT's mission;
  - new commands `missions`, `mission-load`, `mission-start`,
    `mission-status`;
  - `status.mission`.
- **Bot:**
  - text, and voice once transcribed: Plan? → plan → ▶️ Start mission →
    "🚗 The car will drive. Start?" → load + start;
  - 10 s per question, plans live 5 min;
  - one planning at a time, none while a mission runs;
  - the panel shows the mission, and 🛑 comes first while it runs;
  - `--cli` gets `say:` / `voice:` with plan/start/yes.

**Unit tests:**

| Package | Tests |
|---|---|
| f1tenth_ctl | 104 passed |
| f1tenth_telegram, venv | 99 passed |
| f1tenth_telegram, system Python | 82 passed, 17 skipped |

New:
- the plan/start state machine:
  - plan → start / discard;
  - stale taps on either question;
  - plan older than 5 min, including when it ages between the questions;
  - no plan while planning or while a mission runs;
  - refusals from f1tenth_ctl: not READY, latched, mission running, invalid
    file;
- mission storage: a recorded success and a recorded error RESULT line,
  dedupe by offset, unique ids, never regenerated;
- refusals of `mission-load` / `mission-start`;
- the summary;
- voice:
  - model loading, unavailable, disabled;
  - duration limit;
  - temp files deleted on success and on failure;
  - language;
  - the Telegram voice path to the same Plan question;
- `--cli say:` and `voice:`.

**Real checks:**

| Check | Result |
|---|---|
| A stored-plan file through the stack's own validator (`load_mission_file`, sim workspace) | valid, `llm_a1b2c3, 3 move(s)`. A fixture with a wrong field was rejected with the loader's own message |
| Italian commands made offline with libespeak-ng → OGG/Opus, `small` vs `medium` on CUDA float16 (RTX 5050) | **small**: 1 of 3 right ("vai dritto per due metri…" → "Ma hai detto per due metri…"), ~0.15 s each. **medium**: 2 of 3 right ("Vai dritto per due metri e poi fermati."; "vai verso la persona" → "MAI MERZO LA PERSONA"), ~0.27 s each. Default: **medium** |
| The bot's own path (`--cli --do voice:check_it.ogg`, medium, device auto) | CUDA float16; model ready in 6.6 s; «Vai dritto per due metri e poi fermati.» in 0.32 s; then "planner not running" (no stack) |
| Idle memory | without the model: RSS 48 MB. With medium: RSS 890 MB, VRAM 1984 MiB. small: ~0.7 GB VRAM |
| PyAV | faster-whisper 1.2.1 fails with PyAV 19 (`metadata_errors`); PyAV 16.1.0 works |

**Not tested:**
- the real planner over this path (no llama-server on linus);
- a mission actually loaded and started on a stack;
- a human voice (only the synthetic one);
- the bot under systemd with the speech model.

