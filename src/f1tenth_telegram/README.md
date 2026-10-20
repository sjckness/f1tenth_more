# f1tenth_telegram

A Telegram control panel for the car.

- It runs on **linus**, the always-on hub, never on the car.
- It has no ROS dependency. It reaches the car only through
  `ssh <host> f1tenth_ctl ... --json` over Tailscale.
- `--cli` runs the same handlers in a terminal, without Telegram or a token.

```
Telegram ⇄ (long polling) ⇄ f1tenth_telegram on linus ─ssh─▶ f1tenth_ctl on the Orin
```

> **⚠️ The Telegram 🛑 is NOT the primary emergency stop.**
>
> It goes through Telegram's servers, linus, ssh over Tailscale and a fresh
> ROS node on the car. That adds up to **seconds**: in the sim, about 3–5 s
> from the tap to zero drive at the mux.
>
> Use it as a backstop when the car is out of reach. Keep the RC transmitter,
> or the physical switch, in hand whenever the car can move.

## The panel

One status message per chat, edited in place after every action. Send
`/panel` or `/start` to get one; a newer panel replaces the old one.

**First line:** `Orin 🟢 online` or `🔴 offline`.

- Online means one `ssh <host> true` round trip worked. The ping result is
  extra information.
- Offline: only 🔄.

**Online:** 🔌 Connect and 🔄. Connect runs `f1tenth_ctl connect`, which:

- starts the discovery server if it is down;
- checks the environment: ROS sourced, workspace packages, systemd user
  manager, thresholds file.

When both are ok the panel says "ready to launch".

**Connected:** the panel shows

- READY ✅ or ❌, with the failing checks;
- each process: 🟢 running, ⚪ stopped, 🔴 crashed (exit code), 🟠 running
  but not started by the bot;
- odometry and scan rates, the map→base_link TF age, the battery and the
  mission state.

The buttons:

| Button | Action | Confirmation |
|---|---|---|
| ▶️/⏹ Stack | start/stop the stack | start |
| ▶️/⏹ Planner | start/stop the LLM planner | start |
| ▶️/⏹ Logger | start/stop the mission logger | no |
| 🔄 Status | refresh | no |
| ☠️ Kill all ROS 2 | `f1tenth_ctl killall` | yes |
| 🛑 EMERGENCY | `f1tenth_ctl estop` | **never** |
| 🔓 Reset e-stop (only while latched) | `f1tenth_ctl estop-reset` | yes |

**After 🛑:**
- the e-stop latches;
- the panel shows `🔴 E-STOP LATCHED` and READY ❌ `estop_latched`;
- a 🔓 Reset e-stop button appears.

**The reset** aborts the interrupted mission, holds zero drive on the top mux
lane, restarts only `behavior`, and checks that the latch reads clear (see
`src/f1tenth_ctl/README.md`, estop-reset). It takes about 10 s.

**Busy handling.** One action runs at a time. A second tap meanwhile is
refused, not queued, because a queued command would run late. 🛑 never waits:
it runs even while a killall is in progress. `/estop` works without the panel.

## Safety rules in the code

- **Whitelist.** Only the numeric user ids in `telegram.allowed_user_ids` can
  use the bot. Updates from anyone else are ignored, never answered, and
  logged with the sender's id.
- **Stale commands.**
  - A message older than `max_command_age_s` (10 s) is dropped.
  - Telegram does not timestamp button presses, so buttons are guarded
    differently:
    - pending updates are dropped when the bot starts;
    - a button of an old panel is refused;
    - a confirmation expires 10 s after the question. Confirmed actions:
    start stack, start planner, kill all, reset e-stop.
- **Timeouts.** Every ssh call has a hard timeout (`timeouts_s`). A timeout or
  an ssh failure triggers a fresh online check, so a car that went away shows
  as 🔴 offline.

## One polling process per token: a dev bot and a prod bot

Telegram allows **one** `getUpdates` consumer per bot token. If a second
process polls with the same token (a test in a terminal, a copy on another
machine), both get `409 Conflict` and lose updates.

So create **two bots** with @BotFather:

- **prod:** used only by the systemd unit on linus;
- **dev:** for running by hand while developing.

Give each its own token file, and never put a token in the repository.

## Install on linus

1. Create the venv:

   ```bash
   python3 -m venv --system-site-packages ~/.local/share/f1tenth_telegram/venv
   ~/.local/share/f1tenth_telegram/venv/bin/pip install 'python-telegram-bot>=21,<23'
   ```

   `--system-site-packages` gives the venv the system PyYAML. The bot was
   tested with python-telegram-bot 22.8.

2. Run the tests (they include the Telegram glue, without contacting
   Telegram):

   ```bash
   cd ~/dev_ws/f1tenth_more_telegram/src/f1tenth_telegram
   ~/.local/share/f1tenth_telegram/venv/bin/python -m pytest test/
   ```

Without python-telegram-bot, `colcon test` / `pytest` skip the 7 tests in
`test_tg.py` and run the rest.

## Test without Telegram: --cli

```bash
cd ~/dev_ws/f1tenth_more_telegram/src/f1tenth_telegram
# interactive: type a button number, an action name, or q
bin/f1tenth_telegram --cli --config config/telegram_sim.yaml
# scripted
bin/f1tenth_telegram --cli --config config/telegram_sim.yaml \
  --do connect --do start:stack --do confirm --do until-ready:240 --do estop
```

- Actions:
  - `refresh`, `connect`, `status`, `estop`;
  - `killall`;
  - `start:<p>` and `stop:<p>` for `p` = stack, planner or logger;
  - `confirm` and `cancel`.
- Scripted mode also accepts `wait:S` and `until-ready:S`.
- `config/telegram_sim.yaml` runs f1tenth_ctl locally with the Gazebo sim
  profile. `config/telegram.yaml` is the car.

## Create the bot

1. **BotFather.** In Telegram, open **@BotFather**:
   1. Send `/newbot`, pick a name and a username ending in `bot`, and copy
      the token.
   2. Do it twice, for prod and dev.
   3. Optional:
      - `/setprivacy` → Enable;
      - `/setjoingroups` → Disable;
      - `/setcommands` → `panel - control panel`, `estop - emergency stop (backstop)`.
2. **Your user id.** Message **@userinfobot**, which answers with your
   numeric id. Alternatively, start the bot with an empty whitelist, send it
   `/panel`, and read the id in its log line `ignored an update from user id …`.
3. **Whitelist.** Put the id in `config/telegram.yaml`:

   ```yaml
   telegram:
     allowed_user_ids: [123456789]
   ```

4. **Token file, outside the repository:**

   ```bash
   mkdir -p ~/.config/f1tenth_telegram
   ( umask 077; echo 'TELEGRAM_BOT_TOKEN=<prod token>' > ~/.config/f1tenth_telegram/env )
   chmod 600 ~/.config/f1tenth_telegram/env
   ```

   The bot refuses a token file that group or others can read. The dev bot:
   run by hand with `TELEGRAM_BOT_TOKEN=<dev token>` in the environment,
   which wins over the file.

5. **ssh to the car.** Check BatchMode ssh from linus:
   `ssh -o BatchMode=yes jetson-orin true`. It works today: key
   `~/.ssh/id_ed25519`, and `~/.ssh/config` has `jetson-orin → 100.96.201.7`,
   user `fabiocar`.
6. **f1tenth_ctl on the car.** f1tenth_ctl must exist at
   `target.ctl_command` on the car. See `src/f1tenth_ctl/README.md`,
   "On the Orin".
7. **First run by hand:**
   `PYTHON=~/.local/share/f1tenth_telegram/venv/bin/python bin/f1tenth_telegram`,
   then `/panel` in Telegram.
8. **systemd user unit.** `systemd/f1tenth-telegram.service` is in the repo
   and not installed. The commands are in its header:

   ```bash
   cp systemd/f1tenth-telegram.service ~/.config/systemd/user/
   systemctl --user daemon-reload && systemctl --user enable --now f1tenth-telegram
   sudo loginctl enable-linger andreas   # Linger is off on linus today
   journalctl --user -u f1tenth-telegram -f
   ```

   The unit points at the `~/dev_ws/f1tenth_more_telegram` worktree. Change
   the paths if the bot is meant to run from another checkout.
