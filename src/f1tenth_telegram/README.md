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

**The 🛑 reply** gives one line for the result, then one line per step:

```
🛑 E-STOP: car held ✅
✅ zero drive on /safety_stop: 100 msgs to 1 subscriber(s) (2250 ms)
✅ emergency_stop (latch): emergency stop engaged (640 ms)
✅ abort_mission (mission + MPC hold): no mission to abort (state=IDLE) (9 ms)
```

"car held" means the zero drive reached the mux, and either the latch or the
abort answered. Otherwise the first line says `car NOT confirmed held ❌`.

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

## Setup on linus

These are the steps that worked on 2026-10-20. linus runs zsh.

1. **BotFather.** In Telegram, open **@BotFather**:
   - send `/newbot`, pick a name and a username ending in `bot`;
   - `/setjoingroups` → Disable;
   - `/setprivacy` → Enable;
   - `/setcommands` → `panel - control panel` and `estop - emergency stop (backstop)`.

   For development, make a second bot the same way (see above).
2. **Your user id.** Ask **@userinfobot**. Put the id in
   `config/telegram.yaml`; it is committed:

   ```yaml
   telegram:
     allowed_user_ids: [857433485]
   ```

3. **Token file, outside the repository.** Type the token at a hidden
   prompt, so it never reaches the screen, the shell history or `ps`:

   ```zsh
   mkdir -p ~/.config/f1tenth_telegram && chmod 700 ~/.config/f1tenth_telegram
   read -rs "T?Token: "; echo
   ( umask 077; printf '%s=%s\n' TELEGRAM_BOT_TOKEN "$T" > ~/.config/f1tenth_telegram/env )
   unset T
   chmod 600 ~/.config/f1tenth_telegram/env
   ```

   The bot refuses the file if group or others can read it.
4. **venv with python-telegram-bot.** Tested with 22.8.
   `--system-site-packages` gives the venv the system PyYAML.

   ```zsh
   python3 -m venv --system-site-packages ~/.local/share/f1tenth_telegram/venv
   ~/.local/share/f1tenth_telegram/venv/bin/pip install 'python-telegram-bot>=21,<23'
   ```

5. **Run by hand once**, then send `/panel` to the bot in Telegram. Stop it
   with Ctrl-C before step 6: two pollers on one token conflict.

   ```zsh
   export PYTHON=~/.local/share/f1tenth_telegram/venv/bin/python
   cd ~/dev_ws/f1tenth_more_telegram/src/f1tenth_telegram
   bin/f1tenth_telegram --config config/telegram.yaml
   ```

   The first log line reads
   `starting: ipv4 forced yes, timeout 20 s, target Orin (jetson-orin), 1 whitelisted user(s)`.
6. **systemd user unit**, plus Linger so it runs without a login:

   ```zsh
   mkdir -p ~/.config/systemd/user
   cp ~/dev_ws/f1tenth_more_telegram/src/f1tenth_telegram/systemd/f1tenth-telegram.service \
      ~/.config/systemd/user/
   systemctl --user daemon-reload
   systemctl --user enable --now f1tenth-telegram
   sudo loginctl enable-linger andreas
   journalctl --user -u f1tenth-telegram -f
   ```

   - The unit uses `PYTHON` from the venv and the token file as its
     `EnvironmentFile`.
   - It waits for `network-online.target`, and restarts always, after 5 s.
   - The bot itself retries getMe and polling forever
     (`bootstrap_retries=-1`).
   - After a `git pull`, run `systemctl --user restart f1tenth-telegram`. If
     the unit file changed, copy it again and `daemon-reload` first.
7. **ssh to the car**, for the panel to show 🟢:
   `ssh -o BatchMode=yes jetson-orin true` must work. f1tenth_ctl must exist
   at `target.ctl_command` on the car (`src/f1tenth_ctl/README.md`, "On the
   Orin").

**Networking** (`telegram:` in `config/telegram.yaml`):

| Key | Default | What it does |
|---|---|---|
| `force_ipv4` | `true` | Every Bot API connection goes over IPv4 (an httpx transport bound to `0.0.0.0`). linus's IPv6 route is broken: getMe timed out while `curl -4` answered in 0.2 s |
| `timeout_s` | 20 | Connect, read, write and pool timeout of each request. Long polling adds its own poll time to the read timeout |

**Tests:**

```zsh
cd ~/dev_ws/f1tenth_more_telegram/src/f1tenth_telegram
~/.local/share/f1tenth_telegram/venv/bin/python -m pytest test/
```

They include the Telegram glue, without contacting Telegram. Without
python-telegram-bot, `colcon test` / `pytest` skip the 8 tests that need it
and run the rest.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `httpx.ReadTimeout` / `TimedOut` at start (getMe) | IPv6 broken: `curl -6 https://api.telegram.org` fails, `curl -4` works | `force_ipv4: true` (the default); raise `timeout_s` on a slow link |
| `Conflict: terminated by other getUpdates request` (409) | Two processes poll with one token | Stop one: `systemctl --user stop f1tenth-telegram`, or Ctrl-C the manual run. `pgrep -af f1tenth_telegram` shows them. Use the dev bot for manual runs |
| `… is readable by others: chmod 600 …` | Token file permissions | `chmod 600 ~/.config/f1tenth_telegram/env` |
| `no TELEGRAM_BOT_TOKEN and no token file` | No env file | Step 3 above |
| `ModuleNotFoundError: No module named 'telegram'`, or "python-telegram-bot is not installed" | Started with the system Python | `export PYTHON=~/.local/share/f1tenth_telegram/venv/bin/python`; the unit sets it |
| The bot never answers | Your id is not in `allowed_user_ids` | Look for `ignored an update from user id …` in the journal |
| `Dropped: older than 10 s` | Delayed delivery, or a wrong clock on linus | Send again; check `timedatectl` |
| Panel 🔴 offline, `ssh: connect to host … timed out` | The Orin is off, not on Tailscale, or ssh fails | `tailscale status \| grep jetson-orin`; `ssh -o BatchMode=yes jetson-orin true` |
| Connect: `f1tenth_ctl not found` | Not deployed in the Orin's checkout | Merge and pull there (`src/f1tenth_ctl/README.md`) |

## Security

- **Never paste the token anywhere:** not in a chat, an issue, a commit, a
  shell command line, history or a log.
  - Enter it only with `read -rs` (step 3).
  - The bot keeps httpx's request log at WARNING, because Bot API URLs
    contain the token.
- **If it leaks:** in @BotFather, `/revoke`, then pick the bot. Write the new
  token to the env file as in step 3, then
  `systemctl --user restart f1tenth-telegram`.
- **Groups:** `/setjoingroups` → Disable, so nobody can add the bot to a
  group.
- **Access:** the whitelist is the only access control. Keep it to the people
  who may stop the car.
- **Never commit** the env file, a venv or a token.

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
