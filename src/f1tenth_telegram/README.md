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
| 📝 Command | explains that a message (typed or spoken) is a planner command | no |
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

## Talking to the planner

> **Safety: a message never starts the car.**
>
> Typed or spoken, a command only produces a *plan*. A mission runs only
> after two more taps:
> - ▶️ Start mission under the plan;
> - ✅ Yes, start under "🚗 The car will drive. Start?".
>
> f1tenth_ctl then refuses the start unless READY, the e-stop is clear and no
> other mission runs. Once started, the mission is stopped only by 🛑, by
> its own end, or by the car's own safety.

**Any plain text message** from a whitelisted user (not a `/command`), or a
voice message once transcribed (see Voice commands), goes through these
steps:

1. **Checks.** Line breaks collapse to spaces; the result must be at most 500
   characters and sent less than 10 s ago. Two refusals:
   - a planning already in progress, or a mission starting or running: "⏳
     Not planned: …", with the reason;
   - planner not running: a fresh panel with ▶️ Planner.

   Otherwise: `Plan: «text»?` with **[🧭 Plan] [❌ Cancel]**, valid for 10 s.
2. **🧭 Plan.** `f1tenth_ctl send planner "<text>"` sends it to the planner.
   - The planner runs **plan-only**: `llm_planner_node --dry-run`, its own
     flag. It translates, prints and writes the mission, and never calls
     `/mission/load_mission` or `/mission/start_mission`.
   - The message follows the planner's output, edited every 2 s with
     `planner-log --since <cursor>`.
   - It stops at the planner's `RESULT {…}` line, at 60 s, or when the
     planner stops.
   - f1tenth_ctl stores the planned mission from that RESULT line under an id
     (`<timestamp>-<hash>`), exactly as planned. It is never regenerated.
3. **The plan.** One message with:
   - a human summary: mission id, number of moves, max speed (or "controller
     default"), reverse (the schema has none), targets, length (or "not
     known ahead" when a move ends on a condition), the longest it can take
     (the sum of the timeouts), one line per move;
   - anything the planner could **not** translate, marked ⚠️;
   - the mission JSON, collapsed;
   - **[▶️ Start mission] [❌ Discard]**.

   If the planner refused or failed, the message gives its reason and has no
   Start button.
4. **▶️ Start mission.** A second question, `🚗 The car will drive. Start?`,
   with **[✅ Yes, start] [❌ No]**, valid for 10 s.
5. **✅ Yes, start.** `f1tenth_ctl mission-load <id>` loads *that* stored
   plan, after validating it with the stack's own parser. Then
   `mission-start` checks again and starts it. The mission begins after the
   loader's 15 s countdown.

**The rules:**
- **Time limits.** Every question expires 10 s after it is asked. A plan
  older than 5 min cannot be started: plan again.
- **One at a time.** One planning at a time. A newer plan replaces the
  offer of an older one, and one plan starts once.
- **The panel** shows the current mission (id, state, elapsed). While a
  mission is starting or running, 🛑 EMERGENCY is the panel's **first** row.
- **The journal** logs every planner command, plan and start attempt with the
  user id.

**On the car:**
- The planner needs llama-server: the stack's `intelligence` component.
  Without it, the planner exits after 90 s and shows as crashed.
- f1tenth_ctl starts the planner with `--dry-run` (`config/ctl.env`). Started
  by hand without it, the planner behaves as before: it loads and starts
  what it plans.
- **The bot plans only through a plan-only planner.** `planner-log` reports
  `plan_only`, read from the planner's command line. The bot checks it
  before "Plan?" and again at 🧭 Plan, and refuses unless it is `true`:
  - a planner started without `--dry-run`, for example by an older
    f1tenth_ctl, is refused;
  - so is an older f1tenth_ctl that does not report `plan_only`.

  Fix: update src/f1tenth_ctl on the car, then ⏹ and ▶️ Planner.

**In a terminal**, the same steps:

```
bin/f1tenth_telegram --cli --config … --do 'say:vai dritto per due metri' --do plan --do start --do yes
bin/f1tenth_telegram --cli --config … --do 'voice:/path/cmd.ogg' --do plan
```

## Voice commands

**What happens.** Send the bot a voice note, or an audio file.
- It is transcribed **on linus**, with faster-whisper inside the bot.
- The bot replies `🎙 transcribing…`, then edits that message to
  `🎙 «transcript»` with the hint "wrong? type the command as text".
- The same `Plan: «…»?` follows: exactly the steps above. **There is no path
  from a voice message to a start** that skips 🧭 Plan, ▶️ Start mission and
  ✅ Yes, start.
- It accepts voice notes and audio files from whitelisted users only, at most
  `max_duration_s` (60 s). The transcript goes through the same 500-character
  check as text.

**Setup:**

```zsh
V=~/.local/share/f1tenth_telegram/venv
$V/bin/pip install faster-whisper 'av<17'        # PyAV 19 breaks faster-whisper 1.2.1
$V/bin/pip install nvidia-cublas-cu12 'nvidia-cudnn-cu12==9.*'   # GPU only, ~2.2 GB
```

**CUDA notes.**
- faster-whisper (CTranslate2) loads cuBLAS 12 and cuDNN 9 at the first
  inference. They come from the two wheels above, and only through
  `LD_LIBRARY_PATH`:
  - the systemd unit sets it;
  - `bin/f1tenth_telegram` adds it for manual runs.
- `device: auto` tries CUDA float16 with a warm-up, then falls back to CPU
  int8.
- **Tested on 2026-10-20** on linus, an RTX 5050 (compute 12.0, driver 580):

  | | small | medium (the default) |
  |---|---|---|
  | VRAM | ~0.7 GB | ~2.0 GB |
  | Bot RSS at idle (48 MB without a model) | ~0.8 GB | ~0.9 GB |
  | Transcribing a 2 s command | ~0.15 s | ~0.3 s |
  | Load (cached) | 1 s | 3–7 s |

- The bot's VRAM is taken for as long as it runs. Mind it next to Isaac
  Sim. `model: small` gives most of it back, at a cost in Italian accuracy.

**Config** (`transcription:` in `config/telegram.yaml`):

| Key | Default | |
|---|---|---|
| `enabled` | `true` | `false`: voice messages get "voice commands are disabled" |
| `model` | `medium` | `small` misheard the verb in 2 of 3 synthetic Italian commands, medium in 1 of 3 |
| `device` | `auto` | `cuda` / `cpu` to force one |
| `language` | `it` | a Whisper language code, or `auto` |
| `max_duration_s` | 60 | longer audio is refused, before it is downloaded |

**Behaviour while loading or broken.**
- The model loads **once**, in a background thread, when the bot starts.
  Start-up is not blocked. Until it is ready, a voice message gets "the
  speech model is still loading, retry in a few seconds".
- Transcription runs in a worker thread, never on the asyncio loop: 🛑 and
  the panel stay responsive during it.
- If faster-whisper is missing or the model cannot load, the bot runs on
  without voice. Voice messages then get "transcription unavailable", logged
  once.

**Privacy.**
- The audio is downloaded from Telegram into a temporary directory on linus,
  transcribed there, and the directory is removed in every case. No audio is
  kept.
- Nothing is sent to any speech service: the model runs locally. On its first
  use, the model is downloaded once from Hugging Face.
- The journal keeps the user id, the audio's length, the transcription time
  and the transcript.

**A test command without a phone:**
`python3 tools/make_voice_sample.py "vai dritto per due metri" /tmp/cmd.ogg`.
It uses libespeak-ng to make an OGG/Opus file, then `--do voice:/tmp/cmd.ogg`.

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

   For voice messages, also install faster-whisper (and the CUDA wheels):
   see "Voice commands".

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
| Voice: "transcription unavailable" | faster-whisper missing, or the model failed to load | The journal says why, once. Install it (Voice commands); `device: cpu` if CUDA fails |
| Voice on CPU although there is a GPU; `libcublas.so.12` / `libcudnn` not found | `LD_LIBRARY_PATH` lacks the venv's nvidia wheels | Use the unit, or `bin/f1tenth_telegram`, which set it; or install the wheels |
| `TypeError: open() got an unexpected keyword argument 'metadata_errors'` | PyAV ≥ 17 with faster-whisper 1.2.1 | `pip install 'av<17'` (16.1.0 works) |
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
- `help:command` is the 📝 Command button.
- `say:<text>` (a typed message) or `voice:<file>` (a voice message,
  transcribed here), then `plan` / `cancel`, then `start` / `discard`, then
  `yes` / `no`: the steps of "Talking to the planner".
- `config/telegram_sim.yaml` runs f1tenth_ctl locally with the Gazebo sim
  profile. `config/telegram.yaml` is the car.
