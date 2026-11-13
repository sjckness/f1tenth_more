# RUNBOOK — the office HIL campaign, command by command

Everything to type, in order, with the check that has to pass before the next step. Values are
this setup's own, read from `sim/isaac/config/hil.local.yaml`; nothing below needs editing.

| mark | where |
|---|---|
| **W** | the Windows workstation. `W$` = Git Bash, `W>` = `cmd` |
| **O** | the Orin, host shell over `ssh fabiocar@192.168.200.11` |
| **C** | inside the Orin container `f1tenth_hil_car` |

Four shells end up open at once: **W** for Isaac, **O1** stack, **O2** recorder, **O3** driver.

**Unattended instead (Phases 2–4 in one go, overnight):** after Phases 0–1,

```bat
W>  scripts\sim\win\hil_follow.bat          :: leave it open; it starts and restarts Isaac itself
```
```bash
O   scripts/sim/campaign/hil_night.sh start  # detaches; `status` / `stop` any time
```

and read `office_hil_campaign/MORNING.md` the next day: every step with its outcome, each run
with its evidence, and the verdicts still to fill (the night judges nothing). The follower talks
to the Orin only outward over ssh (needs the password-less key), so no port or firewall rule is
added; the chair go/no-go falls back to the `campaign_ma_bright` preset by itself. Details in the
headers of `scripts/sim/campaign/hil_night.sh` and `scripts/sim/win/hil_follow.py`.

---

## Phase 0 — get this work onto the Orin

The Orin needs this branch because `campaign.json` records the root's git commit as the
campaign's provenance, and because the Isaac presets and the driver live in it. The link has no
gateway, so do not count on the Orin reaching GitHub: a bundle over the link always works.

```bash
# W$  (in the workstation checkout)
git -C "/f/Users/Andreas Steffens/dev_ws/f1tenth_hil" status --short      # what is about to go
git -C "/f/Users/Andreas Steffens/dev_ws/f1tenth_hil" add -A
git -C "/f/Users/Andreas Steffens/dev_ws/f1tenth_hil" commit -m "office_hil_campaign: 3 commands x 3 repetitions, Orin in the loop"
cd "/f/Users/Andreas Steffens/dev_ws/f1tenth_hil"
git bundle create ../campaign.bundle planner-modes-hil
git rev-parse --short HEAD                                   # <-- note it, call it WS_HEAD
scp ../campaign.bundle fabiocar@192.168.200.11:/tmp/
```

```bash
# O
cd /home/fabiocar/dev_ws/f1tenth_hil
git status --porcelain            # MUST be empty. If not: git stash -u (and say so in the notes)
git fetch /tmp/campaign.bundle planner-modes-hil
git checkout -B campaign-office FETCH_HEAD
git rev-parse --short HEAD        # must equal WS_HEAD
ls office_hil_campaign/prompts.yaml scripts/sim/campaign/hil_campaign.sh
```

**Stop if** the two short hashes differ: the campaign would record a commit that is not what ran.

---

## Phase 1 — recreate the container (one time only)

Mounts are fixed when a container is created, so this is a recreate, not a reconfigure. Make the
archive directory **first**: if docker creates it, it belongs to root and the recorder cannot
write.

```bash
# O
mkdir -p ~/f1tenth_sim_archive
docker rm -f f1tenth_hil_car
docker run -d --name f1tenth_hil_car --network host --ipc host --runtime nvidia \
  --init --restart no \
  -v /home/fabiocar/projects/llm/models:/home/fabiocar/projects/llm/models:ro \
  -v /home/fabiocar/dev_ws/f1tenth_hil:/home/fabiocar/dev_ws/f1tenth_hil \
  -v /home/fabiocar/f1tenth_sim_archive:/home/fabiocar/f1tenth_sim_archive \
  -e F1T_CONTAINER_ROLE=car -e F1T_AUTOSTART=none \
  -e ROS_DOMAIN_ID=44 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e F1TENTH_DISCOVERY_SERVER=192.168.200.11:11812 \
  f1tenth:jazzy-orin-ecb457b-norust sleep infinity
```

Four checks, all four must answer:

```bash
# O
docker inspect -f '{{.HostConfig.NetworkMode}}' f1tenth_hil_car            # host
docker exec f1tenth_hil_car ls /home/fabiocar/dev_ws/f1tenth_hil/scripts/sim/campaign/hil_campaign.sh
docker exec f1tenth_hil_car ls /home/fabiocar/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf
docker exec f1tenth_hil_car /opt/llama.cpp/bin/llama-server --version
touch ~/f1tenth_sim_archive/.w && docker exec f1tenth_hil_car touch /home/fabiocar/f1tenth_sim_archive/.w2 && ls ~/f1tenth_sim_archive
```

**Stop if** `NetworkMode` is not `host`: the Orin's kernel cannot load `bridge.ko`, so a
container on any other network silently comes up with only `lo` and no DNS.

### Optional: the two robustness patches

The container runs the **image's** `f1tenth_ws`, not this checkout, so the two patches taken from
`origin/doors-enter` (a corrupted run no longer aborts the whole export; `analyze_tests`
isolates a bad run) are not active unless that one package is overlaid. Nothing else needs it,
and the campaign runs without it.

```bash
# C   (scripts/sim/hil_orin shell)
cd /home/fabiocar/dev_ws/f1tenth_hil
colcon build --symlink-install --packages-select f1tenth_logger
```

`hil_campaign.sh` sources `install/setup.bash` when it exists, so after this every call uses the
overlay — all of them or none, never a mix.

---

## Phase 2 — preflight both sides

```bash
# O
cd /home/fabiocar/dev_ws/f1tenth_hil
scripts/sim/hil_orin preflight          # docker, host net, net.core.rmem_max >= 16 MB, link, DS port
```

```bat
W> cd /d f:\Users\Andreas Steffens\dev_ws\f1tenth_hil
W> scripts\sim\win\hil_preflight.bat --orin
```

**Stop if** `rmem_max` is below 16 MB — a 0.92 MB image is fragmented into ~15 datagrams that
must all arrive, and the default 64 kB buffer drops them in bursts. `hil_orin preflight` prints
the `sysctl` line to run with sudo.

---

## Phase 3 — the smoke run: M02, once, end to end

M02 is the only mission that needs no perception at all — no object to recognise, no door — so
it is the one that tells you whether the *pipeline* works, with nothing else mixed in.

### 3a. Isaac, with M02's scene

```bat
W> scripts\sim\win\hil_sim_link.bat --layout campaign_mb
```

Watch `scripts\sim\win\run.log` for `RTF 1.000` and no `XMLPARSER` lines. Leave it running.

```bat
W> scripts\sim\win\hil_topics.bat
```

### 3b. The stack, on the Orin — terminal O1

```bash
# O1
cd /home/fabiocar/dev_ws/f1tenth_hil
scripts/sim/hil_orin stack enable_intelligence:=true enable_mission_logger:=true \
      planner_controller_mode:=corridor_mpc
```

`enable_intelligence:=true` is what starts llama-server; it reloads the model in 20–30 s.

### 3c. The HIL gate — the one check that decides everything

```bash
# O   (a second shell; the stack stays in O1)
scripts/sim/hil_orin check
```

This drives the car for 2 s. It answers `/clock` and the RTF, every sensor topic and its QoS,
the rates, the stamp age (the link delay), the whole TF chain and the loop rate.

**Stop here if it fails.** `output/hil/recon.md` records that the Orin container has never run
`sim:=isaac`, so this is the first time that combination is exercised; nothing further is worth
attempting until it passes.

```bash
# O
docker exec f1tenth_hil_car curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8083/health   # 200
```

503 means the model is still loading — wait a minute and repeat.

### 3d. The recorder — terminal O2, started once and left alone

It owns the campaign's repetition counter and it must be up **before** any command is sent, or
the run is simply not recorded. It survives the stack restarts between missions.

```bash
# O2
cd /home/fabiocar/dev_ws/f1tenth_hil
scripts/sim/hil_orin shell
# C:
ros2 launch f1tenth_logger test_campaign_logger.launch.py \
  root:=/home/fabiocar/dev_ws/f1tenth_hil \
  config:=/home/fabiocar/dev_ws/f1tenth_hil/src/f1tenth_logger/config/office_hil_campaign_logger.yaml
```

The config is given by its path in the **source** tree on purpose: it then needs no build. Its
startup lines must print `.../office_hil_campaign` and every subscription.

### 3e. Translation check — no motion

```bash
# O3
cd /home/fabiocar/dev_ws/f1tenth_hil
scripts/sim/campaign/hil_campaign.sh preflight
scripts/sim/campaign/hil_campaign.sh translate
```

Safe with the recorder running: `--dry-run` returns before the planner announces a plan, so it
cannot open a test folder.

**Stop if** any command reports a non-empty `unsupported`: the planner would ask an interactive
`[s/N]` that nothing can answer, and the session stalls.

### 3f. One real run

```bash
# O3
scripts/sim/campaign/hil_campaign.sh mission M02 --reps 1
```

It prints a verdict sheet at the end. Pass means **all** of:

```bash
# O3
ls office_hil_campaign/M02_wall_left_wall/P002-R001-*/            # kinematics.csv, imu.csv, mpc.csv,
                                                                  # corridors.jsonl, plan.json, events.jsonl
grep mission_finished office_hil_campaign/M02_wall_left_wall/P002-R001-*/events.jsonl
cat office_hil_campaign/aux/M02/*/verdict_sheet.txt               # final GT pose near (11.63, -21.62)
scripts/sim/campaign/hil_campaign.sh bags                         # one row, outcome COMPLETE
```

and the bag's first message must precede `mission_started` — that is what the 15 s countdown the
driver sets is for.

### 3g. The whole back half, on that one run

Prove the analysis chain before producing eight more runs for it.

```bash
# O3
scripts/sim/campaign/hil_campaign.sh judge M02
#   -> it prints the unjudged test_id and stops. Now fill, in
#      office_hil_campaign/campaign_results.csv, for that row:
#        success    1 or 0     (the verdict sheet has the measurement)
#        transl_ok  1 or 0     (plan.json vs the expected_sequence comment in prompts.yaml)
#        notes      free text
scripts/sim/campaign/hil_campaign.sh analyze
grep "evaluated by hand" office_hil_campaign/analysis/report.txt   # must say 1
```

---

## Phase 4 — the nine runs

Repetition numbers are allocated by the recorder as `max(existing) + 1`, so after the smoke run
M02 needs two more, not three.

```bash
# O3
scripts/sim/campaign/hil_campaign.sh mission M02 --reps 2        # -> R002, R003
scripts/sim/campaign/hil_campaign.sh judge M02
```

### Boundary: M02 → M01

The scene is fixed at Isaac launch, so this is manual, in this order:

```
1. O1: Ctrl+C the stack, then WAIT 8 s        (5 s Fast DDS participant lease: the next
                                               bringup must not see this one's nodes)
2. W:  Ctrl+C Isaac, then
       scripts\sim\win\hil_sim_link.bat --layout campaign_ma
       wait for RTF 1.000 in run.log
3. O1: scripts/sim/hil_orin stack enable_intelligence:=true enable_mission_logger:=true \
             planner_controller_mode:=corridor_mpc
4. O2: leave the recorder alone.
```

### M01's go/no-go: can the camera see the chair at all

Do this **before** the three runs. With `campaign_ma` up, the stack running and **no mission**:

```bash
# C   (scripts/sim/hil_orin shell)
ros2 topic echo --once /camera/detections
ros2 topic echo --once /costmap/semantic_tracks
```

A `chair` must appear within about 5 s — that is `go_to`'s acquire timeout, so what you cannot
see standing still, the mission cannot acquire either. If nothing comes: raise the three lights
`campaign_ma` adds (`sim/isaac/config/scenes/office.yaml`, intensity 30000 → 60000), restart
Isaac, and look again. Headless `RaytracedLighting` has no indirect diffuse; this is the most
likely thing to go wrong in the whole campaign, and it is a per-mission go/no-go.

```bash
# O3
scripts/sim/campaign/hil_campaign.sh mission M01 --reps 3
scripts/sim/campaign/hil_campaign.sh judge M01
```

### Boundary: M01 → M03

Same four steps, with `--layout campaign_md`. Then confirm the person is actually in the scene —
the driver checks this too and refuses to send the command without it:

```bash
# C
ros2 topic echo --once /sim/ground_truth/objects | grep -A 2 class_id
```

```bash
# O3
scripts/sim/campaign/hil_campaign.sh mission M03 --reps 3
scripts/sim/campaign/hil_campaign.sh judge M03
```

---

## Phase 5 — report and bring it home

```bash
# O3
scripts/sim/campaign/hil_campaign.sh analyze
cat office_hil_campaign/analysis/report.txt | head -40
ls office_hil_campaign/analysis/M0*/            # per-mission report, summary, dashboard, overview map
cat office_hil_campaign/analysis/summary_table.csv
scripts/sim/campaign/hil_campaign.sh bags
```

The campaign is finished when `analysis/report.txt` reads `not yet evaluated: 0`. Anything else
means the data exists and the verdicts do not, which is what happened to the first campaign.

```bash
# O   commit the campaign on the Orin and bundle it back
cd /home/fabiocar/dev_ws/f1tenth_hil
git add office_hil_campaign && git commit -m "office_hil_campaign: 9 runs, judged"
git bundle create /tmp/campaign_results.bundle campaign-office
```

```bash
# W$
scp fabiocar@192.168.200.11:/tmp/campaign_results.bundle ..
git fetch ../campaign_results.bundle campaign-office
# the bags are large: pull the manifests first, the bags on demand
rsync -a --include='*/' --include='*.manifest.json' --exclude='*' \
      fabiocar@192.168.200.11:f1tenth_sim_archive/complete/ ./bags_manifests/
rsync -a --info=progress2 fabiocar@192.168.200.11:f1tenth_sim_archive/complete/ ./bags/
```

---

## If something stops

| symptom | what it is |
|---|---|
| `hil_campaign.sh preflight` says the container cannot see the repo | the repo bind mount is missing — Phase 1, and it needs a container recreate |
| `preflight` says `test_campaign_logger` is not running | terminal O2, step 3d. Without it a run is not recorded at all |
| `translate` reports a non-empty `unsupported` | an ordinal or an unsupported target in `prompts.yaml`; the planner will ask `[s/N]` |
| `pc_record.py ready` aborts naming a second controller | a stack from before is still in the graph: Ctrl+C it and wait 8 s |
| the scene guard fails on the start pose | Isaac is running the wrong `--layout` for this mission |
| the scene guard fails on the person | `--layout campaign_md` was not used |
| `restart.sh` fails a check | the repetition is deliberately **not** started; that is the finding, not an error to work around |
| `go_to` ends `target_not_found` | lighting — see M01's go/no-go above |
| `enter_door` ends `door_not_found` / `no_strong_door` | it needs two sightings from poses ≥ 1 m apart inside a 5 s acquire timeout; raise the timeout or lower the spread, and **write the change in the notes** |
