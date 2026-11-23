# WORK_FINDINGS — macchina work (DESKTOP-Q3F81S6), 2026-11-23

Raccolta fatta in sola lettura su `work` per preparare un batch HIL office comandato dall'Orin.
Due parti: **(A)** esito della ricerca del report "36 test fattibili", **(B)** le informazioni
operative per lanciare il batch.

---

## A. La lista cercata: trovata, ed è di **35** pose, non 36

Non era in git e non era in nessun report: era negli **scratchpad delle sessioni Claude Code**
del 2026-11-13, in `%LOCALAPPDATA%\Temp\claude\56e189eecf0f421f\`.

**`door_runup.json`** (sessione `e41e0b8e`, 2026-11-13 14:57, 87 KB) — output di
`sim/isaac/tools/find_door_runup.py --run 5 --side left`:

> **130 varchi nel mondo, 35 rincorse tenute.**

Una riga = un test fattibile con la sua posa di partenza `{x, y, z, yaw_deg}` in
`husarion_office_2x`: una posa da cui l'auto percorre **5 m in rettilineo** e arriva con un
**varco esattamente 2 m alla propria sinistra** — la geometria di
`"go straight 5m then enter the door on the left"`. Tenuto solo se la posa sta su pavimento con
più di un raggio di footprint di franco, la corsa è libera per almeno 5 m e il franco non scende
mai sotto quel raggio lungo il tragitto. Per ogni riga: varco e larghezza, `free_ahead_m`,
`free_beyond_arrival_m`, `min_lateral_clearance_m`, `start_clearance_m`.

Di quei 35 candidati **in git era finito solo il migliore**, come preset `door_left` in
`sim/isaac/config/scenes/office.yaml` (commit `c72de207`, 1,25 m di varco, 1,70 m di franco alla
partenza, 0,45 m minimo lungo la corsa, 12,7 m liberi oltre l'arrivo). La lista intera non era mai
stata salvata.

Nello stesso posto c'erano anche gli insiemi grezzi dietro il report a 4 comandi (sessione
`b579ad0e`): `turn_left.json` 565 pose, `turn_right.json` 552, `mB_LEFT.json` 189 (le "189 pose
equivalenti" citate nel report), `mB_RIGHT.json` 241, `door_missions.json` 23 varchi
attraversabili con posa di partenza, `mD.json` 5, `doorways.json` 164, `openings2.json` 23 (11
rilevabili), `objects2.json` 93.

Tutto copiato in **`docs/analysis/simulation/2026-11-13_office-feasible-tests/`** su questo
branch, con la lista anche in forma leggibile (`door_runup_left_5m.txt`) e gli script che l'hanno
generata. Dettagli e come rigenerarla: il README di quella cartella.

**Sono 35, non 36.** Se i 36 sono un'altra cosa: l'unica altra lista di quell'ordine di grandezza
su questa macchina è `turn_left.json` filtrata — **26** pose con
`straight2m_after_chair_free: true`, le sole del mondo dove un "go straight 2 m" dopo una sedia
avrebbe spazio. Nessun file, blob git, sessione o tar su `work` contiene una lista di 36 voci:
qui sotto c'è dove ho guardato. Se il numero 36 è esatto, quel report è sull'Orin o su linus.

### Dove ho guardato

| posto | esito |
|---|---|
| WSL `/home/andreas/dev_ws/f1tenth_hil` (checkout buono, `main`, origin `sjckness/f1tenth_more`) | tutti i branch locali e remote-tracking, 79 tag, `git stash list` (vuoto), `git worktree list` (solo il principale), file non tracciati e ignorati. Nessun remote `orin/*` configurato qui. |
| `git log --all -i --grep` su office / feasib / fattib / start pose | i soli commit office sono `c72de207`, `6b38cea8`, `8842c1f1`, `c6e03af3`, `47ac7b8b` (tutti 2026-11-13) |
| `git grep -i -E 'feasib\|fattibil\|start_pose'` su ~600 revisioni (`sim/**`, `output/**`, `docs/analysis/**`, `office_hil_campaign/**`) | solo il report a 4 comandi e i report di campagna del 2026-09/10 |
| Windows `F:\Users\Andreas Steffens\dev_ws\f1tenth_hil` | **checkout rotto**: `.git` ha solo `objects/` e `modules/`, nessun `HEAD` né `refs/`; working tree ridotto a `src/f1tenth_external` e `src/f1tenth_hardware`. Gli oggetti però ci sono tutti → li ho esplorati da un repo temporaneo con `objects/info/alternates` (vedi sotto) |
| tutti gli oggetti di quel pack (287 MB, 1061 commit, 18 570 blob, 9 389 tree, 79 tag) | scansione del contenuto di ogni blob: nessun blob con una lista di ~36 verdetti/pose. Unica cosa non su `origin`: `c7c59cc0` "chore: allineamento work, spazio liberato" — ora è `origin/main` |
| `F:\Users\Andreas Steffens\work\isaac_hil` | svuotata (restano solo `CLAUDE.md` e `.claude`) |
| `~/f1tenth_salvataggi/work-20261119.tar.gz` (521 voci) | `hil.local.yaml`, `sim/isaac/usd/generated/office/`, `output/planner_compare/`, `scripts/sim/win/night/*.log` (15 log Isaac della notte, numerati 1-21). Nessun report office |
| `~/f1tenth_salvataggi/work-sim-archive-20261119.tar.gz` (2,5 GB, 589 voci) | solo bag/grafi/maschere delle 5 run Hydra office del 2026-11-12 |
| `~/f1tenth_salvataggi/work-report-20261119/` | `office_missions/` (README, report.txt, results.json, office_hil.html, map_geo.json — il report a 4 comandi), `office_hil_campaign/` (README + RUNBOOK), `office_hydra/`, `planner_compare/`, `hil/`, `hydra_runs/` |
| sessioni Claude Code Windows: `~/.claude/projects/*/*.jsonl` (7 progetti, 18 sessioni + sotto-agenti) | estratti tutti i prompt umani: l'unica sessione di analisi office è `b579ad0e` (2026-11-13), che ha prodotto il report a 4 comandi. Grep su `36 test`, `fattibil`, `feasib`, `start pose`, `posa di partenza`: nessun match utile |
| memoria Claude `~/.claude/projects/*/memory/`, `~/.claude/plans/`, `~/.claude/file-history/`, `tool-results/` | il piano `ottimo-lavoro-per-le-golden-snail.md` dice esplicitamente "**3 comandi, 3 ripetizioni, 9 run**" |
| sessioni Claude Code in WSL | `~/.claude/projects` non esiste: su WSL non è mai girato Claude Code |
| artifact pubblicati (`Artifact action list`, 14) | uno solo sull'office: **Missioni Ufficio HIL** <https://claude.ai/artifact/WoHRuJqTysCDB2ehdM8KhY> (2026-11-13) = il report a 4 comandi |
| `~/.copilot/chats`, log Copilot, `Desktop`, `Downloads`, `Documents`, `hil_local/claude_outputs`, `$RECYCLE.BIN` di F: | niente di pertinente |
| **scratchpad delle sessioni Claude**, `%LOCALAPPDATA%\Temp\claude\56e189eecf0f421f\` (315 file) | **qui era la roba**: `door_runup.json` (35) e la cartella `mapan/`. Nessun JSON con una lista fra 30 e 40 voci oltre a quello |
| altri cloni/worktree (`find` su `F:` e su `/` in WSL per `.git`) | solo `workspace/momoROV`, `workspace/underwater_ws` e i due checkout f1tenth già citati |
| **Orin** `fabiocar@192.168.200.11`, controllata il 2026-11-23 quando la macchina è tornata raggiungibile | confermato che **non c'è**: `~/dev_ws/f1tenth_hil/output/office_missions/results.json` è il report a **5 missioni** (`missions` → lista di 5, campi `feasible`/`id`/`obstacles`/`waypoints`), cioè lo stesso a 4 comandi; `office_hil_campaign/` contiene la campagna M01/M02/M03 della notte 13/11, non liste di pose. Grep su `fattibil`/`feasib`/`\b36\b` in `office_hil_campaign/`, `docs/`, `output/`: nessuna lista di 36 |

### Se servono più di 35 pose

Lo stesso strumento, mai eseguito sull'altro lato, dà un secondo insieme della stessa taglia —
e `--run` diverso ne dà altri. Senza simulatore, serve solo l'SDF + le mesh in `~/.gz/fuel`:

```bash
git checkout exp/2026-11-13_office-hil-campaign          # main NON ha sim/
python3 sim/isaac/tools/find_door_runup.py --run 5 --side right --json output/door_runup_right.json
python3 sim/isaac/tools/find_door_runup.py --run 3 --side left  --json output/door_runup_left_3m.json
```

### L'altro materiale office già in git (`origin/main`)

`docs/analysis/llm-pipeline/2026-11-13_office-missions/` — 5 catene di move percorse sulla griglia
di occupazione dell'SDF, senza simulatore (`sim/isaac/tools/office_missions.py`):

| id | comando | verdetto | posa di partenza |
|---|---|---|---|
| M-A | go straight, enter the door on the left and then approach the chair | FEASIBLE | (9.33, −15.87) yaw 90° |
| M-A2 | variante: la porta a DESTRA (cucina NE, 6 sedie) | FEASIBLE | (17.33, −6.37) yaw 90° |
| M-B | go straight at the wall, turn left and then approach the wall | FEASIBLE | (16.83, −9.87) yaw 180° |
| M-C | at the wall turn left, then go to the chair, then go straight 2 meters | 3 mosse su 4 | (16.83, −9.87) yaw 180° |
| M-D | at the wall turn left, then go to the person, then enter the door on the right | FEASIBLE, serve una persona | (16.83, −9.87) yaw 180° |

E `docs/analysis/_unsorted/2026-11-13_office-hil-campaign/` — il protocollo a 3 comandi × 3
ripetizioni (RUNBOOK completo, **nessun risultato**: della notte restano solo i 15 log Isaac nel
tar della work).

---

## B. Informazioni operative per il batch

### B.1 Avviare Isaac sul mondo office con una posa di partenza data

Isaac gira **nativo su Windows**, non in Docker (il renderer RTX non funziona in Docker
Desktop/WSL2). Ambiente: venv `F:\envs\isaac` con `isaacsim` da pip.
`scripts\sim\win\hil_win_env.bat` lo attiva e lo chiamano tutti gli altri `.bat`
(override con la variabile `ISAAC_ENV`).

**Comando esatto, sul link verso l'Orin:**

```bat
scripts\sim\win\hil_sim_link.bat --layout campaign_mb
```

che è `hil_win.py sim --net link --layout campaign_mb`, con stdout+stderr in
`scripts\sim\win\run.log` (il precedente diventa `run_prev.log`).

Varianti: `hil_sim_local.bat` (tutto sul PC), `hil_sim_wsl.bat` (Hydra nella distro WSL),
`hil_sim_gui.bat` (con GUI, costa ~4 % di RTF), `hil_world_convert.bat` / `hil_world_plan.bat`
(rigenerare l'USD dall'SDF).
Una volta sola, da amministratore: `hil_link_setup.bat add` e `hil_firewall.bat add`; poi
`hil_orin_check.bat` e `hil_preflight.bat`.

Opzioni di `hil_win.py sim`: `--world --layout --gui --keyboard --rate HZ --duration S
--set FILE.KEY=VALUE` (ripetibile) `--dry-run`, `--net local|link|wsl|wsltcp`,
`--discovery-server ADDR:PORT`.

**`hil.local.yaml` — l'unico file macchina-specifico** (`sim/isaac/config/hil.local.yaml`, in
`.gitignore`, prevale su `hil.yaml`). Valori reali della work, recuperati da
`work-20261119.tar.gz`:

```yaml
link:  {workstation_ip: 192.168.200.2, orin_ip: 192.168.200.11, prefix: 24,
        speed_mbit: 1000, socket_buffer_mb: 16, async_writer: false, flow_mbit: 800}
ros:   {domain_id: 44, rmw: rmw_fastrtps_cpp,
        discovery_server_host: orin, discovery_server_port: 11812}
sim:   {isaac_dir: sim/isaac, image: f1tenth_isaac:5.1.0, container: f1t_isaac_hil,
        world: office, layout: corridor, gui: false,
        stereo: {width: 640, height: 360, rate: 10.0, right_camera: false}, extra_set: []}
orin:  {ssh: fabiocar@192.168.200.11, repo: /home/fabiocar/dev_ws/f1tenth_hil,
        container: f1tenth_hil_car, image: f1tenth:jazzy-orin-ecb457b-norust,
        stack_args: '', models_dir: /home/fabiocar/projects/llm/models,
        require_host_network: true}
wsl:   {distro: Ubuntu-24.04}
```

Il file **non c'è più sul disco** (il checkout Windows è stato svuotato): va riestratto dal tar o
riscritto. `hil.yaml` versionato ha gli stessi campi ma con gli indirizzi `10.42.60.x`, che su
questa macchina **non** sono quelli usati.

**La posa di partenza non è un argomento di riga di comando.** Viene da `scene.car.start` del
preset in `sim/isaac/config/scenes/office.yaml` ed è **fissata al lancio di Isaac**.

| preset `--layout` | posa (x, y, z, yaw_deg) | note |
|---|---|---|
| `corridor` (default) | 16.814, 0.946, 0.02, −90° | 24,85 m liberi davanti, 1,25 m per lato |
| `door` | 10.064, −19.404, 0.02, 180° | 2 m davanti a un varco di 1,25 m, frontale |
| `door_left` | 16.464, −12.604, 0.02, 180° | rincorsa di 5 m, varco 1,25 m a (11.464, −14.604) esattamente 2 m a sinistra |
| `open_space` | 15.364, −11.554, 0.02, 0° | il cerchio libero più grande, 2,80 m in ogni direzione |
| `people` | = `corridor` | + 3 personaggi che camminano |
| `campaign_ma` | 9.33, −15.87, 0.02, 90° | M01 porta a sx + sedia; +3 sphere light a 30000 |
| `campaign_ma_bright` | idem | le stesse 3 luci a 60000 (fallback del RUNBOOK se YOLO non vede la sedia) |
| `campaign_mb` | 16.83, −9.87, 0.02, 180° | M02 muro → sx → muro; **nessuna percezione**, è lo smoke case |
| `campaign_md` | 16.83, −9.87, 0.02, 180° | M03 + persona statica a (11.63, −13.30) + luce sul varco |

**Posa arbitraria senza aggiungere un preset:**

```bat
scripts\sim\win\hil_sim_link.bat --layout campaign_mb ^
   --set scene.car.start.x=9.33 --set scene.car.start.y=-15.87 ^
   --set scene.car.start.yaw_deg=90.0 --set scene.car.start.z=0.02
```

L'ordine in `sim/isaac/sim/config.py:load_config` è: 1) `scene.layout`, 2) merge del preset sui
default della scena, 3) **tutti gli altri `--set` sopra** — quindi un override di `car.start`
vince sul preset. In alternativa `sim.extra_set` in `hil.local.yaml`.

⚠️ Due trappole, entrambe documentate nel repo:
- `_merge` fa deep-merge delle mappe ma **sostituisce le liste per intero**: un preset (o un
  `--set`) che elenca solo le proprie `sphere_lights` spegne in silenzio le altre. Per questo ogni
  preset campagna ripete tutte e 7 le luci base.
- La luce, non la geometria, è il rischio dell'office: headless `RaytracedLighting` non ha diffusa
  indiretta e tre waypoint di M01 stanno a 6,2-6,4 m dalla luce più vicina. Prima di bruciare dei
  run: scena su, nessuna missione, e si guarda `/camera/detections` e `/costmap/semantic_tracks`
  dalla posa di partenza.

### B.2 Riposizionare l'auto tra un run e l'altro, senza riavviare Isaac

`scripts/sim/restart.sh` (gira nell'ambiente ROS dello stack, via `sim_isaac.sh run`):

```bash
scripts/sim/restart.sh                 # /reset_all, settle 5 s, i 5 check
scripts/sim/restart.sh --sim-only      # solo /sim/reset di Isaac (stack non attivo)
scripts/sim/restart.sh --settle 8 --net local
# altre opzioni: --start X,Y,YAW_DEG (solo per la verifica), --domain N
```

Il default è il reset completo dello stack, **`/reset_all`** (`std_srvs/Trigger`, nodo
`reset_manager`): stop + abort della missione → `/sim/reset` (auto **e** persona alle pose di
start, servizio `wheel_odom.method` dell'overlay del sim) → `/set_pose` dell'EKF locale → restart
dello SLAM → `/ekf_global/set_pose` → hook. Poi `--settle` secondi e cinque check, una riga
OK/FAIL ciascuno (exit 1 se uno fallisce):

| check | tolleranza |
|---|---|
| pose | `/sim/ground_truth/odom` entro 2 cm / 1° dalla start pose (latched `/sim/ground_truth/start_pose`) |
| speed | \|v\| ≤ 0,05 m/s (= `stop.standstill_speed_mps`) |
| mission | `/mission/status` non LOADED/RUNNING/HOLDING, nessun e-stop latched (dopo un abort legge ABORTED: è lo stato idle) |
| mpc | ogni campione di `/drive` su 1 s a speed 0. L'`/mpc/hold` ingaggiato dall'abort **non** è un errore: lo rilascia il prossimo `/mission/start_mission` |
| localization | `/odometry/filtered` entro 5 cm / 0,05 rad da 0 0 0; `/ekf_global/odometry/filtered` e `/slam/pose` entro 15 cm |

Nel driver: `hil_campaign.sh` → `reset_car()` = "/reset_all e i cinque check"; se un check
fallisce **la ripetizione non parte**, ed è quello il risultato da annotare.
Dal lato Windows c'è anche `scripts\sim\win\hil_reset.bat` → `hil_win.py reset`, che chiama solo
`/sim/reset`.

**Limite da tenere presente per il batch:** `/sim/reset` riporta l'auto **sempre** a
`scene car.start`, cioè alla posa del preset con cui Isaac è partito. Non esiste un servizio per
spostarla a caldo in una posa diversa. Quindi **una posa di partenza = una sessione Isaac**: un
batch su N pose richiede N lanci (o N preset). È esattamente il problema per cui esistono
`scripts/sim/campaign/hil_night.sh` + `scripts\sim\win\hil_follow.bat`.

### B.3 Come si pilota l'Orin da qui

Lo schema già scritto nel repo è **l'Orin che comanda e Windows che segue** (non il contrario):

- Sull'Orin, `scripts/sim/campaign/hil_night.sh start [--reps N]` si stacca dal terminale e
  gestisce tutta la notte: Isaac `campaign_mb` → stack → gate → recorder → preflight → translate →
  M02 ×1 + chain check → M02 ×(N−1) → boundary a `campaign_ma` (+ go/no-go sedia, fallback
  `campaign_ma_bright`) → M01 ×N → boundary a `campaign_md` → M03 ×N → analyze → tutto giù →
  `MORNING.md`. Un boundary è: stack giù dall'interno del container, 8 s di lease Fast DDS, Isaac
  riavviato sul nuovo layout, stack su, llama-server healthy, un controller nel grafo.
- Sulla workstation, `scripts\sim\win\hil_follow.bat` resta aperto: ogni 3 s legge via **ssh** una
  mailbox sull'Orin (`~/.local/state/f1tenth_hil/isaac/`: `request` = `"SEQ run LAYOUT"` o
  `"SEQ stop"`, `state` = key=value + coda di run.log) e (ri)avvia Isaac con quel layout, come
  farebbe `hil_sim_link.bat`. Connette **solo in uscita**: nessuna porta in ascolto e nessuna
  regola di firewall sul PC. Riavvia un Isaac morto fino a 2 volte e lo segnala; si rifiuta di
  partire se c'è già un `sim/main.py` attivo; tiene Windows sveglio. Un log per lancio in
  `scripts\sim\win\night\<seq>_<layout>.log`.
- Driver manuale, sull'Orin: `scripts/sim/campaign/hil_campaign.sh preflight | translate |
  all --reps 3 | recorder | ready | see`, e `scripts/sim/hil_orin up|stack|shell|check`.

Per **lanciare da Windows** basta invertire il punto di ingresso: `ssh <orin> 'cd ~/dev_ws/... &&
scripts/sim/campaign/hil_night.sh start --reps N'` e poi tenere aperto `hil_follow.bat`. Il
protocollo mailbox non va toccato.

### B.4 SSH verso l'Orin: funziona, dal lato Windows, sul cavo diretto

**Verificato il 2026-11-23 alle 12:24, con l'Orin accesa e collegata:**

| prova | esito |
|---|---|
| `ping 192.168.200.11` | **0 % perso**, RTT < 1 ms |
| `ssh -o BatchMode=yes fabiocar@192.168.200.11` | **OK**, chiave, nessuna password → `jetson-orin`, `fabiocar` |
| `ssh orin` (alias → 100.96.201.7, Tailscale) | ancora `timed out`: usare **l'IP del cavo**, non l'alias |
| `ping 100.96.201.7` | 100 % perso — su questo PC Tailscale gira in server mode sotto un altro utente Windows (`DESKTOP-Q3F81S6\Francesco Niosi`) e questo account non lo può interrogare; l'Orin invece ha `tailscale0` su |

- **Windows** (`F:\Users\Andreas Steffens\.ssh\`): `config` con `Host orin` → `HostName
  100.96.201.7` / `User fabiocar` e `Host thor` → `192.168.200.10` / `andre`; chiave
  `id_ed25519` (2026-11-13); `known_hosts` con `192.168.200.10`, `192.168.200.11`, `github.com`.
  ⚠️ L'alias `orin` punta a Tailscale: finché resta così, va usato `fabiocar@192.168.200.11`
  (oppure cambiato `HostName`, a scelta tua — non l'ho toccato).
- **WSL Ubuntu-24.04**: `~/.ssh` **non esiste** — nessuna config, nessuna chiave: da WSL l'Orin non
  si raggiunge. (WSL serve solo a Hydra.) Nota: in WSL la distro di default è `docker-desktop`,
  quindi serve sempre `wsl -d Ubuntu-24.04`.
- Interfacce dell'Orin: `eno1` ha **due** indirizzi, `192.168.200.11/24` (il cavo verso la
  workstation) e `192.168.0.15/24`; wifi `wlP1p1s0` 172.26.13.3/23; `tailscale0` 100.96.201.7.
- Lato workstation: **192.168.200.2** su "Ethernet 2" (Intel X710 10GBASE-T). La `.10` era della
  Thor, che l'Orin ha sostituito su quel cavo.

Nessuna modifica a rete, firewall o route è stata fatta; lo snapshot di partenza resta in
`workspace\hil_local\rete_originale.txt`.

### B.5 Stato dell'Orin adesso (verificato)

| cosa | valore |
|---|---|
| host | `jetson-orin`, kernel **5.15.185-tegra**, JetPack **R36 rev 5.0** (16/01/2026), aarch64 |
| uptime | 23 min al momento del controllo — **riaccesa oggi** |
| disco | `/dev/mmcblk0p1` 57 G, **41 G usati, 14 G liberi (75 %)** — ⚠️ pochi per una notte di bag |
| container attivi | **nessuno** (`docker ps` vuoto): lo stack è giù |
| immagini | ricostruite **oggi 2026-11-23 11:35**: `f1tenth:jazzy-orin-ef41147-norust` e `f1tenth:campaign3-rrt-ef41147`. ⚠️ `hil.local.yaml` indica ancora `f1tenth:jazzy-orin-ecb457b-norust`: va aggiornato o il preflight cerca un'immagine vecchia |
| checkout | `/home/fabiocar/dev_ws/f1tenth_hil`, branch **`campaign-office`** = `bf71c464` (2026-11-13), con modifiche locali (`office_hil_campaign/analysis/run_selection.yaml`, due file del logger) e tutta la campagna della notte **non committata** |
| ha `sim/` | **sì** (`sim/isaac/config/`), e `sim/isaac/config/hil.local.yaml` **esiste** (4673 B, 11/11) — è quello perso su Windows: si può ricopiare da qui |
| manca | `sim/isaac/usd/generated/office/` — ma l'USD serve solo lato Windows |
| script HIL | tutti presenti: `scripts/sim/campaign/{hil_campaign.sh,hil_night.sh,hil_overlay.sh}`, `scripts/sim/{restart.sh,hil_orin,hil_check,sim_isaac_hil,hil_config.py}` |
| mailbox | `~/.local/state/f1tenth_hil/isaac/` c'è e è **ferma e pulita**: `state` con `seq=22 phase=stopped rtf=1.000 note="serving: 22 stop" host=DESKTOP-Q3F81S6`, `request` = `22 stop`. Il prossimo giro parte da **seq 23** |
| `origin/*` sull'Orin | fetch vecchio: `origin/main` = `74c08897` (25/10). Il vero `origin/main` oggi è `c7c59cc0` |
| campagna già girata | `office_hil_campaign/` con M01/M02/M03 della notte 13/11 e `MORNING.md`: M01 **FAIL** (0/3 run), M02 3 run ma tutti `ABORTED` e fuori tolleranza (1,2-1,4 m su ≤ 0,4), M03 3 run `ABORTED` a 2,8-3,0 m su ≤ 0,3. Nessun successo registrato: è materiale da rigiudicare, non un risultato |

### B.6 Versioni

| cosa | valore |
|---|---|
| commit che faceva girare Isaac su work | **`47ac7b8b`** — `office_hil_campaign: an unattended night`, branch `planner-modes-hil` della work, 2026-11-13 17:05 |
| dove sta quel commit | **non è su `origin/main`**: tag `archive/work-planner-modes-hil-20261119` (tag `288a7ea3` → `47ac7b8b`) e `exp/2026-11-13_office-hil-campaign`. Altri tag utili: `exp/2026-11-13_office-missions` (`6b38cea8`), `exp/2026-11-13_hil-windows-wsl` (`8842c1f1`), `exp/2026-11-12_office-hydra-run` |
| ⚠️ `main` | `c7c59cc0` (2026-11-19) **non contiene** `sim/`, `office_hil_campaign/` né `third_party/`. Per rilanciare Isaac serve `git checkout exp/2026-11-13_office-hil-campaign` |
| Isaac Sim | **5.1.0.0** (pip: `isaacsim`, `isaacsim-kernel`, `isaacsim-extscache-*`) in `F:\envs\isaac`; `OMNI_KIT_ACCEPT_EULA=YES` |
| GPU | **NVIDIA GeForce RTX 4090**, 24 564 MiB, driver **560.94** |
| OS workstation | Windows 10 Pro 22H2 (10.0.19045), DESKTOP-Q3F81S6 |
| WSL | Ubuntu-24.04 + docker-desktop (quest'ultima è la **default**), solo per Hydra |
| LLM planner | `qwen2.5-3b-instruct-q5_k_m.gguf`, modelli in `/home/fabiocar/projects/llm/models`, llama-server health su `http://127.0.0.1:8083/health` |
| mondo | `src/f1tenth_sim/worlds/husarion_office_2x.sdf` (2× di `husarion_gz_worlds`, submodule a `87271ef`) → `sim/isaac/usd/generated/office/office.usd` via `tools/sdf_to_usd.py`. 99 modelli, 210 k triangoli, 28,5 × 27,1 × 2,6 m, 573 m² di pavimento guidabile al piano del lidar |
| checkout WSL | `/home/andreas/dev_ws/f1tenth_hil`, `main`, origin `https://github.com/sjckness/f1tenth_more` |
| checkout Windows | `F:\Users\Andreas Steffens\dev_ws\f1tenth_hil` — **inutilizzabile**: `.git` senza `HEAD`/`refs` (solo `objects/` 275 MB e `modules/` 62 MB), working tree ridotto a due pacchetti `src/`. Gli oggetti sono integri: `git init --bare` + `objects/info/alternates` li legge tutti |

### B.7 Cosa serve prima di poter lanciare qualcosa

Il **lato Orin è pronto** (repo con `sim/`, `hil.local.yaml`, tutti gli script, mailbox ferma a
seq 22, immagini di oggi). Quello che manca è tutto sulla **workstation**:

1. ~~Orin accesa e raggiungibile~~ — **fatto**, `ssh fabiocar@192.168.200.11` risponde (B.4).
2. **Un checkout Windows funzionante con `sim/`**: quello attuale è rotto. Da ricreare, p.es.
   `git clone` + `git checkout exp/2026-11-13_office-hil-campaign`, oppure riestrarre da
   `work-20261119.tar.gz`.
3. **`sim/isaac/config/hil.local.yaml`** lato Windows: perso. Si copia dall'Orin
   (`scp fabiocar@192.168.200.11:~/dev_ws/f1tenth_hil/sim/isaac/config/hil.local.yaml .`) e va
   corretto il tag immagine da `ecb457b` a **`ef41147`**.
4. **L'USD dell'office** rigenerato (`tools/sdf_to_usd.py`, serve la cache Fuel in `~/.gz/fuel`)
   o riestratto da `work-20261119.tar.gz` (`sim/isaac/usd/generated/office/`).
5. Sull'Orin: allineare il branch (`campaign-office` ha lavoro non committato — da salvare prima
   di qualsiasi checkout) e bind-mountare repo e archivio bag alla creazione del container;
   `hil_campaign.sh preflight` dice cosa manca. Attenzione ai **14 G liberi**.
6. La lista delle pose: c'è, `docs/analysis/simulation/2026-11-13_office-feasible-tests/` (35).
   Un preset per posa in `office.yaml`, oppure i tre `--set scene.car.start.*` per lancio.
