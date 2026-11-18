# PWA "F1Tenth Lab Control": ricognizione (Fase 0)

2026-10-20. Sola lettura: nessun codice scritto, nulla avviato. Fonti:
- `campaign-final` (cd180d8, worktree `~/wt/cf`);
- `telegram-control` (b131b9b);
- l'Orin via ssh, solo comandi di lettura. Verso fine ricognizione l'Orin non
  rispondeva più (timeout ssh): le porte in ascolto restano da verificare.

Riferimenti `file:riga` relativi a `src/` di `campaign-final`, salvo dove
indicato.

---

## 0. Prima di tutto: cinque cose che cambiano il piano

1. **L'Orin non gira su `campaign-final`, e `campaign-final` non contiene
   telegram-control.**
   - L'Orin è sul branch `mpc_corr_v2_reverse` (4f53ab8, già su origin).
     Contiene `f1tenth_ctl`/`f1tenth_telegram` copiati da telegram-control
     (1c925a0, 32790ac, 4f53ab8; l'albero di `src/f1tenth_ctl` è identico a
     telegram-control).
   - `campaign-final` (locale, origin e il ref dell'Orin, tutti cd180d8) non
     ha nessuno dei commit di telegram-control.
   - Rispetto a `campaign-final`, l'Orin ha 21 commit in più (retromarcia
     MPC, memoria semantica, countdown solo col logger) e 21 in meno
     (`hard_stop_node`, cioè lo stop LiDAR di ultima istanza, e le porte).
   - **Proposta (§5, passo 0):** `pwa-control` = `campaign-final` +
     fast-forward a `telegram-control`. telegram-control parte esattamente da
     cd180d8, quindi il risultato è ciò che la SPEC dà per scontato.
   - Il gateway aggiunge solo cartelle nuove, più un piccolo refactor in
     `f1tenth_ctl`, quindi si porta sul branch dell'Orin con un cherry-pick.
   - **Domanda:** su quale branch girerà la macchina per le prove in lab?
     Cambia una cosa per la sicurezza: lo stop LiDAR di ultima istanza esiste
     solo su `campaign-final`.

2. **Le missioni non hanno giri, raceline né waypoint con nome.**
   - "2 giri lab_loop → box" dei mockup non esiste: nessun lap, raceline o
     loop nello schema, nel planner o nel BT.
   - Una missione è una lista di *move*:
     - `drive` straight / wall_turn;
     - `go_to_object`;
     - `turn`, `goal_distance`, `goal_pose`.

     Ogni move ha la sua `stop_condition`, `timeout_sec` e `on_timeout`.
   - Card PIANO e "Missione in corso" vanno adattate: §1.4 e §3.3.

3. **`/mission/status` non dà avanzamento.**
   - Ha solo `state`, `json_path` ed `emergency_stop_active`.
   - Il passo attivo si ricava da `/mission/move_outcome` (un messaggio per
     move *finito*); il tempo lo cronometra il gateway dal fronte RUNNING.
   - Il countdown non si vede: per 15 s lo stato resta `LOADED`, e lo dice
     solo la risposta di `start_mission` ("starting in 15.0 s").

4. **HTTPS su Tailscale non è ancora attivo.**
   - L'Orin ha Tailscale 1.102.2 (`jetson-orin.tail5ddb73.ts.net`), ma
     `CertDomains` è vuoto e `tailscale serve` non ha configurazione.
   - Servono due azioni tue, una volta sola:
     - abilitare **HTTPS Certificates** nella console admin di Tailscale
       (pagina DNS);
     - `sudo tailscale set --operator=fabiocar`.

5. **Sull'Orin mancano FastAPI/uvicorn e Node.**
   - Ci sono `aiohttp` 3.14.3, `pydantic` 2.13.4, `ffmpeg`, `jtop`, Python
     3.10.12, pip con accesso a internet.
   - Mancano `fastapi`, `uvicorn`, `faster-whisper`, `node`/`npm`.
   - Conseguenze: un venv per il gateway; il frontend si compila su linus
     (Node 20.20.2).

---

## 1. Dove sono e come si usano i pezzi esistenti

### 1.1 `f1tenth_ctl` (telegram-control; identico sul branch dell'Orin)

- **Codice:** `src/f1tenth_ctl/`. CLI `bin/f1tenth_ctl <comando>`: una riga
  JSON su stdout.
- **Env:** `config/ctl.env`, copia del blocco ROS di `~/.bashrc`
  dell'Orin:
  - `ROS_DISCOVERY_SERVER=127.0.0.1:11811`;
  - `ROS_SUPER_CLIENT=TRUE`;
  - `LD_LIBRARY_PATH` CUDA.
- **Processi:** unità systemd utente transitorie:
  - `f1t-stack`: `ros2 launch f1tenth_bringup supervisor_bringup.launch.py`;
  - `f1t-planner`: `ros2 run llm llm_planner_node --dry-run`, stdin da FIFO;
  - `f1t-logger`: `ros2 launch f1tenth_logger mission_logger.launch.py enable_mission_logger:=true`;
  - più `f1t-discovery` e `f1t-monitor`.
- **Stop:** SIGINT al gruppo → 10 s → SIGTERM → 5 s → SIGKILL.
- **Stato all'ultima lettura:** attive discovery, monitor, planner, stack.
  Linger=yes, quindi le unità utente sopravvivono a logout e riavvio.
- **Comandi:**

  | Comando | Effetto |
  |---|---|
  | `connect` | discovery su + controllo dell'env |
  | `start\|stop stack\|planner\|logger` | avvio / arresto di un processo |
  | `status` | processi + READY, < 5 s |
  | `estop` | zero su `/safety_stop` 2 s a 50 Hz + `/mission/emergency_stop` + `/mission/abort_mission`, con `car_held` |
  | `estop-reset` | abort + zero tenuto + riavvio di `behavior` via `/restart_component`; verifica che il latch sia libero |
  | `send planner "<testo>"` | una riga sullo stdin del planner |
  | `planner-log --since N` | righe del planner dopo il byte N; `results[]` con la missione salvata; `plan_only` |
  | `missions`, `mission-load <id>`, `mission-start --id <id>`, `mission-status` | piani salvati, caricamento, avvio, stato |
  | `killall`, `check`, `logs` | pulizia, controllo env, log |

- **API Python:** ogni comando è `f1tenth_ctl.cli.cmd_<nome>(settings, args)`
  e restituisce lo stesso dict del JSON (cli.py:128-737). `args` è un
  `argparse.Namespace`, che il gateway costruisce da sé.
- **Gestione dei piani:**
  - lo stato sta in `~/.local/state/f1tenth_ctl/` (`missions/`,
    `current.json`), condiviso tra bot e gateway;
  - `mission-start` rifiuta se non READY, e-stop latched, altra missione
    attiva, piano più vecchio di 5 min, o id diverso da quello caricato.

### 1.2 READY / check

- **Soglie:** `src/f1tenth_ctl/config/ready.yaml`.
- **Giudizio:** puro, in `f1tenth_ctl/ready.py`.
- **Misure:** l'unità `f1t-monitor` (rosprobe `Meter`) scrive uno snapshot;
  `cmd_status` (cli.py:260) lo legge.
- **Check reali** (6 obbligatori + 1 informativo):

  | Nome | Cosa | Soglia |
  |---|---|---|
  | `odometry` | `/odometry/filtered` (EKF locale) | ≥ 10 Hz |
  | `scan` | `/scan` | ≥ 20 Hz |
  | `tf_map_base_link` | TF map→base_link | età ≤ 0.5 s |
  | `mission_service` | `/mission/load_mission` disponibile | – |
  | `estop_latched` | `/mission/status.emergency_stop_active` | deve essere false |
  | `battery` | `/diagnostics/battery_status.voltage` | ≥ 10.8 V |
  | `wheel_odom` (non obbligatorio) | `/odom` | solo mostrato |

- **Uscita di `status`:**
  - `ready`, `failing[]`, `estop_latched`, `mission`, `processes{}` (stato,
    pid, `managed`) e `probe.nodes` (numero di nodi);
  - `checks[]` con `{name, ok, value, want, why, required}`.
- La riga del mockup "Localizzazione, cov. xy 0.42 m > soglia 0.15 m" non
  esiste. La UI mostra i check reali, con etichetta italiana e
  `value` vs `want`.
- `n/m CHECK OK` = obbligatori superati / obbligatori.

### 1.3 Trascrittore vocale del bot

- **Codice:** `src/f1tenth_telegram/f1tenth_telegram/transcribe.py`,
  `Transcriber`. Configurazione: dataclass `Transcription` in `config.py`.
- **Comportamento:**
  - faster-whisper caricato in background;
  - CUDA float16 se funziona, altrimenti CPU int8;
  - stati `disabled/loading/ready/unavailable`;
  - `transcribe_bytes(write, suffix)`: directory temporanea sempre
    cancellata, fallback ffmpeg→WAV, limite `max_duration_s`.
- Importa solo stdlib e la propria config, non Telegram: si riusa con un
  import.
- **Oggi gira su linus** (RTX 5050, modello medium, ~0.3 s a comando).
- **Sull'Orin:**
  - faster-whisper non è installato;
  - le wheel PyPI di CTranslate2 per aarch64 sono **solo CPU**; CUDA su
    Jetson richiede una build da sorgente;
  - `av` di sistema è 17.1.0, mentre faster-whisper 1.2.1 vuole `av<17`:
    va fissato nel venv.

  Proposta in §4.

### 1.4 Planner LLM: piano senza avvio

- `llm_planner_node` è una **CLI, non un servizio**: nessun
  servizio/azione/subscription. Il nodo è `llm_planner_node`.
- **`--dry-run`** esiste su entrambi i branch:
  - traduce, scrive il file (`share/f1tenth_behavior/missions/llm_generated/<mission_id>.json`)
    e non chiama nessun servizio;
  - senza argomenti legge comandi da stdin (`comando> `).
- **Riga RESULT:** l'ultima riga per comando è `RESULT {json}`:
  - `status`: `ok | llm_error | translator_error | refused | node_error`;
  - `stage`, `error_type`, `error_message`;
  - su `ok` anche `mission_id`, `n_moves`, `plan_seconds`, `unsupported`,
    `notes`, `mission` (il JSON completo) e `mission_path`.
- **f1tenth_ctl fa già tutto il resto:**
  - planner in `--dry-run` con FIFO;
  - `send` + `planner-log` (che salva ogni missione `ok` sotto un id
    `YYYYmmddTHHMMSS-sha6` e ne fa il riassunto con `missions.summarize`);
  - interblocco `plan_only`, letto dall'argv del processo;
  - `mission-load` / `mission-start`.
- **Dipendenza:** llama-server (componente `intelligence`, porta 8083,
  qwen2.5-3b, `-ngl 999`). Senza, il planner esce dopo 90 s.
- **Riassunto disponibile** (`missions.summarize`):
  - `n_moves`, `moves[{i, action, speed_mps, until, timeout_s, on_timeout, terminal}]`;
  - `max_speed_mps`, `distance_m` (null se aperta), `max_duration_s`;
  - `targets`, `reverse`.

  Al posto di "giri / v max / arrivo" della card: **move · v max ·
  lunghezza (o "fino all'oggetto") · durata massima**.

### 1.5 Snapshot del behavior tree

- **Albero:** `py_trees_ros.trees.BehaviourTree` in
  `/behavior_executor_node` (behavior_executor_node.py:803), tick a 10 Hz.
- **Lo stream di default è spento**: `default_snapshot_stream=false`, e il
  repo non lo cambia.
- **Senza toccare parametri:** il servizio
  `/behavior_executor_node/snapshot_streams/open`
  (`py_trees_ros_interfaces/srv/OpenSnapshotStream`: `topic_name`,
  `parameters{snapshot_period, blackboard_data, blackboard_activity}`)
  apre uno stream dedicato al client e restituisce il topic.
- **Messaggio:** `py_trees_ros_interfaces/msg/BehaviourTree`. Ogni
  `Behaviour` ha `name`, `class_name`, `own_id`, `parent_id`, `child_ids`,
  `type` (SEQUENCE/SELECTOR/…), `status` (INVALID/RUNNING/SUCCESS/FAILURE)
  e `is_active`.
- **Mappatura UI:** SUCCESS → verde, RUNNING → bianco, INVALID → idle
  grigio tratteggiato, FAILURE → rosso.
- **Struttura:** `root` Selector →
  - `emergency`: IsBatteryLow | IsEmergencyStopTriggered |
    IsProximityTooClose | IsSystemOverheated, poi `Stop`;
  - `handle_obstacle`: spento di default;
  - `mission`: MissionActive → oggetto | PublishMoveGoal, GoToObject,
    CheckStopCondition, AdvanceMove;
  - `navigation`: HasMpcGoal.
- **Altri topic del BT:**
  - `/behavior/tree_status` (`BehaviorTreeStatus`: `active_lane`,
    `lane_names[]`, `lane_statuses[]`, `emergency_trip`,
    `safety_stop_active`, `stop_source`) a 10 Hz;
  - `/bt/tree_visualization` (PNG graphviz colorato, ≤ 2.5 Hz): utile come
    fallback.
- `py_trees_ros_interfaces` è disponibile solo come dipendenza transitiva di
  py_trees_ros. Sull'Orin va verificato `/opt/ros/humble/share/py_trees_ros_interfaces`
  (non fatto: Orin irraggiungibile).

### 1.6 ackermann_mux: lane teleop e lane di sicurezza

**Configurazione:** `f1tenth_bringup/config/mux.yaml`, identico sui due
branch, nessun `locks:`.

| Lane | Topic | Priorità | Timeout |
|---|---|---|---|
| safety_stop | `/safety_stop` | **200** | 0.2 s |
| joystick | **`/teleop`** | **100** | 0.2 s |
| calibration | `/calibration_drive` | 50 | 0.2 s |
| navigation | `/drive` | 10 | 0.2 s |

- **Uscita:** `/ackermann_drive` → `ackermann_to_vesc_node`.
  `header.frame_id` dice la sorgente: `''` = MPC, `base_link/emergency`,
  `base_link/hard_stop`, `base_link/obstacle`, `base_link`.
- **Su `/safety_stop`** (vince su tutto, joystick compreso):
  - lo `Stop` del BT, che copre e-stop latched, batteria bassa,
    surriscaldamento e prossimità 0.15 m;
  - l'event-stop di IsProximityTooClose (~40 Hz);
  - **`hard_stop_node`**, ritorno LiDAR entro 0.20 m (solo `campaign-final`,
    nel componente `control`);
  - `reset_manager` durante `/reset_all`.
- **Su `/teleop` oggi non pubblica nessuno:**
  - `joy.launch.py` si lancia a mano. Se gira, il profilo `default` manda
    zeri a 20 Hz su `/teleop` senza dead-man e occupa la lane.
  - `stack_startup_sequence` (sterzo avanti-indietro all'avvio) solo sul
    percorso `stack_bringup`, non col supervisor.
- **Il mux non inietta zeri:** quando tutte le lane scadono tace, e nel repo
  non è configurato nessun timeout VESC. In pratica l'MPC pubblica zeri su
  `/drive` a 10 Hz quando è fermo o in hold. Il gateway non ci conta:
  manda zeri espliciti (§3.4).

### 1.7 Topic per la telemetria

| Dato | Topic / fonte | Note |
|---|---|---|
| Posa (mappa) | `/ekf_global/odometry/filtered` (frame `map`, 20 Hz) | x, y, ψ; niente sottoscrizione a `/tf` |
| Posa (odom) | `/odometry/filtered` (frame `odom`, 20 Hz) | serve per map←odom (sotto) |
| Velocità misurata | `/odometry/filtered` twist.linear.x | stessa che usa l'MPC; `/odom` (VESC) come riserva |
| Velocità comandata | `/ackermann_drive` (uscita del mux) | ciò che riceve davvero il VESC; `frame_id` = sorgente |
| Batteria | `/diagnostics/battery_status` (`BatteryStatus`: has_data, voltage, min_voltage, ok; 2 Hz) | **nessuna percentuale né numero di celle nel repo** |
| Temperatura VESC | `/sensors/core` `ntc_temp_mos1/2/3` | `temp_fet` e `temp_motor` restano 0: il driver non li scrive |
| CPU | `/diagnostics/system_status` (`SystemStatus`: cpu_percent, cpu_temp_c, gpu…; 1 Hz) | solo con lo stack acceso |
| Mappa | `/slam/map` (OccupancyGrid 0.05 m, ogni ~5 s) | `/map` è una mappa statica `square_100m`, non SLAM |
| "Raceline" | **non esiste** | il riferimento è l'orizzonte MPC: `/mpc/solver_status` `pred_x/pred_y` (frame `odom`) |
| Errore laterale | **non esiste** | vicini: `/mpc/wall_track`, `/mpc/min_obstacle_distance_forward` |
| Avanzamento missione | `/mission/move_outcome` (un `MoveOutcome` per move finito) | più `/mission/status` |

- **map←odom** si ricava dalle due EKF, `T_map_base · T_odom_base⁻¹`, per
  disegnare l'orizzonte MPC sulla mappa senza `/tf`.
- **Batteria:** 10.8 V di minimo suggerisce una LiPo 3S, ma è una mia
  deduzione.

### 1.8 Archivio delle run, report e video

- **Logger:** `mission_logger_node` (`f1tenth_logger`), registratore
  rosbag2 in-process.
  - Parte su `LOADED`, si ferma su uno stato terminale.
  - Opt-in: `enable_mission_logger` è false di default dal 2026-10-16.
- **Path reale:** `~/f1tenth_archive`, da `mission_logger_runs_dir` in
  `stack_params.yaml:2332`; nessuna variabile d'ambiente.
  - Struttura: `active/` (in registrazione), `complete/` (finalizzate) e
    `incomplete/` (bag senza metadata).
  - Sull'Orin: complete 248, active 20 (mai finalizzate), incomplete 1.
  - Ci sono anche `mission_row/` e `verify/`, fatte a mano.
  - `~/f1tenth_data/runs` non è usato dal logger (una sola cartella, vuota).
- **Cartella di una run:** `<YYYY-MM-DDTHH-MM-SS UTC>_mission-<mission_id>/`
  - `<run_id>.manifest.json`;
  - `<run_id>.params.yaml`;
  - `bag/`;
  - `<run_id>.extract.parquet`;
  - `snapshots/<run_id>.NNN.jpg` (fino a 10).
- **Manifest:**
  - `run_id`, `mission_id`, `mission_json_path`;
  - `start_time`, `end_time`, `mission_start_time`, `preroll_s`;
  - `outcome`, `git{branch,commit,dirty}`, `bag_bytes`, `resolved_params`.
- **Valori di `outcome`:** `COMPLETE`, `ABORTED`, `EMERGENCY_STOP`,
  `INTERRUPTED`, `NOT_STARTED`, `LOADED`, `IDLE`, null.
- **Report:** nessun report HTML/PDF per run esiste o viene generato.
  - In automatico c'è solo il JSON di scoring del BT:
    `<ws>/src/f1tenth_behavior/mission_reports/<mission_id>_<stamp>.json`.
  - I report di campagna (`first_test_campaign/analysis/…`, `output/*.md`)
    sono fatti a mano.
- **Video:** solo a mano.
  - `mission_render`: parquet→mp4, senza ROS, in
    `<ws>/mission_videos/<run_id>.mp4`.
  - `tools/test_campaign_video.py` per le cartelle di test-campaign.
  - Nell'archivio dell'Orin c'è un solo mp4, del 2026-09-02.
- **Run manuali:** la guida manuale non viene registrata, perché il logger
  parte solo con una missione. La riga "Guida manuale" del mockup Run non ha
  fonte.
- **Solo sul branch dell'Orin** (a14ec2f): il countdown di 15 s c'è solo se
  lo stack ha `enable_mission_logger:=true`. Senza logger,
  **`start_mission` parte subito**.

---

## 2. Architettura: confermata, con sette precisazioni

```
iPhone (PWA, Safari) ──HTTPS/WSS──▶ tailscale serve (Orin, cert *.ts.net)
                                       │ http://127.0.0.1:8770
                                       ▼
                     f1tenth-gateway.service (unità systemd utente, fuori dallo stack)
                     ├─ FastAPI + uvicorn: REST, 1 WebSocket, file statici della PWA
                     ├─ f1tenth_ctl importato in-process: status/READY, processi,
                     │  planner, missioni, estop-reset (thread pool)
                     ├─ rclpy (Humble), un nodo `pwa_gateway` in un thread:
                     │  telemetria, /teleop, e-stop con publisher già "matchati", abort
                     └─ Transcriber di f1tenth_telegram (import)
```

1. **Bind solo su 127.0.0.1**, non sull'IP Tailscale.
   - `tailscale serve --bg --https=443 http://127.0.0.1:8770` termina il TLS
     col certificato `*.ts.net` ed espone il servizio **solo nella tailnet**.
   - L'IP Tailscale dato direttamente non avrebbe TLS valido, e Safari
     richiede un contesto sicuro per microfono e service worker.
   - La porta **non è 8765**: è di foxglove_bridge. 8770 è da verificare con
     `ss -ltn`.
2. **Ambiente ROS = `ctl.env`.** Il gateway vede il grafo come lo vede
   `f1tenth_ctl` (Discovery Server, super client) e funziona anche quando
   lo stack è giù.
3. **`f1tenth_ctl` importato, non lanciato come processo.**
   - Stessi dict del JSON senza ~0.5 s di avvio Python a chiamata.
   - Le operazioni lunghe (stop di 15 s, piano fino a 60 s) vanno in un
     thread pool.
   - I suoi sottoprocessi ROS (monitor, rosprobe) restano com'erano.
4. **E-STOP in-process.**
   - `f1tenth_ctl estop` come sottoprocesso deve avviare rclpy e aspettare
     il match col mux: da 1 a 3 s prima del primo zero.
   - Il gateway tiene publisher e client **creati all'avvio e già
     matchati**, e riusa `rosprobe.ZeroDrive`/`ServiceStep`/`estop_result`
     tramite un piccolo refactor: `rosprobe.estop` accetta un nodo
     esistente, il comportamento della CLI resta identico.
   - Se il percorso in-process fallisce, ripiega su `f1tenth_ctl estop` come
     sottoprocesso.
5. **Il teleop pubblica solo su `/teleop`** (priorità 100).
   - Sopra la navigazione, sotto tutte le sorgenti di `/safety_stop`.
   - Il gateway non pubblica mai su `/drive` e scrive su `/safety_stop` solo
     per l'e-stop.
   - Nessun cambio a `mux.yaml`.
6. **Python sull'Orin:** venv `~/.local/share/f1tenth_gateway/venv
   --system-site-packages` con `fastapi`, `uvicorn` e, se scegli la
   trascrizione sull'Orin, `faster-whisper` più `av<17`. rclpy arriva dal
   `setup.bash` sorgente.
   - **Alternativa:** `aiohttp`, già installato. Scarto perché FastAPI dà
     validazione con pydantic e un `TestClient` WebSocket che rende semplici
     i test richiesti.
   - Il codice resta compatibile con Python 3.10: niente `TaskGroup`,
     `tomllib` o `datetime.UTC`.
7. **Frontend compilato su linus.** Sull'Orin non c'è Node; la build
   (~300 KB) va sull'Orin con git (default) o rsync: §4, domanda 8.

**Il bot Telegram resta com'è.** Condivide con il gateway lo stato missioni
di `f1tenth_ctl` e la FIFO del planner.

**Limite noto:** due piani lanciati nello stesso istante da bot e PWA possono
attribuirsi il RESULT sbagliato. Il gateway li serializza tra sé con un
file-lock; il bot non lo prende.

---

## 3. Contratto API definitivo

Tutte le chiamate REST vogliono `Authorization: Bearer <token>`. Il WebSocket
non può avere header da browser, quindi il token arriva nel primo messaggio
(§3.2). I file statici sono pubblici e non contengono segreti.

### 3.1 REST

| Metodo | Path | Effetto | Implementazione |
|---|---|---|---|
| GET | `/api/status` | `ready`, `checks[]` (value/want), `processes{stack,planner,logger}` (stato, nodi, uptime), `estop_latched`, `mission{state,id,…}`, `battery{v,pct}`, `vesc_temp_c`, `cpu_percent` | `cmd_status` in cache, aggiornato ogni 1 s in background, più i topic §1.7 |
| POST | `/api/process/{stack\|planner\|logger}` `{action: start\|stop}` | 202 + transizione visibile in `status` (`starting…`/`stopping…`); un'operazione per processo alla volta, nessun retry | `cmd_start` / `cmd_stop` |
| POST | `/api/estop` (anche WS `{type:"estop"}`) | revoca il teleop + zero su `/teleop` e su `/safety_stop` (2 s, 50 Hz) + `/mission/emergency_stop` + `/mission/abort_mission`; risponde `steps{}` e `car_held` | in-process (§2.4); ripiego `f1tenth_ctl estop` |
| POST | `/api/estop/reset` *(da confermare, §4, domanda 5)* | sblocca il latch: abort, zero tenuto, riavvio di `behavior`, verifica | `cmd_estop_reset` |
| POST | `/api/mission/transcribe` (multipart, ≤ 60 s, ≤ 2 MB; `audio/mp4` da iOS) | `{text, duration_s, transcribe_s}`; l'audio non viene mai conservato | `Transcriber.transcribe_bytes` |
| POST | `/api/mission/plan` `{text}` (≤ 500 caratteri, una riga) | 202 `{job_id}`; rifiuta se il planner è spento, non plan-only, o c'è già un piano o una missione in corso | `cmd_send` + `cmd_planner_log` |
| GET | `/api/mission/plan/{job_id}` | `{state: planning\|done\|failed, plan_id, summary{n_moves,max_speed_mps,distance_m,max_duration_s,targets}, steps[], json, valid, errors[], unsupported[], notes[], plan_seconds, log_tail}` | `results[]` di `planner-log`; `valid` = RESULT `ok` **e** `load_mission_file` passa |
| POST | `/api/mission/start` `{plan_id}` | load + start; risponde `countdown_s` (15 oppure 0); nessun retry | `cmd_mission_load` + `cmd_mission_start`, con i loro rifiuti |
| POST | `/api/mission/abort` | `/mission/abort_mission` (missione ABORTED, MPC in hold, **niente latch**) | `std_srvs/Trigger` in-process |
| GET | `/api/runs?limit=20` | `[{id, mission_id, mission_label, outcome, started, duration_s, finalized, report{ok,why}, video{ok,why}}]` | manifest in `~/f1tenth_archive/{complete,active,incomplete}` |
| GET | `/api/runs/{id}/report` | pagina HTML generata al volo: manifest, move della missione, esiti per move (`mission_reports/*.json`), snapshot jpg | id validato con regex e path dentro l'archivio |
| GET | `/api/runs/{id}/video` | mp4 della run se esiste (`<run_dir>/*.mp4` o `<ws>/mission_videos/<run_id>*.mp4`), con supporto **Range**, necessario a `<video>` su iOS; altrimenti 404 col motivo | |
| GET | `/api/map.png?v=N`, `/api/map` | `/slam/map` resa come PNG (zlib, senza Pillow) + `resolution`, `origin`, `width`, `height` | solo se c'è un client in Telemetria |

### 3.2 WebSocket `/ws`

**Client → server**

| Messaggio | Quando |
|---|---|
| `{type:"auth", token}` | primo messaggio, entro 2 s, altrimenti chiusura 4401 |
| `{type:"ping", id, t}` | ogni 500 ms (RTT e prova di vita) |
| `{type:"drive_acquire"}` / `{type:"drive_release"}` | entrata e uscita da Guida |
| `{type:"teleop", seq, thr, steer, limit, deadman}` | 20 Hz solo con dead-man attivo; al rilascio subito un messaggio con `deadman:false, thr:0` |
| `{type:"estop"}` | il pulsante EMERGENZA (parte anche la POST) |

**Server → client**

| Messaggio | Frequenza | Contenuto |
|---|---|---|
| `hello` | dopo l'auth | limiti del teleop, `auto_abort_s`, versione |
| `telemetry` | 10 Hz | `{t, pose{x,y,yaw}, v_meas, v_cmd, steer_cmd, cmd_source, safety{active,source}, mission{state,id,move_index,n_moves,move_id,elapsed_s,countdown_left_s}, battery{v,pct}, estop_latched}` |
| `status` | 1 Hz | READY, checks, processi, CPU, temperatura VESC |
| `bt` | al cambiamento, ≤ 2 Hz | `{nodes:[{id,parent,name,kind,status}]}` |
| `path` | 1 Hz | punti nuovi decimati (≥ 5 cm), `horizon` MPC in frame `map`, `reset` a inizio missione |
| `map` | al cambiamento | `{version}`; il client scarica `/api/map.png?v=` |
| `drive` | eventi | `granted \| busy \| refused \| revoked` + motivo |
| `pong` | risposta al ping | |
| `event` | eventi | `{level, text}`, es. "missione interrotta: connessione persa", "E-STOP da Telegram" |

**Rilevamento "connessione persa":**
- **Client:** nessun messaggio dal server per 1.5 s (la telemetria a 10 Hz
  garantisce traffico).
- **Server:** nessun messaggio dal client per 1.5 s chiude la sessione.

### 3.3 Adattamenti dei mockup imposti dai dati reali

- **Cockpit**
  - Sottotitoli dei processi:
    - Stack `running · <nodi> nodi · <uptime>`;
    - Planner `running · plan-only` (non "50 Hz");
    - Logger `in attesa` oppure `rec <run> · <MB>`, dalla cartella `active/`.
  - Riga dei numeri: VEL m/s, VESC °C (max dei tre NTC), CPU % (da
    `/diagnostics/system_status`).
- **Batteria:** V, più una % stimata solo se confermi le celle (§4,
  domanda 7).
- **Missione / PIANO**
  - Badge VALIDATO / ERRORI.
  - Tre numeri: **move · v max · lunghezza** (oppure "fino a <oggetto>").
  - Passi = `steps[].action` + "fino a <stop_condition>", poi JSON
    espandibile.
  - ⚠️ per `unsupported` e note.
  - Il piano scade dopo 5 min, come nel bot.
- **Missione in corso**
  - IN CORSO / AVVIO TRA n s (countdown).
  - TEMPO.
  - **PASSO k/n** al posto di GIRO.
  - Barra = move completati / n.
  - VEL REALE.
  - **v CMD** al posto di ERR. LAT., che non esiste; in alternativa la
    distanza frontale, ma il tipo del topic è da verificare.
- **Telemetria**
  - Mappa SLAM come sfondo, percorso fatto, posa.
  - **Orizzonte MPC tratteggiato** al posto della raceline.
  - Velocità 30 s reale vs comandata.
  - BT come grafo.
- **Run:** solo missioni registrate dal logger, senza guida manuale.
  Etichetta = `mission_id` più il riassunto del piano se `f1tenth_ctl` lo
  conosce.
  - Esiti: OK = `COMPLETE`, ABORT = `ABORTED`, E-STOP = `EMERGENCY_STOP`,
    grigio = il resto.
  - Report sempre disponibile se il manifest è leggibile.
  - Video disattivato con motivo "nessun mp4: generalo con
    `mission_render`".

### 3.4 Requisiti SPEC §5: come li copro

| # | Requisito | Come |
|---|---|---|
| 1 | Watchdog 250 ms; chiusura WS → zero | loop di pubblicazione a 50 Hz. Ultimo teleop valido più vecchio di 250 ms, oppure WS chiuso → zero subito, poi zeri per 0.5 s ("coda"), poi silenzio: la lane scade e libera la navigazione |
| 2 | Rilascio del gas → zero immediato | il client manda `deadman:false` al `pointerup`/`pointercancel`/`visibilitychange`; il server pubblica lo zero nello stesso callback, senza aspettare il tick |
| 3 | Solo la lane teleop, mai bypassare la sicurezza | publisher solo su `/teleop` (100 < 200). Guida rifiutata se un altro nodo pubblica `/teleop` (es. joystick), se il mux non è sottoscritto, se l'e-stop è latched, o se una missione è in countdown/RUNNING/HOLDING. Lease revocata quando ne parte una. `frame_id = base_link/pwa_teleop` |
| 4 | Clamp lato server | velocità ≤ min(limite client, `max_speed_mps` di config); sterzo ≤ ±0.30 rad (inviluppo servo +0.320/−0.309); salita ≤ 1.0 m/s², discesa immediata; sterzo ≤ 1.5 rad/s; input accettati ≤ 50 msg/s e `seq` crescente; NaN o fuori [−1, 1] → zero e revoca |
| 5 | Mai sotto 0.4 m/s; retromarcia no | `|thr| ≤ zona morta (0.15)` o `thr < 0` → 0; altrimenti v = 0.4 + (thr−zm)/(1−zm)·(limite−0.4). 0.4 è `min_moving_speed_mps` di `stack_params.yaml`: accoppiamento documentato, letto dal file se presente |
| 6 | Un solo client di guida | lease legata alla connessione WS; il secondo riceve `busy` ("guida occupata"); la lease si libera alla chiusura o al rilascio |
| 7 | Niente retry automatici sulle azioni critiche | start, abort e stop processo: una chiamata, esito mostrato; il client disabilita il pulsante finché non arriva la risposta |

**In più, non richiesti dalla SPEC:**
- token confrontato a tempo costante;
- 5 tentativi sbagliati al minuto, poi 429;
- log di ogni comando con l'utente Tailscale (header `Tailscale-User-Login`
  aggiunto da `tailscale serve`).

---

## 4. Domande aperte: default proposti

1. **Perdita del client durante una missione autonoma.**
   - **Default: sì, abort (non e-stop) dopo 2 s.** Vale solo per le missioni
     avviate dal gateway, e solo se nessun client autenticato ha mandato
     traffico negli ultimi 2 s. Vale anche durante il countdown.
   - È un abort: non latcha, si riparte con un nuovo piano. Al ritorno del
     client arriva l'evento "missione interrotta: connessione persa".
   - Una missione partita da Telegram o da terminale non viene toccata.
   - Motivo: il mockup promette "arresto affidato al watchdog". Senza
     questa regola, durante una missione non sarebbe vero, e le missioni
     sono corte (timeout 30-90 s, 0.5 m/s), quindi interromperle costa poco.
   - **Costo:** bloccare l'iPhone durante una missione la interrompe. L'app
     chiede di tenere lo schermo acceso e prova la Wake Lock API (supporto
     nelle app aggiunte alla Home da verificare sul tuo iPhone).
2. **Dove sono i check di READY:** in `f1tenth_ctl`, cioè `ready.yaml` +
   `ready.py` + unità `f1t-monitor` (§1.2). Il gateway chiama `cmd_status`.
   Nessun nodo dedicato.
3. **Interfaccia del planner per avere il piano senza avviarlo:** non è un
   servizio. `llm_planner_node --dry-run` come unità `f1t-planner` di
   `f1tenth_ctl`, poi `send` + `planner-log`, che salva la missione e ne fa
   il riassunto; l'avvio passa da `mission-load`/`mission-start` (§1.4).
   Nessuna modifica al planner.
4. **Dati per la mappa 2D:** non c'è raceline né file di pista.
   - Sfondo: `/slam/map`.
   - Posa e percorso: `/ekf_global/odometry/filtered`.
   - Tratteggiata: l'orizzonte MPC (`/mpc/solver_status`, frame odom
     convertito in map).

**Domande nuove emerse dalla ricognizione:**

5. **Sblocco dell'E-STOP dall'app.**
   - La SPEC dice "nessun reset", ma `f1tenth_ctl estop-reset` esiste: abort
     + zero tenuto + riavvio di `behavior`, provato in sim.
   - Senza sblocco, ogni EMERGENZA premuta in lab costa un riavvio completo
     dello stack.
   - **Default proposto:** nel banner E-STOP un pulsante "SBLOCCA (riavvia
     BT)" a pressione prolungata di 2 s, abilitato solo senza lease di guida
     attiva.
6. **Branch base e branch della macchina** (§0.1).
   - **Default:** `pwa-control` = `campaign-final` + fast-forward a
     `telegram-control`.
   - La macchina girerà su quel branch? Se resta `mpc_corr_v2_reverse`, i
     commit del gateway si portano con un cherry-pick, ma manca lo stop
     LiDAR di ultima istanza.
7. **Batteria in %:** confermi che è una 3S?
   - **Default:** mostro i volt sempre e una % lineare 3.5–4.2 V/cella,
     marcata "stima", solo con `battery_cells` impostato in config. Senza,
     solo volt.
8. **Build del frontend sull'Orin.**
   - **Default:** la build compilata viene committata in
     `src/f1tenth_gateway/www/`, così l'Orin si aggiorna solo con git.
   - Alternativa: non committarla e copiarla con `scripts/pwa_deploy.sh`
     (rsync da linus).
9. **Trascrizione vocale.**
   - **Default:** sull'Orin, CPU int8, modello `small`, 4 thread, caricato
     al primo uso e non all'avvio, per non pesare sulla CPU dello stack.
     Stima: alcuni secondi per una frase.
   - Alternative:
     - (b) il gateway inoltra l'audio a linus, dove c'è la GPU;
     - (c) dettatura della tastiera iOS: zero codice, già disponibile nel
       campo testo.

     La (c) funziona comunque. Il pulsante hold-to-talk resta disattivato
     con motivo se il modello manca.
10. **Velocità massima del teleop (cap lato server).**
    - **Default:** `max_speed_mps: 1.0`, stepper 0.4–1.0 a passi di 0.1,
      partenza 0.6.
    - Il mockup mostra 2.0, ma le missioni sono limitate a 0.5 da
      `max_forward_speed_mps`. Alzare il cap è una riga di config.
11. **Guida richiede READY?**
    - **Default: no.** Bastano stack in esecuzione, mux sottoscritto a
      `/teleop`, nessun altro publisher su `/teleop`, niente latch e nessuna
      missione attiva.
    - Motivo: per guidare a mano localizzazione e servizio missioni non
      servono, e batteria e prossimità restano coperte dal BT su
      `/safety_stop`.

---

## 5. Piano di implementazione

Commit piccoli su `pwa-control`, push a fine sessione. Nessun cambio ai
default dello stack, a `mux.yaml` o al BT.

**Passo 0: base**
- Fast-forward di `pwa-control` a `telegram-control` (se confermi §4,
  domanda 6).
- Commit di `docs/pwa/` (SPEC + mockup) e di questo file.

**Passo 1: `f1tenth_ctl`, refactor senza cambi di comportamento**
- `rosprobe.estop(spec)` diventa `estop_on(node, executor, spec)` più un
  wrapper identico all'attuale, così il gateway riusa la stessa sequenza
  con il suo nodo.
- Un test che fissa l'equivalenza.
- File: `src/f1tenth_ctl/f1tenth_ctl/rosprobe.py`,
  `src/f1tenth_ctl/test/test_estop_on_node.py`.

**Passo 2: `src/f1tenth_gateway`** (ament_python)
- Packaging, secondo CLAUDE.md:
  - `setup.py` con `extras_require={'test': ['pytest']}`;
  - `package.xml` senza `--` nei commenti.
- Moduli puri, testabili senza ROS:
  - `config.py`: YAML + env; token da `F1T_GATEWAY_TOKEN`, rifiutato se
    sotto i 32 caratteri;
  - `teleop.py`: mappatura gas/sterzo, zona morta, clamp, rate limit, lease,
    watchdog come macchina a stati con orologio iniettato;
  - `missions_view.py`: piano → summary/steps; stato missione + move_outcome
    → passo k/n, tempo, countdown; auto-abort per perdita del client;
  - `runs.py`: scansione dell'archivio, pagina di report, ricerca dei video,
    validazione degli id;
  - `mapimg.py`: OccupancyGrid → PNG con zlib.
- Porte verso l'esterno:
  - `ros_port.py`: interfaccia (`telemetry()`, `publish_teleop()`,
    `estop()`, `abort()`, `bt()`, `map()`…);
  - `ros_bridge.py`: implementazione rclpy, nodo `pwa_gateway`, executor in
    un thread, sottoscrizioni pesanti solo con client collegati;
  - `mock_ros.py`: implementazione finta per linus e per l'e2e. Simula
    stati della missione, odometria, BT, e-stop latch e mux, e registra
    ogni comando pubblicato su `/teleop` e `/safety_stop` per i test.
  - `ctl_port.py`: `f1tenth_ctl` in-process con lo stesso contratto, più un
    finto per i test.
- Server:
  - `app.py`: FastAPI (REST, WS, auth, file statici, header di sicurezza);
  - `__main__.py`: `f1tenth_gateway --config … [--mock]`.
- Configurazione e avvio:
  - `config/gateway.yaml`, `config/gateway_mock.yaml`;
  - `bin/f1tenth_gateway` (sorgente l'env ROS, avvia il venv);
  - `systemd/f1tenth-gateway.service` (unità utente,
    `EnvironmentFile=%h/.config/f1tenth_gateway/env`, `Restart=always`).
- Test (`test/`):
  - `test_teleop_mapping.py`, `test_teleop_watchdog.py`,
    `test_teleop_lease.py`, `test_clamp_rate_limit.py`;
  - `test_auth.py`, `test_api_mock.py` (REST + WS con `TestClient`);
  - `test_estop.py`, `test_mission_flow.py`, `test_auto_abort.py`;
  - `test_runs.py`, `test_mapimg.py`;
  - `test_ros_bridge_humble.py`: si salta senza rclpy.

**Passo 3: `apps/pwa`** (Vite + TypeScript + Preact, nessuna UI library)
- Struttura:
  - `src/tokens.css` con i token SPEC §2;
  - font Inter e JetBrains Mono self-hosted con `@fontsource`, niente
    Google Fonts;
  - `src/api.ts`, `src/ws.ts` (auth, ping, riconnessione);
  - `src/state.ts`.
- Componenti in `src/components/`:
  - `EstopButton` (normale / NON RAGGIUNGIBILE), `TabBar`;
  - `SlideToStart`, `HoldButton` (anello di progresso);
  - `Joystick` (Pointer Events, `touch-action: none`, multi-touch per
    `pointerId`);
  - `BtGraph`, `SpeedChart`, `Trajectory`;
  - `Banner` (connessione persa / E-STOP ATTIVO).
- Schermate in `src/screens/`: `Pair` (inserimento del token), `Cockpit`,
  `Missione`, `MissioneInCorso`, `Guida` (solo orizzontale, overlay "Ruota
  il telefono"), `Telemetria`, `Run`.
- PWA:
  - `public/manifest.webmanifest`, icone, `apple-touch-icon`;
  - `public/sw.js` scritto a mano: cache solo dell'app shell, mai `/api` né
    `/ws`.
- Safe area con `env(safe-area-inset-*)`, viewport `viewport-fit=cover`.
- `npm run build` scrive in `src/f1tenth_gateway/www/`.

**Passo 4: test e2e** (`apps/pwa/e2e/`, Playwright)
- Il gateway gira in `--mock` su linus.
- Progetti: iPhone 390×844 e 844×390. Uso WebKit se `playwright install
  webkit` funziona senza `sudo` per le dipendenze di sistema, altrimenti
  Chromium con `hasTouch` e user agent iPhone.
- Casi:
  - abbinamento col token;
  - READY / NOT READY con check falliti;
  - switch dei processi con lo stato di transizione;
  - EMERGENZA, poi banner E-STOP;
  - connessione persa: il mock chiude il WS, i comandi si disattivano, c'è
    il banner e la riconnessione;
  - piano → slide-to-start (un tap non basta) → in corso → abort a
    pressione prolungata;
  - Guida: verticale dà "Ruota", orizzontale con due pollici, il rilascio
    del gas manda zero, il secondo client riceve "guida occupata".

**Passo 5: verifica Humble su linus**
- Container `f1tenth_more:cpu` (Humble, Python 3.10.12, già presente):
  unit test del gateway e un test del bridge rclpy contro publisher e
  servizi finti.
- `ROS_DOMAIN_ID` isolato e `ROS_LOCALHOST_ONLY=1`: nessun contatto con lo
  stack né con la sim.

**Passo 6: documentazione e consegna**
- `src/f1tenth_gateway/README.md`:
  - installazione sull'Orin: venv, unità, token con `read -rs`, chmod 600;
  - `tailscale serve`, con i due prerequisiti di §0.4;
  - aggiunta alla Home su iPhone;
  - risoluzione problemi;
  - sicurezza.
- Checklist di prova sulla macchina, prima a ruote sollevate, in fondo al
  README e nel riepilogo finale.

**Fuori dalla v1:**
- registrazione delle run manuali;
- generazione automatica dei video;
- trascrizione su GPU Jetson;
- takeover della guida da un secondo client;
- retromarcia.
