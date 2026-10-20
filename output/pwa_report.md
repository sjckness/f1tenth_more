# PWA "F1Tenth Lab Control": report finale (fasi 1–4)

Data: 2026-10-20. Branch: `pwa-control`. Tutto è stato provato su linus: con il
gateway finto (`--mock`), con rclpy reale sotto Humble (immagine
`f1tenth_more:cpu`) e con Playwright (WebKit e Chromium).

**Niente è stato provato sulla macchina, sull'Orin o su un iPhone vero.** Quello
che non è stato eseguito è marcato *non verificato*.

---

## 1. Riepilogo

### Branch

| Branch | Commit | Stato |
|---|---|---|
| `campaign-final` | `cd180d8` | **fermo**: non spostato, non pushato (aspetta il push di vesc, §11) |
| `cf-next` | `3feae47` | la nuova base: campaign-final + door-fixes + obstacles-v2 + MPC v2 reverse + fix; nessun codice dell'app |
| `pwa-control` | cf-next + i commit sotto | solo app: telegram-control completo + gateway + PWA |

`pwa-control` è stato ribasato su `cf-next` `3feae47` con `git rebase
--rebase-merges`. Il merge di telegram-control resta un merge.

Rispetto a `cf-next` aggiunge solo:
- `apps/pwa`;
- `src/f1tenth_gateway`;
- `src/f1tenth_ctl` e `src/f1tenth_telegram`;
- `docs/pwa` e `output/pwa_*`;
- `output/telegram`;
- `tools/test_gateway_humble.sh`;
- due note in `CLAUDE.md` (da telegram-control).

### Commit (`git log --first-parent cf-next..pwa-control`)

| Hash | Messaggio |
|---|---|
| `a45d87e` | Merge telegram-control (b131b9b) into the app branch |
| `2819c62` | f1tenth_ctl: the ROS environment for a shell -- discovery server, super client, --no-daemon |
| `55b00e7` | f1tenth_ctl rosprobe: estop_with(node, ...) -- the e-stop on a node that already exists |
| `ce4c356` | docs/pwa: SPEC and the 7 mockups; output/pwa_recon.md (Phase 0 recon) |
| `a1a2863` | f1tenth_gateway: the PWA's gateway (REST + WebSocket + rclpy + f1tenth_ctl), mock and tests |
| `c95b413` | f1tenth_gateway: RosBridge on real rclpy; tools/test_gateway_humble.sh |
| `20b8c68` | f1tenth_gateway mock: POST /api/_mock/reset -- a fresh READY car between e2e tests |
| `c58cdda` | f1tenth_gateway: READY follows the e-stop latch at once |
| `2c3eeb9` | apps/pwa: F1Tenth Lab Control, the iPhone PWA (Vite + Preact) |
| `76915d2` | apps/pwa: scripts/deploy.sh -- dist/ to the Orin over Tailscale (npm run deploy) |
| `fadab1d` | apps/pwa: Playwright e2e against the gateway's pretend car (npm run test:e2e) |
| `9b43f43` | apps/pwa: npm run screens -- every screen and state into output/pwa_screens/, beside its mockup |
| `ce84001` | f1tenth_gateway tests: skip the app tests per test when fastapi is missing |
| `4586b99` | f1tenth_gateway: README -- install on the Orin, token, systemd, tailscale serve, iPhone, tests |
| `1aa4a45` | apps/pwa: Guida warns when the screen cannot be kept on |
| *(ultimo)* | questo report + `output/pwa_screens/` |

**La correzione dell'ambiente ROS** (`2819c62`) riguarda `f1tenth_ctl`.

- `f1tenth_ctl check` ha una voce in più, `super_client`. È un avviso, non un
  requisito: segnala se `ROS_SUPER_CLIENT` non è `TRUE` con un Discovery Server
  configurato.
- Commento in `ctl.env` e sezione nel README ("The same graph from a shell"):
  `set -a; . ctl.env; set +a`, poi `ros2 … --no-daemon`.
- Aggiornato `output/telegram/discovery.md`.
- Un test in più in `test_process_manager.py`.

`ROS_DISCOVERY_SERVER=127.0.0.1:11811` e `ROS_SUPER_CLIENT=TRUE` erano già in
`ctl.env`. Il gateway li prende da lì.

### File

- **Gateway** `src/f1tenth_gateway/`:
  - `f1tenth_gateway/app.py`: REST, WS, regole;
  - `teleop.py`: guida, watchdog, clamp;
  - `mission.py`: stato missione, auto-abort;
  - `ros_bridge.py`: nodo rclpy;
  - `ctl_port.py`: f1tenth_ctl in-process;
  - `transcription.py` + `transcribe_worker.py`;
  - `runs.py`;
  - `mapimg.py`;
  - `mock.py`;
  - `config.py`;
  - `__main__.py`.
  - Altro: `bin/f1tenth_gateway`, `systemd/f1tenth-gateway.service`,
    `config/gateway.yaml`, `config/gateway_mock.yaml`, `requirements.txt`,
    `README.md` (italiano).
  - 7 file di test più `test/fixtures/`.
- **PWA** `apps/pwa/`:
  - `src/app.tsx`, `store.ts`, `ws.ts`, `api.ts`;
  - `components/ui.tsx`, `Joystick.tsx`, `pointer.ts`;
  - `screens/Cockpit|Missione|Guida|Telemetria|Run.tsx`;
  - `public/manifest.webmanifest`, `sw.js`, icone;
  - `scripts/deploy.sh`;
  - `e2e/*.spec.ts`;
  - `playwright.config.ts`.
  - `dist/` non è nel repo.
- **f1tenth_ctl**: `rosprobe.py` (`estop_with`), `cli.py` (`super_client`),
  `ctl.env`, README, 2 test.
- **Strumenti e documenti**: `tools/test_gateway_humble.sh`, `docs/pwa/` (SPEC +
  mockup), `output/pwa_recon.md`, `output/pwa_screens/`, questo report.

---

## 2. Architettura realizzata

```
iPhone · PWA (Safari, "Aggiungi alla schermata Home")
   │ HTTPS + WSS, solo nella tailnet
   ▼
tailscale serve  :443  (certificato *.ts.net)              ── Orin ──
   │ http://127.0.0.1:8770        (porta: listen.port / --port)
   ▼
f1tenth-gateway.service  (unità systemd UTENTE, fuori dallo stack, Restart=always, KillSignal=SIGINT)
bin/f1tenth_gateway: . ctl.env  →  venv/bin/python -m f1tenth_gateway
 ├─ uvicorn + FastAPI (event loop): REST, 1 WebSocket, file statici di www_dir
 │    ├─ status_loop 1 Hz    → f1tenth_ctl status (thread pool) → "status" a tutti
 │    └─ telemetry_loop 10 Hz → "telemetry"; ogni 0.2 s "bt"; ogni 1 s "path"/"map";
 │                              guardia client persi (1.5 s), guardia lease (2 Hz), auto-abort
 ├─ RosBridge: nodo rclpy "pwa_gateway", MultiThreadedExecutor (4 thread) in un thread suo
 │    ├─ timer 50 Hz → Teleop.tick() → /teleop                     (callback group esclusivo)
 │    ├─ pub /safety_stop + client /mission/emergency_stop, /mission/abort_mission: creati all'avvio
 │    ├─ sub sempre: /mission/status (transient local), /mission/move_outcome, /diagnostics/battery_status
 │    └─ sub solo con client connessi: /odom, /ekf_global/odometry/filtered, /odometry/filtered,
 │       /ackermann_drive, /diagnostics/system_status, /behavior/tree_status, /mpc/solver_status,
 │       /slam/map, /sensors/core; snapshot stream del BT
 ├─ CtlPort: f1tenth_ctl importato (cli.cmd_status/start/stop/estop/estop-reset/send/planner-log/
 │           mission-load/mission-start), un lock per processo, un lock per i piani
 └─ TranscriberProcess: processo figlio (nice 10, faster-whisper CPU int8), scaricato dopo 10 min
```

**Porte.**
- 443: `tailscale serve`.
- 127.0.0.1:8770: il gateway. Non è raggiungibile dalla LAN; si cambia con
  `listen.port` o `--port`.
- 127.0.0.1:11811: il Discovery Server, dove il gateway si collega come super
  client.
- Il gateway non apre altre porte.

**Collegamento a ROS.** `bin/f1tenth_gateway` carica lo stesso `ctl.env` di
`f1tenth_ctl`: Humble, l'install del workspace, Discovery Server, super client.
Il nodo `pwa_gateway` vede quindi il grafo che vede `f1tenth_ctl` e vive anche
con lo stack spento.

**Collegamento a f1tenth_ctl.**
- È importato, non lanciato: ottiene gli stessi dict del `--json` senza avviare
  Python a ogni chiamata.
- Le chiamate lunghe vanno nel thread pool: stop dello stack, piano fino a
  60 s, estop-reset.
- L'E-STOP usa `rosprobe.estop_with()` sul nodo del gateway, con publisher e
  client già pronti. Se fallisce, ripiega su `f1tenth_ctl estop`.

**Unità systemd**: `src/f1tenth_gateway/systemd/f1tenth-gateway.service`.
- `EnvironmentFile=%h/.config/f1tenth_gateway/env` (il token, chmod 600).
- `ExecStart=%h/dev_ws/f1tenth_more/src/f1tenth_gateway/bin/f1tenth_gateway`.
- `KillSignal=SIGINT`: allo stop revoca la Guida e pubblica un ultimo zero su
  `/teleop`.

---

## 3. API finale

Ogni chiamata REST vuole `Authorization: Bearer <token>`. Fanno eccezione:
- `/api/runs/*/report|video|snapshots`, che accettano anche il cookie di
  sessione `f1t_session` (`<video>` e i link non possono mandare header);
- i file statici.

Dopo 5 token sbagliati in un minuto il gateway risponde 429. Ci sono CSP e
`nosniff`. `index.html` e `sw.js` sono `no-store`.

### REST

| Metodo | Path | Effetto | Dietro |
|---|---|---|---|
| POST | `/api/session` | cookie `f1t_session` (12 h, HttpOnly, Secure, SameSite=Strict, path `/api`) | |
| GET | `/api/status` | `ready`, `failing[]`, `checks[]` (name, kind, ok, value, want, why), `processes{stack,planner,logger}`, `estop_latched`, `mission{…}`, `battery`, `vesc_temp_c`, `cpu_percent`, `drive{busy,blockers}`, `reset_allowed`, `transcription` | `f1tenth_ctl status` in cache 1 s; il check `estop` preso da `/mission/status` |
| POST | `/api/process/{stack\|planner\|logger}` `{action}` | 202 + job; 409 se c'è già un'operazione, o se è uno stop dello stack con la macchina in moto o una missione | `cmd_start` / `cmd_stop` |
| GET | `/api/jobs/{id}` | `{state: running\|done\|failed, result}` | |
| POST | `/api/estop` (anche WS `estop`) | revoca la Guida, zero su `/teleop`, poi `estop_with`: zero su `/safety_stop`, `/mission/emergency_stop`, `/mission/abort_mission`; risponde `car_held`, `steps`, `path: gateway\|f1tenth_ctl` | `rosprobe.estop_with`; ripiego `cmd_estop` |
| POST | `/api/estop/reset` | 202 + job; 409 se non latched, Guida aperta o macchina non ferma da 1 s | `cmd_estop_reset` |
| POST | `/api/mission/transcribe` (multipart `audio`) | `{text, …}`; 409 con missione attiva o Guida aperta; 413 sopra 2 MB | processo figlio faster-whisper |
| POST | `/api/mission/plan` `{text}` | 202 + job → `{plan_id, valid, summary, steps[], json, errors[], …}`; 409 con missione attiva o un altro piano | `cmd_send` + `cmd_planner_log` (planner plan-only) + `validate_mission_file` |
| POST | `/api/mission/start` `{plan_id}` | `{countdown_s, load, start}`; 404/409 se il piano è sconosciuto o più vecchio di 5 min, se non è valido, con E-STOP, con una missione o con la Guida aperta; NOT READY lo rifiuta `cmd_mission_start` | `cmd_mission_load` + `cmd_mission_start` |
| POST | `/api/mission/abort` | `/mission/abort_mission`; 502 se fallisce | `std_srvs/Trigger` in-process |
| GET | `/api/runs?limit=20` | elenco dall'archivio | `~/f1tenth_archive` |
| GET | `/api/runs/{id}/report` | pagina HTML | |
| GET | `/api/runs/{id}/snapshots/{n}` | jpg | |
| GET | `/api/runs/{id}/video` | mp4 con `Range` | |
| GET | `/api/map`, `/api/map.png` | `/slam/map` come PNG + metadati | |
| POST | `/api/_mock/{set,reset,freeze,drop,state}` | **solo con `--mock`** (test e screenshot) | |

### WebSocket `/ws`

**Client → server**
- `{type:"auth", token}`: primo messaggio, entro 2 s; altrimenti chiusura 4401
  (4429 se bloccato).
- `{type:"ping", id, t}`: ogni 500 ms.
- `{type:"drive_acquire"}` / `{type:"drive_release"}`: entrata e uscita da
  Guida.
- `{type:"teleop", seq, thr, steer, limit, deadman}`: 20 Hz col pollice sul gas;
  al rilascio subito `{deadman:false}`.
- `{type:"estop"}`.

**Server → client**
- `hello`: limiti del teleop, `auto_abort_s`, `lost_after_s`.
- `status`: 1 Hz, più subito quando il latch E-STOP cambia.
- `telemetry`: 10 Hz, `{pose, v_meas, v_cmd, steer_cmd, cmd_source, safety,
  mission, battery, estop_latched, drive, reset_allowed}`.
- `bt`: al cambiamento, al massimo 5 Hz.
- `path`: 1 Hz, decimato, con `horizon` MPC.
- `map`: `{version}`.
- `drive`: `granted | busy | refused | released | revoked | tripped |
  not_holder | invalid`, più il motivo.
- `pong`.
- `event`: `{level, text}`; gli ultimi 5 arrivano anche alla connessione.

**Cosa usa sul grafo**
- Pubblica: `/teleop` (AckermannDriveStamped, `frame_id`
  `base_link/pwa_teleop`) e `/safety_stop` (solo per l'E-STOP).
- Servizi: `/mission/emergency_stop`, `/mission/abort_mission`,
  `/behavior_executor_node/snapshot_streams/open|close`; load e start passano
  da f1tenth_ctl.
- Non pubblica mai su `/drive` e non modifica `mux.yaml`.

---

## 4. Deviazioni

### Dalla SPEC

| # | SPEC | Realizzato | Motivo |
|---|---|---|---|
| 1 | bind sull'interfaccia Tailscale | bind 127.0.0.1 dietro `tailscale serve` | TLS valido (`*.ts.net`) per microfono e service worker; nessuna esposizione in LAN (recon §2.1, approvato) |
| 2 | E-STOP senza reset dall'app | SBLOCCA: pressione di 2 s nel banner, abilitato solo con velocità misurata 0 da 1 s, via `f1tenth_ctl estop-reset` | decisione 2 |
| 3 | `POST /api/mission/plan` sincrono | 202 + job (`GET /api/jobs/{id}`), come stop dei processi ed estop-reset | il piano può durare 60 s, lo stop 15 s |
| 4 | WS con header bearer | token nel primo messaggio | il WebSocket del browser non manda header |
| 5 | GIRO, ERR. LAT., raceline | PASSO k/n, v CMD, orizzonte MPC su mappa SLAM | non esistono giri né raceline (recon §3.3, approvato) |
| 6 | PIANO: giri · v max · arrivo | move · v max · "fino a" (o distanza) | le missioni sono move |
| 7 | batteria V + % | solo V | decisione 6 |
| 8 | testo "watchdog di bordo" in Connessione persa | le regole vere: Guida ferma in 250 ms; missione avviata dall'app → ABORT dopo 2 s; le altre continuano | il testo del mockup non era vero per le missioni |
| 9 | (nessun pulsante) | PIANIFICA esplicito | il piano costa un giro del planner: si chiede di proposito |
| 10 | — | READY segue subito il latch E-STOP | prima, per ≤ 1 s, "READY" sotto il banner "E-STOP ATTIVO" |
| 11 | — | un solo avviso (toast) alla volta, sopra il titolo | tre toast coprivano ABORT durante il countdown |
| 12 | Wake Lock non nella SPEC | in missione e in Guida, avviso se manca (anche in Guida) | decisione 3 |

### Dalla ricognizione

| # | Recon | Realizzato | Motivo |
|---|---|---|---|
| 1 | Transcriber importato nel gateway | processo figlio con `nice 10`, scaricato dopo 10 min | decisione 4: CPU separata dal loop del gateway |
| 2 | audio ≤ 60 s | ≤ 30 s (`transcription.max_duration_s`), ≤ 2 MB | comandi brevi; meno CPU sull'Orin |
| 3 | lock dei piani su file, condiviso col bot | lock in-process (`threading.Lock`) | il bot non lo prenderebbe comunque; **il limite resta** (§8) |
| 4 | 0.4 m/s letto da `stack_params.yaml` a runtime | costante `MIN_MOVING_SPEED_MPS = 0.4` + test di accoppiamento col file | nessuna dipendenza da un file a runtime; il test fallisce se cambia |
| 5 | input fuori da [−1, 1] → zero e revoca | non finito → zero e disarmo; fuori range → clamp | un valore 1.02 da un tocco al bordo non deve revocare la guida |
| 6 | sottoscrizioni pesanti solo con Telemetria aperta | solo con almeno un client connesso | più semplice; il costo in CPU sull'Orin è *non verificato* |
| 7 | — | `/api/session` (cookie) | `<video>` e i link al report non possono mandare il bearer |

---

## 5. Sicurezza

Risultati: **ok** = test eseguito e passato. *Non verificato* = mai provato su
macchina o iPhone.

### SPEC §5

| # | Requisito | Dove | Test | Esito |
|---|---|---|---|---|
| 1a | Watchdog: nessun teleop valido da 250 ms → zero | `teleop.py:204` (in `tick`, timer a 50 Hz in `ros_bridge.py:155`); `watchdog_s` `teleop.py:54` | `test_teleop.py:131` `test_the_watchdog_zeroes_250_ms_after_the_last_input`; `test_app_mock.py:324` `test_the_watchdog_on_the_wire`; rclpy reale: `test_ros_bridge.py:67` | ok (mock + Humble) |
| 1b | Chiusura WS → zero | `app.py:882` → `drop()` `app.py:548` → `teleop.release` + publish | `test_teleop.py:178`; `test_app_mock.py:303`; e2e `guida.spec.ts` "closing the app while driving: the watchdog stops the car" | ok |
| 1c | Dopo uno zero: 0.5 s di zeri, poi silenzio (la lane `/teleop` scade dopo 0.2 s, `mux.yaml`) | `teleop.py:212-216` | `test_teleop.py:157` | ok |
| 1d | Dopo uno scatto del watchdog bisogna alzare il pollice | `teleop.py:183` | `test_teleop.py:140` | ok |
| 2 | Rilascio del gas → zero immediato | client: `Guida.tsx:81` → `release()` `Guida.tsx:68` (`deadman:false`); anche `pointercancel` `Joystick.tsx:61`, `visibilitychange`/`pagehide` `Guida.tsx:59-60`, uscita `Guida.tsx:44-45`; server: `app.py:889` pubblica nello stesso callback | `test_teleop.py:150`; e2e "gas held … released: a zero at once" (WebKit); multitouch (Chromium) | ok; iPhone *non verificato* |
| 3 | Solo la lane teleop di ackermann_mux | publisher solo `/teleop` (`ros_bridge.py:112`), `frame_id` `base_link/pwa_teleop` (`ros_bridge.py:162`); Guida rifiutata/revocata con stack spento, mux che non ascolta, altri publisher su `/teleop`, E-STOP, missione (`app.py:278`, guardia a 2 Hz `app.py:488`) | `test_ros_bridge.py:67` (rclpy, topic e frame); `test_app_mock.py:343`, `:354`; e2e "no Guida … joystick", "E-STOP while driving revokes the guida" | ok; sul mux vero *non verificato* |
| 4 | Clamp lato server: velocità, sterzo, rate | `HARD_MAX_SPEED_MPS` `teleop.py:39`; limite client 0.4..max `teleop.py:187`; sterzo ±0.30 rad `teleop.py:191`; rampe `teleop.py:229-231`; ≤ 50 input/s `teleop.py:178` | `test_teleop.py:65`, `:77`, `:84`, `:98`, `:108`, `:116`, `:123`; e2e "the limit stepper is clamped to 0.4..1.0 and the server enforces it" | ok |
| 5 | Mai sotto 0.4 m/s; zona morta; niente retromarcia | `speed_for` `teleop.py:90-95`; `MIN_MOVING_SPEED_MPS` `teleop.py:42` | `test_teleop.py:31` (accoppiamento con `stack_params.yaml`), `:42`, `:46`, `:54`; e2e "gas held: 0.4..limit" | ok |
| 6 | Un solo client di guida | `teleop.py:122`; risposta `busy` `app.py:897-906` | `test_teleop.py:172`; e2e "a second phone gets guida occupata" | ok |
| 7 | Niente retry automatici (start, abort, stop processo) | `start_mission` `app.py:405` (una chiamata); abort `app.py` `/api/mission/abort` (una chiamata, errore mostrato); processi: un job, 409 se già in corso | `test_app_mock.py:135` `test_a_process_switch_shows_its_transition_and_never_retries`; e2e switch bloccato durante la transizione | ok |

### Decisioni approvate

| Decisione | Dove | Test | Esito |
|---|---|---|---|
| 2 · SBLOCCA solo da fermo, pressione lunga | `reset_allowed` `app.py:297`, `_update_still` `app.py:310`; UI `ui.tsx:146-156` (HoldButton 2 s) | `test_app_mock.py:251`, `:266`; e2e "EMERGENZA: … unlock only when still", "reset is disabled while the car moves" | ok; `estop-reset` reale *non verificato* |
| 3a · Client perso in missione → ABORT dopo 2 s, solo missioni dell'app | `mission.py:139-145`, `app.py:532` | `test_mission_tracker.py:52`, `:66`, `:74`; `test_app_mock.py:202` | ok (mock); con il BT vero *non verificato* |
| 3b · Wake Lock in missione e in Guida; avviso se manca | `app.tsx:30-48`, `:84-93`; avviso `app.tsx:104`, `Guida.tsx` (`wake-warning`) | e2e "no Screen Wake Lock: a warning while a mission runs", "…Guida says the screen may go off", "with the Wake Lock, no warning" | ok in WebKit; iOS *non verificato* |
| 4 · Trascrizione sull'Orin, CPU, processo `nice`, rifiutata in missione/Guida | `transcription.py`, `transcribe_worker.py`; rifiuti `app.py` `/api/mission/transcribe` | `test_app_mock.py:272` (con `FakeTranscriber`) | regole ok; faster-whisper reale *non verificato* |
| 5 · Guida: tetto 1.0, stepper 0.4–1.0, default 0.6 | `teleop.py:39`, `config.py` `teleop.default_limit_mps`, `Guida.tsx` stepper | `test_teleop.py:65`, `:77`; e2e stepper | ok |
| 6 · Batteria solo V | `ui.tsx` TopBar, `Guida.tsx` | screenshot | ok |
| 7 · Build su linus, `dist/` ignorato, deploy con rsync | `apps/pwa/.gitignore`, `scripts/deploy.sh` | solo il ramo d'errore (host irraggiungibile → exit 1) | deploy reale *non verificato* |
| EMERGENZA: WS e REST insieme; con connessione persa → "NON RAGGIUNGIBILE" | `ui.tsx:97-108`, `:110-114`; `app.py:911` | `test_app_mock.py:217`; e2e EMERGENZA e "connection lost" | ok |
| READY segue il latch | `_live_estop` `app.py:259`, push `app.py:107` | `test_app_mock.py:225`, `:239` (fallisce senza il push: provato) | ok |
| Stop dello stack rifiutato in moto o in missione | `app.py:655` | `test_app_mock.py:147` | ok |

---

## 6. Test

| Suite | Comando | Dove | Risultato |
|---|---|---|---|
| Gateway (venv: fastapi, httpx) | `cd src/f1tenth_gateway && PYTHONPATH=.:../f1tenth_ctl:../f1tenth_telegram python -m pytest -q test` (Jazzy sourced) | linus, Python 3.12 | **72 passed, 3 skipped** (i 3 test rclpy: `f1tenth_messages` non compilato per Jazzy) |
| Gateway, Python di sistema | stesso comando con `python3` | linus | **46 passed, 29 skipped** (26 test app senza fastapi, 3 rclpy); prima del fix `ce84001` la directory sarebbe stata saltata intera |
| Gateway + f1tenth_ctl sotto Humble | `tools/test_gateway_humble.sh` | docker `f1tenth_more:cpu`, Python 3.10.12, pytest 6.2.5 | gateway **75 passed** (incluso rclpy reale); f1tenth_ctl **93 passed, 15 skipped** (process manager: niente `systemd --user` nel container; su linus passano), `estop_with` con rclpy reale incluso |
| f1tenth_ctl | `cd src/f1tenth_ctl && python3 -m pytest -q test` | linus, Jazzy | **108 passed** |
| f1tenth_telegram | `cd src/f1tenth_telegram && python3 -m pytest -q test` | linus | **83 passed, 17 skipped** |
| flake8 / pep257 | dentro la suite del gateway | | ok |
| TypeScript + build | `npm run build` (`tsc --noEmit` + vite) | linus | ok (JS 77 kB, 25 kB gzip) |
| e2e | `F1T_GATEWAY_PYTHON=<venv>/bin/python npm run test:e2e` | WebKit 390×844 (14), WebKit 844×390 (8), Chromium CDP multitouch (1) | **23 passed** |
| Screenshot | `npm run screens` | WebKit | **11 passed**, 16 schermate + 7 confronti |

Il gateway finto simula:
- processi con transizioni;
- check;
- piano → load → countdown → move → fine;
- ABORT ed E-STOP con latch;
- Guida, con pubblicazioni registrate;
- congelamento della connessione;
- archivio di run (fixture).

### Non testato, e perché

- **Macchina vera, stack vero, mux vero**: l'Orin è spenta. Non si sa se
  ackermann_mux dia davvero la lane a `/teleop` e se la tolga quando la coda di
  zeri finisce.
- **iPhone vero**: Playwright usa WebKit su Linux, non Safari iOS. Non provati:
  safe area, multitouch reale, "Aggiungi alla schermata Home", memoria
  separata, Wake Lock in modalità standalone, MediaRecorder (`audio/mp4`),
  permesso del microfono.
- **Trascrizione reale**: faster-whisper non è stato eseguito. Il mock usa
  `FakeTranscriber`; tempi e carico CPU sull'Orin sono ignoti.
- **E-STOP end to end**: `estop_with` è testato con rclpy reale contro
  publisher e servizi finti (Humble), mai contro il BT e il mux veri. Latenza
  ignota.
- **`f1tenth_ctl estop-reset`, `mission-load/start`, planner plan-only** sul
  grafo vero: testati solo nelle loro suite (con mock) e nel round Telegram in
  sim.
- **Snapshot stream del BT**: il nome del servizio e
  `py_trees_ros_interfaces` sull'Orin non sono verificati.
- **`tailscale serve`, certificato, unità systemd, `deploy.sh` riuscito**:
  richiedono l'Orin.
- **Multitouch in WebKit**: Playwright non ha tocchi CDP per WebKit. Il test a
  due pollici gira in Chromium; in WebKit si usano pointer event sintetici.

---

## 7. Schermate (`output/pwa_screens/`)

Le genera `npm run screens` (WebKit, iPhone 13, DPR 3) contro il gateway
finto.

- `app_*.png`: le schermate dell'app.
- `cmp_*.png`: mockup (a sinistra o sopra) e app (a destra o sotto), alla
  stessa scala.

Differenza comune a tutte: il mockup ha 54 px fissi per la barra di stato
dell'iPhone, l'app usa `env(safe-area-inset-top)`, che in Playwright vale 0.
Sull'iPhone l'app scende di conseguenza (*non verificato*).

| Schermata | File | Differenze dal mockup |
|---|---|---|
| Abbinamento | `app_01_pairing.png` | non c'è mockup: titolo, campo token (password), ABBINA attivo da 32 caratteri |
| Cockpit READY | `app_02_cockpit_ready.png` | |
| Cockpit NOT READY | `cmp_03_cockpit_not_ready.png` | 4/6 check reali al posto di 9/11; mostra solo i falliti (LiDAR, TF); processi reali (`running · 63 nodi · 00:12:36`, `plan-only`, `fermo`); BATT senza % |
| Missione, piano | `cmp_04_missione_piano.png` | PIANIFICA esplicito; riassunto move · v max · "fino a"; passi dalle move; "avviabile ancora 4:56" (scadenza 5 min) |
| Countdown | `app_05_missione_countdown.png` | "AVVIO TRA n s" (non nel mockup); il toast resta sopra il titolo |
| In corso | `cmp_06_missione_in_corso.png` | PASSO 1/3 invece di GIRO; v CMD invece di ERR. LAT.; barra = move finite (0% durante la prima); pill READY invece di ONLINE |
| Completata | `app_07_missione_completata.png` | non c'è mockup: COMPLETATA, 3/3, "fatto" per ogni move, NUOVO COMANDO |
| Interrotta | `app_08_missione_interrotta.png` | non c'è mockup: INTERROTTA in ambra, PASSO –/3 |
| Telemetria | `cmp_09_telemetria.png` | mappa SLAM (finta) al posto della pista; orizzonte MPC tratteggiato al posto della raceline; BT reale (root, mission, navigation…) centrato sul nodo attivo e scorrevole |
| Run | `cmp_10_run.png` | ora + `mission_id`; esiti OK / ABORT / E-STOP / INCOMPLETA; Video disattivato con motivo, ripetuto su ogni card (prolisso) |
| Guida verticale | `app_11_guida_verticale.png` | non c'è mockup: "Ruota il telefono" |
| Guida, rilasciato | `cmp_12_guida_rilasciato.png` | limite 0.6 (default) invece di 2.0; scala 0..limite; BATT senza % |
| Guida, attivo | `app_13_guida_attivo.png` | gas e sterzo insieme, anello verde, DEAD-MAN ATTIVO, 0.59 m/s |
| Guida non disponibile | `app_14_guida_non_disponibile.png` | non c'è mockup: motivo ("un altro nodo pubblica su /teleop") + RIPROVA; joystick spenti |
| Connessione persa | `cmp_15_connessione_persa.png` | testo con le regole vere (§4.8); gli switch mostrano l'ultimo stato attenuato invece di tutti grigi |
| E-STOP attivo | `app_16_estop_attivo.png` | non c'è mockup: banner E-STOP ATTIVO con SBLOCCA (pressione di 2 s); READY → NOT READY con la voce E-STOP |

---

## 8. Limiti noti e TODO

**P1: prima della prima prova.**
- Tutto il §11 ("Bloccato dall'Orin").
- Verificare sulla macchina che una pubblicazione su `/teleop` vinca su `/drive`
  e perda contro `/safety_stop`. Verificare anche che il mux rilasci la lane
  dopo la coda di zeri: con `ros2 topic echo /ackermann_drive --no-daemon`
  guardare `frame_id`.
- Verificare `/behavior_executor_node/snapshot_streams/open` e
  `py_trees_ros_interfaces`. Senza, il BT resta vuoto: non è un problema di
  sicurezza.

**P2.**
- **Piani concorrenti bot + PWA**: il lock è solo nel gateway. Due piani nello
  stesso secondo da Telegram e dall'app possono scambiarsi il RESULT. Va
  evitato a mano.
- Carico CPU della trascrizione sull'Orin con lo stack acceso: non misurato. È
  vietata in missione e in Guida.
- Le sottoscrizioni pesanti (odom, mappa, solver, BT) sono attive con qualsiasi
  client connesso, non solo in Telemetria. Il costo non è misurato.
- iOS: safe area, Wake Lock standalone, MediaRecorder, multitouch reale (§6).
- La durata dell'audio non è limitata sul client: oltre 30 s risponde il
  server.

**P3.**
- Si vede solo l'ultimo toast; gli eventi precedenti restano nel log del
  gateway (`journalctl`).
- Barra di avanzamento per move finite (0% durante la prima move).
- In Run il motivo "nessun video" è ripetuto su ogni card.
- La pill dice READY durante una missione (il mockup dice ONLINE).
- Percentuale della batteria (decisione 6: per ora no).
- Nessun test automatico di `deploy.sh` riuscito, dell'unità systemd o di
  `tailscale serve`.

---

## 9. Deploy e installazione sull'iPhone

I dettagli sono in `src/f1tenth_gateway/README.md`. In ordine:

1. **Orin, codice.** Dopo il push di vesc e il fast-forward di campaign-final
   (§11):
   ```bash
   cd ~/dev_ws/f1tenth_more && git fetch origin && git checkout pwa-control
   git submodule update
   ```
   Poi ricompila: `colcon build --symlink-install`. `pwa-control` ha lo stack di
   cf-next, quindi vanno ricompilati i pacchetti cambiati rispetto a ciò che
   girava.
2. **Porta.** `ss -ltn | grep -E ':8770\b'` deve essere vuoto. Altrimenti
   cambia `listen.port` in `src/f1tenth_gateway/config/gateway.yaml`.
3. **Venv.**
   ```bash
   python3 -m venv --system-site-packages ~/.local/share/f1tenth_gateway/venv
   ~/.local/share/f1tenth_gateway/venv/bin/pip install -r src/f1tenth_gateway/requirements.txt
   ```
   Poi, per la voce: `pip install faster-whisper==1.2.1 'av<17'` e scarica il
   modello `small`.
4. **Token.** Scrivi `~/.config/f1tenth_gateway/env` (`F1T_GATEWAY_TOKEN=…`,
   almeno 32 caratteri) con `chmod 600`. Il comando è nel README, §3.
5. **systemd.**
   ```bash
   cp src/f1tenth_gateway/systemd/f1tenth-gateway.service ~/.config/systemd/user/
   sudo loginctl enable-linger fabiocar
   systemctl --user daemon-reload && systemctl --user enable --now f1tenth-gateway
   ```
6. **PWA, da linus.** `cd apps/pwa && npm ci && npm run deploy`. Fa la build e
   l'rsync in `jetson-orin:.local/share/f1tenth_gateway/www`.
7. **HTTPS.** Nella console di Tailscale: DNS → MagicDNS + HTTPS Certificates.
   Sull'Orin: `sudo tailscale serve --bg --https=443 http://127.0.0.1:8770`.
   Verifica: `curl https://jetson-orin.<tailnet>.ts.net/api/status` → 401.
8. **iPhone.**
   1. Tailscale acceso.
   2. Safari → `https://jetson-orin.<tailnet>.ts.net`.
   3. Condividi → **Aggiungi alla schermata Home**.
   4. Apri **dall'icona**, incolla il token, premi ABBINA.
   5. Al primo microfono, consenti.

---

## 10. Checklist della prima prova sulla macchina

**Ruote sollevate** (macchina sul supporto) fino al punto 12 compreso. Tieni il
radiocomando in mano. In un terminale sull'Orin:

```bash
set -a; . ~/dev_ws/f1tenth_more/src/f1tenth_ctl/config/ctl.env; set +a
ros2 topic echo /ackermann_drive --no-daemon     # frame_id = chi comanda
```

In un altro: `journalctl --user -u f1tenth-gateway -f`.

| # | Azione | Atteso | Se fallisce, controlla |
|---|---|---|---|
| 1 | `systemctl --user status f1tenth-gateway` | `active (running)`; nel journal `Uvicorn running on http://127.0.0.1:8770` | env/token (README §3), porta (`ss -ltnp`), venv |
| 2 | iPhone: apri l'app dall'icona e abbina | pill ONLINE, LAT < 150 ms | Tailscale sull'iPhone, `tailscale serve status`, certificato |
| 3 | Cockpit con stack spento | NOT READY, voce "stack"; switch Stack spento | `f1tenth_ctl status --json` dà lo stesso? |
| 4 | Switch Stack → acceso | "in avvio…", switch bloccato, poi `running · N nodi`; READY quando i check passano | journal; `f1tenth_ctl status`; Discovery Server |
| 5 | **EMERGENZA** (ferma) | banner E-STOP ATTIVO in < 1 s; READY → NOT READY "E-STOP"; toast "E-STOP: auto ferma"; zeri su `/safety_stop` | journal: `path` gateway o f1tenth_ctl; `ros2 service list --no-daemon \| grep emergency` |
| 6 | SBLOCCA (pressione di 2 s) con le ruote ferme | dopo 1 s fermo il pulsante si abilita; il banner sparisce; il BT riparte | `reset_allowed.why` nel banner; `/odom` arriva? `f1tenth_ctl estop-reset` a mano |
| 7 | Guida: ruota il telefono | DEAD-MAN RILASCIATO; nessun avviso schermo (se c'è: niente Wake Lock) | motivo nel riquadro ambra (stack, mux, joystick, E-STOP, missione) |
| 8 | Pollice sul gas | `/ackermann_drive` `frame_id: base_link/pwa_teleop`; le ruote girano, almeno 0.4 m/s, fino al limite 0.6 | mux: `ros2 topic info /teleop --no-daemon` (1 subscriber); `frame_id` diverso = un'altra lane vince |
| 9 | Solleva il pollice | zero subito, ruote ferme; dopo la coda di zeri (0.5 s) e il timeout della lane (0.2 s, `mux.yaml`) `frame_id` torna alla navigazione | journal; `test_teleop` passa? |
| 10 | Sterzo da solo, poi con il gas | senza gas niente moto; con il gas sterzo ±17° (0.30 rad), servo nel range | il segno: destra = negativo in ROS |
| 11 | **Watchdog**: gas premuto, chiudi l'app (swipe su) o modalità aereo | ruote ferme entro ~0.3 s; journal: lease rilasciata / client perso | se gira oltre: STOP col radiocomando; journal `watchdog_trips` |
| 12 | Secondo iPhone in Guida | "guida occupata" | |
| 13 | Missione a ruote sollevate: "vai dritto per un metro", PIANIFICA | VALIDATO, 1 move, v max ≤ 0.5 | planner plan-only? (`f1tenth_ctl status`: planner `plan-only`) |
| 14 | Slide (mezza slide: niente; intera: avvio) | AVVIO TRA 15 s → IN CORSO, PASSO 1/1; poi COMPLETATA | `/mission/status`; rifiuti nel toast |
| 15 | Missione in corso → ABORT (1 s) | INTERROTTA; nessun latch E-STOP | `/mission/abort_mission` |
| 16 | **Connessione persa in missione**: avvia dall'app, poi modalità aereo sull'iPhone | dopo ~2 s il journal mostra "missione interrotta (abort): nessun client"; al ritorno l'app mostra INTERROTTA | `mission.auto_abort_s`; la missione era dell'app? (quelle da Telegram non vengono interrotte, per scelta) |
| 17 | EMERGENZA durante una missione | latch, missione ferma, banner | come il punto 5 |
| 18 | Microfono (tieni premuto, parla, rilascia) | testo nel campo; tempo e CPU da annotare | faster-whisper, modello scaricato, journal; rifiutato in missione/Guida (giusto) |
| 19 | Telemetria | mappa, percorso, orizzonte tratteggiato, BT colorato | `py_trees_ros_interfaces`; `/slam/map` |
| 20 | Run | l'ultima run c'è; Report si apre | `~/f1tenth_archive`; Video solo se c'è un mp4 |
| 21 | Stop dello stack con una missione in corso | rifiutato con motivo | |
| 22 | **A terra**, area libera, radiocomando in mano: Guida a 0.4–0.6 m/s, poi una missione corta | come sopra | al primo dubbio: EMERGENZA **e** radiocomando |

---

## 11. Bloccato dall'Orin

L'Orin è spenta. Senza di lei non si può fare questo, nell'ordine:

1. **Push del submodule vesc, commit `0e74a60`.** `cf-next` (merge `247264e`)
   punta a quel commit, che esiste solo sull'Orin. Va pushato dall'Orin al
   remote di vesc, altrimenti un clone di `cf-next`/`pwa-control` non può fare
   `git submodule update`.
2. **Fast-forward di `campaign-final` a `cf-next`**, solo dopo la tua conferma
   del punto 1. Oggi `campaign-final` resta `cd180d8`, non spostato e non
   pushato.
3. **Porta libera.** `ss -ltn | grep -E ':8770\b'` sull'Orin; se è occupata,
   cambiare `listen.port`.
4. **`py_trees_ros_interfaces`**: c'è sull'Orin? Il servizio
   `/behavior_executor_node/snapshot_streams/open` esiste sul nodo vero?
5. **`tailscale serve` e certificato**: HTTPS Certificates nel tailnet, `serve`
   su 443 → 127.0.0.1:8770, `curl` → 401.
6. **Deploy.** Checkout `pwa-control` più la build dei pacchetti cambiati, poi
   venv, requirements, faster-whisper e modello, token, unità systemd con
   linger, `npm run deploy` da linus.
7. **Prove sulla macchina**: la checklist del §10, a ruote sollevate.
