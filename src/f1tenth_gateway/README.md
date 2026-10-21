# f1tenth_gateway

Il gateway della PWA **F1Tenth Lab Control** (`apps/pwa`): REST + WebSocket sull'Orin,
tra l'iPhone e il grafo ROS della macchina.

```
iPhone (Safari, app sulla Home)
   │  https://jetson-orin.<tailnet>.ts.net   (Tailscale, certificato del tailnet)
   ▼
tailscale serve :443 ──▶ 127.0.0.1:8770  f1tenth_gateway (uvicorn, unità systemd utente)
                              ├─ rclpy, un nodo: /teleop, /safety_stop, /mission/*, telemetria
                              ├─ f1tenth_ctl in-process: status / start / stop / estop-reset / planner
                              └─ trascrizione: un processo a parte (nice 10), faster-whisper CPU
```

> **L'EMERGENZA dell'app non è l'arresto di emergenza primario.** Passa da Wi-Fi,
> Tailscale e il gateway. Quando la macchina può muoversi, tieni in mano il
> radiocomando o l'interruttore fisico.

## Cosa fa il gateway

- **Stato**: READY e i check di `f1tenth_ctl`, i processi (stack, planner,
  logger), la missione da `/mission/status` e `/mission/move_outcome`. La
  telemetria arriva a 10 Hz: posa, velocità misurata e comandata, sorgente del
  comando, batteria, VESC, CPU. Ci sono anche la mappa SLAM, la traiettoria, il
  BT e l'orizzonte MPC.
- **Carico dell'Orin** (`sysstats.py`, nel messaggio di stato, campo `system`):
  CPU totale e per core (`/proc/stat`, core online/offline), GPU % e frequenza,
  i processi vincolati a dei core (`Cpus_allowed_list`, cioè i `taskset -c` dei
  launch) con il loro CPU %. Letto dal gateway stesso, a 1 Hz e solo con un
  telefono collegato; i processi ogni 5 s. Niente jtop, tegrastats o sudo: solo
  `/proc` e sysfs. GPU sull'Orin (L4T R36):
  `/sys/devices/platform/bus@0/17000000.gpu/load` (per mille) e
  `/sys/class/devfreq/17000000.gpu/cur_freq`; su linus `nvidia-smi`, altrimenti n/a.
- **Missioni**:
  - il comando (scritto o dettato) va al planner `plan-only` di `f1tenth_ctl`;
  - il piano validato è avviabile per 5 minuti;
  - lo start passa da `f1tenth_ctl mission-load` e `mission-start`, che
    ricontrollano READY ed E-STOP;
  - l'ABORT passa da `/mission/abort_mission`.
- **EMERGENZA**: zero su `/safety_stop` (la lane a priorità 200 di
  `ackermann_mux`) e `/mission/emergency_stop`. Usa il nodo del gateway e i
  client già pronti; se fallisce, ripiega su `f1tenth_ctl estop`.
  - **SBLOCCA** fa `f1tenth_ctl estop-reset`, ma solo con la macchina ferma
    (velocità misurata 0 da 1 s).
- **Guida**: `/teleop` (lane 100 di `ackermann_mux`, `frame_id`
  `base_link/pwa_teleop`).
  - Un solo telefono alla volta.
  - Watchdog di 250 ms, poi zero.
  - Velocità 0, oppure tra 0.4 m/s e il limite (massimo 1.0 m/s) in avanti.
  - Retromarcia: gas sotto la zona morta, lineare da 0 a
    `teleop.max_reverse_speed_mps` (0.4 m/s, come l'MPC; 0 la disabilita),
    senza il minimo di 0.4. Il cambio di verso passa per lo zero:
    almeno `reverse_hold_s` (0.3 s) e finché l'odometria dice che l'auto
    si muove ancora nel verso opposto (`stop_speed_mps`, 0.05 m/s).
  - Sterzo al massimo 0.30 rad, con rate limit.
  - Non sostituisce né aggira `/safety_stop`, l'emergenza del BT o lo stop
    LiDAR.
- **Run**: elenco da `~/f1tenth_archive`, report HTML, video (con range) quando
  c'è.

Regole lato server (non le decide l'app):

- La Guida è rifiutata se:
  - lo stack è spento;
  - `ackermann_mux` non ascolta `/teleop`;
  - un altro nodo pubblica su `/teleop` (joystick);
  - una missione è in corso;
  - l'E-STOP è attivo.
- La missione è rifiutata se:
  - la macchina è NOT READY;
  - l'E-STOP è attivo;
  - il piano è scaduto;
  - c'è già una missione;
  - la Guida è aperta.
- Una missione avviata dall'app viene interrotta (ABORT, non E-STOP) se per 2 s
  nessun client è connesso. Le missioni avviate da altri (Telegram, shell) non
  vengono toccate.
- Lo stop dello stack è rifiutato se la macchina si muove o c'è una missione.
- La trascrizione è rifiutata durante una missione o con la Guida aperta.
- Nessuna azione critica viene ritentata in automatico.

## Installazione sull'Orin (una volta)

Il gateway gira dal checkout, senza `colcon build`. Il checkout dell'Orin deve
essere sul branch con l'app (`pwa-control` = campaign-final + app). Comandi da
`~/dev_ws/f1tenth_more`.

### 1. Porta

```bash
ss -ltn | grep -E ':8770\b' || echo "8770 libera"
```

Se è occupata, cambia `listen.port` in `config/gateway.yaml` e usa la stessa
porta per `tailscale serve` (passo 6). Non usare 8765 (foxglove_bridge) né 8083
(llama-server). Per una prova va bene anche `--port`.

### 2. Venv e pacchetti

`--system-site-packages` serve perché il gateway usa rclpy, yaml e numpy di ROS
Humble.

```bash
python3 -m venv --system-site-packages ~/.local/share/f1tenth_gateway/venv
~/.local/share/f1tenth_gateway/venv/bin/pip install -r src/f1tenth_gateway/requirements.txt
# comandi vocali (facoltativo): CPU int8
~/.local/share/f1tenth_gateway/venv/bin/pip install faster-whisper==1.2.1 'av<17'
# il modello si scarica la prima volta: meglio farlo ora, con la rete
~/.local/share/f1tenth_gateway/venv/bin/python -c \
  "from faster_whisper import WhisperModel; WhisperModel('small', device='cpu', compute_type='int8')"
```

`transcription.model` è `small` (`base` è più veloce ma peggiore in italiano).
Senza faster-whisper il gateway funziona lo stesso: il microfono risponde con
un errore e il comando si scrive a mano.

Se `/behavior_executor_node/snapshot_streams/open` esiste, il BT in Telemetria
richiede `py_trees_ros_interfaces`:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -c "import py_trees_ros_interfaces.srv" && echo ok
```

Senza, la scheda BT resta vuota e il resto funziona.

### 3. Token

Il token sta solo nel file d'ambiente, mai nel repo, in chat o su Telegram:

```bash
mkdir -p ~/.config/f1tenth_gateway && chmod 700 ~/.config/f1tenth_gateway
umask 077
python3 -c 'import secrets; print("F1T_GATEWAY_TOKEN=" + secrets.token_urlsafe(32))' \
  > ~/.config/f1tenth_gateway/env
chmod 600 ~/.config/f1tenth_gateway/env
```

Il token deve avere almeno 32 caratteri, altrimenti il gateway non parte.

Per portarlo sull'iPhone usa un canale privato (gestore di password, AirDrop da
un Mac), poi cancellalo da lì.

Per cambiarlo: riscrivi il file e riavvia il gateway. I telefoni tornano alla
schermata di abbinamento.

### 4. Unità systemd (utente)

```bash
mkdir -p ~/.config/systemd/user ~/.local/state/f1tenth_gateway
cp src/f1tenth_gateway/systemd/f1tenth-gateway.service ~/.config/systemd/user/
sudo loginctl enable-linger fabiocar     # le unità utente partono al boot, senza login
systemctl --user daemon-reload
systemctl --user enable --now f1tenth-gateway
tail -F ~/.local/state/f1tenth_gateway/gateway.log
```

`~/.local/state/f1tenth_gateway` va creato **prima**: systemd apre il log anche
per `ExecStartPre`, quindi crearlo da dentro l'unità fallisce con `209/STDOUT`.

Il log va in un **file**, non nel journal: sull'Orin il journal è volatile
(`/run/log/journal`, `root:systemd-journal`) e `fabiocar` non è né in `adm` né
in `systemd-journal`, quindi `journalctl --user -u f1tenth-gateway` risponde
"No journal files were opened due to insufficient permissions" -- un log vuoto
proprio quando serve. Con sudo: `sudo journalctl --user-unit f1tenth-gateway`.

Se il gateway gira da un **worktree** (`git worktree add ~/dev_ws/f1tenth_more_pwa
pwa-control`, per non spostare il checkout principale), nella copia in
`~/.config/systemd/user/` cambia `ExecStart` nel percorso del worktree. Tutto il
resto resta: `ctl.env` del worktree sorgente comunque
`~/dev_ws/f1tenth_more/install/setup.bash`, in sola lettura, che è quello che
serve -- il gateway vede il grafo e i tipi di messaggio dello stack.

`bin/f1tenth_gateway` esegue due passi:

1. Carica `src/f1tenth_ctl/config/ctl.env`, che contiene:
   - ROS Humble;
   - l'install del workspace;
   - `ROS_DISCOVERY_SERVER=127.0.0.1:11811`;
   - `ROS_SUPER_CLIENT=TRUE`.
2. Avvia il venv.

Così il gateway vede lo stesso grafo di `f1tenth_ctl`.

L'unità è fuori dallo stack, che `f1tenth_ctl` avvia e ferma. All'arresto
riceve SIGINT: revoca la Guida e pubblica un ultimo zero su `/teleop`.

### 5. L'app (dal portatile/linus)

```bash
cd apps/pwa
npm ci
npm run deploy        # build + rsync di dist/ in jetson-orin:.local/share/f1tenth_gateway/www
```

`F1T_DEPLOY_HOST` e `F1T_DEPLOY_DIR` cambiano la destinazione. Il gateway non
va riavviato: `index.html` e `sw.js` sono serviti `no-store`.

### 6. HTTPS su Tailscale

Una volta, nella console di amministrazione di Tailscale, in DNS: abilita
**MagicDNS** e **HTTPS Certificates**. Poi sull'Orin:

```bash
sudo tailscale serve --bg --https=443 http://127.0.0.1:8770
tailscale serve status
curl -s -o /dev/null -w '%{http_code}\n' https://jetson-orin.<tailnet>.ts.net/api/status   # 401
```

Il certificato è quello del tailnet. Il gateway resta su 127.0.0.1: dalla LAN
non si raggiunge.

Con una versione di `tailscale` più vecchia la sintassi di `serve` è diversa:
vedi `tailscale serve --help`.

### 7. Il QR di abbinamento

```bash
~/.local/share/f1tenth_gateway/venv/bin/python src/f1tenth_gateway/tools/pairing_qr.py
```

Stampa nel terminale il QR di `https://<orin>.<tailnet>.ts.net/#token=<token>`.
L'app legge il token dal **frammento** (`apps/pwa/src/store.ts`,
`tokenFromHash`), lo salva e lo toglie subito dalla barra degli indirizzi con
`history.replaceState`.

Il frammento, non la query string: un frammento non viene inviato con la
richiesta, quindi il token non finisce nel log del gateway, in quello di
tailscale, in un proxy di mezzo o in un `Referer`.

Lo script non stampa il token né il link completo (`--show-url` lo fa, in un
terminale di cui ti fidi). Usa `qrencode -t ansiutf8` se c'è, altrimenti il
pacchetto `qrcode` del venv (Python puro).

**Il QR abbina Safari, non l'app sulla Home.** `start_url` del manifest è
`/#/cockpit`, e un'app aperta dall'icona ha una memoria separata: il frammento
non ci arriva. Il QR serve per provare il giro completo in Safari senza
digitare 43 caratteri; nell'app dall'icona il token va incollato una volta
(gestore di password o AirDrop, come sotto).

## iPhone

1. L'app **Tailscale** deve essere attiva e connessa allo stesso tailnet.
2. Apri `https://jetson-orin.<tailnet>.ts.net` in **Safari**, o inquadra il QR
   del passo 7: in Safari l'abbinamento è automatico.
3. Scegli Condividi → **Aggiungi alla schermata Home**.
4. Apri l'app **dall'icona**: sulla Home ha una memoria separata da Safari, quindi
   l'abbinamento va fatto lì.
5. Incolla il token e premi ABBINA.
6. Al primo uso del microfono (tieni premuto), concedi il permesso.

La Guida funziona solo in orizzontale. Durante una missione e in Guida l'app
chiede di tenere lo schermo acceso (Screen Wake Lock). Se il browser non lo
permette mostra un avviso: in quel caso tieni l'app in primo piano.

## Configurazione

- `config/gateway.yaml` contiene solo ciò che cambia sull'auto.
- Tutte le chiavi e i loro default sono in `f1tenth_gateway/config.py`.
- Una chiave sconosciuta è un errore, così un refuso non viene ignorato.
- `--host` e `--port` sovrascrivono `listen`.
- `--mock` usa `config/gateway_mock.yaml`.

Chiavi utili:

| Chiave | Default | Note |
|---|---|---|
| `listen.port` | 8770 | la stessa di `tailscale serve` |
| `teleop.max_speed_mps` | 1.0 | mai sopra 1.0 (limite nel codice) |
| `teleop.default_limit_mps` | 0.6 | limite iniziale in Guida (0.4..1.0 a passi di 0.1) |
| `teleop.watchdog_s` | 0.25 | nessun input valido per questo tempo → zero |
| `mission.auto_abort_s` | 2.0 | missione avviata dall'app senza client → ABORT |
| `estop.reset_still_s` | 1.0 | macchina ferma da questo tempo per SBLOCCA |
| `client.lost_after_s` | 1.5 | nessun messaggio dal gateway → "connessione persa" |
| `transcription.model` | small | `base` più veloce |

## Su linus: il gateway con la macchina finta

Senza ROS e senza macchina: `--mock` simula processi, check, missioni, Guida,
EMERGENZA e archivio.

```bash
python3 -m venv /tmp/gwvenv && /tmp/gwvenv/bin/pip install -r src/f1tenth_gateway/requirements.txt httpx==0.28.1 pytest
cd apps/pwa && npm ci && npm run build && cd ../..
F1T_GATEWAY_VENV=/tmp/gwvenv \
F1T_GATEWAY_TOKEN=$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))') \
  src/f1tenth_gateway/bin/f1tenth_gateway --mock      # http://127.0.0.1:8770
```

In sviluppo, `npm run dev` (in `apps/pwa`) inoltra `/api` e `/ws` a
127.0.0.1:8770.

## Test

```bash
# gateway (venv con fastapi + httpx), con flake8/pep257
source /opt/ros/jazzy/setup.bash
cd src/f1tenth_gateway && PYTHONPATH=.:../f1tenth_ctl:../f1tenth_telegram:$PYTHONPATH \
  /tmp/gwvenv/bin/python -m pytest -q test

# gateway + f1tenth_ctl sotto Humble / Python 3.10 (immagine f1tenth_more:cpu), anche rclpy reale
tools/test_gateway_humble.sh

# app: Playwright contro il gateway finto (WebKit 390x844 e 844x390, Chromium per il multitouch)
cd apps/pwa && npx playwright install webkit chromium
F1T_GATEWAY_PYTHON=/tmp/gwvenv/bin/python npm run test:e2e
F1T_GATEWAY_PYTHON=/tmp/gwvenv/bin/python npm run screens   # → output/pwa_screens/ (prima npm run build)
```

Con il Python di sistema (senza fastapi) i test dell'app vengono saltati uno per
uno; gli altri girano.

## Problemi

| Sintomo | Causa probabile | Verifica |
|---|---|---|
| Il gateway non parte: `token` | `F1T_GATEWAY_TOKEN` assente o < 32 caratteri | `systemctl --user cat f1tenth-gateway`, `~/.config/f1tenth_gateway/env` |
| `address already in use` | porta occupata | `ss -ltnp \| grep 8770`; cambia `listen.port` e `tailscale serve` |
| Safari: sito non raggiungibile | Tailscale spento sull'iPhone, o `serve` non attivo | `tailscale serve status` sull'Orin |
| lamacchinina sempre NOT READY "discovery" | discovery server giù o env diverso | `f1tenth_ctl check`; il gateway carica `ctl.env` |
| Guida: "ackermann_mux non ascolta /teleop" | stack spento o mux senza la lane teleop | `ros2 topic info /teleop --no-daemon` (con `ctl.env`) |
| Guida: "un altro nodo pubblica su /teleop" | joystick acceso | spegni il joy node |
| Microfono: errore | faster-whisper non installato, o modello non scaricato | passo 2; `~/.local/state/f1tenth_gateway/gateway.log` |
| BT vuoto in Telemetria | `py_trees_ros_interfaces` assente, o il nodo del BT non espone gli snapshot stream | passo 2 |
| GPU "n/a" sull'Orin | nessun file `load` della GPU leggibile (altro L4T?) | `cat /sys/devices/platform/bus@0/17000000.gpu/load`; `ls /sys/class/devfreq/`; i percorsi in `sysstats.py` (`GPU_LOAD_PATHS`) |
