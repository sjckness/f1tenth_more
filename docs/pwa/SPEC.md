# F1Tenth Lab Control — PWA iPhone · Specifica v1

Web app mobile (PWA, Safari iOS, "Aggiungi a Home") per controllare la macchina F1Tenth in laboratorio.
Riferimento visivo: cartella `design/` (7 artboard). Sono mockup statici in un formato proprietario
(`<x-dc>`, `<sc-for>`, `{{hole}}`): servono **solo come riferimento di layout, colori, dimensioni e testi**,
non vanno importati né riusati come codice.

---

## 1. Architettura proposta (da confermare in Fase 0)

```
iPhone (PWA, Safari) ──HTTPS/WSS via Tailscale──▶ f1tenth_gateway (Orin)
                                                   ├─ serve la PWA (static build)
                                                   ├─ REST + 1 WebSocket
                                                   ├─ rclpy (Humble): topic/servizi dello stack
                                                   └─ f1tenth_ctl: start/stop processi
```

- **Gateway sull'Orin, fuori dallo stack** (servizio systemd utente), perché deve poter avviare/fermare lo stack
  e perché la guida manuale non può passare da una catena SSH (latenza). Il bot Telegram su linus resta com'è.
- Python: FastAPI + uvicorn + rclpy. Riusare `f1tenth_ctl` e la logica di READY/check del branch telegram-control,
  non riscriverle.
- **HTTPS obbligatorio**: Safari dà accesso al microfono (`getUserMedia`) e al service worker solo in secure context.
  Usare `tailscale serve` / `tailscale cert` sul nome `*.ts.net` dell'Orin.
- Auth: token bearer in variabile d'ambiente (non nel repo), bind solo sull'interfaccia Tailscale.
- Frontend: Vite + TypeScript + React (o Preact), nessuna UI library pesante. Font: Inter + JetBrains Mono.

## 2. Design system

| Token | Valore | Uso |
|---|---|---|
| bg | `#0A0C0F` | sfondo |
| surface | `#13161B` | card |
| surface-2 | `#1A1E25` | pulsanti secondari, riga attiva |
| border | `#262B34` | bordi, divisori |
| text | `#ECEEF1` | testo, dati |
| muted | `#9099A6` | label (contrasto ≥ 4.5:1 su surface) |
| disabled | `#5E6672` | solo elementi disattivi |
| green | `#34D27B` | READY / ok / success |
| amber | `#F5A524` | warning / abort |
| red (testo) | `#FF5A50` | errore / failure |
| e-stop | `#D7261E` | solo pulsante EMERGENZA (testo bianco) |

- Il colore si usa **solo per gli stati**; grafici e dati sono neutri (bianco/grigi).
- Numeri in JetBrains Mono con `font-variant-numeric: tabular-nums`; label MONO maiuscole 11–12 px, letter-spacing 0.1em.
- Target touch ≥ 44 px (comandi principali 56–72 px). Uso con una mano: comandi in basso.
- Safe area iOS: `env(safe-area-inset-*)`, niente status bar finta.

## 3. Struttura

Tab bar in basso con 5 sezioni: **lamacchinina · Missione · Guida · Telemetria · Run**.
Pulsante **EMERGENZA** fisso, sempre visibile sopra la tab bar (in Guida: in alto al centro).

### lamacchinina (`design/Main.dc.html`)
- Il nome visibile è **lamacchinina**, così: minuscolo, una parola (tab, titolo, link "indietro").
  Prima si chiamava Cockpit: route `#/cockpit` e componente `Cockpit.tsx` restano col vecchio nome.
- Barra connessione: ONLINE/OFFLINE, latenza RTT (ms), batteria (V e %).
- Indicatore grande READY / NOT READY (verde/ambra) + `n/m CHECK OK` + lista dei **soli check falliti**
  (icona errore rossa o warning ambra, nome, dettaglio con valore e soglia).
- Tre interruttori di processo: **Stack, Planner, Logger**, con sottotitolo di stato (es. `running · 14 nodi · 00:12:31`).
  Durante la transizione: stato "in avvio…/in arresto…", switch non toccabile.
- **LLM2** (solo branch `two-llm-arch`): sotto i processi, la riga **Parametri MPC** con uno switch che mette
  in pausa / riprende LLM2, il tuner dei parametri dell'MPC (`POST /api/llm2 {active}` → `/llm2/set_active`).
  - Lo switch mostra lo stato che riporta LLM2 (`status.llm2.state`, da `/llm2/status`), **mai l'ultimo
    tocco**: `attivo` (verde, switch acceso) o `in pausa` (grigio, switch spento), più l'ultima latenza
    (`latenza 443 ms`, resta in pausa) e, sotto, i contatori `ok · invalid · timeout`.
  - Nessuna conferma. Durante la chiamata (`pausa in corso…` / `ripresa in corso…` in ambra; da un altro
    telefono `comando in corso…`, `status.llm2.pending`) lo switch non è toccabile e resta sullo stato reale.
  - Errore (timeout, risposta inattesa, LLM2 non attivo): toast rosso col motivo del gateway
    (`Pausa LLM2 fallita: …`), lo switch resta sullo stato reale. Il successo lo annuncia il gateway a
    tutti i telefoni (`LLM2 in pausa (…)`).
  - LLM2 non attivo (`state` `not_running`: nessun `/llm2/status` da 5 s o servizio assente, cioè
    `enable_llm2:=false` o nodo morto): riga grigia "LLM2 non attivo" col motivo (`why`), switch
    disattivato. Lo stesso con un gateway che non manda `status.llm2` (campaign-final).
  - Connessione persa: come i processi (comandi disattivati, stato `?`).
- Riga di numeri: velocità, temperatura VESC, CPU, GPU (% e, sotto, la frequenza; `n/a` se il gateway
  non legge nessuna GPU). Toccare la CPU apre la vista CPU.

### Missione (`design/Missione.dc.html`, `design/MissioneInCorso.dc.html`)
- Input testo + pulsante microfono **hold-to-talk** (MediaRecorder, rilascio = invio → trascrizione → testo nel campo,
  modificabile prima dell'invio).
- Card PIANO: badge VALIDATO / ERRORI, riassunto (giri, v max, arrivo), passi numerati, **JSON espandibile**.
- **Slide-to-start** (trascinare il cursore fino in fondo; un tap non basta). Disabilitato se NOT READY o piano non valido.
- Missione in corso: stato, tempo, avanzamento, passo attivo, velocità reale, errore laterale,
  **ABORT** con pressione prolungata (~1 s, con anello di progresso). Abort ≠ emergenza.

### Guida (`design/Guida.dc.html`) — solo orizzontale
- iOS non permette di bloccare l'orientamento in una PWA: se il telefono è in verticale mostrare "Ruota il telefono".
- Joystick sinistro **verticale = gas**, joystick destro **orizzontale = sterzo**, entrambi tornano al centro al rilascio.
- **Dead-man = pollice sul joystick del gas**: si muove solo finché quel tocco è attivo. Rilascio → stop immediato.
  Il joystick si illumina di verde quando il dead-man è attivo.
- Limite di velocità con stepper (− / +), applicato anche lato server.
- Velocità reale (grande) vs comandata, con barra.
- Pulsante "indietro" a lamacchinina al posto della tab bar. Multi-touch obbligatorio (due pollici insieme).

### Telemetria (`design/Telemetria.dc.html`)
- Traiettoria 2D: pista/raceline tratteggiata, percorso fatto, posa della macchina; x, y, ψ.
- Grafico velocità ultimi 30 s: reale (linea piena) vs comandata (tratteggiata).
- Behavior tree come grafo colorato: success verde, running bianco, idle grigio tratteggiato, failure rosso.
  Il BT è py_trees_ros: usare il topic snapshot del tree (verificare quale in Fase 0).
- In alto CPU e GPU come in lamacchinina; toccare la CPU apre la vista CPU.

### CPU (route `#/cpu`, senza tab)
- Si apre toccando la CPU in lamacchinina o in Telemetria; la freccia "indietro" torna lì
  (aperta da URL: a lamacchinina). La tab bar resta, con evidenziata la vista di partenza.
- Una barra per core con il suo % (grigio sotto 60 %, ambra 60–85 %, rosso da 85 %); core offline
  tratteggiati, senza %. Righe da 4 barre al massimo (12 core = 3 righe), leggibili in verticale.
- Sopra ogni core i chip dei processi vincolati a quel core (un processo su più core compare su
  ognuno), col nome accorciato a un `_` (`ekf_global_filter_node` → `ekf_global`); legenda col
  nome intero, un colore per processo (lo stesso dei chip), i suoi core e il suo CPU %
  (100 = un core, media su 5 s); la riga "N processi non vincolati".
- Colori dei processi: 6 tinte, senza giallo e rosso (sopra le barre sembrerebbero il carico);
  dal 7° processo le stesse tinte col bordo tratteggiato, dal 13° punteggiato. Il colore resta al
  processo (nome e core) finché è vincolato; uno che sparisce libera il suo.
- Thread del kernel esclusi (i per-CPU come `ksoftirqd/N` sono vincolati per natura): non sono né
  tra i vincolati né tra i non vincolati; la nota in fondo ne dice il numero. Esclusi anche i
  processi in uscita (zombie, o senza più riga di comando).
- Dati: `status.system` del gateway (`sysstats.py`): core a 1 Hz, processi ogni 5 s.

### Run (`design/Run.dc.html`)
- Ultime run: id, missione, esito (OK verde / ABORT ambra / E-STOP rosso), data, durata, link **Report** e **Video**.
- Fonte: l'archivio del logger sull'Orin (`~/f1tenth_archive/...`, verificare il path reale). Se report/video non esistono
  per quella run, link disattivato con motivo.

### Stato "Connessione persa" (`design/ConnessionePersa.dc.html`)
- Banner rosso in alto con tempo dall'ultimo messaggio e riconnessione automatica.
- **Tutti i comandi disattivati** (switch, slide-to-start, abort, joystick); dati mostrati come fermi.
- EMERGENZA sostituito da "EMERGENZA NON RAGGIUNGIBILE · arresto affidato al watchdog".
- La navigazione tra le tab resta possibile.

### Stato aggiuntivo: E-STOP attivo
`/mission/emergency_stop` resta latched fino al riavvio di nodo/BT (nessun reset). L'app deve mostrarlo
(banner rosso fisso "E-STOP ATTIVO · riavvia il BT/stack") e bloccare i comandi di missione finché non sparisce.

## 4. API (contratto proposto)

REST (tutte con `Authorization: Bearer <token>`):

| Metodo | Path | Effetto |
|---|---|---|
| GET | `/api/status` | connessione, batteria, ready, checks[], processi, estop_latched, mission_state, `system` (CPU per core, GPU, processi vincolati), `llm2` (solo `two-llm-arch`: `state` active/paused/not_running, `why`, `latency_ms`, `counters`, `pending`) |
| POST | `/api/process/{stack\|planner\|logger}` `{action: start\|stop}` | via f1tenth_ctl |
| POST | `/api/estop` | `/mission/emergency_stop` + zero sul lane teleop |
| POST | `/api/llm2` `{active: true\|false}` | solo `two-llm-arch`: `/llm2/set_active` (pausa / ripresa di LLM2); 409 se LLM2 non è attivo o c'è già una chiamata, 502 su timeout (2 s) o risposta inattesa |
| POST | `/api/mission/transcribe` (audio) | → `{text}` (riusare il trascrittore del bot Telegram) |
| POST | `/api/mission/plan` `{text}` | → `{plan_id, summary, steps[], json, valid, errors[]}` dal planner LLM |
| POST | `/api/mission/start` `{plan_id}` | load_mission + start_mission |
| POST | `/api/mission/abort` | `/mission/abort_mission` |
| GET | `/api/runs?limit=20` | lista run |
| GET | `/api/runs/{id}/report`, `/api/runs/{id}/video` | file statici |

WebSocket `/ws`:
- server → client, 10 Hz: `{t, pose, v_meas, v_cmd, mission_status, battery}`; `bt` solo quando cambia; `path` decimato.
- client → server, 20 Hz solo in Guida con dead-man attivo: `{type:"teleop", seq, thr, steer}` (−1…1).
- ping/pong per la latenza e per il rilevamento "connessione persa" (client: nessun messaggio da 1.5 s → persa).

## 5. Requisiti di sicurezza (non negoziabili)

1. **Watchdog teleop sul gateway**: nessun messaggio teleop valido da 250 ms → comando zero. Chiusura WS → zero.
2. Rilascio del gas sul client → invio immediato di un messaggio zero (non aspettare il watchdog).
3. Il teleop passa dal **lane teleop di ackermann_mux** (verificare topic e priorità): mai bypassare i lane di sicurezza
   (safety_stop, emergenza, stop LiDAR di ultima istanza).
4. Clamp lato server: limite di velocità, sterzo massimo, rate limit.
5. Vincolo della macchina: mai in movimento sotto 0.4 m/s in avanti. Mappatura gas: zona morta (|gas| ≤ `deadzone`,
   0.15) → 0; gas in avanti fuori dalla zona morta → velocità da 0.4 m/s al limite.
   **Retromarcia** (gas sotto −`deadzone`): velocità negativa, lineare da 0 al bordo della zona morta fino a
   −`max_reverse_speed_mps` (parametro del gateway, default 0.4 m/s, uguale a `max_reverse_speed_mps` dell'MPC in
   stack_params.yaml; mai oltre il limite del conducente; 0 = retromarcia disabilitata come nella v1). Il minimo di
   0.4 m/s vale solo in avanti. Ogni cambio di verso passa per lo zero: comando 0 per almeno `reverse_hold_s`
   (default 0.3 s) e finché la velocità misurata (odometria, se più recente di 0.5 s) supera `stop_speed_mps`
   (default 0.05 m/s) nel verso opposto; senza misura fresca decide solo il tempo. Dead-man, watchdog 250 ms e
   rilascio = zero immediato sono gli stessi della marcia avanti. In Guida, finché il comando è negativo, un
   indicatore RETROMARCIA (etichetta e colore ambra) è sempre visibile, con "attesa arresto" durante lo zero
   del cambio di verso. Il joystick del gas mappa la posizione assoluta del pollice: con la retromarcia
   abilitata la metà inferiore non è più una zona di riposo (nella v1 dava 0), e le estremità del joystick
   sono etichettate AVANTI e RETRO (ambra). Attenzione: dietro l'auto non c'è sensore oltre la finestra
   del LiDAR.
6. Un solo client di guida alla volta (il secondo riceve "guida occupata").
7. Le azioni critiche (start missione, abort, process stop) non hanno retry automatici.

## 6. Domande aperte (proporre un default in Fase 0)

- Perdita del client **durante una missione autonoma**: fermare la missione (default proposto: sì, dopo 2 s) o lasciarla finire?
- Dove si trova oggi la lista dei check di READY (bot Telegram, f1tenth_ctl, nodo dedicato?).
- Interfaccia reale del planner LLM (servizio? topic?) per ottenere il piano senza avviarlo.
- Dati pista/raceline per la mappa 2D: da quale topic/file.
