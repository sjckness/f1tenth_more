---
id: 2026-11-23_office-door-batch
title: "Office HIL: 35 pose x 3 planner x 3 ripetizioni, rettilineo 3 m e porta a sinistra"
area: simulation
date: 2026-11-23
tags: [isaac, hil, orin, office-world, enter-door, planner-modes, batch]
modules: {f1tenth_behavior: "0.2.0", f1tenth_planners: "0.1.0", mpc_controller: "0.1.0", f1tenth_costmap: "0.1.0", f1tenth_bringup: "0.4.0", f1tenth_params: "0.2.0", f1tenth_navigation: "0.1.0", f1tenth_perception: "0.1.0", f1tenth_localization: "0.1.0", f1tenth_messages: "0.1.0"}
branch: "hil-office-batch"
commit: "29e6c2a16d6c4301340529b19d3d959de8968a7b"
status: open
hypothesis: "Su tutte le pose fattibili dell'office (find_door_runup.py), i tre planner_controller_mode eseguono rettilineo + enter_door left con lo stesso successo?"
result: "5/315 run riusciti; corridor_mpc 1/105, corridor_rrt 2/105, p2p_geometric 2/105 (IC 95 % 0-5 / 1-7 / 1-7 %, sovrapposti). Il batch non misura i planner: 80/315 run non partono (infrastruttura) e dei 235 volati 212 si fermano in enter_door, comune ai tre modi. 18 dei 23 COMPLETE sono falsi positivi."
decision: "Il confronto fra planner non e' concludente e va rifatto dopo le correzioni elencate nei prossimi passi (PhysX/NaN allo spawn, verdetto di enter_door, move id nell'analizzatore). Nessun planner viene scelto o scartato su questi dati."
raw_data: "Orin: ~/f1tenth_hil_batch/20261123_175245_office_door_left_5m/ (runs/*.csv per run, logs/, figures/). Work: scripts\\sim\\win\\batch\\20261123_175245_office_door_left_5m\\ (batch.log 1.3 MB, isaac_pNN_*.log, report.md generato). Tabella per run derivata dal batch.log in data/runs_from_batch_log.csv."
---
# Office HIL batch 20261123_175245_office_door_left_5m

Sessione del 2026-11-23, 17:52-23:06 (5.22 h), 315 run portati a termine senza interruzioni.

Questo report e' l'analisi della sessione. Il `report.md` generato da `hil_agent.py analyze` (copia sulla
work) contiene le tabelle, ma lasciava `Conclusioni: n/d (da scrivere dopo aver guardato i fallimenti)`.
Le conclusioni qui sotto vengono da `batch.log`, che registra l'esito di ogni run, e dai log di Isaac.

## Setup

- Stack: immagine `f1t-1992797` (commit 19927972be28), container HIL sull'Orin, `sim:=isaac`,
  `enable_intelligence:=false` (missione da file, niente LLM).
- Sim: Isaac sulla work, mondo office, layout `door_left`, `scene.car.start` sovrascritto per posa.
- Missione: `straight_3m_then_left_door_direct.json` (rettilineo 3 m a 0.4 m/s, poi `enter_door` left).
- Pose: 35 da `docs/analysis/simulation/2026-11-13_office-feasible-tests/data/door_runup_left_5m.json`;
  modi `corridor_mpc`, `corridor_rrt`, `p2p_geometric`; 3 ripetizioni = 315 run.

Le versioni in `modules` sono quelle che l'immagine ha dichiarato (`versions.json` sull'Orin) e due non
si vedono da `main`, quindi `tools/core/analysis/index.py --check` le segnala: `f1tenth_bringup` 0.4.0
esiste su `hil-office-batch` (su `main` e' 0.1.0), mentre `f1tenth_behavior` 0.2.0 non e' in nessun branch
di questo clone - il commit dell'immagine (19927972be28) non e' raggiungibile da qui, quindi il valore
resta come registrato e non verificato.

## Metodo

Per posa Isaac viene riavviato con la posa; per modo lo stack parte da zero; per ripetizione `/reset_all`
piu' i controlli di `restart.sh` (tolleranze office 5 cm / 1.5 deg), load + start della missione,
registrazione fino a COMPLETE/ABORTED + 3 s. **Successo** = missione COMPLETE **e**, dalla ground truth di
Isaac, l'auto finisce nel varco vero (|along| <= 0.3 m, |lateral| <= w/2 - 0.186 m, yaw entro 20 deg)
oppure lo ha attraversato dentro l'apertura.

Rifare l'analisi sull'Orin: `python3 scripts/car/orin/hil_agent.py analyze --session <dir>`.
I conteggi di questo report si rifanno dal solo log della work, senza Orin:

    python3 data/parse_batch_log.py <sessione>/batch.log data/runs_from_batch_log.csv

`data/runs_from_batch_log.csv` ha 315 righe (posa, modo, ripetizione, stato, motivo del reset fallito,
esito del rettilineo e della porta), ricavate dalle righe `[agent ...]` di `batch.log`. I totali
coincidono con quelli del report generato dall'Orin.

## Risultati

| modo | run | successo | IC 95 % | COMPLETE | nel varco (GT) | attraversato | durata mediana [s] | CPU ctrl [ms/s sim] |
|---|---|---|---|---|---|---|---|---|
| corridor_mpc | 105 | 1 (1.0 %) | 0-5 % | 7 | 0 | 3 | 54.65 | 863.5 |
| corridor_rrt | 105 | 2 (1.9 %) | 1-7 % | 10 | 0 | 2 | 17.78 | 262.3 |
| p2p_geometric | 105 | 2 (1.9 %) | 1-7 % | 6 | 0 | 2 | 19.47 | 256.5 |

Gli intervalli di confidenza si sovrappongono tutti: su questi dati i tre modi non sono distinguibili.
La CPU e' la mediana su tutti e 105 i run del modo, quindi confrontabile. La **durata mediana no**: e'
calcolata sui soli run riusciti, quindi i 54.65 s di `corridor_mpc` sono un run singolo.

I 5 successi stanno su 2 pose sole: **posa 15** (mpc 1/3, rrt 1/3, p2p 2/3) e **posa 31** (rrt 1/3).
Le altre 33 pose fanno 0/9. Quello che cambia l'esito e' la posa, non il planner.

### Dove sono finiti i 315 run

| | run | quota |
|---|---|---|
| non hanno mai volato (infrastruttura) | 80 | 25 % |
| fermati nel rettilineo 3 m (`timeout:abort` a 30 s) | 18 | 6 % |
| arrivati a `enter_door` e fermati la' | 194 | 62 % |
| `enter_door` dichiara la porta presa (COMPLETE) | 23 | 7 % |
| di cui davvero nel varco o attraverso (successo) | 5 | 1.6 % |

Gli 80 run persi prima di volare:

| causa | run | pose |
|---|---|---|
| `reset_failed`, velocita' NaN | 54 | 5, 6, 7, 11, 17, 27 (9/9 ciascuna) |
| `reset_failed`, check `speed` falso | 9 | 3 (9/9) |
| `isaac_failed` (3 lanci di Isaac, nessuno su) | 9 | 35 |
| `stack_not_ready` (240 s x 2 tentativi) | 6 | 34, su `corridor_rrt` e `p2p_geometric` |
| `reset_failed`, `/sim/reset` assente | 1 | 31 |
| `start_failed` | 1 | 30 |

Esito dei 235 run che hanno volato, nella fase porta:

| esito di `move_1_enter_left_door` | run |
|---|---|
| `door_needs_steered_reverse` | 96 |
| `no_strong_door` | 62 |
| `stop_condition:door_entered` (COMPLETE) | 23 |
| `door_not_found` | 20 |
| `door_entry_failed` | 11 |
| `timeout:abort` | 5 |
| mai arrivati alla fase (rettilineo in timeout) | 18 |

Le figure (`figures/success_by_mode.png`, `figures/pose_NN.png`) e i CSV per run restano sull'Orin: da
questa macchina l'Orin non e' raggiungibile (`ssh`: host key non accettata), quindi non sono in git.

## Conclusioni

**1. Il batch non ha confrontato i planner.** 1/105, 2/105, 2/105 con intervalli di confidenza
sovrapposti, e i 5 successi concentrati su 2 pose. I fallimenti cadono quasi tutti in strati che i tre
modi condividono: il reset, il behaviour `enter_door` e la percezione della porta. Il
`planner_controller_mode` non e' la variabile che questi dati misurano, e nessuna scelta fra i tre e'
giustificabile da qui.

**2. Una posa su sei rompe la fisica di Isaac, e il reset se ne accorge troppo tardi.** Le pose 5, 6, 7,
11, 17 e 27 perdono tutti e 9 i run con lo stesso messaggio: `stop: no standstill on /odometry/filtered
within 5 s (last speed nan m/s)`. La causa e' nei log di Isaac, che per queste 6 pose pesano ~123 MB
contro i ~54 KB delle pose sane. A circa 17 s dal lancio:

1. `Invalid PhysX transform detected for /World/car/Leatherback/Rigid_Bodies/...` su tutti i corpi
   dell'auto (Chassis, Shock_Front/Rear, Steering_Knuckle, Suspension, Wheel_*);
2. `PhysX error: Illegal BroadPhaseUpdateData` (`BpBroadPhaseABP.cpp:4037`), 8741 volte nei primi 40 MB
   di log: e' quello che gonfia il file;
3. `Sensor reading not finite -> rigid-body IMU fallback: gyro [nan nan nan] acc [nan nan nan] q [nan nan nan]`
   - anche il fallback e' NaN.

Il NaN arriva su `/odometry/filtered`, lo step `stop` del reset non vede mai una velocita' e rinuncia,
due volte, e la posa e' persa per intero. E' deterministico per posa: l'auto nasce in uno stato non
valido, non e' sfortuna. 54 run su 315 sono buttati prima di partire.

**3. Il verdetto del behaviour e' sbagliato nel 78 % dei casi in cui dice di aver vinto.** 23 run
arrivano a `stop_condition:door_entered` e la missione va COMPLETE; per la ground truth 18 di quei 23 non
sono ne' dentro ne' attraverso l'apertura vera. `nel varco (GT)` e' **0 su tutti e tre i modi**: tutti e 5
i successi arrivano da `attraversato`, cioe' dalla traiettoria che taglia l'apertura, mai dall'auto che si
ferma dentro. Oggi niente nello stack sa distinguere "sono nel varco" da "mi sono fermato altrove": senza
il controllo di ground truth questo batch si sarebbe letto come 23/315.

**4. La correzione geometrica della missione non ha funzionato.** Il file della missione spiega perche'
esiste: con il rettilineo da 5 m l'auto finisce **sull'**asse della porta, la curva di Dubins
(rho ~ 1.06 m) la supera di 1.06 m e `EnterDoor` risponde `door_needs_steered_reverse` senza muoversi; il
rettilineo da 3 m si ferma 2 m prima per lasciare spazio alla curva. Con 3 m
`door_needs_steered_reverse` resta il primo fallimento, 96 run su 217, su tutti e tre i modi. La
geometria del run-up non e' (l'unica) causa: l'ipotesi scritta nella missione e' smentita da questo batch.

**5. La porta non si vede in 82 run su 217** (`no_strong_door` 62 + `door_not_found` 20, 38 %), e dipende
dalla posa: la 34 fa 3/3 `door_not_found` sull'unico modo che e' riuscito a volare la'. La larghezza del
varco va da 0.7 a 1.6 m e l'unica posa che funziona piu' di una volta (15) e' a 0.95 m, quindi la
larghezza da sola non lo spiega.

**6. `corridor_mpc` costa 3.3 volte la CPU degli altri due senza rendere nulla**: 863.5 contro 262.3 e
256.5 ms di CPU del controller per secondo simulato, mediana su tutti i run, a parita' di successo.

**7. Un difetto dell'analizzatore, da sapere quando si rilegge il report generato.**
`scripts/car/orin/hil_agent.py:812` cerca il rettilineo come `move_0_straight_5m`, ma la missione girata
e' `straight_3m_then_left_door_direct`, il cui move si chiama `move_0_straight_3m`. Quindi `move0_*` e'
vuoto per tutti i 315 run, con due effetti nel report generato: la colonna `rettilineo ok` segna 0/105 su
tutti i modi anche se 217 run raggiungono `stop_condition:distance_reached`, e in `failure_label` il ramo
`straight:*` non puo' mai scattare, cosi' i 18 run andati in timeout **nel rettilineo** finiscono
etichettati sotto `door:*` o `mission:ABORTED`. I numeri che contano (`success`, `gt_in_door`,
`crossed_door`) non ne dipendono.

Nota: i 20 tentativi di reset falliti con `slam: no /slam/pose within 60 s` (14 pose diverse) si
recuperano al secondo tentativo (19 reset-2 OK su 20) e non costano run, ma costano ~60 s ciascuno.

## Prossimi passi

- Capire cosa hanno in comune le pose 5, 6, 7, 11, 17, 27 in `door_runup_left_5m.json`: il sospetto e' una
  posa di partenza che interseca la geometria dell'office, visto che il PhysX si rompe allo spawn. Finche'
  resta, 54 run su 315 sono persi in partenza.
- Far fallire subito e a voce alta lo step `stop` del reset quando la velocita' e' NaN, invece di
  aspettare 2 x 5 s; e in `car-hil` trattare un `/odometry/filtered` NaN come "riavvia Isaac", non come
  "ritenta il reset".
- Dare a `enter_door` una condizione di fine che non sia la sua stessa stima della porta, oppure portare
  il controllo di ground truth dentro il giro: 18 COMPLETE su 23 erano falsi.
- Rimisurare l'ipotesi `door_needs_steered_reverse`: il rettilineo da 3 m non l'ha toccata. Serve sapere
  dove e' l'auto quando `EnterDoor` risponde cosi' (i CSV per run sull'Orin hanno la traccia GT).
- Correggere `hil_agent.py:812` leggendo i move id dalla missione invece di scriverli nel codice, e
  rilanciare `analyze` sulla sessione: non serve rivolare niente.
- Decidere se `corridor_mpc` resta nel confronto a 3.3x di CPU e zero vantaggio misurato.
- Rendere l'Orin raggiungibile da questa macchina (host key) per portare in git figure e CSV per run.

Collegato: [2026-11-13_office-feasible-tests](../2026-11-13_office-feasible-tests/report.md) (le 35 pose),
[2026-11-13_planner-modes-compare](../../mpc-control/2026-11-13_planner-modes-compare/report.md) (i tre
modi su singoli run),
[_unsorted/2026-11-13_office-hil-campaign](../../_unsorted/2026-11-13_office-hil-campaign/report.md) (la
campagna office HIL che era rimasta senza risultati).
