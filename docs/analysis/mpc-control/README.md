# Area: mpc-control (MPC e controllo)

MPC `mpc_corr` (RTI/OSQP), pesi e tuning, corridoio, ostacoli nel costo, retromarcia, velocità e arresto, VESC.

## 1. Stato attuale

> MPC ok sugli statici; ostacoli in moto migliorati solo in Isaac (dodge 12-13/18); nulla del 2026-11 è stato provato sull'auto.

- Solver affidabile sul campo: 1285/1285 QP risolti, solve p95 20.4 ms su periodo 100 ms (2026-09-01).
- Su `/drive` un solo clamp (+1.0 / 0.0 m/s) dal 2026-09-16; i limiti interni del solver (-1/+3 m/s) restano diversi.
- Velocità: la legge di comando legacy porta l'auto a 0.23-0.27 m/s quando la missione chiede 0.5; `offset_free` è il nuovo default ma è verificato solo su dati d'archivio e rig.
- Arresto: con la rampa di decelerazione l'errore di arresto resta entro 0.068 m fino a 1.5 m/s (solo modello nominale).
- Retromarcia: la manovra separata gira sull'auto (12 `distance_done` su 13); la retromarcia scelta dall'MPC non passa le porte (xfail).
- Ostacoli: in Isaac il target sulla centerline con `w_obs` 200 supera gli statici S1-S3 (8/9), S4 va in e-stop; con dischi predetti e scia gli ostacoli in moto passano da 5/18 a 12/18 (13/18 con le sonde sul corpo, 0 collisioni).
- Margine persona: nessun valore di `obstacle_class_margin_m` regge il dropout del campo visivo (rig nominale).
- 13 test di `mpc_controller` falliscono su tutti i branch: fissano valori di config cambiati.
- Planner-controller alternativi: in `corridor` i tre modi (`corridor_mpc`, `corridor_rrt`, `p2p_geometric`) chiudono tutti COMPLETE entro 3.14 m, ma `corridor_rrt` e `p2p_geometric` costano 84-91 ms di CPU per secondo simulato contro i 400-500 di `corridor_mpc` (HIL Isaac, 2026-11-13).
- `corridor_rrt` in `base_room` si fermava a 2.037/2.038 m su due run (188 plan falliti di fila, timeout ABORTED): il target rotolante di un move DRIVE entra in qualunque muro piu' vicino di `rrt_corridor_length_drive_m`. Con `rrt_target_retreat` chiude COMPLETE a 3.105 m in 6.5 s, con il plan time medio da 1.09 a 12.68 ms (max 81.97).
- L'ingresso porta in `office`/`door_left` fallisce allo stesso punto (7.66-7.72 m) per tutti e tre i modi, su tre ripetizioni: non e' una questione di planner.

## 2. Decisioni prese

- Clamp unico su `/drive` a +1.0 / 0.0 m/s ([2026-09-16_chase-overspeed](2026-09-16_chase-overspeed/report.md))
- `go_to_object` come gap, `gap_min` 0.42 m, default 0.5 m ([2026-09-17_object-gap](2026-09-17_object-gap/report.md))
- Nessun margine persona adottato; prove a terra con manichino ([2026-09-17_person-pass-margin](2026-09-17_person-pass-margin/report.md))
- `boundary_source=walls` di default; hard stop LiDAR come nodo proprio (soglia 0.20 m) ([2026-10-19_avoidance-wall-boundaries](2026-10-19_avoidance-wall-boundaries/report.md))
- `allow_reverse` resta false; un solo meccanismo di retromarcia all'avvio ([2026-10-20_mpc-v2-reverse-memory](2026-10-20_mpc-v2-reverse-memory/report.md))
- `speed_command_mode: offset_free` default, legacy bit-identico come rollback ([2026-10-21_speed-gap](2026-10-21_speed-gap/report.md))
- Target sempre sulla centerline, `mpc_w_obs` 8 -> 200, regola di lato a sinistra ([2026-11-17_isaac-mpc-tuning](2026-11-17_isaac-mpc-tuning/report.md))
- Set consigliato alpha + scia + sonde (`mpc_params_tuned.yaml`); regola di lato informata da sola sconsigliata ([2026-11-18_mpc-dodge](2026-11-18_mpc-dodge/report.md))
- `rrt_target_retreat` adottato, OFF per default: per un target terminale rifiutare di pianificare dentro un muro resta giusto ([2026-11-13_planner-modes-compare](2026-11-13_planner-modes-compare/report.md))

## 3. Domande aperte / prossimi esperimenti

- Rifare il loop di Fase 3 del dodge con un criterio che regga il rumore (50-80 % di successo per run) e portare il set sull'Orin ([dodge](2026-11-18_mpc-dodge/report.md)).
- d = 2 (cilindri a 0.4 m/s), ostacoli frontali M2/P2, e-stop di M3/S4 a margine circa 0.
- Verificare sull'auto `offset_free`, la rampa di arresto e il front-wall lock (tutti solo su rig o replay).
- S4 in Isaac: chi impone la velocità alla partenza (speed channel o floor)?
- Pesi trovati con `unknown_objects_mode: active` (sim) contro `off` (auto): trasferibilità.

## 4. Esperimenti

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| data | esperimento | status | risultato | moduli (versione) |
|---|---|---|---|---|
| 2026-11-18 | [MPC 'dodge' in Isaac: velocità degli ostacoli, dischi predetti, scia, sonde sul corpo, arco di ritorno](2026-11-18_mpc-dodge/report.md) | open | A/B rifatto (6 seed, livelli 0-2): baseline dodge 5/18 M 3/12; alpha 9/18 M 7/12; alpha+scia 12/18 M 9/12; alpha+scia+sonde 13/18 M 8/12 S 9/13, 0 collisioni; d=2 resta un muro (<= 3/6) | mpc_controller 0.0.0, f1tenth_costmap 0.0.1, f1tenth_messages 0.0.1, f1tenth_params 0.0.1 |
| 2026-11-17 | [Harness Isaac per il tuning MPC 'ostacolo davanti' e MPC con target sulla centerline](2026-11-17_isaac-mpc-tuning/report.md) | done | Iterazione 1 (default): S1 3/3, S2 3/3, S3 2/3, S4 0/3 (e-stop), S5 1/1; M1 mobile 0/1 (collisione); scarti laterali 1.3-1.6 m per un Ø0.5 | mpc_controller 0.0.0, f1tenth_params 0.0.1 |
| 2026-11-13 | [corridor_mpc, corridor_rrt e p2p_geometric a confronto: 11 sessioni HIL in Isaac](2026-11-13_planner-modes-compare/report.md) | open | In corridor tutti e tre COMPLETE entro 3.14 m; corridor_rrt costa 87-90 ms di CPU per secondo simulato contro i 400-500 di corridor_mpc (p2p 84-105). In base_room corridor_rrt va ABORTED fermo a 2.037/2.038 m (188 plan… | f1tenth_planners 0.0.1, mpc_controller 0.1.0, f1tenth_params 0.1.0, llm 0.1.0 |
| 2026-10-21 | [Perché l'auto va a 0.23 m/s quando la missione chiede 0.5](2026-10-21_speed-gap/report.md) | open | Nessun cap su vdes; l'MPC pianifica 0.5 (168/171 tick) ma MPC_corr pubblica v_cmd = v_meas + a_cmd*ts; con r=0.758 punto fisso 0.353 m/s (misurato 0.25-0.27). Con offset_free sul plant da 104 run: 0.40 m/s per vdes 0.4,… | mpc_controller 0.0.0, f1tenth_hardware 0.0.1, f1tenth_params 0.0.1 |
| 2026-10-20 | [Rampa di decelerazione: closed loop a modello nominale](2026-10-20_stop-distance/report.md) | done | Errore di arresto soglia/rampa: 0.6 m/s -0.331/+0.015 m; 1.0 m/s -0.682/-0.025 m; 1.5 m/s -1.271/-0.068 m; decel. picco rampa 0.59-0.66 m/s^2. | mpc_controller 0.0.0 |
| 2026-10-20 | [MPC v2: manovra retro, memoria oggetti, integrazione BT, avvio LLM](2026-10-20_mpc-v2-reverse-memory/report.md) | open | Su linus: test_reverse_manoeuvre 45 pass; mpc_controller 900 test con le stesse 21 failure di 868ec8a; f1tenth_behavior 442 pass con stub py_trees_ros. STATUS 10-21: sull'auto 13 retromarce, 12 distance_done, 1 rear_cle… | mpc_controller 0.0.0, f1tenth_costmap 0.0.1, f1tenth_behavior 0.0.1, llm 0.0.1, f1tenth_params 0.0.1, f1tenth_messages 0.0.1 |
| 2026-10-20 | [13 test di mpc_controller gia' falliti](2026-10-20_mpc-known-failures/report.md) | done | Su linus: 1058 passati, 13 falliti, 1 saltato, 2 xfail; gli stessi 13 su campaign-final (cd180d8) e mpc_corr_v2_reverse (4f53ab8). Nessuno causato dai merge; quasi tutti fissano un valore di config cambiato (es. passo p… | mpc_controller 0.0.0, f1tenth_params 0.0.1 |
| 2026-10-19 | [Retromarcia scelta dall'MPC (candidati A/B/C1.0)](2026-10-19_reverse-mpc/report.md) | done | Su armonico nominale: mai retromarcia in corridoio libero, retro a obiettivo 1.5 m dietro, inversione per 3.5 m dietro, nessun dithering; non passa le porte (scenari b, c). Test mpc: 19 failed/801 passed come a 868ec8a. | mpc_controller 0.0.0, f1tenth_params 0.0.1 |
| 2026-10-19 | [Avoidance: muri tracciati come righe half-plane per l'MPC](2026-10-19_avoidance-wall-boundaries/report.md) | open | A1 (boundary_source=walls di default) e A2 (hard_stop_node, soglia 0.20 m) completati; 32 test nuovi, mpc_controller 782 pass / 11 fail preesistenti, costmap 256 pass. A3: nessun passaggio ravvicinato nei bag, clearance… | mpc_controller 0.0.0, f1tenth_costmap 0.0.1, f1tenth_navigation 0.0.0, f1tenth_params 0.0.1 |
| 2026-09-17 | [Passaggio di una persona in piedi: dropout FOV e margine persona](2026-09-17_person-pass-margin/report.md) | done | Nessun margine passa: con dropout, margine 0.4 dà gap -0.024 m sulla linea e +0.117 m a offset 0.3. Default footprint senza margine: -0.205 m. Dropout costa ~0.05-0.12 m, avviene a 2.0-3.7 s. | mpc_controller 0.0.0, f1tenth_perception 0.0.1, f1tenth_behavior 0.0.1 |
| 2026-09-17 | [go_to_object come gap con minimo derivato dal controller](2026-09-17_object-gap/report.md) | done | gap_min 0.42 m, gap di default 0.5 m; gap assestato 0.427-0.499 m, object_reached in tutti gli 8 scenari; approccio a 70 gradi sovrashoota a 0.42 m (live r -0.079) | mpc_controller 0.0.0, f1tenth_params 0.0.1, f1tenth_costmap 0.0.1 |
| 2026-09-16 | [Inversione e sovravelocita' di /drive in mpc_corr (chase)](2026-09-16_chase-overspeed/report.md) | done | Nel rig di inseguimento baseline picco +2.074 / -1.043 m/s; con w_psi=0 e w_psi_stage=0 picco +0.756, nessuna retromarcia. Archivio: 101 run, min -0.231, max 1.206 m/s. | mpc_controller 0.0.0, f1tenth_messages 0.0.1 |
| 2026-09-14 | [Indagine wall_turn: fase osservabile su /mpc/wall_track e punto di correzione del corridoio](2026-09-14_wall-turn-investigation/report.md) | done | Fase osservabile su /mpc/wall_track a 10 Hz (con 4 caveat); psi_end non esiste nel codice; /mpc/wall_track in 0 bag su ~120 run (14 wall_turn/drive_turn_180) | mpc_controller 0.0.0, f1tenth_messages 0.0.1, f1tenth_perception 0.0.1, f1tenth_params 0.0.1 |
<!-- AUTO:END -->
