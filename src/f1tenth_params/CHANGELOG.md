# Changelog: `f1tenth_params`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_params`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.4.0] - 2026-11-24

### Added
- `f1tenth_params.stack_provenance`: `read_image_provenance` e `stack_launch_cmd`, spostate senza
  modifiche da `f1tenth_logger/mission_logger_node.py` per condividerle con `stack_info_node` di
  `f1tenth_diagnostics` (modulo puro, solo stdlib).
- `enable_stack_info` in `stack_params.yaml` (default `true`): lancia `stack_info_node`.

Sviluppata come 0.3.0 sul branch `stack-info` (da `main`); rinumerata 0.4.0 al merge in
`campaign3-rrt`, dove 0.3.0 era già `mpc_avoidance_mode`.

## [0.3.0] - 2026-11-24

### Added
- `mpc_avoidance_mode` (default `centerline`), `obstacle_target_shift_m` (0.30, di nuovo: era stato
  tolto con `7c217d95`) e `mpc_legacy_w_obs` (8.0): il modo `legacy` di
  [`mpc_controller` 0.2.0](../f1tenth_control/mpc_controller/CHANGELOG.md).

## [0.2.0] - 2026-11-19

### Added
- `rrt_target_retreat` in `stack_params.yaml` (default `false`): il flag di `corridor_rrt` che fa
  arretrare il punto da raggiungere quando l'estremo del corridoio e' bloccato. Descrizione completa,
  con i numeri misurati, in [`f1tenth_planners` 0.1.0](../f1tenth_control/f1tenth_planners/CHANGELOG.md).

  Provenienza: macchina **work** (`DESKTOP-Q3F81S6`), branch `planner-modes-hil`, commit `3af7d70c`
  del 2026-11-13, portato su `main` con cherry-pick il 2026-11-19.
  `Evidence: docs/analysis/mpc-control/2026-11-13_planner-modes-compare/report.md`

### Fixed
- `version` in `setup.py` riallineato a `package.xml` (era rimasto a 0.0.1 dopo il bump a 0.1.0 del
  2026-11-18).

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` dopo il merge della linea Jazzy
  (`origin/main` 74c08897), nessuna modifica al codice rispetto a 0.0.1. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.1] - 2026-10-24

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-08-10 al 2026-10-24. Voci ricostruite da `git log main -- src/f1tenth_params` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Nuovo pacchetto `f1tenth_params` con `stack_params.yaml` come unica fonte dei default dei parametri di launch, più le chiavi per rilevamento muri e vincolo di confine. (`ed098e35`, `e2f5fed8`; 2026-08-13)
- Parametri del controllore MPC a corridoio (`mpc_corr`): opt-out da launch per periodo del corridoio e vincoli hard di confine, set di pesi normalizzato e limiti di sterzo derivati dalla calibrazione del servo. Corridoio a trapezio con rapporto fisso 2.3/1.3, predizione 0.2 s con N=20 (orizzonte 4 s). (`431aed8a`, `ea66819f`, `863304aa`, `a6dd23e2`, `30b0580e`; 2026-10-13)
- Corridoio d'oggetto per `go_to_object`: direzione mantenuta e agganciata al bersaglio, flag per tick (`inside_turn_radius`, `target_behind`), distanza espressa come gap con minimo derivato dal controllore, arco verso il bersaglio dietro parametro, schema corridoio v2 con valutatore condiviso, lunghezza massima. Evidence: docs/analysis/mpc-control/2026-09-17_object-gap/report.md (`03286d64`, `f2f18f46`, `edd0e6bc`, `90c51d6b`, `51c6154e`, `79663383`, `bc3b0490`; 2026-09-23)
- Pianificatore LLM: `go_to` diventa una mossa `go_to_object` (`go_to_enabled`), intento minimale e traduttore che non inventa nulla, avvio missione alla conferma di caricamento. (`d28a86d7`, `4f1f0458`, `9c240241`; 2026-10-20)
- Margini per classe di ostacolo (`obstacle_class_margin_m = {person: 0.5}`), dischi ostacolo dimensionati dall'impronta e non dall'altezza, margine persona vuoto in caso di perdita del campo visivo. Evidence: docs/analysis/mpc-control/2026-09-17_person-pass-margin/report.md (`4e0391d3`, `56ec0265`, `bfeb5038`; 2026-10-20)
- Frame del veicolo `base_front` (tutte le distanze frontali misurate dal paraurti), posizioni di paraurti, LiDAR e ZED dalla misura a nastro del 2026-10-13, interasse 0.305 m, `base_link` a terra. (`10d3eb34`, `bdc7c93b`, `d7b2770e`, `f48e4205`, `83e8bb99`; 2026-10-14)
- Etichette di scena (`enable_scene_labels`), maschera di scena a una classe per pixel di profondità, maschera propria dal contorno reale dell'alloggiamento, profilazione per stadio (`perception_stage_timing`), immagine annotata solo se sottoscritta. Evidence: docs/analysis/perception/2026-10-13_scene-labels-replay/report.md (`7d033afc`, `00844225`, `14f449ba`, `2c407b8a`, `a92e133e`, `18ad95d1`, `8b681899`; 2026-10-15)
- Tracker multi-muro (`use_wall_tracker`) con osservazioni di muro per ciclo (fusione LiDAR/stereo), contratto frontale WALL/CLEAR/NO_DATA, muri come piani con marker a lastra e layer semantico, pavimento inferito, gate di larghezza, gate E1-E3 e F1, livello unico di conferma (`wall_confirm_*`). Evidence: docs/analysis/localization-frames/2026-10-18_wall-tracker-v2/report.md (`91918900`, `6a9b8437`, `2a2a8c54`, `b188f121`, `e3793e47`, `2e2f1008`, `193fe1de`, `25c9297f`, `a205fb77`, `c44b5036`, `bb297fe5`, `4ecea87e`; 2026-10-21)
- Righe di muro (semipiani dal tracker) come sorgente di confine di default (`boundary_source`). Evidence: docs/analysis/mpc-control/2026-10-19_avoidance-wall-boundaries/report.md (`91fe0b0e`, `d3671b17`; 2026-10-19)
- Rilevamento porte sul tracker multi-muro: nodo con `/perception/doors` solo confermate, montanti per muri di vetro, memoria dei pannelli, host frontali opzionali (`door_host_front_facing`, default off). Evidence: docs/analysis/perception/2026-10-20_doors-wall-tracker/report.md (`5d75fa6a`, `35e9c91d`, `a3351136`, `b2495a3a`, `961c996b`; 2026-10-20)
- Memoria semantica degli oggetti con decadimento legato alla visibilità; un solo percorso ostacoli verso l'MPC con `unknown_objects_mode` e tetto di velocità. Evidence: docs/analysis/perception/2026-10-19_obstacles-v2/report.md (`b5f46ea4`, `c33f969c`, `6f48d197`, `79562b18`; 2026-10-20)
- Retromarcia come scelta propria dell'MPC (candidati di direzione, `allow_reverse`), costo terminale consapevole dei muri, manovra con arbitraggio automatico, `reverse_status` su entrambi i meccanismi, rifiuto di partire con entrambi attivi. Evidence: docs/analysis/mpc-control/2026-10-19_reverse-mpc/report.md (`780fd09c`, `9f780042`, `b5f46ea4`, `b7f95963`, `fc4c2728`, `f8844004`; 2026-10-20)
- Blocco di parametri `wall_lock_enabled` per agganciare il muro verso cui avanza la mossa; la rampa di decelerazione punta al muro agganciato. Evidence: docs/analysis/localization-frames/2026-10-22_front-wall-lock/report.md (`6002cb08`, `00635090`; 2026-10-22)
- MPC: comando di velocità senza offset (modello di ritardo e osservatore di disturbo), rampa di arresto che arriva al punto di stop. Evidence: docs/analysis/mpc-control/2026-10-20_stop-distance/report.md (`4701caa0`, `6c693ac2`; 2026-10-21)
- `wall_turn` dell'MPC: distanza dai muri semantici dietro flag, rampa di velocità e freno su LiDAR grezzo indipendente dal planner, margine di sicurezza 0.7 m. (`4262b74f`, `0ca071fa`, `40edbbd0`; 2026-10-24)
- Due LLM: messaggio `MpcParamUpdate`, configurazione del server LLM2, argomenti di launch `enable_llm2`, whitelist per gli aggiornamenti parametri di LLM2 (parametri di sola lettura, `w_obs` mai regolabile). Evidence: docs/analysis/llm-pipeline/2026-10-21_two-llm-arch/report.md (`bc8eebdf`, `4e93fdc0`, `384621fc`; 2026-10-22)
- YOLO: backend TensorRT con fallback `.pt` e script di export, pipeline GPU (`gpu_pipeline`, default on), limiti di thread e scarto frame a 15 Hz. Evidence: docs/analysis/perception/2026-10-22_yolo-gpu-recon/report.md (`26c76f6c`, `e78f2400`, `5792e2ba`; 2026-10-22)
- Logger di missione: archivio di run a due stati, registrazione profondità ZED a 5 Hz (`mission_logger_record_depth`), registrazione opt-in, overlay del report di run con maschera di scena e LiDAR. (`6ecda96e`, `fa79f593`, `774b2e0f`, `e9d18c26`; 2026-10-22)
- Argomento di launch `enable_comms` e livello di copertura Wi-Fi (`f1tenth_comms`), off di default. Evidence: docs/analysis/hardware-comms/2026-10-21_comms-base-wifi/report.md (`4bbe1229`, `e72e67c5`; 2026-10-21)

### Changed
- Lease Fast DDS 5 s e parametri di R3 (thread, TF) (`04ef8214`, `ceb7c884`; 2026-10-23).
- Calibrazione e default di campagna: velocità massima in avanti 0.5 m/s e velocità go_to 0.4, guadagno erpm, guadagni servo, limiti di sterzo MPC, spostamento del bersaglio ostacolo 0.36 -> 0.30, corridoio a 1.0 s. (`04152e23`, `9b8f1b86`, `f000ae0f`, `6dc5a37d`, `ee9fb1db`; 2026-10-13)
- Il precheck batteria all'avvio diventa solo consultivo, resta l'e-stop a runtime. (`9cbc064b`; 2026-09-08)
- Politica dei tassi per ridurre la CPU: EKF a 20 Hz, muro virtuale a ogni nuova stima, `costmap_boundary` 10 Hz, layer muri semantico 5 Hz, tracker muri su 1 scansione su 4, `extract_lines` in C (24.6 ms -> 1.1 ms a scansione). Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`c4215787`, `e41f606a`, `752f1600`, `9786bfc5`; 2026-10-16)
- Il ruolo `unknown` sostituisce `other` nel tracker muri; le tracce provvisorie sono riportate come `unknown`. (`c1b4883a`; 2026-10-20)
- Default aggiornati dopo i test: conteggio alla partenza missione 0 e a sorgente unica, attivo solo mentre il logger registra; `unknown_objects_mode` spento (gli oggetti solo-LiDAR bloccavano l'auto); rilevamento porte spento di default; `enable_intelligence` false (planner LLM spento). (`a14ec2f5`, `3ac5865e`, `ed2e4a42`, `fb913d16`, `9bed2561`; 2026-10-22)
- `wall_distance_node` spento di default (`enable_wall_distance`). (`686d9119`; 2026-10-16)

### Fixed
- Il gate di confidenza FRONT non dipende più dalla distanza (il muro lontano risultava CLEAR); il muro in coasting vale come frontale al massimo 0.3 s; niente FRONT fantasma da pezzi collineari oltre un'apertura. (`6cbda121`, `9a1ecc10`, `d5789630`; 2026-10-18)
- Un punto a lato dell'auto è escluso dalla distanza frontale invece di essere forzato a 0.01. (`d8609b50`; 2026-10-13)
- Nessun limite di velocità 0.0 in retromarcia; un solo clamp su ogni velocità `/drive` (niente retromarcia non voluta, niente oltre 1.0 m/s). (`e00cd8d7`, `fc4c2728`; 2026-10-20)
- Il toggle di registrazione ora raggiunge davvero il logger. (`774b2e0f`; 2026-10-16)
