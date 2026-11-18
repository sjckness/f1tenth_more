# Changelog: `f1tenth_bringup`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_bringup`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` dopo il merge della linea Jazzy
  (`origin/main` 74c08897), nessuna modifica al codice rispetto a 0.0.1. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.1] - 2026-10-23

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-06-19 al 2026-10-23. Voci ricostruite da `git log main -- src/f1tenth_bringup` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Modalità sim Isaac `sim:=none|isaac` nel supervisor bringup, e2e su linus 9/9 PASS (`b82735fd`, `4d667b5f`; 2026-10-16). Evidence: docs/analysis/platform/2026-10-16_jazzy-campaign-port/report.md
- `component_supervisor` (istanza unica forzata, `obstacle_clearance` avviato con lo stack), EKF map->odom, throttling Foxglove, camera sorgente-agnostica (webcam/ZED) con inferenza YOLO reale. (`10997333`, `f8bab668`, `eeb4b384`, `0d1c90d9`; 2026-07-02..2026-09-21)
- Nuovo pacchetto `f1tenth_logger`: registratore e replay renderer spostati fuori dal bringup; registrazione opt-in con toggle che ora raggiunge il logger. (`903a0f18`, `774b2e0f`; 2026-09-03..2026-10-16)
- Registrazione di `lidar_front_wall`, `wall_distance` e `swept_clearance`; fonti dati della campagna (`/mpc/status`, `/corridor`, `/safety/event`, `/obstacle_clearance`). (`286327c8`, `3e50039b`; 2026-09-14..2026-09-21)
- Bringup della campagna finale: relay viz, whitelist Foxglove, `reset_manager`, calibrazione (guadagno erpm, guadagni servo, limiti sterzo MPC). (`3f3cfba6`, `9b8f1b86`; 2026-10-13)
- Livello di copertura Wi-Fi (`wifi_coverage_node` in `f1tenth_comms`, off di default) e `enable_comms` come launch argument di entrambi i bringup. (`4bbe1229`, `e72e67c5`; 2026-10-21)
- Architettura a due LLM: messaggio `MpcParamUpdate`, config server LLM2 e launch argument `enable_llm2`. Evidence: docs/analysis/llm-pipeline/2026-10-21_two-llm-arch/report.md (`bc8eebdf`, `87395195`; 2026-10-22)
- Porte/porte-vetro: regola di merge collineare unica, anta aperta, veto vetro, `/viz/doors` sempre attivo e loggato; marker dei muri esistenti sul bridge con lifetime. (`35e9c91d`, `a94f2cba`; 2026-10-16..2026-10-19)

### Changed
- Profilo Fast DDS con lease 5 s per ogni partecipante (`04ef8214`; 2026-10-23). Evidence: docs/analysis/platform/2026-10-05_fix-batch-5/report.md
- R3: correzioni CPU in Python (TF solo da /tf_static, map->odom a polling, swept_clearance su richiesta, limiti dei thread) (`ceb7c884`; 2026-10-23). Evidence: docs/analysis/platform/2026-10-23_main-jazzy-rust/report.md
- Politica di frequenza: EKF a 20 Hz, muro virtuale su nuove stime, costmap_boundary 10 Hz, layer muro semantico 5 Hz, `reset_manager` senza lavoro a vuoto; hardening Humble, predicato di e-stop unico, chiave `tf_static_only`. Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`9786bfc5`, `752f1600`, `a87e828c`; 2026-10-16)
- `wall_distance_node` spento di default (`enable_wall_distance`). Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`686d9119`; 2026-10-16)
- ekf_global: gate e Q sostituiti con valori misurati; `gyro_variance_z` in rad^2/s^2; urg_node pubblica l'intensità. (`6ea176d7`, `d6dcf45a`, `25902ef6`; 2026-09-08..2026-09-14)
- Odom: eliminate le due zone morte di `vesc_to_odom_backup`, segno della retromarcia fissato. (`9cbc064b`; 2026-10-20)
- Il conto alla rovescia di inizio missione scatta solo se il logger di missione registra. (`a14ec2f5`; 2026-10-20)
- Pianificatore LLM spento di default (`enable_intelligence` false) nel main consolidato. (`9bed2561`; 2026-10-22)

### Fixed
- Correzioni di build, test e ambiente del porting Jazzy (`d9c381e9`, `2893d419`; 2026-10-16..2026-10-19).
- Il gruppo diagnostics non risulta più "crashato" con il logger spento. (`82c0afa3`; 2026-10-16)
