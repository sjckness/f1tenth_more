# Changelog: `f1tenth_diagnostics`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_diagnostics`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.2.0] - 2026-11-24

### Added
- `stack_info_node` (da `diagnostics_server.launch.py`, se `enable_stack_info`): pubblica su
  `/viz/stack_info` un `diagnostic_msgs/DiagnosticStatus` (reliable + transient_local, depth 1,
  all'avvio e ogni 10 s) con profilo (`car` / `sim-<host>` / `native`), immagine o albero git,
  `branch@commit`, comando di lancio, `stack_args`, setup ROS. WARN con un trial in `stack_args`,
  un albero dirty o un commit sconosciuto. Logica pura in `stack_info.py`, test in
  `test/test_stack_info.py`.
- Nel container lo stack non vede `F1T_CONTAINER_ROLE`, `F1TENTH_IMAGE_REF`, `F1TENTH_IMAGE_ID`
  (f1tenth_ctl ripulisce l'ambiente prima di sorgere `ctl_container.env`): il nodo li legge da
  `/proc/1/environ`.

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` dopo il merge della linea Jazzy
  (`origin/main` 74c08897), nessuna modifica al codice rispetto a 0.0.1. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.1] - 2026-10-23

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-07-13 al 2026-10-23. Voci ricostruite da `git log main -- src/f1tenth_diagnostics` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Nodi di calibrazione, batteria e osservatore di sistema, con README; porte di sicurezza della calibrazione (fase A). (`a18ac0f3`, `77dde41c`, `8dcee01e`; 2026-08-10..2026-08-19)
- Calibrazione ad anello aperto di offset di sterzo e passo. (`dd843afb`; 2026-09-08)
- Nodo `ekf_cost_observer` per misurare direttamente il costo per aggiornamento dell'EKF. (`1635aeda`; 2026-09-08)

### Changed
- Pre-controllo batteria: il gate all'avvio è solo consultivo, l'e-stop a runtime resta. (`9cbc064b`; 2026-09-08)
- Frame `base_front`: ogni distanza frontale è misurata dal paraurti. (`10d3eb34`; 2026-10-13)
- `gyro_variance_z` in rad^2/s^2, l'unità effettiva del topic. (`d6dcf45a`; 2026-09-10)

### Fixed
- Il check e-stop della calibrazione sterzo attende fino a 5 s il grafo e il launch imposta ROS_SUPER_CLIENT (`f7087b48`, `3eb4e3fd`; 2026-10-16..2026-10-23). Evidence: docs/analysis/platform/2026-10-03_fix-batch-3/report.md
- `system_observer` salta una thermal zone che restituisce None (crash sul Thor) (`586ad023`; 2026-10-23). Evidence: docs/analysis/platform/2026-10-03_fix-batch-1/report.md

### Removed
- **BREAKING:** `ekf_cost_observer` (nodo, test, script, launch argument ed entry point), perché l'EKF resta a 20 Hz. Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`0146d174`; 2026-10-16)
