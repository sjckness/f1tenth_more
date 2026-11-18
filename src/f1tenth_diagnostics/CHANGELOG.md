# Changelog: `f1tenth_diagnostics`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_diagnostics`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` al commit `e376731f`, nessuna modifica al codice
  rispetto a 0.0.1. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.1] - 2026-10-16

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-07-13 al 2026-10-16. Voci ricostruite da `git log main -- src/f1tenth_diagnostics` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Nodi di calibrazione, batteria e osservatore di sistema, con README; porte di sicurezza della calibrazione (fase A). (`a18ac0f3`, `77dde41c`, `8dcee01e`; 2026-08-10..2026-08-19)
- Calibrazione ad anello aperto di offset di sterzo e passo. (`dd843afb`; 2026-09-08)
- Nodo `ekf_cost_observer` per misurare direttamente il costo per aggiornamento dell'EKF. (`1635aeda`; 2026-09-08)
### Changed
- Pre-controllo batteria: il gate all'avvio è solo consultivo, l'e-stop a runtime resta. (`9cbc064b`; 2026-09-08)
- Frame `base_front`: ogni distanza frontale è misurata dal paraurti. (`10d3eb34`; 2026-10-13)
- `gyro_variance_z` in rad^2/s^2, l'unità effettiva del topic. (`d6dcf45a`; 2026-09-10)
### Removed
- **BREAKING:** `ekf_cost_observer` (nodo, test, script, launch argument ed entry point), perché l'EKF resta a 20 Hz. Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`0146d174`; 2026-10-16)
