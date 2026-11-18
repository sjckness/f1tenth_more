# Changelog: `f1tenth_localization`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_localization`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` al commit `e376731f`, nessuna modifica al codice
  rispetto a 0.0.0. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.0] - 2026-10-16

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-06-19 al 2026-10-16. Voci ricostruite da `git log main -- src/f1tenth_localization` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Nodo `ekf_cost_observer` per misurare il costo per aggiornamento dell'EKF, poi spento di default. (`1635aeda`, `f2b4393b`; 2026-09-08..2026-10-16)
- Doppio EKF (checkpoint di sessione 2026-08-21). (`76804f59`; 2026-08-21)
### Changed
- Politica di frequenza: EKF a 20 Hz, muro virtuale calcolato sulle nuove stime. Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`9786bfc5`; 2026-10-16)
### Removed
- **BREAKING:** `ekf_cost_observer` (nodo, entry point, dipendenza e launch argument), deciso di tenere l'EKF a 20 Hz. Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`0146d174`; 2026-10-16)
