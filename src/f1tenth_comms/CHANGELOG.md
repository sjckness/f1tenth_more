# Changelog: `f1tenth_comms`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_comms`).

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

## [0.0.1] - 2026-10-21

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-10-21 al 2026-10-21. Voci ricostruite da `git log main -- src/f1tenth_comms` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Nodo `link_manager`: heartbeat, stati OK/DEGRADED/LOST, riconnessione, più il risponditore lato base. Evidence: docs/analysis/hardware-comms/2026-10-21_comms-base-wifi/report.md (`b7baa835`; 2026-10-21)
- Livello di copertura Wi-Fi (`wifi_coverage_node`), spento di default. Evidence: docs/analysis/hardware-comms/2026-10-21_comms-base-wifi/report.md (`4bbe1229`; 2026-10-21)
- `enable_comms` come argomento di launch di entrambi i bringup, non solo nel yaml. (`e72e67c5`; 2026-10-21)
