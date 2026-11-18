# Changelog: `corridor_perception`

Tutte le modifiche rilevanti di questo modulo (`src/corridor_perception`).

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

## [0.0.1] - 2026-10-20

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-09-22 al 2026-10-20. Voci ricostruite da `git log main -- src/corridor_perception` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Pacchetto di baseline per l'estrazione della struttura per scansione da bag reali (`119f239`; 2026-09-22).
- Tre gate applicati oltre al fit di linea e backend di estrazione in C (`extract_lines` da 24.6 ms a 1.1 ms a scansione), con test di equivalenza dei backend al gap di merge dello stack (0.70 m) (`25c9297`, `c4215787`, `4b8e6d3`; 2026-10-20). Evidence: docs/analysis/platform/2026-10-16_cpu-h1-h5/report.md
