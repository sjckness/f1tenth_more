# Changelog: `f1tenth_more`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_more`).

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

## [0.0.1] - 2026-10-22

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-06-19 al 2026-10-22. Voci ricostruite da `git log main -- src/f1tenth_more` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- README di tutti i pacchetti e indice documentale di primo livello che li collega. (`581dfcc6`, `15621b17`, `e3378fd9`, `81bde294`; 2026-08-10..2026-08-11)
- Documentazione dell'indagine wall_turn, checklist di bringup e diagnostica. (`60cb162e`; 2026-09-14)
- Mock LLM2 della due-LLM con test e documentazione. Evidence: docs/analysis/llm-pipeline/2026-10-21_two-llm-arch/report.md (`b2acbe82`; 2026-10-22)
### Changed
- Metapacchetto con lista `exec_depend` completata; dichiarato l'extra `test` in `setup.py`. (`b3805ec7`, `aaae3770`; 2026-08-10..2026-09-03)
- Pass stack-wide: percezione a segmentazione, corridoio MPC, logging di missione; messa a punto del corridoio e rework del launch di detection. (`96c6bbcb`, `ace2446e`; 2026-09-03..2026-09-10)
- CPU del behaviour tree per messaggio e non per tick: e-stop LiDAR event-driven, `IsProximityTooClose` in numpy, handler QoS rimossi dopo il setup, rendering PNG su thread. Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`69d2e1cb`, `a87e828c`; 2026-10-16)
