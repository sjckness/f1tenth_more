# Changelog: `f1tenth_ctl`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_ctl`).

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
2026-10-20 al 2026-10-22. Voci ricostruite da `git log main -- src/f1tenth_ctl` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- CLI di gestione processi dell'auto: stack, planner, logger, READY, e-stop. Evidence: docs/analysis/hardware-comms/2026-10-20_telegram-control/report.md (`c9338de4`; 2026-10-20)
- Comando `estop-reset` e campo `estop_latched` in READY; un solo ambiente ROS per target, verificato; ambiente ROS per la shell (discovery server, super client, `--no-daemon`). (`f4a12c9c`, `2819c62f`; 2026-10-20)
- `rosprobe.estop_with(node, ...)` per l'e-stop su un nodo già esistente. (`55b00e7f`; 2026-10-20)
### Changed
- Il conteggio alla partenza è quello armato dal loader, non un 15 s fisso; il guard di caricamento usa il conteggio armato dallo start. (`04db46fe`, `22f2bf51`; 2026-10-22)
- Planner LLM spento di default (`enable_intelligence` false). (`9bed2561`; 2026-10-22)
### Fixed
- La scoperta di `/mission/status` ha 15 s a disposizione invece di 3. (`adb91439`; 2026-10-21)
