# Changelog: `f1tenth_telegram`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_telegram`).

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
2026-10-20 al 2026-10-23. Voci ricostruite da `git log main -- src/f1tenth_telegram` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Pannello di controllo Telegram via ssh + `f1tenth_ctl`, con test in simulazione. Evidence: docs/analysis/hardware-comms/2026-10-20_telegram-control/report.md (`120b5684`; 2026-10-20)
- Comandi del planner da Telegram (invio e log del planner) in due passi (piano, poi avvio) e comandi vocali. Evidence: docs/analysis/hardware-comms/2026-10-20_telegram-control/report.md (`9591c4e3`, `b131b9bf`; 2026-10-20)
- `estop-reset` e `estop_latched` in READY. (`f4a12c9c`; 2026-10-20)

### Changed
- Profilo sim per Isaac (`0ecdd9bf`; 2026-10-23).
- L'e-stop trattiene la missione; il bot usa IPv4 con retry. (`ef74f9e5`; 2026-10-20)
- Un avvio senza conto alla rovescia dichiara che si muove subito, non dopo 15 s. (`0b23bf19`; 2026-10-22)
