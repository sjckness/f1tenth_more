# Changelog: `f1tenth_hardware`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_hardware/f1tenth_hardware`).

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

## [0.0.1] - 2026-10-13

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-06-19 al 2026-10-13. Voci ricostruite da `git log main -- src/f1tenth_hardware/f1tenth_hardware` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Porte di sicurezza della calibrazione (fase A). (`8dcee01e`; 2026-08-19)

### Changed
- Offset del centro sterzo del VESC ritarato su 50 bag archiviati. (`9896615c`; 2026-09-08)
- Pre-controllo batteria: gate d'avvio consultivo, e-stop a runtime mantenuto. (`9cbc064b`; 2026-09-08)
- Calibrazione della campagna finale: guadagno erpm, guadagni servo, limiti di sterzo MPC. (`9b8f1b86`; 2026-10-13)
