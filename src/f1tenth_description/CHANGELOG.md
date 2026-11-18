# Changelog: `f1tenth_description`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_description`).

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

## [0.0.0] - 2026-10-15

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-06-19 al 2026-10-15. Voci ricostruite da `git log main -- src/f1tenth_description` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Frame `base_front` (xacro) che porta ogni distanza frontale al paraurti, con modulo `vehicle_frames` (`10d3eb3`; 2026-10-13).
- Pubblicazione di `/joint_states` così i giunti mobili raggiungono il TF (`bad6d7c`; 2026-09-08).

### Changed
- Geometria del veicolo dalle misure reali del 2026-10-13/15: posizioni di paraurti, LiDAR e ZED, interasse 0.305 m, `base_link` a terra, altezze di laser (z 0.190) e ZED (z 0.205), beccheggio di 2.083 gradi del supporto ZED (`bdc7c93`, `d7b2770`, `f48e4205`, `83e8bb9`, `704dc40`; 2026-10-15).
