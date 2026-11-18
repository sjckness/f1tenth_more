# Changelog: `f1tenth_sim`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_sim`).

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

## [0.0.0] - 2026-10-13

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-06-19 al 2026-10-13. Voci ricostruite da `git log main -- src/f1tenth_sim` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Supporto Docker per la Thor. (`895d9ca9`; 2026-06-19)
- Mission logic, EKF doppio, confini costmap e gate di calibrazione (checkpoint di sessione). (`76804f59`; 2026-08-21)
### Changed
- Frame del veicolo: interasse 0.305 m nell'URDF; paraurti, LiDAR e ZED seguono. (`d7b2770e`; 2026-10-13)
### Fixed
- Dichiarato l'extra `test` in `setup.py` così `colcon test` esegue pytest. (`aaae3770`; 2026-09-03)
