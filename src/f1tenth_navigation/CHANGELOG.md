# Changelog: `f1tenth_navigation`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_navigation`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.0] - 2026-11-18

### Changed
- Prima versione numerata: il modulo com'è su `main` dopo il merge della linea Jazzy
  (`origin/main` 74c08897), nessuna modifica al codice rispetto a 0.0.0. Da qui ogni modifica porta un bump di versione e una voce in questo file.

## [0.0.0] - 2026-10-22

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-05-30 al 2026-10-22. Voci ricostruite da `git log main -- src/f1tenth_navigation` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Pacchetto spostato nel workspace colcon sotto `src/`; pubblicazione della mappa pista su `/map` e TF `map->odom`. (`9997d580`, `38121375`; 2026-06-15)
- Doppio EKF, confini costmap, logica di missione e gate di calibrazione (checkpoint); supporto Docker per la Thor. (`76804f59`, `895d9ca9`; 2026-08-21)
- Sottoscrizione MPC_corr e whitelist per gli aggiornamenti parametri di LLM2. Evidence: docs/analysis/llm-pipeline/2026-10-21_two-llm-arch/report.md (`4e93fdc0`; 2026-10-22)

### Changed
- `odom_tf_broadcaster` pubblica il delta relativo. (`0d7714fc`; 2026-06-15)
- Le righe di muro sono la sorgente di confine di default e gli argomenti di launch ora arrivano davvero. Evidence: docs/analysis/mpc-control/2026-10-19_avoidance-wall-boundaries/report.md (`d3671b17`; 2026-10-19)

### Fixed
- `slam.launch.py` non attivava mai slam_toolbox; dichiarate le dipendenze slam_toolbox e lifecycle_msgs (`3954b6af`, `e02ba592`; 2026-10-16). Evidence: docs/analysis/platform/2026-10-01_thor-phase0/report.md
- Dichiarato l'extra `test` in `setup.py`. (`aaae3770`; 2026-09-03)

### Removed
- Rimosso `tests_require` deprecato da `setup.py`. (`e1cf5247`; 2026-06-15)
