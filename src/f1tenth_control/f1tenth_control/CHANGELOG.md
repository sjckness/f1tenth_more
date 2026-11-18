# Changelog: `f1tenth_control`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_control/f1tenth_control`).

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
2026-06-19 al 2026-10-22. Voci ricostruite da `git log main -- src/f1tenth_control/f1tenth_control` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Launch file `*.launch.py` e `mpc_corr.launch.py`; nodo di clearance frontale; pacchetto di controllo wall_turn (tracker, turn planner, d_wall, log del modello). (`8ee3a84e`, `ace2446e`, `4c35975a`; 2026-08-10..2026-09-14)
- Argomenti per il corridoio d'oggetto (arco, lunghezza massima, tolleranza per curva). (`51c6154e`, `79663383`, `bc3b0490`; 2026-09-23)
- Righe-muro (semipiani dal tracker multi-muro) dietro `boundary_source`, con i launch argument che ora arrivano davvero. Evidence: docs/analysis/mpc-control/2026-10-19_avoidance-wall-boundaries/report.md (`91fe0b0e`, `d3671b17`; 2026-10-19)
- `hard_stop_node`: stop LiDAR di ultima istanza fuori dal behaviour tree (solo `/scan` a 40 Hz, nessuno sterzo), consapevole della direzione dietro `hard_stop_allow_reverse` (default true). (`16c91ded`, `b31155e1`; 2026-10-19..2026-10-20)
- Sottoscrizione `MPC_corr` e whitelist per gli aggiornamenti parametri di LLM2, con mock e test. (`4e93fdc0`, `b2acbe82`; 2026-10-22)

### Changed
- Pesi `mpc_corr` normalizzati e unica fonte via `stack_params`, limiti di sterzo reali dalla calibrazione del servo, geometria del corridoio e spostamento target ostacolo (0.36 -> 0.30) fissati per misura; opt-out da launch argument per frequenza corridoio e vincoli duri. (`431aed8a`, `ee9fb1db`, `6dc5a37d`, `863304aa`, `ea66819f`; 2026-09-03..2026-09-09)
- Campagna finale: passo di predizione MPC 0.2 s, N = 20 (orizzonte 4 s, ciclo 10 Hz). (`30b0580e`; 2026-10-13)

### Fixed
- Il default `cpu_affinity` di `mpc_corr` non era mai collegato. (`5200d2d1`; 2026-08-13)
