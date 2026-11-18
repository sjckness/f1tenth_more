# Changelog: `f1tenth_behavior`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_behavior`).

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

## [0.0.1] - 2026-10-24

Versione segnaposto di `ros2 pkg create`, mai incrementata: copre la storia del modulo su `main` dal
2026-07-13 al 2026-10-24. Voci ricostruite da `git log main -- src/f1tenth_behavior` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Nuovo pacchetto con albero di comportamento: corsia di emergenza, sottoalbero di missione, visualizzazione del BT. (`37be92eb`; 2026-08-10)
- Robustezza missione: preflight, scoring a ciclo chiuso, test di regressione, audit delle dipendenze; rilascio di `/mpc/hold` su `start_mission`. (`f960cf25`, `6930c3cc`; 2026-08-21)
- Modalità drive: mosse `DriveCommand` a esecuzione aperta (schema 3.0). (`e164f80d`; 2026-09-10)
- Nodo `front_clearance` (legge `/perception/front_distance` e rifiuta il valore sentinella), rincorsa prima della curva e mossa dritta dopo di essa. (`ace2446e`, `63a6080c`, `77d99aa3`; 2026-09-14)
- Mossa di missione `go_to_object` con relativo handler, uscita da object mode di `mpc_corr`, eco di `track_id` e gap in tempo reale, GRACE come stop-and-wait, `object_reached` accetta uno stop latched a veicolo fermo, minimo 0.4 imposto dal loader. Evidence: docs/analysis/mpc-control/2026-09-17_object-gap/report.md (`22ff0315`, `edd0e6bc`, `2bed817a`, `6260f761`, `0b82c192`, `f48f4f3e`, `e5f47311`; 2026-09-17)
- Dati di go_to_object e clamp `/drive` nel bag e nel riassunto del run; etichetta `drive` per le mosse drive nello scoring. (`8ce08fa4`, `46dfb110`; 2026-09-16)
- Logger della campagna di test (`f1tenth_logger`): registratore per test, successo manuale, conteggio alla partenza, sorgenti `/mpc/status`, `/corridor`, `/safety/event`, `/obstacle_clearance`, provenienza del piano. (`dc158d24`, `3e50039b`, `8e0d48c5`, `6de6ac34`, `bc4631ad`; 2026-09-21)
- Missione `drive_test_straight_2m` e missioni di campagna sincronizzate con l'auto. (`1bb5ea24`, `ca66fc27`; 2026-10-24)
- Integrazione BT della manovra di retromarcia (`bt_reverse_integration`). Evidence: docs/analysis/mpc-control/2026-10-19_reverse-mpc/report.md (`b7f95963`; 2026-10-20)
- Stop frontale con aggancio al muro verso cui avanza la mossa (`wall_lock_enabled`). Evidence: docs/analysis/localization-frames/2026-10-22_front-wall-lock/report.md (`6002cb08`; 2026-10-21)
### Changed
- L'abort di missione ingaggia sempre `/mpc/hold`; i report di missione sono scritti atomicamente; il latch `goal_reached` è per run e non per id di mossa. (`07023d86`, `5e0ea770`, `5eb1a9d9`; 2026-09-16)
- Il precheck batteria all'avvio è consultivo; la missione `bottle_then_person` attende l'assestamento e scende a 5 m. (`9cbc064b`, `72c2e03c`; 2026-09-08)
- La registrazione parte da missione LOADED con il conto alla rovescia come pre-roll; poi conto a 0 di default, a sorgente unica in `stack_params` e attivo solo se il logger registra. (`407ff6de`, `a14ec2f5`, `3ac5865e`; 2026-10-21)
- Distanze frontali misurate dal paraurti (`base_front`) e frame del veicolo dalle misure del 2026-10-13. (`10d3eb34`, `bdc7c93b`, `d7b2770e`; 2026-10-13)
- Stop a muro e `wall_turn` leggono l'unica verità del muro: CLEAR guida, NO_DATA tiene, 50 m non arriva mai al solver; lo stop frontale richiede il nodo dietro il topic del muro. (`ca8e15bc`, `127a6993`, `796f3693`; 2026-10-15)
- Riduzione CPU: il BT lavora per messaggio e non per tick, e-stop LiDAR a eventi, un solo predicato di e-stop, chiave `tf_static_only`, hardening Humble. Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md (`69d2e1cb`, `9dde6ee4`, `a87e828c`; 2026-10-16)
- La rampa di arresto arriva al punto di stop invece di superarlo. Evidence: docs/analysis/mpc-control/2026-10-20_stop-distance/report.md (`6c693ac2`; 2026-10-20)
### Fixed
- Corretto il sistema di riferimento della mossa dritta dopo una curva e lo spin da `orientation_delta` di 180 gradi. (`2bd64204`; 2026-08-21)
- Lo stop a muro tiene (hold) con distanza del muro vecchia o assente. (`17caae26`; 2026-10-14)
### Removed
- Rimossa `tests_require` morto da `setup.py`; dichiarato l'extra `test` così `colcon test` esegue pytest. (`626fb7b6`, `aaae3770`; 2026-09-03)
