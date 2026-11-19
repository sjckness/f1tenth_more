# Changelog: `f1tenth_planners`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_control/f1tenth_planners`).

Formato [Keep a Changelog 1.1.0](https://keepachangelog.com/it-IT/1.1.0/), versioni
[SemVer](https://semver.org/lang/it/): MAJOR rompe un'interfaccia usata da altri (topic, servizio,
messaggio, parametro), MINOR aggiunge una funzione o cambia un comportamento, PATCH corregge.
Ogni bump di `<version>` in `package.xml` ha la sua voce qui, nello stesso commit. Una voce può citare
l'analisi che la sostiene: `Evidence: docs/analysis/<area>/<id>/report.md`.

## [Unreleased]

## [0.1.0] - 2026-11-19

### Added
- `corridor_rrt`: `rrt_target_retreat`. Quando RRT* non trova un percorso verso il target, un solo
  retry verso il punto libero più lontano sulla centerline del corridoio (`retreat_goal()` cammina
  dall'estremo lontano indietro, almeno `rrt_corridor_length_min_m`/2 dall'auto), invece di fermarsi.
  Mai ricorsivo. Il corridoio continua a essere costruito fino al target con il modulo di geometria
  condiviso con `mpc_corr`: si muove solo il punto da raggiungere dentro di esso, e `plan_in_corridor`
  accetta un goal opzionale.

  **OFF per default**, perché cambia cosa fa il modo. Per un target terminale rifiutare di pianificare
  dentro un muro è giusto; per un move DRIVE no, perché quel target è un punto che rotola
  `rrt_corridor_length_drive_m` davanti all'auto e quindi entra in qualunque muro più vicino di così.
  Misurato in `base_room` (muro a 4.61 m per il lidar): senza il flag il planner riesce per 4.1 s, poi
  188 fallimenti consecutivi che bruciano ogni volta tutto `rrt_time_budget_s`, auto parcheggiata a
  2.04 m e missione ABORTED per timeout — riproducibile al millimetro su due run (2.037, 2.038 m). Con
  il flag: un solo fallimento, "target retreated to 2.64 m", COMPLETE a 3.105 m in 6.5 s. Nella scena
  `corridor` non scatta mai (7 piani, 7 risolti): il percorso di default è invariato.

  Costo: un piano fallito fa girare RRT* due volte, quindi il caso peggiore per tick è
  `2 * rrt_time_budget_s`. Si vede nel plan time, da 1.09 ms medi a 12.68 (p95 57.8, max 81.97).

  Provenienza: macchina **work** (`DESKTOP-Q3F81S6`), branch `planner-modes-hil`, commit `3af7d70c`
  del 2026-11-13, portato su `main` con cherry-pick il 2026-11-19 (`archive/work-rrt-p2p-20261119`).
  `Evidence: docs/analysis/mpc-control/2026-11-13_planner-modes-compare/report.md`

### Changed
- Prima versione numerata del modulo (era 0.0.1, mai incrementata): da qui ogni modifica porta un
  bump di versione e una voce in questo file. Nessun'altra modifica al codice rispetto a 0.0.1.
