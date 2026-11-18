# Changelog: `f1tenth_costmap`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_costmap`).

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
2026-08-21 al 2026-10-23. Voci ricostruite da `git log main -- src/f1tenth_costmap` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Corridoio sicuro convesso (politopo) in `costmap_boundary_node`, spento di default (`f18abf9`, `6302178`; 2026-09-09).
- Layer semantico: incertezza di posizione/classe da segmentazione, `semantic_tracks` con timestamp di cattura, layer muri semantici a lastra (`96c6bbc`, `86f48b3`, `e3793e4`; 2026-10-15).
- Filtri di lunghezza e persistenza sulle righe della costmap e oggetti tracciati, spenti di default (`36f45e7`; 2026-10-19).
- Ostacoli LiDAR sconosciuti: proprietà delle celle, candidati isola, tracker unico per oggetti riconosciuti e sconosciuti, `unknown_objects_mode`, tetto di velocità, dischi a catena per ostacoli allungati; un solo percorso ostacoli verso l'MPC (`88cfc1c`, `6ece508`, `79562b1`, `67beb26`; 2026-10-19). Evidence: docs/analysis/perception/2026-10-19_obstacles-v2/report.md
- Memoria semantica degli oggetti con decadimento sensibile alla visibilità, un solo gate per oggetti vivi e ricordati (`c33f969`, `6f48d19`; 2026-10-20).

### Changed
- R3: correzioni CPU in Python (`ceb7c884`; 2026-10-23).
- Distanze frontali misurate dal frame `base_front` (paraurti); un punto a lato dell'auto è escluso invece di essere clampato (`10d3eb3`, `d8609b5`; 2026-10-13).
- Consumatori: CLEAR guida, NO_DATA tiene, 50 m non raggiunge mai il solver (`796f369`; 2026-10-15).
- Frequenze ridotte: `costmap_boundary` a 10 Hz, layer muri semantico a 5 Hz, `reset_manager` senza lavoro a vuoto (`752f160`; 2026-10-16). Evidence: docs/analysis/platform/2026-10-16_cpu-cleanup/report.md
- `go_to_object`: distanza come gap con minimo derivato dal controllore (`edd0e6b`; 2026-09-17).
- Ruolo `unknown` al posto di `other` nel tracker muri (`c1b4883`; 2026-10-20).

### Fixed
- Ciclo infinito di `inflate_polytope` con numpy 1.26 (`0f767f03`; 2026-10-23). Evidence: docs/analysis/platform/2026-10-01_thor-phase0/report.md
- Nella griglia semantica `other` è una cella a sé e non viene più contata come `back` (`b5a97a3`; 2026-10-16).
- Predicato unico di e-stop e hardening Humble (chiave `tf_static_only`) (`a87e828`; 2026-10-16).
