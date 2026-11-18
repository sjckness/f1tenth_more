# Changelog: `f1tenth_messages`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_messages`).

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
2026-08-13 al 2026-10-22. Voci ricostruite da `git log main -- src/f1tenth_messages` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Pacchetto di sole interfacce: BatteryStatus, BoundaryConstraint(Array), MissionStatus, Obstacle2D(Array), SystemStatus e relativi (`e2dbe78`; 2026-08-13).
- BehaviorTreeStatus, MpcSolverStatus (`96c6bbc`; 2026-09-03) e DriveCommand per mosse aperte, schema 3.0 (`e164f80`; 2026-09-10).
- Famiglia Wall*: WallDistance, WallEstimate, WallLineFit, WallSegment, WallTrack (`e52af56`; 2026-09-14).
- ObjectGoal, ObjectApproachStatus (flag per tick: `inside_turn_radius`, `target_behind`; eco di `track_id` e gap live), DriveClamp, MoveOutcome (`d2e068f`, `f2f18f4`, `22ff031`, `e00cd8d`, `8ce08fa`, `edd0e6b`, `2bed817`, `e5f4731`; 2026-09-17).
- MultiWallTrack(Array), StereoColumns, WallObservation(Array) per tracker multi-muro e fusione LiDAR/stereo (`9191890`, `6a9b843`, `d673824`, `2a2a8c5`, `c44b503`; 2026-10-16).
- Door e DoorArray per `/perception/doors`, con `jamb_height_unknown` e stato CANDIDATE (`3034c94`, `35e9c91`, `a335113`, `48df121`; 2026-10-20). Evidence: docs/analysis/perception/2026-10-28_semantic-doors/report.md
- ReverseStatus e servizio RequestReverse; SemanticObject(Array) (`b5f46ea`, `c33f969`, `6f48d19`; 2026-10-20).
- LinkStatus (copertura Wi-Fi), SpeedChannelStatus (comando di velocità senza offset) e MpcParamUpdate (due LLM) (`4bbe122`, `4701caa`, `bc8eebd`; 2026-10-22).
### Changed
- WallEstimate espone il contratto WALL / CLEAR / NO_DATA; WallLineFit riporta taglio di rilevamento, piani e muro virtuale (`b188f12`, `8b352d2`; 2026-10-15).
- **BREAKING:** il ruolo `other` di MultiWallTrack diventa `unknown` (`c1b4883`; 2026-10-20).
