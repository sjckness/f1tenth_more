# Changelog: `f1tenth_perception`

Tutte le modifiche rilevanti di questo modulo (`src/f1tenth_perception`).

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
2026-06-19 al 2026-10-22. Voci ricostruite da `git log main -- src/f1tenth_perception` (commit e data tra parentesi), solo i
cambi significativi.

### Added
- Sorgente camera indipendente dalla periferica (webcam/ZED) con inferenza YOLO reale; modello di segmentazione `yolo26s-seg.pt` come default, con script di benchmark e verifica del deploy live (`0d1c90d`, `16432ef`, `96c6bbc`; 2026-09-03).
- Nodi di rilevamento muro dal solo LiDAR: `lidar_front_wall` (fit del muro dal settore frontale), rilevamento di pareti di vetro da una singola scansione, nodo `wall_distance` (stima d_wall) e `swept_clearance` con geometria `swept_corridor` (`1dc7d51`, `65cb0f3`, `822c210`, `0243fcd`, `6871b08`, `286327c`; 2026-09-14). Evidence: docs/analysis/perception/2026-10-16_glass-lidar/report.md
- Dischi ostacolo dimensionati dall'ingombro a terra invece che dall'altezza, con margine per classe (`obstacle_class_margin_m`, poi `{person: 0.5}`); il margine persona non sopravvive più al dropout del campo visivo; la distanza di `go_to_object` è un gap con minimo derivato dal controllore (`4e0391d`, `56ec026`, `edd0e6b`, `bfeb503`; 2026-09-17).
- Etichettatura della scena stereo (maschera di scena una classe per pixel di profondità, a frame intero, senza ROI; profondità degli oggetti solo su OTHER), con watchdog, profiler di stadio e overlay di debug (`5c30b4d`, `a92e133`, `18ad95d`, `0887eeb`; 2026-10-22). Evidence: docs/analysis/perception/2026-10-13_scene-labels-replay/report.md
- Frame `base_front`: ogni distanza frontale è misurata dal paraurti; posizioni di paraurti, LiDAR e ZED dalle misure con metro del 2026-10-13 (`10d3eb3`, `bdc7c93`; 2026-10-13).
- Immagine di debug della percezione a 5 Hz e topic di debug dello stimatore muro, con renderer MP4 lato host (`0334b8c`; 2026-10-13).
- Tracker multi-muro (`multi_wall_tracker`, parametro `use_wall_tracker`, poi attivo di default) con osservazioni di fusione LiDAR/stereo per ciclo, cross-check per colonna, contratto del muro frontale WALL / CLEAR / NO_DATA, muri come piani (marker a lastra verticale e layer semantico), pavimento inferito, ruoli geometrici, gate di larghezza isteretico e conferma a un solo livello (`wall_confirm_*`) (`91918900`, `53f6a9c`, `6a9b843`, `2a2a8c5`, `b188f12`, `e3793e4`, `2e2f100`, `193fe1d`, `c44b5036`, `4ecea87`; 2026-10-21). Evidence: docs/analysis/localization-frames/2026-10-18_wall-tracker-v2/report.md
- Pubblicazione in visualizzazione di tutti i muri tracciati su `/viz/walls`, con builder di marker, durata e sorgente per traccia (`a94f2cb`, `30db6e2`, `bf891be`, `90c51d6`; 2026-10-18). Evidence: docs/analysis/localization-frames/2026-10-16_viz-walls/report.md
- Rilevamento porte sul tracker multi-muro: regola di merge collineare, battente aperto, veto vetro, stipiti per pareti di vetro, nodo dedicato con `/perception/doors` solo confermate e `/viz/doors`; host frontali opzionali (`door_host_front_facing`) (`5d75fa6`, `35e9c91`, `a335113`, `31a1839`, `b2495a3`, `961c996`; 2026-10-20). Evidence: docs/analysis/perception/2026-10-20_doors-wall-tracker/report.md
- Blocco del muro per l'arresto in `front_clearance`: il controllo si aggancia al muro verso cui punta la manovra (`wall_lock_enabled`) (`6002cb0`; 2026-10-21). Evidence: docs/analysis/localization-frames/2026-10-22_front-wall-lock/report.md
- Temporizzazione per stadio (`perception_stage_timing`, default false) e, per `yolo_detector_node`, backend TensorRT con fallback `.pt`, script di export, pipeline GPU (`gpu_pipeline`, default on), limiti thread, sync CUDA bloccante e scarto frame a 15 Hz (`0084422`, `c45a621`, `26c76f6`, `e78f240`, `5792e2b`; 2026-10-22). Evidence: docs/analysis/perception/2026-10-22_yolo-gpu-recon/report.md
### Changed
- Frame del veicolo dalle misure reali: interasse 0.305 m, `base_link` a terra, altezze misurate di LiDAR (z 0.190) e ZED (z 0.205), beccheggio di 2.083 gradi del supporto ZED; la ZED viene lanciata come ZED 2i (`d7b2770`, `f48e4205`, `83e8bb9`, `704dc40`, `1ff0644`; 2026-10-15).
- Prestazioni: QoS "frame più recente" (KEEP_LAST 1, best effort) su immagini e profondità; immagine annotata disegnata e pubblicata solo se sottoscritta e non registrata di default; coda di sync di `detection_3d_node` da 60 a 15; etichette di scena accoppiate alla profondità per timestamp più vicino; la ZED non trasmette più il TF dell'IMU a 179 Hz (`01c6bff`, `f809b1d`, `14f449b`, `4c85668`, `dd04a2b`, `16b2013`; 2026-10-14).
- Costo CPU: `extract_lines` riscritto in C (da 24.6 ms a 1.1 ms a scansione), tracker muri su una scansione su quattro, budget di pubblicazione per ruolo, payload di debug opt-in, `wall_distance_node` spento di default (`enable_wall_distance`), EKF a 20 Hz, muro virtuale solo su nuove stime; oltre 8 m lo stereo non è più evidenza (`c4215787`, `e41f606`, `eaffd16`, `686d9119`, `9786bfc`, `bb297fe`, `05c5898`; 2026-10-16). Evidence: docs/analysis/platform/2026-10-16_cpu-h1-h5/report.md
- Ruolo `unknown` al posto di `other` nel tracker muri, tracce tentative riportate come unknown; il bersaglio della manovra non è mai un ostacolo, un solo gate per oggetti vivi e ricordati (`c1b4883`, `6f48d19`; 2026-10-20).
- Il rilevamento porte è spento di default (`lidar_front_wall_door_detection_enabled: false`) (`fb913d1`; 2026-10-20).
### Fixed
- `front_clearance_node`: un punto a lato dell'auto è escluso invece di essere clampato a 0.01 m; caso "troppo vicino" gestito esplicitamente (`d8609b5`, `6871b08`; 2026-10-13).
- Il gate di confidenza del FRONT non dipende più dalla distanza (il muro lontano risultava CLEAR); una scansione senza trasformata pubblica NO_DATA; nessun FRONT fantasma da pezzi collineari attraverso un'apertura (`6cbda12`, `0c1cd41`, `d578963`; 2026-10-18).
- Un muro stretto dietro una persona rilevata viene ricreato; un muro in coasting vale come muro frontale per al massimo 0.3 s (`61708b6`, `9a1ecc1`; 2026-10-20).
- Test della configurazione di lancio allineato al default segmentazione (`89a9995`; 2026-09-03).
