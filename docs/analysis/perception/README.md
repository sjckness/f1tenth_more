# Area: perception (Percezione)

Camera ZED, YOLO, maschere di scena e self mask, vetro, porte, semantic layer e ostacoli.

## 1. Stato attuale

> Semantic layer e ostacoli unknown validati su replay; porte, vetro e parte stereo mai validati su dati reali.

- Semantic layer unito (memoria + unknown): su 71 bag, 0 cicli scoperti su 243 tracce confermate.
- Ostacoli v2: un solo tracker per oggetti riconosciuti e isole LiDAR; solve MPC con 8 ostacoli p95 9.37 ms; percorso di upgrade a classe mai eseguito su dati reali (camera morta nei bag).
- Scene labels: parte LiDAR validata (errore max 0.014 m), parte stereo non validabile (nessuna depth nei bag); default spento.
- Porte: né il rilevatore sul tracker muri né quello dal semantic layer confermano porte vere sul corridor10m; il detector freespace trova 28 porte "strong" senza ground truth; `enter_door` si ferma al bordo.
- Vetro dall'intensità LiDAR: non decidibile, 1 sola traccia vetro utile.
- Self mask: la proposta copre l'hardware con il 7.84 % del frame invece del 14.78 %; patch non applicata.
- YOLO su GPU: ricognizione fatta, mancano dati con persone per il gate di parità (28 frame, 0 persone).

## 2. Decisioni prese

- `enable_scene_labels` resta false ([2026-10-13_scene-labels-replay](2026-10-13_scene-labels-replay/report.md))
- MPC prende ostacoli solo da `/costmap/semantic_obstacles` (max 8, scadenza 1.0 s); `unknown_objects_mode` off/shadow/active ([2026-10-19_obstacles-v2](2026-10-19_obstacles-v2/report.md))
- Porte come overlay sui muri; ruolo `other` rinominato `unknown` ([2026-10-20_doors-wall-tracker](2026-10-20_doors-wall-tracker/report.md))
- Porte su `/costmap/semantic_doors`, mai su `semantic_objects` ([2026-10-28_semantic-doors](2026-10-28_semantic-doors/report.md))
- `glass_score_lidar` solo come secondo parere ([2026-10-16_glass-lidar](2026-10-16_glass-lidar/report.md))

## 3. Domande aperte / prossimi esperimenti

- Una bag con depth e immagini (corridoio e porta reale) sblocca scene labels, porte stereo, foot dei muri e upgrade degli ostacoli.
- Registrazione con persone per il gate di parità YOLO GPU; engine TensorRT di segmentazione.
- Applicare o no la patch della self mask.
- Run davanti al vetro a incidenza < 5 gradi con muro di controllo.

## 4. Esperimenti

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| data | esperimento | status | risultato | moduli (versione) |
|---|---|---|---|---|
| 2026-10-28 | [Rilevamento porte dal semantic layer (replay e freespace)](2026-10-28_semantic-doors/report.md) | open | Rilevatore da muri su corridor10m_20261016T144537: 57 935 gap, 0 porte (nessun gap largo 0.70-1.40 m). Freespace su corridor10m_20261019T131815: 506 cicli, 12.9 ms, 56 confermate e 28 strong (medial axis: 0); con spread… | f1tenth_costmap 0.0.1, f1tenth_behavior 0.0.1, f1tenth_perception 0.0.1 |
| 2026-10-22 | [Ricognizione nodo YOLO per ridurre la CPU (GPU/TensorRT)](2026-10-22_yolo-gpu-recon/report.md) | open | Orin: torch 2.11.0 cu126, TensorRT 10.3.0, ultralytics 8.4.90 identico a linus, nessun engine di segmentazione; dati offline per il parity gate: 28 frame 640x360 senza persone (corridor10m_20261015T171753). | f1tenth_perception 0.0.1 |
| 2026-10-22 | [Self mask: com'è oggi, quanto costa, cosa metterci](2026-10-22_self-mask-proposal/report.md) | open | Maschera attuale 14.78 % del frame (34 057 px), hardware reale 3.61 %; proposta 7.84 % (18 054 px): -6.95 punti, copre 100 % dell'hardware, 0.0 % di scena nuova nascosta. La via depth non funziona (vetro e pavimento sen… | f1tenth_perception 0.0.1, f1tenth_params 0.0.1 |
| 2026-10-20 | [Replay offline del semantic layer unito (memoria + unknown)](2026-10-20_semantic-merge-replay/report.md) | done | 71 bag su 72 rigiocate: 113 tracce persona e 243 totali confermate, 94/193 passate in MEMORY, 7/13 hand-over al LiDAR, cicli di buco 0 su entrambe. | f1tenth_costmap 0.0.1, f1tenth_perception 0.0.1 |
| 2026-10-20 | [Rilevamento porte sul tracker multi-muro (door-fixes)](2026-10-20_doors-wall-tracker/report.md) | open | Replay offline su corridor10m_20261016T144537 (700 cicli): nessuna porta confermata dopo l'addendum, nessun avvistamento positivo; 2574 gap giudicati, codice dominante width (1546). Prior Manhattan 83.2 gradi, forza 1.0… | f1tenth_perception 0.0.1, f1tenth_messages 0.0.1, f1tenth_params 0.0.1 |
| 2026-10-19 | [Obstacles v2: muri a righe, ogni ostacolo nel semantic layer](2026-10-19_obstacles-v2/report.md) | open | Bag corridoio: 13 oggetti sconosciuti confermati per frame (99 nel run), 0 falsi speed cap, nessuno entro 0.48 m delle 312 traiettorie; solve MPC p95 con 8 ostacoli 9.37 ms (0.99x, budget 1.3x); passo LiDAR ~2 ms su lin… | f1tenth_costmap 0.0.1, mpc_controller 0.0.0, f1tenth_params 0.0.1 |
| 2026-10-16 | [Vetro dall'intensita LiDAR: fattibilita su corridor10m](2026-10-16_glass-lidar/report.md) | open | Non decidibile: 1 sola traccia vetro (3 finestre vs 49); intensita x range^2 AUC 0.083, pass-through 0.000 (il LiDAR non vede attraverso a 26-45 deg); nessun campione a incidenza <5 deg. | f1tenth_perception 0.0.1 |
| 2026-10-13 | [Scene labels: replay offline su bag archiviati](2026-10-13_scene-labels-replay/report.md) | open | Parte LiDAR validata: ON vs riferimento LiDAR max 0.014 m (a), 0.008 m (b), persona mai come muro (0 tick); CPU 6.2-7.3 % di un core a ~40 Hz. Parte stereo non validabile: nessuna depth nei bag. | f1tenth_perception 0.0.1 |
<!-- AUTO:END -->
