# Area: localization-frames (Localizzazione, frame e muri)

map/odom, EKF, `/tf`, frame del veicolo, tracker dei muri e ancoraggio del corridoio, visualizzazione dei muri.

## 1. Stato attuale

> Tracker muri corretto sul muro lontano (P0), foot da depth non superato (P1); wall lock solo su replay.

- Regressione del muro lontano chiusa: con la confidenza range-aware il tempo in WALL sul corridor10m passa da 2.88 s a 69.00 s (auto 68.61 s), errore p95 da 1.217 a 0.004 m.
- Il foot del muro dalla depth non passa il gate P1: residuo 0.187 m contro 0.10 m.
- Front-wall lock (stop agganciato al muro della mossa): coperto da unit test e replay, mai provato sull'auto né su una schivata; con la conferma più stretta FRONT è tenuto sul 51 % dei cicli contro 82 %.
- `/viz/walls` pubblicato dal nodo a 5 Hz solo con subscriber; colori diversi da `/perception/walls/markers`.
- La parità della localizzazione su Jazzy (EKF, `slam_pose_relay`) sta in [platform](../platform/README.md).
- Odometria: `/sensors/core.distance_traveled` sale con ERPM 0 (9807 tick); drift circa 17 % citato nel wall lock: n/d sulla causa.

## 2. Decisioni prese

- Fix P0 della confidenza con 3 parametri sintetici (`range_ref` 5.0 m, `min_inlier_ref` 8.0, `max_rms_scale` 6.0) ([2026-10-18_wall-tracker-v2](2026-10-18_wall-tracker-v2/report.md))
- Mai agganciare un FRONT solo stereo; la rampa MPC mira al muro agganciato ([2026-10-22_front-wall-lock](2026-10-22_front-wall-lock/report.md))
- `/viz/walls` pubblicato dal nodo, topic dei marker nella whitelist del bridge ([2026-10-16_viz-walls](2026-10-16_viz-walls/report.md))

## 3. Domande aperte / prossimi esperimenti

- Il foot del muro richiede una classe wall vera, pavimento inferito oltre 3 m o un pitch camera misurato.
- FRONT senza range cap (episodio a 18.9 m).
- Registrare un M02 con persona nel percorso e rigiocarlo con `wall_lock_replay.py`.
- Allineare i colori dei muri tra `/viz/walls` e `/perception/walls/markers`.

## 4. Esperimenti

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| data | esperimento | status | risultato | moduli (versione) |
|---|---|---|---|---|
| 2026-10-22 | [Front-wall lock per 'avvicinati al muro'](2026-10-22_front-wall-lock/report.md) | open | Replay: lock su corridor10m 4->103->128 (0.01 m/1.7 deg); primo lock stereo-only fallito su bf7938 (traccia 321 rifiutata a 0.88 m/25.7 deg); conferma piu lenta: FRONT tenuto sul 51% dei 796 cicli vs 82%. | f1tenth_perception 0.0.1, f1tenth_behavior 0.0.1, mpc_controller 0.0.0 |
| 2026-10-18 | [Regressione muro lontano: confidenza range-aware (P0) e gate P1 non superato](2026-10-18_wall-tracker-v2/report.md) | open | Colpevole d578963 (confidenza con riferimenti assoluti): WALL 2.88 s -> 69.00 s (auto 68.61), accordo 0.124 -> 0.808 (0.948 senza NO_DATA), err p95 1.217 -> 0.004 m; P1 gate NON superato: residuo foot 0.187 m contro 0.1… | f1tenth_perception 0.0.1, f1tenth_params 0.0.1, corridor_perception 0.0.1 |
| 2026-10-16 | [Muri in Foxglove: diagnosi marker /perception/walls/markers e nuovo /viz/walls](2026-10-16_viz-walls/report.md) | done | Topic sano (1845 msg nel bag, ~10 Hz) ma fuori dalla whitelist del bridge; published() restituisce 1-2 track, mai 'other'; ogni array ha DELETEALL (400/400) e lifetime (0,0) su 1356 marker | f1tenth_perception 0.0.1, f1tenth_bringup 0.0.1, f1tenth_params 0.0.1 |
<!-- AUTO:END -->
