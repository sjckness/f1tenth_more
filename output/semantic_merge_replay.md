# Replay offline del semantic layer unito (memoria + oggetti unknown)

2026-10-20, branch `cf-next`. Strumento: `tools/semantic_merge_replay.py`.

## Cosa fa lo strumento

Rigioca su bag registrate le **funzioni vere** di `semantic_layer_node` dopo
il merge di obstacles-v2, con i valori di `stack_params.yaml`. Non usa ROS:
decodifica come `tools/obstacles_v2_replay.py` (rosbags + i `.msg` del repo).

- **Ogni batch di detection** passa da `update_tracks_batch`, con la memoria
  attiva (`prune_confirmed=False`), lo z dal transform della camera, poi
  `dedupe_tracks`.
- **A 10 Hz, sul timestamp dello scan**, due cicli in sequenza:
  - il ciclo LiDAR: ownership delle celle, candidati, `update_unknown_batch`
    con il filtro di plausibilità, dedupe;
  - il ciclo memoria (`memory_cycle`): visibilità LiDAR e camera calcolate
    come nel nodo, frustum dai `/tf_static`, "camera viva" se un batch è
    arrivato entro `semantic_layer_sensor_stale_s`. Il passaggio al LiDAR
    (`hand_over_to_lidar`) parte quando la memoria lascia una traccia.

**Copertura.** Per ogni oggetto riconosciuto confermato si guarda, a ogni
ciclo, se l'MPC lo avrebbe:
- *live* (riconosciuto e corrente);
- *memory* (in MEMORY e plausibile);
- *unknown(hand-over)* (lo stesso id dopo il passaggio al LiDAR).

Un **buco** è un ciclo, tra il primo e l'ultimo ciclo coperto, in cui
l'oggetto non è coperto.

```bash
python3 tools/semantic_merge_replay.py --out <dir> BAG [BAG ...]
```

## Su quali run

Su linus ci sono 72 bag di missioni con `/camera/detections_3d`, `/scan` e
`/ekf_global/odometry/filtered`, tutte in `~/f1tenth_archive/mission_row/`
(missioni `bottle_then_person`, `llm_*`, `wall_turn`).
- 71 sono state rigiocate;
- 1 è illeggibile (`2026-09-03T12-45-06_mission-bottle_then_person`: il
  `.db3` non ha la tabella `topics`).

La bag della campagna recente (`corridor10m_20261016T144537` in `~/bags`) non
ha la posa globale, quindi non serve qui. I dati della campagna di ottobre
sono sull'Orin.

## Risultati

| | Persone | Tutte le classi |
|---|---|---|
| Tracce riconosciute confermate | 113 | 243 |
| Passate in MEMORY almeno una volta | 94 | 193 |
| Passate al LiDAR come unknown confermato (hand-over) | 7 | 13 |
| Cicli di buco | **0** | **0** |

- **Il passaggio al LiDAR funziona senza buchi.** Esempio,
  `2026-09-02T15-15-50_mission-bottle_then_person`, traccia 0 (persona):
  - live da t = 1788362155.2;
  - memory da 158.98, quando esce dal campo della camera mentre l'auto gira;
  - dopo i 15 s di memoria, a 173.99, il LiDAR la vede ancora: diventa
    unknown confermato, **stesso id e stessa posizione** (3.77, 1.34);
  - resta coperta altri 6.3 s, finché il LiDAR non vede più niente lì.
- **Lo stesso id torna riconosciuto.** In
  `2026-09-17T12-59-58_mission-llm_f7c3305e21e1`, traccia 13: live, poi
  unknown(hand-over), poi YOLO la rivede e torna live con lo stesso id
  (upgrade di v2), poi memory. Nessun ciclo scoperto.
- **Molto tempo in MEMORY per le persone ai bordi dell'immagine.**
  - Nella bag `2026-10-08T11-27-18` la persona sta a circa −48° nel frame
    della camera, con mezzo campo visivo di 50.5°.
  - La memoria considera "in vista" un oggetto solo se **tutta** la sua
    impronta è nel frustum.
  - Così, tra una detection e l'altra, la traccia alterna live e memory: è
    il comportamento voluto (un oggetto tagliato dal bordo non è una prova
    che sia sparito), e resta coperta.
- **Il filtro di plausibilità unico esclude 8 tracce di persona,** 7 nella
  bag `2026-09-11T14-14-24`, mai coperte.
  - Lo z del centro in base_link è negativo (mediana −0.41 m, minimo
    −1.15 m), una profondità sbagliata o una montatura diversa a settembre;
    altre falliscono sul raggio.
  - È lo stesso scarto che `obstacle_projector_node` faceva sulle detection
    live prima di v2.
  - Le celle LiDAR sotto quelle tracce non vengono assorbite, quindi se
    c'è davvero qualcosa diventa un unknown.

## Cosa NON dice questo replay

- **Non c'è l'MPC**: copertura vuol dire "l'oggetto è nella lista che l'MPC
  riceverebbe", non che la traiettoria lo eviti.
- **Senza StereoColumns**, che le bag di settembre-ottobre non hanno,
  l'occlusione stereo usa solo il verdetto LiDAR.
- **I tempi di calcolo non sono quelli dell'Orin.**
