# Area: field-tests (Prove sul campo e analisi dei run)

Campagne di test sull'auto, analisi dei run e delle bag, sessioni notturne su dati reali.

## 1. Stato attuale

> Prima campagna analizzata (stop spiegati, bag spesso incompleti); dopo il 2026-10-22 nessun nuovo run analizzato qui.

- 2026-09-01: 9 stop su 9 missioni spiegati (corridoio camera 66 %, prossimità LiDAR 33 %, picchi GPU); solver MPC mai in errore.
- 2026-09-02: lo sweep di `w_psi` indica 5.0 (escursione laterale 0.25-0.69 m); falso COMPLETE ricorrente; 7.8 % di QP infeasible.
- Prima campagna (M01-M05): sotto 0.12-0.14 m/s l'auto non si muove; la guardia camera `front_distance` ha preso una persona per il muro (R009).
- campaign-final: tutti i 57 run con manifest girati su un albero sporco senza diff catturato; il virtual wall descritto non esisteva.
- Replay del corridor10m: contratto WALL/CLEAR uguale al registrato sul 61.7-66 % dei cicli, 0 fantasmi nel replay; nel run del 2026-10-16 un muro fantasma di 3.24 s ha molto probabilmente fermato l'auto circa 5 m prima.
- Le bag dei corridor10m sono spesso senza camera o depth: limitano tutte le validazioni stereo.

## 2. Decisioni prese

- Load trip GPU separato e spento di default; soglie di prossimità 0.15 m ([2026-09-01_mission-analysis](2026-09-01_mission-analysis/report.md))
- `w_psi` = 5.0 ([2026-09-02_mission-live-validation](2026-09-02_mission-live-validation/report.md))
- Registrazione da LOADED con pre-roll; distanze frontali dal paraurti; dt MPC 0.2 s ([2026-10-13_campaign-final-report](2026-10-13_campaign-final-report/report.md))
- Recorder opt-in, tolleranze rect = parametri delle colonne ([2026-10-16_nightly-campaign-final](2026-10-16_nightly-campaign-final/report.md))
- D7: tenere V1 del wall tracker (nessun cambio) ([2026-10-19_run-corridor10m-131815](2026-10-19_run-corridor10m-131815/report.md))

## 3. Domande aperte / prossimi esperimenti

- Una bag di corridoio con camera e depth (verificare `record_zed_depth`).
- Perché il replay vede meno WALL del registrato (5.95 s contro 35.91 s).
- Floor di velocità sulle drive move e gestione stuck della mossa dritta.
- Nuovi run sull'auto con le modifiche di novembre (MPC dodge, offset_free, wall lock).

## 4. Esperimenti

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| data | esperimento | status | risultato | moduli (versione) |
|---|---|---|---|---|
| 2026-10-21 | [Notte 2026-10-21: muri, reverse, app, LLM2, report run](2026-10-21_night-streams/report.md) | done | campaign-final e main a 4370897; two-llm-arch b89a3bd; build Humble su Orin 41/41 pacchetti. Reverse: causa del timeout era il conteggio distanza via ancora mappa (non il VESC); LLM2 mock: 151/151 risposte valide, laten… | f1tenth_perception 0.0.1, mpc_controller 0.0.0, f1tenth_gateway 0.0.1, f1tenth_ctl 0.0.1, llm 0.0.1, f1tenth_logger 0.0.1, f1tenth_params 0.0.1 |
| 2026-10-19 | [Analisi offline della run corridor10m_20261019T131815](2026-10-19_run-corridor10m-131815/report.md) | open | Bag senza camera (depth/stereo_columns/scene_labels 0 msg); /scan 33.1 Hz vs 40; fronte 97.1 % WALL, errore mediano 0.015 m, p90 0.094 m; V3 muro destro 43.7 % -> 84.3 % ma tracce corte 43 -> 110 e +9.6 ms. | f1tenth_perception 0.0.1, f1tenth_logger 0.0.1 |
| 2026-10-16 | [Analisi offline run corridor10m_20261016T144537](2026-10-16_run-corridor10m-144537/report.md) | done | Bag pulito (/scan 39.93 Hz, nessun buco); maschera migliorata (invalid 0.24->0.06, mask tick 83->33 ms p50). Muro fantasma di 3.24 s (t=147.0-150.24) con errore -4.905 m: la macchina molto probabilmente si e' fermata ~5… | f1tenth_perception 0.0.1, f1tenth_costmap 0.0.1 |
| 2026-10-16 | [Sessione notturna campaign-final (F1-F6, G): stereo, header aliasing, recorder, glass](2026-10-16_nightly-campaign-final/report.md) | done | Header aliasing: /perception/front_distance 0.229 m corto sul 67.6 % dei frame (163/241) ma colonne bit-identiche; tracker 46.76 -> 44.20 ms p50 (0.61 ms sopra target 43.59); vetro: 1 sola traccia dopo windowing, nessun… | f1tenth_perception 0.0.1, corridor_perception 0.0.1, f1tenth_costmap 0.0.1, f1tenth_bringup 0.0.1, f1tenth_logger 0.0.1 |
| 2026-10-16 | [Replay offline di corridor10m: contratto WALL/CLEAR e tracker muri](2026-10-16_corridor10m-replays/report.md) | open | Stato contratto identico su 136/206 cicli (66.0 %) in baseline, 127/206 (61.7 %) in after; wall->clear 67-76 cicli, quasi tutti spiegati dal cap max_published=4. Nessun episodio fantasma (0). | f1tenth_perception 0.0.1 |
| 2026-10-13 | [Report campaign-final: commit della campagna e parti B-F, virtual wall](2026-10-13_campaign-final-report/report.md) | open | Tutti i 57 run con manifest sono dirty (branch test-logging); 17 bag su 58 vuoti, avvio mediano in ritardo 8.2 s. Il virtual wall descritto non esiste; A4 wall_confirm.py e' sana con correzioni. | f1tenth_logger 0.0.1, f1tenth_perception 0.0.1, f1tenth_costmap 0.0.1, mpc_controller 0.0.0, f1tenth_behavior 0.0.1, f1tenth_description 0.0.0, f1tenth_params 0.0.1 |
| 2026-10-09 | [Fix campagna M02 R009-R011: Part A (MPC dt, corridoio, keep-out, BLOCKED) e backlog Part B](2026-10-09_campaign-fix-m02/report.md) | open | Part A (A1-A5) completa, verificata solo su linus (unit test, rig nominale, harness 32 run): nessun comando in (0,0.4) m/s, nessun flip del lato di passaggio; oggetti 1.0 m passano solo con true-centre reporting | mpc_controller 0.0.0, f1tenth_params 0.0.1, f1tenth_perception 0.0.1, f1tenth_behavior 0.0.1, f1tenth_logger 0.0.1, llm 0.0.1 |
| 2026-10-08 | [M02 avvicinamento al muro, R009-R011: perché ogni run si è fermata](2026-10-08_first-test-campaign-analysis/report.md) | open | R009 guard scattata su una persona (~1.5 m) con muro a >=3.6 m; R010 ok (guard sul muro a 1.7 m); R011 auto ferma 32 s con comando 0.087-0.098 m/s (parte solo da 0.12-0.14 m/s), reverse clampato a 0. | f1tenth_perception 0.0.1, mpc_controller 0.0.0, f1tenth_behavior 0.0.1, llm 0.0.1 |
| 2026-09-02 | [Validazione live missioni 2026-09-02: 13 run, sweep di w_psi](2026-09-02_mission-live-validation/report.md) | done | I 13 run sono uno sweep di w_psi (1.0/2.0/5.0/10.0), non una validazione: escursione laterale max 0.49-2.29 m a 1.0 contro 0.25-0.69 m a 5.0. Falso COMPLETE ricorso; 7.8 % solve QP infeasible (baseline 0/1285). | mpc_controller 0.0.0, f1tenth_behavior 0.0.1 |
| 2026-09-01 | [Analisi missioni 2026-09-01: perché l'auto si ferma](2026-09-01_mission-analysis/report.md) | done | MPC 1285/1285 QP risolti, 0 deadline mancate (solve mediana 12.4 ms). 9 episodi di stop = 64.5 s su 153.3 s (42 %): corridoio camera 66 %, prossimità lidar 33 %, picchi GPU 3 episodi (4.0 s); 0 inspiegati. | f1tenth_behavior 0.0.1, mpc_controller 0.0.0, f1tenth_costmap 0.0.1, f1tenth_perception 0.0.1, f1tenth_logger 0.0.1, f1tenth_messages 0.0.1 |
<!-- AUTO:END -->
