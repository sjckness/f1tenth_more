---
id: 2026-10-08_first-test-campaign-analysis
title: "M02 avvicinamento al muro, R009-R011: perché ogni run si è fermata"
area: field-tests
date: 2026-10-08
tags: [first-test-campaign, m02-approach-wall, front-distance, stall, speed-floor]
modules: {f1tenth_perception: "0.0.1", mpc_controller: "0.0.0", f1tenth_behavior: "0.0.1", llm: "0.0.1"}
branch: "doors-enter"
commit: "fe58e209b502"
status: open
hypothesis: "Spiegare perché ciascuna delle run R009-R011 (muro, guard front_clearance < 2.0 m, 0.4 m/s) è terminata come è terminata."
result: "R009 guard scattata su una persona (~1.5 m) con muro a >=3.6 m; R010 ok (guard sul muro a 1.7 m); R011 auto ferma 32 s con comando 0.087-0.098 m/s (parte solo da 0.12-0.14 m/s), reverse clampato a 0."
decision: "Prossimi passi: loggare front_distance e debug raw (front_clearance_publish_debug_raw: true), riprodurre R009 da fermi; il floor min_moving_speed_mps non copre le drive move; il piano si ferma a 2.0 m non 0.5 m."
raw_data: "git exp/2026-10-08_first-test-campaign-analysis:first_test_campaign/analysis/M01/; exp/2026-10-08_first-test-campaign-analysis:first_test_campaign/analysis/M02/; exp/2026-10-08_first-test-campaign-analysis:first_test_campaign/analysis/M02_20261008_R011_camera_roi.png; exp/2026-10-08_first-test-campaign-analysis:first_test_campaign/analysis/M02_20261019_R012-R020/ (2 file .json/.txt); exp/2026-10-08_first-test-campaign-analysis:first_test_campaign/analysis/M03/; exp/2026-10-08_first-test-campaign-analysis:first_test_campaign/analysis/M04/; +7 gruppi (vedi _inventory.md). File >1 MB non copiati: first_test_campaign/analysis/M02_20261008_R011_camera_roi.png. Esterni: dati per run della campagna (manifest, csv, jsonl, plot) in main:first_test_campaign/M01_forward_3m..M05_turn_left_wall/ e, con il nome storico, jazzy:first_test_campaing/; bag nell'archivio dell'auto"
migrated_from: "doors-enter:first_test_campaign/analysis/"
migrated_on: 2026-11-18
---
# M02 avvicinamento al muro, R009-R011: perché ogni run si è fermata

> Report scritto durante la migrazione (2026-11-18): questo esperimento non aveva un report proprio. Il contenuto viene solo dai file elencati sotto; "n/d" dove i file non lo dicono.

## Setup

Dati originali: `doors-enter:first_test_campaign/analysis/`.

## Metodo

Spiegare perché ciascuna delle run R009-R011 (muro, guard front_clearance < 2.0 m, 0.4 m/s) è terminata come è terminata.

## Risultati

R009 guard scattata su una persona (~1.5 m) con muro a >=3.6 m; R010 ok (guard sul muro a 1.7 m); R011 auto ferma 32 s con comando 0.087-0.098 m/s (parte solo da 0.12-0.14 m/s), reverse clampato a 0.

- Guard su camera /perception/front_distance: la persona è trapelata (R009), valore non osservabile perché non registrato
- Sotto 0.12-0.14 m/s l'auto non si muove (R011: 32 s fermo, LiDAR 4.48-4.49 m)

## Conclusioni

Prossimi passi: loggare front_distance e debug raw (front_clearance_publish_debug_raw: true), riprodurre R009 da fermi; il floor min_moving_speed_mps non copre le drive move; il piano si ferma a 2.0 m non 0.5 m.

## Prossimi passi

- Motivo pixel-level del leak persona in front_distance (ROI 180x70, percentili, mask non allineata nel tempo)
- Floor di velocità minima assente sulle drive move; nessuna gestione stuck/blocked per la straight move

## File migrati

- [figures/M01/dashboard.png](figures/M01/dashboard.png) (da `first_test_campaign/analysis/M01/dashboard.png`)
- [data/M01/mission_table.csv](data/M01/mission_table.csv) (da `first_test_campaign/analysis/M01/mission_table.csv`)
- [data/M01/mission_table.tsv](data/M01/mission_table.tsv) (da `first_test_campaign/analysis/M01/mission_table.tsv`)
- [figures/M01/overview_map.png](figures/M01/overview_map.png) (da `first_test_campaign/analysis/M01/overview_map.png`)
- [data/M01/run_table.csv](data/M01/run_table.csv) (da `first_test_campaign/analysis/M01/run_table.csv`)
- [data/M01/run_table.tsv](data/M01/run_table.tsv) (da `first_test_campaign/analysis/M01/run_table.tsv`)
- [data/M01/summary.csv](data/M01/summary.csv) (da `first_test_campaign/analysis/M01/summary.csv`)
- [figures/M02/dashboard.png](figures/M02/dashboard.png) (da `first_test_campaign/analysis/M02/dashboard.png`)
- [data/M02/mission_table.csv](data/M02/mission_table.csv) (da `first_test_campaign/analysis/M02/mission_table.csv`)
- [data/M02/mission_table.tsv](data/M02/mission_table.tsv) (da `first_test_campaign/analysis/M02/mission_table.tsv`)
- [figures/M02/overview_map.png](figures/M02/overview_map.png) (da `first_test_campaign/analysis/M02/overview_map.png`)
- [data/M02/run_table.csv](data/M02/run_table.csv) (da `first_test_campaign/analysis/M02/run_table.csv`)
- [data/M02/run_table.tsv](data/M02/run_table.tsv) (da `first_test_campaign/analysis/M02/run_table.tsv`)
- [data/M02/summary.csv](data/M02/summary.csv) (da `first_test_campaign/analysis/M02/summary.csv`)
- [M02_20261008_R009-R011_notes.md](M02_20261008_R009-R011_notes.md) (da `first_test_campaign/analysis/M02_20261008_R009-R011_notes.md`)
- [figures/M02_20261008_R009-R011_why_it_stopped.png](figures/M02_20261008_R009-R011_why_it_stopped.png) (da `first_test_campaign/analysis/M02_20261008_R009-R011_why_it_stopped.png`)
- [data/M02_20261019_R012-R020/campaign_results_R012-R020.csv](data/M02_20261019_R012-R020/campaign_results_R012-R020.csv) (da `first_test_campaign/analysis/M02_20261019_R012-R020/campaign_results_R012-R020.csv`)
- [figures/M02_20261019_R012-R020/dashboard.png](figures/M02_20261019_R012-R020/dashboard.png) (da `first_test_campaign/analysis/M02_20261019_R012-R020/dashboard.png`)
- [figures/M02_20261019_R012-R020/overview_map_M02_approach_wall.png](figures/M02_20261019_R012-R020/overview_map_M02_approach_wall.png) (da `first_test_campaign/analysis/M02_20261019_R012-R020/overview_map_M02_approach_wall.png`)
- [data/M02_20261019_R012-R020/summary.csv](data/M02_20261019_R012-R020/summary.csv) (da `first_test_campaign/analysis/M02_20261019_R012-R020/summary.csv`)
- [figures/M03/dashboard.png](figures/M03/dashboard.png) (da `first_test_campaign/analysis/M03/dashboard.png`)
- [data/M03/mission_table.csv](data/M03/mission_table.csv) (da `first_test_campaign/analysis/M03/mission_table.csv`)
- [data/M03/mission_table.tsv](data/M03/mission_table.tsv) (da `first_test_campaign/analysis/M03/mission_table.tsv`)
- [figures/M03/overview_map.png](figures/M03/overview_map.png) (da `first_test_campaign/analysis/M03/overview_map.png`)
- [data/M03/run_table.csv](data/M03/run_table.csv) (da `first_test_campaign/analysis/M03/run_table.csv`)
- [data/M03/run_table.tsv](data/M03/run_table.tsv) (da `first_test_campaign/analysis/M03/run_table.tsv`)
- [data/M03/summary.csv](data/M03/summary.csv) (da `first_test_campaign/analysis/M03/summary.csv`)
- [figures/M04/dashboard.png](figures/M04/dashboard.png) (da `first_test_campaign/analysis/M04/dashboard.png`)
- [data/M04/mission_table.csv](data/M04/mission_table.csv) (da `first_test_campaign/analysis/M04/mission_table.csv`)
- [data/M04/mission_table.tsv](data/M04/mission_table.tsv) (da `first_test_campaign/analysis/M04/mission_table.tsv`)
- [figures/M04/overview_map.png](figures/M04/overview_map.png) (da `first_test_campaign/analysis/M04/overview_map.png`)
- [data/M04/run_table.csv](data/M04/run_table.csv) (da `first_test_campaign/analysis/M04/run_table.csv`)
- [data/M04/run_table.tsv](data/M04/run_table.tsv) (da `first_test_campaign/analysis/M04/run_table.tsv`)
- [data/M04/summary.csv](data/M04/summary.csv) (da `first_test_campaign/analysis/M04/summary.csv`)
- [figures/M05/dashboard.png](figures/M05/dashboard.png) (da `first_test_campaign/analysis/M05/dashboard.png`)
- [data/M05/mission_table.csv](data/M05/mission_table.csv) (da `first_test_campaign/analysis/M05/mission_table.csv`)
- [data/M05/mission_table.tsv](data/M05/mission_table.tsv) (da `first_test_campaign/analysis/M05/mission_table.tsv`)
- [figures/M05/overview_map.png](figures/M05/overview_map.png) (da `first_test_campaign/analysis/M05/overview_map.png`)
- [data/M05/run_table.csv](data/M05/run_table.csv) (da `first_test_campaign/analysis/M05/run_table.csv`)
- [data/M05/run_table.tsv](data/M05/run_table.tsv) (da `first_test_campaign/analysis/M05/run_table.tsv`)
- [data/M05/summary.csv](data/M05/summary.csv) (da `first_test_campaign/analysis/M05/summary.csv`)
- [figures/M06/dashboard.png](figures/M06/dashboard.png) (da `first_test_campaign/analysis/M06/dashboard.png`)
- [data/M06/mission_table.csv](data/M06/mission_table.csv) (da `first_test_campaign/analysis/M06/mission_table.csv`)
- [data/M06/mission_table.tsv](data/M06/mission_table.tsv) (da `first_test_campaign/analysis/M06/mission_table.tsv`)
- [figures/M06/overview_map.png](figures/M06/overview_map.png) (da `first_test_campaign/analysis/M06/overview_map.png`)
- [data/M06/run_table.csv](data/M06/run_table.csv) (da `first_test_campaign/analysis/M06/run_table.csv`)
- [data/M06/run_table.tsv](data/M06/run_table.tsv) (da `first_test_campaign/analysis/M06/run_table.tsv`)
- [data/M06/summary.csv](data/M06/summary.csv) (da `first_test_campaign/analysis/M06/summary.csv`)
- [figures/M07_chair_then_straight/dashboard.png](figures/M07_chair_then_straight/dashboard.png) (da `first_test_campaign/analysis/M07_chair_then_straight/dashboard.png`)
- [data/M07_chair_then_straight/m07_R001-R004_campaign_results.csv](data/M07_chair_then_straight/m07_R001-R004_campaign_results.csv) (da `first_test_campaign/analysis/M07_chair_then_straight/m07_R001-R004_campaign_results.csv`)
- [data/M07_chair_then_straight/mission_table.csv](data/M07_chair_then_straight/mission_table.csv) (da `first_test_campaign/analysis/M07_chair_then_straight/mission_table.csv`)
- [data/M07_chair_then_straight/mission_table.tsv](data/M07_chair_then_straight/mission_table.tsv) (da `first_test_campaign/analysis/M07_chair_then_straight/mission_table.tsv`)
- [figures/M07_chair_then_straight/overview_map.png](figures/M07_chair_then_straight/overview_map.png) (da `first_test_campaign/analysis/M07_chair_then_straight/overview_map.png`)
- [figures/M07_chair_then_straight/overview_map_M07_chair_then_straight_4m.png](figures/M07_chair_then_straight/overview_map_M07_chair_then_straight_4m.png) (da `first_test_campaign/analysis/M07_chair_then_straight/overview_map_M07_chair_then_straight_4m.png`)
- [data/M07_chair_then_straight/run_table.csv](data/M07_chair_then_straight/run_table.csv) (da `first_test_campaign/analysis/M07_chair_then_straight/run_table.csv`)
- [data/M07_chair_then_straight/run_table.tsv](data/M07_chair_then_straight/run_table.tsv) (da `first_test_campaign/analysis/M07_chair_then_straight/run_table.tsv`)
- [figures/M07_chair_then_straight/single_R001.png](figures/M07_chair_then_straight/single_R001.png) (da `first_test_campaign/analysis/M07_chair_then_straight/single_R001.png`)
- [figures/M07_chair_then_straight/single_R002.png](figures/M07_chair_then_straight/single_R002.png) (da `first_test_campaign/analysis/M07_chair_then_straight/single_R002.png`)
- [figures/M07_chair_then_straight/single_R003.png](figures/M07_chair_then_straight/single_R003.png) (da `first_test_campaign/analysis/M07_chair_then_straight/single_R003.png`)
- [figures/M07_chair_then_straight/single_R004.png](figures/M07_chair_then_straight/single_R004.png) (da `first_test_campaign/analysis/M07_chair_then_straight/single_R004.png`)
- [data/M07_chair_then_straight/summary.csv](data/M07_chair_then_straight/summary.csv) (da `first_test_campaign/analysis/M07_chair_then_straight/summary.csv`)
- [figures/M08/dashboard.png](figures/M08/dashboard.png) (da `first_test_campaign/analysis/M08/dashboard.png`)
- [data/M08/mission_table.csv](data/M08/mission_table.csv) (da `first_test_campaign/analysis/M08/mission_table.csv`)
- [data/M08/mission_table.tsv](data/M08/mission_table.tsv) (da `first_test_campaign/analysis/M08/mission_table.tsv`)
- [figures/M08/overview_map.png](figures/M08/overview_map.png) (da `first_test_campaign/analysis/M08/overview_map.png`)
- [data/M08/run_table.csv](data/M08/run_table.csv) (da `first_test_campaign/analysis/M08/run_table.csv`)
- [data/M08/run_table.tsv](data/M08/run_table.tsv) (da `first_test_campaign/analysis/M08/run_table.tsv`)
- [data/M08/summary.csv](data/M08/summary.csv) (da `first_test_campaign/analysis/M08/summary.csv`)
- [figures/dashboard.png](figures/dashboard.png) (da `first_test_campaign/analysis/dashboard.png`)
- [figures/overview_map.png](figures/overview_map.png) (da `first_test_campaign/analysis/overview_map.png`)
- [figures/overview_map_M01_forward_3m.png](figures/overview_map_M01_forward_3m.png) (da `first_test_campaign/analysis/overview_map_M01_forward_3m.png`)
- [figures/overview_map_M02_approach_wall.png](figures/overview_map_M02_approach_wall.png) (da `first_test_campaign/analysis/overview_map_M02_approach_wall.png`)
- [figures/overview_map_M02_approach_wall_last5.png](figures/overview_map_M02_approach_wall_last5.png) (da `first_test_campaign/analysis/overview_map_M02_approach_wall_last5.png`)
- [figures/overview_map_M04_person.png](figures/overview_map_M04_person.png) (da `first_test_campaign/analysis/overview_map_M04_person.png`)
- [figures/overview_map_M05_turn_left_wall_R3-5-6-7-8.png](figures/overview_map_M05_turn_left_wall_R3-5-6-7-8.png) (da `first_test_campaign/analysis/overview_map_M05_turn_left_wall_R3-5-6-7-8.png`)
- [figures/overview_map_P001_R006-R011.png](figures/overview_map_P001_R006-R011.png) (da `first_test_campaign/analysis/overview_map_P001_R006-R011.png`)
- [figures/overview_map_P003_R004-R005-R006-R009-R010.png](figures/overview_map_P003_R004-R005-R006-R009-R010.png) (da `first_test_campaign/analysis/overview_map_P003_R004-R005-R006-R009-R010.png`)
- [figures/overview_map_P004_R007-R011.png](figures/overview_map_P004_R007-R011.png) (da `first_test_campaign/analysis/overview_map_P004_R007-R011.png`)
- [figures/overview_map_P004_R014-R018.png](figures/overview_map_P004_R014-R018.png) (da `first_test_campaign/analysis/overview_map_P004_R014-R018.png`)
- [figures/overview_map_P005_R001-R003.png](figures/overview_map_P005_R001-R003.png) (da `first_test_campaign/analysis/overview_map_P005_R001-R003.png`)
- [figures/overview_map_P005_R003-R005-R006.png](figures/overview_map_P005_R003-R005-R006.png) (da `first_test_campaign/analysis/overview_map_P005_R003-R005-R006.png`)
- [data/run_table.csv](data/run_table.csv) (da `first_test_campaign/analysis/run_table.csv`)
- [data/run_table.tsv](data/run_table.tsv) (da `first_test_campaign/analysis/run_table.tsv`)
- [data/selected/campaign_results_selected.csv](data/selected/campaign_results_selected.csv) (da `first_test_campaign/analysis/selected/campaign_results_selected.csv`)
- [figures/selected/dashboard.png](figures/selected/dashboard.png) (da `first_test_campaign/analysis/selected/dashboard.png`)
- [figures/selected/overview_map.png](figures/selected/overview_map.png) (da `first_test_campaign/analysis/selected/overview_map.png`)
- [figures/selected/overview_map_M01_20260921_R001-R005.png](figures/selected/overview_map_M01_20260921_R001-R005.png) (da `first_test_campaign/analysis/selected/overview_map_M01_20260921_R001-R005.png`)
- [figures/selected/overview_map_M01_20260930_R007-R011.png](figures/selected/overview_map_M01_20260930_R007-R011.png) (da `first_test_campaign/analysis/selected/overview_map_M01_20260930_R007-R011.png`)
- [figures/selected/overview_map_M01_forward_3m.png](figures/selected/overview_map_M01_forward_3m.png) (da `first_test_campaign/analysis/selected/overview_map_M01_forward_3m.png`)
- [figures/selected/overview_map_M02_approach_wall.png](figures/selected/overview_map_M02_approach_wall.png) (da `first_test_campaign/analysis/selected/overview_map_M02_approach_wall.png`)
- [figures/selected/overview_map_M03_bottle.png](figures/selected/overview_map_M03_bottle.png) (da `first_test_campaign/analysis/selected/overview_map_M03_bottle.png`)
- [figures/selected/overview_map_M04_20260924_R007-R011.png](figures/selected/overview_map_M04_20260924_R007-R011.png) (da `first_test_campaign/analysis/selected/overview_map_M04_20260924_R007-R011.png`)
- [figures/selected/overview_map_M04_20261001_R014-R018.png](figures/selected/overview_map_M04_20261001_R014-R018.png) (da `first_test_campaign/analysis/selected/overview_map_M04_20261001_R014-R018.png`)
- [figures/selected/overview_map_M04_person.png](figures/selected/overview_map_M04_person.png) (da `first_test_campaign/analysis/selected/overview_map_M04_person.png`)
- [figures/selected/overview_map_M05_turn_left_wall.png](figures/selected/overview_map_M05_turn_left_wall.png) (da `first_test_campaign/analysis/selected/overview_map_M05_turn_left_wall.png`)
- [data/selected/summary.csv](data/selected/summary.csv) (da `first_test_campaign/analysis/selected/summary.csv`)
- [data/summary.csv](data/summary.csv) (da `first_test_campaign/analysis/summary.csv`)
- [data/summary_table.csv](data/summary_table.csv) (da `first_test_campaign/analysis/summary_table.csv`)
- [data/summary_table.tsv](data/summary_table.tsv) (da `first_test_campaign/analysis/summary_table.tsv`)
