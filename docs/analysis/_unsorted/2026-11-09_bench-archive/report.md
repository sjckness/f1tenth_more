---
id: 2026-11-09_bench-archive
title: "Bench avvio missione planner e piano pulizia archivio"
area: _unsorted
date: 2026-11-09
tags: [bench, planner-start, archive-cleanup, llama-server, logger]
modules: {llm: "0.0.1", f1tenth_logger: "0.0.1", f1tenth_ctl: "0.0.1"}
branch: "isaac-mpc-tuning"
commit: "be9fa2506540"
status: open
hypothesis: "n/d (il piano non lo dice; i JSON confrontano flusso production oneshot/persistent prima/dopo llama-server -np 1 -c 4096)"
result: "Bench del 2026-11-09 hanno lasciato 342 voci in archivio (209 MB, 98 llm + 244 ctl); ne le soglie di latenza ne gli esiti sono commentati nel documento."
decision: "Piano di rimozione elencato, NON eseguito; per evitare l'archiviazione serve riavviare lo stack con enable_mission_logger:=false o mission_logger_runs_dir; nessuno skip per-missione."
raw_data: "git exp/2026-11-09_bench-archive:output/bench/archive_bench_runs_2026-11-09.txt; exp/2026-11-09_bench-archive:output/bench/archive_bench_runs_2026-11-09_full.txt. Esterni: /scratch/fabiocar/f1tenth_container/archive/complete/ (342 voci), .../ctl/missions/, output/bench/*.json, archive_bench_runs_2026-11-09_full.txt"
migrated_from: "isaac-mpc-tuning:output/bench/"
migrated_on: 2026-11-18
---
# Bench avvio missione planner e piano pulizia archivio

> Report scritto durante la migrazione (2026-11-18): questo esperimento non aveva un report proprio. Il contenuto viene solo dai file elencati sotto; "n/d" dove i file non lo dicono.

## Setup

Dati originali: `isaac-mpc-tuning:output/bench/`.

## Metodo

n/d (il piano non lo dice; i JSON confrontano flusso production oneshot/persistent prima/dopo llama-server -np 1 -c 4096)

## Risultati

Bench del 2026-11-09 hanno lasciato 342 voci in archivio (209 MB, 98 llm + 244 ctl); ne le soglie di latenza ne gli esiti sono commentati nel documento.

- Il mission logger registra ogni missione che raggiunge LOADED e non ha skip per-missione
- Bench 2026-11-09: 248 run registrati, 342 voci d'archivio sul container Orin (209 MB)

## Conclusioni

Piano di rimozione elencato, NON eseguito; per evitare l'archiviazione serve riavviare lo stack con enable_mission_logger:=false o mission_logger_runs_dir; nessuno skip per-missione.

## Prossimi passi

- Eseguire (o scartare) la pulizia dopo revisione
- Documentare i risultati dei bench prod_*/cross_after_*

## File migrati

- [archive_cleanup_plan.md](archive_cleanup_plan.md) (da `output/bench/archive_cleanup_plan.md`)
- [data/cross_after_oneshot.json](data/cross_after_oneshot.json) (da `output/bench/cross_after_oneshot.json`)
- [data/cross_after_persistent.json](data/cross_after_persistent.json) (da `output/bench/cross_after_persistent.json`)
- [data/prod_A.json](data/prod_A.json) (da `output/bench/prod_A.json`)
- [data/prod_B.json](data/prod_B.json) (da `output/bench/prod_B.json`)
- [data/prod_B_poll2.json](data/prod_B_poll2.json) (da `output/bench/prod_B_poll2.json`)
- [data/prod_C.json](data/prod_C.json) (da `output/bench/prod_C.json`)
- [data/prod_C_kill.json](data/prod_C_kill.json) (da `output/bench/prod_C_kill.json`)
- [data/prod_C_off.json](data/prod_C_off.json) (da `output/bench/prod_C_off.json`)
- [data/prod_after.json](data/prod_after.json) (da `output/bench/prod_after.json`)
- [data/prod_after2.json](data/prod_after2.json) (da `output/bench/prod_after2.json`)
- [data/prod_after_spaced_0_vai_dall.json](data/prod_after_spaced_0_vai_dall.json) (da `output/bench/prod_after_spaced_0_vai_dall.json`)
- [data/prod_after_spaced_0_vai_drit.json](data/prod_after_spaced_0_vai_drit.json) (da `output/bench/prod_after_spaced_0_vai_drit.json`)
- [data/prod_after_spaced_1_vai_dall.json](data/prod_after_spaced_1_vai_dall.json) (da `output/bench/prod_after_spaced_1_vai_dall.json`)
- [data/prod_after_spaced_1_vai_drit.json](data/prod_after_spaced_1_vai_drit.json) (da `output/bench/prod_after_spaced_1_vai_drit.json`)
- [data/prod_base2.json](data/prod_base2.json) (da `output/bench/prod_base2.json`)
- [data/prod_before.json](data/prod_before.json) (da `output/bench/prod_before.json`)
- [data/prod_smoke.json](data/prod_smoke.json) (da `output/bench/prod_smoke.json`)
- [data/smoke.json](data/smoke.json) (da `output/bench/smoke.json`)
- [data/suite_head_oneshot.json](data/suite_head_oneshot.json) (da `output/bench/suite_head_oneshot.json`)
- [data/suite_head_persistent.json](data/suite_head_persistent.json) (da `output/bench/suite_head_persistent.json`)
- [data/suite_image_oneshot.json](data/suite_image_oneshot.json) (da `output/bench/suite_image_oneshot.json`)
