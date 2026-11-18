---
id: 2026-09-18_llm-dry-campaign
title: "Campagna dry-run del planner LLM (50 prompt)"
area: llm-pipeline
date: 2026-09-18
tags: [llm-planner, dry-run, prompt-campaign, qwen25-3b, translator]
modules: {llm: "0.0.1"}
branch: "jazzy"
commit: "1d008532f15f"
status: done
hypothesis: "Misurare esito prompt -> LLM -> translator in --dry-run su 50 prompt (nessun comando alla macchina), con modello qwen25_3b_instruct."
result: "Run 20260918_160413 su 50 prompt: 17 ok, 24 refused, 5 translator_error, 4 llm_error; goal_door_opening 10/10 refused; run piccoli di prova su timeout e URL irraggiungibile."
decision: "n/d"
raw_data: "git exp/2026-09-18_llm-dry-campaign:test_campaign/runs/20260918_160025/; exp/2026-09-18_llm-dry-campaign:test_campaign/runs/20260918_160225/; exp/2026-09-18_llm-dry-campaign:test_campaign/runs/20260918_160231/; exp/2026-09-18_llm-dry-campaign:test_campaign/runs/20260918_160413/. Esterni: test_campaign/runs/20260918_160025, 160225, 160231, 160413 (results.jsonl, summary.json su git ref jazzy); nessun report.md, dati solo da summary.json/run_info.json"
migrated_from: "jazzy:test_campaign/"
migrated_on: 2026-11-18
---
# Campagna dry-run del planner LLM (50 prompt)

> Report scritto durante la migrazione (2026-11-18): questo esperimento non aveva un report proprio. Il contenuto viene solo dai file elencati sotto; "n/d" dove i file non lo dicono.

## Setup

Dati originali: `jazzy:test_campaign/`.

## Metodo

Misurare esito prompt -> LLM -> translator in --dry-run su 50 prompt (nessun comando alla macchina), con modello qwen25_3b_instruct.

## Risultati

Run 20260918_160413 su 50 prompt: 17 ok, 24 refused, 5 translator_error, 4 llm_error; goal_door_opening 10/10 refused; run piccoli di prova su timeout e URL irraggiungibile.

- Planner LLM in dry-run: 17/50 prompt ok (34%), 24/50 rifiutati, 9/50 errori
- Categoria goal_door_opening: 0/10 ok, tutti refused

## Conclusioni

n/d

## Prossimi passi

- Cause dei 14 refused 'ambiguous_after_rejection' e dei 5 'retries_exhausted' non analizzate nei documenti

## File migrati

- [data/prompts.json](data/prompts.json) (da `test_campaign/prompts.json`)
- [run_dry_campaign.py](run_dry_campaign.py) (da `test_campaign/run_dry_campaign.py`)
- [data/runs/20260918_160025/run_info.json](data/runs/20260918_160025/run_info.json) (da `test_campaign/runs/20260918_160025/run_info.json`)
- [data/runs/20260918_160025/summary.json](data/runs/20260918_160025/summary.json) (da `test_campaign/runs/20260918_160025/summary.json`)
- [data/runs/20260918_160225/run_info.json](data/runs/20260918_160225/run_info.json) (da `test_campaign/runs/20260918_160225/run_info.json`)
- [data/runs/20260918_160225/summary.json](data/runs/20260918_160225/summary.json) (da `test_campaign/runs/20260918_160225/summary.json`)
- [data/runs/20260918_160231/run_info.json](data/runs/20260918_160231/run_info.json) (da `test_campaign/runs/20260918_160231/run_info.json`)
- [data/runs/20260918_160231/summary.json](data/runs/20260918_160231/summary.json) (da `test_campaign/runs/20260918_160231/summary.json`)
- [data/runs/20260918_160413/run_info.json](data/runs/20260918_160413/run_info.json) (da `test_campaign/runs/20260918_160413/run_info.json`)
- [data/runs/20260918_160413/summary.json](data/runs/20260918_160413/summary.json) (da `test_campaign/runs/20260918_160413/summary.json`)
