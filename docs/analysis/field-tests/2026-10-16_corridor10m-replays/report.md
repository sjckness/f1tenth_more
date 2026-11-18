---
id: 2026-10-16_corridor10m-replays
title: "Replay offline di corridor10m: contratto WALL/CLEAR e tracker muri"
area: field-tests
date: 2026-10-16
tags: [replay, corridor10m, wall-tracker, stereo-columns, virtual-wall, front-contract]
modules: {f1tenth_perception: "0.0.1"}
branch: "orin/orin-wip-2026-10-22"
commit: "02cd0f2084da"
status: open
hypothesis: "Rigiocare il codice corrente su corridor10m_20261015T171753 (varianti baseline/after/d/e) e confrontare stato WALL/CLEAR e tracce col registrato."
result: "Stato contratto identico su 136/206 cicli (66.0 %) in baseline, 127/206 (61.7 %) in after; wall->clear 67-76 cicli, quasi tutti spiegati dal cap max_published=4. Nessun episodio fantasma (0)."
decision: "n/d (i documenti sono solo output di replay e summary, senza sezione di decisione)"
raw_data: "git exp/2026-10-16_corridor10m-replays:output/replay_after/corridor10m_20261015T171753.info.txt; exp/2026-10-16_corridor10m-replays:output/replay_after2/corridor10m_20261015T171753.camera.mp4; exp/2026-10-16_corridor10m-replays:output/replay_after2/corridor10m_20261015T171753.info.txt; exp/2026-10-16_corridor10m-replays:output/replay_after2/corridor10m_20261015T171753.topdown.mp4; exp/2026-10-16_corridor10m-replays:output/replay_baseline/corridor10m_20261015T171753.info.txt; exp/2026-10-16_corridor10m-replays:output/replay_baseline2/corridor10m_20261015T171753.info.txt; +7 gruppi (vedi _inventory.md). Esterni: corridor10m_20261015T171753 (~/bags/yolo_gpu/corridor10m_20261015T171753)"
migrated_from: "orin/orin-wip-2026-10-22:output/replay_after/, output/replay_after2/, output/replay_baseline/, output/replay_baseline2/, output/replay_d_after/, output/replay_d_before/, output/replay_e/"
migrated_on: 2026-11-18
---
# Replay offline di corridor10m: contratto WALL/CLEAR e tracker muri

> Report scritto durante la migrazione (2026-11-18): questo esperimento non aveva un report proprio. Il contenuto viene solo dai file elencati sotto; "n/d" dove i file non lo dicono.

## Setup

Dati originali: `orin/orin-wip-2026-10-22:output/replay_after/, output/replay_after2/, output/replay_baseline/, output/replay_baseline2/, output/replay_d_after/, output/replay_d_before/, output/replay_e/`.

## Metodo

Rigiocare il codice corrente su corridor10m_20261015T171753 (varianti baseline/after/d/e) e confrontare stato WALL/CLEAR e tracce col registrato.

## Risultati

Stato contratto identico su 136/206 cicli (66.0 %) in baseline, 127/206 (61.7 %) in after; wall->clear 67-76 cicli, quasi tutti spiegati dal cap max_published=4. Nessun episodio fantasma (0).

- Replay vs registrato: contratto identico su 136/206 cicli (66.0 %); i wall->clear dipendono dal cap a 4 tracce piu' storico mancante.
- Con CLEAR la distanza vera in striscia non scende sotto 14.638 m; 0 episodi fantasma.
- Variante replay_e: 21 tracce, 5.95 s in WALL contro 35.91 s registrati.

## Conclusioni

n/d (i documenti sono solo output di replay e summary, senza sezione di decisione)

## Prossimi passi

- Perche' il replay vede meno WALL del registrato (5.95 s vs 35.91 s nella variante e).

## File migrati

- [replay_after/corridor10m_20261015T171753.replay.md](replay_after/corridor10m_20261015T171753.replay.md) (da `output/replay_after/corridor10m_20261015T171753.replay.md`)
- [data/replay_after/corridor10m_20261015T171753.summary.json](data/replay_after/corridor10m_20261015T171753.summary.json) (da `output/replay_after/corridor10m_20261015T171753.summary.json`)
- [replay_after/corridor10m_20261015T171753.summary.md](replay_after/corridor10m_20261015T171753.summary.md) (da `output/replay_after/corridor10m_20261015T171753.summary.md`)
- [replay_after2/corridor10m_20261015T171753.replay.md](replay_after2/corridor10m_20261015T171753.replay.md) (da `output/replay_after2/corridor10m_20261015T171753.replay.md`)
- [data/replay_after2/corridor10m_20261015T171753.summary.json](data/replay_after2/corridor10m_20261015T171753.summary.json) (da `output/replay_after2/corridor10m_20261015T171753.summary.json`)
- [replay_after2/corridor10m_20261015T171753.summary.md](replay_after2/corridor10m_20261015T171753.summary.md) (da `output/replay_after2/corridor10m_20261015T171753.summary.md`)
- [replay_baseline/corridor10m_20261015T171753.replay.md](replay_baseline/corridor10m_20261015T171753.replay.md) (da `output/replay_baseline/corridor10m_20261015T171753.replay.md`)
- [data/replay_baseline/corridor10m_20261015T171753.summary.json](data/replay_baseline/corridor10m_20261015T171753.summary.json) (da `output/replay_baseline/corridor10m_20261015T171753.summary.json`)
- [replay_baseline/corridor10m_20261015T171753.summary.md](replay_baseline/corridor10m_20261015T171753.summary.md) (da `output/replay_baseline/corridor10m_20261015T171753.summary.md`)
- [replay_baseline2/corridor10m_20261015T171753.replay.md](replay_baseline2/corridor10m_20261015T171753.replay.md) (da `output/replay_baseline2/corridor10m_20261015T171753.replay.md`)
- [data/replay_baseline2/corridor10m_20261015T171753.summary.json](data/replay_baseline2/corridor10m_20261015T171753.summary.json) (da `output/replay_baseline2/corridor10m_20261015T171753.summary.json`)
- [replay_baseline2/corridor10m_20261015T171753.summary.md](replay_baseline2/corridor10m_20261015T171753.summary.md) (da `output/replay_baseline2/corridor10m_20261015T171753.summary.md`)
- [replay_d_after/corridor10m_20261015T171753.replay.md](replay_d_after/corridor10m_20261015T171753.replay.md) (da `output/replay_d_after/corridor10m_20261015T171753.replay.md`)
- [data/replay_d_after/corridor10m_20261015T171753.summary.json](data/replay_d_after/corridor10m_20261015T171753.summary.json) (da `output/replay_d_after/corridor10m_20261015T171753.summary.json`)
- [replay_d_after/corridor10m_20261015T171753.summary.md](replay_d_after/corridor10m_20261015T171753.summary.md) (da `output/replay_d_after/corridor10m_20261015T171753.summary.md`)
- [replay_d_before/corridor10m_20261015T171753.replay.md](replay_d_before/corridor10m_20261015T171753.replay.md) (da `output/replay_d_before/corridor10m_20261015T171753.replay.md`)
- [data/replay_d_before/corridor10m_20261015T171753.summary.json](data/replay_d_before/corridor10m_20261015T171753.summary.json) (da `output/replay_d_before/corridor10m_20261015T171753.summary.json`)
- [replay_d_before/corridor10m_20261015T171753.summary.md](replay_d_before/corridor10m_20261015T171753.summary.md) (da `output/replay_d_before/corridor10m_20261015T171753.summary.md`)
- [replay_e/corridor10m_20261015T171753.replay.md](replay_e/corridor10m_20261015T171753.replay.md) (da `output/replay_e/corridor10m_20261015T171753.replay.md`)
- [data/replay_e/corridor10m_20261015T171753.summary.json](data/replay_e/corridor10m_20261015T171753.summary.json) (da `output/replay_e/corridor10m_20261015T171753.summary.json`)
- [replay_e/corridor10m_20261015T171753.summary.md](replay_e/corridor10m_20261015T171753.summary.md) (da `output/replay_e/corridor10m_20261015T171753.summary.md`)
