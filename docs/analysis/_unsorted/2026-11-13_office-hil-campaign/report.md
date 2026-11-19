---
id: 2026-11-13_office-hil-campaign
title: "Campagna office HIL: 3 comandi x 3 ripetizioni con l'Orin nel giro (solo runbook)"
area: _unsorted
date: 2026-11-13
tags: [isaac, hil, orin, campaign, llm, runbook, no-results]
modules: {llm: "0.1.0", f1tenth_logger: "0.1.0", mpc_controller: "0.1.0", f1tenth_behavior: "0.1.0"}
branch: "planner-modes-hil"
commit: "47ac7b8b"
status: open
hypothesis: "Portare 3 comandi in linguaggio naturale per 9 run (3 ripetizioni) attraverso la pipeline completa LLM -> missione -> behaviour tree -> mpc_corr -> /drive -> Isaac, in HIL, con lo stack invariato nel container sull'Orin"
result: "n/d: la campagna non ha prodotto un report. Di eseguito restano 15 log di Isaac della notte (circa 50 kB l'uno, scripts/sim/win/night/{1..21}_campaign_m[abd].log), salvati nel tar della work. Nessun esito per run, nessuna tabella, nessun verdetto. Il protocollo e le tre missioni sono invece scritti per intero."
decision: "n/d"
raw_data: "scripts/sim/win/night/ (15 log Isaac) in ~/f1tenth_salvataggi/work-20261119.tar.gz sulla work. Gli script della campagna (office_hil_campaign/, hil_campaign.sh, hil_night.sh, hil_follow.bat) e il logger office_hil_campaign_logger.yaml sono nel tag archive/work-planner-modes-hil-20261119, non su main."
migrated_from: "planner-modes-hil:office_hil_campaign/README.md + RUNBOOK.md"
migrated_on: 2026-11-19
---
# Campagna office HIL: perche' sta in `_unsorted`

Questo `report.md` e' stato scritto in migrazione (2026-11-19, recupero dalla macchina **work**).

I due documenti recuperati — [campaign_README.md](campaign_README.md) e [RUNBOOK.md](RUNBOOK.md) —
descrivono **come** eseguire la campagna: le tre missioni, l'ordine dei run, i tre terminali, il
perche' di tre sessioni Isaac separate. Sono un runbook, non un'analisi: non contengono nessun
risultato misurato.

Dei 9 run previsti non esiste alcun esito scritto. L'unica traccia di esecuzione sono 15 log di
Isaac della notte in `scripts/sim/win/night/`, uno per run tentato, numerati da 1 a 21 con buchi
(`1,2,4,6,7,9,10,12,14,15,17,18,19` per `campaign_mb`/`ma`, `20,21` per `campaign_md`): log del
simulatore, non del risultato della missione. Il lato ROS di quei run non e' stato trovato sulla
work.

Resta in `_unsorted` per questo: appena si trovano i risultati (sull'Orin, in
`~/dev_ws/f1tenth_hil/output/`, oppure rieseguendo la campagna) diventa un esperimento di
`field-tests` o di `simulation`; finche' non ci sono, non c'e' niente da concludere.

Le tre missioni sono le stesse la cui geometria era stata verificata prima di eseguirle:
[llm-pipeline/2026-11-13_office-missions](../../llm-pipeline/2026-11-13_office-missions/report.md).
