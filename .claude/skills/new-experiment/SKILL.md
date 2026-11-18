---
name: new-experiment
description: Usala per aprire una nuova analisi o prova in docs/analysis/ con le versioni dei moduli che girano sull'Orin, branch e commit gia' compilati.
---
# new-experiment

Wrapper di `tools/f1t exp new` (tools/core/analysis/new.sh). Non committa.

## Passi
1. `tools/f1t exp new <area> <slug> --title "Titolo" [--modules pkg1,pkg2] [--hil-run last]`
   (aree: le cartelle di docs/analysis; slug in minuscolo-con-trattini).
2. Compila hypothesis/result/decision/raw_data e il corpo del report.
3. `tools/f1t exp index --check`, commit docs-only su main, tag `exp/<id>` sul commit testato.

## Successo
- Stampa il percorso del report; `modules:` letto dall'Orin (riga "versioni dei moduli lette
  dall'Orin"), `--hil-run` copia result.json/checks.json in data/.

## Se fallisce
- Orin non raggiungibile: versioni prese dal checkout, scritto nel Setup; dillo all'utente.
- `esiste gia'`: cambia slug o data. `area sconosciuta`: l'errore elenca le aree.
