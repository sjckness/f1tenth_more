---
name: hil-up
description: Usala per avviare un run hardware-in-the-loop (Isaac Sim su linus o work, stack nell'immagine dell'Orin) con precheck e attese attive.
---
# hil-up

Wrapper di `tools/f1t hil precheck` e `tools/f1t hil run` (tools/core/hil/). L'utente lancia l'HIL:
avvialo solo se lo chiede.

## Passi
1. `tools/f1t hil precheck -disp <linus|work> [--scenario NOME]`: deve finire con `HIL PRECHECK: OK`.
2. `tools/f1t hil run <scenario> -disp <linus|work>` (scenari in tools/core/hil/scenarios/:
   `smoke` = regressione prima di un deploy, chiude da solo; `office-corridor` resta su).
   Candidata al deploy: `--image f1tenth:jazzy-orin-<sha7>[-norust]`.
3. Per chiudere: skill hil-down.

## Successo
- Riepilogo senza FAIL; `logs/hil/<data>_<disp>_<scenario>/result.json` con `"status": "OK"`.

## Se fallisce
- Precheck FAIL: elenca le righe all'utente (rete 10.42.60.x, VRAM: freeparty-llm, stack dell'auto
  attivo, versioni diverse da main -> `--allow-mismatch` solo se l'utente lo vuole).
- Un passo del run FAIL: run.sh ha gia' chiuso tutto; log del passo nella cartella del run
  (`sim.log`, `stack.log`). Non rilanciare in loop: riporta il passo e il log.
