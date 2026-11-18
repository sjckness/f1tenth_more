---
name: hil-down
description: Usala per chiudere un run HIL (stack sull'Orin, sim, bridge) e verificare che non resti nessun processo o container.
---
# hil-down

Wrapper di `tools/f1t hil down` (tools/core/hil/down.sh).

## Passi
1. `tools/f1t hil down -disp <linus|work>` (stesso -disp del run; legge logs/hil/current.env).

## Successo
- Exit 0; righe OK per `container HIL rimosso`, `processo della sim fermo`,
  `container della sim fermo`, `host della sim pulito`, `Orin pulito`.
- `logs/hil/current.env` rimosso, `stack.log` e `down.log` nella cartella del run.

## Se fallisce
- Exit 1: il riepilogo elenca cosa resta (container, processi, porta DS). Rilancia una volta;
  se resta, riferisci all'utente. Mai `pkill -f` nella stessa shell del pattern (CLAUDE.md).
- Orin non raggiungibile: lo stack HIL resta su; ripeti quando l'Orin risponde.
