---
name: orin-status
description: Usala per sapere cosa gira sull'Orin (immagine, sha, versioni moduli, uptime, distanza da main) e se qualcuno ha modificato file a mano (drift).
---
# orin-status

Wrapper di `tools/f1t orin status` (tools/core/orin/status.sh). Solo letture.

## Passi
1. `tools/f1t orin status` (aggiungi `--fetch` se il commit dell'immagine non e' su linus).
2. Per una versione precisa di un modulo: skill version-lookup con `--deployed`.

## Successo
- Exit 0; il riepilogo dice modalita', container healthy, immagine e pin, revisione, moduli
  rispetto a origin/main, drift (file cambiati nel container, repo dell'Orin sporchi, override).

## Come leggerlo
- `testata (label)` WARN: immagine non costruita da `f1t build`.
- `moduli vs origin/main` WARN: l'auto non gira main; elenca le differenze.
- Drift WARN: qualcuno ha modificato file sull'Orin. Non correggerlo li': riportalo all'utente; la
  correzione va su linus -> container-build -> orin-deploy.
- Override a caldo: `~/.config/f1tenth_car/stack_args` (unico posto ammesso per parametri al volo).

## Se fallisce
- Exit 1 con `ssh ... FAIL`: Orin spento o fuori rete (Tailscale). Gli altri check sono SKIP.
