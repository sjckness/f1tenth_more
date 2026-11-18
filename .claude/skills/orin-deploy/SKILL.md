---
name: orin-deploy
description: Usala per mettere sull'Orin un'immagine gia' costruita e testata (pin, restart, health check, rollback automatico) o per tornare alla precedente.
---
# orin-deploy

Wrapper di `tools/f1t deploy` (tools/core/deploy/deploy.sh). Azione sull'auto: chiedi conferma all'utente.

## Passi
1. `tools/f1t orin status` per vedere cosa gira e se lo stack e' attivo.
2. `tools/f1t diff-params --to <commit>`: mostra all'utente i parametri che cambiano.
3. A secco: `tools/f1t deploy --dry-run <immagine|commit>` (tutti i cancelli, niente cambiato).
4. Deploy: `tools/f1t deploy <immagine|commit>`; prima di un deploy importante `--after-hil`
   (serve un `tools/f1t hil run smoke -disp <host> --image <immagine>` OK).
5. Tornare indietro: `tools/f1t deploy --rollback`.

## Successo
- Riepilogo senza FAIL, `health check | sano in N s`, `deployed.json | <percorso>`.

## Se fallisce
- Cancello FAIL (tree sporco, immagine non testata, stack attivo, modalita' native): l'Orin non e'
  stato toccato; riferisci la riga all'utente. `--force-untested` / `--force-busy` / `--switch-mode`
  solo se l'utente lo chiede.
- Health check FAIL: il rollback e' automatico; riporta le righe `health check` e `rollback`.
- deployed.json in WARN: /etc/f1tenth manca (tools/SUDO_TODO.md), il deploy e' comunque fatto.
