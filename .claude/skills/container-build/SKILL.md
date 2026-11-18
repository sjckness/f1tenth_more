---
name: container-build
description: Usala per costruire l'immagine Docker dell'Orin da un commit di linus (build remota sull'Orin, test e label tested=true) prima di un deploy.
---
# container-build

Wrapper di `tools/f1t build` (tools/core/build/build.sh). Nessuna logica qui.

## Passi
1. Il working tree deve essere committato (l'immagine contiene solo il commit).
2. Prova a secco: `tools/f1t build --dry-run [COMMIT]` (solo letture: controlli locali e sull'Orin).
3. Build: `tools/f1t build [COMMIT]`; se uno stack gira sull'Orin la build rifiuta: chiedi
   all'utente prima di aggiungere `--stop-stack` (ferma il container dell'auto).
4. Lunga (~10 min solo sorgenti, ~1 h a freddo): `--detach`, poi `tools/f1t build --follow <sha7>`.

## Successo
- Riepilogo senza FAIL e riga `immagine testata | f1tenth:jazzy-orin-<sha7>[-norust] (f1tenth.tested=true)`.
- Log della build in `logs/build/<sha7>_*.log`, record in `logs/build/<sha7>.json`.

## Se fallisce
- Leggi la riga FAIL del riepilogo: dice il cancello (tree sporco, Orin, stack attivo, disco, input).
- `fail build`: log in `logs/build/`; `fail test`: pytest nell'immagine, sistema il codice su linus,
  committa, rilancia. Mai modificare file sull'Orin. Registry: non c'e' (tools/SUDO_TODO.md).
