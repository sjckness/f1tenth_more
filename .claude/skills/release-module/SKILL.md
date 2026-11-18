---
name: release-module
description: Usala per rilasciare una versione di un modulo ROS (bump SemVer in package.xml, voce CHANGELOG da bozza git log, commit e tag <pkg>/vX.Y.Z).
---
# release-module

Wrapper di `tools/f1t release` (tools/core/release/release.sh). Niente push.

## Passi
1. Scegli il bump: MAJOR rompe topic/parametri/interfacce, MINOR aggiunge, PATCH corregge.
2. `tools/f1t release <package> <patch|minor|major|X.Y.Z> --dry-run`: mostra la bozza della voce.
3. Riscrivi la bozza in frasi con l'utente (o lascia che la modifichi in $EDITOR), poi
   `tools/f1t release <package> <bump>` da terminale (chiede conferma), o `--yes` se l'utente ha
   gia' approvato il testo.

## Successo
- Commit `release: <pkg> vX.Y.Z` con solo package.xml, CHANGELOG.md (e setup.py), tag annotato
  `<pkg>/vX.Y.Z`; `tools/f1t whatis <pkg>` mostra la nuova voce.

## Se fallisce
- `modifiche in stage`: committa o togli dallo stage il resto prima.
- `tag esiste` / `non e' maggiore`: scegli un'altra versione.
