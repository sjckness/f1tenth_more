---
name: version-lookup
description: Usala quando serve sapere cosa contiene una versione di un modulo (CHANGELOG, analisi che la citano, parametri cambiati), anche quella che gira sull'Orin; sempre prima di indagare un bug su una versione.
---
# version-lookup

Wrapper di `tools/f1t whatis` (tools/core/analysis/whatis.py, solo stdlib, pochi secondi).

## Passi
1. `tools/f1t whatis <package> [versione]` (senza versione: quella di package.xml).
2. Versione dell'auto: `tools/f1t whatis <package> --deployed`.
3. Diff completo dei parametri: il comando `git diff <pkg>/vA <pkg>/vB -- ...` stampato in fondo.

## Successo
- Tre sezioni: CHANGELOG (versione + 2 precedenti), Analisi (`*` = citano quella versione,
  con risultato e decisione), Parametri (yaml cambiati tra i tag).

## Se fallisce
- `modulo sconosciuto`: nome del package (`<name>` in package.xml), non della cartella.
- `tag ... assente`: versione mai rilasciata con `f1t release`; usa il CHANGELOG e git log.
- `--deployed` FAIL: Orin non raggiungibile; usa `tools/f1t orin status` quando torna.
