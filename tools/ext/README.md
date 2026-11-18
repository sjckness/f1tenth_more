# tools/ext: estensioni di branch

Una cartella per estensione: `tools/ext/<nome>/` con un `README.md` di 3 righe (a cosa serve, branch,
quando cancellarlo) e script `*.sh` / `*.py` eseguibili, lanciati con `f1t ext <nome> <comando>`.
Usano `tools/core/lib/*` (source "$F1T_CORE/lib/log.sh"), mai una copia. Uno script utile a tutti si
promuove in `tools/core/` con una PR su main; il resto resta nel tag `exp/<id>` e si cancella.
