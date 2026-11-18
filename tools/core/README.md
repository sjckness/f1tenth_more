# tools/core: il tooling `f1t`

Un solo comando, `tools/f1t <gruppo> <comando>` (`f1t help` elenca tutto, ogni comando accetta `-h`).
Principio: **si modifica solo su linus; l'Orin riceve immagini costruite da un commit e non si tocca a
mano.** Le skill di Claude Code in `.claude/skills/` chiamano questi comandi e basta.

| comando | cosa fa | skill |
|---|---|---|
| `f1t build [--dry-run] [COMMIT]` | immagine dell'Orin da un commit: bundle git -> build sull'Orin in un worktree temporaneo -> pytest nell'immagine -> label `f1tenth.tested=true` | container-build |
| `f1t deploy [--dry-run] [--after-hil] [IMG\|COMMIT]`, `--rollback` | pin + restart + health check, rollback automatico, `deployed.json` | orin-deploy |
| `f1t orin status` | cosa gira, versioni vs main, drift | orin-status |
| `f1t diff-params` | yaml cambiati tra il deployato e un commit | orin-deploy |
| `f1t hil precheck\|run\|down -disp <linus\|work>` | HIL Isaac Sim + Orin | hil-up, hil-down |
| `f1t whatis <pkg> [ver\|--deployed]` | CHANGELOG, analisi, parametri di una versione | version-lookup |
| `f1t exp new <area> <slug>`, `f1t exp index` | nuova analisi con le versioni dell'Orin; indici | new-experiment |
| `f1t release <pkg> [patch\|minor\|major]` | bump SemVer, voce CHANGELOG, commit, tag | release-module |
| `f1t doctor`, `f1t core sync`, `f1t setup`, `f1t selftest` | manutenzione del tooling | - |

## Struttura

```
tools/f1t                stub immutabile -> tools/core/f1t.sh
tools/core/              IDENTICO su tutti i branch, si modifica SOLO su main
  VERSION CHANGELOG.md   SemVer del tooling (MAJOR se cambia un comando o un argomento)
  config.env             unica configurazione (IP, utenti, percorsi, soglie); per macchina: ~/.config/f1t/config.env
  hosts/<nome>.env       host della sim HIL (-disp)          schema/*.keys  chiavi obbligatorie
  lib/                   log.sh (log_*, check, summary), config.sh, checks.sh (sonde), remote.sh (ssh)
  <gruppo>/<cmd>.sh|py   un comando; "# desc:" nelle prime righe lo rende visibile, "# alias:" aggiunge nomi
  <gruppo>/remote/       script che girano SULL'ORIN (ssh bash -s < script), mai here-doc dentro ssh
  hil/scenarios/*.env    scenari HIL (_template.env per crearne uno)
  tests/                 f1t selftest (bash puro, repo finto, ssh finto: < 30 s)
tools/ext/<nome>/        estensioni di un branch: f1t ext <nome> <cmd>; usano tools/core/lib, mai copiarla
```

Ogni script: `set -euo pipefail`, source di `lib/log.sh`, stampa la configurazione effettiva prima di
agire, un `check` per ogni controllo (valore stampato), `summary` finale (exit != 0 se c'e' un FAIL).
Log senza colori in `logs/tools/<script>_<data>.log` (fuori da git).

## Rete di sicurezza

1. `f1t selftest`: un test per comando (help, argomenti sbagliati, dry-run, Orin irraggiungibile = FAIL pulito).
2. CI (`.github/workflows/`): `tools-core.yml` (shellcheck + selftest + bump VERSION) su ogni modifica a
   tools/core; `ros-packages.yml` (colcon build + test dei package cambiati e dipendenti) su ogni PR
   verso main. Il blocco del merge richiede la branch protection (tools/SUDO_TODO.md).
3. Versione: modifica a tools/core = bump di `VERSION` + voce in `CHANGELOG.md` (l'hook lo impone su main).
4. Config validate: ogni script controlla le chiavi di `schema/*.keys` e dice quale manca e dove.
5. Deploy a gradini: solo immagini `f1tenth.tested=true`; `--after-hil` vuole uno smoke HIL OK su
   quell'immagine; rollback automatico se l'health check fallisce entro `F1T_HEALTH_TIMEOUT_S`.
6. Scenario `smoke`: avvio, topic vivi, 10 s dritto, poi chiude.
7. `f1t diff-params` prima del deploy (il deploy lo stampa da solo).

Branch diversi da main: `f1t doctor` dice se tools/core e' indietro, `f1t core sync` lo riallinea con un
commit "chore: sync tools/core vX.Y.Z". L'hook (`f1t setup`) blocca i commit che toccano tools/core
fuori da main (`F1T_CORE_EDIT=1` solo per fix urgenti da riportare su main). Quando un branch di
analisi si chiude, uno script utile di `tools/ext/<nome>` si promuove in core con una PR su main.
