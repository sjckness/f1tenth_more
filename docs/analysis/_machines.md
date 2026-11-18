# Macchine: stato dei cloni di f1tenth_more

Rilievo e pulizia per macchina. Una sezione per host.

## tpad

Data: 2026-11-18. Eseguito in locale sul tpad (hostname `TPad`).

### Rilievo (prima della pulizia)

- Repo: `/home/andreas/dev_ws/f1tenth_more`, remoto `origin` = `git@github.com:sjckness/f1tenth_more.git`
- Branch corrente: `jazzy` (70dc09fa, 85 commit dietro `origin/jazzy`); `main` locale assente
- Branch locali: solo `jazzy`, 0 commit fuori da `origin/*` e dai tag
- Stash: nessuno. Worktree: solo quello principale
- Altri cloni di f1tenth_more in `~` (profondità 4): nessuno
- Processi nella cartella: VS Code, la sessione Claude Code, gitstatus (cartella in uso, non cancellata, solo riallineata)
- Dimensione `.git`: 285 MiB (pack 257.6 MiB)

| elemento | classe | cosa c'è di unico | azione |
|---|---|---|---|
| branch `jazzy` | pulito | niente | `git branch -D` |
| `M .claude/settings.json` | unico | `"enabledPlugins": {}` aggiunto | salvato nel tar, poi reset |
| `M .gitignore` | unico | riga `.sjck/` aggiunta | salvato nel tar, poi reset |
| `?? pacman/` | unico | 17 file, 132K (gioco JS con test) | salvato nel tar, rimosso dal repo |
| `.sjck/` (ignorato su jazzy) | unico | stato sjck-graph, 2.5M | salvato nel tar, lasciato su disco |
| cartella repo | in uso | aperta da VS Code e Claude Code | non cancellata |

### Azioni fatte

1. Nessun tag `archive/tpad-*`: nessun commit unico da salvare, nessun push.
2. Tar: `~/f1tenth_salvataggi/tpad-20261118.tar.gz` (sul tpad, 789 kB). Contiene anche `tpad-jazzy-worktree.diff` con il diff delle due modifiche.
3. `git fetch --all --prune` (potati `origin/llm`, `origin/scene-graph`, `origin/sick`, `origin/test-logging`), `main` creato da `origin/main` (d8093a68), `git reset --hard origin/main`.
4. `git branch -D jazzy`, `git worktree prune`. Submodule `ackermann_mux` e `vesc` riportati ai commit fissati da `main` (`git submodule update`); erano puliti e senza commit unici.
5. `git gc --prune=now`: `.git` da 291716 KiB a 288488 KiB, liberati circa 3.2 MiB.

Rimasto: `.sjck/` non tracciato nella cartella del repo (su `main` non è in `.gitignore`).
Da fare: `ackermann_mux` è un pacchetto CMake e la sua sorgente è cambiata; ricostruirlo prima di usarlo.

### Contenuto del tar

```
tpad-jazzy-worktree.diff
.claude/settings.json
.gitignore
pacman/test/t03.test.js
pacman/test/t06.test.js
pacman/test/t04.test.js
pacman/test/smoke.test.js
pacman/test/harness.js
pacman/test/t02.test.js
pacman/index.html
pacman/js/scenes.js
pacman/js/input.js
pacman/js/render.js
pacman/js/audio.js
pacman/js/cutscenes.js
pacman/js/attract.js
pacman/js/main.js
pacman/js/game.js
pacman/js/killscreen.js
pacman/js/core.js
.sjck/findings.jsonl
.sjck/out/T05.val.json
.sjck/out/T03.val.json
.sjck/out/T03.json
.sjck/out/T04.val.json
.sjck/out/T06.val.json
.sjck/out/T04.json
.sjck/out/T05.json
.sjck/out/T02.val.json
.sjck/out/T02.json
.sjck/out/REVIEW.json
.sjck/out/T01.val.json
.sjck/out/T01.json
.sjck/out/T06.json
.sjck/state.json
.sjck/watchdog_state.json
.sjck/watchdog.jsonl
.sjck/console/runs/2026-10-26T21-37-13-236Z.jsonl
.sjck/console/runs/2026-10-26T21-13-31-324Z.jsonl
.sjck/console/runs/2026-10-26T21-15-30-478Z.jsonl
.sjck/view.html
.sjck/backlog.md
.sjck/decisions/D-1793051769534-074d3c.json
.sjck/archive/pacman-baseline/index.html
.sjck/cards/T01.md
.sjck/cards/T05.md
.sjck/cards/T06.md
.sjck/cards/T04.md
.sjck/cards/T03.md
.sjck/cards/T02.md
.sjck/findings.lock
.sjck/interfaces/pacman.md
```
