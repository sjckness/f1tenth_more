# Decisions: jazzy-campaign port and Isaac sim mode

Each entry: decision, alternatives, why. "PROPOSAL" entries are logic changes that were NOT made in code.

| # | Decision | Alternatives | Why |
|---|---|---|---|
| D1 | All port work in a **separate git worktree** `~/dev_ws/f1tenth_more_jc` (same repository, branch `jazzy-campaign`); the main checkout is left exactly as found | checkout the new branch in `~/dev_ws/f1tenth_more` | the main checkout has uncommitted submodule moves (zed wrapper and interfaces dirty, ackermann_mux on b3c0b08) and untracked files; building `jazzy-campaign` there needs `git submodule update`, which would overwrite them. Branches created in the worktree are in the same repo, so `git branch` in the main checkout shows them |
| D2 | Source of truth = **origin/campaign-final 7b17e2f** (newest available) | wait for the Orin | `e3793e47` (2026-10-15 bags) and `736d46fe14` (2026-10-14 bags) were never pushed; GitHub refuses them by SHA. The port is therefore based on an OLDER state than the car runs: see `phase0.md` for what is missing |
| D3 | `jazzy-campaign` upstream unset after creation | keep tracking origin/campaign-final | `git worktree add -b` set the upstream to origin/campaign-final; with `push.default=upstream` a stray push would have targeted campaign-final |
