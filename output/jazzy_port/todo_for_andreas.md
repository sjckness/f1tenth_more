# TODO for Andreas: jazzy-campaign port

## 1. Push the car's campaign-final from the Orin (Phase 0: commits missing on origin)

The 2026-10-14 and 2026-10-15 bags were recorded from `736d46fe14` and `e3793e47f595c501e0c9796e93791faf0cc9edef` (dirty tree). Neither is on GitHub. Meanwhile origin/campaign-final gained 3 linus commits the Orin does not have (`cdfd736`, `a8a61db`, `7b17e2f`: test-campaign data and docs only). So the branches have **diverged**: a plain push from the Orin will be rejected, and campaign-final must never be force-pushed.

On the Orin (user `fabiocar`, repo `~/dev_ws/f1tenth_more`, per the bag manifests):
```bash
cd ~/dev_ws/f1tenth_more
git status                                    # the bags say the tree was dirty: decide what to commit
git log --oneline origin/campaign-final..campaign-final     # the Orin-only commits (736d46f .. e3793e4 ..)
git submodule foreach --recursive 'git log --oneline -1 --decorate; git status --short | head -5'
# keep the dirty work: commit it (or put it on its own branch) BEFORE anything else
git add -A && git commit -m "campaign-final: Orin working tree of 2026-10-15"     # if it should be kept
git fetch origin
git merge origin/campaign-final               # brings cdfd736/a8a61db/7b17e2f (docs + campaign data), no conflicts expected
git push origin campaign-final                # fast-forward, no force
```
If a submodule shows local commits (unlikely: the pins on origin match the forks), push them to the fork first, e.g. `git -C src/f1tenth_hardware/vesc push origin HEAD:<branch>`, then commit the new pointer in the superproject.

Then on linus, the port must be redone on top of the new tip: `jazzy-campaign` is a re-application of the migration, so rebasing it onto the new campaign-final (or re-running the Phase 2 steps) is the next step.

## 2. Stale lock in the main checkout
`~/dev_ws/f1tenth_more/.git/modules/ackermann_mux/index.lock` (0 bytes, 2026-10-07) blocks git operations inside that submodule. Remove it if no git process is running: `rm ~/dev_ws/f1tenth_more/.git/modules/ackermann_mux/index.lock`.

## 3. apt packages for linus (needs sudo; user-space stand-ins are in ~/dev_ws/jc_deps_ws and ~/dev_ws/jc_venv)
```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-py-trees-ros ros-jazzy-py-trees-ros-interfaces \
  ros-jazzy-topic-tools ros-jazzy-rosbridge-server python3-ruamel.yaml \
  ros-jazzy-serial-driver ros-jazzy-io-context ros-jazzy-asio-cmake-module ros-jazzy-udp-msgs
# hardware-only, needed on the car/Thor rather than linus: ros-jazzy-urg-node ros-jazzy-v4l2-camera ros-jazzy-zed-description
# pip (PEP 668: venv, or PIP_BREAK_SYSTEM_PACKAGES=1): osqp==1.1.3 pyarrow==25.0.1 ultralytics==8.4.90 (numpy<2)
```
After that, `rm -rf ~/dev_ws/jc_deps_ws` and rebuild jazzy-campaign without the overlay.
