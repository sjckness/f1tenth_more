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

## 4. shellcheck on linus
`sudo apt install shellcheck`. Today it runs from a pip wheel in a scratch venv (`pip install shellcheck-py`, 0.11.0); `scripts/sim/sim_isaac.sh` and `tools/e2e_test.sh` are clean with it.

## 5. Back-port the sim mode to campaign-final (D9)
The Phase 3 commit on `jazzy-campaign` (`git show --stat b82735f`) is API-neutral on Humble. Back-port it as a new branch from campaign-final once the Orin state is pushed (§1); do not push to campaign-final directly:
```bash
git switch -c campaign-final-sim origin/campaign-final
git cherry-pick b82735f        # expect conflicts only in output/jazzy_port/* (drop those) and scripts/env/jazzy.sh (not on campaign-final: drop)
colcon build && python3 -m pytest src/f1tenth_bringup/test/test_sim_mode.py src/f1tenth_behavior/test/test_preflight.py
```
`sim_isaac.sh` then needs `--ros-distro humble` on the Isaac side (isaac_ws launch.sh supports it) and `/opt/ros/humble` instead of `scripts/env/jazzy.sh` on the stack side.

## 6. Thor: next steps (two-machine sim)
1. On the Thor, check out `jazzy-campaign` and build it. Do not use `--packages-ignore`: the Thor has the ZED SDK and the VESC packages.
2. Connect linus and the Thor with the direct cable. If the Thor's link address is not 10.42.0.2, export `F1TENTH_SIM_DISCOVERY_IP=<Thor link IP>` on both machines.
3. Start the sim on linus: `scripts/sim/sim_isaac.sh isaac --net eth --scene base_room`.
4. Start the stack on the Thor: `scripts/sim/sim_isaac.sh stack --net eth`. It has 14 CPUs, so no host overlay is used and the car's pinning applies.
5. Run the e2e checks from the Thor against the running pair, with the same network as `sim_isaac.sh stack --net eth`:
   ```bash
   export ROS_DOMAIN_ID=42 F1TENTH_DISCOVERY_SERVER=${F1TENTH_SIM_DISCOVERY_IP:-10.42.0.2}:11811
   source scripts/env/jazzy.sh          # turns F1TENTH_DISCOVERY_SERVER into ROS_DISCOVERY_SERVER
   unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY
   C="env ROS_SUPER_CLIENT=TRUE python3 tools/sim/e2e_checks.py"
   $C hz; $C perception; $C reset
   $C mission --mission "$(ros2 pkg prefix f1tenth_behavior)/share/f1tenth_behavior/missions/drive_test_straight_2m.json" --expect-distance 2.0
   ```
   The perception and reset defaults (`--wall-x 6.0`, `--start-x 1.0 --start-y 1.5`) fit base_room only. `tools/e2e_test.sh` is the one-machine variant.
6. Check the camera bandwidth over the cable. The stack subscribes to `/camera/image_raw` (bgra8) and `/zed2/zed_node/depth/depth_registered` (32FC1). At 640x360 and 15 Hz each is about 13.8 MB/s, so about 28 MB/s (220 Mbit/s) in total. The left/right rectified images have no subscriber on the Thor, so Fast DDS does not send them over the cable. Turning the right camera off (`--set sensors.stereo.right_camera=false`) only saves render time and VRAM on linus. If the link is too slow, lower `sensors.stereo.rate`.

## 7. Isaac: the parked car creeps (isaac_ws D25)
The parked car moves backwards at -6.5 mm/s with a yaw rate of -0.005 rad/s while its wheel targets are 0. Three fixes were tried and none helped. Ideas not yet tried:
- the wheels' collision approximation and contact offset;
- the friction combine mode;
- the 20 passive suspension DOFs of the Leatherback.

## 8. Isaac: the scene person is T-posed (isaac_ws D26, jazzy port D23)
The static `M_Medical_01.usd` has its arms out (about a 1.5 m span), so YOLO's box is about 1.4 m wide, and `go_to_person` stops 0.77 m short of the commanded gap (ground truth). To fix it, give the character an idle pose: an animation graph via omni.anim.people, or a posed asset. The e2e test (`tools/e2e_test.sh`, step 8) prints `person_gap_m` to check the fix.
