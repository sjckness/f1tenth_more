# Orin: Humble reference runs for Phase 3 (MPC)

This produces the Humble side of the Phase 3 MPC comparison on the Orin (Humble,
repo at `e47e646`, user `fabiocar`). It has three parts:

- **A. Frozen-input solver run** (about 1 min, no ROS): re-solves the 419 MPC ticks
  captured on Thor using the Orin's Python, NumPy, SciPy and osqp. This is the
  cross-platform function-level test, and it also pins down the Orin's osqp
  version (Phase 3 Step 2).
- **B. Three open-loop replays** (about 1 min each): the same input bag and the same
  injected goal as the three Jazzy runs on Thor, through the Humble stack.
- **C. One capture replay** (about 1 min): Humble/Orin solve timing on the real
  hardware, idle. Optional, but useful for Step 5.

Safety: the replays run the MPC node only, on isolated `ROS_DOMAIN_ID=77`
with localhost-only discovery. No drivers, no mux, nothing that reaches the
VESC. **Stop the car stack before starting.** Step 2 checks this.

Everything below can be copied and pasted. The only thing to fill in is the
Orin's address on the `ORIN=` lines.
If any command errors, or any check doesn't match, stop and send me the output.

## 1. On Thor: copy the harness, inputs and source bag over

```bash
ORIN=fabiocar@<orin-host-or-ip>      # <- fill in
cd ~/dev_ws/f1tenth_more
ssh "$ORIN" 'mkdir -p ~/jazzy_parity_mpc/bag'
scp -r scripts/jazzy_parity "$ORIN":~/jazzy_parity_mpc/
scp output/phase3/mpc_frozen_inputs.npz output/phase3/goal_drive.yaml "$ORIN":~/jazzy_parity_mpc/
scp ~/bags/humble_reference/humble_obstacle_run/bag/bag_0.db3 \
    ~/bags/humble_reference/humble_obstacle_run/bag/metadata.yaml \
    "$ORIN":~/jazzy_parity_mpc/bag/
```

The bag is about 250 MB.

## 2. On the Orin: checks and environment record

```bash
ssh fabiocar@<orin-host-or-ip>
```

Then, on the Orin:

```bash
cd ~/jazzy_parity_mpc

# Nothing of the car stack may be running (no VESC driver, no mux, no MPC):
pgrep -af "vesc|ackermann|mpc_corr|ros2 launch|stack_bringup" || echo "CLEAN"
# -> must print CLEAN. If not, stop the stack first.

sha256sum bag/bag_0.db3
# expect 7f9327a7c3d3fb9c05a98fcde638fb4f18673557dcd4ed2388698a7c0994781c
sha256sum mpc_frozen_inputs.npz
# expect e04ada09707ac42014fedcb72faa5336f17f4e2b5cf3bfa2abfaffa5bab11910

git -C ~/dev_ws/f1tenth_more rev-parse HEAD
# expect e47e64683fb61e63a9c393c3df91cf3df89cb190
git -C ~/dev_ws/f1tenth_more status --short -- src/f1tenth_control src/f1tenth_params \
    src/f1tenth_behavior src/f1tenth_intelligence src/f1tenth_messages
# expect no output
(cd ~/dev_ws/f1tenth_more && sha256sum \
    src/f1tenth_control/mpc_controller/mpc_controller/mpc_solver.py \
    src/f1tenth_control/mpc_controller/mpc_controller/vehicle_model.py \
    src/f1tenth_params/f1tenth_params/object_geometry.py)
# expect 0883e2026707f46fea15ddc1cf295e0d37d576561f3f2754a986d1b663deb0df  .../mpc_solver.py
#        64257b10b8a0de6e43e853528ef0fd7011c3aed79e019d3d7925d6b8981f4775  .../vehicle_model.py
#        5380163fdb31277b585298cbf0eaa080853156b4f9308d2ad3a16bbeaa4d86b7  .../object_geometry.py

# Same Python environment the MPC node runs in:
source /opt/ros/humble/setup.bash
source ~/dev_ws/f1tenth_more/install/setup.bash
{
  echo '--- numpy / python (requested line)'
  python3 -c "import numpy, sys; print(numpy.__version__, sys.version)"
  echo '--- pip show osqp'
  pip3 show osqp
  echo '--- what the node actually imports'
  python3 -c "import osqp, scipy, numpy; print('osqp', osqp.__version__, osqp.__file__); print('scipy', scipy.__version__); print('numpy', numpy.__file__)"
  echo '--- osqp install time (bag recorded 2026-09-23)'
  ls -ld --time-style=full-iso "$(python3 -c 'import osqp, os; print(os.path.dirname(osqp.__file__))')"
  echo '--- platform'
  uname -a; nproc
  echo '--- ros2 bag play --delay supported (must be 1)'
  ros2 bag play --help | grep -c -- "--delay"
} 2>&1 | tee orin_env.txt
```

Stop and send me `orin_env.txt` if osqp was installed or upgraded **after
2026-09-23**, or if the `--delay` line doesn't print `1`.

## 3. Part A: frozen-input solver run (no ROS needed)

```bash
cd ~/jazzy_parity_mpc
python3 jazzy_parity/run_mpc_frozen.py --repo ~/dev_ws/f1tenth_more \
    --inputs mpc_frozen_inputs.npz --out mpc_orin_outputs.npz --label orin \
    2>&1 | tee orin_frozen.txt
```

This should end with `ticks solved: 419` and `wrote mpc_orin_outputs.npz`.

## 4. Part B: three Humble replays

```bash
cd ~/jazzy_parity_mpc/jazzy_parity
export F1TENTH_REPO=~/dev_ws/f1tenth_more

# The goal, rebuilt through the Humble stack's own translate -> parse_mission ->
# PublishMoveGoal path. It must match Thor's byte for byte:
python3 make_mpc_goal.py ../goal_drive_orin.yaml ~/dev_ws/f1tenth_more
diff ../goal_drive_orin.yaml ../goal_drive.yaml && echo "GOAL SAME"

# The replay input bag, built here with Humble's rosbag2:
python3 filter_bag_for_layer.py ../bag ../mpc_input \
    --topics /odometry/filtered /costmap/boundaries /perception/obstacles_2d \
             /mpc/hold /scan /tf /tf_static \
    --inject /mpc/goal_drive f1tenth_messages/msg/DriveCommand ../goal_drive_orin.yaml \
    --inject-after /odometry/filtered 0.5
python3 bag_digest.py ../mpc_input | tee ../orin_input_digest.txt
# last line must be:
# content_sha256 b621d170574ff9198fc46dad194c1c070d15360b4eaa9fed6a6b50073984da42

for i in 1 2 3; do
  bash replay_localization.sh mpc ../mpc_input ../runs/humble_$i 77
done
grep -o "re-anchored)=[^ ]* rad" ../runs/humble_*/node.log
# expect three lines ending  +0.3009 rad
grep -c "SOLVE/in" ../runs/humble_*/node.log
# expect roughly 380-420 each
```

## 5. Part C: one capture replay (Orin solve timing, idle)

```bash
cd ~/jazzy_parity_mpc/jazzy_parity
MPC_PARAMS=$HOME/jazzy_parity_mpc/runs/humble_1/params.yaml \
  bash replay_localization.sh mpc_capture ../mpc_input ../runs/humble_capture 77
tail -1 ../runs/humble_capture/node.log
# expect: captured ... solve_mpc_step calls -> .../capture.npz
```

## 6. Pack the results; on Thor, copy them back

On the Orin:

```bash
cd ~/jazzy_parity_mpc
tar czf orin_mpc_results.tgz orin_env.txt orin_frozen.txt mpc_orin_outputs.npz \
    orin_input_digest.txt goal_drive_orin.yaml runs/
ls -la orin_mpc_results.tgz
```

On Thor:

```bash
ORIN=fabiocar@<orin-host-or-ip>      # <- same as step 1
cd ~/dev_ws/f1tenth_more
mkdir -p output/phase3/orin
scp "$ORIN":~/jazzy_parity_mpc/orin_mpc_results.tgz output/phase3/orin/
tar xzf output/phase3/orin/orin_mpc_results.tgz -C output/phase3/orin/
```

Then tell me the files are in place. I'll run the comparisons below, fill in
the Phase 3 report and commit.

```bash
P=scripts/jazzy_parity; O=output/phase3/orin; R=output/phase3/runs
SRC=~/bags/humble_reference/humble_obstacle_run/bag
python3 $P/compare_mpc_frozen.py output/phase3/mpc_thor_outputs.npz $O/mpc_orin_outputs.npz \
    --inputs output/phase3/mpc_frozen_inputs.npz | tee output/phase3/mpc_frozen_thor_vs_orin.txt
python3 $P/compare_runs.py mpc - $O/runs/humble_{1,2,3}/bag --out output/phase3/humble_floor --recorded-humble $SRC
for h in 1 2 3; do
  python3 $P/compare_runs.py mpc $O/runs/humble_$h/bag $R/jazzy_{1,2,3}/bag \
      --out output/phase3/vs_humble_$h --recorded-humble $SRC
done
```

## Optional: a second osqp version (Step 2), on either machine

To test another osqp release without touching the system or `~/.local`:

```bash
pip3 install --no-deps --target ~/osqp_targets/osqp-X.Y.Z "osqp==X.Y.Z"
python3 jazzy_parity/run_mpc_frozen.py --repo ~/dev_ws/f1tenth_more \
    --inputs mpc_frozen_inputs.npz --out mpc_osqp_X.Y.Z.npz --label osqp-X.Y.Z \
    --osqp-target ~/osqp_targets/osqp-X.Y.Z
# replay with it:  OSQP_TARGET=~/osqp_targets/osqp-X.Y.Z bash replay_localization.sh mpc ...
```

## Cleanup (optional, on the Orin)

```bash
rm -rf ~/jazzy_parity_mpc
```
