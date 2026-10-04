# Orin: Humble reference runs for Phase 4 (behaviour tree + extract)

This produces the Humble side of two Phase 4 comparisons on the Orin (Humble,
repo at `e47e646`, user `fabiocar`):

- **A. Three behaviour-tree replays** (about 1 min each): the same BT input bag
  and the same mission, loaded and started at the same bag time, as the three
  Jazzy runs on Thor.
- **B. One extract** (seconds) of the archived `humble_obstacle_run` bag with
  Humble's own `mission_extract`. Thor compares its *content* against Thor's
  extract of the same bag.
- It also records the Orin's py_trees / py_trees_ros versions.

Safety:
- The BT replay runs `behavior_executor_node` (+ `twist_to_ackermann_node`)
  on isolated `ROS_DOMAIN_ID=78` with localhost-only discovery.
- Three **name-only stub nodes** (`mpc_corr`, `ackermann_to_vesc_node`,
  `costmap_boundary_node`) satisfy the mission-start preflight. They publish
  nothing.
- No driver, no mux, nothing that reaches the VESC.
- **Stop the car stack before starting.** Step 2 checks this.

Everything below can be copied and pasted. Fill in the Orin's address on the
`ORIN=` lines. If any command errors, or any check doesn't match, stop and
send me the output.

## 1. On Thor: copy the harness and the source bag

```bash
ORIN=fabiocar@<orin-host-or-ip>      # <- fill in
cd ~/dev_ws/f1tenth_more
ssh "$ORIN" 'mkdir -p ~/jazzy_parity_bt/bag'
scp -r scripts/jazzy_parity "$ORIN":~/jazzy_parity_bt/
scp ~/bags/humble_reference/humble_obstacle_run/bag/bag_0.db3 \
    ~/bags/humble_reference/humble_obstacle_run/bag/metadata.yaml \
    "$ORIN":~/jazzy_parity_bt/bag/
```

The bag is about 250 MB. If the Phase 3 copy is still at
`~/jazzy_parity_mpc/bag`, you can skip the bag `scp` and run
`cp ~/jazzy_parity_mpc/bag/* ~/jazzy_parity_bt/bag/` on the Orin instead.

## 2. On the Orin: checks and environment record

```bash
ssh fabiocar@<orin-host-or-ip>
```

Then, on the Orin:

```bash
cd ~/jazzy_parity_bt
pgrep -af "vesc|ackermann|mpc_corr|behavior_executor|ros2 launch|stack_bringup" || echo "CLEAN"
# -> must print CLEAN

sha256sum bag/bag_0.db3
# expect 7f9327a7c3d3fb9c05a98fcde638fb4f18673557dcd4ed2388698a7c0994781c
git -C ~/dev_ws/f1tenth_more rev-parse HEAD
# expect e47e64683fb61e63a9c393c3df91cf3df89cb190
git -C ~/dev_ws/f1tenth_more status --short -- src/f1tenth_behavior src/f1tenth_intelligence \
    src/f1tenth_params src/f1tenth_messages src/f1tenth_logger
# expect no output

source /opt/ros/humble/setup.bash
source ~/dev_ws/f1tenth_more/install/setup.bash
{
  echo '--- py_trees / py_trees_ros (Thor/Jazzy has 2.5.0 / 2.5.0)'
  dpkg -l | grep -i "py-trees"
  pip3 list 2>/dev/null | grep -i "py-trees\|py_trees"
  echo '--- python / numpy / pyarrow'
  python3 -c "import numpy, sys, pyarrow; print(numpy.__version__, pyarrow.__version__, sys.version)"
  echo '--- ros2 bag play --delay supported (must be 1)'
  ros2 bag play --help | grep -c -- "--delay"
} 2>&1 | tee orin_env.txt
```

## 3. Part A: three BT replays

```bash
cd ~/jazzy_parity_bt/jazzy_parity
export F1TENTH_REPO=~/dev_ws/f1tenth_more

python3 filter_bag_for_layer.py ../bag ../bt_input \
    --topics /odometry/filtered /ekf_global/odometry/filtered /scan \
             /diagnostics/system_status /camera/detections /costmap/semantic_tracks \
             /costmap/front_clearance \
    --rename /costmap/front_clearance /perception/front_distance
python3 bag_digest.py ../bt_input | tee ../orin_bt_input_digest.txt
# last line must be:
# content_sha256 89146b2def46b1417cf49964d0fc4ae28feb14b4d157a0cde613adcd581e05c5

export BT_BAG_T0_NS=1790176693141120239   # the bag's start, same value Thor used
for i in 1 2 3; do
  bash replay_localization.sh bt ../bt_input ../runs/bt_humble_$i 78
  sleep 2
done
grep -h '"success"' ../runs/bt_humble_*/mission_driver.json
# expect, per run: load success true, then start success true ("starting in 3.0 s")
```

Run these one after another, as written. Two replays started at once on the
same machine kill each other's processes. If a run prints
`FATAL: /behavior_executor_node never appeared in the graph`, rerun just that
run. This was seen once on Thor, on the first run after a long idle period.

## 4. Part B: extract the archived bag with Humble's mission_extract

```bash
cd ~/jazzy_parity_bt
python3 -m f1tenth_logger.mission_extract bag --out-dir extract_orin 2>&1 | tail -1
ls extract_orin
```

## 5. Pack the results; on Thor, copy them back

On the Orin:

```bash
cd ~/jazzy_parity_bt
tar czf orin_bt_results.tgz orin_env.txt orin_bt_input_digest.txt runs/ extract_orin/
```

On Thor:

```bash
ORIN=fabiocar@<orin-host-or-ip>      # <- same as step 1
cd ~/dev_ws/f1tenth_more
mkdir -p output/phase4/orin
scp "$ORIN":~/jazzy_parity_bt/orin_bt_results.tgz output/phase4/orin/
tar xzf output/phase4/orin/orin_bt_results.tgz -C output/phase4/orin/
```

Then tell me the files are in place. I'll run the comparisons below and
update the Phase 4 report.

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
cd scripts/jazzy_parity
R=../../output/phase4/runs; O=../../output/phase4/orin
python3 bt_metrics.py --input-bag ../../output/phase4/inputs/bt_input \
    --out ../../output/phase4/bt_metrics_vs_humble.json \
    --runs jazzy_1=$R/bt_jazzy_1/bag jazzy_2=$R/bt_jazzy_2/bag jazzy_3=$R/bt_jazzy_3/bag \
    --humble-runs humble_1=$O/runs/bt_humble_1/bag humble_2=$O/runs/bt_humble_2/bag humble_3=$O/runs/bt_humble_3/bag \
    --recorded ~/bags/humble_reference/humble_obstacle_run/bag \
    --goal-yaml ../../output/phase3/goal_drive.yaml
python3 compare_extracts.py ../../output/phase4/reports/humble_obstacle_run/*.extract.parquet \
    $O/extract_orin/*.extract.parquet | tee ../../output/phase4/extract_thor_vs_orin.txt
```

## Cleanup (optional, on the Orin)

```bash
rm -rf ~/jazzy_parity_bt
```
