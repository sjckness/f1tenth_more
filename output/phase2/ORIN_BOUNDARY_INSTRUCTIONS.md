# Orin: frozen-input `costmap_boundary` run

This runs `costmap_boundary.py` on the Orin against the same 817 frozen inputs
Thor already ran. The point is to tell a numerics difference apart from a
reconstruction artifact.
It needs no ROS, makes no changes to the Orin's repo, takes a few seconds, and
copies about 100 KB each way.

Everything is copy-paste. The only thing to fill in is the Orin's address on
the first line.

## 1. On Thor: copy the script and inputs over

```bash
ORIN=fabiocar@<orin-host-or-ip>      # <- fill in
cd ~/dev_ws/f1tenth_more
ssh "$ORIN" 'mkdir -p ~/jazzy_parity_boundary'
scp scripts/jazzy_parity/run_boundary_frozen.py \
    output/phase2/boundary_frozen_inputs.npz \
    "$ORIN":~/jazzy_parity_boundary/
sha256sum output/phase2/boundary_frozen_inputs.npz
# expect e62bcd9fb2103b6e955b240405e6ab492097b9c761bb46f4106598d579291fd9
```

## 2. On the Orin: record the environment, then run

```bash
ssh fabiocar@<orin-host-or-ip>
```

Then, on the Orin:

```bash
# Same Python environment the production stack runs in. The ROS setup can put
# a different numpy on the path than bare python3 would see.
source /opt/ros/humble/setup.bash
source ~/dev_ws/f1tenth_more/install/setup.bash

cd ~/jazzy_parity_boundary
sha256sum boundary_frozen_inputs.npz          # must match the value from step 1

# Pull the source exactly as it was at e47e646. This works whatever the
# working tree is currently on, and touches nothing in the repo.
git -C ~/dev_ws/f1tenth_more show \
    e47e646:src/f1tenth_costmap/f1tenth_costmap/costmap_boundary.py \
    > costmap_boundary_e47e646.py
sha256sum costmap_boundary_e47e646.py
# expect 07a8c3da00d4e408544b7a20a262e6a27d36d3d3a2976d4dc8d199810a98dae9

{
  echo '--- numpy / python (requested line)'
  python3 -c "import numpy, sys; print(numpy.__version__, sys.version)"
  echo '--- numpy location and install time (bag was recorded 2026-09-23)'
  python3 -c "import numpy, os; print(os.path.dirname(numpy.__file__))"
  ls -ld --time-style=full-iso "$(python3 -c 'import numpy, os; print(os.path.dirname(numpy.__file__))')"
  dpkg -l python3-numpy 2>/dev/null | tail -1
  pip3 list 2>/dev/null | grep -i '^numpy '
  grep -h ' install \| upgrade ' /var/log/dpkg.log* 2>/dev/null | grep -i numpy
  echo '--- platform'
  uname -a
  ldd --version | head -1
  echo '--- repo'
  git -C ~/dev_ws/f1tenth_more rev-parse HEAD
  git -C ~/dev_ws/f1tenth_more status --short src/f1tenth_costmap/
} 2>&1 | tee orin_env.txt

python3 run_boundary_frozen.py \
    --repo ~/dev_ws/f1tenth_more \
    --module costmap_boundary_e47e646.py \
    --inputs boundary_frozen_inputs.npz \
    --out boundary_orin_outputs.npz \
    --label orin 2>&1 | tee orin_run.txt
```

`orin_run.txt` should end with `ticks run: 817`, `constraints: 2451`, and
`module_sha256: 07a8c3da…`.

Stop and send me the output if:

- either sha256 doesn't match;
- the script errors;
- the dpkg/pip lines show numpy was installed or upgraded **after
  2026-09-23**. In that case today's numpy is not the one that produced the bag,
  and the run would not settle the question.

## 3. On Thor: copy the results back

```bash
ORIN=fabiocar@<orin-host-or-ip>      # <- same as step 1
cd ~/dev_ws/f1tenth_more
scp "$ORIN":~/jazzy_parity_boundary/{boundary_orin_outputs.npz,orin_env.txt,orin_run.txt} \
    output/phase2/
```

Then tell me the files are in place. I'll run the comparison below, update the
Phase 2 report, and commit.

```bash
python3 scripts/jazzy_parity/compare_boundary_frozen.py \
    output/phase2/boundary_thor_outputs.npz output/phase2/boundary_orin_outputs.npz \
    --inputs output/phase2/boundary_frozen_inputs.npz \
    --humble output/phase2/boundary_humble_recorded.npz \
    | tee output/phase2/boundary_thor_vs_orin.txt
```

## Cleanup (optional, on the Orin)

```bash
rm -rf ~/jazzy_parity_boundary
```
