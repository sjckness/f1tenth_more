# Obstacles v2 — log

Worktree `~/dev_ws/f1tenth_more_obst` on linus (Ubuntu 24.04, Jazzy), branch
`obstacles-v2` from `orin/campaign-final` `29dd3aa` (fetched from the Orin
2026-10-19 15:21; the Orin had just rebooted, uptime 1 min, no Claude process
running). Backup tag `backup/pre-obstacles-v2-29dd3aa` (local only). Nothing
pushed. No stack, sensor or node launched against real/sim topics; no driving.

Verification on linus (Jazzy) only: py_compile, ament_flake8/pep257, pytest.
Tests that construct rclpy nodes ran on Jazzy rclpy against a pre-built
Jazzy `f1tenth_messages` (the .msg files are byte-identical to the Orin's:
`git diff 868ec8a..29dd3aa -- src/f1tenth_messages` is empty) and the
worktree's own sources/stack_params.yaml through an ament prefix of symlinks
(no colcon build of this branch). Every such test is also listed in the
report as "to run on the Orin".

## A — the avoidance session, checked, not redone

`orin/campaign-final` 29dd3aa: A1 (`d3671b1`, wall rows default + launch-arg
forwarding), A2 (`16c91de`, hard_stop_node) and the `.git` cleanup (their A5,
no commit) are committed and documented in output/avoidance/log.md. NOT done
there: A3 beyond the premise check (no per-passage table, no PNGs, no fix for
the two defects it found), their A4 (costmap island census) and
output/avoidance/report.md. Per the work order these are reported, not
redone; B starts here.

## Bags

rsync from the Orin into ~/bags/ (Tailscale): corridor_obstacles_20261019T135307
(127 MB) and corridor10m_20261019T131815 (327 MB). Read with `rosbags`
0.11.8 deserialisers + a plain sqlite3 reader (the obstacle bag's
metadata.yaml stores offered_qos_profiles in a form rosbags' Reader rejects),
custom types registered from the repo's .msg files.
