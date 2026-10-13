## What 1d00853 contains beyond the campaign base 573f313 (the dirty tree)

**Scope.** This section covers git range `573f313..1d00853` in `/home/andreas/dev_ws/f1tenth_more`. Every run of the first test campaign (P001..P005, 2026-09-21 14:55 to 2026-10-01 11:34 CEST, 58 rows in `~/matlab_ws/f1tenth_more/data/index/runs.csv`) was recorded on the Orin from branch test-logging with a dirty tree. The last eight runs, P004-R012..R019 (2026-10-01 11:16:34-11:34:50 CEST), ran at 573f313. Nothing in this section was produced by checking anything out. Everything comes from `git show/diff/log/grep` on refs, `stat`/hashes on `~/f1tenth_archive` and `~/matlab_ws` (read only), and the prior analysis `scratchpad/partA_result.json`. Each claim from that file that is cited here was re-checked, except where it is marked "not re-verified".

**The range is one commit.** `git log 573f313..1d00853` lists only `1d00853 "7/10 tests"`, authored and committed 2026-10-07 11:28:01 +0200 by `andreas`. `git rev-list --parents -n1 1d00853` gives a single parent, 573f313. No intermediate commit messages exist to date anything. The whole diff (861 files, +185691/-172) arrives at once, so per-file timing has to come from content, archive data and file mtimes.

**Headline facts verified for this section:**
- Every 2026-10-01 params snapshot is git blob `44f88f95`, the same blob as `1d00853:src/f1tenth_params/config/stack_params.yaml`. That covers all 9 runs in `~/f1tenth_archive/mission_row/2026-10-01T09-*` (the 8 campaign runs plus the trailing non-campaign run 09-36-57) and also 2026-09-30T15-28-19. The blob at 573f313 is `28ba26c5`. Each snapshot's mtime equals its run start to the second (e.g. `2026-10-01T09-16-37…params.yaml` has mtime 11:16:37 CEST), which fits a copy made at record time.
- **The steering bounds were loaded, not just present in the file.** `/drive` `cmd_steer` in `1d00853:first_test_campaing/M04_person/<run>/commands.csv` reaches max 0.319008 / min -0.309013 in P004-R012, 0.319014 / -0.309151 in R016 and 0.319014 / -0.309128 in R018, sitting at +0.319 for 10-15 samples per run (`dirtydiff/steer_minmax.py`). At 573f313 the QP bound would cap it at +0.278.
- **The steering gains were live.** Re-running `partA/dirty/servo_gain_readout.py campaign` reads left k = 0.9658, which means gain -1.0552, in R012 (n=20), R016 (n=21) and R018 (n=30). It reads right k = 1.0190, gain -1.1133, in R012 and R018. These are 1d00853's `steering_calibration.yaml` values. 573f313 has -1.2135 for both sides.

### 1. `git diff 573f313 1d00853 --stat` (verbatim, 862 lines)

```
 docs/topic_optimization/1_code_analysis.md         |  506 ++
 docs/topic_optimization/2_live_measurements.md     |  429 ++
 docs/topic_optimization/3_reconciliation.md        |  141 +
 docs/topic_optimization/4_proposal.md              |  416 ++
 .../P001-R001-20260921T145508/commands.csv         |  144 +
 .../P001-R001-20260921T145508/corridor_plot.png    |  Bin 0 -> 132409 bytes
 .../P001-R001-20260921T145508/corridors.jsonl      |    9 +
 .../P001-R001-20260921T145508/events.jsonl         |    4 +
 .../P001-R001-20260921T145508/imu.csv              |  716 +++
 .../P001-R001-20260921T145508/kinematics.csv       |  716 +++
 .../P001-R001-20260921T145508/llm_calls.csv        |    2 +
 .../P001-R001-20260921T145508/llm_calls.jsonl      |    1 +
 .../P001-R001-20260921T145508/meta.json            |   74 +
 .../P001-R001-20260921T145508/mpc.csv              |   92 +
 .../P001-R002-20260921T145711/commands.csv         |  144 +
 .../P001-R002-20260921T145711/corridor_plot.png    |  Bin 0 -> 132233 bytes
 .../P001-R002-20260921T145711/corridors.jsonl      |    9 +
 .../P001-R002-20260921T145711/events.jsonl         |    4 +
 .../P001-R002-20260921T145711/imu.csv              |  719 +++
 .../P001-R002-20260921T145711/kinematics.csv       |  719 +++
 .../P001-R002-20260921T145711/llm_calls.csv        |    2 +
 .../P001-R002-20260921T145711/llm_calls.jsonl      |    1 +
 .../P001-R002-20260921T145711/meta.json            |   74 +
 .../P001-R002-20260921T145711/mpc.csv              |   92 +
 .../P001-R003-20260921T145854/commands.csv         |  142 +
 .../P001-R003-20260921T145854/corridor_plot.png    |  Bin 0 -> 134324 bytes
 .../P001-R003-20260921T145854/corridors.jsonl      |    9 +
 .../P001-R003-20260921T145854/events.jsonl         |    4 +
 .../P001-R003-20260921T145854/imu.csv              |  733 +++
 .../P001-R003-20260921T145854/kinematics.csv       |  733 +++
 .../P001-R003-20260921T145854/llm_calls.csv        |    2 +
 .../P001-R003-20260921T145854/llm_calls.jsonl      |    1 +
 .../P001-R003-20260921T145854/meta.json            |   74 +
 .../P001-R003-20260921T145854/mpc.csv              |   88 +
 .../P001-R004-20260921T150023/commands.csv         |  140 +
 .../P001-R004-20260921T150023/corridor_plot.png    |  Bin 0 -> 129114 bytes
 .../P001-R004-20260921T150023/corridors.jsonl      |    9 +
 .../P001-R004-20260921T150023/events.jsonl         |    4 +
 .../P001-R004-20260921T150023/imu.csv              |  697 +++
 .../P001-R004-20260921T150023/kinematics.csv       |  697 +++
 .../P001-R004-20260921T150023/llm_calls.csv        |    2 +
 .../P001-R004-20260921T150023/llm_calls.jsonl      |    1 +
 .../P001-R004-20260921T150023/meta.json            |   74 +
 .../P001-R004-20260921T150023/mpc.csv              |   89 +
 .../P001-R005-20260921T150157/commands.csv         |  142 +
 .../P001-R005-20260921T150157/corridor_plot.png    |  Bin 0 -> 131784 bytes
 .../P001-R005-20260921T150157/corridors.jsonl      |    8 +
 .../P001-R005-20260921T150157/events.jsonl         |    4 +
 .../P001-R005-20260921T150157/imu.csv              |  704 +++
 .../P001-R005-20260921T150157/kinematics.csv       |  704 +++
 .../P001-R005-20260921T150157/llm_calls.csv        |    2 +
 .../P001-R005-20260921T150157/llm_calls.jsonl      |    1 +
 .../P001-R005-20260921T150157/meta.json            |   74 +
 .../P001-R005-20260921T150157/mpc.csv              |   86 +
 .../P001-R006-20260930T172816/commands.csv         |  217 +
 .../P001-R006-20260930T172816/corridor_debug.jsonl |   74 +
 .../P001-R006-20260930T172816/corridors.jsonl      |   14 +
 .../P001-R006-20260930T172816/events.jsonl         |    6 +
 .../P001-R006-20260930T172816/horizon.jsonl        |  143 +
 .../P001-R006-20260930T172816/imu.csv              | 1082 +++++
 .../P001-R006-20260930T172816/kinematics.csv       | 1080 +++++
 .../P001-R006-20260930T172816/llm_calls.csv        |    2 +
 .../P001-R006-20260930T172816/llm_calls.jsonl      |    1 +
 .../P001-R006-20260930T172816/meta.json            |   87 +
 .../P001-R006-20260930T172816/mpc.csv              |  144 +
 .../P001-R006-20260930T172816/plan.json            |   20 +
 .../P001-R006-20260930T172816/tracks.jsonl         |  287 ++
 .../P001-R007-20260930T173000/commands.csv         |  224 +
 .../P001-R007-20260930T173000/corridor_debug.jsonl |   79 +
 .../P001-R007-20260930T173000/corridors.jsonl      |   15 +
 .../P001-R007-20260930T173000/events.jsonl         |    6 +
 .../P001-R007-20260930T173000/horizon.jsonl        |  149 +
 .../P001-R007-20260930T173000/imu.csv              | 1116 +++++
 .../P001-R007-20260930T173000/kinematics.csv       | 1093 +++++
 .../P001-R007-20260930T173000/llm_calls.csv        |    2 +
 .../P001-R007-20260930T173000/llm_calls.jsonl      |    1 +
 .../P001-R007-20260930T173000/meta.json            |   87 +
 .../P001-R007-20260930T173000/mpc.csv              |  150 +
 .../P001-R007-20260930T173000/plan.json            |   20 +
 .../P001-R007-20260930T173000/tracks.jsonl         |  292 ++
 .../P001-R008-20260930T173125/commands.csv         |  423 ++
 .../P001-R008-20260930T173125/corridor_debug.jsonl |  279 ++
 .../P001-R008-20260930T173125/corridors.jsonl      |   33 +
 .../P001-R008-20260930T173125/events.jsonl         |    6 +
 .../P001-R008-20260930T173125/horizon.jsonl        |  350 ++
 .../P001-R008-20260930T173125/imu.csv              | 2111 +++++++++
 .../P001-R008-20260930T173125/kinematics.csv       | 1986 ++++++++
 .../P001-R008-20260930T173125/llm_calls.csv        |    2 +
 .../P001-R008-20260930T173125/llm_calls.jsonl      |    1 +
 .../P001-R008-20260930T173125/meta.json            |   87 +
 .../P001-R008-20260930T173125/mpc.csv              |  351 ++
 .../P001-R008-20260930T173125/plan.json            |   20 +
 .../P001-R008-20260930T173125/tracks.jsonl         |  589 +++
 .../P001-R009-20260930T173424/commands.csv         |  212 +
 .../P001-R009-20260930T173424/corridor_debug.jsonl |   69 +
 .../P001-R009-20260930T173424/corridors.jsonl      |   14 +
 .../P001-R009-20260930T173424/events.jsonl         |    6 +
 .../P001-R009-20260930T173424/horizon.jsonl        |  138 +
 .../P001-R009-20260930T173424/imu.csv              | 1059 +++++
 .../P001-R009-20260930T173424/kinematics.csv       |  955 ++++
 .../P001-R009-20260930T173424/llm_calls.csv        |    2 +
 .../P001-R009-20260930T173424/llm_calls.jsonl      |    1 +
 .../P001-R009-20260930T173424/meta.json            |   87 +
 .../P001-R009-20260930T173424/mpc.csv              |  139 +
 .../P001-R009-20260930T173424/plan.json            |   20 +
 .../P001-R009-20260930T173424/tracks.jsonl         |  273 ++
 .../P001-R010-20260930T173632/commands.csv         |  236 +
 .../P001-R010-20260930T173632/corridor_debug.jsonl |   92 +
 .../P001-R010-20260930T173632/corridors.jsonl      |   15 +
 .../P001-R010-20260930T173632/events.jsonl         |    6 +
 .../P001-R010-20260930T173632/horizon.jsonl        |  160 +
 .../P001-R010-20260930T173632/imu.csv              | 1176 +++++
 .../P001-R010-20260930T173632/kinematics.csv       | 1120 +++++
 .../P001-R010-20260930T173632/llm_calls.csv        |    2 +
 .../P001-R010-20260930T173632/llm_calls.jsonl      |    1 +
 .../P001-R010-20260930T173632/meta.json            |   87 +
 .../P001-R010-20260930T173632/mpc.csv              |  161 +
 .../P001-R010-20260930T173632/plan.json            |   20 +
 .../P001-R010-20260930T173632/tracks.jsonl         |  295 ++
 .../P001-R011-20260930T173814/commands.csv         |  212 +
 .../P001-R011-20260930T173814/corridor_debug.jsonl |   68 +
 .../P001-R011-20260930T173814/corridors.jsonl      |   13 +
 .../P001-R011-20260930T173814/events.jsonl         |    6 +
 .../P001-R011-20260930T173814/horizon.jsonl        |  138 +
 .../P001-R011-20260930T173814/imu.csv              | 1054 +++++
 .../P001-R011-20260930T173814/kinematics.csv       | 1052 +++++
 .../P001-R011-20260930T173814/llm_calls.csv        |    2 +
 .../P001-R011-20260930T173814/llm_calls.jsonl      |    1 +
 .../P001-R011-20260930T173814/meta.json            |   87 +
 .../P001-R011-20260930T173814/mpc.csv              |  139 +
 .../P001-R011-20260930T173814/plan.json            |   20 +
 .../P001-R011-20260930T173814/tracks.jsonl         |  273 ++
 .../P001-R012-20260930T173951/commands.csv         |  200 +
 .../P001-R012-20260930T173951/corridor_debug.jsonl |   55 +
 .../P001-R012-20260930T173951/corridors.jsonl      |   12 +
 .../P001-R012-20260930T173951/events.jsonl         |    6 +
 .../P001-R012-20260930T173951/horizon.jsonl        |  125 +
 .../P001-R012-20260930T173951/imu.csv              |  995 ++++
 .../P001-R012-20260930T173951/kinematics.csv       |  995 ++++
 .../P001-R012-20260930T173951/llm_calls.csv        |    2 +
 .../P001-R012-20260930T173951/llm_calls.jsonl      |    1 +
 .../P001-R012-20260930T173951/meta.json            |   87 +
 .../P001-R012-20260930T173951/mpc.csv              |  126 +
 .../P001-R012-20260930T173951/plan.json            |   20 +
 .../P001-R012-20260930T173951/tracks.jsonl         |  256 +
 first_test_campaing/M01_forward_3m/mission.json    |    6 +
 .../P002-R001-20260922T142524/commands.csv         |  171 +
 .../P002-R001-20260922T142524/corridor_plot.png    |  Bin 0 -> 132115 bytes
 .../P002-R001-20260922T142524/corridors.jsonl      |   10 +
 .../P002-R001-20260922T142524/events.jsonl         |    6 +
 .../P002-R001-20260922T142524/imu.csv              |  850 ++++
 .../P002-R001-20260922T142524/kinematics.csv       |  792 ++++
 .../P002-R001-20260922T142524/llm_calls.csv        |    2 +
 .../P002-R001-20260922T142524/llm_calls.jsonl      |    1 +
 .../P002-R001-20260922T142524/meta.json            |   85 +
 .../P002-R001-20260922T142524/mpc.csv              |   98 +
 .../P002-R001-20260922T142524/plan.json            |   21 +
 .../P002-R002-20260922T142712/commands.csv         |  168 +
 .../P002-R002-20260922T142712/corridor_plot.png    |  Bin 0 -> 122380 bytes
 .../P002-R002-20260922T142712/corridors.jsonl      |    9 +
 .../P002-R002-20260922T142712/events.jsonl         |    6 +
 .../P002-R002-20260922T142712/imu.csv              |  831 ++++
 .../P002-R002-20260922T142712/kinematics.csv       |  828 ++++
 .../P002-R002-20260922T142712/llm_calls.csv        |    2 +
 .../P002-R002-20260922T142712/llm_calls.jsonl      |    1 +
 .../P002-R002-20260922T142712/meta.json            |   85 +
 .../P002-R002-20260922T142712/mpc.csv              |   91 +
 .../P002-R002-20260922T142712/plan.json            |   21 +
 .../P002-R003-20260922T142859/commands.csv         |  167 +
 .../P002-R003-20260922T142859/corridor_plot.png    |  Bin 0 -> 133290 bytes
 .../P002-R003-20260922T142859/corridors.jsonl      |    9 +
 .../P002-R003-20260922T142859/events.jsonl         |    6 +
 .../P002-R003-20260922T142859/imu.csv              |  831 ++++
 .../P002-R003-20260922T142859/kinematics.csv       |  831 ++++
 .../P002-R003-20260922T142859/llm_calls.csv        |    2 +
 .../P002-R003-20260922T142859/llm_calls.jsonl      |    1 +
 .../P002-R003-20260922T142859/meta.json            |   85 +
 .../P002-R003-20260922T142859/mpc.csv              |   94 +
 .../P002-R003-20260922T142859/plan.json            |   21 +
 .../P002-R004-20260922T143058/commands.csv         |  165 +
 .../P002-R004-20260922T143058/corridor_plot.png    |  Bin 0 -> 130362 bytes
 .../P002-R004-20260922T143058/corridors.jsonl      |    9 +
 .../P002-R004-20260922T143058/events.jsonl         |    6 +
 .../P002-R004-20260922T143058/imu.csv              |  821 ++++
 .../P002-R004-20260922T143058/kinematics.csv       |  771 +++
 .../P002-R004-20260922T143058/llm_calls.csv        |    2 +
 .../P002-R004-20260922T143058/llm_calls.jsonl      |    1 +
 .../P002-R004-20260922T143058/meta.json            |   85 +
 .../P002-R004-20260922T143058/mpc.csv              |   90 +
 .../P002-R004-20260922T143058/plan.json            |   21 +
 .../P002-R005-20260922T143244/commands.csv         |  165 +
 .../P002-R005-20260922T143244/corridor_plot.png    |  Bin 0 -> 132689 bytes
 .../P002-R005-20260922T143244/corridors.jsonl      |    9 +
 .../P002-R005-20260922T143244/events.jsonl         |    6 +
 .../P002-R005-20260922T143244/imu.csv              |  821 ++++
 .../P002-R005-20260922T143244/kinematics.csv       |  819 ++++
 .../P002-R005-20260922T143244/llm_calls.csv        |    2 +
 .../P002-R005-20260922T143244/llm_calls.jsonl      |    1 +
 .../P002-R005-20260922T143244/meta.json            |   85 +
 .../P002-R005-20260922T143244/mpc.csv              |   90 +
 .../P002-R005-20260922T143244/plan.json            |   21 +
 .../P002-R006-20260923T170928/commands.csv         |  104 +
 .../P002-R006-20260923T170928/corridor_plot.png    |  Bin 0 -> 120597 bytes
 .../P002-R006-20260923T170928/corridors.jsonl      |    4 +
 .../P002-R006-20260923T170928/events.jsonl         |    6 +
 .../P002-R006-20260923T170928/horizon.jsonl        |   30 +
 .../P002-R006-20260923T170928/imu.csv              |  577 +++
 .../P002-R006-20260923T170928/kinematics.csv       |  560 +++
 .../P002-R006-20260923T170928/llm_calls.csv        |    2 +
 .../P002-R006-20260923T170928/llm_calls.jsonl      |    1 +
 .../P002-R006-20260923T170928/meta.json            |   87 +
 .../P002-R006-20260923T170928/mpc.csv              |   31 +
 .../P002-R006-20260923T170928/plan.json            |   21 +
 .../P002-R006-20260923T170928/tracks.jsonl         |   59 +
 .../P002-R007-20260923T171225/commands.csv         |  267 ++
 .../P002-R007-20260923T171225/corridor_debug.jsonl |  124 +
 .../P002-R007-20260923T171225/corridor_plot.png    |  Bin 0 -> 149941 bytes
 .../P002-R007-20260923T171225/corridors.jsonl      |   19 +
 .../P002-R007-20260923T171225/events.jsonl         |    6 +
 .../P002-R007-20260923T171225/horizon.jsonl        |  192 +
 .../P002-R007-20260923T171225/imu.csv              | 1331 ++++++
 .../P002-R007-20260923T171225/kinematics.csv       | 1271 +++++
 .../P002-R007-20260923T171225/llm_calls.csv        |    2 +
 .../P002-R007-20260923T171225/llm_calls.jsonl      |    1 +
 .../P002-R007-20260923T171225/meta.json            |   87 +
 .../P002-R007-20260923T171225/mpc.csv              |  193 +
 .../P002-R007-20260923T171225/plan.json            |   21 +
 .../P002-R007-20260923T171225/tracks.jsonl         |  315 ++
 .../P002-R008-20260923T171756/commands.csv         |  564 +++
 .../P002-R008-20260923T171756/corridor_debug.jsonl |  436 ++
 .../P002-R008-20260923T171756/corridor_plot.png    |  Bin 0 -> 160393 bytes
 .../P002-R008-20260923T171756/corridors.jsonl      |   53 +
 .../P002-R008-20260923T171756/events.jsonl         |    6 +
 .../P002-R008-20260923T171756/horizon.jsonl        |  492 ++
 .../P002-R008-20260923T171756/imu.csv              | 3151 +++++++++++++
 .../P002-R008-20260923T171756/kinematics.csv       | 3018 ++++++++++++
 .../P002-R008-20260923T171756/llm_calls.csv        |    2 +
 .../P002-R008-20260923T171756/llm_calls.jsonl      |    1 +
 .../P002-R008-20260923T171756/meta.json            |   87 +
 .../P002-R008-20260923T171756/mpc.csv              |  493 ++
 .../P002-R008-20260923T171756/plan.json            |   21 +
 .../P002-R008-20260923T171756/tracks.jsonl         |  419 ++
 first_test_campaing/M02_approach_wall/mission.json |    6 +
 .../P003-R001-20260928T164818/commands.csv         |  135 +
 .../P003-R001-20260928T164818/corridor_debug.jsonl |    0
 .../P003-R001-20260928T164818/corridors.jsonl      |    6 +
 .../P003-R001-20260928T164818/events.jsonl         |    6 +
 .../P003-R001-20260928T164818/horizon.jsonl        |   42 +
 .../M03_bottle/P003-R001-20260928T164818/imu.csv   |  720 +++
 .../P003-R001-20260928T164818/kinematics.csv       |  622 +++
 .../P003-R001-20260928T164818/llm_calls.csv        |    2 +
 .../P003-R001-20260928T164818/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R001-20260928T164818/meta.json |   90 +
 .../M03_bottle/P003-R001-20260928T164818/mpc.csv   |   43 +
 .../M03_bottle/P003-R001-20260928T164818/plan.json |   22 +
 .../P003-R001-20260928T164818/tracks.jsonl         |  108 +
 .../P003-R002-20260928T165040/commands.csv         |  127 +
 .../P003-R002-20260928T165040/corridor_debug.jsonl |    0
 .../P003-R002-20260928T165040/corridors.jsonl      |    4 +
 .../P003-R002-20260928T165040/events.jsonl         |    6 +
 .../P003-R002-20260928T165040/horizon.jsonl        |   35 +
 .../M03_bottle/P003-R002-20260928T165040/imu.csv   |  631 +++
 .../P003-R002-20260928T165040/kinematics.csv       |  629 +++
 .../P003-R002-20260928T165040/llm_calls.csv        |    2 +
 .../P003-R002-20260928T165040/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R002-20260928T165040/meta.json |   90 +
 .../M03_bottle/P003-R002-20260928T165040/mpc.csv   |   36 +
 .../M03_bottle/P003-R002-20260928T165040/plan.json |   22 +
 .../P003-R002-20260928T165040/tracks.jsonl         |  146 +
 .../P003-R003-20260928T165238/commands.csv         |  127 +
 .../P003-R003-20260928T165238/corridor_debug.jsonl |    0
 .../P003-R003-20260928T165238/corridors.jsonl      |    0
 .../P003-R003-20260928T165238/events.jsonl         |    6 +
 .../P003-R003-20260928T165238/horizon.jsonl        |    0
 .../M03_bottle/P003-R003-20260928T165238/imu.csv   |  628 +++
 .../P003-R003-20260928T165238/kinematics.csv       |  617 +++
 .../P003-R003-20260928T165238/llm_calls.csv        |    2 +
 .../P003-R003-20260928T165238/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R003-20260928T165238/meta.json |   86 +
 .../M03_bottle/P003-R003-20260928T165238/mpc.csv   |    1 +
 .../M03_bottle/P003-R003-20260928T165238/plan.json |   22 +
 .../P003-R003-20260928T165238/tracks.jsonl         |   87 +
 .../P003-R004-20260928T165409/commands.csv         |  170 +
 .../P003-R004-20260928T165409/corridor_debug.jsonl |   18 +
 .../P003-R004-20260928T165409/corridors.jsonl      |    8 +
 .../P003-R004-20260928T165409/events.jsonl         |    6 +
 .../P003-R004-20260928T165409/horizon.jsonl        |   58 +
 .../M03_bottle/P003-R004-20260928T165409/imu.csv   |  903 ++++
 .../P003-R004-20260928T165409/kinematics.csv       |  893 ++++
 .../P003-R004-20260928T165409/llm_calls.csv        |    2 +
 .../P003-R004-20260928T165409/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R004-20260928T165409/meta.json |   90 +
 .../M03_bottle/P003-R004-20260928T165409/mpc.csv   |   59 +
 .../M03_bottle/P003-R004-20260928T165409/plan.json |   22 +
 .../P003-R004-20260928T165409/tracks.jsonl         |  119 +
 .../P003-R005-20260928T165545/commands.csv         |  153 +
 .../P003-R005-20260928T165545/corridor_debug.jsonl |    8 +
 .../P003-R005-20260928T165545/corridors.jsonl      |   10 +
 .../P003-R005-20260928T165545/events.jsonl         |    6 +
 .../P003-R005-20260928T165545/horizon.jsonl        |   46 +
 .../M03_bottle/P003-R005-20260928T165545/imu.csv   |  790 ++++
 .../P003-R005-20260928T165545/kinematics.csv       |  786 ++++
 .../P003-R005-20260928T165545/llm_calls.csv        |    2 +
 .../P003-R005-20260928T165545/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R005-20260928T165545/meta.json |   90 +
 .../M03_bottle/P003-R005-20260928T165545/mpc.csv   |   47 +
 .../M03_bottle/P003-R005-20260928T165545/plan.json |   22 +
 .../P003-R005-20260928T165545/tracks.jsonl         |  131 +
 .../P003-R006-20260928T165817/commands.csv         |  125 +
 .../P003-R006-20260928T165817/corridor_debug.jsonl |    0
 .../P003-R006-20260928T165817/corridors.jsonl      |    9 +
 .../P003-R006-20260928T165817/events.jsonl         |    6 +
 .../P003-R006-20260928T165817/horizon.jsonl        |   47 +
 .../M03_bottle/P003-R006-20260928T165817/imu.csv   |  621 +++
 .../P003-R006-20260928T165817/kinematics.csv       |  618 +++
 .../P003-R006-20260928T165817/llm_calls.csv        |    2 +
 .../P003-R006-20260928T165817/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R006-20260928T165817/meta.json |   90 +
 .../M03_bottle/P003-R006-20260928T165817/mpc.csv   |   48 +
 .../M03_bottle/P003-R006-20260928T165817/plan.json |   22 +
 .../P003-R006-20260928T165817/tracks.jsonl         |  145 +
 .../P003-R007-20260928T165947/commands.csv         |  129 +
 .../P003-R007-20260928T165947/corridor_debug.jsonl |    0
 .../P003-R007-20260928T165947/corridors.jsonl      |    0
 .../P003-R007-20260928T165947/events.jsonl         |    6 +
 .../P003-R007-20260928T165947/horizon.jsonl        |    0
 .../M03_bottle/P003-R007-20260928T165947/imu.csv   |  642 +++
 .../P003-R007-20260928T165947/kinematics.csv       |  610 +++
 .../P003-R007-20260928T165947/llm_calls.csv        |    2 +
 .../P003-R007-20260928T165947/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R007-20260928T165947/meta.json |   86 +
 .../M03_bottle/P003-R007-20260928T165947/mpc.csv   |    1 +
 .../M03_bottle/P003-R007-20260928T165947/plan.json |   22 +
 .../P003-R007-20260928T165947/tracks.jsonl         |  114 +
 .../P003-R008-20260928T170036/commands.csv         |  126 +
 .../P003-R008-20260928T170036/corridor_debug.jsonl |    0
 .../P003-R008-20260928T170036/corridors.jsonl      |    0
 .../P003-R008-20260928T170036/events.jsonl         |    6 +
 .../P003-R008-20260928T170036/horizon.jsonl        |    0
 .../M03_bottle/P003-R008-20260928T170036/imu.csv   |  626 +++
 .../P003-R008-20260928T170036/kinematics.csv       |  547 +++
 .../P003-R008-20260928T170036/llm_calls.csv        |    2 +
 .../P003-R008-20260928T170036/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R008-20260928T170036/meta.json |   86 +
 .../M03_bottle/P003-R008-20260928T170036/mpc.csv   |    1 +
 .../M03_bottle/P003-R008-20260928T170036/plan.json |   22 +
 .../P003-R008-20260928T170036/tracks.jsonl         |  106 +
 .../P003-R009-20260928T170138/commands.csv         |  107 +
 .../P003-R009-20260928T170138/corridors.jsonl      |    3 +
 .../P003-R009-20260928T170138/events.jsonl         |    6 +
 .../P003-R009-20260928T170138/horizon.jsonl        |   26 +
 .../M03_bottle/P003-R009-20260928T170138/imu.csv   |  529 +++
 .../P003-R009-20260928T170138/kinematics.csv       |  478 ++
 .../P003-R009-20260928T170138/llm_calls.csv        |    2 +
 .../P003-R009-20260928T170138/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R009-20260928T170138/meta.json |   90 +
 .../M03_bottle/P003-R009-20260928T170138/mpc.csv   |   27 +
 .../M03_bottle/P003-R009-20260928T170138/plan.json |   22 +
 .../P003-R009-20260928T170138/tracks.jsonl         |   88 +
 .../P003-R010-20260928T170337/commands.csv         |  112 +
 .../P003-R010-20260928T170337/corridors.jsonl      |    4 +
 .../P003-R010-20260928T170337/events.jsonl         |    6 +
 .../P003-R010-20260928T170337/horizon.jsonl        |   31 +
 .../M03_bottle/P003-R010-20260928T170337/imu.csv   |  555 +++
 .../P003-R010-20260928T170337/kinematics.csv       |  531 +++
 .../P003-R010-20260928T170337/llm_calls.csv        |    2 +
 .../P003-R010-20260928T170337/llm_calls.jsonl      |    1 +
 .../M03_bottle/P003-R010-20260928T170337/meta.json |   90 +
 .../M03_bottle/P003-R010-20260928T170337/mpc.csv   |   32 +
 .../M03_bottle/P003-R010-20260928T170337/plan.json |   22 +
 .../P003-R010-20260928T170337/tracks.jsonl         |  116 +
 first_test_campaing/M03_bottle/mission.json        |    6 +
 .../P004-R001-20260922T143740/commands.csv         |  166 +
 .../P004-R001-20260922T143740/corridor_plot.png    |  Bin 0 -> 281829 bytes
 .../P004-R001-20260922T143740/corridors.jsonl      |   14 +
 .../P004-R001-20260922T143740/events.jsonl         |    6 +
 .../M04_person/P004-R001-20260922T143740/imu.csv   |  827 ++++
 .../P004-R001-20260922T143740/kinematics.csv       |  826 ++++
 .../P004-R001-20260922T143740/llm_calls.csv        |    2 +
 .../P004-R001-20260922T143740/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R001-20260922T143740/meta.json |   85 +
 .../M04_person/P004-R001-20260922T143740/mpc.csv   |   88 +
 .../M04_person/P004-R001-20260922T143740/plan.json |   22 +
 .../P004-R002-20260922T143855/commands.csv         |  149 +
 .../P004-R002-20260922T143855/corridor_plot.png    |  Bin 0 -> 241242 bytes
 .../P004-R002-20260922T143855/corridors.jsonl      |   10 +
 .../P004-R002-20260922T143855/events.jsonl         |    6 +
 .../M04_person/P004-R002-20260922T143855/imu.csv   |  745 +++
 .../P004-R002-20260922T143855/kinematics.csv       |  741 +++
 .../P004-R002-20260922T143855/llm_calls.csv        |    2 +
 .../P004-R002-20260922T143855/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R002-20260922T143855/meta.json |   85 +
 .../M04_person/P004-R002-20260922T143855/mpc.csv   |   66 +
 .../M04_person/P004-R002-20260922T143855/plan.json |   22 +
 .../P004-R003-20260923T145652/commands.csv         |  146 +
 .../P004-R003-20260923T145652/corridor_debug.jsonl |    2 +
 .../P004-R003-20260923T145652/corridor_plot.png    |  Bin 0 -> 206649 bytes
 .../P004-R003-20260923T145652/corridors.jsonl      |    7 +
 .../P004-R003-20260923T145652/events.jsonl         |    6 +
 .../M04_person/P004-R003-20260923T145652/imu.csv   |  825 ++++
 .../P004-R003-20260923T145652/kinematics.csv       |  785 ++++
 .../P004-R003-20260923T145652/llm_calls.csv        |    2 +
 .../P004-R003-20260923T145652/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R003-20260923T145652/meta.json |   89 +
 .../M04_person/P004-R003-20260923T145652/mpc.csv   |   53 +
 .../M04_person/P004-R003-20260923T145652/plan.json |   22 +
 .../P004-R004-20260923T165958/commands.csv         |  192 +
 .../P004-R004-20260923T165958/corridor_debug.jsonl |   37 +
 .../P004-R004-20260923T165958/corridor_plot.png    |  Bin 0 -> 166934 bytes
 .../P004-R004-20260923T165958/corridors.jsonl      |   12 +
 .../P004-R004-20260923T165958/events.jsonl         |    6 +
 .../P004-R004-20260923T165958/horizon.jsonl        |   94 +
 .../M04_person/P004-R004-20260923T165958/imu.csv   | 1050 +++++
 .../P004-R004-20260923T165958/kinematics.csv       |  984 ++++
 .../P004-R004-20260923T165958/llm_calls.csv        |    2 +
 .../P004-R004-20260923T165958/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R004-20260923T165958/meta.json |   90 +
 .../M04_person/P004-R004-20260923T165958/mpc.csv   |   95 +
 .../M04_person/P004-R004-20260923T165958/plan.json |   22 +
 .../P004-R004-20260923T165958/tracks.jsonl         |  115 +
 .../P004-R005-20260923T170639/commands.csv         |  162 +
 .../P004-R005-20260923T170639/corridor_debug.jsonl |   25 +
 .../P004-R005-20260923T170639/corridor_plot.png    |  Bin 0 -> 232755 bytes
 .../P004-R005-20260923T170639/corridors.jsonl      |   10 +
 .../P004-R005-20260923T170639/events.jsonl         |    6 +
 .../P004-R005-20260923T170639/horizon.jsonl        |   79 +
 .../M04_person/P004-R005-20260923T170639/imu.csv   |  906 ++++
 .../P004-R005-20260923T170639/kinematics.csv       |  876 ++++
 .../P004-R005-20260923T170639/llm_calls.csv        |    2 +
 .../P004-R005-20260923T170639/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R005-20260923T170639/meta.json |   90 +
 .../M04_person/P004-R005-20260923T170639/mpc.csv   |   80 +
 .../M04_person/P004-R005-20260923T170639/plan.json |   22 +
 .../P004-R005-20260923T170639/tracks.jsonl         |  104 +
 .../P004-R006-20260924T134512/commands.csv         |  126 +
 .../P004-R006-20260924T134512/corridor_debug.jsonl |    0
 .../P004-R006-20260924T134512/corridor_plot.png    |  Bin 0 -> 43669 bytes
 .../P004-R006-20260924T134512/corridors.jsonl      |    0
 .../P004-R006-20260924T134512/events.jsonl         |    6 +
 .../P004-R006-20260924T134512/horizon.jsonl        |    0
 .../M04_person/P004-R006-20260924T134512/imu.csv   |  624 +++
 .../P004-R006-20260924T134512/kinematics.csv       |  623 +++
 .../P004-R006-20260924T134512/llm_calls.csv        |    2 +
 .../P004-R006-20260924T134512/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R006-20260924T134512/meta.json |   86 +
 .../M04_person/P004-R006-20260924T134512/mpc.csv   |    1 +
 .../M04_person/P004-R006-20260924T134512/plan.json |   22 +
 .../P004-R006-20260924T134512/tracks.jsonl         |    0
 .../P004-R007-20260924T134658/commands.csv         |  209 +
 .../P004-R007-20260924T134658/corridor_debug.jsonl |   60 +
 .../P004-R007-20260924T134658/corridor_plot.png    |  Bin 0 -> 207841 bytes
 .../P004-R007-20260924T134658/corridors.jsonl      |   10 +
 .../P004-R007-20260924T134658/events.jsonl         |    6 +
 .../P004-R007-20260924T134658/horizon.jsonl        |   91 +
 .../M04_person/P004-R007-20260924T134658/imu.csv   | 1071 +++++
 .../P004-R007-20260924T134658/kinematics.csv       | 1055 +++++
 .../P004-R007-20260924T134658/llm_calls.csv        |    2 +
 .../P004-R007-20260924T134658/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R007-20260924T134658/meta.json |   90 +
 .../M04_person/P004-R007-20260924T134658/mpc.csv   |   92 +
 .../M04_person/P004-R007-20260924T134658/plan.json |   22 +
 .../P004-R007-20260924T134658/tracks.jsonl         |   88 +
 .../P004-R008-20260924T134911/commands.csv         |  188 +
 .../P004-R008-20260924T134911/corridor_debug.jsonl |   40 +
 .../P004-R008-20260924T134911/corridor_plot.png    |  Bin 0 -> 373950 bytes
 .../P004-R008-20260924T134911/corridors.jsonl      |   24 +
 .../P004-R008-20260924T134911/events.jsonl         |    6 +
 .../P004-R008-20260924T134911/horizon.jsonl        |  108 +
 .../M04_person/P004-R008-20260924T134911/imu.csv   |  937 ++++
 .../P004-R008-20260924T134911/kinematics.csv       |  884 ++++
 .../P004-R008-20260924T134911/llm_calls.csv        |    2 +
 .../P004-R008-20260924T134911/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R008-20260924T134911/meta.json |   90 +
 .../M04_person/P004-R008-20260924T134911/mpc.csv   |  109 +
 .../M04_person/P004-R008-20260924T134911/plan.json |   22 +
 .../P004-R008-20260924T134911/tracks.jsonl         |  206 +
 .../P004-R009-20260924T135136/commands.csv         |  227 +
 .../P004-R009-20260924T135136/corridor_debug.jsonl |   72 +
 .../P004-R009-20260924T135136/corridor_plot.png    |  Bin 0 -> 237183 bytes
 .../P004-R009-20260924T135136/corridors.jsonl      |   11 +
 .../P004-R009-20260924T135136/events.jsonl         |    6 +
 .../P004-R009-20260924T135136/horizon.jsonl        |   89 +
 .../M04_person/P004-R009-20260924T135136/imu.csv   | 1253 +++++
 .../P004-R009-20260924T135136/kinematics.csv       | 1244 +++++
 .../P004-R009-20260924T135136/llm_calls.csv        |    2 +
 .../P004-R009-20260924T135136/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R009-20260924T135136/meta.json |   90 +
 .../M04_person/P004-R009-20260924T135136/mpc.csv   |   90 +
 .../M04_person/P004-R009-20260924T135136/plan.json |   22 +
 .../P004-R009-20260924T135136/tracks.jsonl         |  152 +
 .../P004-R010-20260924T135341/commands.csv         |  189 +
 .../P004-R010-20260924T135341/corridor_debug.jsonl |   43 +
 .../P004-R010-20260924T135341/corridor_plot.png    |  Bin 0 -> 322825 bytes
 .../P004-R010-20260924T135341/corridors.jsonl      |   17 +
 .../P004-R010-20260924T135341/events.jsonl         |    6 +
 .../P004-R010-20260924T135341/horizon.jsonl        |  112 +
 .../M04_person/P004-R010-20260924T135341/imu.csv   |  943 ++++
 .../P004-R010-20260924T135341/kinematics.csv       |  926 ++++
 .../P004-R010-20260924T135341/llm_calls.csv        |    2 +
 .../P004-R010-20260924T135341/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R010-20260924T135341/meta.json |   90 +
 .../M04_person/P004-R010-20260924T135341/mpc.csv   |  113 +
 .../M04_person/P004-R010-20260924T135341/plan.json |   22 +
 .../P004-R010-20260924T135341/tracks.jsonl         |  188 +
 .../P004-R011-20260924T135544/commands.csv         |  183 +
 .../P004-R011-20260924T135544/corridor_debug.jsonl |   29 +
 .../P004-R011-20260924T135544/corridor_plot.png    |  Bin 0 -> 217106 bytes
 .../P004-R011-20260924T135544/corridors.jsonl      |   10 +
 .../P004-R011-20260924T135544/events.jsonl         |    6 +
 .../P004-R011-20260924T135544/horizon.jsonl        |   91 +
 .../M04_person/P004-R011-20260924T135544/imu.csv   |  954 ++++
 .../P004-R011-20260924T135544/kinematics.csv       |  905 ++++
 .../P004-R011-20260924T135544/llm_calls.csv        |    2 +
 .../P004-R011-20260924T135544/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R011-20260924T135544/meta.json |   90 +
 .../M04_person/P004-R011-20260924T135544/mpc.csv   |   92 +
 .../M04_person/P004-R011-20260924T135544/plan.json |   22 +
 .../P004-R011-20260924T135544/tracks.jsonl         |  123 +
 .../P004-R012-20261001T111634/commands.csv         |  187 +
 .../P004-R012-20261001T111634/corridor_debug.jsonl |   39 +
 .../P004-R012-20261001T111634/corridor_plot.png    |  Bin 0 -> 359682 bytes
 .../P004-R012-20261001T111634/corridors.jsonl      |   22 +
 .../P004-R012-20261001T111634/events.jsonl         |    6 +
 .../P004-R012-20261001T111634/horizon.jsonl        |  108 +
 .../M04_person/P004-R012-20261001T111634/imu.csv   |  930 ++++
 .../P004-R012-20261001T111634/kinematics.csv       |  929 ++++
 .../P004-R012-20261001T111634/llm_calls.csv        |    2 +
 .../P004-R012-20261001T111634/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R012-20261001T111634/meta.json |   90 +
 .../M04_person/P004-R012-20261001T111634/mpc.csv   |  109 +
 .../M04_person/P004-R012-20261001T111634/plan.json |   22 +
 .../P004-R012-20261001T111634/tracks.jsonl         |  217 +
 .../P004-R013-20261001T111911/commands.csv         |  269 ++
 .../P004-R013-20261001T111911/corridor_debug.jsonl |  106 +
 .../P004-R013-20261001T111911/corridor_plot.png    |  Bin 0 -> 133747 bytes
 .../P004-R013-20261001T111911/corridors.jsonl      |   18 +
 .../P004-R013-20261001T111911/events.jsonl         |    9 +
 .../P004-R013-20261001T111911/horizon.jsonl        |  169 +
 .../M04_person/P004-R013-20261001T111911/imu.csv   | 1426 ++++++
 .../P004-R013-20261001T111911/kinematics.csv       | 1317 ++++++
 .../P004-R013-20261001T111911/llm_calls.csv        |    2 +
 .../P004-R013-20261001T111911/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R013-20261001T111911/meta.json |   90 +
 .../M04_person/P004-R013-20261001T111911/mpc.csv   |  170 +
 .../M04_person/P004-R013-20261001T111911/plan.json |   22 +
 .../P004-R013-20261001T111911/tracks.jsonl         |  236 +
 .../P004-R014-20261001T112202/commands.csv         |  207 +
 .../P004-R014-20261001T112202/corridor_debug.jsonl |   62 +
 .../P004-R014-20261001T112202/corridor_plot.png    |  Bin 0 -> 209865 bytes
 .../P004-R014-20261001T112202/corridors.jsonl      |   16 +
 .../P004-R014-20261001T112202/events.jsonl         |    6 +
 .../P004-R014-20261001T112202/horizon.jsonl        |  128 +
 .../M04_person/P004-R014-20261001T112202/imu.csv   | 1033 ++++
 .../P004-R014-20261001T112202/kinematics.csv       | 1027 ++++
 .../P004-R014-20261001T112202/llm_calls.csv        |    2 +
 .../P004-R014-20261001T112202/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R014-20261001T112202/meta.json |   90 +
 .../M04_person/P004-R014-20261001T112202/mpc.csv   |  129 +
 .../M04_person/P004-R014-20261001T112202/plan.json |   22 +
 .../P004-R014-20261001T112202/tracks.jsonl         |  243 +
 .../P004-R015-20261001T112447/commands.csv         |  226 +
 .../P004-R015-20261001T112447/corridor_debug.jsonl |   80 +
 .../P004-R015-20261001T112447/corridor_plot.png    |  Bin 0 -> 245367 bytes
 .../P004-R015-20261001T112447/corridors.jsonl      |   19 +
 .../P004-R015-20261001T112447/events.jsonl         |    7 +
 .../P004-R015-20261001T112447/horizon.jsonl        |  146 +
 .../M04_person/P004-R015-20261001T112447/imu.csv   | 1126 +++++
 .../P004-R015-20261001T112447/kinematics.csv       | 1080 +++++
 .../P004-R015-20261001T112447/llm_calls.csv        |    2 +
 .../P004-R015-20261001T112447/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R015-20261001T112447/meta.json |   90 +
 .../M04_person/P004-R015-20261001T112447/mpc.csv   |  147 +
 .../M04_person/P004-R015-20261001T112447/plan.json |   22 +
 .../P004-R015-20261001T112447/tracks.jsonl         |  240 +
 .../P004-R016-20261001T112933/commands.csv         |  203 +
 .../P004-R016-20261001T112933/corridor_debug.jsonl |   56 +
 .../P004-R016-20261001T112933/corridor_plot.png    |  Bin 0 -> 242294 bytes
 .../P004-R016-20261001T112933/corridors.jsonl      |   17 +
 .../P004-R016-20261001T112933/events.jsonl         |    6 +
 .../P004-R016-20261001T112933/horizon.jsonl        |  123 +
 .../M04_person/P004-R016-20261001T112933/imu.csv   | 1017 ++++
 .../P004-R016-20261001T112933/kinematics.csv       | 1016 ++++
 .../P004-R016-20261001T112933/llm_calls.csv        |    2 +
 .../P004-R016-20261001T112933/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R016-20261001T112933/meta.json |   90 +
 .../M04_person/P004-R016-20261001T112933/mpc.csv   |  124 +
 .../M04_person/P004-R016-20261001T112933/plan.json |   22 +
 .../P004-R016-20261001T112933/tracks.jsonl         |  205 +
 .../P004-R017-20261001T113120/commands.csv         |  255 +
 .../P004-R017-20261001T113120/corridor_debug.jsonl |  112 +
 .../P004-R017-20261001T113120/corridor_plot.png    |  Bin 0 -> 269303 bytes
 .../P004-R017-20261001T113120/corridors.jsonl      |   21 +
 .../P004-R017-20261001T113120/events.jsonl         |    6 +
 .../P004-R017-20261001T113120/horizon.jsonl        |  129 +
 .../M04_person/P004-R017-20261001T113120/imu.csv   | 1199 +++++
 .../P004-R017-20261001T113120/kinematics.csv       | 1092 +++++
 .../P004-R017-20261001T113120/llm_calls.csv        |    2 +
 .../P004-R017-20261001T113120/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R017-20261001T113120/meta.json |   90 +
 .../M04_person/P004-R017-20261001T113120/mpc.csv   |  130 +
 .../M04_person/P004-R017-20261001T113120/plan.json |   22 +
 .../P004-R017-20261001T113120/tracks.jsonl         |  243 +
 .../P004-R018-20261001T113259/commands.csv         |  214 +
 .../P004-R018-20261001T113259/corridor_debug.jsonl |   71 +
 .../P004-R018-20261001T113259/corridor_plot.png    |  Bin 0 -> 238838 bytes
 .../P004-R018-20261001T113259/corridors.jsonl      |   20 +
 .../P004-R018-20261001T113259/events.jsonl         |    6 +
 .../P004-R018-20261001T113259/horizon.jsonl        |  137 +
 .../M04_person/P004-R018-20261001T113259/imu.csv   | 1081 +++++
 .../P004-R018-20261001T113259/kinematics.csv       | 1012 ++++
 .../P004-R018-20261001T113259/llm_calls.csv        |    2 +
 .../P004-R018-20261001T113259/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R018-20261001T113259/meta.json |   90 +
 .../M04_person/P004-R018-20261001T113259/mpc.csv   |  138 +
 .../M04_person/P004-R018-20261001T113259/plan.json |   22 +
 .../P004-R018-20261001T113259/tracks.jsonl         |  243 +
 .../P004-R019-20261001T113450/commands.csv         |  113 +
 .../P004-R019-20261001T113450/corridor_debug.jsonl |    0
 .../P004-R019-20261001T113450/corridor_plot.png    |  Bin 0 -> 163863 bytes
 .../P004-R019-20261001T113450/corridors.jsonl      |    4 +
 .../P004-R019-20261001T113450/events.jsonl         |    6 +
 .../P004-R019-20261001T113450/horizon.jsonl        |   24 +
 .../M04_person/P004-R019-20261001T113450/imu.csv   |  663 +++
 .../P004-R019-20261001T113450/kinematics.csv       |  656 +++
 .../P004-R019-20261001T113450/llm_calls.csv        |    2 +
 .../P004-R019-20261001T113450/llm_calls.jsonl      |    1 +
 .../M04_person/P004-R019-20261001T113450/meta.json |   90 +
 .../M04_person/P004-R019-20261001T113450/mpc.csv   |   25 +
 .../M04_person/P004-R019-20261001T113450/plan.json |   22 +
 .../P004-R019-20261001T113450/tracks.jsonl         |   54 +
 first_test_campaing/M04_person/mission.json        |    6 +
 .../P005-R001-20260924T135827/commands.csv         | 1170 +++++
 .../P005-R001-20260924T135827/corridor_debug.jsonl |  124 +
 .../P005-R001-20260924T135827/corridor_plot.png    |  Bin 0 -> 235884 bytes
 .../P005-R001-20260924T135827/corridors.jsonl      |   20 +
 .../P005-R001-20260924T135827/events.jsonl         |    6 +
 .../P005-R001-20260924T135827/horizon.jsonl        |  183 +
 .../P005-R001-20260924T135827/imu.csv              | 4964 ++++++++++++++++++++
 .../P005-R001-20260924T135827/kinematics.csv       | 4873 +++++++++++++++++++
 .../P005-R001-20260924T135827/llm_calls.csv        |    2 +
 .../P005-R001-20260924T135827/llm_calls.jsonl      |    1 +
 .../P005-R001-20260924T135827/meta.json            |   87 +
 .../P005-R001-20260924T135827/mpc.csv              |  184 +
 .../P005-R001-20260924T135827/plan.json            |   35 +
 .../P005-R001-20260924T135827/tracks.jsonl         | 1117 +++++
 .../P005-R002-20260924T140159/commands.csv         |  326 ++
 .../P005-R002-20260924T140159/corridor_debug.jsonl |  203 +
 .../P005-R002-20260924T140159/corridor_plot.png    |  Bin 0 -> 263030 bytes
 .../P005-R002-20260924T140159/corridors.jsonl      |   26 +
 .../P005-R002-20260924T140159/events.jsonl         |    5 +
 .../P005-R002-20260924T140159/horizon.jsonl        |  272 ++
 .../P005-R002-20260924T140159/imu.csv              | 1624 +++++++
 .../P005-R002-20260924T140159/kinematics.csv       | 1564 ++++++
 .../P005-R002-20260924T140159/llm_calls.csv        |    2 +
 .../P005-R002-20260924T140159/llm_calls.jsonl      |    1 +
 .../P005-R002-20260924T140159/meta.json            |   87 +
 .../P005-R002-20260924T140159/mpc.csv              |  273 ++
 .../P005-R002-20260924T140159/plan.json            |   36 +
 .../P005-R002-20260924T140159/tracks.jsonl         |  475 ++
 .../P005-R003-20260924T140430/commands.csv         |  347 ++
 .../P005-R003-20260924T140430/corridor_debug.jsonl |  202 +
 .../P005-R003-20260924T140430/corridor_plot.png    |  Bin 0 -> 222755 bytes
 .../P005-R003-20260924T140430/corridors.jsonl      |   26 +
 .../P005-R003-20260924T140430/events.jsonl         |    6 +
 .../P005-R003-20260924T140430/horizon.jsonl        |  270 ++
 .../P005-R003-20260924T140430/imu.csv              | 1730 +++++++
 .../P005-R003-20260924T140430/kinematics.csv       | 1622 +++++++
 .../P005-R003-20260924T140430/llm_calls.csv        |    2 +
 .../P005-R003-20260924T140430/llm_calls.jsonl      |    1 +
 .../P005-R003-20260924T140430/meta.json            |   87 +
 .../P005-R003-20260924T140430/mpc.csv              |  271 ++
 .../P005-R003-20260924T140430/plan.json            |   36 +
 .../P005-R003-20260924T140430/tracks.jsonl         |  459 ++
 .../P005-R004-20260924T140952/commands.csv         |  156 +
 .../P005-R004-20260924T140952/corridor_debug.jsonl |   34 +
 .../P005-R004-20260924T140952/corridor_plot.png    |  Bin 0 -> 173675 bytes
 .../P005-R004-20260924T140952/corridors.jsonl      |   10 +
 .../P005-R004-20260924T140952/events.jsonl         |    6 +
 .../P005-R004-20260924T140952/horizon.jsonl        |  102 +
 .../P005-R004-20260924T140952/imu.csv              | 1729 +++++++
 .../P005-R004-20260924T140952/kinematics.csv       | 1341 ++++++
 .../P005-R004-20260924T140952/llm_calls.csv        |    2 +
 .../P005-R004-20260924T140952/llm_calls.jsonl      |    1 +
 .../P005-R004-20260924T140952/meta.json            |   87 +
 .../P005-R004-20260924T140952/mpc.csv              |  103 +
 .../P005-R004-20260924T140952/plan.json            |   36 +
 .../P005-R004-20260924T140952/tracks.jsonl         |  383 ++
 .../P005-R005-20260924T141303/commands.csv         |  339 ++
 .../P005-R005-20260924T141303/corridor_debug.jsonl |  195 +
 .../P005-R005-20260924T141303/corridor_plot.png    |  Bin 0 -> 239634 bytes
 .../P005-R005-20260924T141303/corridors.jsonl      |   26 +
 .../P005-R005-20260924T141303/events.jsonl         |    6 +
 .../P005-R005-20260924T141303/horizon.jsonl        |  266 ++
 .../P005-R005-20260924T141303/imu.csv              | 1710 +++++++
 .../P005-R005-20260924T141303/kinematics.csv       | 1515 ++++++
 .../P005-R005-20260924T141303/llm_calls.csv        |    2 +
 .../P005-R005-20260924T141303/llm_calls.jsonl      |    1 +
 .../P005-R005-20260924T141303/meta.json            |   87 +
 .../P005-R005-20260924T141303/mpc.csv              |  267 ++
 .../P005-R005-20260924T141303/plan.json            |   36 +
 .../P005-R005-20260924T141303/tracks.jsonl         |  462 ++
 .../P005-R006-20260924T141529/commands.csv         |  344 ++
 .../P005-R006-20260924T141529/corridor_debug.jsonl |  200 +
 .../P005-R006-20260924T141529/corridor_plot.png    |  Bin 0 -> 241870 bytes
 .../P005-R006-20260924T141529/corridors.jsonl      |   26 +
 .../P005-R006-20260924T141529/events.jsonl         |    6 +
 .../P005-R006-20260924T141529/horizon.jsonl        |  269 ++
 .../P005-R006-20260924T141529/imu.csv              | 1722 +++++++
 .../P005-R006-20260924T141529/kinematics.csv       | 1720 +++++++
 .../P005-R006-20260924T141529/llm_calls.csv        |    2 +
 .../P005-R006-20260924T141529/llm_calls.jsonl      |    1 +
 .../P005-R006-20260924T141529/meta.json            |   87 +
 .../P005-R006-20260924T141529/mpc.csv              |  270 ++
 .../P005-R006-20260924T141529/plan.json            |   36 +
 .../P005-R006-20260924T141529/tracks.jsonl         |  457 ++
 .../P005-R007-20260924T141924/commands.csv         |  346 ++
 .../P005-R007-20260924T141924/corridor_debug.jsonl |  202 +
 .../P005-R007-20260924T141924/corridors.jsonl      |   26 +
 .../P005-R007-20260924T141924/events.jsonl         |    6 +
 .../P005-R007-20260924T141924/horizon.jsonl        |  269 ++
 .../P005-R007-20260924T141924/imu.csv              | 1735 +++++++
 .../P005-R007-20260924T141924/kinematics.csv       | 1647 +++++++
 .../P005-R007-20260924T141924/llm_calls.csv        |    2 +
 .../P005-R007-20260924T141924/llm_calls.jsonl      |    1 +
 .../P005-R007-20260924T141924/meta.json            |   87 +
 .../P005-R007-20260924T141924/mpc.csv              |  270 ++
 .../P005-R007-20260924T141924/plan.json            |   36 +
 .../P005-R007-20260924T141924/tracks.jsonl         |  483 ++
 .../P005-R008-20260924T142157/commands.csv         |  311 ++
 .../P005-R008-20260924T142157/corridor_debug.jsonl |  173 +
 .../P005-R008-20260924T142157/corridors.jsonl      |   26 +
 .../P005-R008-20260924T142157/events.jsonl         |    6 +
 .../P005-R008-20260924T142157/horizon.jsonl        |  239 +
 .../P005-R008-20260924T142157/imu.csv              | 1719 +++++++
 .../P005-R008-20260924T142157/kinematics.csv       | 1701 +++++++
 .../P005-R008-20260924T142157/llm_calls.csv        |    2 +
 .../P005-R008-20260924T142157/llm_calls.jsonl      |    1 +
 .../P005-R008-20260924T142157/meta.json            |   87 +
 .../P005-R008-20260924T142157/mpc.csv              |  240 +
 .../P005-R008-20260924T142157/plan.json            |   36 +
 .../P005-R008-20260924T142157/tracks.jsonl         |  270 ++
 .../P005-R009-20260924T142419/commands.csv         |  344 ++
 .../P005-R009-20260924T142419/corridor_debug.jsonl |  201 +
 .../P005-R009-20260924T142419/corridors.jsonl      |   26 +
 .../P005-R009-20260924T142419/events.jsonl         |    6 +
 .../P005-R009-20260924T142419/horizon.jsonl        |  270 ++
 .../P005-R009-20260924T142419/imu.csv              | 1719 +++++++
 .../P005-R009-20260924T142419/kinematics.csv       | 1635 +++++++
 .../P005-R009-20260924T142419/llm_calls.csv        |    2 +
 .../P005-R009-20260924T142419/llm_calls.jsonl      |    1 +
 .../P005-R009-20260924T142419/meta.json            |   87 +
 .../P005-R009-20260924T142419/mpc.csv              |  271 ++
 .../P005-R009-20260924T142419/plan.json            |   36 +
 .../P005-R009-20260924T142419/tracks.jsonl         |  443 ++
 .../M05_turn_left_wall/mission.json                |    6 +
 first_test_campaing/analysis/dashboard.png         |  Bin 0 -> 218668 bytes
 first_test_campaing/analysis/overview_map.png      |  Bin 0 -> 177386 bytes
 .../analysis/overview_map_M01_forward_3m.png       |  Bin 0 -> 190956 bytes
 .../analysis/overview_map_M02_approach_wall.png    |  Bin 0 -> 146801 bytes
 .../overview_map_M02_approach_wall_last5.png       |  Bin 0 -> 148341 bytes
 .../analysis/overview_map_M04_person.png           |  Bin 0 -> 405946 bytes
 .../overview_map_M05_turn_left_wall_R3-5-6-7-8.png |  Bin 0 -> 371816 bytes
 .../analysis/overview_map_P004_R007-R011.png       |  Bin 0 -> 452310 bytes
 .../analysis/overview_map_P004_R014-R018.png       |  Bin 0 -> 299449 bytes
 .../analysis/overview_map_P005_R001-R003.png       |  Bin 0 -> 295021 bytes
 .../analysis/overview_map_P005_R003-R005-R006.png  |  Bin 0 -> 310003 bytes
 first_test_campaing/analysis/report.txt            |   64 +
 .../selected/campaign_results_selected.csv         |   36 +
 .../analysis/selected/dashboard.png                |  Bin 0 -> 285759 bytes
 .../analysis/selected/overview_map.png             |  Bin 0 -> 554421 bytes
 .../overview_map_M01_20260921_R001-R005.png        |  Bin 0 -> 194189 bytes
 .../overview_map_M01_20260930_R007-R011.png        |  Bin 0 -> 252101 bytes
 .../selected/overview_map_M01_forward_3m.png       |  Bin 0 -> 299574 bytes
 .../selected/overview_map_M02_approach_wall.png    |  Bin 0 -> 183453 bytes
 .../analysis/selected/overview_map_M03_bottle.png  |  Bin 0 -> 342225 bytes
 .../overview_map_M04_20260924_R007-R011.png        |  Bin 0 -> 454402 bytes
 .../overview_map_M04_20261001_R014-R018.png        |  Bin 0 -> 300610 bytes
 .../analysis/selected/overview_map_M04_person.png  |  Bin 0 -> 534741 bytes
 .../selected/overview_map_M05_turn_left_wall.png   |  Bin 0 -> 368675 bytes
 first_test_campaing/analysis/selected/report.txt   |  132 +
 first_test_campaing/analysis/selected/summary.csv  |   12 +
 first_test_campaing/analysis/summary.csv           |    8 +
 first_test_campaing/backfill.json                  |  137 +
 first_test_campaing/campaign.json                  |    6 +
 first_test_campaing/campaign_results.csv           |   22 +
 first_test_campaing/export_settings.json           |   13 +
 first_test_campaing/prompts.yaml                   |    6 +-
 first_test_campaing/results.csv                    |   90 +
 output/mission_recorder_late_start.md              |   96 +
 scripts/check_cpu_pinning.py                       |    5 +
 scripts/drive_3m.py                                |  123 +
 .../corridor_perception/axis.py                    |  120 +-
 .../corridor_perception/tracker.py                 |  692 +++
 .../missions/bottle_then_person.json               |   25 +-
 src/f1tenth_behavior/missions/go_to_person.json    |   10 +-
 src/f1tenth_behavior/missions/test.json            |   40 +
 .../test/test_start_mission_go_to_object.py        |  101 +
 src/f1tenth_bringup/config/components.yaml         |   13 +
 src/f1tenth_bringup/config/reset_manager.yaml      |   80 +
 src/f1tenth_bringup/config/vesc.yaml               |   15 +-
 src/f1tenth_bringup/config/viz_relays.yaml         |  113 +
 .../f1tenth_bringup/component_supervisor_node.py   |    4 +-
 .../f1tenth_bringup/reset_manager_node.py          |  467 ++
 .../f1tenth_bringup/viz_jpeg_node.py               |  125 +
 .../launch/foxglove_bridge.launch.py               |   86 +-
 src/f1tenth_bringup/launch/reset_manager.launch.py |   40 +
 src/f1tenth_bringup/launch/viz_relays.launch.py    |  115 +
 src/f1tenth_bringup/package.xml                    |   15 +
 src/f1tenth_bringup/setup.py                       |    2 +
 .../test/test_foxglove_whitelists.py               |   71 +
 .../test/test_reset_manager_component.py           |   57 +
 .../mpc_controller/test/test_steering_limits.py    |   81 +-
 .../test/test_wall_turn_increment.py               |   15 +-
 .../mpc_controller/test/test_weight_set.py         |   11 +-
 .../config/steering_calibration.yaml               |   17 +-
 .../llm/llm/llm_planner_node.py                    |    8 +-
 src/f1tenth_intelligence/llm/llm/plan_translate.py |   44 +-
 .../llm/prompts/planner_system_prompt.v2.it.txt    |    8 +-
 .../planner_system_prompt.v2.no_go_to.it.txt       |    8 +-
 .../llm/test/golden/ex4_unsupported_jump.json      |    6 +-
 .../llm/test/test_intent_translate.py              |   16 +-
 .../config/test_campaign_logger.yaml               |    5 +
 .../f1tenth_logger/matlab_export/__init__.py       |   19 +
 .../f1tenth_logger/matlab_export/bag.py            |  437 ++
 .../f1tenth_logger/matlab_export/campaign.py       |  252 +
 .../f1tenth_logger/matlab_export/convert.py        |  155 +
 .../f1tenth_logger/matlab_export/db_root.py        |   86 +
 .../f1tenth_logger/matlab_export/exporter.py       |  396 ++
 .../f1tenth_logger/mission_logger_node.py          |   18 +-
 .../f1tenth_logger/test_campaign/run_select.py     |  112 +
 src/f1tenth_logger/package.xml                     |    5 +
 src/f1tenth_logger/setup.py                        |    6 +-
 .../test/test_campaign/test_run_select.py          |   75 +
 src/f1tenth_logger/test/test_matlab_export.py      |  340 ++
 .../test/test_viz_topics_not_recorded.py           |   20 +
 src/f1tenth_params/config/stack_params.yaml        |  166 +-
 src/f1tenth_perception/models/yolo26_office.pt     |  Bin 0 -> 45226166 bytes
 test_campaign/prompts.json                         |   56 +
 test_campaign/run_dry_campaign.py                  |  543 +++
 test_campaign/runs/20260918_160025/results.jsonl   |    3 +
 test_campaign/runs/20260918_160025/run_info.json   |   47 +
 test_campaign/runs/20260918_160025/summary.json    |   60 +
 test_campaign/runs/20260918_160225/results.jsonl   |    1 +
 test_campaign/runs/20260918_160225/run_info.json   |   47 +
 test_campaign/runs/20260918_160225/summary.json    |   31 +
 test_campaign/runs/20260918_160231/results.jsonl   |    1 +
 test_campaign/runs/20260918_160231/run_info.json   |   47 +
 test_campaign/runs/20260918_160231/summary.json    |   31 +
 test_campaign/runs/20260918_160413/results.jsonl   |   50 +
 test_campaign/runs/20260918_160413/run_info.json   |   43 +
 test_campaign/runs/20260918_160413/summary.json    |  550 +++
 tools/matlab/+f1db/build.m                         |   17 +
 tools/matlab/+f1db/default_root.m                  |   14 +
 tools/matlab/+f1db/open.m                          |   45 +
 tools/matlab/+f1db/private/read_index.m            |  125 +
 tools/matlab/+f1db/private/resolve_ids.m           |   26 +
 tools/matlab/+f1db/private/to_timetable.m          |   34 +
 tools/matlab/+f1db/run.m                           |   39 +
 tools/matlab/+f1db/runs.m                          |   33 +
 tools/matlab/README.md                             |  178 +
 tools/prune_runs.py                                |  501 ++
 861 files changed, 185691 insertions(+), 172 deletions(-)
```

### 2. `src/f1tenth_params/config/stack_params.yaml`: 573f313 (blob 28ba26c5) to 1d00853 (blob 44f88f95, the campaign snapshot)

`git diff 573f313 1d00853 -- src/f1tenth_params/config/stack_params.yaml` changes 2 values and adds 11 keys. Every other hunk changes only a description or a comment. Readers were found with `git grep <key> 573f313`, with stack_params.yaml itself and `*.md` excluded. Line numbers in the "1d00853" column refer to `1d00853:src/f1tenth_params/config/stack_params.yaml`.

#### 2a. Values that changed or were added

| param | 573f313 | 1d00853 (= campaign snapshot) | which node reads it (at 573f313) | behavioural impact |
|---|---|---|---|---|
| `mpc_steering_angle_min_rad` | `-0.283` (l.1274) | `-0.309` (l.1283) | `mpc_corr` (`MPC_corr.py:1281`, `declare_parameter('delta_min', get_value(...))`). The launch path is `f1tenth_control/launch/mpc_corr.launch.py:159-161` (launch arg `delta_min`, default from `get_default`) and `:287`, which passes it to the node. Tests: `test_steering_limits.py:88`, `test_wall_turn_increment.py:48`, `test_weight_set.py:306`. | Lower QP bound on δ (`mpc_solver.py:427,902`). Also the right-turn bound for wall_turn R_min (`wall_turn.py:196`, `min_turn_radius` l.113), the corridor ramp-span clamp (`MPC_corr.py:4888-4890`), the object-approach bound (`object_approach.py:474,545`) and the saturation flag (`MPC_corr.py:4033`). Right R_min goes from 1.049 to 0.955 m. Per the new comment block (1d00853 l.970-977), a 90° right turn commits at dFront 2.75 m instead of 2.97 m. **Live on 10-01:** `/drive` min is -0.309 in R012/R016/R018. |
| `mpc_steering_angle_max_rad` | `0.278` (l.1277) | `0.319` (l.1286) | `mpc_corr` (`MPC_corr.py:1284`). Launch: `mpc_corr.launch.py:162-164, 288`. Tests: `test_steering_limits.py:89`, `test_wall_turn_increment.py:49`, `test_weight_set.py:307`, `test_object_corridor_mode.py:314`. | Upper δ bound: about 15 % more left lock available to the QP. Left R_min goes from 1.069 to 0.923 m, and the 90° left commit point moves from 3.02 to 2.68 m. Steering rate time-to-lock changes (test_weight_set: 0.566 to 0.618 s on the min side). **Live on 10-01:** `/drive` max is 0.319 and stays saturated there for 10-15 samples per run. |
| `foxglove_topic_whitelist` (added) | absent | 15 anchored regexes (l.2107: `^/viz/.*`, `^/tf$`, … `^/costmap/semantic_markers$`) | No reader at 573f313 (inert). At 573f313, `foxglove_bridge.launch.py:131-141` sets no `topic_whitelist`, so the bridge default `.*` applies. In 1d00853 it is read by `foxglove_bridge.launch.py:156`. | Only with the dirty launch file: the bridge advertises only the /viz copies and pass-through topics, not raw images. Visualisation only, with no control path. It cuts Wi-Fi/CPU load (`2_live_measurements.md` reports foxglove_bridge at 51.3 % CPU idle). |
| `foxglove_client_topic_whitelist` (added) | absent | 4 regexes (l.2129: `/clicked_point`, `/goal_pose`, `/initialpose`, `/move_base_simple/goal`) | No reader at 573f313 (inert). In 1d00853: `foxglove_bridge.launch.py:174`. | Restricts what a Foxglove client may publish. No effect on autonomous runs. |
| `foxglove_send_buffer_limit` (added) | absent | `2500000` (l.2143) | No reader at 573f313 (inert). In 1d00853: `foxglove_bridge.launch.py:145-147,180`. | About 1 s of per-client backlog instead of about 4 s. Viz latency only. |
| `foxglove_use_compression` (added) | absent | `false` (l.2146) | No reader at 573f313 (inert). In 1d00853: `foxglove_bridge.launch.py:149-151,182`. | Keeps the bridge default (no deflate). None. |
| `enable_viz_relays` (added) | absent | `true` (l.2153) | No reader at 573f313 (inert). In 1d00853: `viz_relays.launch.py:63,103`, started as a second `dev_tools` launch (`1d00853:src/f1tenth_bringup/config/components.yaml`). | With the dirty bringup it starts the `/viz/*` throttle container and `viz_jpeg_node`. Those are lazy, but the container shows 51.7 % CPU idle together with the ZED (`2_live_measurements.md`, 09-24). Cores are shared with the stack, so there is a CPU effect. Nothing in the control path. |
| `viz_rate_scan` (added) | absent | `5.5` (l.2160) | No reader at 573f313 (inert). In 1d00853: `viz_relays.launch.py:38` (`RATE_KEYS`). | /viz/scan rate. Viz only. |
| `viz_rate_costmap` (added) | absent | `5.5` (l.2163) | No reader at 573f313 (inert). In 1d00853: `viz_relays.launch.py:38`. | Viz only. |
| `viz_rate_image_annotated` (added) | absent | `3.3` (l.2166) | No reader at 573f313 (inert). In 1d00853: `viz_relays.launch.py:38`. | Viz only. |
| `viz_rate_default` (added) | absent | `5.5` (l.2169) | No reader at 573f313 (inert). In 1d00853: `viz_relays.launch.py:39`. | Viz only. |
| `viz_jpeg_quality` (added) | absent | `75` (l.2172) | No reader at 573f313 (inert). In 1d00853: `viz_relays.launch.py:96`. | JPEG quality of the /viz compressed images. Viz only. |
| `foxglove_service_whitelist` (added) | absent | `'^(?!/zed2/)(?!.*/(describe_parameters\|…\|set_parameters_atomically)$).*'` (l.2178) | No reader at 573f313 (inert). In 1d00853: `foxglove_bridge.launch.py:141-143,178`. | Hides the ZED and parameter services from Foxglove's list. None on control. |

The snapshot shows these keys were in the file at record time. It does not show that the 11 foxglove/viz keys were consumed on 10-01. The snapshot is a `shutil.copyfile` at record time (`573f313:src/f1tenth_logger/f1tenth_logger/mission_logger_node.py` l.902-906, per partA), and the bags contain no trace of the relays or the bridge: `grep -a -c` for `reset_manager|viz_relay|viz_jpeg|foxglove|dev_tools` in `2026-10-01T09-16-37…/bag/bag_0.db3` and `…09-36-57…` returns 0. They count as consumed only if the dirty `foxglove_bridge.launch.py` and `viz_relays.launch.py` were also present (see section 3).

#### 2b. Hunks that change only a description or a comment (no value change)

| key or block | change | runtime effect |
|---|---|---|
| `wall_distance_max_psi_correction.description` (l.797) | "delta_max is 0.278 rad" becomes "0.319 rad". The default `0.20` is unchanged. | None. The reader is `wall_distance_node.py:208` (`param('max_psi_correction', 0.20)`), and the value did not change. |
| `corr_turn_u_start.description` (l.955) | Adds "(the pre-2026-09-17 steering bounds; 0.92 m left / 0.96 m right since)" to the R_min note. The default `0.00` is unchanged. | None. The reader is `MPC_corr.py:936-938`. |
| `mpc_steering_angle_min_rad/max_rad.description` | -16.22° becomes -17.70°, and +15.93° becomes +18.28°. | None beyond the value change in 2a. |
| wall_turn comment block (l.967-977) | R_min goes from 1.069/1.049 to 0.923/0.955 m. Trigger distances become 2.30/2.36 m (74.5°) and 2.68/2.75 m (90°). Adds a "Before the 2026-09-17 steering gain change" paragraph. | Comment only. It documents the effect of the δ bounds. |
| "WHERE … COME FROM" block (l.1245-1282) | The derivation is rewritten for gains -1.0552/-1.1133 (angles +0.31975/-0.30935 rad). The -0.264/+0.314 paragraph and the asymmetry paragraph are rewritten and a "2026-09-17" history note is added. | Comment only. It ties stack_params to `steering_calibration.yaml` (section 3), which `test_steering_limits.py` enforces. |
| Comment headers above the 11 added keys (l.2088-2106, 2124-2128, 2136-2142, 2149-2152, 2156-2159, 2175-2177) | New explanatory comments dated "2026-09-22 audit" / "Measured 2026-09-22". | Comment only. They date the foxglove/viz design to 09-22, before the 10-01 runs. |

### 3. The other files in 573f313..1d00853, classified

**Labels.** "Campaign-period" means the change was in the Orin's working tree during the campaign window (09-21 to 10-01) or earlier and was carried into it. Where it matters, each entry also says whether the change was present by the 10-01 runs and whether it could affect them. "Post-campaign" means 2026-10-02 to 10-07.

**What evidence exists.** 1d00853 has one parent and no intermediate commits, so commit messages cannot date anything. The `7/10 tests` subject says nothing about content. Archive mtimes on linus were preserved from the car: snapshot mtimes equal run start times to the second. No archive file was found whose mtime post-dates 10-01 and which reflects a source change. The only post-10-01 mtimes in the relevant trees are `~/matlab_ws/f1tenth_more/scripts/+campaign`, `+imuproc` (10-09), which are not in this diff.

#### 3a. Config with a runtime trace in campaign data

| file | class | conf. | evidence |
|---|---|---|---|
| `src/f1tenth_hardware/f1tenth_hardware/config/steering_calibration.yaml` (gain_left -1.2135 → -1.0552, gain_right -1.2135 → -1.1133, comment says "Scaled 2026-09-17") | campaign-period, **live on 10-01** | high | Bag readout via `partA/dirty/servo_gain_readout.py`, re-run for this section: R012, R016 and R018 at 573f313 give exactly -1.0552/-1.1133. The same values appear at 119f239 (P004-R001/R002, 09-22) and bc3b049 (09-24). Every campaign commit has -1.2135. |
| `src/f1tenth_bringup/config/vesc.yaml` (speed_to_erpm_gain 5499.27 → 4516.9; comment "Confirmed on the car 2026-09-21 … 4575") | campaign-period, **live on 10-01** | high that the gain was changed; medium that it was exactly 4516.9 | `partA/dirty/ratio_all.txt`: the SLAM/odom path ratio is about 1.21 through 09-17 and 1.15-1.27 at 09-21 12:35-12:45 UTC. It is about 1.00 from 2026-09-21T12-52-54 on, and 1.051/1.013/1.005/1.013 for the 10-01 runs 09-16-37/09-29-36/09-33-02/09-36-57. This was not re-derived from bags here. |
| `first_test_campaing/prompts.yaml` (M01 "3 m" → "4 m", M03 "bottle" → "potted plant", M05 "turn left at 1m from the wall" → "turn left at the wall") | campaign-period | high | The `llm_calls.jsonl` prompt field in the 1d00853 run folders shows "go ahead 3 meters" in P001-R006 (09-30 17:28) and "4 meters" from R007 on, "go to the potted plant" from P003-R001 (09-28), and "at 1m" in P005-R001 with "at the wall" from R002 (09-24). `~/f1tenth_archive/test_campaign/prompts.yaml` (mtime 2026-09-21 14:36) is the pre-edit file. |

#### 3b. LLM planner

| file | class | conf. | evidence |
|---|---|---|---|
| `src/f1tenth_intelligence/llm/llm/plan_translate.py` (`WALL_THRESH_DEFAULT_M = 2.0`, `WALL_THRESH_BEFORE_TURN_M = 3.0`, `_default_wall_thresh`) | campaign-period, live from 09-22 | high (09-22 to 09-24); **not exercised on 10-01** | In P002-R001..R008 (09-22/23) and P005-R002..R009 (09-24), `llm_raw` is `{"mode":"straight","guard":"wall"}` with no `thresh`, `rejections: []`, and a translated `front_clearance distance 2.0` (e.g. `1d00853:first_test_campaing/M02_approach_wall/P002-R001-20260922T142524/llm_calls.jsonl`). At 573f313, `schemas/intent_v1.json:22-26` requires `thresh`, so the committed translator would have rejected it. `git log --all -S WALL_THRESH_DEFAULT_M` first hits 1d00853. On 10-01, all M04 plans were `{"mode":"go_to","target":"person"}` (R012, R016 checked), which never reaches this path. |
| `llm/prompts/planner_system_prompt.v2.it.txt`, `…v2.no_go_to.it.txt` (wall default 3.0 → 2.0, plus a 3.0-before-turn rule) | campaign-period | medium | Indirect only. The model dropping `thresh` from 09-22 on matches the new plan_translate comment ("the 3B model drops thresh … whenever the prompt's wall default changes"). The prompt text itself was not logged. |
| `llm/llm/llm_planner_node.py` (v1 inline `SYSTEM_PROMPT`: 3.0 → 1.0) | campaign-period by association | low | Inert at runtime: `DEFAULT_PLANNER_PATH = 'v2'` (`573f313:…/llm_planner_node.py:566`), and every campaign `llm_raw` is v2 intent JSON. Its timing is not separately evidenced. |
| `llm/test/test_intent_translate.py`, `llm/test/golden/ex4_unsupported_jump.json` | campaign-period (they track the two rows above) | medium | They assert the 2.0/3.0 defaults. Tests only. |

#### 3c. Bringup: foxglove allowlist, /viz relays, reset_manager

| file | class | conf. | evidence |
|---|---|---|---|
| `f1tenth_bringup/launch/foxglove_bridge.launch.py`, `launch/viz_relays.launch.py`, `config/viz_relays.yaml`, `f1tenth_bringup/viz_jpeg_node.py`, `f1tenth_bringup/reset_manager_node.py`, `launch/reset_manager.launch.py`, `config/reset_manager.yaml`, `config/components.yaml` (dev_tools + viz_relays; new `reset_manager` component), `component_supervisor_node.py` (`_ALWAYS_AUTO_START` + `'reset_manager'`), `f1tenth_bringup/setup.py` (2 console_scripts), `f1tenth_bringup/package.xml` (exec_depends), `scripts/check_cpu_pinning.py` (viz_relay_container, viz_jpeg_node) | campaign-period | medium-high for presence by 09-24; **unknown** whether they ran on 10-01 | `1d00853:docs/topic_optimization/2_live_measurements.md` (run of 2026-09-24 17:08-17:55 on the car) lists `reset_manager` at 34.8 % CPU and `component_container (ZED + viz relays)` at 51.7 %. `1_code_analysis.md` l.55 lists reset_manager in the auto-start set and l.72 lists `enable_viz_relays true`. `viz_relays.yaml` is dated 2026-09-22. The stack_params keys these files read are in every snapshot from P004-R003 (09-23) to 10-01 (partA). There is no 10-01 bag trace either way (grep count 0, section 2). |
| `f1tenth_bringup/test/test_foxglove_whitelists.py`, `test/test_reset_manager_component.py` | campaign-period by association | low | Tests of the rows above. Nothing dates them separately. |

#### 3d. MPC tests that follow the steering change

| file | class | conf. | evidence |
|---|---|---|---|
| `mpc_controller/test/test_steering_limits.py`, `test_wall_turn_increment.py` (R 0.923/0.955; 0.935 → 1.083), `test_weight_set.py` (0.566 → 0.618 s; ±0.30935/0.31975) | campaign-period | medium | They encode the 09-17 gains and bounds, which were live from 09-17 (section 3a). test_steering_limits recomputes the envelope from steering_calibration.yaml, so with the new yaml it fails unless updated. Their own edit time is unknown. Tests only. |

#### 3e. Logger, MATLAB export, campaign tools

| file | class | conf. | evidence |
|---|---|---|---|
| `f1tenth_logger/matlab_export/{__init__,bag,campaign,convert,db_root,exporter}.py`, `f1tenth_logger/setup.py` (`export_matlab`), `f1tenth_logger/package.xml` (rosbags note, "0.11.5 on the Jetson"), `config/test_campaign_logger.yaml` (`matlab_export.db_root`), `test/test_matlab_export.py` | campaign-period dirty tree, but written **after the last run** (10-01 11:35 to 13:48) | high | The `~/matlab_ws/f1tenth_more/data` dir has mtime 2026-10-01 13:48:29. The first `.mat` has mtime 13:48:34 and header "Created on: Thu Oct 1 13:48:33 2026". `runs.csv` has mtime 13:50:06 and has a `mat_file` column. The test fixture uses run id `2026-10-01T09-29-36_mission-llm_x` (l.258-262), the P004-R016 archive id. No runtime effect on the campaign. |
| `tools/matlab/+f1db/*` (8 files), `tools/matlab/README.md` | 10-01 afternoon, final versions possibly later | high for 10-01 origin; unknown for the last edits | 5 of 8 files are byte-identical to `~/matlab_ws/f1tenth_more/scripts/+f1db/` copies with mtime 2026-10-01 13:51-13:52 (default_root, run, runs, resolve_ids, to_timetable). `build.m`, `open.m` and `read_index.m` differ from the 14:20-14:21 workspace copies, so those three were edited after 10-01 14:21, by 10-07 at the latest. The README example filters `date == "2026-10-01"` (l.142). |
| `f1tenth_logger/test_campaign/run_select.py`, `test/test_campaign/test_run_select.py` | 10-01 afternoon or later | medium | The docstring says it is used by "the MATLAB exporter now". Examples cite P004-R016 and P003-R009. |
| `f1tenth_logger/mission_logger_node.py` (`recordable_topics`, never record `/viz/`), `test/test_viz_topics_not_recorded.py` | unknown (after 09-22, because it references viz_relays.yaml) | low | No data trace: `_DEFAULT_TOPICS` contains no /viz topic either way, and the 10-01 manifests/bags have the same shape as before. |
| `tools/prune_runs.py` | unknown | low | Content only (archive layout, "121 of the 122 runs in complete/"). Nothing dates it. |
| `scripts/drive_3m.py` | unknown, possibly 09-21 | low | A steering-offset calibration helper that drives 3 m on /odom. vesc.yaml's comment mentions a 3 m tape run on 2026-09-21, but nothing ties this file to it. |

#### 3f. Perception, behaviour, missions, model

| file | class | conf. | evidence |
|---|---|---|---|
| `src/corridor_perception/corridor_perception/axis.py`, `tracker.py` ("Step 1") | unknown (after 09-22) | low | Step 0 is `119f239` (2026-09-22). `git grep corridor_perception 573f313 -- 'src/*/launch/*' 'src/*/config/*'` finds nothing, so it is offline and has no run impact. |
| `src/f1tenth_behavior/missions/go_to_person.json` (now `llm_00498c643b95`) | campaign-period | medium | Overwritten with a plan first run as `mission_row/2026-09-22T12-35-45_mission-llm_00498c643b95`. The last archive run with the committed id `go_to_person` is 2026-09-17T12-57-08. Inert, because campaign runs load LLM plans. |
| `src/f1tenth_behavior/missions/bottle_then_person.json` (now `llm_plan_1789054103`, schema 3.0) | pre-campaign dirty edit, carried through | medium | The epoch 1789054103 is 2026-09-10 17:28 CEST. The last run with id `bottle_then_person` is 2026-09-10T09-48-19. The last commit of the file is ace2446 (2026-09-10 13:27). Inert. |
| `src/f1tenth_behavior/missions/test.json` | unknown | low | No archive run with mission id `test`. Inert. |
| `src/f1tenth_behavior/test/test_start_mission_go_to_object.py` | unknown (after 09-17) | low | The docstring cites the 2026-09-17 crash. Test only. |
| `src/f1tenth_perception/models/yolo26_office.pt` (45 MB) | campaign-period (untracked file in the tree since ≤09-16) | high | `573f313:tools/gen_intent_target_enum.py:15-20` (last changed 22ff031, 09-16) names it as one of "the two checkpoints currently in the tree". Not the active model: 10-01 manifest `resolved_params.yolo_model = yolo26s-seg.pt`. |

#### 3g. Trivial: docs, outputs, recorded data (grouped)

| files | class | conf. | evidence |
|---|---|---|---|
| `docs/topic_optimization/1_code_analysis.md`, `2_live_measurements.md`, `3_reconciliation.md` | campaign-period (09-24) | high | Each states 2026-09-24 (l.18 / l.3 / l.3). |
| `docs/topic_optimization/4_proposal.md` | campaign-period, probably | medium | No date in the file. It is the 4th part of the 09-24 series. |
| `output/mission_recorder_late_start.md` | 10-01, after the runs | high | l.5: "the 58 runs … matched … by `export_matlab` (2026-10-01)". |
| `first_test_campaing/<mission>/<run>/*` (all 58 run folders, about 760 files), `*/mission.json`, `campaign.json`, `backfill.json`, `export_settings.json`, `results.csv`, `campaign_results.csv` | campaign-period (logger output, untracked) | high | Written by the test-campaign logger into the repo (`campaign_dir /home/fabiocar/…/first_test_campaing`, `1d00853:first_test_campaing/analysis/report.txt` l.1). `campaign.json` and `backfill.json` are byte-identical to `~/f1tenth_archive/test_campaign/` copies (mtime 2026-09-21 13:32 / 16:00). `export_settings.json` and `campaign_results.csv` differ from those 09-21 copies, so they were later rewritten (date unknown). |
| `first_test_campaing/analysis/**` (PNGs, report.txt, summary.csv, `selected/`) | 10-01 or later; unknown whether before or after 10-02 | medium | File names include `P004_R014-R018` and `M04_20261001_R014-R018`, so they post-date 10-01 11:33. |
| `test_campaign/prompts.json`, `run_dry_campaign.py`, `runs/20260918_*` (12 files) | pre-campaign, untracked (09-18) | high | `run_info.json`: started 2026-09-18T16:00:25+02:00, prompts_file `/home/fabiocar/…`, workspace_head 8cde87f, branch scene-graph. |

**Net result.** Every file whose timing can be tested is campaign-period (09-10 to 10-01) or 10-01 after the runs. None is shown to be post-campaign (10-02 to 10-07), except possibly the last edits of `tools/matlab/+f1db/{build,open}.m` and `private/read_index.m`, and the files marked "unknown". The changes that can affect run behaviour and that the data confirms live on 10-01 are: the δ bounds, the steering gains, and the erpm gain. The plan_translate default was live but not exercised on 10-01. The foxglove/viz/reset_manager bringup work was present by 09-24 but cannot be confirmed for 10-01.

### 4. How to confirm this on the Orin

Run these read-only commands in `/home/fabiocar/dev_ws/f1tenth_more` before any checkout, pull or build.

**Reflog.** Run `git reflog --date=iso -n 300` and `git reflog show --date=iso test-logging test-campaign`. Three entries would settle the question:
- `commit: 7/10 tests` at 2026-10-07 11:28 on that machine, directly after a `checkout: moving from test-logging to test-campaign`. That proves 1d00853 was committed from the Orin's dirty tree on top of 573f313.
- No HEAD move or `reset`/`stash` between 2026-10-01 11:16 and 11:35. That proves the 8 runs shared one tree.
- The HEAD move to 573f313 itself (expected after 573f313's commit time, 2026-09-30 18:25, and before 10-01 09:16 UTC).

**mtimes.** Run `stat -c '%y %n'` on each file from section 3. These are weak evidence: the 2026-10-08 main↔test-campaign checkouts (partA hist) rewrote every file that differs between e47e646 and 1d00853, so most will show 10-08. Files that do not differ, untracked files and gitignored files keep their edit times. Examples: `first_test_campaing/**` run folders, `test_campaign/runs/*`, `src/f1tenth_perception/models/yolo26_office.pt`, `src/f1tenth_bringup/config/vesc.yaml.bak.*` (if any), `~/.ros/log/*` for 2026-10-01 09:13-09:37 UTC, `~/matlab_data` or the db_root used by `export_matlab`. Expect `.mat` files near 2026-10-01 13:48.

**git diff HEAD.** Run `git diff HEAD --stat` and `git status --porcelain -uall`. With HEAD at 1d00853 (or a descendant), anything still dirty dates from after 10-07. The expected example is the uncommitted `corr_L_base 1.5` noted in `16b7b95:output/campaign_fix_plan.md`. Corridor logs show L = 3.0 through 2026-10-08 14:29 CEST, so that edit is not campaign-time. Run `git diff 573f313 HEAD -- <file>` for each file in section 3 to confirm nothing reverted between 10-01 and 10-07.

**Launch logs.** The decisive evidence for the "unknown" bringup rows is in `~/.ros/log` for 2026-10-01 09:13-09:37 UTC: whether `reset_manager`, `viz_relay_container`/`viz_jpeg_node`, and `foxglove_bridge` with a `topic_whitelist` were started, and which `speed_to_erpm_gain` and servo gains `vesc_driver`/`ackermann_to_vesc` loaded.
