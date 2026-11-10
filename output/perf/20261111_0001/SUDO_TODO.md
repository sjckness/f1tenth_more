# Needs sudo (not done: this run used none)

Each line: the command, what it is now, what it would unlock. All on linus.

| # | command | now | unlocks |
|---|---|---|---|
| 1 | `sudo sysctl kernel.yama.ptrace_scope=0` | 1 | `py-spy record -p <pid>` on a RUNNING node (py-spy itself installs with `pip install --user py-spy`, no sudo). This is the missing measurement of this baseline: mpc_corr uses 69 % of a core in the replay and its solver accounts for 11; a 30 s py-spy of the live node says where the other ~55 points go (logging, file I/O, 40 Hz callbacks). Same for front_clearance_node (40 %) and lidar_front_wall_node (28 %). Without it only `py-spy record -- python3 ...` on a process py-spy starts itself works, which the launch files do not allow without editing them. |
| 2 | `sudo sysctl kernel.perf_event_paranoid=1` | 4 | `perf stat` / `perf record -g` (perf is installed, `/usr/bin/perf`) on the C++ nodes (foxglove_bridge, the viz relay container, slam_toolbox, robot_localization) and on Python nodes' native time (numpy, scipy.ndimage, DDS serialisation). |
| 3 | `sudo cpupower frequency-set -g performance` | governor `powersave` (intel_pstate, EPP `performance`) | a fixed clock for the micro-benchmarks. Two identical runs differed by 1.7 % (median) to 8 % (max) in one pair and by 3.9 % to 12 % in another; with the governor pinned and the machine quiet the floor should drop under 2 %. |
| 4 | `sudo apt install ros-jazzy-asio-cmake-module` | missing | `vesc_driver` builds on linus (`serial_driver` is there, its `io_context` dependency needs this module), so a plain `colcon build` works instead of `--packages-ignore vesc vesc_driver vesc_ackermann`. |
| 5 | `sudo cset shield -c 7 -k on` (package `cpuset`) | no isolated core | one core with nothing else on it for `bench_micro.py --core 7`; `taskset` alone pins the benchmark but does not keep other processes off that core. |

Not sudo, but it needs the other session's agreement: the two Isaac Sim containers
(`recursing_lewin`, `more_spot_isaac`) used 2.7 of the 8 cores, 4.5 GB of the GPU and
most of the RAM during every measurement here. A level B run on a quiet machine is one
command (`scripts/perf/run_replay.sh <bag> <out>`); it should be repeated before any
phase 2 before/after comparison of a node.
