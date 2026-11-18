2026-10-16 13:30:31  T1 launch supervisor_bringup (no logger)
launch pid=4509 pgid=4509 sid=4509
2026-10-16 13:33:05  SIGINT to launch group 4509

## 2026-10-16 live CPU test — ABORTED in phase 1 (no /scan)

| time | event |
|---|---|
| 13:29:59 | pre-flight: ZED 2i present (2b03:f880 + HID f881) |
| 13:29:59 | pre-flight: TCP 192.168.0.10:10940 **open** -> hokuyo_ok |
| 13:30:09 | nvpmodel MODE_50W (mode 3); clocks 8x1497600 + 4x729600 kHz |
| 13:30:09 | temps cpu 47.8 / gpu 45.3 / cv0 45.4 / cv1 43.8 / cv2 47.8 C; soc*/tj unreadable (EAGAIN) |
| 13:30:09 | df ~: 551G free (37%); ROS_DISCOVERY_SERVER=127.0.0.1:11811 |
| 13:30:34 | T1 launched: supervisor_bringup.launch.py, no logger, pid/pgid/sid 4509, log output/live/stack_nolog.log |
| 13:31:01..13:31:47 | probe x6 (6 s each, 46 s): /scan 0 msgs, /perception/walls 0 msgs, both present in the graph |
| 13:31:43+ | lidar log: "could not open ethernet port" / "Error connecting to Hokuyo 192.168.0.10:10940", retry every 2.5 s |
| 13:32:08 | one urg_node_driver only (pid 4692, mine); no local socket to 10940 in `ss -tnp` |
| 13:32:08 | raw TCP probe now **FAILS** (it had succeeded at 13:29:59) |
| 13:32:3x | ping 192.168.0.10: 4/4 and 5/5 replies, rtt 0.40/0.63/0.71 ms -> link + IP fine, port 10940 refused |
| 13:33:05 | SIGINT to process group -4509 |
| 13:33:15 | launch gone; all 21 supervisor-tracked pgids gone; zero stack processes left |
| 13:33:3x | TCP 10940 still FAILS twice with urg_node dead -> lidar-side socket state, not a client leak |

Side observation from stack_nolog.log (no logger session): with
`enable_mission_logger:=false` the mission_logger component exits 0 immediately and
the supervisor logs it as a crash, respawning it on a 2 s cadence for the whole
session (restart budget exhausted, then re-armed, loop). Each respawn is a fresh
`ros2 launch` python process. Not measured, but a CPU candidate for phase b/d.

No motion commands were issued. No code changes. Phases a/b/c/d NOT run.
2026-10-16 13:36:27  retry: ping lidar (no tcp probe)
2026-10-16 13:36:35  T1 relaunch supervisor_bringup (no logger)
launch pid=pgid=sid=10383
2026-10-16 13:39:02  SIGINT to launch group 10383

## retry 13:36 — same abort, now with the exact failure mode

| time | event |
|---|---|
| 13:36:27 | ping 192.168.0.10: 4/4, rtt 0.66/0.72/0.85 ms (no raw TCP probe this time) |
| 13:36:35 | T1 relaunched, pid=pgid=sid 10383, same log path |
| 13:36:55..13:37:41 | probe x6 (46 s): /scan 0 msg, /perception/walls 0 msg |
| 13:36:42 | first urg_node error of this run, then every ~2.5 s |
| 13:37:57 | connect_ex(192.168.0.10,10940) -> rc 11 EAGAIN after the full 3 s timeout = SYN unanswered |
| 13:37:57 | `ss -tan`: one SYN-SENT 192.168.0.15:57276 -> 192.168.0.10:10940, no RST, no SYN-ACK |
| 13:37:57 | NOT port exhaustion: range 32768-60999, 24 TCP sockets total, 0 timewait, conntrack 260/262144 |
| 13:38:36 | port **80** also times out (3.00 s, no RST) -> every TCP port drops SYNs, not just 10940 |
| 13:38:36 | ARP 192.168.0.10 lladdr 00:1d:9b:1e:1f:d7 REACHABLE; Orin eno1 192.168.0.15/24 |
| 13:39:02 | SIGINT to -10383; 13:39:11 launch gone, all tracked pgids gone, zero stack processes |

Diagnosis: the device answers ARP and ICMP but silently drops TCP SYNs on both 10940 and
80 (timeout, never a refusal). That is the sensor's own TCP stack, not the Orin: no socket
leak, no port exhaustion, link and IP layer healthy. A device that had really just
rebooted would either answer or refuse.

## 13:45 — user's own stack, same failure; network ruled out entirely

| time | event |
|---|---|
| 13:45:25 | connect 10940 and 80: both rc 11 EAGAIN after the full 3 s -> SYNs still dropped |
| 13:45:25 | ping 3/3, ttl 80, 0.65-0.77 ms, no DUP! -> no duplicate address |
| 13:45:3x | swept 192.168.0.1-254: **only** 192.168.0.10 answers (00:1d:9b:1e:1f:d7) -> no other client can be holding the sensor's single session |
| 13:45:3x | eno1: 100 Mb/s full duplex, link detected, RX/TX errors 0, dropped 0 |
| 13:46:02 | user's stack (pid 15767, sid 3636, started ~13:43, launched by the user - NOT killed by me) |
| 13:46:02 | its urg_node (pid 15870, fd 19) socket: SYN-SENT 192.168.0.15:56278 -> 192.168.0.10:10940 |
| 13:46:0x | /scan 0 msg, /perception/walls 0 msg, /perception/virtual_wall/debug 0 msg in 8 s |

Network, cabling, addressing and client side are all clean. The only TCP session that
worked today was the pre-flight probe at 13:29:59; the sensor has dropped every SYN since
~13:31:43. Powered + green link LED proves the PHY is up, not that the sensor rebooted.
2026-10-16 13:48:58  T1 launch #3 (no logger), user-requested
launch pid=34613

## 13:49 — launch #3 (user-requested) + the proof of where the fault is

| time | event |
|---|---|
| 13:48:5x | branch confirmed: campaign-final @ 9c46add; user's stack had exited, no remnants |
| 13:49:14 | T1 launch #3, pid 34613; urg_node 34887 SYN-SENT 192.168.0.15:44416 -> .10:10940 |
| 13:50:0x | urg_node SIGSTOPped (reversible), 3 connects, 3 s each: eno1 TX **+9** = 3x(SYN+2 retransmits) -> our SYNs do leave the NIC; SIGCONT after |
| 13:50:0x | local TCP to own 192.168.0.15:10941 connects rc=0 -> no OUTPUT rule on the port |
| 13:50:4x | kernel counters over 3 more connects: Tcp:ActiveOpens +3, TCPSynRetrans +3, TCPTimeouts +3, TcpTimeoutRehash +3. No SYN-ACK, no RST, no InErrs, no InCsumErrors |
| 13:50:4x | the 10 eno1 RX packets = IpExt:InMcastPkts +9 / Udp:IgnoredMulti +9 (multicast noise), NOT sensor TCP |
| 13:51:15 | 17 ports probed (21,22,23,80,102,443,502,1000,2000,8000,8080,10940-43,19000,30000): **all 17 time out identically**, not one RST |

VERDICT: a healthy IP stack answers a closed port with RST. This device answers ARP and
ICMP and emits **zero** TCP segments on any port. ARP/ICMP are served below the TCP layer,
so the sensor's firmware is up but its TCP stack is dead. Orin side is fully exonerated:
SYNs reach the wire, no firewall, no port exhaustion, no socket leak, no checksum errors,
0 NIC errors, 100 Mb/s full duplex, nothing else on the subnet, no duplicate IP.
Config is correct: sensors.yaml ip_address 192.168.0.10, ip_port 10940, multiecho false.

The only state change between a working TCP server (pre-flight connect at 13:29:59 OK) and
a dead one (~13:31:43 onward) was that pre-flight raw connect, closed by bash without a
SCIP quit. It needs power actually removed; a green link LED does not mean the CPU rebooted.
2026-10-16 14:00:04  /scan check before phase a
2026-10-16 14:01:16  T1 launch #4 (no logger) + phase a
launch pid=13975
2026-10-16 14:04:33  phase a start (ROS_SUPER_CLIENT=TRUE)
2026-10-16 14:05:40  phase a done
2026-10-16 14:06:52  stop stack #4 (pgid 13975), start logger session
2026-10-16 14:07:01  T2 launch (enable_mission_logger:=true)
T2 launch pid=23988
2026-10-16 14:08:23  phase b start
2026-10-16 14:09:31  phase b done
2026-10-16 14:10:56  stop T2 (pgid 23988)

## 14:01-14:11 — the real blocker, then phases a and b

| time | event |
|---|---|
| 14:01:21 | after the power cycle urg_node connects: "Connected to network device with intensity and ID: B2128130" / "Streaming data." |
| 14:02:04 | **ZED container dies**: exit 1, "Error opening camera: CAMERA NOT DETECTED" x3 then "Camera detection timeout" (lsusb still shows the ZED 2i) |
| 14:02:1x | external rclpy subscriber: /scan sensor_data 0 msg, reliable 0 msg, **publishers of /scan seen: 0**, while lidar_front_wall_node logs "299 scans p50 4.70 p95 7.13" |
| 14:03:4x | root cause of every 0 Hz probe: plain Discovery-Server CLIENT gets no EDP relay. Fix is already documented in src/f1tenth_bringup/launch/foxglove_bridge.launch.py: **ROS_SUPER_CLIENT=TRUE** |
| 14:04:0x | same probe with ROS_SUPER_CLIENT=TRUE: /scan **40.3 Hz**, walls **10.45**, virtual_wall/debug **40.26** |
| 14:04:33-14:05:40 | phase a (60 s), no logger -> output/cpu_test/live/phase_a |
| 14:06:52-14:07:06 | stack #4 stopped, T2 launched with enable_mission_logger:=true (pid 23988); lidar reconnects in 14 s |
| 14:08:23-14:09:31 | phase b (60 s), logger live (sqlite3; mcap writer not installed) -> phase_b |
| 14:10:56-14:11:07 | T2 stopped: launch gone, all tracked pgids gone, no remnants, lidar socket closed |

NOTE for every past cpu_test run: without ROS_SUPER_CLIENT=TRUE its rates.json is all zeros/nulls
by construction, not because the topics were silent.
