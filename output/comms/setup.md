# Base hotspot, link_manager and Wi-Fi coverage: setup and test plan

Branch `comms-base` (from `pwa-control` `9c5371d`), 2026-10-21. Phases 0–2 of
the "base + hotspot + auto-reconnect" design.

**What has run, and where.** Everything below ran on linus only: unit tests
on Jazzy and on Humble/Python 3.10 (the `f1tenth_more:cpu` image, no
network), and a 95 s synthetic run of both nodes with fake TF, Wi-Fi, base
and `nmcli`. **Nothing ran on the Orin, on the tpad or with a real hotspot.**
On linus no hotspot was brought up and no network setting was changed; the AP
checks in §3 are read-only. On the Orin, only read-only `nmcli`/`/proc`
queries were made over ssh.

---

## 1. The design

```
 BASE PC (linus or tpad)                         CAR (Orin, Humble)
 ───────────────────────                         ──────────────────
 hotspot SSID f1tenth-base   ◄──── Wi-Fi ────►   wlP1p1s0, NM profile f1tenth-base
   10.42.50.1/24, WPA2                             autoconnect priority 50
   ipv4 shared: DHCP, DNS, NAT                     eduroam (priority 0) = backup
   to this PC's internet

 heartbeat_responder         ◄── UDP 47800 ───   link_manager (every 0.5 s, one
   echoes on every address                          round to both addresses)
                                                   1. tailscale_host 100.89.193.76 (linus)
                                                   2. local_host     10.42.50.1 (fallback)
                                                   -> /comms/link_status
                                                   while LOST: nmcli connection up
                                                   (rate-limited, worker thread)

 Foxglove / PWA over Tailscale                    wifi_coverage_node (1 Hz)
                                                   RSSI + link_status + TF map->base_link
                                                   -> /comms/coverage_rssi, _rtt, _cells
```

- **The car decides for itself.** The gateway's "no client for 2 s during an
  app mission → abort" (`f1tenth_gateway/mission.py`) and the e-stop are
  unchanged. Neither node reads or writes anything on the drive, e-stop or
  mission path. No BT changes.
- **NetworkManager owns the association.** link_manager only asks it to bring
  the hotspot profile up, and only while the base is unreachable.
- **Addressing.** Tailscale first. The base's hotspot address is the
  fallback, used as soon as Tailscale stops answering (e.g. the Orin rebooted
  without internet, so Tailscale cannot log in).

## 2. What is on the branch

| Commit | Content |
|---|---|
| 1 `f1tenth_comms: the Wi-Fi coverage layer` | package `f1tenth_comms`: `wifi_coverage_node`, `wifi.py`, `coverage.py`, `tf_poll.py`; `f1tenth_messages/LinkStatus.msg`; bringup: the `comms` component, `enable_comms` (default **false**), Foxglove whitelist |
| 2 `f1tenth_comms: link_manager … + base responder` | `link_manager`, `heartbeat.py`, `reconnect.py`; `scripts/base/heartbeat_responder.py` + systemd user unit + installer; `tools/comms_synthetic_run.py` |
| 3 `f1tenth_gateway: /comms/link_status in the status payload` | the optional step: `link` in the gateway's status; the PWA does not show it yet |
| 4 `scripts + docs` | `scripts/base/hotspot.sh`, `check_ap.sh`, `hotspot.conf.example`; `scripts/orin/comms_network.sh`; this file; the Foxglove layout; the synthetic-run results |

Nothing is pushed.

## 3. AP mode: read-only checks

### linus (2026-10-21, `scripts/base/check_ap.sh`, output in `check_ap_linus_20261021.txt`)

`iw` is not installed on linus; it was run from the Ubuntu package
unpacked in a scratch directory, without installing anything.

| | |
|---|---|
| Card / driver | Realtek RTL8822CE (PCIe), `rtw88_8822ce`, regulatory domain IT (ETSI) |
| Interface modes | IBSS, managed, **AP**, AP/VLAN, monitor; NetworkManager `WIFI-PROPERTIES.AP: yes` |
| Interface combinations | `#{ managed } <= 1, #{ AP } <= 1, total <= 2, #channels <= 1`: a client and an AP **at the same time, on one channel** |
| AP-usable channels | 1–13 (2.4 GHz); 36–48 and 149–165 (5 GHz). 52–144 need radar detection: not as an AP |
| Uplink now | `wlp4s0` on **eduroam**, channel 56 (5280 MHz, a radar/DFS channel), and it carries linus's **only** default route |
| Ethernet | `enp5s0` = `thor-link`, manual 10.42.0.1/24, the cable to the Thor: no internet |
| Firewall | `ufw` active |

**Verdict for linus.**
- It can be an AP. But its one radio is also its only internet.
- *Dedicated mode* (the radio becomes the AP): eduroam goes, and so does
  linus's internet, so the car would get none. Tailscale between car and linus
  then depends on cached state; the local path (10.42.50.1) still works, which
  is exactly what the fallback is for.
- *Concurrent mode* (eduroam + a virtual AP, sharing eduroam): the radio
  allows it, but only on eduroam's channel, and **right now that is 56, where
  an AP is not allowed**. It would work while eduroam happens to be on 1–13 or
  36–48, and break when eduroam roams.
- Usable as a base with internet only with a second radio (a USB Wi-Fi adapter
  that can do AP) or another uplink (USB tethering, a lab ethernet port
  instead of the Thor cable), or with the travel router (§4.6).
- `10.42.0.0/24` is taken by thor-link: the hotspot uses **10.42.50.1/24**.

### Orin (read-only over ssh, 2026-10-21)

| | |
|---|---|
| Wi-Fi | `wlP1p1s0`, driver `rtl88x2ce`, NM AP yes; `/proc/net/wireless` gives dBm (−38 at the time); `/usr/sbin/iw` present |
| User | `fabiocar` (Ubuntu 22.04.5, Python 3.10.12) |
| `nmcli general permissions` over ssh | `network-control: auth`, `wifi.scan: auth`: **`nmcli connection up` from the stack would be refused** until the polkit rule in §5.2 is in place |
| Wi-Fi profiles | `eduroam` priority 0, `home-wifi` 10, `TIM-71892397` **100**, others 0 |

### tpad (not checked: offline on Tailscale at the time)

On the tpad, read-only:

```bash
scripts/base/check_ap.sh            # card, driver, NM AP, iw modes/combinations, verdict
ip route show default               # where its internet comes from
nmcli device                        # ethernet with internet? then dedicated mode is fine
```

With `iw` missing: `sudo apt install iw` (or only the NM `AP: yes/no` is known).
If the tpad has ethernet with internet, dedicated mode on its Wi-Fi is the
simple, robust setup.

## 4. Base setup (by hand, on linus or the tpad)

### 4.1 Config

```bash
cp scripts/base/hotspot.conf.example scripts/base/hotspot.conf    # git-ignored
$EDITOR scripts/base/hotspot.conf    # HOTSPOT_PSK, HOTSPOT_IFNAME; mode and band
```

Use the **same SSID, password and address on both bases**: the car's single
profile then joins either, and `local_host` never changes. Only
`tailscale_host` differs per base (§5.4).

### 4.2 Hotspot

```bash
scripts/base/check_ap.sh                 # read-only
scripts/base/hotspot.sh up               # asks before taking a radio that carries a connection
scripts/base/hotspot.sh status           # profile, address, clients, leases, uplink, responder, firewall
scripts/base/hotspot.sh down             # the profile stays; autoconnect is off
```

`up` creates or updates the NM profile `f1tenth-base`: `802-11-wireless.mode
ap`, WPA2 (RSN/CCMP), `ipv4.method shared` with `ipv4.addresses
10.42.50.1/24` (NM runs DHCP and DNS and NATs to the default route),
`ipv6.method disabled`, autoconnect off.

- Dedicated (`HOTSPOT_VIRTUAL=0`): band/channel from the config (default 2.4 GHz
  channel 6: range through walls matters more than throughput here).
- Concurrent (`HOTSPOT_VIRTUAL=1`, needs `iw` and sudo): adds `f1tap0` (type AP,
  its own locally administered MAC) on the same radio and puts the AP on the
  uplink's channel; refuses a radar/no-IR channel.

### 4.3 Firewall (ufw is active on linus)

```bash
IF=wlp4s0     # or f1tap0 in concurrent mode; hotspot.sh status prints it
sudo ufw allow in on $IF to any port 67 proto udp       # DHCP
sudo ufw allow in on $IF to any port 53                 # DNS
sudo ufw allow in on $IF to any port 47800 proto udp    # heartbeat via the hotspot
sudo ufw allow in on tailscale0 to any port 47800 proto udp
sudo ufw route allow in on $IF                          # only if the car has no internet
```

Not tried: NM's shared mode adds its own NAT rules; whether ufw's FORWARD
policy also needs the `route allow` line depends on the backend. Test with
`ping 1.1.1.1` from the car.

### 4.4 Heartbeat responder

```bash
scripts/base/install_heartbeat.sh          # ~/.local/bin + systemd user unit, enable --now
sudo loginctl enable-linger $USER          # keep it running without a login session
journalctl --user -u f1tenth-heartbeat -f  # every 60 s: echoes per car address and path
scripts/base/install_heartbeat.sh --uninstall
```

UDP 47800 on 0.0.0.0, so the Tailscale and the hotspot addresses are both
covered, also when they come up later. It answers only packets that start
with `F1HB` and are 16–64 bytes, with the same bytes (no amplification).
Stdlib Python, no ROS.

### 4.5 Quick check from the base itself

```bash
python3 - <<'EOF'
import socket, struct, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(1)
pkt = struct.pack('!4sBBHIIQ', b'F1HB', 1, 1, 0, 1, 1, 0)
for addr in ('10.42.50.1', '100.89.193.76'):
    t = time.monotonic(); s.sendto(pkt, (addr, 47800))
    try: s.recvfrom(64); print(addr, 'ok', round((time.monotonic() - t) * 1e3, 2), 'ms')
    except socket.timeout: print(addr, 'no answer')
EOF
```

### 4.6 Fallback: a travel router

When no base PC can be AP and keep its internet (linus today): a small
travel router (GL.iNet class) configured with **the same SSID, password and
10.42.50.0/24, gateway 10.42.50.1**, uplink by ethernet, by repeating eduroam
(WPA2-Enterprise client; many travel routers can) or by USB tethering. The
base PC joins it as a client and runs the responder.

The car needs no change, with one exception: link_manager's `local_host`
must be the base PC's address on the router (e.g. a DHCP reservation
10.42.50.2), not the router's 10.42.50.1. Pass `comms_local_host:=10.42.50.2`.

## 5. Orin setup (by hand, ON THE ORIN; nothing here was run)

### 5.1 The hotspot profile

```bash
cd ~/dev_ws/f1tenth_more
# copy the base's scripts/base/hotspot.conf to the same path here (same SSID/PSK)
scripts/orin/comms_network.sh check                    # read-only
scripts/orin/comms_network.sh install                  # creates f1tenth-base, does not activate it
```

The profile: `wlP1p1s0`, WPA2-PSK, autoconnect **priority 50** (eduroam stays
0 and stays as it is: the backup), `autoconnect-retries 0` (retry forever: NM's
default blocks a profile for 5 min after 4 failures), Wi-Fi power save off.
`TIM-71892397` (100) and `home-wifi` (10) stay above it, as they are today.

NM never leaves a working eduroam for the hotspot by itself; that switch is
link_manager's reconnect (§6.3), or by hand:

```bash
# Over ssh via Tailscale (jetson-orin = 100.96.201.7), the session survives the switch
# if the hotspot shares internet. Otherwise reach the car at its hotspot lease
# (base: scripts/base/hotspot.sh status) as fabiocar@10.42.50.x.
nohup sh -c 'sleep 2; nmcli connection up id f1tenth-base' >/tmp/hotspot_up.log 2>&1 &
```

### 5.2 Permission for link_manager's `nmcli connection up`

Over ssh or from a service, `network-control` and `wifi.scan` are `auth`
today, so every reconnect attempt would fail with "Not authorized".

```bash
scripts/orin/comms_network.sh polkit        # sudo: a .pkla on 22.04 (polkit 0.105)
nmcli general permissions | grep -E 'network-control|wifi.scan'    # both must say yes
```

It allows exactly those two actions, for that one user.

### 5.3 Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_messages f1tenth_comms \
  f1tenth_params f1tenth_bringup f1tenth_gateway
colcon test --packages-select f1tenth_comms && colcon test-result --verbose    # 72 tests
```

The tests take a private ROS domain and clear `ROS_DISCOVERY_SERVER`, so they
never join the running stack's graph. `f1tenth_messages` only gained a message;
nothing that uses the existing ones needs a rebuild.

### 5.4 Running it

Off by default. With the stack up, start it as a component:

```bash
./scripts/stackctl.py start comms           # stop comms / restart comms likewise
```

or alone, without the stack (needs TF only for the coverage grid):

```bash
ros2 launch f1tenth_comms comms.launch.py comms_reconnect_dry_run:=true
ros2 launch f1tenth_comms comms.launch.py comms_tailscale_host:=100.108.225.104   # the tpad as base
```

To start it with every stack: `enable_comms: true` in
`f1tenth_params/config/stack_params.yaml` (it is not a CLI argument of
`supervisor_bringup.launch.py`, like `enable_wall_distance`).

**First runs: `comms_reconnect_dry_run:=true`.** link_manager then logs the
`nmcli connection up` it would run and never runs it. Note that
`stackctl.py start comms` uses the yaml, where `reconnect_dry_run` is false:
for a dry run through the component, set it in `config/comms.yaml` first.

## 6. Parameters (`src/f1tenth_comms/config/comms.yaml`)

Floats keep their decimal point: Humble refuses an int for a double parameter.

### 6.1 link_manager: the link

| Parameter | Default | Meaning |
|---|---|---|
| `tailscale_host` | `100.89.193.76` | the base's Tailscale IP (linus); a MagicDNS name also works, resolved in a thread |
| `local_host` | `10.42.50.1` | the base on the hotspot |
| `port` | 47800 | the responder's port |
| `period_s` | 0.5 | one heartbeat round (both paths) |
| `timeout_s` | 2.0 | a path is UP while it answered within this; no path UP = **LOST** |
| `degraded_silence_s` | 1.2 | the active path silent this long = DEGRADED; another path that is answering takes over at once |
| `degraded_loss_pct` | 20 | active path loss over `window_s` above this = DEGRADED |
| `degraded_rtt_ms` | 250 | active path mean RTT over `window_s` above this = DEGRADED |
| `window_s` | 10 | loss and mean RTT window |
| `ok_after_s` | 2.0 | back to OK only after this long without any DEGRADED reason |
| `failback_s` | 3.0 | Tailscale must answer this long without a gap before it is the active path again |

### 6.2 link_manager: reconnect (only while LOST)

| Parameter | Default | Meaning |
|---|---|---|
| `reconnect_enabled` / `reconnect_dry_run` | true / false | off; or log the command only |
| `hotspot_connection` | `f1tenth-base` | the NM profile (§5.1); `hotspot_ssid` '' = read from it |
| `retry_period_s` | 2.5 | time trigger (0 = off) |
| `retry_backoff_max_s` | 0 | > 0: the period doubles per attempt of the same outage, up to this |
| `retry_distance_m` | 3.0 | every 3 m travelled (odom), 0 = off |
| `retry_on_new_cell` | true | entering a coverage cell not mapped yet |
| `retry_min_interval_s` | 2.0 | never two attempts closer than this, whatever fires |
| `nmcli_wait_s` | 15 | `nmcli --wait` of the `connection up` |
| `skip_if_rssi_above_dbm` | −65 | already on the hotspot with a better signal: skip (the base is down, not the radio); 0 = never skip |
| `require_ssid_in_scan` | true | on another network (eduroam): switch only if the hotspot SSID is in NM's scan list (`--rescan auto`) |

One attempt, in a one-thread pool: `nmcli device show` (skip while NM is
connecting, or the device is unavailable/unmanaged), the checks above, then
`nmcli --wait 15 connection up id f1tenth-base ifname wlP1p1s0`. Its result
is in `/comms/link_status.last_reconnect_result` and logged when it changes.

### 6.3 wifi_coverage_node

| Parameter | Default | Meaning |
|---|---|---|
| `resolution_m` | 1.0 | cell size, map frame |
| `sample_period_s` | 1.0 | one sample per second |
| `tf_max_age_s` | 1.0 | older `map -> base_link`: the sample is skipped (counted) |
| `link_status_timeout_s` | 3.0 | older link_status = link unknown (RSSI still recorded) |
| `rssi_good_dbm` / `rssi_bad_dbm` | −40 / −90 | RSSI layer 0 / 100 |
| `rtt_bad_ms` | 200 | RTT layer 100 (p95 per cell) |
| `no_link_fraction` | 0.5 | a cell is drawn as no link from this share of its samples |
| `max_extent_m` | 200 | samples further from the map origin are dropped |
| `tf_polling` | true | TF taken once per sample; false = a normal TransformListener (§9) |

## 7. Topics

| Topic | Type | Rate |
|---|---|---|
| `/comms/link_status` | `f1tenth_messages/LinkStatus` | every `period_s` (2 Hz) |
| `/comms/coverage_rssi`, `/comms/coverage_rtt` | `nav_msgs/OccupancyGrid`, latched, frame `map` | ≤ 1 Hz, only when changed |
| `/comms/coverage_cells` | `std_msgs/String` (JSON), latched | with the grids, or every 5 s when only numbers changed |
| `/comms/coverage_reset` | `std_srvs/Trigger` | |

**Grid values (both layers, int8, higher is worse):**

| Value | RSSI layer | RTT layer |
|---|---|---|
| −1 | never visited | no RTT sample in the cell |
| 0..100 | mean RSSI: 0 at ≥ −40 dBm, 100 at ≤ −90 dBm, linear | p95 RTT: 0 at 0 ms, 100 at ≥ 200 ms, linear |
| −128 | **no link**: ≥ 50 % of the cell's samples had no link | same |

A sample has no link when the Wi-Fi is not associated, or when a fresh
`/comms/link_status` says LOST. A cell's real numbers (mean/min RSSI, p95
RTT, loss %, samples, no-link samples, first/last update) are in
`/comms/coverage_cells`. The grids are drawn 5 cm (RSSI) and 10 cm (RTT)
above the map. Session only: the grid starts empty with the node and
`ros2 service call /comms/coverage_reset std_srvs/srv/Trigger` empties it.

The gateway's `/api/status` (and the WebSocket status) now has `link`:
`{state, path, address, rtt_ms, loss_pct, rssi_dbm, since_reply_s,
reconnect_attempts, reconnect_result}`, or `null`.

## 8. Foxglove

`output/comms/foxglove_comms_layout.json`: a 3D panel (map frame, `/slam/map`
plus the RSSI layer; the RTT layer is in the list, hidden), a state
transitions panel (`state_name`, `path`, `wifi_associated`), a plot (RTT,
RSSI, loss) and the raw `link_status`. Import it with *Layouts → Import from
file*. The four `/comms` topics are on the bridge's whitelist.

**Not tried in Foxglove.** If an imported setting does not take, set it in
the 3D panel's sidebar, *Topics → /comms/coverage_rssi*:

- Color mode **Custom**
- Min color (0 = good) green `#1a9850`, max color (100 = bad) red `#d73027`
- Unknown color (−1) fully transparent, so the map shows through
- Invalid color (−128 = **no link**) purple `#7b00a8`

The same for `/comms/coverage_rtt`; show one layer at a time.

## 9. CPU (synthetic run on linus)

`tools/comms_synthetic_run.py` runs both nodes as processes against fake
TF, Wi-Fi, base and `nmcli`, through Tailscale silent, both silent (LOST,
reconnect attempts), 35 % loss, recovery and a reset (§10). CPU from
`/proc/<pid>/stat`, **% of one linus core**, Humble image:

| /tf rate | wifi_coverage_node | link_manager (link OK) | link_manager (whole run, with LOST) |
|---|---|---|---|
| 100 Hz | 1.06 % | 0.23 % | 0.39 % |
| 20 Hz | 0.41 % | 0.23 % | 0.27 % |

RSS ~59 MB each (Python + rclpy). The car's `/tf` had 19–105 messages/s in
the bags of 2026-10-16/19.

**Why TF is polled.** rclpy spends ~0.35 ms of a linus core per received
message whatever the callback does: a TransformListener, an empty /tf
callback and a raw one all cost 3.5–4 % at 100 Hz (Jazzy and Humble). With
a TransformListener the coverage node measured 5.5 % at 100 Hz. `tf_poll.py`
keeps `/tf` and `/tf_static` on a side node no executor spins
(`<node>_tf_poll`) and drains them once per sample into a tf2 Buffer: 0.7–0.9 %
for the drain alone. link_manager needs a pose only while LOST, so it has no
TF subscription at all otherwise.

**Not measured on the Orin.** Its cores are slower than linus's; expect a
multiple of these numbers, still a small share of 8 cores. Measure in §10
step 7.

## 10. Test on the car, in this order

Wheels up or stack not driving for steps 1–4.

1. **Base.** `check_ap.sh`, `hotspot.sh up`, `hotspot.sh status`, firewall
   (§4.3), `install_heartbeat.sh`, the self-check in §4.5 (both addresses ok).
2. **Orin profile.** `comms_network.sh check`, `install`, `polkit`; switch to
   the hotspot (§5.1); `ping 10.42.50.1`, `ping 1.1.1.1` (internet through the
   base), `tailscale status` (car ↔ base direct or via DERP).
3. **link_manager alone, dry run.** `ros2 launch f1tenth_comms comms.launch.py
   enable_wifi_coverage:=false comms_reconnect_dry_run:=true` in the car's ROS
   environment (`set -a; . src/f1tenth_ctl/config/ctl.env; set +a`), and
   `ros2 topic echo /comms/link_status --no-daemon`:
   - OK via `tailscale`; on the base `journalctl --user -u f1tenth-heartbeat`
     shows the car's address on both paths;
   - `sudo tailscale down` on the base: `path` goes to `local` within ~1.2 s,
     state stays OK; `tailscale up`: back to `tailscale` 3 s later;
   - `hotspot.sh down`: LOST after 2 s, and the log shows `dry-run: would run …`
     or a `skipped: …` reason every 2–3 s, until NM has put the car on eduroam
     and Tailscale answers again (then OK via tailscale); `hotspot.sh up`.
4. **Coverage, standing still.** Stack up (SLAM on), `stackctl.py start comms`;
   in Foxglove (§8) one cell under the car, `/comms/coverage_cells` with
   samples and RSSI ≈ `cat /proc/net/wireless`.
5. **Coverage, moving.** Push or teleop the car through the rooms and the
   corridor at walking speed. Look for: no-link cells (purple) where the
   signal goes; the RTT layer; `counts.tf_missing/tf_stale` in the cells JSON
   stay ~0. Reset with `/comms/coverage_reset` and repeat.
6. **Reconnect for real** (dry run off, `reconnect_dry_run: false`):
   - walk out of hotspot range until LOST and NM falls back to eduroam;
     come back: the next trigger (time, 3 m, new cell) must find
     `f1tenth-base` in the scan and switch back; watch
     `last_reconnect_result` and `nmcli device`;
   - hotspot down while the car stays: NM moves the car to eduroam. If the
     base answers over Tailscale there, the link is OK and nothing runs; if
     not (base offline too), the attempts are `skipped: … not in scan`,
     every 2.5 s, and eduroam is not disturbed;
   - stop the responder only (`systemctl --user stop f1tenth-heartbeat`):
     on the hotspot with a good signal the attempts are `skipped: on
     f1tenth-base at … dBm`: the car is not re-associated in a loop.
7. **CPU on the Orin.** `pidstat -p $(pgrep -d, -f 'wifi_coverage_node|link_manager') 10 6`
   with the stack running.
8. **Gateway.** With the PWA open, `curl -s -H "Authorization: Bearer $TOKEN"
   https://jetson-orin.tail5ddb73.ts.net/api/status | jq .link`.
9. **Watchdog unchanged.** An app mission with the phone's Wi-Fi switched off:
   it still aborts after 2 s, as before; link_manager plays no part.

Only after that: `enable_comms: true`, if it should start with every stack.

## 11. Synthetic run (linus, Humble)

```bash
tools/test_comms_humble.sh /tmp/x -- python3 /work/src/tools/comms_synthetic_run.py --tf-hz 100 --out /work/s.json
```

12/12 checks passed (`synthetic_run_humble_100hz.json`):

| t [s] | What the fakes do | link_status |
|---|---|---|
| 0–20 | both paths answer | DEGRADED (proving, then the 2 s dwell), OK via tailscale at 3.7 s |
| 20–35 | Tailscale silent | OK via local at 21.2 s (1.2 s after the last reply), no DEGRADED |
| 35–50 | both silent; car parked 40–46 s | DEGRADED 36.2, LOST 37.2; 5 `connection up` calls, 2.0–3.0 s apart, triggers `new_cell` (moving) and `time` (parked) |
| 50–65 | 35 % of replies dropped on both paths | DEGRADED throughout; the active path switched 6 times, at least 1 s apart |
| 65– | both answer | back on tailscale at 68.2 s (3 s fresh); OK at 72.2 s, once its loss over the 10 s window had been under 20 % for 2 s; no flicker; no attempt after LOST ended |
| 80 | `/comms/coverage_reset` | "cleared 25 cells", 1×1 unknown grid, then 8 new cells |

Grids before the reset: 11×4 cells, 13 with a level, 12 no-link (7 of them
beyond x = 8 m where the fake Wi-Fi was disassociated, the rest on the LOST
stretch), 19 never visited; the RTT layer has the same no-link cells; grids
never closer than 0.997 s.

## 12. Open questions

- **Thresholds.** LOST after 2 s, DEGRADED at 1.2 s silence / 20 % loss / 250
  ms mean RTT, OK after 2 s clean. LOST at 2 s matches the gateway's own
  2 s auto-abort, but they are independent. Wanted tighter or looser?
- **RSSI scale.** −40 → 0, −90 → 100 is a guess; set it from the first
  coverage walk (the cells JSON has the raw dBm).
- **Retry parameters.** 2.5 s / 3 m / new cell / ≥ 2 s apart, `nmcli
  --wait 15`, skip on the hotspot above −65 dBm, backoff off (you asked for
  every 2–3 s). Turn the backoff on (`retry_backoff_max_s: 20`) if long
  outages make the logs or NM noisy.
- **Which base.** One `tailscale_host` at a time (linus by default).
  Switching to the tpad: `comms_tailscale_host:=100.108.225.104`, or the yaml.
- **linus as a base** needs a second radio or the travel router (§3). Which?
- **eduroam with the hotspot in range but Tailscale fine.** No switch today
  (reconnect runs only while LOST). Should the car prefer the hotspot then?
- **`enable_comms` from the command line** (like `enable_mission_logger`)
  instead of only the yaml: needs a supervisor change; not done.
- **PWA.** `link` is in the gateway status; showing it is the next step.
