# HIL setup: Isaac Sim on the workstation, the car stack on the Jetson Orin

Branch `hil-isaac`. The stack on the Orin is the **car's own stack**, unchanged: `sim:=isaac`
swaps the device drivers for the sim's topics and puts every node on `/clock`, and nothing else
differs. Everything machine-specific is in **one file**, `sim/isaac/config/hil.yaml`.

Read in order. Steps that need `sudo` or an install are collected at the end (steps 8-10) for
you to run; nothing above them touches the system.

```
workstation (RTX 5050)                        Jetson Orin
+--------------------------------+            +---------------------------------+
| Isaac Sim 5.1 container        |  Ethernet  | Jazzy container, host network   |
| sim/isaac, host network        |<==========>| the car's stack, sim:=isaac     |
| /scan /odom /imu /camera /depth|  10.42.60  | Fast DDS Discovery Server       |
| <- /ackermann_drive            |   .1/.2    | use_sim_time:=true              |
+--------------------------------+            +---------------------------------+
```

---

## 1. Get the repository (workstation)

```bash
git clone <this repo> ~/dev_ws/f1tenth_hil && cd ~/dev_ws/f1tenth_hil
git checkout hil-isaac
git submodule update --init src/husarion_gz_worlds      # the office meshes (60 MB, Apache-2.0)
```

Nothing outside the clone is needed except the Isaac image (step 3) and the Gazebo Fuel mesh
cache (step 4). **If the workstation is linus itself**, both are already there: the Isaac image is
`isaac_ws_devcontainer-isaac-sim:latest` (step 3 shows the one-line retag) and `~/.gz/fuel` holds
every Fuel mesh the office needs (checked: 20 of 20).

## 2. Fill in the one config file

```bash
cp sim/isaac/config/hil.yaml sim/isaac/config/hil.local.yaml   # gitignored; the scripts prefer it
$EDITOR sim/isaac/config/hil.local.yaml
```

What matters:

| key | default | what to check |
|---|---|---|
| `link.workstation_ip` / `link.orin_ip` | `10.42.60.1` / `10.42.60.2` | a free /24. `10.42.0.x` is the linus–Thor cable and `10.42.50.x` the car's hotspot, so leave those alone |
| `ros.domain_id` | `44` | free in this repository (0 = the car, 42 = the Gazebo/Isaac two-machine sim, 77/87/91 = test harnesses) |
| `ros.discovery_server_port` | `11812` | **not** 11811: the car's production server may be running on the Orin |
| `ros.discovery_server_host` | `orin` | keeps the server on the stack host, exactly as on the car, so the sim adds no process to the Orin |
| `sim.world` / `sim.layout` | `office` / `corridor` | `office`, `room6x6`, `corridor` (+ the NVIDIA fallbacks) |
| `sim.stereo.rate` | `10.0` | the link budget (see step 7) |
| `orin.ssh`, `orin.repo`, `orin.container` | `fabiocar@10.42.60.2`, … | the Orin's user, a checkout of this branch there, and the container name |

Check what the scripts read back:

```bash
scripts/sim/sim_isaac_hil env
```

## 3. The Isaac image (workstation)

```bash
docker tag isaac_ws_devcontainer-isaac-sim:latest f1tenth_isaac:5.1.0   # if that image exists
# otherwise (needs an NGC login, ~20 GB):
docker login nvcr.io          # user '$oauthtoken', password = your NGC API key
sim/isaac/docker/build.sh
```

## 4. Generate the office world (workstation, once)

The office is the **same Gazebo world the stack's Gazebo sim runs**
(`src/f1tenth_sim/worlds/husarion_office_2x.sdf`). Isaac Sim 5.1 has no SDF importer, so
`sim/isaac/tools/sdf_to_usd.py` converts the meshes with Isaac's asset converter and authors the
world structure from the SDF. Check first, then convert:

```bash
scripts/sim/sim_isaac_hil world --plan        # no Isaac needed; prints meshes, colliders, bbox
scripts/sim/sim_isaac_hil world --convert     # inside the Isaac container, writes usd/generated/
```

`--plan` must end with `"missing": 0` and `"structure_without_collision": []`. If Fuel meshes are
missing, either run it where Gazebo has already cached them or add `--download` (it fetches them
over https into `~/.gz/fuel`). The generated stage is **not** committed; regenerate it after any
change to the SDF so Gazebo and Isaac never drift apart.

`--convert` also prints `bounding box: SDF ... vs stage ... (max difference X m)`: that is the
check that the conversion kept the metric scale. It should be centimetres, not metres.

The other two worlds need nothing: `room6x6` (the real test room, 1:1 with `test_room_6x6.sdf`)
and `corridor` (20 x 2.2 m, glossy floor, optional glass panel).

## 5. Bring the link up (both machines, step 8 has the sudo lines)

Then, on the workstation:

```bash
scripts/sim/sim_isaac_hil preflight --orin
```

It checks: the config, the world file and every scene/layout it uses, the generated office USD,
docker and the nvidia runtime, the image, the GPU, the route to the Orin (and that it does **not**
go through a gateway), ping, the camera budget against the link speed, that the stack's launch
arguments exist in this checkout, and - with `--orin` - the Orin's own `hil_orin env`, failing
loudly if the domain does not match.

## 6. Start the sim (workstation) and the stack (Orin)

```bash
# workstation
scripts/sim/sim_isaac_hil sim                      # add --gui to watch, --world room6x6 to switch
# Orin (host shell, in the checkout named by orin.repo)
scripts/sim/hil_orin preflight
scripts/sim/hil_orin up                            # starts the Jazzy container (host network)
scripts/sim/hil_orin stack                         # the car's stack, sim:=isaac, use_sim_time
```

Start the sim first: Isaac needs 20-70 s to the first `/clock` (shader cache), and the stack waits
for it. `hil_orin stack` runs exactly:

```
ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac use_sim_time:=true \
  discovery_server_address:=<orin ip> discovery_server_port:=11812 \
  fastdds_profile:=/tmp/f1tenth_hil_profile.xml
```

The container must be on the **host network**: the Orin's kernel cannot load `bridge.ko`, so a
container started without `--network host` silently comes up with only `lo` and no DNS
(CLAUDE.md, "Orin Docker"). `hil_orin` refuses to use a container that is not.

**The container tooling itself lives on `main-jazzy-rust`** (`scripts/docker_orin.sh`,
`docker/orin/compose.yaml`, `~/.local/bin/stack_mode`), not on this branch. `hil_orin up` uses
`scripts/docker_orin.sh` when it is present and otherwise prints the explicit `docker run` for
you. Three ways to close that gap, cheapest first:

1. **Nothing** -- run the `docker run` that `hil_orin up` prints (host network, host IPC,
   `F1T_AUTOSTART=none`), then `hil_orin stack`. It needs an image pinned in
   `~/.config/f1tenth/car_image` or `orin.image` in `hil.yaml`. It skips docker_orin.sh's
   preconditions (group ids, free space, image drift), which a HIL session does not need.
2. **Two checkouts on the Orin** (recommended): `scripts/docker_orin.sh up` from the
   `main-jazzy-rust` checkout -- the one the running image was built from -- and `hil_orin stack`
   / `check` from this branch's checkout. `hil_orin` finds the container by name, so the two
   never have to be the same tree. Build and run the second checkout from a clean shell
   (CLAUDE.md, "Thor: a second checkout needs a clean shell": the Orin's profile sources the
   other checkout's install).
3. **Merge the container tooling.** Not a cherry-pick of one file: `docker/orin/` differs by 27
   files (compose.yaml, compose.harness.yaml, the host scripts and their tests) and
   `docker_orin.sh` checks the native tree against the image's `CAR_SYNC.json`, so it refuses
   unless the tree it runs from is the one the image was built from. This is the main-jazzy /
   main-jazzy-rust promotion, not a HIL step.

## 7. Check it (Orin)

```bash
scripts/sim/hil_orin check            # copies scripts/sim/hil_check in and runs it there
```

It reports, and fails on: `/clock` present and advancing with the real-time factor; every sensor
topic published; every rate within 10 % of what it should be (the real car's value is printed
next to it); the age of each message on arrival (the link's one-way delay, in sim time); the TF
tree `odom -> base_link -> laser / imu / the ZED chain`; then it sends one ackermann command and
confirms the car moved in Isaac; and finally the control loop's own rate and latency.

It publishes on `/ackermann_drive`, the mux **output**, which bypasses the stack's arbitration on
purpose - it tests the link, not the mux. Pass `--no-drive` if the stack is driving a mission.

Bandwidth, for when it does not keep up: the two images are 0.92 MB each and Fast DDS sends one
unicast copy per remote subscriber (2 readers for the RGB image, 3 for depth), so 640x360 at
15 Hz is ~570 Mbit/s and at 10 Hz ~370 Mbit/s (`output/zenoh/recon.md` 2.1-2.2). Every knob is on
the **sim** side (`sim.stereo` in the config, or `--rate` / `--set`), never in the stack:

```bash
scripts/sim/sim_isaac_hil sim --rate 6             # ~220 Mbit/s
# or, in hil.local.yaml: sim.stereo.width/height, sim.stereo.right_camera: false
# last resort, documented in the config: link.async_writer: true (shapes the bursts, costs latency)
```

## 8. Needs sudo: the link (both machines)

Static addresses on the direct cable (replace the interface names; `ip -o link` lists them):

```bash
# workstation
sudo ip addr add 10.42.60.1/24 dev <iface>
sudo ip link set <iface> up
# Orin
sudo ip addr add 10.42.60.2/24 dev <iface>
sudo ip link set <iface> up
```

Permanent versions: NetworkManager (`nmcli con add type ethernet ifname <iface> ip4 10.42.60.1/24`)
or netplan. On the Orin, `nmcli` over ssh needs polkit (noted in the comms work: network control
over ssh is an "auth" action there), so do it on the console or with a netplan file.

## 9. Needs sudo: socket buffers (both machines)

A 0.92 MB image is fragmented into ~15 UDP datagrams that must all arrive. The Fast DDS profile
asks for 16 MB socket buffers, and the kernel silently caps them:

```bash
sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216
# permanent:
echo -e 'net.core.rmem_max=16777216\nnet.core.wmem_max=16777216' | sudo tee /etc/sysctl.d/60-f1tenth-hil.conf
```

`hil_orin preflight` checks `rmem_max` and says exactly this if it is too small.

## 10. Optional, needs sudo (workstation)

```bash
sudo cpupower frequency-set -g performance      # Isaac logs "CPU performance profile is powersave"
sudo chown -R "$USER:$USER" ~/.cache/isaac-sim  # if an old root container created it
```

---

## The worlds, and what they are for

| world | layouts | geometry |
|---|---|---|
| `office` | `corridor`, `door`, `open_space`, `people` | the husarion office, 28.5 x 27.1 m, 148 collision geometries, 210 k triangles, 573 m2 of drivable floor |
| `room6x6` | `default`, `glossy`, `plant`, `obstacle` | the real test room: 6.00 x 6.00 m inner, walls 2.5 m, person in the middle, car at (0.5, 3.0) facing the wall 5.5 m ahead |
| `corridor` | `glossy`, `matte`, `glass`, `person` | 20 x 2.2 m, lights every 4 m, glossy floor, 3 m glass panel in the left wall at x 8..11 |

Office spawn poses, **computed from the SDF** by `sim/isaac/tools/pick_spawn_poses.py` (it
projects every collision geometry onto the lidar plane, per triangle, and reads the poses off that
grid) - re-run it after regenerating the world:

| layout | x | y | yaw | what it gives |
|---|---|---|---|---|
| `corridor` | 16.814 | 0.946 | -90 deg | 24.85 m clear ahead, 1.25 m to each side |
| `door` | 10.064 | -19.404 | 180 deg | 2.0 m in front of a **1.25 m doorway** at (8.064, -19.404), 6 m free beyond it, 0.65 m minimum clearance on the way in |
| `open_space` | 15.364 | -11.554 | 0 deg | the largest free circle on the floor, 2.80 m |

The world's one `Door` model is a **closed leaf** with a flight of stairs 0.6 m in front of it, so
it is a wall, not a doorway; the `door` layout uses a real opening in the wall, which is what the
door detector looks for anyway. Ten such openings were found; the other nine are in the JSON the
tool prints.

Walking people (`--layout people`, off otherwise) are **ours**: the SDF has no actors at all.
They follow checked waypoints (minimum clearance 0.80 / 0.80 / 1.66 m) as kinematic bodies with
the person capsule collider, and publish their live pose in `/sim/ground_truth/objects`. There is
no walk cycle - the mesh glides. Real walking needs `omni.anim.people` with a navmesh and a
command file, driven from these same waypoints.

## Licence

The office world and its meshes come from
[husarion_gz_worlds](https://github.com/husarion/husarion_gz_worlds), **Apache License 2.0**
(`src/husarion_gz_worlds/LICENSE`), pinned at commit `87271ef` (0.1.0). The furniture meshes it
references by URL are Gazebo Fuel models by Open Robotics, fetched into `~/.gz/fuel`. Neither is
modified: the SDF is read, the meshes are converted into a separate output directory, and the
originals stay as they are.

## What is not done

1. **`/scan` has no `intensities[]` in the sim**, so `glass_detector_node` cannot be exercised -
   the corridor's glass world is only half usable until the sim fills that field.
2. The office USD has never been generated or opened: `--convert` needs Isaac, and the machine
   this was written on has none. The static plan passes; step 4 is the first real test.
3. `expected.wall_ahead_range_from_base` is 0 for the office layouts - fill it in per layout from
   `sim/isaac/tools/find_pose.py` after the first run.
4. The Orin container has never run `sim:=isaac`: step 7 is the command that will say.
