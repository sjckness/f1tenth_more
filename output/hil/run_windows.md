# Running the HIL: Isaac Sim native on Windows + the car stack on the Jetson Orin

How to launch the whole thing, in the order that works. This is the Windows variant of
[setup.md](setup.md) (which assumes a Linux workstation and Docker): Isaac Sim 5.1 runs
**natively** from a pip environment, because Isaac's RTX renderer does not see the GPU inside
Docker Desktop / WSL2 ("no nvidia device nodes", "CUDA GPU not found").

Everything machine-specific is in `sim/isaac/config/hil.local.yaml` (git-ignored copy of `hil.yaml`).

## Layout

```
Windows PC (RTX 4090)                           Jetson Orin (jetson-orin)
Isaac Sim 5.1, native (F:\envs\isaac)           container f1tenth_hil_car (host network)
  Ethernet 2  192.168.200.2  <== cable ==>        eno1  192.168.200.11
  domain 44, Fast DDS unicast                     Discovery Server 192.168.200.11:11812
  Foxglove Studio  --ws://192.168.200.11:8765-->  foxglove_bridge (started by the stack)
```

| What | Value |
|---|---|
| ROS domain / RMW | 44 / `rmw_fastrtps_cpp` |
| Bridge distro on the PC | `jazzy` (Isaac's internal rclpy, Python 3.11) |
| Link | 192.168.200.0/24, the same cable and NIC as the Thor (`.10`); the Orin is `.11` so both can sit on one switch |
| Orin repo / container | `/home/fabiocar/dev_ws/f1tenth_hil` / `f1tenth_hil_car` (never `f1tenth_car`, the production one) |
| Image | `f1tenth:jazzy-orin-ecb457b-norust` (set in `orin.image`) |

## Daily launch

Order matters: the sim first (Isaac needs 20-70 s for the first `/clock`), then the stack.

**1. PC** (cmd, no administrator)

```bat
cd /d "F:\Users\Andreas Steffens\dev_ws\f1tenth_hil\scripts\sim\win"
hil_sim_link.bat --gui
```

Output goes to `run.log` (`type run.log`). Healthy: `RTF 1.000` lines every 10 s, no `XMLPARSER`.
Without `--gui` it runs headless. No Orin needed? Use `hil_sim_local.bat` (loopback only).

**2. Orin** (`ssh fabiocar@192.168.200.11`)

```bash
cd ~/dev_ws/f1tenth_hil
docker ps --format '{{.Names}} {{.Status}}'   # f1tenth_hil_car must be Up (else: docker start f1tenth_hil_car)
scripts/sim/hil_orin stack                    # leave this terminal open
```

**3. Orin, second terminal: check**

```bash
scripts/sim/hil_orin check --no-drive         # /clock, sensor topics, rates, TF; publishes no drive command
```

**4. PC: Foxglove**

Foxglove Studio (desktop app; the browser version refuses plain `ws://`) -> *Open connection* ->
*Foxglove WebSocket* -> `ws://192.168.200.11:8765`.

**Stop**: Ctrl+C on the stack, then close Isaac or Ctrl+C on the PC. Do not just close the
terminals: processes are left running inside the container (`docker restart f1tenth_hil_car` cleans up).

## LLM planner (optional)

llama.cpp is already **in the image** (`/opt/llama.cpp`, built for sm_87; the installed `stack_params.yaml`
points at it, see `docker/orin/set_llama_paths.py`). It is not on `PATH`, so `which llama-server`
finds nothing: that does not mean it is missing. Only the `.gguf` models are not in the image, so the
container gets the models directory read-only, at the same path as on the host (the planner's
`model_path` is `~/projects/llm/models/...` and the container user is the host user). Set it once in
`hil.local.yaml` (`orin.models_dir: /home/fabiocar/projects/llm/models`); `scripts/sim/hil_orin up`
then prints the `docker run` with the mount. Mounts can only be set when the container is created:

```bash
docker rm -f f1tenth_hil_car
docker run -d --name f1tenth_hil_car --network host --ipc host --runtime nvidia \
  --init --restart no -v /home/fabiocar/projects/llm/models:/home/fabiocar/projects/llm/models:ro \
  -e F1T_CONTAINER_ROLE=car -e F1T_AUTOSTART=none \
  -e ROS_DOMAIN_ID=44 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e F1TENTH_DISCOVERY_SERVER=192.168.200.11:11812 \
  f1tenth:jazzy-orin-ecb457b-norust sleep infinity
docker exec f1tenth_hil_car /opt/llama.cpp/bin/llama-server --version                      # sanity check
docker exec f1tenth_hil_car ls /home/fabiocar/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf
```

(Do not mount the host's `/scratch/fabiocar/llama.cpp`: that binary has no sm_87 cubins and
JIT-compiles every kernel at first use, about two minutes of cold start.)

Then start the stack with the component on, and send a command from a second shell:

```bash
scripts/sim/hil_orin stack enable_intelligence:=true

scripts/sim/hil_orin shell                    # inside the container:
curl -s http://127.0.0.1:8083/health          # 200 = ready, 503 = model still loading (about a minute)
ros2 run llm llm_planner_node --dry-run "go ahead 2 meters"   # translate + write the mission, no motion
ros2 run llm llm_planner_node "go ahead 2 meters"             # the car moves in the sim: watch Foxglove
```

`scripts/sim/llm.sh` is the Thor's wrapper (it goes through `sim_isaac.sh`); it is not used here.

## One-time setup

**PC**
- Isaac environment `F:\envs\isaac` (pip `isaacsim[all,extscache]==5.1.0`, torch `2.7.0+cu126`).
  The launcher scripts take it from `ISAAC_ENV` (default `F:\envs\isaac`).
- `sim/isaac/config/hil.local.yaml`: `link.workstation_ip: 192.168.200.2`, `link.orin_ip: 192.168.200.11`,
  `orin.ssh: fabiocar@192.168.200.11`, `orin.repo`, `orin.container: f1tenth_hil_car`,
  `orin.image: f1tenth:jazzy-orin-ecb457b-norust`. Copy the same file to the Orin checkout
  (`scp ... fabiocar@192.168.200.11:~/dev_ws/f1tenth_hil/sim/isaac/config/`).
- Office world, once: `hil_world_convert.bat --download` (downloads 20 Fuel meshes, uses the GPU).
  Other worlds (`room6x6`, `corridor`) need no conversion: `hil_sim_link.bat --world room6x6`.
- Firewall (administrator, once): inbound UDP between the PC and the Orin. The existing
  `HIL Thor in ingresso (UDP)` rule is per address pair (`.2` <-> `.10`); the Orin needs its own:
  `netsh advfirewall firewall add rule name="HIL Orin in ingresso (UDP)" dir=in action=allow protocol=UDP localip=192.168.200.2 remoteip=192.168.200.11 profile=any`.
  Remove with `... delete rule name="HIL Orin in ingresso (UDP)"`. Firewall rules filter by program,
  protocol, address and port, never by MAC.
  Hydra in WSL2 needs its own rule, because the WSL network (172.26.x here) matches none of the
  pairs above and `vEthernet (WSL)` has no network category, so it falls under Public where inbound
  is blocked. Measured: Isaac binds `172.26.128.1:18400/18410/18411` and WSL still sees nothing,
  while UDP to port 53 on the same address works (that is how the distro resolves DNS). The rule is
  in [hydra_wsl_status.md](hydra_wsl_status.md); do **not** scope it with `program=`, since the
  process that owns the sockets is the uv interpreter the venv is built on, not
  `F:\envs\isaac\Scripts\python.exe`.
- SSH key without password: `ssh-keygen -t ed25519`, then append the `.pub` to the Orin's `~/.ssh/authorized_keys`.

**Orin**
- `git clone --branch hil-hydra-integration https://github.com/sjckness/f1tenth_more ~/dev_ws/f1tenth_hil`
  and `git submodule update --init src/husarion_gz_worlds`. A second checkout, so the production one is untouched.
- Socket buffers (until reboot; run it on its own line, a pasted `sudo` swallows the lines after it):
  `sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216`.
- Create the container once (see the `docker run` above, with or without the two `-v` lines).
  `hil_orin up` only prints this command on this branch; it does not run it.
- Address on the cable (not persistent): `sudo ip addr add 192.168.200.11/24 dev eno1`.

## What the `.bat` files do (`scripts/sim/win/`)

| File | Use |
|---|---|
| `hil_preflight.bat [--orin]` | read-only checks: config, Isaac, bridge, GPU, scene, USD, link |
| `hil_world_plan.bat` / `hil_world_convert.bat` | office SDF -> report / -> USD (once) |
| `hil_sim_local.bat` / `hil_sim_gui.bat` | the sim alone on loopback, headless / with window. No Orin |
| `hil_sim_link.bat [--gui]` | the sim on the HIL link, Discovery Server on the Orin |
| `hil_sim_wsl.bat [--gui]` | the sim for **Hydra in the WSL2 distro on this PC**, GT semantics on, no Orin. Needs one firewall rule: [hydra_wsl_status.md](hydra_wsl_status.md) |
| `hil_topics.bat`, `hil_hz.bat TOPIC [S]`, `hil_reset.bat` | look at the ROS graph / call `/sim/reset` (bridge rclpy) |
| `hil_orin_check.bat` | ping, port 22, ssh to the Orin (read-only) |
| `hil_link_setup.bat`, `hil_firewall.bat` | administrator: address on the link / firewall rule |

The logic is in `hil_win.py` (it mirrors `sim_isaac_hil`; keep the `--set` list in sync with it).
Options of `sim`: `--world --layout --gui --keyboard --rate HZ --duration S --set K=V --dry-run`.

## Things that went wrong (and the fix)

| Symptom | Cause | Fix |
|---|---|---|
| Isaac in Docker on Windows: no GPU, no render | Docker Desktop/WSL2 exposes `/dev/dxg`, not `/dev/nvidia0` | run Isaac natively (this document) |
| Office world: only a huge water dispenser visible | `asset_converter` writes `metersPerUnit=0.01`; DAE meshes come out in cm, STL/OBJ stay raw | fixed in `sim/isaac/tools/sdf_to_usd.py` (DAE scale x `metersPerUnit`); bbox now matches the SDF to 1e-6 m |
| `XMLPARSER Error ... flow_controller_descriptors` on both sides, RTF fell to 0.44 | that element is invalid in Jazzy's Fast DDS, so the **whole** HIL profile was ignored | removed from `src/f1tenth_bringup/config/sim/dds/hil.xml.in` (`link.async_writer` stays off); apply on the Orin checkout too |
| `exec: ros2: not found` from `hil_orin stack` | the image sets ROS up in `/etc/bash.bashrc`, which `bash -lc` does not read | `scripts/sim/hil_orin` now sources `/opt/f1tenth_ws/scripts/env/jazzy.sh` first |
| `hil_orin up` picks the image "tag" | it reads the first word of `car_image`, which starts with `tag` | set `orin.image` explicitly |
| `hil_orin preflight`: "the workstation does not answer ping" | Windows blocks inbound ICMP on a Public network | harmless: `up/stack/check` do not depend on it |
| IP assigned to the wrong adapter | `Ethernet` was an "AMI Remote NDIS" device (the BMC), not a port | the HIL link is **Ethernet 2** (Intel X710); check `Get-NetAdapter` before `netsh` |
| `which llama-server` finds nothing in the container | it is not on `PATH`; the binary is `/opt/llama.cpp/bin/llama-server` (in the image). What is missing is the models directory | mount `orin.models_dir` (see "LLM planner") |
| Python `python3` not found in Git Bash | the Microsoft Store alias shadows it | use the `.bat` files (they activate `F:\envs\isaac`) |

## Not tested in this setup

`hil_topics.bat`, `hil_hz.bat`, `hil_reset.bat`, driving the car (`hil_orin check` without `--no-drive`),
`--keyboard`, and the 10.42.60.x addresses of `hil.yaml` (replaced here by `192.168.200.x`).
