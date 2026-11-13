#!/usr/bin/env python3
"""hil_win -- the WORKSTATION side of the HIL setup on NATIVE WINDOWS (Isaac Sim from pip, no Docker).

The Windows counterpart of scripts/sim/sim_isaac_hil (bash + Docker, Linux only). Run it with the
Python of the Isaac environment (the .bat files next to this file do that):

    hil_win.py preflight [--orin]      checks only: config, Isaac, bridge, GPU, scene, link, USD, Orin
    hil_win.py world --plan            the office SDF -> report (no Isaac needed)
    hil_win.py world --convert         convert the office SDF to USD (starts Isaac headless, GPU)
    hil_win.py sim [--net local|link]  start Isaac Sim (Ctrl+C stops it)
    hil_win.py topics [SECONDS]        list the ROS topics the sim publishes (bridge rclpy)
    hil_win.py hz TOPIC [SECONDS]      message rate of one topic
    hil_win.py reset                   call /sim/reset
    hil_win.py env                     print the environment of a run

    --net local   (default) Fast DDS on loopback only, no Discovery Server: the sim alone, no Orin
    --net link    the HIL link: profile from hil.xml.in bound to link.workstation_ip, Discovery
                  Server on the Orin (ros.discovery_server_*), exactly as sim_isaac_hil does
    --net wsl     Hydra in the WSL2 distro on THIS PC: profile from wsl.xml.in with unicast initial
                  peers between the vEthernet (WSL) address and the distro's, no Discovery Server
                  and no Orin. Both addresses are read from the running distro at every launch,
                  because a WSL restart changes them. See output/hil/hydra_wsl_status.md.
Options of `sim`: --world --layout --gui --keyboard --rate HZ --duration S --set K=V (repeatable) --dry-run.

Every machine-specific value comes from sim/isaac/config/hil.yaml (or hil.local.yaml), read through
scripts/sim/hil_config.py. The sim arguments below mirror isaac_args() in sim_isaac_hil: change one,
change the other. This script changes nothing on the system (no network, firewall or registry).
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import re
import socket
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))
import hil_config  # noqa: E402  (scripts/sim/hil_config.py)

DDS_SRC = REPO / "src" / "f1tenth_bringup" / "config" / "sim" / "dds"
WORK = Path(tempfile.gettempdir()) / "hil_win"
DISTRO = os.environ.get("ROS_DISTRO") or "jazzy"


def log(msg: str) -> None:
    print(f"[hil {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def die(msg: str) -> "None":
    print(f"[hil] ERROR: {msg}", file=sys.stderr)
    raise SystemExit(1)


# ---- configuration -----------------------------------------------------------------------
class Cfg:
    def __init__(self) -> None:
        self.data = hil_config.load()
        g = lambda k, d=None: (hil_config._get(self.data, k) if hil_config._get(self.data, k) is not None else d)  # noqa: E731
        self.get = g
        self.ws_ip = str(g("link.workstation_ip"))
        self.orin_ip = str(g("link.orin_ip"))
        self.domain = str(g("ros.domain_id"))
        self.rmw = str(g("ros.rmw"))
        self.ds = hil_config.derived(self.data)["HIL_DISCOVERY_SERVER"]
        self.isaac_dir = REPO / str(g("sim.isaac_dir"))
        self.world = str(g("sim.world"))
        self.layout = str(g("sim.layout", ""))
        self.gui = bool(g("sim.gui", False))
        st = g("sim.stereo", {}) or {}
        self.stereo = {"rate": st.get("rate", 15.0), "width": st.get("width", 640),
                       "height": st.get("height", 360), "right_camera": str(st.get("right_camera", False)).lower()}
        self.extra_set = [str(x) for x in (g("sim.extra_set", []) or [])]
        self.buf_mb = int(g("link.socket_buffer_mb", 16))
        self.flow_mbit = int(g("link.flow_mbit", 800))
        self.async_writer = bool(g("link.async_writer", False))
        self.orin_ssh = str(g("orin.ssh"))
        self.wsl_distro = str(g("wsl.distro", "Ubuntu-24.04"))
        self.wsl_tcp_port = int(g("wsl.tcp_port", 42100))
        self.camera_mbit = hil_config.derived(self.data)["HIL_CAMERA_MBIT"]


# ---- Isaac environment -------------------------------------------------------------------
def bridge_dir() -> Path | None:
    spec = importlib.util.find_spec("isaacsim")
    if not spec or not spec.submodule_search_locations:
        return None
    p = Path(list(spec.submodule_search_locations)[0]) / "exts" / "isaacsim.ros2.bridge"
    return p if p.is_dir() else None


def isaac_version() -> str:
    try:
        from importlib.metadata import version
        return version("isaacsim")
    except Exception:  # noqa: BLE001
        return "?"


def base_env(cfg: Cfg, profile: Path, discovery: str | None) -> dict[str, str]:
    env = dict(os.environ)
    bd = bridge_dir()
    env.update({
        "OMNI_KIT_ACCEPT_EULA": "YES", "ACCEPT_EULA": "Y", "PRIVACY_CONSENT": "Y",
        "PYTHONUTF8": "1", "PIP_CONFIG_FILE": "nul",
        "ROS_DOMAIN_ID": cfg.domain, "RMW_IMPLEMENTATION": cfg.rmw, "ROS_DISTRO": DISTRO,
        "FASTRTPS_DEFAULT_PROFILES_FILE": str(profile), "ISAAC_WS": str(cfg.isaac_dir),
    })
    env.pop("PYTHONPATH", None)
    for k in ("ROS_LOCALHOST_ONLY", "ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_AUTOMATIC_DISCOVERY_RANGE"):
        env.pop(k, None)
    if discovery:
        env["ROS_DISCOVERY_SERVER"] = discovery
    if bd is not None:
        env["ISAAC_ROS_BRIDGE_DIR"] = str(bd)
        # the bridge DLLs are read at process start: they must be on PATH before Python runs
        env["PATH"] = str(bd / DISTRO / "lib") + os.pathsep + env.get("PATH", "")
    return env


# ---- DDS profile -------------------------------------------------------------------------
def render_profile(cfg: Cfg, link_ip: str) -> Path:
    """hil.xml.in -> a profile bound to link_ip (same substitutions as sim_isaac_hil)."""
    async_xml = ""
    if cfg.async_writer:
        async_xml = ('  <data_writer profile_name="f1tenth_hil_writer" is_default_profile="true">\n    <qos>\n'
                     '      <publishMode><kind>ASYNCHRONOUS</kind><flow_controller_name>hil_images</flow_controller_name>'
                     '</publishMode>\n    </qos>\n  </data_writer>')
    text = (DDS_SRC / "hil.xml.in").read_text(encoding="utf-8")
    for key, value in (("@ETH_IP@", link_ip), ("@SHM_TRANSPORT@", ""),
                       ("@BUFFER_BYTES@", str(cfg.buf_mb * 1024 * 1024)),
                       ("@FLOW_BYTES@", str(cfg.flow_mbit * 1000000 // 8 // 100)),
                       ("@ASYNC_WRITER@", async_xml)):
        text = text.replace(key, value)
    WORK.mkdir(parents=True, exist_ok=True)
    out = WORK / "hil_link.xml"
    out.write_text(text, encoding="utf-8")
    ET.parse(out)      # a malformed profile is silently ignored by Fast DDS: fail here instead
    return out


def wsl_addresses(distro: str) -> tuple[str, str]:
    """(windows_side_ip, distro_ip) of the running WSL distro, read from inside it.

    Inside WSL the default gateway IS the Windows vEthernet (WSL) address, so the distro knows
    both. Read every launch, never cached or configured: a WSL restart changes both.

    No shell and no awk on the WSL side: a quoted shell program handed to wsl.exe comes out
    mangled (an `awk '{print $3}'` lost its program and echoed whole lines), the same nested
    quoting trap CLAUDE.md records for ssh here-docs. Each command is run directly and parsed here.
    """
    def run(*argv: str) -> str:
        try:
            out = subprocess.run(["wsl.exe", "-d", distro, "--", *argv],
                                 capture_output=True, text=True, errors="replace", timeout=60)
        except (OSError, subprocess.TimeoutExpired) as exc:
            die(f"could not ask WSL distro '{distro}' for its addresses: {exc}")
        if out.returncode != 0:
            die(f"'{' '.join(argv)}' failed in WSL distro '{distro}' "
                f"({(out.stderr or '').replace(chr(0), '').strip()!r}). Is it running?  wsl -l -v")
        return (out.stdout or "").replace("\x00", "")

    ip_re = r"\d{1,3}(?:\.\d{1,3}){3}"
    own = re.findall(rf"\b{ip_re}\b", run("hostname", "-I"))
    gw = re.search(rf"\bvia\s+({ip_re})\b", run("ip", "route", "show", "default"))
    if not own:
        die(f"WSL distro '{distro}' reported no address (hostname -I). Is it running?  wsl -l -v")
    if not gw:
        die(f"WSL distro '{distro}' has no default route, so the Windows side of the link is "
            "unknown (ip route show default)")
    return gw.group(1), own[0]


def render_wsl_profile(cfg: Cfg, own_ip: str, peer_ip: str) -> Path:
    """wsl.xml.in -> the Windows side of the profile (the mirror of scripts/hydra/wsl_dds_profile.sh)."""
    text = (DDS_SRC / "wsl.xml.in").read_text(encoding="utf-8")
    for key, value in (("@OWN_IP@", own_ip), ("@PEER_IP@", peer_ip),
                       ("@BUFFER_BYTES@", str(cfg.buf_mb * 1024 * 1024))):
        text = text.replace(key, value)
    WORK.mkdir(parents=True, exist_ok=True)
    out = WORK / "wsl_link.xml"
    out.write_text(text, encoding="utf-8")
    ET.parse(out)      # a malformed profile is silently ignored by Fast DDS: fail here instead
    return out


def render_wsl_tcp_profile(cfg: Cfg, peer_ip: str) -> Path:
    """wsl_tcp.xml.in -> the Windows side, as the TCP CLIENT.

    No listening port and no advertised locator: Isaac only connects out, which is the one
    direction this firewall allows without a rule (see the template's header for the numbers).
    """
    port = cfg.wsl_tcp_port
    peers = ("        <initialPeersList>\n"
             "          <locator><tcpv4>\n"
             f"            <address>{peer_ip}</address>\n"
             f"            <physical_port>{port}</physical_port>\n"
             f"            <port>{port}</port>\n"
             "          </tcpv4></locator>\n"
             "        </initialPeersList>")
    text = (DDS_SRC / "wsl_tcp.xml.in").read_text(encoding="utf-8")
    for key, value in (("@LISTEN_BLOCK@", ""), ("@META_BLOCK@", ""), ("@PEER_BLOCK@", peers),
                       ("@OWN_IP@", ""), ("@TCP_PORT@", str(port)),
                       ("@BUFFER_BYTES@", str(cfg.buf_mb * 1024 * 1024))):
        text = text.replace(key, value)
    WORK.mkdir(parents=True, exist_ok=True)
    out = WORK / "wsl_tcp_link.xml"
    out.write_text(text, encoding="utf-8")
    ET.parse(out)
    return out


def local_ipv4s() -> set[str]:
    try:
        out = subprocess.run(["ipconfig"], capture_output=True, text=True, errors="replace").stdout
    except OSError:
        return set()
    return set(re.findall(r"\b(\d{1,3}(?:\.\d{1,3}){3})\b", out))


def ping(ip: str) -> bool:
    return subprocess.run(["ping", "-n", "1", "-w", "1500", ip], capture_output=True).returncode == 0


def tcp_open(ip: str, port: int, t: float = 2.0) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=t):
            return True
    except OSError:
        return False


# ---- sim arguments (mirror of isaac_args() in scripts/sim/sim_isaac_hil) -----------------
def layout_for(cfg: Cfg, world: str, layout: str | None) -> str:
    explicit = bool(layout)
    layout = layout or cfg.layout
    scene = cfg.isaac_dir / "config" / "scenes" / f"{world}.yaml"
    if layout and not explicit and scene.is_file():
        import yaml
        presets = (yaml.safe_load(scene.read_text(encoding="utf-8")) or {}).get("presets") or {}
        if layout not in presets:
            log(f"world '{world}' has no layout '{layout}' (that is sim.layout of '{cfg.world}'): using its default")
            return ""
    return layout


def sim_args(cfg: Cfg, a: argparse.Namespace) -> list[str]:
    world = a.world or cfg.world
    layout = layout_for(cfg, world, a.layout)
    rate = a.rate if a.rate is not None else cfg.stereo["rate"]
    s = ["--scene", world, "--rtf-log",
         "--set", "ros_interface.publish_static_tf=false",
         "--set", "car_spec.sensors.zed.lens_center_xyz=[0.2293615, 0.0, 0.2196267]",
         "--set", "car_spec.sensors.zed.pitch=-0.03635",
         "--set", "sensors.odometry.covariance.pose_xy=0.2",
         "--set", "sensors.odometry.covariance.pose_yaw=0.03",
         "--set", "sensors.odometry.covariance.twist_vx=0.0",
         "--set", "sensors.imu.noise.gyro_stddev=0.0013217",
         "--set", f"sensors.stereo.rate={rate}",
         "--set", f"sensors.stereo.width={cfg.stereo['width']}",
         "--set", f"sensors.stereo.height={cfg.stereo['height']}",
         "--set", f"sensors.stereo.right_camera={cfg.stereo['right_camera']}"]
    if layout:
        s += ["--set", f"scene.layout={layout}"]
    gui = a.gui or cfg.gui
    if a.keyboard:
        s += ["--set", "ros_interface.keyboard.enabled=true"]
        if not gui:
            log("--keyboard without --gui: carb.input has no window, the keyboard will not work")
    for x in cfg.extra_set + list(a.set):
        s += ["--set", x]
    if gui:
        s += ["--gui"]
    if a.duration:
        s += ["--duration", str(a.duration)]
    return s


# ---- subcommands -------------------------------------------------------------------------
def pick_profile(cfg: Cfg, net: str) -> tuple[Path, str | None, str]:
    if net == "local":
        return DDS_SRC / "isaac_local.xml", None, "127.0.0.1"
    if net in ("wsl", "wsltcp"):
        own, peer = wsl_addresses(cfg.wsl_distro)
        log(f"WSL distro '{cfg.wsl_distro}': windows side {own}, distro {peer}")
        if own not in local_ipv4s():
            log(f"WARNING: {own} is not on any adapter here; the vEthernet (WSL) address may have "
                "changed since the distro started")
        if net == "wsltcp":
            log(f"TCP client: connecting out to {peer}:{cfg.wsl_tcp_port} (no inbound rule needed; "
                "start the WSL side FIRST, it is the server)")
            return render_wsl_tcp_profile(cfg, peer), None, own
        return render_wsl_profile(cfg, own, peer), None, own
    ips = local_ipv4s()
    if cfg.ws_ip not in ips:
        log(f"WARNING: {cfg.ws_ip} (link.workstation_ip) is not assigned to any adapter: the link is down "
            "(run hil_link_setup.bat as administrator, or use --net local)")
    return render_profile(cfg, cfg.ws_ip), cfg.ds, cfg.ws_ip


def pick_profile_for(cfg: Cfg, a: argparse.Namespace) -> tuple[Path, str | None, str]:
    """pick_profile() plus the --discovery-server override (see its help text)."""
    prof, ds, ip = pick_profile(cfg, a.net)
    override = getattr(a, "discovery_server", "")
    return prof, (override or ds), ip


def cmd_env(cfg: Cfg, a: argparse.Namespace) -> int:
    prof, ds, ip = pick_profile_for(cfg, a)
    print(f"config            {hil_config.config_path()}")
    print(f"python            {sys.executable}  (isaacsim {isaac_version()})")
    print(f"ROS_DISTRO        {DISTRO}    bridge: {bridge_dir()}")
    print(f"ROS_DOMAIN_ID     {cfg.domain}    RMW_IMPLEMENTATION {cfg.rmw}")
    print(f"ROS_DISCOVERY_SERVER {ds or '(none: local mode)'}")
    print(f"FASTRTPS profile  {prof}    bound to {ip}")
    print(f"world / layout    {cfg.world} / {cfg.layout}")
    print(f"camera            {cfg.stereo['width']}x{cfg.stereo['height']} at {cfg.stereo['rate']} Hz, "
          f"~{cfg.camera_mbit} Mbit/s on the wire in link mode")
    return 0


def cmd_preflight(cfg: Cfg, a: argparse.Namespace) -> int:
    bad = 0

    def ok(m): log(f"OK    {m}")
    def warn(m): log(f"WARN  {m}")
    def fail(m):
        nonlocal bad
        bad += 1
        log(f"FAIL  {m}")

    ok(f"config {hil_config.config_path()}")
    ok(f"python {sys.executable} (isaacsim {isaac_version()})")
    bd = bridge_dir()
    if bd is None:
        fail("isaacsim.ros2.bridge not found: is isaacsim installed in this Python?")
    else:
        for part in ("lib", "rclpy"):
            (ok if (bd / DISTRO / part).is_dir() else fail)(f"bridge {DISTRO}\\{part}")
    smi = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total", "--format=csv,noheader"],
                         capture_output=True, text=True)
    (ok if smi.returncode == 0 else fail)(f"GPU: {smi.stdout.strip() or 'nvidia-smi failed'}")
    try:
        import torch
        (ok if torch.cuda.is_available() else warn)(f"torch {torch.__version__}, cuda available: {torch.cuda.is_available()}")
    except Exception as exc:  # noqa: BLE001
        warn(f"torch not importable here ({exc})")
    world = cfg.world
    chk = subprocess.run([sys.executable, str(cfg.isaac_dir / "tools" / "check_config.py"), "--scene", world],
                         capture_output=True, text=True, errors="replace", env=dict(os.environ, PYTHONUTF8="1"))
    (ok if chk.returncode == 0 else fail)(f"scene '{world}' config loads" + ("" if chk.returncode == 0 else f":\n{chk.stdout}{chk.stderr}"))
    if world == "office":
        usd = cfg.isaac_dir / "usd" / "generated" / "office" / "office.usd"
        (ok if usd.is_file() else warn)(f"office USD {'present' if usd.is_file() else 'missing: run hil_world_convert.bat once'}")
    ips = local_ipv4s()
    (ok if cfg.ws_ip in ips else warn)(f"link: {cfg.ws_ip} {'assigned' if cfg.ws_ip in ips else 'not assigned (only needed for --net link)'}")
    ok(f"camera budget {cfg.camera_mbit} Mbit/s (link mode, one copy per remote subscriber)")
    if a.orin:
        (ok if ping(cfg.orin_ip) else fail)(f"ping {cfg.orin_ip}")
        (ok if tcp_open(cfg.orin_ip, 22) else fail)(f"ssh port 22 on {cfg.orin_ip}")
    log("preflight found problems (above)" if bad else "preflight OK")
    return 1 if bad else 0


def cmd_world(cfg: Cfg, a: argparse.Namespace, rest: list[str]) -> int:
    tools = cfg.isaac_dir / "tools" / "sdf_to_usd.py"
    out = cfg.isaac_dir / "usd" / "generated" / "office"
    env = dict(os.environ, PYTHONUTF8="1", OMNI_KIT_ACCEPT_EULA="YES")
    if "--convert" in rest:
        cmd = [sys.executable, str(HERE / "convert_runner.py"), str(tools),
               "--sdf", str(REPO / "src" / "f1tenth_sim" / "worlds" / "husarion_office_2x.sdf"),
               "--models-dir", str(REPO / "src" / "husarion_gz_worlds" / "models"),
               "--fuel-cache", os.environ.get("GZ_FUEL_CACHE", str(Path.home() / ".gz" / "fuel")),
               "--out", str(out), "--report", str(out / "report.json"), *rest]
    else:
        cmd = [sys.executable, str(tools), "--plan",
               "--report", str(cfg.isaac_dir / "usd" / "generated" / "office_plan.json"), *rest]
    log("cmd: " + " ".join(cmd))
    if a.dry_run:
        return 0
    return subprocess.call(cmd, env=env)


def cmd_sim(cfg: Cfg, a: argparse.Namespace) -> int:
    prof, ds, ip = pick_profile_for(cfg, a)
    env = base_env(cfg, prof, ds)
    cmd = [sys.executable, str(cfg.isaac_dir / "sim" / "main.py"), *sim_args(cfg, a)]
    log(f"net={a.net}  domain={cfg.domain}  distro={DISTRO}  profile={prof}")
    log(f"discovery server: {ds or 'none'}   bridge: {bridge_dir()}")
    log("cmd: " + " ".join(cmd))
    if a.dry_run:
        return 0
    if "--gui" not in cmd:
        log("headless: the first /clock takes 20-70 s (Kit start, shader cache)")
    p = subprocess.Popen(cmd, env=env, cwd=str(cfg.isaac_dir))
    try:
        return p.wait()
    except KeyboardInterrupt:
        log("Ctrl+C: stopping Isaac Sim (up to 40 s)")
        try:
            return p.wait(timeout=40)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            p.terminate()
            return p.wait()


def probe(cfg: Cfg, a: argparse.Namespace, args: list[str]) -> int:
    prof, ds, _ = pick_profile_for(cfg, a)
    env = base_env(cfg, prof, ds)
    return subprocess.call([sys.executable, str(HERE / "ros_probe.py"), *args], env=env)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("preflight", "world", "sim", "topics", "hz", "reset", "env"))
    ap.add_argument("--net", choices=("local", "link", "wsl", "wsltcp"), default="local")
    ap.add_argument("--discovery-server", default="", metavar="ADDR:PORT",
                    help="override the Discovery Server of --net (wsl/wsltcp have none by "
                         "default). Needed when the WSL side runs the full stack: "
                         "supervisor_bringup.launch.py always sets ROS_DISCOVERY_SERVER, so "
                         "Isaac has to use the same server instead of unicast initial peers.")
    ap.add_argument("--orin", action="store_true")
    ap.add_argument("--world", default="")
    ap.add_argument("--layout", default="")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--keyboard", action="store_true")
    ap.add_argument("--rate", type=float, default=None)
    ap.add_argument("--duration", type=float, default=0.0, help="sim: stop after this many sim seconds")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--dry-run", action="store_true")
    a, rest = ap.parse_known_args()
    cfg = Cfg()
    if a.cmd == "env":
        return cmd_env(cfg, a)
    if a.cmd == "preflight":
        return cmd_preflight(cfg, a)
    if a.cmd == "world":
        return cmd_world(cfg, a, rest)
    if a.cmd == "sim":
        if rest:
            die(f"unknown option(s): {' '.join(rest)} (--help)")
        return cmd_sim(cfg, a)
    return probe(cfg, a, [a.cmd, *rest])


if __name__ == "__main__":
    sys.exit(main())
