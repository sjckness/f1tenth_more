#!/usr/bin/env python3
"""hil_office_batch -- the office HIL batch, driven from the WORKSTATION (the work, Isaac native).

Every feasible start pose x every planner mode x N repetitions, in one unattended run, then the
analysis. This side owns Isaac (one launch per pose: the start pose is fixed at launch) and drives
the Orin over ssh through car-hil (scripts/car/orin/hil.sh, installed there by scripts/car/deploy.sh
from linus): container, Discovery Server, stack up/down per mode, one repetition per call.
Nothing is edited on the Orin; the results stay there, in ~/f1tenth_hil_batch/<session>/.

    hil_office_batch.bat                         the whole batch (scripts/sim/office_batch/batch.yaml)
    hil_office_batch.bat --smoke                 1 pose, 1 mode, 1 repetition: the chain end to end
    hil_office_batch.bat --poses 2,16 --modes corridor_rrt --reps 1
    hil_office_batch.bat --smoke --poses 16 --mission scripts/sim/office_batch/missions/X.json
    hil_office_batch.bat --session 20261123_210000_office_door_left_5m    resume: skips finished runs
    hil_office_batch.bat --analyze-only SESSION
    hil_office_batch.bat --dry-run               the plan and the checks, nothing started

Per pose: Isaac (hil_win.py sim --net link --layout <batch layout> --set scene.car.start.*), then
car-hil wait-sim; per mode: car-hil stack-up (fresh bringup: the mode is fixed at launch), then
per repetition car-hil run (/reset_all + checks, load + start the mission, record to the end),
then car-hil stack-down; then Isaac off. A pose whose Isaac never comes up, or a mode whose stack
never gets READY, is marked (isaac_failed / stack_not_ready) and the batch goes on.

Before anything starts: the car-hil / car-hil-agent on the Orin must be byte-identical to this
checkout's scripts/car/orin/hil.sh / hil_agent.py, car-hil preflight must pass, and the office USD
must exist here. Ctrl+C stops Isaac and the Orin side cleanly.

When the two scripts differ, the batch installs them itself (the Orin is reachable only from the
work, on the cable): the same thing as `scripts/car/deploy.sh --tools-only` does from linus, for
these two files only, taken from the COMMIT (git show HEAD:...), never from the working tree. Only
from a clean checkout whose HEAD is on a remote branch, i.e. code that came from linus through git:
the work does not develop. --no-install-tools keeps the old behaviour (stop on a mismatch).

--mission PATH replaces batch.yaml:mission for this session (relative to the checkout, or absolute).
The session records which one ran (session.json: mission, mission_id, mission_sha256); resuming a
session with a different mission is refused, so one session never mixes two missions.

The work is shared: every --yield-check-s seconds (default batch.yaml yield_check_s, 300; 0 = off)
the batch looks at the Windows sessions (WTS API). When another user logs in, or reconnects to a
session that was disconnected when the batch started, the batch gives way: it kills Isaac and the
ssh in flight at once, then stack-down, analysis of what finished, container down. The repetition
in flight is lost (no result: --session SID redoes it later). A batch never starts while another
user's session is active.

Test-only options: --sessions-file F (JSON list of {"user", "state", "id"} read at every check,
instead of the WTS API; state "active" or "disconnected"), --ssh TARGET (instead of orin.ssh), --sim-cmd "..." (instead of Isaac; {x} {y}
{z} {yaw} are substituted; the command must run until killed or until its stdin closes).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
WIN = os.name == "nt"
sys.path.insert(0, str(HERE.parent))
import hil_config  # noqa: E402  (scripts/sim/hil_config.py)

CAR_HIL = ".local/bin/car-hil"          # relative to the Orin user's home (ssh starts there)
ORIN_TOOLS = {"car-hil": REPO / "scripts/car/orin/hil.sh",
              "car-hil-agent": REPO / "scripts/car/orin/hil_agent.py"}
MODES = ("corridor_mpc", "corridor_rrt", "p2p_geometric")
LOGF = None


def log(msg: str) -> None:
    line = f"[batch {time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    if LOGF is not None:
        LOGF.write(line + "\n")
        LOGF.flush()


def die(msg: str) -> "None":
    log(f"ERRORE: {msg}")
    raise SystemExit(1)


# ---- ssh -------------------------------------------------------------------------------------
class Orin:
    def __init__(self, target: str) -> None:
        self.base = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
                     "-o", "ServerAliveInterval=10", "-o", "ServerAliveCountMax=6", target]
        self.target = target
        self.proc: subprocess.Popen | None = None
        self.guard = True     # False for the cleanup after a yield: stack-down / analyze / down

    def check(self) -> None:
        if self.guard and YIELD.is_set():
            raise Yielded(YIELD_REASON[0])

    def run(self, args: list[str], timeout: float, input_text: str | None = None,
            echo: bool = True, retries: int = 2) -> tuple[int, str]:
        """car-hil ARGS on the Orin. rc 255 (ssh itself failed) is retried; output is returned
        and, with echo, printed line by line as it arrives."""
        remote = " ".join(shlex.quote(a) for a in [CAR_HIL, *args])
        for attempt in range(retries + 1):
            self.check()
            rc, out = self._call(remote, timeout, input_text, echo)
            self.check()
            if rc != 255:
                return rc, out
            log(f"ssh non riuscito (tentativo {attempt + 1}/{retries + 1}): {out.strip()[-200:]}")
            time.sleep(10 * (attempt + 1))
        return rc, out

    def sh(self, remote: str, timeout: float = 60) -> tuple[int, str]:
        return self._call(remote, timeout, None, False)

    def put(self, rel_path: str, data: bytes, timeout: float = 60) -> tuple[int, str]:
        """data -> ~/rel_path on the Orin, mode 755. Bytes, not text: a text pipe on Windows would
        turn LF into CRLF."""
        try:
            r = subprocess.run(self.base + [f"install -D -m 755 /dev/stdin {shlex.quote(rel_path)}"],
                               input=data, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               timeout=timeout)
        except subprocess.TimeoutExpired:
            return 124, f"timeout {timeout:.0f} s"
        except OSError as exc:
            return 255, str(exc)
        return r.returncode, r.stdout.decode("utf-8", "replace")

    def _call(self, remote, timeout, input_text, echo):
        try:
            p = subprocess.Popen(self.base + [remote], stdin=subprocess.PIPE if input_text is not None
                                 else subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
        except OSError as exc:
            return 255, str(exc)
        self.proc = p
        if input_text is not None:
            try:
                p.stdin.write(input_text)
                p.stdin.close()
            except OSError:
                pass
        out, t0 = [], time.monotonic()
        def reader():
            for ln in p.stdout:
                out.append(ln)
                if echo:
                    print("    " + ln.rstrip(), flush=True)
                if LOGF is not None:
                    LOGF.write("    " + ln)
        th = threading.Thread(target=reader, daemon=True)
        th.start()
        while p.poll() is None:
            if time.monotonic() - t0 > timeout:
                log(f"timeout {timeout:.0f} s su: {remote[:120]}")
                p.kill()
                p.wait()
                th.join(2)
                return 124, "".join(out)
            time.sleep(0.2)
        th.join(5)
        self.proc = None
        return p.returncode, "".join(out)

    def kill_current(self) -> None:
        p = self.proc
        if p is not None and p.poll() is None:
            p.kill()


# ---- Isaac -----------------------------------------------------------------------------------
def stray_isaac() -> list[int]:
    if not WIN:
        return []
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | "
          "Where-Object { $_.CommandLine -like '*main.py*--scene*' } | ForEach-Object { $_.ProcessId }")
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                           capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return []
    return [int(x) for x in r.stdout.split() if x.strip().isdigit()]


def kill_tree(pid: int) -> None:
    if WIN:
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(pid)], capture_output=True)
    else:
        try:
            os.killpg(pid, signal.SIGTERM)
            time.sleep(2)
            os.killpg(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


class Isaac:
    def __init__(self, batch: dict, sim_cmd: str, logdir: Path) -> None:
        self.batch, self.sim_cmd, self.logdir = batch, sim_cmd, logdir
        self.p: subprocess.Popen | None = None
        self.logf: Path | None = None

    def cmd(self, pose: dict) -> list[str]:
        s = pose["start"]
        if self.sim_cmd:
            return [a.format(x=s["x"], y=s["y"], z=s.get("z", 0.02), yaw=s["yaw_deg"])
                    for a in shlex.split(self.sim_cmd)]
        return [sys.executable, str(HERE / "hil_win.py"), "sim", "--net", "link",
                "--world", self.batch.get("world", "office"), "--layout", self.batch["layout"],
                "--set", f"scene.car.start.x={s['x']}", "--set", f"scene.car.start.y={s['y']}",
                "--set", f"scene.car.start.z={s.get('z', 0.02)}",
                "--set", f"scene.car.start.yaw_deg={s['yaw_deg']}"]

    def start(self, pose: dict, attempt: int) -> None:
        self.stop()
        if YIELD.is_set():
            raise Yielded(YIELD_REASON[0])
        for pid in stray_isaac():
            log(f"un altro Isaac gira gia' (pid {pid}): lo chiudo, due sim pubblicherebbero /clock")
            kill_tree(pid)
            time.sleep(3)
        self.logf = self.logdir / f"isaac_p{pose['idx']:02d}_{attempt}.log"
        cmd = self.cmd(pose)
        log(f"Isaac: posa {pose['idx']} -> {self.logf.name}")
        log("  cmd: " + " ".join(cmd))
        out = open(self.logf, "w", encoding="utf-8", errors="replace")
        kw = ({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if WIN else {"start_new_session": True})
        self.p = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.PIPE,
                                  cwd=str(HERE), **kw)
        out.close()

    def alive(self) -> bool:
        return self.p is not None and self.p.poll() is None

    def stop(self) -> None:
        if self.p is not None and self.p.poll() is None:
            log(f"Isaac: stop (pid {self.p.pid})")
            try:
                self.p.stdin.close()
            except OSError:
                pass
            kill_tree(self.p.pid)
            try:
                self.p.wait(timeout=30)
            except subprocess.TimeoutExpired:
                log("ATTENZIONE: Isaac non e' uscito 30 s dopo il kill")
        for pid in stray_isaac():
            kill_tree(pid)
        self.p = None

    def tail(self, n: int = 8) -> str:
        if not self.logf or not self.logf.exists():
            return ""
        return "\n".join(self.logf.read_text(encoding="utf-8", errors="replace").splitlines()[-n:])


# ---- other users on the work ------------------------------------------------------------------
YIELD = threading.Event()
YIELD_REASON = [""]


class Yielded(Exception):
    pass


def win_sessions() -> list[dict]:
    """The Windows sessions with a user: [{"user": "DOMAIN\\name", "state": "active"|"disconnected"|...,
    "id": N}]. WTS API, not `query user`: its output is localized."""
    import ctypes
    from ctypes import wintypes

    class SI(ctypes.Structure):
        _fields_ = [("SessionId", wintypes.DWORD), ("pWinStationName", wintypes.LPWSTR),
                    ("State", ctypes.c_int)]
    wts = ctypes.WinDLL("wtsapi32")
    wts.WTSEnumerateSessionsW.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD,
                                          ctypes.POINTER(ctypes.POINTER(SI)), ctypes.POINTER(wintypes.DWORD)]
    wts.WTSQuerySessionInformationW.argtypes = [wintypes.HANDLE, wintypes.DWORD, ctypes.c_int,
                                                ctypes.POINTER(wintypes.LPWSTR), ctypes.POINTER(wintypes.DWORD)]
    wts.WTSFreeMemory.argtypes = [ctypes.c_void_p]
    states = {0: "active", 1: "connected", 4: "disconnected", 5: "idle"}
    arr, n = ctypes.POINTER(SI)(), wintypes.DWORD()
    if not wts.WTSEnumerateSessionsW(None, 0, 1, ctypes.byref(arr), ctypes.byref(n)):
        raise OSError(f"WTSEnumerateSessionsW: {ctypes.GetLastError()}")
    out = []
    try:
        for i in range(n.value):
            sid, st = arr[i].SessionId, arr[i].State
            vals = []
            for cls in (5, 7):              # WTSUserName, WTSDomainName
                buf, ln = wintypes.LPWSTR(), wintypes.DWORD()
                if wts.WTSQuerySessionInformationW(None, sid, cls, ctypes.byref(buf), ctypes.byref(ln)):
                    vals.append(buf.value or "")
                    wts.WTSFreeMemory(buf)
                else:
                    vals.append("")
            if vals[0]:
                out.append({"user": f"{vals[1]}\\{vals[0]}" if vals[1] else vals[0],
                            "state": states.get(st, str(st)), "id": int(sid)})
    finally:
        wts.WTSFreeMemory(arr)
    return out


def me() -> str:
    return (os.environ.get("USERNAME") or os.environ.get("USER") or "").lower()


def others(sessions: list[dict], user: str) -> list[dict]:
    """The sessions of users other than `user` (the name, with or without the domain)."""
    u = user.split("\\")[-1].lower()
    return [s for s in sessions if s["user"].split("\\")[-1].lower() != u]


def must_yield(now: list[dict], baseline: list[dict]) -> list[dict]:
    """Another user's session that is new since the start, or active now. A session that was already
    there and disconnected at the start is tolerated while it stays disconnected."""
    old = {(s["id"], s["user"].lower()) for s in baseline}
    return [s for s in now if s["state"] == "active" or (s["id"], s["user"].lower()) not in old]


class SessionGuard(threading.Thread):
    """Every `interval` s: probe() -> the other users' sessions; on must_yield, sets YIELD and calls
    on_yield() (kill Isaac and the ssh in flight) once, then stops."""

    def __init__(self, probe, interval: float, baseline: list[dict], on_yield) -> None:
        super().__init__(daemon=True, name="session-guard")
        self.probe, self.interval, self.baseline, self.on_yield = probe, interval, baseline, on_yield
        self.stopped = threading.Event()

    def run(self) -> None:
        failed = False
        while not self.stopped.wait(self.interval):
            try:
                hit = must_yield(self.probe(), self.baseline)
            except Exception as exc:  # noqa: BLE001  a broken probe never stops the batch
                if not failed:
                    log(f"ATTENZIONE: controllo delle sessioni fallito ({exc}): continuo senza")
                failed = True
                continue
            if hit:
                who = ", ".join(f"{s['user']} ({s['state']}, sessione {s['id']})" for s in hit)
                YIELD_REASON[0] = f"un altro utente sulla work: {who}"
                log(f"CEDO LA WORK: {YIELD_REASON[0]}; chiudo Isaac e lo stack, la ripetizione in corso e' persa")
                YIELD.set()
                self.on_yield()
                return


def keep_awake() -> None:
    if WIN:
        import ctypes
        if ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | 0x00000001):
            log("Windows non va in sospensione finche' il batch gira")


# ---- plan ------------------------------------------------------------------------------------
def parse_ids(spec: str) -> list[int]:
    out = []
    for part in spec.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def git_info() -> dict:
    def g(*a):
        try:
            return subprocess.check_output(["git", "-C", str(REPO), *a], text=True,
                                           stderr=subprocess.DEVNULL).strip()
        except (OSError, subprocess.CalledProcessError):
            return None
    return {"git_commit": g("rev-parse", "HEAD"), "git_branch": g("rev-parse", "--abbrev-ref", "HEAD"),
            "git_dirty": bool(g("status", "--porcelain", "--untracked-files=no"))}


def git_bytes(*a: str) -> bytes | None:
    try:
        return subprocess.check_output(["git", "-C", str(REPO), *a], stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return None


def install_gate(gi: dict) -> str:
    """'' when this checkout may install its car-hil scripts on the Orin, else why not."""
    if not gi["git_commit"]:
        return "non e' un checkout git"
    if gi["git_dirty"]:
        return "il checkout ha modifiche non committate"
    remotes = (git_bytes("branch", "-r", "--contains", "HEAD") or b"").decode().split()
    if not remotes:
        return f"HEAD {gi['git_commit'][:12]} non e' su nessun branch remoto (fai git fetch; la work non sviluppa)"
    return ""


def remote_tool_sums(orin: "Orin") -> dict:
    _, out = orin.sh(f"sha256sum {CAR_HIL} {CAR_HIL}-agent", 30)
    return {Path(ln.split()[1]).name: ln.split()[0] for ln in out.splitlines() if len(ln.split()) == 2}


def sha256(p: Path) -> str:
    # git on Windows may check text out with CRLF: hash the LF form, as it is on the Orin
    return hashlib.sha256(p.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def resolve_mission(arg: str, batch: dict) -> tuple[Path, str, dict, str]:
    """(path, path as recorded, parsed mission, sha256 of its LF form). arg '' = batch.yaml:mission."""
    p = Path(arg) if arg else REPO / batch["mission"]
    if not p.is_absolute():
        p = p.resolve() if p.exists() else REPO / p
    if not p.is_file():
        die(f"missione {arg or batch['mission']}: file assente ({p})")
    raw = p.read_bytes().replace(b"\r\n", b"\n")
    try:
        doc = json.loads(raw)
    except ValueError as exc:
        die(f"missione {p}: JSON non valido ({exc})")
    if not isinstance(doc, dict) or not doc.get("moves"):
        die(f"missione {p}: niente 'moves'")
    try:
        rec = p.resolve().relative_to(REPO.resolve()).as_posix()
    except ValueError:
        rec = str(p)
    return p, rec, doc, hashlib.sha256(raw).hexdigest()


def run_id(pidx: int, mode: str, rep: int) -> str:
    return f"p{pidx:02d}_{mode}_r{rep}"


def main() -> int:
    global LOGF
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--batch", default=str(REPO / "scripts/sim/office_batch/batch.yaml"))
    ap.add_argument("--session", default="", help="resume this session (skips runs with a result)")
    ap.add_argument("--poses", default="", help="e.g. 1,2,5-7 (default: all)")
    ap.add_argument("--modes", default="", help="comma-separated (default: batch.yaml)")
    ap.add_argument("--reps", type=int, default=0)
    ap.add_argument("--smoke", action="store_true", help="the first pose, the first mode, 1 repetition")
    ap.add_argument("--mission", default="", metavar="PATH",
                    help="mission JSON instead of batch.yaml:mission (recorded in session.json)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--analyze-only", default="", metavar="SESSION")
    ap.add_argument("--allow-version-mismatch", action="store_true",
                    help="run although car-hil on the Orin differs from this checkout")
    ap.add_argument("--no-install-tools", action="store_true",
                    help="do not install this commit's car-hil / car-hil-agent on the Orin when they differ")
    ap.add_argument("--ssh", default="", help="TEST: ssh target instead of orin.ssh")
    ap.add_argument("--sim-cmd", default="", help="TEST: replaces Isaac ({x} {y} {z} {yaw})")
    ap.add_argument("--keep-up", action="store_true", help="leave the Orin container up at the end")
    ap.add_argument("--yield-check-s", type=float, default=-1,
                    help="seconds between checks for other users' sessions on the work (default "
                         "batch.yaml yield_check_s, 300; 0 = never give way)")
    ap.add_argument("--sessions-file", default="", help="TEST: sessions from this JSON, not the WTS API")
    a = ap.parse_args()

    import yaml
    cfgd = hil_config.load()
    cfg = lambda k: hil_config._get(cfgd, k)  # noqa: E731
    orin = Orin(a.ssh or str(cfg("orin.ssh")))
    batch = yaml.safe_load(open(a.batch, encoding="utf-8"))
    orin_ip, ws_ip = str(cfg("link.orin_ip")), str(cfg("link.workstation_ip"))
    port, domain = int(cfg("ros.discovery_server_port")), int(cfg("ros.domain_id"))
    buf = int(cfg("link.socket_buffer_mb") or 16)
    to = batch.get("timeouts", {})

    if a.analyze_only:
        rc, _ = orin.run(["analyze", a.analyze_only], timeout=1800)
        return rc

    # ---- the plan
    poses_doc = json.load(open(REPO / batch["poses_file"], encoding="utf-8"))
    cands = poses_doc["candidates"] if isinstance(poses_doc, dict) else poses_doc
    poses = [dict(c, idx=i + 1) for i, c in enumerate(cands)]
    modes = [m for m in (a.modes.split(",") if a.modes else batch["modes"]) if m]
    for m in modes:
        if m not in MODES:
            die(f"modo sconosciuto: {m} ({', '.join(MODES)})")
    reps = a.reps or int(batch.get("reps", 3))
    sel = poses
    if a.poses:
        want = set(parse_ids(a.poses))
        sel = [p for p in poses if p["idx"] in want]
    if a.smoke:
        sel, modes, reps = sel[:1], modes[:1], 1
    if not sel:
        die("nessuna posa selezionata")
    plan = [(p, m, r) for p in sel for m in modes for r in range(1, reps + 1)]
    sid = a.session or time.strftime("%Y%m%d_%H%M%S_") + batch.get("name", "office_batch") + ("_smoke" if a.smoke else "")
    logdir = HERE / "batch" / sid
    logdir.mkdir(parents=True, exist_ok=True)
    LOGF = open(logdir / "batch.log", "a", encoding="utf-8")
    gi = git_info()
    log(f"sessione {sid}: {len(sel)} pose x {len(modes)} modi x {reps} rip. = {len(plan)} run")
    log(f"checkout {gi['git_branch']} {str(gi['git_commit'])[:12]}{' (modifiche non committate!)' if gi['git_dirty'] else ''}")
    log(f"Orin {orin.target} (link {orin_ip}:{port}, dominio {domain}), workstation {ws_ip}; log {logdir}")
    mpath, mrec, mdoc, msha = resolve_mission(a.mission, batch)
    log(f"missione {mrec} (mission_id {mdoc.get('mission_id', '?')}, sha256 {msha[:12]})"
        + ("" if a.mission else ", da batch.yaml"))
    if a.session:
        rc, out = orin.sh(f"cat f1tenth_hil_batch/{sid}/session.json", 30)
        try:
            old = json.loads(out) if rc == 0 else {}
        except ValueError:
            old = {}
        if old.get("mission_sha256", msha) != msha or (
                "mission_sha256" not in old and old.get("mission_name", mpath.name) != mpath.name):
            die(f"la sessione {sid} ha girato con la missione {old.get('mission', old.get('mission_name'))} "
                f"(sha256 {str(old.get('mission_sha256', '?'))[:12]}), non con {mrec}: "
                "una sessione non mescola due missioni (nuova sessione, o --mission quella)")

    # ---- checks
    remote = remote_tool_sums(orin)
    would_install = False
    differ = [n for n, p in ORIN_TOOLS.items() if remote.get(n) != sha256(p)]
    mism = [f"{n}: Orin {remote.get(n, 'assente')[:12]} != checkout {sha256(ORIN_TOOLS[n])[:12]}"
            for n in differ]
    if mism and not a.no_install_tools and not a.allow_version_mismatch:
        commit = str(gi["git_commit"])[:12]
        why = install_gate(gi)
        if why:
            log(f"non installo car-hil dal checkout: {why}")
        elif a.dry_run:
            for n, m in zip(differ, mism):
                log(f"dry-run: installerei {n} di {commit} sull'Orin ({m})")
            mism, would_install = [], True
        else:
            for n in differ:
                rel = ORIN_TOOLS[n].relative_to(REPO).as_posix()
                data = git_bytes("show", f"HEAD:{rel}")
                if data is None:
                    die(f"git show HEAD:{rel} fallito")
                rc, out = orin.put(f".local/bin/{n}", data)
                if rc != 0:
                    die(f"installazione di {n} sull'Orin fallita (rc {rc}): {out.strip()[-300:]}")
                log(f"installato {n} di {commit} sull'Orin")
            remote = remote_tool_sums(orin)
            mism = [f"{n}: Orin {remote.get(n, 'assente')[:12]} != checkout {sha256(p)[:12]}"
                    for n, p in ORIN_TOOLS.items() if remote.get(n) != sha256(p)]
    if mism:
        msg = ("car-hil sull'Orin non e' quello di questo checkout:\n  " + "\n  ".join(mism)
               + f"\n  committa su linus, git pull qui e rilancia (o scripts/car/deploy.sh --tools-only "
               f"{str(gi['git_commit'])[:12]} da linus)")
        if not a.allow_version_mismatch:
            die(msg)
        log("ATTENZIONE " + msg)
    elif would_install:
        log("car-hil e car-hil-agent sull'Orin: diversi, il run vero li installa da questo checkout")
    else:
        log("car-hil e car-hil-agent sull'Orin == questo checkout")
    if not a.sim_cmd:
        usd = REPO / "sim/isaac/usd/generated/office/office.usd"
        if not usd.is_file():
            die(f"manca {usd}: estrailo da work-20261119.tar.gz (sim/isaac/usd/generated/office/) "
                "o rigeneralo con hil_world_convert.bat")
    yield_s = a.yield_check_s if a.yield_check_s >= 0 else float(batch.get("yield_check_s", 300))
    probe = None
    if yield_s > 0:
        if a.sessions_file:
            probe = lambda: others(json.load(open(a.sessions_file, encoding="utf-8")), me())  # noqa: E731
        elif WIN:
            probe = lambda: others(win_sessions(), me())  # noqa: E731
        else:
            log("controllo delle sessioni: solo su Windows (o --sessions-file), qui spento")
    baseline: list[dict] = []
    if probe is not None:
        try:
            baseline = probe()
        except Exception as exc:  # noqa: BLE001
            log(f"ATTENZIONE: controllo delle sessioni non disponibile ({exc}): il batch non cede la work")
            probe = None
    if probe is not None:
        act = [s for s in baseline if s["state"] == "active"]
        if act:
            die("un altro utente e' attivo sulla work: " + ", ".join(f"{s['user']} (sessione {s['id']})" for s in act)
                + "; il batch non parte (--yield-check-s 0 per partire lo stesso)")
        log(f"sessioni altrui: controllo ogni {yield_s:.0f} s, alla prima nuova o attiva il batch si chiude"
            + (" (gia' presenti, disconnesse: " + ", ".join(s["user"] for s in baseline) + ")" if baseline else ""))
    rc, out = orin.run(["preflight", "--orin-ip", orin_ip, "--ws-ip", ws_ip, "--port", str(port),
                        "--min-free-gb", str(batch.get("min_free_gb", 8))], timeout=120)
    if rc != 0:
        die("car-hil preflight: FAIL (righe sopra)")
    if a.dry_run:
        for p in sel:
            s = p["start"]
            log(f"  posa {p['idx']:2d}: ({s['x']}, {s['y']}) yaw {s['yaw_deg']}, varco {p['door']['width_m']} m "
                f"a ({p['door']['x']}, {p['door']['y']})")
        log("dry-run: niente avviato")
        return 0

    # ---- session
    session = {"name": batch.get("name"), "sid": sid, "batch_file": os.path.relpath(a.batch, REPO),
               "poses_file": batch["poses_file"], "world": batch.get("world", "office"),
               "layout": batch["layout"], "mission_name": mpath.name, "mission": mrec,
               "mission_id": mdoc.get("mission_id"), "mission_sha256": msha,
               "mission_override": bool(a.mission), "modes": modes,
               "reps": reps, "poses": sel, "stack_args": batch.get("stack_args", []), "timeouts": to,
               "workstation": os.environ.get("COMPUTERNAME") or os.uname().nodename,
               "isaac_cmd_example": Isaac(batch, a.sim_cmd, logdir).cmd(sel[0]),
               "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), **gi}
    rc, _ = orin.run(["up", sid, "--orin-ip", orin_ip, "--port", str(port), "--domain", str(domain),
                      "--buffer-mb", str(buf)], timeout=180)
    if rc != 0:
        die("car-hil up fallito")
    if not a.session:
        orin.run(["put", sid, "session.json"], 60, input_text=json.dumps(session, indent=1))
    else:
        rc, out = orin.sh(f"cat f1tenth_hil_batch/{sid}/session.json", 30)
        try:
            old = json.loads(out)
            known = {p["idx"] for p in old["poses"]}
            if not {p["idx"] for p in sel} <= known:
                old["poses"] += [p for p in sel if p["idx"] not in known]
                orin.run(["put", sid, "session.json"], 60, input_text=json.dumps(old, indent=1))
        except (ValueError, KeyError):
            orin.run(["put", sid, "session.json"], 60, input_text=json.dumps(session, indent=1))
    orin.run(["put", sid, "mission.json"], 60, input_text=mpath.read_bytes().replace(b"\r\n", b"\n").decode("utf-8"))
    if batch.get("map_geo") and (REPO / batch["map_geo"]).is_file():
        orin.run(["put", sid, "map_geo.json"], 60, input_text=(REPO / batch["map_geo"]).read_text(encoding="utf-8"),
                 echo=False)
    rc, out = orin.run(["done", sid], 60, echo=False)
    done = set(out.split())
    todo = [(p, m, r) for p, m, r in plan if run_id(p["idx"], m, r) not in done]
    log(f"{len(plan) - len(todo)} run gia' fatti, {len(todo)} da fare")

    keep_awake()
    isaac = Isaac(batch, a.sim_cmd, logdir)
    guard = None
    if probe is not None:
        def on_yield():
            orin.kill_current()
            if isaac.p is not None and isaac.p.poll() is None:
                kill_tree(isaac.p.pid)
        guard = SessionGuard(probe, yield_s, baseline, on_yield)
        guard.start()
    t_batch = time.monotonic()
    n_done = 0
    stack_args = list(batch.get("stack_args", []))

    def mark(p, m, r, status, note):
        orin.run(["mark", sid, run_id(p["idx"], m, r), m, str(r), str(p["idx"]), status, note], 60)

    def stack_up(p, m, attempt_tag) -> bool:
        for k in range(int(to.get("stack_retries", 1)) + 1):
            tag = f"p{p['idx']:02d}_{m}_{attempt_tag}{k}"
            rc, _ = orin.run(["stack-up", sid, m, tag, "--timeout", str(to.get("stack_ready_s", 240)),
                              *stack_args], timeout=float(to.get("stack_ready_s", 240)) + 120)
            if rc == 0:
                return True
            orin.run(["stack-down", sid], 180)
            if not isaac.alive():
                return False
        return False

    def pending(p) -> list[tuple[str, int]]:
        rc, out = orin.run(["done", sid], 60, echo=False)
        dn = set(out.split()) if rc == 0 else set()
        return [(m, r) for pp, m, r in todo if pp["idx"] == p["idx"] and run_id(p["idx"], m, r) not in dn]

    try:
        for p in sel:
            if not pending(p):
                continue
            log(f"===== posa {p['idx']} ({sel.index(p) + 1}/{len(sel)}): {len(pending(p))} run")
            launches = 0
            max_launches = int(to.get("isaac_restarts", 2)) + 1
            while launches < max_launches and pending(p):
                launches += 1
                isaac.start(p, launches)
                rc, _ = orin.run(["wait-sim", sid, "--timeout", str(to.get("isaac_s", 300))],
                                 timeout=float(to.get("isaac_s", 300)) + 60)
                if rc != 0 or not isaac.alive():
                    log(f"Isaac non pronto (lancio {launches}/{max_launches}); coda del log:\n{isaac.tail()}")
                    isaac.stop()
                    time.sleep(6)
                    continue
                for m in modes:
                    mtodo = [r for mm, r in pending(p) if mm == m]
                    if not mtodo:
                        continue
                    if not stack_up(p, m, f"L{launches}_"):
                        if not isaac.alive():
                            break
                        for r in mtodo:
                            mark(p, m, r, "stack_not_ready", "stack never READY")
                        continue
                    for r in mtodo:
                        if not isaac.alive():
                            break
                        rid = run_id(p["idx"], m, r)
                        t0 = time.monotonic()
                        rc, out = orin.run(["run", sid, rid, m, str(r), str(p["idx"]), "--timeout",
                                            str(to.get("run_s", 180))], timeout=float(to.get("run_s", 180)) + 300)
                        n_done += 1
                        el = time.monotonic() - t_batch
                        eta = el / n_done * max(len(todo) - n_done, 0)
                        log(f"{rid}: rc {rc}, {time.monotonic() - t0:.0f} s; {n_done}/{len(todo)}, "
                            f"fine stimata tra {eta / 3600:.1f} h")
                        if rc != 0 and r != mtodo[-1] and isaac.alive():
                            # reset / load / start failed, or the agent died: a fresh stack for the next one
                            log(f"{rid}: rc {rc}, riavvio lo stack prima della prossima ripetizione")
                            orin.run(["stack-down", sid], 180)
                            if not stack_up(p, m, f"L{launches}_after_r{r}_"):
                                break
                    orin.run(["stack-down", sid], 180)
                    if not isaac.alive():
                        break
                if not isaac.alive():
                    log(f"Isaac e' morto durante la posa {p['idx']} (lancio {launches}): coda del log:\n{isaac.tail()}")
                isaac.stop()
                time.sleep(6)    # Isaac's participant out of the Discovery Server before the next one
            for m, r in pending(p):
                # only what is still missing after every launch: stack_not_ready was marked above
                mark(p, m, r, "isaac_failed", f"Isaac not usable after {launches} launches")
    except KeyboardInterrupt:
        log("Ctrl+C: chiudo Isaac e lo stack sull'Orin")
    except Yielded as exc:
        log(f"batch interrotto: {exc}; riprendi con --session {sid}")
    finally:
        if guard is not None:
            guard.stopped.set()
        orin.guard = False
        isaac.stop()
        orin.run(["stack-down", sid], 180)
    log("analisi sull'Orin")
    orin.run(["analyze", sid], timeout=1800)
    rc, out = orin.sh(f"cat f1tenth_hil_batch/{sid}/analysis/report.md", 60)
    if rc == 0:
        (logdir / "report.md").write_text(out, encoding="utf-8")
        log(f"report: {logdir / 'report.md'} (copia); dati: Orin ~/f1tenth_hil_batch/{sid}/")
    if not a.keep_up or YIELD.is_set():
        orin.run(["down", sid], 240)
    log(f"fine in {(time.monotonic() - t_batch) / 3600:.2f} h")
    if YIELD.is_set():
        log(f"CEDUTA LA WORK ({YIELD_REASON[0]}): sessione incompleta, --session {sid} per finirla")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
