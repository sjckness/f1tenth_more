#!/usr/bin/env python3
"""hil_follow -- the WORKSTATION half of an unattended campaign night.

The scene is fixed at Isaac launch and the office campaign needs three of them, so somebody has to
restart Isaac between missions. This is that somebody: it asks the Orin, every few seconds, which
scene it wants (scripts/sim/campaign/hil_night.sh writes it), and (re)starts Isaac with that layout
exactly as hil_sim_link.bat does -- `hil_win.py sim --net link --layout L`.

    hil_follow.bat                 leave it open all night; Ctrl+C stops it, and Isaac with it

It only ever connects OUT, over ssh, to orin.ssh (hil.local.yaml): no listening port and no
firewall rule on this PC. It needs the password-less key run_windows.md sets up; it checks that
first and says so if the key is missing.

THE MAILBOX, on the Orin, in ~/.local/state/f1tenth_hil/isaac/ (outside the checkout on purpose):
    request   "SEQ run LAYOUT"  or  "SEQ stop"         written by hil_night.sh
    state     key=value lines, then the run.log tail   written here at every poll
A new SEQ is the only trigger. The Orin judges this side alive from the state file's mtime, on
its own clock, so the two clocks never have to agree.

What it does on its own, because nobody is watching:
  - keeps Windows from sleeping while it runs (SetThreadExecutionState; ends with the process);
  - restarts Isaac up to twice if it dies while its request is still the current one, and
    reports every restart, since a mission run across a restart is a finding;
  - refuses to start while another Isaac (sim/main.py) is running: two sims would publish /clock.
One log per launch: scripts/sim/win/night/<seq>_<layout>.log (run.log is left to the .bat files).

Test-only options (they run this on Linux too): --mailbox-dir DIR uses a local directory instead
of ssh; --sim-cmd "..." replaces the Isaac command line.
"""
from __future__ import annotations

import argparse
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WIN = os.name == "nt"
MAILBOX = ".local/state/f1tenth_hil/isaac"     # relative to the Orin user's home
POLL_S = 3.0
MAX_RESTARTS = 2
RTF_RE = re.compile(r"RTF (\d+\.\d+) \(sim")


def log(msg: str) -> None:
    print(f"[follow {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ---- transport ---------------------------------------------------------------------------
class Ssh:
    def __init__(self, target: str) -> None:
        self.target = target
        self.base = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                     "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2", target]

    def check(self) -> str | None:
        try:
            r = subprocess.run(self.base + ["echo ok"], capture_output=True, text=True, timeout=20)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return str(exc)
        return None if r.stdout.strip() == "ok" else (r.stderr.strip() or f"rc {r.returncode}")

    def exchange(self, state: str) -> str | None:
        """Write the state, read the request, in ONE connection. None: the Orin is unreachable."""
        remote = (f"mkdir -p {MAILBOX} && cat > {MAILBOX}/state.tmp && mv {MAILBOX}/state.tmp "
                  f"{MAILBOX}/state; cat {MAILBOX}/request 2>/dev/null; true")
        try:
            r = subprocess.run(self.base + [remote], input=state, capture_output=True,
                               text=True, encoding="utf-8", errors="replace", timeout=25)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return r.stdout if r.returncode == 0 else None


class LocalDir:   # test transport
    def __init__(self, d: str) -> None:
        self.d = Path(d)

    def check(self) -> str | None:
        self.d.mkdir(parents=True, exist_ok=True)
        return None

    def exchange(self, state: str) -> str | None:
        tmp = self.d / "state.tmp"
        tmp.write_text(state, encoding="utf-8")
        os.replace(tmp, self.d / "state")
        req = self.d / "request"
        return req.read_text(encoding="utf-8") if req.exists() else ""


# ---- Isaac processes ---------------------------------------------------------------------
def stray_isaac() -> list[int]:
    """PIDs of any Isaac sim/main.py on this PC, ours or not (a hand-started one, an orphan)."""
    if not WIN:
        r = subprocess.run(["pgrep", "-f", r"sim/main\.py --scene"], capture_output=True, text=True)
        return [int(x) for x in r.stdout.split()]
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | "
          "Where-Object { $_.CommandLine -like '*main.py*--scene*' } | "
          "ForEach-Object { $_.ProcessId }")
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                           capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return []
    return [int(x) for x in r.stdout.split() if x.strip().isdigit()]


def kill_tree(pid: int) -> None:
    if WIN:
        # /T while the tree is intact: hil_win.py waits on main.py, so the parent is still there.
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(pid)], capture_output=True)
    else:
        try:
            os.killpg(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass


class Isaac:
    def __init__(self, sim_cmd: list[str] | None) -> None:
        self.sim_cmd = sim_cmd
        self.p: subprocess.Popen | None = None
        self.seq = ""
        self.layout = ""
        self.logf: Path | None = None
        self.started = 0.0
        self.restarts = 0
        self.exit_code: int | None = None
        self.phase = "idle"
        self.rtf = ""

    def cmd(self, layout: str) -> list[str]:
        if self.sim_cmd:
            return [a.replace("{layout}", layout) for a in self.sim_cmd]
        return [sys.executable, str(HERE / "hil_win.py"), "sim", "--net", "link", "--layout", layout]

    def start(self, seq: str, layout: str, restart: bool = False) -> None:
        self.stop()
        left = stray_isaac()
        if left:
            log(f"another Isaac is still running (pid {left}): killing it, two sims would both "
                "publish /clock")
            for pid in left:
                kill_tree(pid)
            time.sleep(3)
        if not restart:
            self.restarts = 0
        self.seq, self.layout = seq, layout
        logs = HERE / "night"
        logs.mkdir(exist_ok=True)
        self.logf = logs / f"{seq}_{layout}{'_r%d' % self.restarts if restart else ''}.log"
        cmd = self.cmd(layout)
        log(f"seq {seq}: starting Isaac, layout {layout} -> {self.logf}")
        log("cmd: " + " ".join(cmd))
        out = open(self.logf, "w", encoding="utf-8", errors="replace")
        kw = ({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if WIN
              else {"start_new_session": True})
        self.p = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT,
                                  stdin=subprocess.DEVNULL, cwd=str(HERE), **kw)
        out.close()
        self.started, self.exit_code, self.phase, self.rtf = time.time(), None, "starting", ""

    def stop(self) -> None:
        if self.p is not None and self.p.poll() is None:
            log(f"stopping Isaac (pid {self.p.pid}, layout {self.layout})")
            kill_tree(self.p.pid)
            try:
                self.p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                log("WARNING: Isaac did not exit 20 s after taskkill")
        for pid in stray_isaac():       # an orphaned main.py, if its parent went first
            kill_tree(pid)
        if self.p is not None and self.phase not in ("exited",):
            self.phase = "stopped"
        self.p = None

    def poll(self) -> None:
        if self.p is None:
            return
        if self.logf and self.logf.exists():
            text = self.logf.read_text(encoding="utf-8", errors="replace")
            m = RTF_RE.findall(text)
            if m:
                self.rtf = m[-1]
                if self.phase == "starting":
                    self.phase = "running"
                    log(f"seq {self.seq}: Isaac is running, RTF {self.rtf} "
                        f"({time.time() - self.started:.0f} s after launch)")
        rc = self.p.poll()
        if rc is not None and self.phase in ("starting", "running"):
            self.exit_code, self.phase = rc, "exited"
            log(f"seq {self.seq}: Isaac EXITED with code {rc}")

    def tail(self, n: int = 25) -> str:
        if not self.logf or not self.logf.exists():
            return ""
        lines = self.logf.read_text(encoding="utf-8", errors="replace").splitlines()
        return "\n".join(lines[-n:])


def state_text(isaac: Isaac, note: str) -> str:
    kv = {"seq": isaac.seq, "layout": isaac.layout, "phase": isaac.phase, "rtf": isaac.rtf,
          "restarts": isaac.restarts, "exit_code": "" if isaac.exit_code is None else isaac.exit_code,
          "pid": isaac.p.pid if isaac.p else "", "uptime_s": int(time.time() - isaac.started) if isaac.p else "",
          "log": str(isaac.logf or ""), "note": note, "host": os.environ.get("COMPUTERNAME", "")}
    return "".join(f"{k}={v}\n" for k, v in kv.items()) + "---- log tail\n" + isaac.tail() + "\n"


def keep_awake() -> None:
    if not WIN:
        return
    import ctypes
    ES_CONTINUOUS, ES_SYSTEM_REQUIRED = 0x80000000, 0x00000001
    if ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED):
        log("Windows will not sleep while this runs (the screen may still turn off)")
    else:
        log("WARNING: could not keep Windows awake: disable sleep by hand for tonight")


# ---- main loop ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mailbox-dir", default="", help="TEST: a local directory instead of ssh")
    ap.add_argument("--sim-cmd", default="", help="TEST: replaces the Isaac command; {layout} is substituted")
    ap.add_argument("--once", action="store_true", help="TEST: exit when a stop request is served")
    a = ap.parse_args()

    if a.mailbox_dir:
        tr = LocalDir(a.mailbox_dir)
        where = a.mailbox_dir
    else:
        sys.path.insert(0, str(HERE.parent))
        import hil_config  # noqa: E402  (scripts/sim/hil_config.py)
        target = str(hil_config._get(hil_config.load(), "orin.ssh"))
        tr = Ssh(target)
        where = f"{target}:~/{MAILBOX}"
    err = tr.check()
    if err:
        log(f"ERROR: cannot reach {where} without a password: {err}")
        log("  a key: ssh-keygen -t ed25519, then append the .pub to the Orin's ~/.ssh/authorized_keys")
        log("  (output/hil/run_windows.md, last line of the PC list)")
        return 1
    log(f"mailbox {where}; polling every {POLL_S:.0f} s. Ctrl+C stops this AND Isaac.")
    left = stray_isaac()
    if left:
        log(f"ERROR: an Isaac is already running here (pid {left}). Close it (Ctrl+C in its "
            "window) and start this again: it will start Isaac itself when the Orin asks.")
        return 1
    keep_awake()

    isaac = Isaac(shlex.split(a.sim_cmd) if a.sim_cmd else None)
    served = ""          # the request line last acted on
    note = "waiting for a request"
    unreachable_since = 0.0
    try:
        while True:
            isaac.poll()
            # a sim that died while its request is still current: restart it, at most twice
            if isaac.phase == "exited" and isaac.restarts < MAX_RESTARTS and served.split()[1:2] == ["run"]:
                isaac.restarts += 1
                note = f"Isaac exited ({isaac.exit_code}); restart {isaac.restarts}/{MAX_RESTARTS}"
                log(note)
                time.sleep(10)
                isaac.start(isaac.seq, isaac.layout, restart=True)
            req = tr.exchange(state_text(isaac, note))
            if req is None:
                if not unreachable_since:
                    unreachable_since = time.time()
                    log("the Orin does not answer; Isaac is left as it is, retrying")
            else:
                if unreachable_since:
                    log(f"the Orin is back after {time.time() - unreachable_since:.0f} s")
                    unreachable_since = 0.0
                line = req.strip().splitlines()[0].strip() if req.strip() else ""
                if line and line != served:
                    parts = line.split()
                    if len(parts) >= 3 and parts[1] == "run":
                        served = line
                        note = f"serving: {line}"
                        isaac.start(parts[0], parts[2])
                    elif len(parts) >= 2 and parts[1] == "stop":
                        served = line
                        note = f"serving: {line}"
                        isaac.stop()
                        isaac.seq, isaac.layout = parts[0], ""
                        log(f"seq {parts[0]}: stop -- Isaac is off, still listening")
                        tr.exchange(state_text(isaac, note))
                        if a.once:
                            return 0
                    else:
                        log(f"ignoring a malformed request: {line!r}")
                        served = line
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        log("Ctrl+C: stopping Isaac and exiting")
        isaac.stop()
        try:
            tr.exchange(state_text(isaac, "follower stopped by hand (Ctrl+C)"))
        except Exception:  # noqa: BLE001
            pass
        return 0


if __name__ == "__main__":
    sys.exit(main())
