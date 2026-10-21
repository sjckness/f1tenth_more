"""
The Wi-Fi signal of one interface, without root and without NetworkManager.

Two sources, in this order:
- /proc/net/wireless. One line per associated interface; the "level" column
  is the signal in dBm when the driver reports dBm (printed negative, e.g.
  "-42."). No subprocess: this is what runs every second. Seen on linus
  (rtw88_8822ce) and on the Orin (rtl88x2ce) on 2026-10-21.
- `iw dev <if> link` ("signal: -42 dBm" / "Not connected."), only when the
  proc file is missing or its level is not in dBm. linus has no `iw`; the
  Orin has /usr/sbin/iw.

Association: /sys/class/net/<if>/operstate is 'up' only while associated
(the driver raises the carrier at association). Without that file the
proc line decides: cfg80211 prints no line for an interface with no
current BSS.
"""

from dataclasses import dataclass
import os
import subprocess
from typing import Callable, List, Optional

PROC_NET_WIRELESS = '/proc/net/wireless'
SYS_CLASS_NET = '/sys/class/net'

# A level outside this range is not a usable dBm reading (-256 is "0 with the
# dBm flag": no value).
RSSI_MIN_DBM = -120.0
RSSI_MAX_DBM = -1.0

_DOWN_OPERSTATES = ('down', 'dormant', 'lowerlayerdown', 'notpresent')


@dataclass
class WifiSample:
    interface: str
    associated: bool
    rssi_dbm: Optional[float]   # None: not associated, or no dBm reading
    source: str                 # 'proc', 'iw', 'operstate', 'none'


def parse_proc_net_wireless(text: str) -> dict:
    """Return {interface: level} for every line of /proc/net/wireless; level as printed."""
    out = {}
    for line in text.splitlines()[2:]:
        if ':' not in line:
            continue
        name, rest = line.split(':', 1)
        fields = rest.split()
        if len(fields) < 3:
            continue
        try:
            out[name.strip()] = float(fields[2].rstrip('.'))
        except ValueError:
            continue
    return out


def rssi_from_level(level: Optional[float]) -> Optional[float]:
    """Return the level as dBm if it is a dBm reading, else None."""
    if level is None or not (RSSI_MIN_DBM <= level <= RSSI_MAX_DBM):
        return None
    return level


def parse_iw_link(text: str):
    """Return (associated, rssi_dbm, ssid) from `iw dev <if> link`."""
    if 'Not connected' in text:
        return False, None, None
    associated = text.lstrip().startswith('Connected to')
    rssi = ssid = None
    for line in text.splitlines():
        line = line.strip()
        if line.startswith('signal:'):
            try:
                rssi = rssi_from_level(float(line.split()[1]))
            except (IndexError, ValueError):
                pass
        elif line.startswith('SSID:'):
            ssid = line[len('SSID:'):].strip()
    return associated, rssi, ssid


def wireless_interfaces(sys_class_net: str = SYS_CLASS_NET) -> List[str]:
    """Return the wireless network interfaces, sorted (the ones with a wireless/ dir)."""
    try:
        names = sorted(os.listdir(sys_class_net))
    except OSError:
        return []
    return [n for n in names if os.path.isdir(os.path.join(sys_class_net, n, 'wireless'))]


def _run_iw(interface: str) -> str:
    return subprocess.run(['iw', 'dev', interface, 'link'], capture_output=True, text=True,
                          timeout=1.0, check=False).stdout


class WifiReader:
    """Read association and RSSI of one interface; '' picks the first wireless one."""

    def __init__(self, interface: str = '', proc_path: str = PROC_NET_WIRELESS,
                 sys_class_net: str = SYS_CLASS_NET, use_iw: bool = True,
                 run_iw: Callable[[str], str] = _run_iw):
        self.proc_path = proc_path
        self.sys_class_net = sys_class_net
        self.use_iw = use_iw
        self._run_iw = run_iw
        self.interface = interface or self._first_interface()

    def _first_interface(self) -> str:
        found = wireless_interfaces(self.sys_class_net)
        if found:
            return found[0]
        # No sysfs entry (a test, a container): the first line of the proc file.
        try:
            with open(self.proc_path) as f:
                names = list(parse_proc_net_wireless(f.read()))
        except OSError:
            names = []
        return names[0] if names else ''

    def _operstate(self) -> Optional[str]:
        try:
            with open(os.path.join(self.sys_class_net, self.interface, 'operstate')) as f:
                return f.read().strip()
        except OSError:
            return None

    def read(self) -> WifiSample:
        if not self.interface:
            return WifiSample('', False, None, 'none')
        operstate = self._operstate()
        if operstate in _DOWN_OPERSTATES:
            return WifiSample(self.interface, False, None, 'operstate')
        try:
            with open(self.proc_path) as f:
                levels = parse_proc_net_wireless(f.read())
        except OSError:
            levels = None
        if levels is not None and self.interface not in levels:
            return WifiSample(self.interface, False, None, 'proc')
        rssi = rssi_from_level(levels.get(self.interface)) if levels else None
        if rssi is not None:
            return WifiSample(self.interface, True, rssi, 'proc')
        if self.use_iw:
            try:
                associated, rssi, _ = parse_iw_link(self._run_iw(self.interface))
                return WifiSample(self.interface, associated, rssi, 'iw')
            except (OSError, subprocess.SubprocessError):
                self.use_iw = False     # not installed, or hanging: do not try again
        # In the proc file but no dBm reading, or no proc file, and no iw: the
        # signal is unknown; associated if either source says so.
        if levels is not None:
            return WifiSample(self.interface, True, None, 'proc')
        return WifiSample(self.interface, operstate == 'up', None, 'operstate')
