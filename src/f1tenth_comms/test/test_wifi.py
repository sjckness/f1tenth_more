"""wifi.py: /proc/net/wireless and `iw` parsing, association, interface choice."""

import os

from f1tenth_comms.wifi import (parse_iw_link, parse_proc_net_wireless, rssi_from_level,
                                WifiReader, wireless_interfaces)

HEADER = ('Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n'
          ' face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n')
# Read on linus (rtw88_8822ce) and on the Orin (rtl88x2ce), 2026-10-21.
LINUS = HEADER + 'wlp4s0: 0000   68.  -42.  -256        0      0      0      0     13        0\n'
ORIN = HEADER + 'wlP1p1s0: 0000   70.  -38.  -256        0      0      0      0      0        0\n'

IW_CONNECTED = """Connected to 64:f6:9d:7b:6b:be (on wlp4s0)
\tSSID: eduroam
\tfreq: 5280.0
\tsignal: -42 dBm
\trx bitrate: 780.0 MBit/s VHT-MCS 9 80MHz VHT-NSS 2
"""


def _proc(tmp_path, text):
    p = tmp_path / 'wireless'
    p.write_text(text)
    return str(p)


def _sys(tmp_path, ifaces):
    root = tmp_path / 'net'
    for name, operstate in ifaces.items():
        d = root / name
        (d / 'wireless').mkdir(parents=True)
        if operstate is not None:
            (d / 'operstate').write_text(operstate + '\n')
    (root / 'eth0').mkdir(parents=True, exist_ok=True)
    return str(root)


def test_parse_the_real_lines():
    assert parse_proc_net_wireless(LINUS) == {'wlp4s0': -42.0}
    assert parse_proc_net_wireless(ORIN) == {'wlP1p1s0': -38.0}
    assert parse_proc_net_wireless(HEADER) == {}


def test_only_negative_levels_are_dbm():
    assert rssi_from_level(-42.0) == -42.0
    assert rssi_from_level(-256.0) is None      # 0 with the dBm flag: no value
    assert rssi_from_level(68.0) is None        # a driver that does not report dBm
    assert rssi_from_level(None) is None


def test_parse_iw_link():
    assert parse_iw_link(IW_CONNECTED) == (True, -42.0, 'eduroam')
    assert parse_iw_link('Not connected.\n') == (False, None, None)


def test_reader_takes_rssi_from_proc_without_running_iw(tmp_path):
    calls = []
    r = WifiReader('wlp4s0', proc_path=_proc(tmp_path, LINUS),
                   sys_class_net=_sys(tmp_path, {'wlp4s0': 'up'}),
                   run_iw=lambda i: calls.append(i) or '')
    s = r.read()
    assert (s.associated, s.rssi_dbm, s.source) == (True, -42.0, 'proc')
    assert calls == []


def test_no_line_in_proc_is_not_associated(tmp_path):
    r = WifiReader('wlp4s0', proc_path=_proc(tmp_path, HEADER),
                   sys_class_net=_sys(tmp_path, {'wlp4s0': None}), use_iw=False)
    s = r.read()
    assert (s.associated, s.rssi_dbm) == (False, None)


def test_operstate_down_wins_over_a_stale_proc_line(tmp_path):
    r = WifiReader('wlp4s0', proc_path=_proc(tmp_path, LINUS),
                   sys_class_net=_sys(tmp_path, {'wlp4s0': 'dormant'}))
    s = r.read()
    assert (s.associated, s.rssi_dbm, s.source) == (False, None, 'operstate')


def test_non_dbm_level_falls_back_to_iw(tmp_path):
    text = HEADER + 'wlan0: 0000   68.  190.  0        0      0      0      0     13        0\n'
    r = WifiReader('wlan0', proc_path=_proc(tmp_path, text),
                   sys_class_net=_sys(tmp_path, {'wlan0': 'up'}),
                   run_iw=lambda i: IW_CONNECTED)
    s = r.read()
    assert (s.associated, s.rssi_dbm, s.source) == (True, -42.0, 'iw')


def test_missing_iw_is_tried_once(tmp_path):
    calls = []

    def no_iw(interface):
        calls.append(interface)
        raise FileNotFoundError('iw')

    r = WifiReader('wlan0', proc_path=str(tmp_path / 'missing'),
                   sys_class_net=_sys(tmp_path, {'wlan0': 'up'}), run_iw=no_iw)
    s1, s2 = r.read(), r.read()
    assert calls == ['wlan0']
    # No proc file and no iw: operstate up says associated, the signal is unknown.
    assert (s1.associated, s1.rssi_dbm) == (True, None)
    assert (s2.associated, s2.rssi_dbm) == (True, None)


def test_interface_is_picked_from_sysfs_then_proc(tmp_path):
    sysdir = _sys(tmp_path, {'wlp4s0': 'up', 'wlan1': 'up'})
    assert wireless_interfaces(sysdir) == ['wlan1', 'wlp4s0']
    assert WifiReader('', sys_class_net=sysdir).interface == 'wlan1'
    empty = str(tmp_path / 'nosys')
    os.makedirs(empty)
    assert WifiReader('', proc_path=_proc(tmp_path, ORIN), sys_class_net=empty).interface \
        == 'wlP1p1s0'
    r = WifiReader('', proc_path=str(tmp_path / 'missing'), sys_class_net=empty)
    assert r.interface == '' and not r.read().associated
