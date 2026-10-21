#!/usr/bin/env python3
"""
The base's side of the f1tenth_comms heartbeat: a UDP echo for the car's link_manager.

Every packet that looks like a heartbeat (starts with b'F1HB', 16..64 bytes)
goes back to its sender byte for byte; anything else is dropped. A reply is
never larger than its request, so the port cannot amplify anything.

    heartbeat_responder.py [--port 47800] [--bind ADDR ...] [--log-every 60]

--bind defaults to 0.0.0.0: every address of this PC, the Tailscale and the
hotspot ones included, also those that come up after it started. With
explicit addresses each one is bound with IP_FREEBIND, so an address that
does not exist yet (hotspot down) is fine.

Every --log-every seconds (0: never) one line: how many heartbeats were
echoed, per sender address and per path the car meant (byte 5 of the
packet: 0 tailscale, 1 local). `journalctl --user -u f1tenth-heartbeat -f`
then shows whether the car is reaching this base, and how.

Standard library only (Python 3.8+): it runs on linus or the tpad without
ROS. scripts/base/install_heartbeat.sh installs it as a systemd user unit.
"""

import argparse
import selectors
import socket
import sys
import time
from typing import Dict, List, Optional, Tuple

MAGIC = b'F1HB'
MIN_LEN, MAX_LEN = 16, 64
PATH_NAMES = {0: 'tailscale', 1: 'local'}
IP_FREEBIND = getattr(socket, 'IP_FREEBIND', 15)


def reply_for(data: bytes) -> Optional[bytes]:
    """Return the echo for one datagram, or None when it is not a heartbeat."""
    if MIN_LEN <= len(data) <= MAX_LEN and data[:4] == MAGIC:
        return data
    return None


def open_sockets(binds: List[str], port: int) -> List[socket.socket]:
    socks = []
    for addr in binds:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if addr not in ('', '0.0.0.0'):
            s.setsockopt(socket.IPPROTO_IP, IP_FREEBIND, 1)
        s.bind((addr, port))
        s.setblocking(False)
        socks.append(s)
    return socks


def serve(socks: List[socket.socket], log_every: float = 60.0, stop=None,
          out=sys.stdout, reply=reply_for) -> Dict[str, int]:
    """Echo until stop() is true (or forever); return the per-sender counts of the last period."""
    sel = selectors.DefaultSelector()
    for s in socks:
        sel.register(s, selectors.EVENT_READ)
    counts: Dict[Tuple[str, str], int] = {}     # (sender, path) -> echoed
    dropped = 0
    t_log = time.monotonic()
    try:
        while stop is None or not stop():
            for key, _ in sel.select(timeout=0.5):
                sock = key.fileobj
                while True:
                    try:
                        data, addr = sock.recvfrom(256)
                    except (BlockingIOError, InterruptedError):
                        break
                    answer = reply(data)
                    if answer is None:
                        dropped += 1
                        continue
                    try:
                        sock.sendto(answer, addr)
                    except OSError:
                        dropped += 1
                        continue
                    k = (addr[0], PATH_NAMES.get(data[5], str(data[5])))
                    counts[k] = counts.get(k, 0) + 1
            now = time.monotonic()
            if log_every > 0 and now - t_log >= log_every:
                total = sum(counts.values())
                who = ', '.join(f'{src} {path} {n}' for (src, path), n in sorted(counts.items()))
                print(f'heartbeat: {total} echoed in {now - t_log:.0f} s'
                      f'{": " + who if who else " (no car)"}'
                      f'{f", {dropped} dropped" if dropped else ""}', file=out, flush=True)
                counts.clear()
                dropped = 0
                t_log = now
    finally:
        sel.close()
    return {f'{src} {path}': n for (src, path), n in counts.items()}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--port', type=int, default=47800)
    ap.add_argument('--bind', action='append', default=None,
                    help='address to listen on (repeatable); default 0.0.0.0')
    ap.add_argument('--log-every', type=float, default=60.0)
    args = ap.parse_args(argv)
    binds = args.bind or ['0.0.0.0']
    socks = open_sockets(binds, args.port)
    print(f'heartbeat_responder: UDP {", ".join(binds)} port {args.port}', flush=True)
    try:
        serve(socks, log_every=args.log_every)
    except KeyboardInterrupt:
        pass
    finally:
        for s in socks:
            s.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
