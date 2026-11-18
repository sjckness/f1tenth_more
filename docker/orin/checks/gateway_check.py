#!/usr/bin/env python3
"""
The gateway as the phone app reaches it: token, REST, WebSocket, run reports.

Runs on: linus (over Tailscale: https://<orin>.<tailnet>.ts.net, the
`tailscale serve` 443 -> 127.0.0.1:8770 of the Orin) or the Orin host
(http://127.0.0.1:8770). Stdlib only (urllib, ssl, a minimal RFC 6455
client), so it needs nothing installed. Plan P8: "gateway over Tailscale
(token, websocket, runs with image provenance)"; run it against the native
gateway and the container's for the same answers. gateway_crash.sh covers
"a gateway crash leaves the stack running".

  gateway_check.py URL --token-file FILE | --token T
                   [--expect-commit SHA] [--insecure] [--out FILE.json]

Checks:
  auth-refused   GET /api/status without a token -> 401
  status         GET /api/status with `Authorization: Bearer` -> 200, a JSON object
  ws-hello       /ws, first message {"type": "auth", "token": ...} -> a "hello"
  ws-bad-token   /ws with a wrong token -> closed with 4401
  runs           GET /api/runs -> 200, a JSON list
  provenance     (--expect-commit) the newest finalized run's report names the
                 commit (the image's SOURCES commit, P4.4 `source: image`)
The token file is the gateway's env file (F1T_GATEWAY_TOKEN=...). The token
is never printed. Exit 0 when every check passes. Each run spends two
refused attempts (no token, wrong token) of the gateway's per-minute budget
(auth.max_failures_per_min, 5 by default): run it at most twice a minute, or
the gateway answers 429 / 4429 to everyone, the phone included, for a minute.
"""
import argparse
import base64
import json
import os
import socket
import ssl
import struct
import sys
import urllib.error
import urllib.parse
import urllib.request


def read_token(path):
    with open(os.path.expanduser(path)) as f:
        for line in f:
            line = line.strip()
            if line.startswith('export '):
                line = line[7:]
            if line.startswith('F1T_GATEWAY_TOKEN='):
                return line.split('=', 1)[1].strip().strip('"\'')
    raise SystemExit('no F1T_GATEWAY_TOKEN in %s' % path)


# ---- minimal WebSocket client (RFC 6455) ----------------------------------

def build_frame(payload, opcode=1, mask_key=None):
    """One final client frame (masked, as a client must)."""
    mask_key = os.urandom(4) if mask_key is None else mask_key
    n = len(payload)
    head = bytes([0x80 | opcode])
    if n < 126:
        head += bytes([0x80 | n])
    elif n < 65536:
        head += bytes([0x80 | 126]) + struct.pack('>H', n)
    else:
        head += bytes([0x80 | 127]) + struct.pack('>Q', n)
    masked = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
    return head + mask_key + masked


def parse_frame(read):
    """(opcode, payload) of the next frame; `read(n)` returns exactly n bytes."""
    b0, b1 = read(2)
    opcode = b0 & 0x0F
    n = b1 & 0x7F
    if n == 126:
        n = struct.unpack('>H', read(2))[0]
    elif n == 127:
        n = struct.unpack('>Q', read(8))[0]
    mask = read(4) if b1 & 0x80 else None
    payload = read(n) if n else b''
    if mask:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return opcode, payload


class WsClient:
    def __init__(self, url, timeout=10.0, insecure=False):
        u = urllib.parse.urlparse(url)
        secure = u.scheme in ('https', 'wss')
        port = u.port or (443 if secure else 80)
        raw = socket.create_connection((u.hostname, port), timeout=timeout)
        if secure:
            ctx = ssl.create_default_context()
            if insecure:
                ctx.check_hostname = False
                ctx.verify_mode = ssl.CERT_NONE
            raw = ctx.wrap_socket(raw, server_hostname=u.hostname)
        self.sock = raw
        key = base64.b64encode(os.urandom(16)).decode()
        req = ('GET /ws HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
               'Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n' % (u.netloc, key))
        self.sock.sendall(req.encode())
        head = b''
        while b'\r\n\r\n' not in head:
            chunk = self.sock.recv(1)
            if not chunk:
                raise ConnectionError('closed during the handshake')
            head += chunk
        status = head.split(b'\r\n', 1)[0].decode(errors='replace')
        if ' 101 ' not in status + ' ':
            raise ConnectionError('handshake refused: %s' % status)

    def _read(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('closed')
            buf += chunk
        return buf

    def send_json(self, obj):
        self.sock.sendall(build_frame(json.dumps(obj).encode()))

    def recv(self):
        """('text', str) or ('close', code); pings answered, others skipped."""
        while True:
            op, payload = parse_frame(self._read)
            if op == 1:
                return 'text', payload.decode(errors='replace')
            if op == 8:
                code = struct.unpack('>H', payload[:2])[0] if len(payload) >= 2 else None
                return 'close', code
            if op == 9:
                self.sock.sendall(build_frame(payload, opcode=10))

    def close(self):
        try:
            self.sock.sendall(build_frame(struct.pack('>H', 1000), opcode=8))
        except OSError:
            pass
        self.sock.close()


# ---- checks ---------------------------------------------------------------

def http(url, token=None, insecure=False, timeout=10.0):
    req = urllib.request.Request(url)
    if token:
        req.add_header('Authorization', 'Bearer ' + token)
    ctx = None
    if insecure and url.startswith('https'):
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
    try:
        with urllib.request.urlopen(req, timeout=timeout, context=ctx) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def run_checks(base, token, expect_commit=None, insecure=False):
    res = {}
    code, _ = http(base + '/api/status', insecure=insecure)
    res['auth-refused'] = (code == 401, 'HTTP %s without a token' % code)
    code, body = http(base + '/api/status', token, insecure)
    try:
        ok = code == 200 and isinstance(json.loads(body), dict)
    except ValueError:
        ok = False
    res['status'] = (ok, 'HTTP %s, %d bytes' % (code, len(body)))
    try:
        ws = WsClient(base, insecure=insecure)
        ws.send_json({'type': 'auth', 'token': token})
        kind, val = ws.recv()
        hello = kind == 'text' and json.loads(val).get('type') == 'hello'
        res['ws-hello'] = (hello, '%s %s' % (kind, (val[:80] if kind == 'text' else val)))
        ws.close()
    except (OSError, ConnectionError, ValueError) as exc:
        res['ws-hello'] = (False, repr(exc))
    try:
        ws = WsClient(base, insecure=insecure)
        ws.send_json({'type': 'auth', 'token': 'not-the-token'})
        kind, val = ws.recv()
        res['ws-bad-token'] = (kind == 'close' and val == 4401, '%s %s' % (kind, val))
        ws.sock.close()
    except (OSError, ConnectionError, ValueError) as exc:
        res['ws-bad-token'] = (False, repr(exc))
    code, body = http(base + '/api/runs', token, insecure)
    try:
        runs = json.loads(body)
    except ValueError:
        runs = None
    if isinstance(runs, dict):
        runs = runs.get('runs', runs)
    res['runs'] = (code == 200 and isinstance(runs, list),
                   'HTTP %s, %s runs' % (code, len(runs) if isinstance(runs, list) else '?'))
    if expect_commit:
        done = [r for r in (runs or []) if isinstance(r, dict) and r.get('finalized')]
        if not done:
            res['provenance'] = (False, 'no finalized run to read')
        else:
            code, body = http('%s/api/runs/%s/report' % (base, done[0]['id']), token, insecure)
            text = body.decode(errors='replace')
            res['provenance'] = (code == 200 and expect_commit[:10] in text,
                                 'run %s report: HTTP %s, commit %s %s' % (
                                     done[0]['id'], code, expect_commit[:10],
                                     'found' if expect_commit[:10] in text else 'NOT found'))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  gateway_check.py'):])
    ap.add_argument('url', help='https://<orin>.<tailnet>.ts.net or http://127.0.0.1:8770')
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--token-file')
    g.add_argument('--token')
    ap.add_argument('--expect-commit')
    ap.add_argument('--insecure', action='store_true', help='skip TLS verification')
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    token = a.token or read_token(a.token_file)
    res = run_checks(a.url.rstrip('/'), token, a.expect_commit, a.insecure)
    for name, (ok, detail) in res.items():
        print('%s gateway %s: %s' % ('PASS' if ok else 'FAIL', name, detail))
    if a.out:
        with open(a.out, 'w') as f:
            json.dump({k: {'ok': v[0], 'detail': v[1]} for k, v in res.items()}, f, indent=1)
    return 0 if all(ok for ok, _ in res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
