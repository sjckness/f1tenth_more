#!/usr/bin/env python3
r"""
pairing_qr.py: print the pairing link as a QR code in the Orin's terminal.

The iPhone's camera reads it and Safari opens

    https://<orin>.<tailnet>.ts.net/#token=<token>

which the app pairs from on its own (apps/pwa/src/store.ts, tokenFromHash):
it stores the token and strips the fragment from the address bar at once.

WHY A FRAGMENT AND NOT A QUERY STRING. A fragment is not sent with the
request, so the token never reaches the gateway's log, tailscale's, or any
proxy in between -- and it is not in the Referer of anything the page loads.
A `?token=` would be in all of them. This is the same reason the token lives
in an EnvironmentFile and not on a command line.

WHAT THIS PRINTS. The QR, the base URL, and nothing else. The token and the
full link are secrets: they are not printed, not logged and not passed as
arguments. --show-url overrides that for a terminal you trust, and says so.

THE QR ITSELF comes from `qrencode -t ansiutf8` when it is installed, and
otherwise from the `qrcode` package in the gateway's venv (pure Python, no
Pillow needed for text output). Run it with that venv's python:

    ~/.local/share/f1tenth_gateway/venv/bin/python \\
        src/f1tenth_gateway/tools/pairing_qr.py

The ERROR CORRECTION is L and the box is drawn with two half-blocks per
module, as `qrencode -t ansiutf8` does. The link is ~88 characters (a 43-char
token from `secrets.token_urlsafe(32)`), which fits a version-5 symbol, 41x41
modules: 21 terminal rows, read across a desk by an iPhone camera.
"""

import argparse
import os
import shutil
import subprocess
import sys
import urllib.parse

ENV_FILE = '~/.config/f1tenth_gateway/env'
TOKEN_VAR = 'F1T_GATEWAY_TOKEN'


def read_token(env_file: str) -> str:
    """
    Return the token from the environment, else from the gateway's env file.

    The env file is the systemd EnvironmentFile: `NAME=value` lines, no
    `export`, no quoting to speak of. Only the one variable is read, so an
    unrelated line cannot leak into the link.
    """
    value = os.environ.get(TOKEN_VAR, '')
    if not value:
        path = os.path.expanduser(env_file)
        try:
            with open(path) as handle:
                for line in handle:
                    line = line.strip()
                    if line.startswith(TOKEN_VAR + '='):
                        value = line.split('=', 1)[1].strip().strip('\'"')
        except OSError as exc:
            raise SystemExit(f'pairing_qr: cannot read {path}: {exc.strerror}')
    if len(value) < 32:
        raise SystemExit(f'pairing_qr: {TOKEN_VAR} is unset or shorter than 32 characters '
                         f'(set it in {env_file}, chmod 600)')
    return value


def tailnet_url() -> str:
    """Return https://<this host>.<tailnet>.ts.net from tailscale, or '' if it cannot."""
    try:
        out = subprocess.run(['tailscale', 'status', '--json'], capture_output=True,
                             text=True, timeout=10, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return ''
    import json
    try:
        name = json.loads(out)['Self']['DNSName'].rstrip('.')
    except (ValueError, KeyError, TypeError):
        return ''
    return f'https://{name}' if name else ''


def qr_lines(text: str) -> list:
    """Return the QR as terminal lines: qrencode if it is installed, else `qrcode`."""
    if shutil.which('qrencode'):
        try:
            out = subprocess.run(['qrencode', '-t', 'ansiutf8', '-l', 'L', '-m', '2', text],
                                 capture_output=True, text=True, timeout=20, check=True)
            return out.stdout.rstrip('\n').split('\n')
        except (OSError, subprocess.SubprocessError):
            pass                      # fall through to the Python encoder
    try:
        import qrcode
    except ImportError:
        raise SystemExit(
            'pairing_qr: no QR encoder. Either install qrencode, or, in the gateway venv:\n'
            '  ~/.local/share/f1tenth_gateway/venv/bin/pip install qrcode\n'
            "and run this script with that venv's python.")
    code = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_L, border=2)
    code.add_data(text)
    code.make(fit=True)
    return _half_blocks(code.get_matrix())


def _half_blocks(matrix: list) -> list:
    """
    Draw a module matrix two rows at a time, as `qrencode -t ansiutf8` does.

    A module is as wide as a terminal cell and half as tall, so one cell holds
    two rows and the symbol comes out square. True (dark) renders as the
    BACKGROUND, because a terminal's default is light-on-dark as often as not
    and a phone wants dark modules on light quiet zone: the half-blocks are
    printed on a white background with black modules, explicitly.
    """
    glyph = {(False, False): '█',    # both rows light
             (False, True): '▀',     # upper light
             (True, False): '▄',     # lower light
             (True, True): ' '}           # both dark
    lines = []
    for top in range(0, len(matrix), 2):
        upper = matrix[top]
        lower = matrix[top + 1] if top + 1 < len(matrix) else [False] * len(upper)
        row = ''.join(glyph[(bool(u), bool(d))] for u, d in zip(upper, lower))
        lines.append('\x1b[47m\x1b[30m' + row + '\x1b[0m')
    return lines


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base-url', default=None,
                    help='https://<orin>.<tailnet>.ts.net (default: from `tailscale status`)')
    ap.add_argument('--env-file', default=ENV_FILE,
                    help=f"the gateway's EnvironmentFile (default {ENV_FILE})")
    ap.add_argument('--show-url', action='store_true',
                    help='also print the link in full -- it CONTAINS THE TOKEN')
    args = ap.parse_args(argv)

    base = (args.base_url or tailnet_url()).rstrip('/')
    if not base:
        raise SystemExit('pairing_qr: no --base-url and `tailscale status` gave no name')
    token = read_token(args.env_file)
    url = f'{base}/#token={urllib.parse.quote(token, safe="")}'

    print()
    for line in qr_lines(url):
        print('  ' + line)
    print()
    print(f'  {base}/')
    print("  Inquadra il QR con la fotocamera dell'iPhone, apri in Safari,")
    print("  poi Condividi -> Aggiungi alla schermata Home e riapri dall'icona.")
    print("  (L'abbinamento sta nella memoria dell'app aperta dall'icona.)")
    if args.show_url:
        print()
        print('  ' + url)
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
