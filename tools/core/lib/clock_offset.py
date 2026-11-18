#!/usr/bin/env python3
"""Differenza di orologio tra questa macchina e un host remoto, via ssh (solo stdlib).

    clock_offset.py [--samples 9] [--warn-ms 5] [--fail-ms 50] -- ssh [opzioni] utente@host

Apre UNA connessione ssh in cui il remoto risponde con time.time_ns() a ogni riga ricevuta, e per
ogni campione misura t0 (invio) e t1 (risposta) in locale: offset = remoto - (t0 + t1) / 2, come
NTP. Tiene il campione con l'RTT minimo; l'incertezza e' RTT/2. Stampa una riga
"offset +1.2 ms (+-0.4 ms, rtt 0.8 ms)"; exit 0 sotto --warn-ms, 10 sotto --fail-ms, 1 altrimenti
o se ssh fallisce.
"""
import argparse
import subprocess
import sys
import time

REMOTE = ("python3 -u -c 'import sys,time\n"
          "for _ in sys.stdin:\n"
          "    sys.stdout.write(\"%d\\n\" % time.time_ns()); sys.stdout.flush()'")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=9)
    ap.add_argument('--warn-ms', type=float, default=5.0)
    ap.add_argument('--fail-ms', type=float, default=50.0)
    ap.add_argument('ssh', nargs=argparse.REMAINDER)
    a = ap.parse_args()
    cmd = [x for x in a.ssh if x != '--']
    if not cmd:
        print('manca il comando ssh dopo --')
        return 1
    try:
        p = subprocess.Popen(cmd + [REMOTE], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True, bufsize=1)
    except OSError as e:
        print(f'ssh non avviato: {e}')
        return 1
    best = None
    try:
        for _ in range(a.samples):
            t0 = time.time_ns()
            p.stdin.write('x\n')
            p.stdin.flush()
            line = p.stdout.readline()
            t1 = time.time_ns()
            if not line.strip():
                break
            rtt = t1 - t0
            off = int(line) - (t0 + t1) // 2
            if best is None or rtt < best[0]:
                best = (rtt, off)
    except (BrokenPipeError, ValueError):
        best = None
    finally:
        try:
            p.stdin.close()
        except OSError:
            pass
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    if best is None:
        err = (p.stderr.read() or '').strip().splitlines()
        print('nessuna risposta dal remoto' + (f': {err[-1]}' if err else ''))
        return 1
    rtt_ms, off_ms = best[0] / 1e6, best[1] / 1e6
    print(f'offset {off_ms:+.1f} ms (+-{rtt_ms / 2:.1f} ms, rtt {rtt_ms:.1f} ms)')
    if abs(off_ms) <= a.warn_ms:
        return 0
    return 10 if abs(off_ms) <= a.fail_ms else 1


if __name__ == '__main__':
    sys.exit(main())
