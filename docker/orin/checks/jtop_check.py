#!/usr/bin/env python3
"""
Check jtop (jetson-stats) from inside the container (backlog M3).

The GPU numbers system_observer_node publishes come through /run/jtop.sock.

Runs on: inside the container (`docker exec f1tenth_car python3 ...` from
the host, or the harness container), and the Orin host for comparison.
Plan P8 "jtop (M3)". diag_start_check.sh then shows system_observer_node's
/diagnostics/system_status with non-zero GPU fields.

  jtop_check.py [--seconds 3] [--out FILE.json]

Pass (exit 0): `from jtop import jtop` imports, the service answers on its
socket within --seconds, and a GPU load and temperature are read.
"""
import argparse
import json
import os
import sys
import time

SOCKET = '/run/jtop.sock'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    res = {'socket': SOCKET, 'socket_exists': os.path.exists(SOCKET)}
    try:
        from jtop import jtop
        res['import'] = True
    except ImportError as exc:
        res['import'] = False
        res['error'] = 'import: %s' % exc
        jtop = None
    if jtop is not None:
        try:
            with jtop() as j:
                t_end = time.time() + a.seconds
                while j.ok() and time.time() < t_end:
                    stats = dict(j.stats)
                    if stats:
                        break
                gpu = stats.get('GPU')
                temps = {k: v for k, v in stats.items() if 'Temp' in str(k)}
                res.update({'connected': True, 'gpu_load': gpu, 'temps': temps,
                            'nvpmodel': str(stats.get('nvp model'))})
        except Exception as exc:  # noqa: BLE001 - JtopException and socket errors
            res['connected'] = False
            res['error'] = '%s: %s' % (type(exc).__name__, exc)
    ok = bool(res.get('connected')) and res.get('gpu_load') is not None
    res['result'] = 'PASS' if ok else 'FAIL'
    text = json.dumps(res, indent=1, default=str)
    print(text)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
