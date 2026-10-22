"""
The transcription worker: faster-whisper in a process of its own, niced.

Started by transcription.TranscriberProcess on the first voice command and
stopped after idle_unload_s, so the model's memory and the CPU it burns are
never the gateway's (decision of 2026-10-20: on the Orin, CPU, a separate
process with nice). It reuses the Telegram bot's Transcriber (temporary
directory always removed, ffmpeg fallback) with a loader of its own: CPU,
int8, a fixed number of threads -- the Orin's PyPI CTranslate2 has no CUDA.

Protocol, one JSON object per line:
    out at start: {"ready": true, "device", "compute_type", "load_s"}
                  or {"ready": false, "error"}
    in:           {"id", "suffix", "data_b64"}
    out:          {"id", "ok": true, "text", "duration_s", "transcribe_s"}
                  or {"id", "ok": false, "error"}
"""

import argparse
import base64
import json
import os
import sys


def _loader(threads):
    def load(cfg):
        from faster_whisper import WhisperModel
        model = WhisperModel(cfg.model, device='cpu', compute_type='int8',
                             cpu_threads=int(threads))
        return model, 'cpu', 'int8'
    return load


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='small')
    ap.add_argument('--language', default='it')
    ap.add_argument('--threads', type=int, default=4)
    ap.add_argument('--nice', type=int, default=10)
    ap.add_argument('--max-duration', type=float, default=30.0)
    args = ap.parse_args(argv)
    if args.nice:
        os.nice(args.nice)

    def say(obj):
        sys.stdout.write(json.dumps(obj) + '\n')
        sys.stdout.flush()

    try:
        from f1tenth_telegram.config import Transcription
        from f1tenth_telegram.transcribe import READY, Transcriber
        cfg = Transcription(enabled=True, model=args.model, device='cpu',
                            language=args.language, max_duration_s=args.max_duration)
        tr = Transcriber(cfg, loader=_loader(args.threads))
        tr.start(background=False)
        if tr.state != READY:
            say({'ready': False, 'error': tr.reason or tr.state})
            return 1
    except Exception as exc:  # noqa: BLE001 - reported to the gateway, not raised
        say({'ready': False, 'error': f'{type(exc).__name__}: {exc}'})
        return 1
    say({'ready': True, 'device': tr.device, 'compute_type': tr.compute_type,
         'load_s': round(tr.load_s or 0.0, 2)})
    for line in sys.stdin:
        req = None
        try:
            req = json.loads(line)
            data = base64.b64decode(req['data_b64'])

            def write(path, data=data):
                with open(path, 'wb') as f:
                    f.write(data)
            res = tr.transcribe_bytes(write, suffix=str(req.get('suffix') or '.m4a'))
            too_long = tr.too_long(res.get('duration_s'))
            if too_long:
                say({'id': req.get('id'), 'ok': False, 'error': too_long})
            else:
                say({'id': req.get('id'), 'ok': True, **res})
        except Exception as exc:  # noqa: BLE001 - one bad request, the worker stays up
            say({'id': req.get('id') if isinstance(req, dict) else None, 'ok': False,
                 'error': f'{type(exc).__name__}: {exc}'})
    return 0


if __name__ == '__main__':
    sys.exit(main())
