"""
Voice commands for the gateway: a worker process, started on demand, niced.

TranscriberProcess.transcribe(data, suffix) blocks (run it in a thread):
it starts transcribe_worker the first time, waits for its model, sends the
audio, waits for the text. One transcription at a time. The worker goes
after idle_unload_s without requests, so its model's RAM is held only while
voice is in use. The app refuses a transcription while a mission is active
or Guida is open (decision of 2026-10-20); that is the caller's check.

FakeTranscriber is the mock's: canned text, no model.
"""

import base64
import importlib.util
import itertools
import json
import os
import select
import subprocess
import sys
import threading
import time
from typing import Dict, Optional


# What the app shows instead of a raw ModuleNotFoundError. The Orin's gateway
# venv has no faster-whisper today, so this is the normal answer, not a crash.
NOT_INSTALLED = 'faster-whisper non installato sul gateway'


class TranscriptionError(Exception):
    pass


class TranscriberProcess:
    def __init__(self, cfg: Dict, python: str = sys.executable, env=None):
        self.cfg = cfg
        self.python = python
        self.env = env
        self.proc: Optional[subprocess.Popen] = None
        self.state = 'disabled' if not cfg.get('enabled', True) else 'idle'
        self.reason: Optional[str] = None
        # Say it up front rather than after the app has recorded something and
        # the worker has failed: the model lives in this same interpreter's
        # environment, so find_spec answers for the worker too.
        if self.state == 'idle' and self.python == sys.executable \
                and importlib.util.find_spec('faster_whisper') is None:
            self.state, self.reason = 'unavailable', NOT_INSTALLED
        self.info: Dict = {}
        self.last_used = 0.0
        self._lock = threading.Lock()
        self._ids = itertools.count(1)

    def status(self) -> Dict:
        return {'state': self.state, 'reason': self.reason, 'model': self.cfg['model'],
                **self.info}

    def _start(self) -> None:
        cmd = [self.python, '-m', 'f1tenth_gateway.transcribe_worker',
               '--model', str(self.cfg['model']), '--language', str(self.cfg['language']),
               '--threads', str(self.cfg['cpu_threads']), '--nice', str(self.cfg['nice']),
               '--max-duration', str(self.cfg['max_duration_s'])]
        self.state, self.reason = 'loading', None
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, text=True, bufsize=1,
                                     env=self.env)
        hello = self._read(timeout_s=float(self.cfg['timeout_s']) + 120.0)
        if not hello or not hello.get('ready'):
            self.reason = (hello or {}).get('error') or 'the worker did not start'
            if 'faster_whisper' in str(self.reason):
                self.reason = NOT_INSTALLED
            self.state = 'unavailable'
            self.stop()
            self.state = 'unavailable'
            raise TranscriptionError(self.reason)
        self.info = {k: hello.get(k) for k in ('device', 'compute_type', 'load_s')}
        self.state = 'ready'

    def _read(self, timeout_s: float) -> Optional[Dict]:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if self.proc is None or self.proc.poll() is not None:
                return None
            ready, _, _ = select.select([self.proc.stdout], [], [], 0.2)
            if ready:
                line = self.proc.stdout.readline()
                if not line:
                    return None
                try:
                    return json.loads(line)
                except ValueError:
                    continue
        return None

    def transcribe(self, data: bytes, suffix: str = '.m4a') -> Dict:
        if self.state == 'disabled':
            raise TranscriptionError('trascrizione disattivata (transcription.enabled: false)')
        if len(data) > int(self.cfg['max_bytes']):
            raise TranscriptionError(f'audio troppo grande ({len(data)} byte)')
        with self._lock:
            if self.proc is None or self.proc.poll() is not None:
                self._start()
            rid = next(self._ids)
            req = {'id': rid, 'suffix': suffix,
                   'data_b64': base64.b64encode(data).decode('ascii')}
            try:
                self.proc.stdin.write(json.dumps(req) + '\n')
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError) as exc:
                self.stop()
                raise TranscriptionError(f'worker gone: {exc}') from exc
            res = self._read(timeout_s=float(self.cfg['timeout_s']))
            self.last_used = time.monotonic()
            if res is None:
                self.stop()
                raise TranscriptionError('nessuna risposta dal trascrittore')
            if not res.get('ok'):
                raise TranscriptionError(res.get('error') or 'trascrizione fallita')
            return {k: res.get(k) for k in ('text', 'duration_s', 'transcribe_s')}

    def reap_idle(self, now: Optional[float] = None) -> bool:
        """Stop the worker after idle_unload_s without requests; True if it did."""
        now = time.monotonic() if now is None else now
        if (self.proc is not None and self.state == 'ready'
                and now - self.last_used > float(self.cfg['idle_unload_s'])):
            with self._lock:
                self.stop()
            return True
        return False

    def stop(self) -> None:
        proc, self.proc = self.proc, None
        if self.state not in ('disabled', 'unavailable'):
            self.state = 'idle'
        if proc is None:
            return
        try:
            proc.stdin.close()
        except OSError:
            pass
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


class FakeTranscriber:
    """The mock's: a fixed sentence after a short wait, nothing kept."""

    def __init__(self, cfg: Dict, text: str = 'vai dritto per due metri e poi fermati'):
        self.cfg = cfg
        self.text = text
        self.state = 'ready' if cfg.get('enabled', True) else 'disabled'
        self.calls = 0

    def status(self) -> Dict:
        return {'state': self.state, 'reason': None, 'model': 'fake'}

    def transcribe(self, data: bytes, suffix: str = '.m4a') -> Dict:
        if self.state == 'disabled':
            raise TranscriptionError('trascrizione disattivata (transcription.enabled: false)')
        if len(data) > int(self.cfg['max_bytes']):
            raise TranscriptionError(f'audio troppo grande ({len(data)} byte)')
        self.calls += 1
        time.sleep(0.3)
        return {'text': self.text, 'duration_s': 2.1, 'transcribe_s': 0.3}

    def reap_idle(self, now=None) -> bool:
        return False

    def stop(self) -> None:
        pass


def worker_env(base=None) -> Dict:
    """Return the worker's environment: the gateway's, nothing added."""
    return dict(base if base is not None else os.environ)
