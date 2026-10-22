"""
Voice messages to text, on linus, with faster-whisper.

The model is loaded once, in a background thread started with the bot, so
start-up is not blocked; until it is ready a voice message is answered
"model loading". Transcription is blocking work and is always run off the
asyncio loop (asyncio.to_thread), so 🛑 stays responsive meanwhile.

device auto: CUDA float16 when ctranslate2 sees a GPU and a short warm-up on
it works (cuBLAS / cuDNN come from the venv's nvidia-* wheels, through
LD_LIBRARY_PATH: see the systemd unit), otherwise CPU int8. If
faster-whisper is missing or the model cannot load, the bot runs on without
voice: messages are answered "transcription unavailable", logged once.

Audio: Telegram voice notes are OGG/Opus. faster-whisper decodes through
PyAV; if that fails, ffmpeg converts to WAV and the WAV is transcribed. The
audio never outlives the call: it is written to a temporary directory that
is always removed.
"""

import logging
import os
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Callable, Optional

from .config import Transcription

log = logging.getLogger('f1tenth_telegram')

DISABLED, LOADING, READY, UNAVAILABLE = 'disabled', 'loading', 'ready', 'unavailable'


class Transcriber:
    def __init__(self, cfg: Transcription, loader: Callable = None):
        self.cfg = cfg
        self.state = DISABLED if not cfg.enabled else LOADING
        self.reason = None
        self.device = None
        self.compute_type = None
        self.load_s = None
        self._model = None
        self._loader = loader or _load_model
        self._lock = threading.Lock()      # one transcription at a time
        self._warned = False

    # ---- loading ------------------------------------------------------------

    def start(self, background: bool = True) -> None:
        if self.state in (DISABLED, READY):
            return
        if background:
            threading.Thread(target=self._load, name='whisper-load', daemon=True).start()
        else:
            self._load()

    def _load(self) -> None:
        t0 = time.monotonic()
        try:
            self._model, self.device, self.compute_type = self._loader(self.cfg)
        except Exception as exc:  # noqa: BLE001 - any failure: run on without voice
            self.state, self.reason = UNAVAILABLE, f'{type(exc).__name__}: {exc}'
            log.warning('transcription unavailable: %s', self.reason)
            self._warned = True
            return
        self.load_s = time.monotonic() - t0
        self.state = READY
        log.info('transcription ready: model %s on %s (%s), loaded in %.1f s',
                 self.cfg.model, self.device, self.compute_type, self.load_s)

    def unavailable_message(self) -> Optional[str]:
        """Return why a voice message cannot be transcribed now, or None."""
        if self.state == READY:
            return None
        if self.state == DISABLED:
            return 'voice commands are disabled (transcription.enabled: false)'
        if self.state == LOADING:
            return 'the speech model is still loading, retry in a few seconds'
        if not self._warned:
            log.warning('transcription unavailable: %s', self.reason)
            self._warned = True
        return 'transcription unavailable on this bot'

    # ---- transcribing ---------------------------------------------------------

    def too_long(self, duration_s: Optional[float]) -> Optional[str]:
        if duration_s is not None and duration_s > self.cfg.max_duration_s:
            return (f'the audio is {duration_s:.0f} s long, at most '
                    f'{self.cfg.max_duration_s:.0f} s')
        return None

    def transcribe_file(self, path: str) -> dict:
        """Transcribe one audio file (blocking: run it in an executor)."""
        language = None if self.cfg.language in ('auto', '') else self.cfg.language
        with self._lock:
            t0 = time.monotonic()
            try:
                text, duration = self._run(path, language)
            except Exception as exc:  # noqa: BLE001 - PyAV could not decode it
                if not shutil.which('ffmpeg'):
                    raise
                log.info('PyAV could not decode the audio (%s): trying ffmpeg', exc)
                wav = path + '.wav'
                subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', path, '-ac', '1',
                                '-ar', '16000', wav], check=True, timeout=60)
                text, duration = self._run(wav, language)
            return {'text': text, 'duration_s': duration,
                    'transcribe_s': round(time.monotonic() - t0, 2)}

    def _run(self, path: str, language: Optional[str]):
        segments, info = self._model.transcribe(path, language=language, beam_size=5)
        text = ' '.join(seg.text.strip() for seg in segments)
        return ' '.join(text.split()), round(float(info.duration), 1)

    def transcribe_bytes(self, write: Callable[[str], None], suffix: str = '.ogg') -> dict:
        """
        Let `write(path)` put the audio in a fresh temporary directory, transcribe it.

        The directory and everything in it are removed afterwards, whatever
        happens: the audio is never kept.
        """
        tmp = tempfile.mkdtemp(prefix='f1t_voice_')
        try:
            path = os.path.join(tmp, 'audio' + suffix)
            write(path)
            return self.transcribe_file(path)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


def _load_model(cfg: Transcription):
    """Load faster-whisper: CUDA float16 if it works, else CPU int8."""
    from faster_whisper import WhisperModel
    import numpy as np

    tries = []
    if cfg.device in ('auto', 'cuda'):
        tries.append(('cuda', 'float16'))
    if cfg.device in ('auto', 'cpu'):
        tries.append(('cpu', 'int8'))
    last = None
    for device, compute in tries:
        try:
            if device == 'cuda':
                import ctranslate2
                if ctranslate2.get_cuda_device_count() < 1:
                    raise RuntimeError('no CUDA device')
            model = WhisperModel(cfg.model, device=device, compute_type=compute)
            # cuBLAS/cuDNN load on first use: a warm-up proves the device works.
            segments, _ = model.transcribe(np.zeros(16000, dtype=np.float32), language='it')
            list(segments)
            return model, device, compute
        except Exception as exc:  # noqa: BLE001 - try the next device
            last = exc
            log.info('whisper on %s/%s failed: %s', device, compute, exc)
    raise last or RuntimeError('no device to try')
