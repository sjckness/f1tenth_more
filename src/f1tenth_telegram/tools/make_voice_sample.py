#!/usr/bin/env python3
"""
Make a spoken test command offline: libespeak-ng (ctypes) -> WAV -> OGG/Opus.

For checking the voice path without a phone. Telegram voice notes are
OGG/Opus, so that is what this writes (through ffmpeg's libopus).

    python3 tools/make_voice_sample.py "vai dritto per due metri" /tmp/cmd.ogg [--voice it]
"""

import argparse
import ctypes
import os
import subprocess
import sys
import tempfile
import wave

AUDIO_OUTPUT_SYNCH_PLAYBACK = 2
AUDIO_OUTPUT_RETRIEVAL = 1
POS_CHARACTER = 1
ESPEAK_CHARS_UTF8 = 1


def synthesize(text: str, voice: str) -> tuple:
    lib = ctypes.CDLL('libespeak-ng.so.1')
    rate = lib.espeak_Initialize(AUDIO_OUTPUT_RETRIEVAL, 0, None, 0)
    if rate <= 0:
        raise RuntimeError('espeak_Initialize failed')
    samples = []
    callback_t = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_short),
                                  ctypes.c_int, ctypes.c_void_p)

    def on_audio(wav, n, _events):
        if wav and n > 0:
            samples.append(ctypes.string_at(wav, n * 2))
        return 0

    cb = callback_t(on_audio)
    lib.espeak_SetSynthCallback(cb)
    if lib.espeak_SetVoiceByName(voice.encode()) != 0:
        raise RuntimeError(f'no espeak-ng voice {voice!r}')
    data = text.encode()
    lib.espeak_Synth(data, len(data) + 1, 0, POS_CHARACTER, 0, ESPEAK_CHARS_UTF8, None, None)
    lib.espeak_Synchronize()
    return rate, b''.join(samples)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('text')
    p.add_argument('out', help='.ogg (Opus) output')
    p.add_argument('--voice', default='it')
    args = p.parse_args(argv)
    rate, pcm = synthesize(args.text, args.voice)
    with tempfile.TemporaryDirectory() as tmp:
        wav = os.path.join(tmp, 'speech.wav')
        with wave.open(wav, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes(pcm)
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', wav, '-c:a', 'libopus',
                        '-b:a', '24k', '-ar', '48000', args.out], check=True)
    print(f'{args.out}: {len(pcm) / 2 / rate:.1f} s of speech, voice {args.voice}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
