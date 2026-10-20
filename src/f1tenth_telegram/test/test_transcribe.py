"""Voice: the model's states, the duration limit, the temporary files (mocked model)."""

import os
from types import SimpleNamespace

from f1tenth_telegram import config, transcribe
import pytest


class FakeModel:
    def __init__(self, text='vai dritto per due metri', duration=2.2, fail=False):
        self.text, self.duration, self.fail = text, duration, fail
        self.paths = []

    def transcribe(self, path, language=None, beam_size=5):
        self.paths.append((path, os.path.exists(path), language))
        if self.fail:
            raise ValueError('cannot decode')
        segs = [SimpleNamespace(text=' vai dritto\n'), SimpleNamespace(text=' per due metri ')]
        return iter(segs), SimpleNamespace(duration=self.duration)


def make(model=None, **cfg):
    t = transcribe.Transcriber(config.Transcription(**cfg),
                               loader=lambda c: (model or FakeModel(), 'cpu', 'int8'))
    return t


def test_ready_after_loading():
    t = make()
    assert t.unavailable_message() == 'the speech model is still loading, retry in a few seconds'
    t.start(background=False)
    assert t.state == transcribe.READY and t.unavailable_message() is None
    assert t.device == 'cpu' and t.compute_type == 'int8'


def test_disabled_and_unavailable():
    assert 'disabled' in make(enabled=False).unavailable_message()

    def broken(cfg):
        raise ImportError('No module named faster_whisper')
    t = transcribe.Transcriber(config.Transcription(), loader=broken)
    t.start(background=False)
    assert t.state == transcribe.UNAVAILABLE and 'faster_whisper' in t.reason
    assert t.unavailable_message() == 'transcription unavailable on this bot'


def test_duration_limit():
    t = make(max_duration_s=60)
    assert t.too_long(59.9) is None and t.too_long(None) is None
    assert t.too_long(75) == 'the audio is 75 s long, at most 60 s'


def test_transcript_and_language():
    model = FakeModel()
    t = make(model)
    t.start(background=False)
    res = t.transcribe_bytes(lambda p: open(p, 'wb').write(b'OggS'))
    assert res['text'] == 'vai dritto per due metri' and res['duration_s'] == 2.2
    assert model.paths[0][1] and model.paths[0][2] == 'it'
    t.cfg.language = 'auto'
    t.transcribe_bytes(lambda p: open(p, 'wb').write(b'OggS'))
    assert model.paths[1][2] is None


def test_the_audio_is_always_deleted():
    model = FakeModel(fail=True)
    t = make(model)
    t.start(background=False)
    seen = []

    def write(p):
        seen.append(p)
        open(p, 'wb').write(b'not audio')
    with pytest.raises(Exception):
        t.transcribe_bytes(write)
    assert seen and not os.path.exists(os.path.dirname(seen[0]))
    model.fail = False
    t.transcribe_bytes(write)
    assert not os.path.exists(os.path.dirname(seen[-1]))


def test_config_section():
    cfg = config.from_dict({'transcription': {'model': 'small', 'device': 'cpu',
                                              'language': 'auto', 'max_duration_s': 30}})
    assert cfg.transcription.model == 'small' and cfg.transcription.max_duration_s == 30.0
    assert config.from_dict({}).transcription.enabled is True
    with pytest.raises(config.ConfigError, match='device'):
        config.from_dict({'transcription': {'device': 'tpu'}})
    with pytest.raises(config.ConfigError, match='unknown keys'):
        config.from_dict({'transcription': {'modle': 'small'}})
