"""The Run tab's archive, the map PNG and the configuration."""

import os
import struct
import time
import zlib

from f1tenth_gateway import config, mapimg
from f1tenth_gateway.runs import Archive
import pytest

HERE = os.path.dirname(__file__)
ARCHIVE = Archive(os.path.join(HERE, 'fixtures', 'archive'),
                  os.path.join(HERE, 'fixtures', 'workspace'))


# ---- runs -------------------------------------------------------------------------------------

def test_newest_first_with_the_outcome_badges():
    runs = ARCHIVE.list_runs(10)
    assert [r['badge'] for r in runs] == ['INCOMPLETA', 'OK', 'ABORT', 'E-STOP', 'OK']
    assert runs[0]['finalized'] is False and runs[1]['duration_s'] == 84.0
    assert runs[1]['mission_id'] == 'llm_mock0001'


def test_video_only_when_an_mp4_exists():
    runs = {r['id']: r for r in ARCHIVE.list_runs(10)}
    assert runs['2026-10-19T17-02-00_mission-wall_turn']['video']['ok']
    no = runs['2026-10-20T12-32-11_mission-llm_mock0001']['video']
    assert not no['ok'] and 'mission_render' in no['why']


def test_the_report_has_the_manifest_the_moves_the_scoring_and_the_snapshots(monkeypatch):
    monkeypatch.setenv('TZ', 'Europe/Rome')
    time.tzset()
    page = ARCHIVE.report_html('2026-10-20T12-32-11_mission-llm_mock0001')
    assert 'COMPLETE' in page and 'straight_1' in page and 'object_reached' in page
    assert 'Esito per move' in page and 'distance_reached' in page
    assert 'src="snapshots/0"' in page


@pytest.mark.parametrize('bad', ['../etc', '2026-10-20T12-32-11_mission-x/../../x', '', 'x'])
def test_odd_ids_are_not_runs(bad):
    assert ARCHIVE.run_dir(bad) is None and ARCHIVE.report_html(bad) is None


def test_the_report_escapes_what_it_shows(tmp_path):
    rid = '2026-10-20T10-00-00_mission-x'
    d = tmp_path / 'complete' / rid
    d.mkdir(parents=True)
    (d / f'{rid}.manifest.json').write_text('{"mission_id": "<script>alert(1)</script>"}')
    page = Archive(str(tmp_path), str(tmp_path)).report_html(rid)
    assert '<script>alert' not in page and '&lt;script&gt;' in page


# ---- map --------------------------------------------------------------------------------------

def _chunks(png):
    assert png[:8] == b'\x89PNG\r\n\x1a\n'
    i, out = 8, {}
    while i < len(png):
        n = struct.unpack('>I', png[i:i + 4])[0]
        kind = png[i + 4:i + 8]
        out.setdefault(kind, []).append(png[i + 8:i + 8 + n])
        i += 12 + n
    return out


def test_the_map_png_is_gray_alpha_flipped_and_reduced():
    w, h = 4, 2
    data = [100, 0, -1, 0,      # row 0 = lowest y
            0, 0, 0, 0]
    img = mapimg.grid_to_png(w, h, data)
    c = _chunks(img['png'])
    width, height, depth, ctype = struct.unpack('>IIBB', c[b'IHDR'][0][:10])
    assert (width, height, depth, ctype) == (4, 2, 8, 4)
    raw = zlib.decompress(c[b'IDAT'][0])
    top, bottom = raw[1:9], raw[10:18]                       # each row: filter byte + 4 x GA
    assert bottom[0:2] == bytes([0xD8, 0xFF])                # occupied, now at the bottom
    assert bottom[4:6] == bytes([0, 0])                      # unknown: transparent
    assert top[0:2] == bytes([0x20, 0xFF])                   # free
    big = mapimg.grid_to_png(2000, 1000, [0] * 2_000_000, max_px=800)
    assert big['factor'] == 3 and (big['width'], big['height']) == (667, 334)


# ---- config -----------------------------------------------------------------------------------

def test_unknown_keys_are_an_error():
    with pytest.raises(config.ConfigError, match='unknown key'):
        config.load(None, {'teleop': {'max_sped_mps': 1.0}})


def test_the_token_comes_from_the_environment_and_is_long():
    cfg = config.load(None)
    with pytest.raises(config.ConfigError):
        config.token(cfg, {'F1T_GATEWAY_TOKEN': 'short'})
    assert config.token(cfg, {'F1T_GATEWAY_TOKEN': 'x' * 32}) == 'x' * 32


def test_the_default_port_is_not_foxglove_s_and_is_a_parameter():
    cfg = config.load(None)
    assert cfg['listen'] == {'host': '127.0.0.1', 'port': 8770}
    assert config.load(None, {'listen': {'port': 8771}})['listen']['port'] == 8771
    with pytest.raises(config.ConfigError):
        config.load(None, {'listen': {'port': 70000}})


def test_relative_paths_are_from_the_config_file():
    cfg = config.load(os.path.join(HERE, '..', 'config', 'gateway_mock.yaml'))
    assert cfg['archive']['root'] == os.path.normpath(os.path.join(HERE, 'fixtures', 'archive'))
    assert cfg['mock'] is True and cfg['auth']['cookie_secure'] is False
