"""
Tests for model_log.ModelLogWriter, the CSV behind tools/mpc_model_check.py.

These defend the rules whose violation corrupts the offline analysis without
any visible error: the exact header the script parses, rows readable before
the file is closed (a crash must not lose the buffer), no dt = 0 row, and a
logger that can never raise into the control loop.
"""

import csv
import re
from pathlib import Path

from mpc_controller.model_log import ModelLogWriter

_SCRIPT = Path(__file__).resolve().parents[4] / 'tools' / 'mpc_model_check.py'


def _rows(path):
    with open(path, newline='') as f:
        return list(csv.reader(f))


def test_header_is_the_one_the_analysis_script_parses():
    """The script's own demo log header is the format it reads."""
    header = re.search(r'header="([^"]+)"', _SCRIPT.read_text()).group(1)
    assert ','.join(ModelLogWriter.HEADER) == header


def test_rows_are_on_disk_before_close(tmp_path):
    """Each row is flushed, so it survives a crash."""
    path = tmp_path / 'log.csv'
    log = ModelLogWriter(str(path))
    assert log.write(0.1, 1.0, 2.0, 0.5, 1.5, 0.12, -0.3)
    rows = _rows(path)
    assert rows[0] == list(ModelLogWriter.HEADER)
    assert [float(v) for v in rows[1]] == [0.1, 1.0, 2.0, 0.5, 1.5, 0.12, -0.3]
    log.close()


def test_values_keep_full_precision(tmp_path):
    """No rounding: timing jitter is one of the things the analysis measures."""
    path = tmp_path / 'log.csv'
    log = ModelLogWriter(str(path))
    t = 1234.123456789012
    log.write(t, 0.1 + 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)
    log.close()
    row = _rows(path)[1]
    assert float(row[0]) == t
    assert float(row[1]) == 0.1 + 0.2


def test_a_row_that_does_not_advance_t_is_skipped(tmp_path):
    """A repeated odometry stamp would be dt = 0 in the analysis."""
    path = tmp_path / 'log.csv'
    log = ModelLogWriter(str(path))
    assert log.write(1.0, 0, 0, 0, 0, 0, 0)
    assert not log.write(1.0, 9, 9, 9, 9, 9, 9)
    assert not log.write(0.9, 9, 9, 9, 9, 9, 9)
    assert log.write(1.2, 1, 1, 1, 1, 1, 1)
    log.close()
    assert log.skipped == 2
    assert [float(r[0]) for r in _rows(path)[1:]] == [1.0, 1.2]


def test_an_empty_path_disables_logging(tmp_path, monkeypatch):
    """'' writes nothing and creates no file."""
    monkeypatch.chdir(tmp_path)
    log = ModelLogWriter('')
    assert not log.enabled
    assert not log.write(1.0, 0, 0, 0, 0, 0, 0)
    assert list(tmp_path.iterdir()) == []


def test_an_unopenable_path_disables_logging_instead_of_raising(tmp_path):
    """The controller must keep running when the log cannot be written."""
    log = ModelLogWriter(str(tmp_path / 'missing_dir' / 'log.csv'))
    assert not log.enabled
    assert isinstance(log.error, OSError)
    assert not log.write(1.0, 0, 0, 0, 0, 0, 0)


def test_a_relative_path_lands_in_the_working_directory(tmp_path, monkeypatch):
    """The default mpc_log.csv resolves against the process cwd."""
    monkeypatch.chdir(tmp_path)
    log = ModelLogWriter('mpc_log.csv')
    log.close()
    assert (tmp_path / 'mpc_log.csv').exists()
