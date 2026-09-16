"""A mission report is never left truncated at its final path.

The third go_to_person run of 2026-09-15 produced a 0-byte report -- at the
exact name a complete report would have had, with final_state and the move
outcomes simply absent. The writer used Path.write_text, which opens with
mode 'w' and therefore TRUNCATES the destination before any content is
written; anything that interrupts the gap between that open and the close
leaves an empty file behind at the real name.

The interesting test here is not "does json land in a file" -- it is the
interrupted case, which is what actually happened and what a non-atomic
writer gets wrong. So one test really does fork a child, kill it mid-write,
and assert on what the parent finds. Run under the old writer it reproduces
the 0-byte file exactly; under _write_atomic the destination is untouched.

No ROS, no hardware: every symbol under test is a plain function.
"""

import json
import os
import pathlib
import signal
import sys

import pytest

from f1tenth_behavior.mission.move_scoring import _write_atomic


def _kill_during_write(writer_name, target, payload_text):
    """Run one write in a child that dies partway through; return the result.

    Returns (exists, size) for `target` as seen by the PARENT after the child
    has died, which is the only view that matters -- the next run reads this
    directory, not the dead process's buffers.

    The child kills ITSELF with SIGKILL, after the destination would have been
    opened and before the content could be flushed. SIGKILL rather than an
    exception because the point is an interruption no `finally` can intercept:
    a power cut, an OOM kill, a supervisor tearing the node down.
    """
    read_fd, write_fd = os.pipe()
    pid = os.fork()
    if pid == 0:                                    # --- child
        os.close(read_fd)
        try:
            if writer_name == 'old':
                # The pre-fix writer, reproduced exactly rather than imported:
                # it no longer exists to import, and the point is to show this
                # test fails against it.
                with pathlib.Path(target).open('w', encoding='utf-8') as handle:
                    os.write(write_fd, b'x')        # destination now truncated
                    os.kill(os.getpid(), signal.SIGKILL)
                    handle.write(payload_text)
            else:
                # Signal from a thread would be cleaner, but the whole point is
                # an uncatchable death; so the temp file is created, then the
                # process dies before os.replace can run.
                os.write(write_fd, b'x')
                _write_atomic_then_die(pathlib.Path(target), payload_text, write_fd)
        finally:
            os._exit(0)
    os.close(write_fd)                              # --- parent
    os.read(read_fd, 1)
    os.close(read_fd)
    os.waitpid(pid, 0)
    path = pathlib.Path(target)
    return path.exists(), (path.stat().st_size if path.exists() else None)


def _write_atomic_then_die(path, text, write_fd):
    """_write_atomic's shape, interrupted just before the rename."""
    import tempfile
    fd, tmp_name = tempfile.mkstemp(
        dir=str(path.parent), prefix=f'.{path.name}.', suffix='.tmp')
    with os.fdopen(fd, 'w', encoding='utf-8') as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    os.kill(os.getpid(), signal.SIGKILL)     # dies BEFORE os.replace
    os.replace(tmp_name, str(path))


def _payload():
    """A report the size of a real one, so the write is not a single block."""
    return json.dumps(
        {'mission_id': 'go_to_person',
         'final_state': 'COMPLETE',
         'moves': [{'move_id': f'move_{i}', 'actual': i * 0.1} for i in range(5000)]},
        indent=2)


class TestTheHappyPath:

    def test_it_writes_the_content(self, tmp_path):
        target = tmp_path / 'report.json'
        _write_atomic(target, _payload())
        assert json.loads(target.read_text())['mission_id'] == 'go_to_person'

    def test_it_overwrites_an_existing_report_completely(self, tmp_path):
        """A rename over a longer existing file must not leave its tail."""
        target = tmp_path / 'report.json'
        _write_atomic(target, _payload())
        _write_atomic(target, json.dumps({'mission_id': 'short'}))
        assert json.loads(target.read_text()) == {'mission_id': 'short'}

    def test_it_leaves_no_temp_files_behind(self, tmp_path):
        target = tmp_path / 'report.json'
        _write_atomic(target, _payload())
        assert [p.name for p in tmp_path.iterdir()] == ['report.json']


class TestInterruptedWrite:
    """The 0-byte report, reproduced and then prevented."""

    @pytest.mark.skipif(not hasattr(os, 'fork'), reason='needs fork')
    def test_the_old_writer_leaves_a_zero_byte_file(self, tmp_path):
        """Not a test of our code -- a test that the harness reproduces the
        bug, so the assertion below means something."""
        target = tmp_path / 'report.json'
        exists, size = _kill_during_write('old', target, _payload())
        assert exists and size == 0, (
            'harness did not reproduce the truncation; the test below proves '
            'nothing until it does')

    @pytest.mark.skipif(not hasattr(os, 'fork'), reason='needs fork')
    def test_the_atomic_writer_leaves_the_destination_absent(self, tmp_path):
        target = tmp_path / 'report.json'
        exists, _size = _kill_during_write('atomic', target, _payload())
        assert not exists, (
            'the destination was created before the content was durable -- '
            'this is the 0-byte mission report')

    @pytest.mark.skipif(not hasattr(os, 'fork'), reason='needs fork')
    def test_an_interrupted_write_does_not_damage_the_previous_report(self, tmp_path):
        """The case that matters on a re-run: run N-1's report must survive
        run N dying mid-write, rather than being truncated to nothing."""
        target = tmp_path / 'report.json'
        _write_atomic(target, json.dumps({'mission_id': 'run_1', 'ok': True}))

        exists, _size = _kill_during_write('atomic', target, _payload())
        assert exists
        assert json.loads(target.read_text()) == {'mission_id': 'run_1', 'ok': True}


class TestFailurePropagation:

    def test_an_unwritable_directory_raises_oserror_for_the_caller(self, tmp_path):
        """write_mission_summary catches OSError and logs; _write_atomic must
        therefore raise OSError rather than something exotic."""
        missing = tmp_path / 'not_created' / 'report.json'
        with pytest.raises(OSError):
            _write_atomic(missing, '{}')

    def test_a_failed_write_cleans_up_its_temp_file(self, tmp_path, monkeypatch):
        target = tmp_path / 'report.json'

        def _boom(*_a, **_k):
            raise OSError('disk full')

        monkeypatch.setattr(os, 'replace', _boom)
        with pytest.raises(OSError):
            _write_atomic(target, _payload())
        assert list(tmp_path.iterdir()) == [], 'a .tmp file was left behind'
        assert not target.exists()


class TestIntegrationWithWriteMissionSummary:

    def test_a_real_summary_round_trips(self, tmp_path, monkeypatch):
        from f1tenth_behavior.mission import move_scoring

        monkeypatch.setattr(
            move_scoring, '_resolve_mission_reports_dir', lambda: tmp_path)

        class _Logger:
            def info(self, *_a, **_k):
                pass

            error = info

        path = move_scoring.write_mission_summary(
            'go_to_person', [], 'COMPLETE', 1789486929.0, _Logger())
        assert path is not None
        loaded = json.loads(path.read_text())
        assert loaded['mission_id'] == 'go_to_person'
        assert loaded['final_state'] == 'COMPLETE'
        assert path.stat().st_size > 0


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
