"""
Coverage for the manifest -> runs.db index.

The invariant under test throughout: runs.db is a CACHE. Everything in it is
derivable from the manifests, name and notes included, so deleting the
database and re-importing must lose nothing. See runs_db's own module
docstring.

Deliberately builds synthetic manifests rather than reading the real ones
under ~/.ros/mission_bags: these must pass on a machine that has never
recorded a mission, and must not depend on run data that can be archived away.
"""

import json
import os

import pytest

from f1tenth_logger import runs_db


def _manifest(tmp_path, run_id, **overrides):
    """Write one manifest shaped exactly like mission_logger_node's own."""
    data = {
        'run_id': run_id,
        'mission_id': 'bottle_then_person',
        'mission_json_path': '/ws/src/f1tenth_behavior/missions/bottle_then_person.json',
        'git': {'branch': 'scene-graph', 'commit': 'abc1234', 'dirty': True},
        'start_time': '2026-09-02T15:27:43.314491+00:00',
        'end_time': '2026-09-02T15:27:57.367944+00:00',
        'outcome': 'ABORTED',
        'bag_path': str(tmp_path / 'bags' / run_id),
        'params_snapshot_path': str(tmp_path / f'{run_id}.params.yaml'),
        'storage_id': 'sqlite3',
    }
    data.update(overrides)
    path = tmp_path / f'{run_id}.manifest.json'
    path.write_text(json.dumps(data, indent=2))
    return str(path)


@pytest.fixture
def env(tmp_path):
    runs_dir = tmp_path / 'runs'
    manifests = tmp_path / 'manifests'
    manifests.mkdir()
    return manifests, str(runs_dir)


class TestImport:
    def test_imports_every_manifest_and_ignores_other_files(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')
        _manifest(manifests, 'run_b')
        (manifests / 'run_a.params.yaml').write_text('not a manifest\n')
        conn = runs_db.connect(runs_dir)
        imported, failed = runs_db.import_dir(conn, str(manifests), runs_dir)
        assert len(imported) == 2
        assert failed == []

    def test_is_idempotent(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        for _ in range(3):
            runs_db.import_dir(conn, str(manifests), runs_dir)
        assert len(runs_db.all_runs(conn)) == 1

    def test_a_corrupt_manifest_is_skipped_not_fatal(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')
        (manifests / 'broken.manifest.json').write_text('{not json')
        conn = runs_db.connect(runs_dir)
        imported, failed = runs_db.import_dir(conn, str(manifests), runs_dir)
        assert len(imported) == 1
        assert len(failed) == 1

    def test_name_defaults_to_run_id(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        assert runs_db.find(conn, 'run_a')['name'] == 'run_a'

    def test_git_dirty_is_preserved(self, env):
        # A commit hash alone names the wrong tree when the working tree was
        # dirty, which is why the manifest carries the flag at all.
        manifests, runs_dir = env
        _manifest(manifests, 'clean_run', git={'branch': 'b', 'commit': 'c',
                                               'dirty': False})
        _manifest(manifests, 'dirty_run')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        assert runs_db.find(conn, 'clean_run')['git_dirty'] == 0
        assert runs_db.find(conn, 'dirty_run')['git_dirty'] == 1


class TestDuration:
    def test_falls_back_to_recorder_start_and_end(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        assert runs_db.find(conn, 'run_a')['duration_s'] == pytest.approx(14.053, abs=1e-3)

    def test_mission_markers_win_over_recorder_span(self, env):
        # Continuous recording means the bag spans pre-roll + mission +
        # post-roll; duration_s must report the MISSION, not the recording.
        manifests, runs_dir = env
        _manifest(manifests, 'run_a',
                  mission_start_time='2026-09-02T15:27:48.000000+00:00',
                  mission_end_time='2026-09-02T15:27:53.000000+00:00')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        assert runs_db.find(conn, 'run_a')['duration_s'] == pytest.approx(5.0)

    def test_an_unfinalized_run_has_no_duration_rather_than_zero(self, env):
        # A recorder that died mid-run leaves end_time null. "Unknown" must
        # stay distinguishable from "instantaneous".
        manifests, runs_dir = env
        _manifest(manifests, 'run_a', end_time=None, outcome=None)
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        row = runs_db.find(conn, 'run_a')
        assert row['duration_s'] is None
        assert row['outcome'] == 'UNKNOWN'


class TestBagIndependence:
    """The archive machine holds manifests and usually no bags at all."""

    def test_import_and_lookup_work_with_no_bag_on_disk(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')          # bags/ is never created
        conn = runs_db.connect(runs_dir)
        imported, failed = runs_db.import_dir(conn, str(manifests), runs_dir)
        assert failed == []
        assert runs_db.find(conn, 'run_a') is not None

    def test_bag_status_reports_absent_then_present(self, env):
        manifests, runs_dir = env
        path = _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        row = runs_db.find(conn, 'run_a')
        assert runs_db.bag_status(row)[0] == 'absent'

        os.makedirs(json.load(open(path))['bag_path'])
        # Recomputed live, NOT re-imported: bag presence is a property of this
        # machine right now, so a stale row must not be able to lie about it.
        assert runs_db.bag_status(runs_db.find(conn, 'run_a'))[0] == 'present'

    def test_bag_status_is_unknown_when_the_manifest_names_no_bag(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a', bag_path=None)
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        assert runs_db.bag_status(runs_db.find(conn, 'run_a'))[0] == 'unknown'


class TestNotesAndRenameWriteThroughToTheManifest:
    def test_a_note_lands_in_the_manifest_not_only_the_row(self, env):
        manifests, runs_dir = env
        path = _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        runs_db.append_note(conn, runs_db.find(conn, 'run_a'), 'clipped it', runs_dir)
        notes = json.load(open(path))['notes']
        assert len(notes) == 1
        assert notes[0]['text'] == 'clipped it'
        assert notes[0]['timestamp']

    def test_notes_append_rather_than_replace(self, env):
        manifests, runs_dir = env
        path = _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        for text in ('first', 'second', 'third'):
            runs_db.append_note(conn, runs_db.find(conn, 'run_a'), text, runs_dir)
        assert [n['text'] for n in json.load(open(path))['notes']] == \
            ['first', 'second', 'third']

    def test_rename_sets_the_manifest_name_and_leaves_run_id_alone(self, env):
        manifests, runs_dir = env
        path = _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        runs_db.rename(conn, runs_db.find(conn, 'run_a'), 'good-one', runs_dir)
        manifest = json.load(open(path))
        assert manifest['name'] == 'good-one'
        assert manifest['run_id'] == 'run_a'

    def test_rename_refuses_a_name_another_run_already_uses(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')
        _manifest(manifests, 'run_b')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        runs_db.rename(conn, runs_db.find(conn, 'run_a'), 'taken', runs_dir)
        with pytest.raises(ValueError):
            runs_db.rename(conn, runs_db.find(conn, 'run_b'), 'taken', runs_dir)

    def test_a_run_is_findable_by_run_id_or_by_name(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        runs_db.rename(conn, runs_db.find(conn, 'run_a'), 'good-one', runs_dir)
        assert runs_db.find(conn, 'run_a')['run_id'] == 'run_a'
        assert runs_db.find(conn, 'good-one')['run_id'] == 'run_a'
        assert runs_db.find(conn, 'nope') is None


class TestTheDatabaseIsOnlyACache:
    def test_deleting_the_db_and_reimporting_loses_no_name_or_notes(self, env):
        manifests, runs_dir = env
        _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        runs_db.rename(conn, runs_db.find(conn, 'run_a'), 'good-one', runs_dir)
        runs_db.append_note(conn, runs_db.find(conn, 'run_a'), 'clipped it', runs_dir)
        conn.close()

        os.remove(os.path.join(runs_dir, 'runs.db'))
        rebuilt = runs_db.connect(runs_dir)
        runs_db.import_dir(rebuilt, str(manifests), runs_dir)

        row = runs_db.find(rebuilt, 'good-one')
        assert row is not None
        assert row['run_id'] == 'run_a'
        assert 'clipped it' in row['notes']

    def test_the_manifest_wins_over_a_stale_row(self, env):
        # One direction only: manifest -> db, never the reverse.
        manifests, runs_dir = env
        path = _manifest(manifests, 'run_a')
        conn = runs_db.connect(runs_dir)
        runs_db.import_dir(conn, str(manifests), runs_dir)
        conn.execute("UPDATE runs SET outcome = 'INVENTED' WHERE run_id = 'run_a'")
        conn.commit()
        runs_db.sync_manifest(conn, path, runs_dir)
        assert runs_db.find(conn, 'run_a')['outcome'] == 'ABORTED'


class TestManifestWritesAreAtomic:
    def test_a_write_leaves_no_temp_file_behind(self, env):
        manifests, runs_dir = env
        path = _manifest(manifests, 'run_a')
        runs_db.write_manifest_fields(path, {'name': 'x'})
        assert not os.path.exists(f'{path}.tmp')
        assert json.load(open(path))['name'] == 'x'
