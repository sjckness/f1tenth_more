"""
mission_debug_video.py tests: debug JPEGs out of a sqlite3 bag, into an MP4.

No ROS: the bag is built here with the sqlite3 schema rosbag2 writes and with
CompressedImage messages CDR-encoded by hand. The encoder below was checked
byte for byte against rclpy.serialization.serialize_message on a Jazzy host.
"""

import shutil
import sqlite3
import struct
import subprocess

from f1tenth_logger import mission_debug_video as mdv
import pytest

# A real JPEG (16x16 grey, quality 50, written by cv2.imencode) so ffmpeg can
# decode it; the test itself needs no OpenCV.
_JPEG = bytes.fromhex(
    'ffd8ffe000104a46494600010100000100010000ffdb004300100b0c0e0c0a100e0d0e1211101318'
    '281a181616183123251d283a333d3c3933383740485c4e404457453738506d51575f626768673e4d'
    '71797064785c656763ffdb0043011112121815182f1a1a2f63423842636363636363636363636363'
    '6363636363636363636363636363636363636363636363636363636363636363636363636363ffc0'
    '0011080010001003012200021101031101ffc4001f00000105010101010101000000000000000001'
    '02030405060708090a0bffc400b5100002010303020403050504040000017d010203000411051221'
    '31410613516107227114328191a1082342b1c11552d1f02433627282090a161718191a2526272829'
    '2a3435363738393a434445464748494a535455565758595a636465666768696a737475767778797a'
    '838485868788898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6'
    'c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9faffc4001f010003'
    '0101010101010101010000000000000102030405060708090a0bffc400b511000201020404030407'
    '05040400010277000102031104052131061241510761711322328108144291a1b1c109233352f015'
    '6272d10a162434e125f11718191a262728292a35363738393a434445464748494a53545556575859'
    '5a636465666768696a737475767778797a82838485868788898a92939495969798999aa2a3a4a5a6'
    'a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2e3e4e5e6e7e8e9ea'
    'f2f3f4f5f6f7f8f9faffda000c03010002110311003f0028a28a00ffd9')


def _cdr_compressed_image(sec, nanosec, frame_id, fmt, data):
    body = bytearray()

    def align(n):
        while len(body) % n:
            body.append(0)

    def string(text):
        raw = text.encode() + b'\0'
        align(4)
        body.extend(struct.pack('<I', len(raw)) + raw)

    align(4)
    body.extend(struct.pack('<iI', sec, nanosec))
    string(frame_id)
    string(fmt)
    align(4)
    body.extend(struct.pack('<I', len(data)) + data)
    return b'\x00\x01\x00\x00' + bytes(body)


def _bag(tmp_path, stamps, topic=mdv.DEBUG_IMAGE_TOPIC):
    run = tmp_path / '2026-10-01T09-33-02_mission-test'
    (run / 'bag').mkdir(parents=True)
    con = sqlite3.connect(run / 'bag' / 'bag_0.db3')
    con.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, '
                'type TEXT NOT NULL, serialization_format TEXT NOT NULL, '
                'offered_qos_profiles TEXT NOT NULL)')
    con.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
                'timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
    con.execute("INSERT INTO topics VALUES (1, ?, 'sensor_msgs/msg/CompressedImage', "
                "'cdr', '')", (topic,))
    for stamp in stamps:
        sec = int(stamp)
        nanosec = int(round((stamp - sec) * 1e9))
        con.execute('INSERT INTO messages(topic_id, timestamp, data) VALUES (1, ?, ?)',
                    (int(stamp * 1e9) + 450_000_000,
                     _cdr_compressed_image(sec, nanosec, 'cam', 'jpeg', _JPEG)))
    con.commit()
    con.close()
    return run


def test_decoder_reads_stamp_format_and_bytes():
    raw = _cdr_compressed_image(1790847190, 123456789, 'zed2_left', 'jpeg', b'\x01\x02\x03')
    stamp, fmt, data = mdv.decode_compressed_image(raw)
    assert stamp == pytest.approx(1790847190.123456789)
    assert fmt == 'jpeg'
    assert data == b'\x01\x02\x03'


def test_decoder_rejects_a_truncated_message():
    raw = _cdr_compressed_image(1, 0, 'cam', 'jpeg', b'\x01\x02\x03')
    with pytest.raises(ValueError):
        mdv.decode_compressed_image(raw[:-2])


def test_frames_come_back_sorted_by_capture_stamp(tmp_path):
    run = _bag(tmp_path, [10.4, 10.0, 10.2])
    frames = mdv.read_frames(mdv.find_db3(run))
    assert [round(s, 3) for s, _ in frames] == [10.0, 10.2, 10.4]
    assert frames[0][1] == _JPEG


def test_other_topics_are_ignored(tmp_path):
    run = _bag(tmp_path, [1.0, 2.0], topic='/camera/image_annotated')
    assert mdv.read_frames(mdv.find_db3(run)) == []


def test_durations_follow_the_stamps_and_a_gap_is_clamped():
    assert mdv.frame_durations([0.0, 0.2, 0.4, 10.0]) == pytest.approx([0.2, 0.2, 2.0, 0.2])


def test_run_id_is_the_run_folder_from_any_entry_point(tmp_path):
    run = _bag(tmp_path, [1.0])
    expected = '2026-10-01T09-33-02_mission-test'
    assert mdv.run_id_of(run) == expected
    assert mdv.run_id_of(run / 'bag') == expected
    assert mdv.run_id_of(run / 'bag' / 'bag_0.db3') == expected


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='needs ffmpeg')
def test_mp4_lasts_as_long_as_the_recorded_frames(tmp_path):
    stamps = [100.0 + 0.2 * i for i in range(25)]          # 5 Hz for 5 s
    run = _bag(tmp_path, stamps)
    assert mdv.main([str(run), '--out-dir', str(tmp_path / 'out')]) == 0
    out = tmp_path / 'out' / '2026-10-01T09-33-02_mission-test.debug.mp4'
    probe = subprocess.run(
        ['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
         '-of', 'default=nw=1:nk=1', str(out)],
        check=True, capture_output=True, text=True)
    assert float(probe.stdout) == pytest.approx(5.0, abs=0.25)


def test_render_run_without_a_bag_returns_none(tmp_path):
    assert mdv.render_run(tmp_path / 'no_such_run', tmp_path / 'out') is None


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='needs ffmpeg')
def test_render_run_names_the_mp4_after_the_run_folder(tmp_path):
    run = _bag(tmp_path, [100.0, 100.2, 100.4])
    out = mdv.render_run(run, tmp_path / 'out')
    assert out == tmp_path / 'out' / '2026-10-01T09-33-02_mission-test.debug.mp4'
    assert out.stat().st_size > 0


def test_module_imports_nothing_from_ros():
    # Same rule as mission_render (test_extract_render_split.py): this runs
    # on a host with no ROS at all.
    import ast
    tree = ast.parse(open(mdv.__file__).read())
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    assert not roots & {'rclpy', 'rosbag2_py', 'rosidl_runtime_py', 'sensor_msgs',
                        'std_msgs', 'cv_bridge', 'cv2'}
