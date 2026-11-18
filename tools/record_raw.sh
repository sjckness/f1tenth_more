#!/usr/bin/env bash
# Raw data recording for hand-pushed clips: ZED SVO2 + a rosbag of LiDAR, odometry, IMUs, TF.
#
#   1. bring the sensors up, with NO command path (motors receive nothing):
#        ros2 launch tools/sensors_only.launch.py
#   2. in another shell:
#        tools/record_raw.sh <clip_name>
#      Ctrl-C stops the SVO and the bag cleanly.
#
# Output: ~/raw_data/<date>_<clip_name>/ with <clip_name>.svo2, bag/ and notes.txt.
set -euo pipefail

clip="${1:?usage: tools/record_raw.sh <clip_name>}"
[[ "$clip" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "clip_name: letters, digits, _ and - only" >&2; exit 2; }
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="${HOME}/raw_data"
out="${root}/$(date +%Y-%m-%dT%H-%M-%S)_${clip}"
mkdir -p "$root"
[[ ! -e "$out" ]] || { echo "$out exists" >&2; exit 2; }

# ZED 2i at HD720 30 fps: raw side-by-side is 2 x 1280 x 720 x 2 B (YUV 4:2:2) = 3.69 MB per
# frame, 6636 MB/min. Stereolabs' average sizes: H265_LOSSLESS ~25% of raw, H265 ~1%; at the
# 60000 kbit/s cap H265 is at most 450 MB/min. The bag adds roughly 15 MB/min.
echo "free space in ${root}: $(df -h --output=avail "$root" | tail -1 | tr -d ' ')"
echo "estimated SVO: H265_LOSSLESS ~1660 MB/min, H265 @60 Mbit/s <=450 MB/min (HD720 30 fps)"

topics=(/scan /tf /tf_static /odometry/filtered /sensors/imu/raw /zed2/zed_node/imu/data
        /zed2/zed_node/depth/camera_info /camera/camera_info)
python3 "$here/zed_svo.py" check "${topics[@]}" || {
    echo "sensors missing: is 'ros2 launch tools/sensors_only.launch.py' up?" >&2; exit 1; }

mkdir -p "$out"
cat > "$out/notes.txt" <<EOF
clip: ${clip}
date: $(date -Iseconds)
git: $(git -C "$here" rev-parse --short HEAD 2>/dev/null || echo unknown)
svo_compression: (filled in below)

distances:
objects:
description:
EOF

mode="$(python3 "$here/zed_svo.py" start "$out/${clip}.svo2")" || { echo "SVO start failed" >&2; exit 1; }
sed -i "s|^svo_compression: .*|svo_compression: ${mode}|" "$out/notes.txt"
echo "SVO recording: ${mode} -> $out/${clip}.svo2"

ros2 bag record -o "$out/bag" --topics "${topics[@]}" &
bag_pid=$!

stopping=0
cleanup() {
    [[ $stopping -eq 1 ]] && return
    stopping=1
    echo "stopping..."
    python3 "$here/zed_svo.py" stop || echo "SVO stop failed: check $out" >&2
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
    echo "done: $out"
    du -sh "$out"/* 2>/dev/null || true
}
trap cleanup INT TERM
wait "$bag_pid" || true
cleanup
