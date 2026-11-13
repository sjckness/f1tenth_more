#!/usr/bin/env bash
# hydra_run.sh -- one documented Hydra run against the Isaac sim: drive, record, save, summarise.
#
#   scripts/hydra/hydra_run.sh [--out DIR] [--segments route.json] [--no-drive] [--tcp]
#                              [--mask-period S] [--graph-period S]
#
# Start the sim FIRST, on Windows:  scripts\sim\win\hil_sim_wsl.bat
# (it turns the GT semantic labels on, which seg_source:=gt needs), then run this in the WSL distro.
#
# WHAT IT DOES, in order:
#   1. renders this side's Fast DDS profile (the addresses change at every WSL restart)
#   2. launches Hydra with hydra_cost:=true, so the 1 Hz CPU/GPU/topic/graph CSV comes for free
#   3. starts `ros2 bag record` on the mask, the RGB, the pose and the graph topics
#   4. runs hydra_run_record.py: /sim/reset, then the validated route, with the guards
#   5. stops everything in the right order and renders the graph to PNG
#
# Nothing goes in /tmp: this distro wipes it, and the first run lost its graphs that way. The run
# folder lives under ~/f1tenth_sim_archive (ext4), never on /mnt/f (a 9p mount, far too slow for a
# bag).
#
# The route is NOT improvised: sim/isaac/tools/check_drive_path.py validates it against the
# world's collision geometry first, and this script refuses to drive if that check fails.
#
# That check needs the world's MESHES, which live where the world was converted: the
# `src/husarion_gz_worlds` submodule plus the Fuel cache of 20 downloaded meshes. A WSL clone has
# neither, and there the grid comes out empty. So validate on the machine that has the world:
#   Windows:  python sim\isaac\tools\check_drive_path.py --json path_check.json
#   here:     scripts/hydra/hydra_run.sh --path-check /mnt/.../path_check.json
# The json is verified to be `ok` AND to describe the same segments as the route being driven.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"          # scripts/hydra -> scripts -> repo root

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/f1tenth_sim_archive/office_hydra_$STAMP"
SEGMENTS=""
EXTRA=()
PROFILE_FLAGS=()
MASK_PERIOD=1.0
GRAPH_PERIOD=10.0
SKIP_CHECK=false
PATH_CHECK=""

while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --segments) SEGMENTS="$2"; shift 2 ;;
    --mask-period) MASK_PERIOD="$2"; shift 2 ;;
    --graph-period) GRAPH_PERIOD="$2"; shift 2 ;;
    --no-drive) EXTRA+=(--no-drive); shift ;;
    --tcp) PROFILE_FLAGS+=(--tcp); shift ;;
    --path-check) PATH_CHECK="$2"; shift 2 ;;
    --skip-path-check) SKIP_CHECK=true; shift ;;
    -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
    *) echo "unknown option $1 (--help)" >&2; exit 2 ;;
  esac
done

mkdir -p "$OUT"
echo "===== run $STAMP -> $OUT"

# ---- 1. the route, checked before anything is started ------------------------------------
ROUTE="${SEGMENTS:-$REPO/sim/isaac/config/routes/office_corridor_loop.json}"
if $SKIP_CHECK; then
  echo "--- route NOT validated (--skip-path-check)"
elif [ -n "$PATH_CHECK" ]; then
  echo "--- using the validation from $PATH_CHECK"
  cp "$PATH_CHECK" "$OUT/path_check.json" 2>/dev/null || true
  python3 - "$PATH_CHECK" "$ROUTE" <<'PY' || exit 1
import json, sys
chk = json.load(open(sys.argv[1], encoding='utf-8'))
r = json.load(open(sys.argv[2], encoding='utf-8'))
want = r.get('segments', r) if isinstance(r, dict) else r
if not chk.get('ok'):
    sys.exit(f"{sys.argv[1]}: the route was REJECTED there (worst {chk.get('worst')})")
norm = lambda ss: [(s['name'], float(s['speed']), float(s['steer']), float(s['dur'])) for s in ss]
if norm(chk.get('segments', [])) != norm(want):
    sys.exit(f"{sys.argv[1]} validated a DIFFERENT route than {sys.argv[2]}: revalidate")
print(f"    ok: {chk['path_length_m']:.2f} m, worst clearance {chk['worst']['clear']:.2f} m, "
      f"margin {chk['margin_m']} m, same {len(want)} segments")
PY
else
  echo "--- validating the route against the world geometry"
  CACHE="$HOME/.cache/f1tenth/office_occ.npz"
  mkdir -p "$(dirname "$CACHE")"
  ARGS=(--cache "$CACHE" --margin 0.20 --json "$OUT/path_check.json" --segments "$ROUTE")
  if ! python3 "$REPO/sim/isaac/tools/check_drive_path.py" "${ARGS[@]}"; then
    echo "the route was not cleared. Fix the segments, or validate where the world meshes are" >&2
    echo "and pass --path-check FILE, or --skip-path-check to drive it anyway." >&2
    exit 1
  fi
fi

# ---- 2. environment ----------------------------------------------------------------------
set +u                                    # ROS setup files read unset variables
# shellcheck disable=SC1091
source "$REPO/scripts/hydra/hydra_env.sh"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
unset ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
FASTRTPS_DEFAULT_PROFILES_FILE="$("$HERE/wsl_dds_profile.sh" "${PROFILE_FLAGS[@]}" --show)"
export FASTRTPS_DEFAULT_PROFILES_FILE
ros2 daemon stop >/dev/null 2>&1 || true

echo "--- is the sim there?"
if ! timeout 30 ros2 topic list 2>/dev/null | grep -qx /clock; then
  echo "no /clock: start the sim on Windows (scripts\\sim\\win\\hil_sim_wsl.bat) and check the" >&2
  echo "firewall rule (output/hil/hydra_wsl_status.md)" >&2
  exit 1
fi
echo "    /clock is there"

HYDRA_PID=""
BAG_PID=""

# Stop one process group with SIGINT and give it time. Order matters: the bag must finalise its
# mcap (a SIGKILLed bag can leave an unreadable file) and Hydra only writes its graph on a clean
# SIGINT, so neither may be rushed.
stop_group() {
  local pid="$1" name="$2" grace="$3"
  [ -n "$pid" ] || return 0
  kill -0 -"$pid" 2>/dev/null || return 0
  echo "    SIGINT $name, up to ${grace}s"
  kill -INT -"$pid" 2>/dev/null || true
  for _ in $(seq 1 "$grace"); do
    kill -0 -"$pid" 2>/dev/null || { echo "    $name closed cleanly"; return 0; }
    sleep 1
  done
  echo "    $name did not stop in ${grace}s, SIGKILL" >&2
  kill -KILL -"$pid" 2>/dev/null || true
}

cleanup() {
  echo "--- stopping"
  stop_group "$BAG_PID" "bag" 45
  stop_group "$HYDRA_PID" "hydra" 30
}
trap cleanup EXIT INT TERM

# ---- 3. Hydra ----------------------------------------------------------------------------
echo "--- Hydra (workstation_full, seg_source=gt, pose_source=isaac_gt, hydra_cost on)"
setsid ros2 launch f1tenth_hydra hydra.launch.py \
  hydra_profile:=workstation_full seg_source:=gt pose_source:=isaac_gt \
  use_sim_time:=true hydra_cost:=true "log_dir:=$OUT/hydra" \
  "scene_semantics:=$REPO/sim/isaac/config/scenes/office.yaml" \
  > "$OUT/hydra.log" 2>&1 &
HYDRA_PID="$!"
sleep 12

# ---- 4. the bag --------------------------------------------------------------------------
# No depth: it is half the bytes and can be regenerated. ~11 MB/s with the rest.
#
# NO --compression-mode file. That compresses the whole bag when it closes, and a ~2 GB mcap takes
# longer than any sane shutdown grace: the first run's bag was still zstd-ing after 20 s, got
# SIGKILLed, and came out unreadable. An uncompressed bag finalises instantly, which matters more
# than the bytes.
echo "--- bag -> $OUT/bag"
setsid ros2 bag record -s mcap \
  -o "$OUT/bag" \
  /clock /tf /tf_static \
  /sim/ground_truth/left/semantic \
  /camera/image_raw /camera/camera_info \
  /sim/ground_truth/odom /sim/ground_truth/front_range \
  /hydra/backend/dsg /hydra/backend/dsg_mesh /hydra/status \
  > "$OUT/bag.log" 2>&1 &
BAG_PID="$!"
sleep 3

# ---- 5. drive and record -----------------------------------------------------------------
echo "--- driving"
RC=0
ARGS=(--out "$OUT" --mask-period "$MASK_PERIOD" --graph-period "$GRAPH_PERIOD")
[ -n "$SEGMENTS" ] && ARGS+=(--segments "$SEGMENTS")
python3 "$HERE/hydra_run_record.py" "${ARGS[@]}" "${EXTRA[@]}" 2>&1 | tee "$OUT/drive.log" || RC=$?

cleanup
trap - EXIT INT TERM
sleep 2

# ---- 6. artefacts ------------------------------------------------------------------------
echo
echo "===== artefacts"
GRAPH="$OUT/scene_graph/dsg_final.sparkdsg"
if [ -f "$GRAPH" ]; then
  python3 -m f1tenth_hydra.dsg_viewer --file "$GRAPH" --summary > "$OUT/graph_summary.txt" 2>&1 \
    && cat "$OUT/graph_summary.txt" || echo "(summary failed, see $OUT/graph_summary.txt)"
  python3 -m f1tenth_hydra.dsg_viewer --file "$GRAPH" --png "$OUT/graph.png" \
    > /dev/null 2>&1 && echo "graph.png written" || echo "(graph.png failed)"
else
  echo "NO dsg_final.sparkdsg: Hydra did not write its graph (did it get a clean SIGINT?)"
fi

echo
echo "--- is the bag readable, and did every topic get messages?"
# A recorder that did not exit cleanly leaves the mcap COMPLETE but writes no metadata.yaml, and
# `ros2 bag info <dir>` then says "Could not find metadata" -- which looks like a lost recording
# and is not one. Rebuild the metadata from the mcap, then report on the directory as usual.
if [ ! -f "$OUT/bag/metadata.yaml" ] && ls "$OUT/bag"/*.mcap >/dev/null 2>&1; then
  echo "    no metadata.yaml (the recorder was killed before writing it): reindexing"
  ros2 bag reindex "$OUT/bag" -s mcap > "$OUT/bag_reindex.log" 2>&1 \
    && echo "    reindexed" || echo "    reindex failed, see bag_reindex.log" >&2
fi
TARGET="$OUT/bag"
[ -f "$OUT/bag/metadata.yaml" ] || TARGET="$(ls "$OUT/bag"/*.mcap 2>/dev/null | head -1)"
if [ -n "$TARGET" ] && ros2 bag info "$TARGET" > "$OUT/bag_info.txt" 2>&1; then
  grep -E "Messages:|Duration:|Bag size:|Count:" "$OUT/bag_info.txt" | sed 's/^/    /' | head -20
  if grep -qE "Count: 0( |$)" "$OUT/bag_info.txt"; then
    echo "    WARNING: a topic recorded 0 messages (see bag_info.txt)" >&2
  fi
else
  echo "    bag NOT readable. See $OUT/bag_info.txt" >&2
fi

echo
echo "--- did the sim survive? (PhysX goes NaN when a car stalls against a wall)"
# The PhysX warnings go to the SIM's log, which lives on the WINDOWS side -- grepping hydra.log
# for them, as the first version did, always printed a reassuring 0.
SIM_LOG="${F1T_SIM_LOG:-/mnt/f/Users/Andreas Steffens/dev_ws/f1tenth_hil/scripts/sim/win/run.log}"
if [ -r "$SIM_LOG" ]; then
  # `grep -c` prints 0 AND exits 1 when there is no match, so `|| echo 0` appended a SECOND line
  # and the comparison below never matched: the first version of this announced that the physics
  # had blown up in the very run where nothing had.
  n="$(grep -c "Invalid PhysX transform" "$SIM_LOG" 2>/dev/null || true)"
  n="${n:-0}"
  if [ "$n" = "0" ]; then
    echo "    0 'Invalid PhysX transform' in the sim log: the car is intact"
  else
    echo "    $n 'Invalid PhysX transform' in the sim log: THE CAR'S PHYSICS BLEW UP." >&2
    echo "    Restart the sim before the next run; its state is no longer trustworthy." >&2
  fi
else
  echo "    (sim log not readable from here; set F1T_SIM_LOG to check)"
fi

COST="$HOME/.ros/hydra_cost/latest"
[ -d "$COST" ] && cp -r "$COST" "$OUT/hydra_cost" 2>/dev/null && echo "hydra_cost copied"

du -sh "$OUT" 2>/dev/null
find "$OUT" -maxdepth 2 -type d -printf '  %p\n' 2>/dev/null | head -10
echo
echo "open the graph:  scripts/hydra/open_graph.sh $OUT"
echo "run.json:        $OUT/run.json"
exit "$RC"
