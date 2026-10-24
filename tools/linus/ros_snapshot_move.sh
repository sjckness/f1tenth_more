#!/usr/bin/env bash
# Move linus's ROS 2 Jazzy apt packages onto snapshots.ros.org/jazzy/<date>,
# the set the Thor runs and the Orin image (docker/orin/Dockerfile) installs.
#
#   tools/linus/ros_snapshot_move.sh [--dry-run]       no root: simulates the move in
#                                                      a private apt state, changes nothing
#   sudo tools/linus/ros_snapshot_move.sh --apply      key + source + pin, then the move
#   sudo tools/linus/ros_snapshot_move.sh --rollback   removes the source and the pin
#                                                      (packages stay; reinstall from
#                                                      packages.ros.org by hand)
#   --snapshot YYYY-MM-DD                              default 2026-09-11
#
# Only ros-jazzy-* follows the snapshot (pin 1001, and packages.ros.org -10 for
# ros-jazzy-*); colcon, rosdep and the rest keep coming from packages.ros.org.
# Refuses while a ROS process or a colcon build runs, and --apply refuses a plan
# with any removal. After --apply every CMake package must be rebuilt clean
# (CLAUDE.md "Rebuild policy"): tools/linus/rebuild_after_apt.sh.
set -euo pipefail

SNAPSHOT=2026-09-11
MODE=dry-run
FPR=4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
KEY="$REPO/docker/orin/keys/ros-snapshots.asc"
OUT="$REPO/output/orin_container"
KEYRING=/usr/share/keyrings/ros-snapshots-keyring.gpg
LIST=/etc/apt/sources.list.d/ros2-snapshot.list
PREF=/etc/apt/preferences.d/ros2-snapshot.pref
ARCH="$(dpkg --print-architecture)"

while (($#)); do
  case "$1" in
    --dry-run) MODE=dry-run ;;
    --apply) MODE=apply ;;
    --rollback) MODE=rollback ;;
    --snapshot) SNAPSHOT="${2:?--snapshot needs a date}"; shift ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ $SNAPSHOT =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}$ ]] || { echo "bad snapshot date: $SNAPSHOT" >&2; exit 2; }

log() { printf '[ros_snapshot_move] %s\n' "$*"; }
die() { printf '[ros_snapshot_move] ERROR: %s\n' "$*" >&2; exit 1; }

source_line() {
  printf 'deb [arch=%s signed-by=%s] http://snapshots.ros.org/jazzy/%s/ubuntu noble main\n' \
    "$ARCH" "$1" "$SNAPSHOT"
}

pin_text() {
  cat <<EOF
# ros-jazzy-* from snapshots.ros.org/jazzy/$SNAPSHOT only (tools/linus/ros_snapshot_move.sh):
# the Thor's and the Orin image's set. Everything else keeps its usual sources.
Package: ros-jazzy-*
Pin: origin snapshots.ros.org
Pin-Priority: 1001

Package: ros-jazzy-*
Pin: origin packages.ros.org
Pin-Priority: -10
EOF
}

installed_ros() {
  dpkg-query -W -f='${db:Status-Abbrev} ${Package}\n' 'ros-jazzy-*' | awk '$1=="ii"{print $2}'
}

# Refuse while anything ROS runs: the move replaces libraries under live processes.
check_idle() {
  local d pid cmd busy=()
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    [[ $pid == "$$" || $pid == "$PPID" ]] && continue
    cmd=$(tr '\0' ' ' <"$d/cmdline" 2>/dev/null) || continue
    case "$cmd" in
      *"/opt/ros/jazzy/"*|*"ros2 launch"*|*"ros2 run"*|*"_ros2_daemon"*|*"colcon build"*|*"colcon test"*|*"fastdds discovery"*)
        busy+=("$pid ${cmd:0:120}") ;;
    esac
  done
  if ((${#busy[@]})); then
    printf '  %s\n' "${busy[@]}" >&2
    die "ROS processes or colcon builds are running (above); stop them first (ros2 daemon stop)"
  fi
}

check_key() {
  [[ -f $KEY ]] || die "key not found: $KEY"
  local home
  home=$(mktemp -d)
  GNUPGHOME="$home" gpg --show-keys --with-colons "$KEY" 2>/dev/null | grep -q "^fpr:::::::::$FPR:" \
    || { rm -r -- "${home:?}"; die "fingerprint of $KEY is not $FPR"; }
  rm -r -- "${home:?}"
}

# Summarise an apt-get -s transcript: counts and every downgrade / removal.
summarise() {
  local sim=$1
  local inst remv down new
  inst=$(grep -c '^Inst ' "$sim" || true)
  remv=$(grep -c '^Remv ' "$sim" || true)
  new=$(grep '^Inst ' "$sim" | grep -vc '^Inst [^ ]* \[' || true)
  # "Inst pkg [old] (new ...)": only upgrades/downgrades carry [old] right after the name.
  down=$(grep -E '^Inst [^ ]+ \[[^]]+\] \(' "$sim" | while read -r _ pkg old nv _; do
    old=${old#[}; old=${old%]}; nv=${nv#(}
    if dpkg --compare-versions "$nv" lt "$old"; then printf '%s %s -> %s\n' "$pkg" "$old" "$nv"; fi
  done)
  log "plan: $inst installs/upgrades ($new new packages), $remv removals, $(printf '%s' "$down" | grep -c . || true) downgrades"
  if [[ -n $down ]]; then printf '%s\n' "$down" | sed 's/^/  downgrade: /'; fi
  grep '^Remv ' "$sim" | sed 's/^/  /' || true
  REMOVALS=$remv
}

# apt-get update, tolerating unrelated broken repositories (linus has one) but
# not a missing snapshot index.
apt_update() {
  local lists=$1 logf=$2; shift 2
  if ! apt-get "$@" update >"$logf" 2>&1; then
    log "apt-get update reported errors (unrelated repositories are tolerated):"
    grep -E '^(E|W):' "$logf" | sed 's/^/  /' || true
  fi
  compgen -G "$lists/snapshots.ros.org_jazzy_${SNAPSHOT}_ubuntu_dists_noble_main_binary-${ARCH}_Packages*" >/dev/null \
    || { tail -20 "$logf"; die "no snapshots.ros.org/jazzy/$SNAPSHOT index was fetched"; }
}

mkdir -p "$OUT"

case "$MODE" in
dry-run)
  check_key
  check_idle
  work=$(mktemp -d)
  trap 'rm -r -- "${work:?}"' EXIT
  mkdir -p "$work"/{lists/partial,parts,prefparts,cache/archives/partial}
  gpg --dearmor <"$KEY" >"$work/keyring.gpg"
  for f in /etc/apt/sources.list.d/*; do [[ -f $f ]] && ln -s "$f" "$work/parts/"; done
  source_line "$work/keyring.gpg" >"$work/parts/ros2-snapshot.list"
  for f in /etc/apt/preferences.d/*; do [[ -f $f ]] && ln -s "$f" "$work/prefparts/"; done
  pin_text >"$work/prefparts/ros2-snapshot.pref"
  aptopts=(-o "Dir::State::Lists=$work/lists" -o "Dir::Etc::SourceParts=$work/parts"
           -o "Dir::Etc::PreferencesParts=$work/prefparts" -o "Dir::Cache=$work/cache"
           -o Debug::NoLocking=1 -o APT::Sandbox::User="$(id -un)")
  log "private apt state in $work (nothing in /etc or /var changes)"
  apt_update "$work/lists" "$work/update.log" "${aptopts[@]}"
  cand=$(apt-cache "${aptopts[@]}" policy ros-jazzy-rosidl-generator-rs | awk '/Candidate:/{print $2}')
  log "candidate ros-jazzy-rosidl-generator-rs: $cand"
  mapfile -t pkgs < <(installed_ros)
  sim="$OUT/linus_ros_move_plan_${SNAPSHOT}.txt"
  apt-get "${aptopts[@]}" -s install --only-upgrade --allow-downgrades "${pkgs[@]}" >"$sim" 2>&1 \
    || { tail -30 "$sim"; die "apt-get -s failed (transcript $sim)"; }
  summarise "$sim"
  log "transcript: $sim"
  [[ $REMOVALS == 0 ]] || die "the plan removes packages: --apply would refuse"
  log "dry run OK. To apply: sudo $REPO/tools/linus/ros_snapshot_move.sh --apply"
  ;;
apply)
  [[ $EUID -eq 0 ]] || die "--apply needs root (sudo)"
  check_key
  check_idle
  stamp=$(date +%Y%m%dT%H%M%S)
  dpkg-query -W -f='${Package} ${Version}\n' 'ros-jazzy-*' 'python3-colcon-*' >"$OUT/linus_ros_before_$stamp.txt" 2>/dev/null || true
  gpg --dearmor <"$KEY" >"$KEYRING.tmp" && mv "$KEYRING.tmp" "$KEYRING"
  chmod 644 "$KEYRING"
  source_line "$KEYRING" >"$LIST"
  pin_text >"$PREF"
  log "wrote $KEYRING, $LIST, $PREF"
  apt_update /var/lib/apt/lists "$OUT/linus_ros_move_update_$stamp.log"
  cand=$(apt-cache policy ros-jazzy-rosidl-generator-rs | awk '/Candidate:/{print $2}')
  log "candidate ros-jazzy-rosidl-generator-rs: $cand"
  mapfile -t pkgs < <(installed_ros)
  sim="$OUT/linus_ros_move_apply_sim_$stamp.txt"
  apt-get -s install --only-upgrade --allow-downgrades "${pkgs[@]}" >"$sim" 2>&1 || { tail -30 "$sim"; die "simulation failed"; }
  summarise "$sim"
  [[ $REMOVALS == 0 ]] || die "the plan removes packages; nothing installed. Undo the source with --rollback"
  DEBIAN_FRONTEND=noninteractive apt-get install -y --only-upgrade --allow-downgrades "${pkgs[@]}"
  dpkg-query -W -f='${Package} ${Version}\n' 'ros-jazzy-*' 'python3-colcon-*' >"$OUT/linus_ros_after_$stamp.txt" 2>/dev/null || true
  if [[ -n ${SUDO_USER:-} ]]; then chown -R "$SUDO_USER:" "$OUT"; fi
  log "done: $(dpkg-query -W -f='${Version}' ros-jazzy-rosidl-generator-rs) rosidl-generator-rs"
  log "next (no sudo): tools/linus/rebuild_after_apt.sh <worktree> for every worktree you run"
  ;;
rollback)
  [[ $EUID -eq 0 ]] || die "--rollback needs root (sudo)"
  rm -f -- "$LIST" "$PREF"
  apt-get update
  log "snapshot source and pin removed; installed packages unchanged"
  ;;
esac
