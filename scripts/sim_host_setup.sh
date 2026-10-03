#!/usr/bin/env bash
# sim_host_setup.sh - reproducible setup of an x86 Ubuntu 24.04 simulation host
# for f1tenth_sim (ROS 2 Jazzy + Gazebo Harmonic via ros_gz).
#
# Record of how the sim PC was set up (output/sim_port_report.md, decision D0:
# native apt install, not Docker), so a second host (linus) can be set up the
# same way. Idempotent: re-running only installs what is missing.
#
# Run as your normal user, from zsh or bash (it calls sudo itself, so rosdep's
# cache stays in your home and not root's):
#   scripts/sim_host_setup.sh            # check, install, verify
#   scripts/sim_host_setup.sh --check    # read-only: report what is missing
#
# Env overrides:
#   MIN_FREE_GB  free space required on / before installing (default 25)
# The shebang runs this under bash from any shell, zsh included. If someone runs
# `zsh scripts/sim_host_setup.sh` or `sh ...` instead, re-exec under bash.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -euo pipefail

MIN_FREE_GB="${MIN_FREE_GB:-25}"
CHECK_ONLY=false
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=true

# Package list from output/sim_port_report.md §0.5 (rosdep keys of
# f1tenth_description / f1tenth_params / f1tenth_messages / f1tenth_sim, plus
# base and test tools). ros-jazzy-ros-gz pulls in the ROS-built Harmonic
# *-vendor packages; an OSRF gz-harmonic install, if present, is left alone.
PKGS=(
  ros-jazzy-ros-base
  ros-jazzy-rmw-fastrtps-cpp
  ros-jazzy-ros-gz
  ros-jazzy-gz-ros2-control
  ros-jazzy-ros2-controllers
  ros-jazzy-controller-manager
  ros-jazzy-joint-state-broadcaster
  ros-jazzy-ackermann-steering-controller
  ros-jazzy-ackermann-msgs
  ros-jazzy-xacro
  ros-jazzy-robot-state-publisher
  ros-jazzy-joint-state-publisher
  ros-jazzy-tf2-ros
  ros-jazzy-tf2-tools
  ros-jazzy-rviz2
  ros-jazzy-foxglove-bridge
  ros-jazzy-ament-copyright
  ros-jazzy-ament-flake8
  ros-jazzy-ament-pep257
  ros-jazzy-ament-lint-auto
  ros-jazzy-ament-lint-common
  ros-jazzy-ament-cmake-auto
  ros-jazzy-rosidl-default-generators
)

say() { printf '\n== %s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

say "Host"
. /etc/os-release
echo "$PRETTY_NAME, $(uname -m), $(hostname)"
[[ "$VERSION_CODENAME" == "noble" ]] || die "Jazzy binaries need Ubuntu 24.04 (noble), found $VERSION_CODENAME"
[[ "$(uname -m)" == "x86_64" ]] || echo "WARNING: not x86_64; only tested on x86_64"

say "Disk"
free_gb=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
echo "free on /: ${free_gb} GB (required: ${MIN_FREE_GB} GB)"

say "ROS apt source"
# Read the source files directly. Not `apt-cache policy | grep -q`: grep exits
# on the first match, apt-cache dies of SIGPIPE, and under pipefail the
# pipeline reports failure, so a configured source read as MISSING at random.
# Handles both formats: one-line .list ("deb ... /ros2/ubuntu noble main") and
# deb822 .sources (ros2-apt-source writes ros2.sources with URIs:/Suites: fields).
ros_source_configured() {
  local f
  for f in /etc/apt/sources.list /etc/apt/sources.list.d/*.list; do
    [[ -r "$f" ]] || continue
    grep -Eq '^[[:space:]]*deb[[:space:]].*packages\.ros\.org/ros2/ubuntu[/[:space:]]+noble([[:space:]]|$)' "$f" && return 0
  done
  for f in /etc/apt/sources.list.d/*.sources; do
    [[ -r "$f" ]] || continue
    grep -Eq '^URIs:.*packages\.ros\.org/ros2/ubuntu' "$f" \
      && grep -Eq '^Suites:.*([[:space:]]|^Suites:)noble([[:space:]]|$)' "$f" \
      && ! grep -Eiq '^Enabled:[[:space:]]*no' "$f" && return 0
  done
  return 1
}
if ros_source_configured; then
  echo "packages.ros.org/ros2 noble: configured"
  have_source=true
else
  echo "packages.ros.org/ros2 noble: MISSING"
  have_source=false
fi

say "Packages"
missing=()
for p in "${PKGS[@]}"; do
  dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q 'install ok installed' || missing+=("$p")
done
if ((${#missing[@]})); then
  echo "missing (${#missing[@]}): ${missing[*]}"
else
  echo "all ${#PKGS[@]} packages installed"
fi

if $CHECK_ONLY; then
  say "--check: nothing changed"
  exit 0
fi

if ((${#missing[@]})) || ! $have_source; then
  ((free_gb >= MIN_FREE_GB)) || die "only ${free_gb} GB free on /, need ${MIN_FREE_GB} GB (set MIN_FREE_GB to override)"

  if ! $have_source; then
    # Official method (docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):
    # the ros2-apt-source package installs the key and the sources entry.
    say "Adding the ROS 2 apt source"
    sudo apt-get update
    sudo apt-get install -y curl software-properties-common
    sudo add-apt-repository -y universe
    ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F\" '{print $4}')
    [[ -n "$ver" ]] || die "could not determine the ros-apt-source release"
    tmp=$(mktemp -d)
    curl -fsSL -o "$tmp/ros2-apt-source.deb" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
    sudo apt-get install -y "$tmp/ros2-apt-source.deb"
    rm -rf "$tmp"
  fi

  say "Installing ${#missing[@]} packages"
  sudo apt-get update
  sudo apt-get install -y ros-dev-tools "${PKGS[@]}"
fi

say "rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy >/dev/null && echo "rosdep cache updated"

# This script itself runs in bash, so it sources setup.bash; your zsh shell
# uses setup.zsh (printed at the end).
say "Verification (sourced /opt/ros/jazzy)"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
echo "ROS_DISTRO=$ROS_DISTRO"
echo "which gz: $(command -v gz || echo 'not found')"
gz sim --versions 2>/dev/null || echo "gz sim --versions failed"
for p in ros_gz_sim ros_gz_bridge gz_ros2_control ackermann_steering_controller joint_state_broadcaster; do
  printf '%-32s %s\n' "$p" "$(ros2 pkg prefix "$p" 2>/dev/null || echo MISSING)"
done

say "Done"
echo "Shell setup is left to you (not written to any rc file). In zsh:"
echo "  source /opt/ros/jazzy/setup.zsh"
echo "  source <workspace>/install/setup.zsh   # after colcon build"
