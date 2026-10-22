#!/usr/bin/env bash
# main on the Orin, in its OWN worktree, next to the campaign checkout.
# PREPARED, NOT RUN.
#
#   campaign   ~/dev_ws/f1tenth_more        (stack-campaign-final-2026-10-22 /
#              campaign-final; build/ install/ log/ untouched by this script)
#   main       ~/dev_ws/f1tenth_more_main   (branch main; its own build/ install/ log/)
#
# Copy it to the Orin and run it there (CLAUDE.md: remote work goes through a
# script, never a here-doc typed into ssh):
#   scp output/consolidation/orin_main.sh fabiocar@jetson-orin:~/orin_main.sh
#   ssh fabiocar@jetson-orin
#   bash ~/orin_main.sh setup <main sha>   # worktree + submodules (git only, car may run)
#   bash ~/orin_main.sh build              # colcon build of main (refuses while a stack runs)
#   bash ~/orin_main.sh shell main         # interactive shell with ONLY main sourced
#   bash ~/orin_main.sh shell campaign     # interactive shell with ONLY the campaign sourced
#   bash ~/orin_main.sh which              # which workspace the running stack comes from
set -euo pipefail

CAMPAIGN="$HOME/dev_ws/f1tenth_more"
MAIN="$HOME/dev_ws/f1tenth_more_main"
ROS_SETUP=/opt/ros/humble/setup.bash
# The Orin's runtime environment from ~/.bashrc, minus its campaign overlay line.
NVLIB="$HOME/.local/lib/python3.10/site-packages/nvidia/cu12/lib"

die() { echo "orin_main: $*" >&2; exit 1; }

stack_pids() {
  # Every process of this user, matched on its arguments here rather than with
  # pgrep -f, which would match this script's own command line when a pattern
  # is passed in. Process names do not work: a ros2 launch is "ros2", a node is
  # its truncated executable name.
  local pid
  for pid in $(pgrep -u "$USER" || true); do
    [ "$pid" = "$$" ] || [ "$pid" = "${BASHPID:-}" ] && continue
    tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null \
      | grep -q -E 'component_supervisor_node|supervisor_bringup\.launch\.py|stack_bringup\.launch\.py' \
      && echo "$pid"
  done
  return 0
}

campaign_fingerprint() {
  # HEAD, tracked-file state (staged and unstaged) and the install/ and build/
  # listings of the campaign checkout. Untracked files are left out: a running
  # stack writes run data into the checkout.
  {
    git -C "$CAMPAIGN" rev-parse HEAD
    git -C "$CAMPAIGN" --no-optional-locks status --porcelain=v1 --untracked-files=no
    git -C "$CAMPAIGN" --no-optional-locks diff --cached | sha256sum
    ls -l --time-style=+%s "$CAMPAIGN/install" "$CAMPAIGN/build"
  } | sha256sum | cut -c1-16
}

cmd_setup() {
  local want="${1:?usage: setup <full sha of the approved main>}"
  [ -e "$MAIN" ] && die "$MAIN already exists"
  local before; before=$(campaign_fingerprint)
  git -C "$CAMPAIGN" fetch origin main
  [ "$(git -C "$CAMPAIGN" rev-parse FETCH_HEAD)" = "$want" ] \
    || die "origin/main is $(git -C "$CAMPAIGN" rev-parse FETCH_HEAD), not the approved $want"
  # The Orin's local main is not checked out anywhere: fast-forward it (refused if not ff).
  git -C "$CAMPAIGN" fetch origin main:main
  git -C "$CAMPAIGN" worktree add "$MAIN" main
  # Submodules of the new worktree live under the campaign repo's
  # .git/worktrees/f1tenth_more_main/modules; the campaign's own are not touched.
  git -C "$MAIN" submodule update --init --recursive
  git -C "$MAIN" submodule status
  [ "$(campaign_fingerprint)" = "$before" ] || die "the campaign checkout changed during setup: look before building"
  echo "OK: $MAIN on main $(git -C "$MAIN" rev-parse --short HEAD); campaign checkout unchanged"
}

clean_env() {
  # A login shell would source the campaign install (~/.bashrc line 119), so
  # every main command starts from an empty environment.
  env -i HOME="$HOME" USER="$USER" LOGNAME="$USER" LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    DISPLAY="${DISPLAY:-}" SSH_AUTH_SOCK="${SSH_AUTH_SOCK:-}" "$@"
}

cmd_build() {
  [ -d "$MAIN/src" ] || die "run setup first"
  [ -z "$(stack_pids)" ] || die "a stack is running (pids $(stack_pids | tr '\n' ' ')): the build takes every core; stop it first"
  local before; before=$(campaign_fingerprint)
  clean_env bash --noprofile --norc -c '
    set -eo pipefail
    source '"$ROS_SETUP"'
    case ":${AMENT_PREFIX_PATH:-}:${CMAKE_PREFIX_PATH:-}:" in
      *"/dev_ws/f1tenth_more/install"*) echo "campaign install leaked into the build env" >&2; exit 1;;
    esac
    cd '"$MAIN"'
    # ~/.colcon/defaults.yaml adds --symlink-install anyway; say it explicitly.
    colcon --log-base log build --symlink-install --build-base build --install-base install
  '
  [ "$(campaign_fingerprint)" = "$before" ] || die "the campaign checkout changed during the build"
  # The ABI check of CLAUDE.md (rebuild policy), on main's install only.
  clean_env bash --noprofile --norc -c '
    source '"$ROS_SETUP"'; source '"$MAIN"'/install/setup.bash
    cd '"$MAIN"'
    find install -path "*/lib/*" \( -type f -o -type l \) | while read -r f; do
      file -L "$f" | grep -q ELF || continue
      ldd -r "$f" 2>&1 | grep "undefined symbol" | grep -v "__rosidl_\|__msg__\|__srv__\|__action__" | sed "s|^|$f: |"
    done
  '
  echo "OK: main built in $MAIN/{build,install,log}; campaign checkout unchanged"
}

cmd_shell() {
  local which_ws="${1:?usage: shell main|campaign}" ws label
  case "$which_ws" in
    main)     ws="$MAIN";     label=MAIN ;;
    campaign) ws="$CAMPAIGN"; label=CAMPAIGN ;;
    *) die "shell main|campaign" ;;
  esac
  [ -f "$ws/install/setup.bash" ] || die "$ws/install/setup.bash missing (not built)"
  local rc; rc=$(mktemp)
  cat > "$rc" <<EOF
source $ROS_SETUP
source $ws/install/setup.bash
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROS_SUPER_CLIENT=TRUE
export LD_LIBRARY_PATH="$NVLIB\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export F1T_WS_LABEL=$label
cd $ws
PS1='[\[\e[1;$([ $label = MAIN ] && echo 35 || echo 32)m\]$label\[\e[0m\] \W]\$ '
echo "== $label workspace: $ws (\$(git rev-parse --abbrev-ref HEAD) \$(git rev-parse --short HEAD))"
echo "== f1tenth_bringup from: \$(ros2 pkg prefix f1tenth_bringup)"
case "\$(ros2 pkg prefix f1tenth_bringup)" in $ws/install/*) ;; *) echo "!! WRONG WORKSPACE SOURCED" ;; esac
running=\$(bash $HOME/orin_main.sh which 2>/dev/null || true)
[ -n "\$running" ] && echo "== already running: \$running"
rm -f $rc
EOF
  exec env -i HOME="$HOME" USER="$USER" LOGNAME="$USER" LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    DISPLAY="${DISPLAY:-}" SSH_AUTH_SOCK="${SSH_AUTH_SOCK:-}" \
    bash --noprofile --rcfile "$rc" -i
}

cmd_which() {
  local pid exe
  for pid in $(stack_pids); do
    exe=$(tr '\0' ' ' < "/proc/$pid/cmdline")
    case "$exe" in
      *"$MAIN/install/"*)     echo "MAIN     pid $pid: ${exe:0:120}" ;;
      *"$CAMPAIGN/install/"*) echo "CAMPAIGN pid $pid: ${exe:0:120}" ;;
      *) # ros2 launch: decide by the AMENT_PREFIX_PATH it runs with
         if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q "^AMENT_PREFIX_PATH=.*$MAIN/install"; then
           echo "MAIN     pid $pid: ${exe:0:120}"
         else
           echo "CAMPAIGN pid $pid: ${exe:0:120}"
         fi ;;
    esac
  done
}

case "${1:-}" in
  setup) shift; cmd_setup "$@" ;;
  build) cmd_build ;;
  shell) shift; cmd_shell "$@" ;;
  which) cmd_which ;;
  *) sed -n '2,17p' "$0"; exit 2 ;;
esac
