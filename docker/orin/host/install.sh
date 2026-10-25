#!/bin/bash
# Install the host side of the car's Jazzy container for this user (plan P6.5).
# No sudo, idempotent (a second run changes nothing), touches nothing the
# native stack uses except one guarded line in ~/.bashrc (the banner):
#
#   docker/orin/host/install.sh [--dry-run]
#
# 1. State directories under F1T_STATE (/scratch/fabiocar/f1tenth_container):
#    ros archive ctl gateway archive_trigger zed_settings hf_cache run logs
#    missions/{mission_reports,mission_videos,llm_generated,corridors_jsons}.
# 2. The ZED calibration, seeded ONCE: every /usr/local/zed/settings/SN*.conf
#    copied (cp -p, never over an existing copy) into $F1T_STATE/zed_settings.
#    The container's SDK 5.4 writes its own copy; the host's 4.2.5 directory is
#    never mounted.
# 3. Host directories compose mounts read-only, created empty when missing:
#    ~/.config/f1tenth_car (optional stack_args), ~/.local/share/f1tenth_gateway/www;
#    ~/.config/f1tenth (the image pin, the unit's EnvironmentFile, created with
#    the defaults archive separate, no drift override).
# 4. Commands, symlinked into ~/.local/bin: f1tenth_ctl_car (Telegram's
#    ctl_command), stack_mode, docker_orin, orin_stack_guard.
# 5. User units, symlinked into ~/.config/systemd/user, daemon-reload:
#    f1tenth-car-container.service (NOT enabled: plan P12) and
#    f1tenth-archive-trigger.path (enabled and started: it only watches the
#    container's trigger file).
# 6. The banner: ~/.local/share/f1tenth_car/bashrc_banner.sh (symlink) and one
#    line in ~/.bashrc that sources it if present.
# An existing non-symlink file where a symlink goes is never replaced (refused).
# Env: F1T_STATE.
set -euo pipefail
SELF=$(readlink -f "${BASH_SOURCE[0]}")
HOST_DIR=$(dirname "$SELF")
REPO=$(cd "$HOST_DIR/../../.." && pwd)
STATE=${F1T_STATE:-/scratch/fabiocar/f1tenth_container}
DRY=0
[ "${1:-}" = --dry-run ] && DRY=1
case "${1:-}" in -h|--help) sed -n '2,/^set -euo/p' "$SELF" | sed '$d; s/^# \{0,1\}//'; exit 0 ;; esac
BANNER_MARK='# f1tenth_car banner'
changes=0

say() { echo "install: $*"; }
do_() {
  changes=$((changes + 1))
  if [ "$DRY" = 1 ]; then printf 'install: dry-run: would run:'; printf ' %q' "$@"; echo; return 0; fi
  say "+ $*"
  "$@"
}

[ "$(id -u)" != 0 ] || { echo "install: run as the car's user (fabiocar), not root" >&2; exit 1; }
case "$STATE/" in
  /scratch/fabiocar/*|"$HOME"/*) ;;
  *) echo "install: F1T_STATE=$STATE is not under /scratch/fabiocar/ or \$HOME" >&2; exit 1 ;;
esac

# 1. state
for d in ros archive ctl gateway archive_trigger zed_settings hf_cache run logs \
         missions/mission_reports missions/mission_videos missions/llm_generated missions/corridors_jsons; do
  [ -d "$STATE/$d" ] || do_ mkdir -p "$STATE/$d"
done

# 2. ZED calibration, once
for conf in /usr/local/zed/settings/SN*.conf; do
  [ -f "$conf" ] || continue
  [ -e "$STATE/zed_settings/$(basename "$conf")" ] || do_ cp -p "$conf" "$STATE/zed_settings/"
done
ls "$STATE"/zed_settings/SN*.conf > /dev/null 2>&1 || [ "$DRY" = 1 ] \
  || say "WARN: no ZED calibration file seeded (/usr/local/zed/settings/SN*.conf not found)"

# 3. host directories and the unit's environment file
for d in "$HOME/.config/f1tenth_car" "$HOME/.local/share/f1tenth_gateway/www" "$HOME/.config/f1tenth" \
         "$HOME/.local/bin" "$HOME/.config/systemd/user" "$HOME/.local/share/f1tenth_car"; do
  [ -d "$d" ] || do_ mkdir -p "$d"
done
if [ ! -f "$HOME/.config/f1tenth/car_container.env" ]; then
  if [ "$DRY" = 1 ]; then
    do_ write "$HOME/.config/f1tenth/car_container.env"
  else
    changes=$((changes + 1))
    printf '# Read by f1tenth-car-container.service (docker_orin.sh up); stack_mode.sh rewrites it.\nF1T_ARCHIVE_MODE=separate\nF1T_ALLOW_DRIFT=0\n' \
      > "$HOME/.config/f1tenth/car_container.env"
    say "+ wrote $HOME/.config/f1tenth/car_container.env"
  fi
fi

# 4./5./6. symlinks
link() {
  local target=$1 linkpath=$2
  if [ -L "$linkpath" ] && [ "$(readlink -f "$linkpath")" = "$(readlink -f "$target")" ]; then return 0; fi
  if [ -e "$linkpath" ] && [ ! -L "$linkpath" ]; then
    echo "install: $linkpath exists and is not a symlink: not replacing it (move it away, then re-run)" >&2
    exit 1
  fi
  [ ! -L "$linkpath" ] || say "relinking $linkpath (was -> $(readlink "$linkpath"))"
  do_ ln -sfn "$target" "$linkpath"
}
link "$HOST_DIR/f1tenth_ctl_car" "$HOME/.local/bin/f1tenth_ctl_car"
link "$HOST_DIR/stack_mode.sh" "$HOME/.local/bin/stack_mode"
link "$REPO/scripts/docker_orin.sh" "$HOME/.local/bin/docker_orin"
link "$REPO/scripts/orin_stack_guard.sh" "$HOME/.local/bin/orin_stack_guard"
link "$HOST_DIR/bashrc_banner.sh" "$HOME/.local/share/f1tenth_car/bashrc_banner.sh"
units_before=$changes
link "$HOST_DIR/systemd/f1tenth-car-container.service" "$HOME/.config/systemd/user/f1tenth-car-container.service"
link "$HOST_DIR/systemd/f1tenth-archive-trigger.path" "$HOME/.config/systemd/user/f1tenth-archive-trigger.path"
if [ "$changes" != "$units_before" ]; then do_ systemctl --user daemon-reload; fi
systemctl --user cat f1tenth-archive.service > /dev/null 2>&1 \
  || say "WARN: f1tenth-archive.service is not installed: the archive trigger has nothing to start"
if ! systemctl --user is-enabled --quiet f1tenth-archive-trigger.path 2>/dev/null \
   || ! systemctl --user is-active --quiet f1tenth-archive-trigger.path 2>/dev/null; then
  do_ systemctl --user enable --now f1tenth-archive-trigger.path
fi

line="[ -f ~/.local/share/f1tenth_car/bashrc_banner.sh ] && . ~/.local/share/f1tenth_car/bashrc_banner.sh  $BANNER_MARK"
if ! grep -qF "$BANNER_MARK" "$HOME/.bashrc" 2>/dev/null; then
  if [ "$DRY" = 1 ]; then
    do_ append "$HOME/.bashrc" "$line"
  else
    changes=$((changes + 1))
    printf '%s\n' "$line" >> "$HOME/.bashrc"
    say "+ appended the banner line to ~/.bashrc"
  fi
fi

if [ "$changes" = 0 ]; then say "nothing to do (already installed)"; else say "$changes change(s)$( [ "$DRY" = 1 ] && echo ' (dry-run: none made)')"; fi
say "not enabled: f1tenth-car-container.service (cutover, plan P12). Next: docker_orin pin <image>; stack_mode container"
