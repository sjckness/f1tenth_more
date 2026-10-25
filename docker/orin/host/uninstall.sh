#!/bin/bash
# Remove what docker/orin/host/install.sh installed (plan P6.5). No sudo,
# idempotent:
#
#   docker/orin/host/uninstall.sh [--dry-run]
#
# Refuses while the container f1tenth_car runs (stack_mode native first).
# Disables and stops f1tenth-archive-trigger.path, disables
# f1tenth-car-container.service (stops it if it is still "active (exited)"),
# removes the symlinks install.sh made (only symlinks that point to an
# f1tenth worktree's docker/orin/host or scripts files), daemon-reload, and the
# banner line in ~/.bashrc.
# Kept: the state under F1T_STATE (runs, ZED copy, logs, history), the image
# pin and the unit's environment file in ~/.config/f1tenth, the images and
# the named volumes (f1tenth_car_*): data, removed only by hand.
set -euo pipefail
DRY=0
[ "${1:-}" = --dry-run ] && DRY=1
case "${1:-}" in -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;; esac
BANNER_MARK='# f1tenth_car banner'
STATE=${F1T_STATE:-/scratch/fabiocar/f1tenth_container}

say() { echo "uninstall: $*"; }
do_() {
  if [ "$DRY" = 1 ]; then printf 'uninstall: dry-run: would run:'; printf ' %q' "$@"; echo; return 0; fi
  say "+ $*"
  "$@"
}

[ "$(id -u)" != 0 ] || { echo "uninstall: run as the car's user, not root" >&2; exit 1; }
if [ "$(docker inspect --type container -f '{{.State.Status}}' f1tenth_car 2> /dev/null || true)" = running ]; then
  echo "uninstall: f1tenth_car runs: stack_mode native first" >&2
  exit 1
fi

units="$HOME/.config/systemd/user"
if systemctl --user cat f1tenth-archive-trigger.path > /dev/null 2>&1; then
  do_ systemctl --user disable --now f1tenth-archive-trigger.path || true
fi
if systemctl --user cat f1tenth-car-container.service > /dev/null 2>&1; then
  if systemctl --user is-enabled --quiet f1tenth-car-container.service 2>/dev/null; then
    do_ systemctl --user disable f1tenth-car-container.service
  fi
  if systemctl --user is-active --quiet f1tenth-car-container.service 2>/dev/null; then
    do_ systemctl --user stop f1tenth-car-container.service
  fi
fi

# Only our symlinks: a link whose target is one of the files install.sh links.
unlink_ours() {
  local linkpath=$1 suffix=$2 target
  [ -L "$linkpath" ] || return 0
  target=$(readlink "$linkpath")
  case "$target" in
    */"$suffix") do_ rm -f -- "$linkpath" ;;
    *) say "keeping $linkpath (-> $target: not installed by install.sh)" ;;
  esac
}
removed_units=0
for u in f1tenth-car-container.service f1tenth-archive-trigger.path; do
  if [ -L "$units/$u" ]; then removed_units=1; fi
  unlink_ours "$units/$u" "docker/orin/host/systemd/$u"
done
[ "$removed_units" = 0 ] || do_ systemctl --user daemon-reload
unlink_ours "$HOME/.local/bin/f1tenth_ctl_car" docker/orin/host/f1tenth_ctl_car
unlink_ours "$HOME/.local/bin/stack_mode" docker/orin/host/stack_mode.sh
unlink_ours "$HOME/.local/bin/docker_orin" scripts/docker_orin.sh
unlink_ours "$HOME/.local/bin/orin_stack_guard" scripts/orin_stack_guard.sh
unlink_ours "$HOME/.local/share/f1tenth_car/bashrc_banner.sh" docker/orin/host/bashrc_banner.sh

if grep -qF "$BANNER_MARK" "$HOME/.bashrc" 2>/dev/null; then
  if [ "$DRY" = 1 ]; then
    say "dry-run: would remove the line marked '$BANNER_MARK' from ~/.bashrc"
  else
    grep -vF "$BANNER_MARK" "$HOME/.bashrc" > "$HOME/.bashrc.f1tenth_car.tmp"
    cat "$HOME/.bashrc.f1tenth_car.tmp" > "$HOME/.bashrc"
    rm -f "$HOME/.bashrc.f1tenth_car.tmp"
    say "+ removed the banner line from ~/.bashrc"
  fi
fi
say "kept: $STATE, ~/.config/f1tenth (pin, unit environment), the images and the f1tenth_car_* volumes"
