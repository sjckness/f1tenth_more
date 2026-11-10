#!/usr/bin/env bash
# open_graph.sh -- open a saved Hydra scene graph in the Python viewer (hydra_dsg_viewer).
#
# Usage: scripts/hydra/open_graph.sh <file.sparkdsg|file.bin|run_dir> [viewer options]
#   run_dir: a mission run folder (log_scene_graph:=true); opens scene_graph/dsg_final.*, or the
#            newest snapshot if the run ended without one. Also accepts a Hydra log dir
#            (~/.hydra/f1tenth/<profile>_<stamp>, its backend/dsg*.json|.sparkdsg).
#   viewer options: --summary, --png out.png, --z-offset 3, --backend matplotlib, ...
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
target="${1:?usage: open_graph.sh <file|run_dir> [viewer options]}"; shift
if [ -d "$target" ]; then
  file="$(ls -1 "$target"/scene_graph/dsg_final.* 2>/dev/null | head -1)"
  [ -n "$file" ] || file="$(ls -1t "$target"/scene_graph/dsg_t*.* 2>/dev/null | head -1)"
  [ -n "$file" ] || file="$(find "$target" -maxdepth 3 \( -name '*.sparkdsg' -o -name 'dsg*.json' \) \
                           -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  [ -n "$file" ] || { echo "open_graph.sh: no scene graph under $target" >&2; exit 1; }
else
  file="$target"
fi
echo "open_graph.sh: $file"
# shellcheck disable=SC1091
set +u; source "$HERE/hydra_env.sh"; set -u
exec python3 -m f1tenth_hydra.dsg_viewer --file "$file" "$@"
