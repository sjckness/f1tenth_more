#!/usr/bin/env bash
# Build the F1TENTH dev container image(s).
# Usage: ./scripts/docker_build.sh {cpu|gpu|all}
set -euo pipefail

cd "$(dirname "$0")/.."   # repo root (where docker-compose.yml lives)

TARGET="${1:-}"

case "$TARGET" in
  cpu) docker compose build f1tenth_cpu ;;
  gpu) docker compose build f1tenth_gpu ;;
  all) docker compose build ;;
  *)
    echo "Usage: $0 {cpu|gpu|all}" >&2
    exit 1
    ;;
esac

cat <<EOF

Build complete for target: ${TARGET}

Next steps:
  Enter the container:   ./scripts/docker_run.sh ${TARGET/all/cpu}
  Inside the container, first time only:
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash
EOF
