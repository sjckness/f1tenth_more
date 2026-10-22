#!/bin/bash
# f1tenth_comms under ROS 2 Humble / Python 3.10 (the Orin's), in the
# f1tenth_more:cpu image, on linus: colcon build of f1tenth_messages and
# f1tenth_comms from a copy of the checkout, then `colcon test` of
# f1tenth_comms. The container has no network at all (--network none): the
# node tests and the heartbeat tests only ever use loopback, and nothing can
# reach another graph or the car.
#
#   tools/test_comms_humble.sh [scratch dir, default /tmp/f1t_comms_humble] [-- extra command]
#
# An extra command runs after the tests, in the same environment (e.g. the
# synthetic run: -- python3 /work/src/tools/comms_synthetic_run.py ...).
set -e
repo="$(cd "$(dirname "$0")/.." && pwd)"
work="${1:-/tmp/f1t_comms_humble}"
shift || true
[ "${1:-}" = "--" ] && shift
mkdir -p "$work"
if [ $# -gt 0 ]; then printf '%q ' "$@" > "$work/extra.cmd"; else : > "$work/extra.cmd"; fi
cat > "$work/run.sh" <<'INNER'
set -e
source /opt/ros/humble/setup.bash
mkdir -p /work/src/src /work/src/tools /work/src/scripts
rm -rf /work/src/src/f1tenth_comms /work/src/src/f1tenth_messages /work/src/scripts/base /work/src/tools/comms_*.py
cp -r /src/src/f1tenth_messages /src/src/f1tenth_comms /work/src/src/
cp -r /src/scripts/base /work/src/scripts/ 2>/dev/null || true
cp /src/tools/comms_*.py /work/src/tools/ 2>/dev/null || true
cd /work
colcon build --base-paths /work/src/src --build-base /work/build --install-base /work/install \
  --packages-select f1tenth_messages f1tenth_comms \
  --event-handlers console_cohesion- status- > /work/build.log 2>&1 || { tail -30 /work/build.log; exit 1; }
source /work/install/setup.bash
echo "== $(python3 --version), $(python3 -m pytest --version 2>&1)"
colcon test --base-paths /work/src/src --build-base /work/build --install-base /work/install \
  --packages-select f1tenth_comms --event-handlers console_cohesion- status- > /work/test.log 2>&1 || true
colcon test-result --test-result-base /work/build/f1tenth_comms --verbose
extra="$(cat /work/extra.cmd)"
if [ -n "$extra" ]; then
  echo "== extra: $extra"
  eval "$extra"
fi
INNER
docker run --rm --network none -v "$repo:/src:ro" -v "$work:/work" f1tenth_more:cpu bash /work/run.sh
