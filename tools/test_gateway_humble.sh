#!/bin/bash
# The gateway's and f1tenth_ctl's tests under ROS 2 Humble / Python 3.10 (the
# Orin's), in the f1tenth_more:cpu image, on linus. Builds f1tenth_messages in
# a scratch dir, installs the gateway's pip packages there, never touches the
# checkout (mounted read-only) and never joins another graph (domain 161,
# localhost only).
#
#   tools/test_gateway_humble.sh [scratch dir, default /tmp/f1t_humble]
set -e
repo="$(cd "$(dirname "$0")/.." && pwd)"
work="${1:-/tmp/f1t_humble}"
mkdir -p "$work"
cat > "$work/run.sh" <<'INNER'
set -e
source /opt/ros/humble/setup.bash
cd /work
[ -d /work/install/f1tenth_messages ] || colcon build --base-paths /src/src/f1tenth_messages \
  --build-base /work/build --install-base /work/install \
  --event-handlers console_cohesion- status- > /work/build.log 2>&1
source /work/install/setup.bash
[ -d /work/pydeps/fastapi ] || python3 -m pip install -q --target /work/pydeps \
  -r /src/src/f1tenth_gateway/requirements.txt httpx==0.28.1 > /work/pip.log 2>&1
[ -d /work/pydeps/pytest_timeout.py ] || python3 -m pip install -q --no-deps \
  --target /work/pydeps pytest-timeout==2.1.0 >> /work/pip.log 2>&1
rm -rf /work/pydeps/pytest /work/pydeps/_pytest /work/pydeps/pluggy
export PYTHONPATH=/work/pydeps:/src/src/f1tenth_gateway:/src/src/f1tenth_ctl:/src/src/f1tenth_telegram:$PYTHONPATH
export ROS_DOMAIN_ID=161 ROS_LOCALHOST_ONLY=1
echo "== $(python3 --version), $(python3 -m pytest -p no:anyio --version 2>&1)"
cd /src/src/f1tenth_gateway
python3 -m pytest -q -p no:cacheprovider -p no:anyio test --timeout 60 --timeout-method thread -rfE
cd /src/src/f1tenth_ctl
python3 -m pytest -q -p no:cacheprovider -p no:anyio test --timeout 120 -rfE
INNER
docker run --rm --network host -v "$repo:/src:ro" -v "$work:/work" f1tenth_more:cpu bash /work/run.sh
