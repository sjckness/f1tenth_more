#!/usr/bin/env bash
# Build this workspace with --symlink-install against a known-good setuptools.
#
# Why this exists: colcon runs every python package with
# `python setup.py develop ...` (`colcon_core/task/python/build.py:222-227`),
# and setuptools >= 80 dropped `develop --uninstall/--editable/--build-directory`.
# The 84.2.1 sitting in ~/.local/lib/python3.12/site-packages (a `pip --user`
# upgrade) therefore breaks `colcon build` for *every* python package here, and
# for anything else built from /usr/bin/python3.
#
# Fix, without touching the system: run colcon from a venv that
#   * inherits /usr/lib/python3/dist-packages (setuptools 68.1.2, colcon, rclpy),
#   * ignores ~/.local via PYTHONNOUSERSITE=1,
# and therefore hands `setup.py` the distro's setuptools. Verified, not assumed:
# the version is asserted below before any building happens.
#
# Note `_PYTHON_CMD = [sys.executable, ...]`: colcon uses the interpreter it is
# itself running under, which is why /usr/bin/colcon (shebang /usr/bin/python3)
# cannot be used and $venv/bin/colcon is generated instead.
#
# Usage: scripts/colcon_build.sh [colcon args...]
#   scripts/colcon_build.sh                                  # full build
#   scripts/colcon_build.sh build --packages-select llm      # one package
#   scripts/colcon_build.sh list                             # any verb works
# `build` gets --symlink-install added unless you pass it yourself (CLAUDE.md
# "Rebuild policy": the workspace is a symlink install, keep it that way).

set -euo pipefail

ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv="$ws/.venv-build"
ros_distro="${ROS_DISTRO:-jazzy}"

# 1. Create the venv once. --without-pip because python3.12-venv (ensurepip) is
#    not installed here and installing it would be a system-wide apt change.
#    --system-site-packages so colcon, rclpy and the distro setuptools are seen.
if [ ! -x "$venv/bin/python3" ]; then
  echo "==> creating $venv (python3 -m venv --without-pip --system-site-packages)"
  python3 -m venv --without-pip --system-site-packages "$venv"
fi

# 2. Ignore ~/.local. This is the whole fix: it drops setuptools 84.2.1 from
#    sys.path so the distro's 68.1.2 wins.
export PYTHONNOUSERSITE=1

# 3. colcon entry point bound to the venv interpreter (same call as the
#    console_script /usr/bin/colcon makes, minus its /usr/bin/python3 shebang).
if [ ! -x "$venv/bin/colcon" ]; then
  cat > "$venv/bin/colcon" <<EOF
#!$venv/bin/python3
import sys

from colcon_core.command import main

sys.argv[0] = 'colcon'
sys.exit(main())
EOF
  chmod +x "$venv/bin/colcon"
fi

# 4. Assert the toolchain actually is the one we expect -- a silent regression
#    here surfaces later as `error: option --uninstall not recognized`.
read -r st_version st_path < <(
  "$venv/bin/python3" -c 'import setuptools; print(setuptools.__version__, setuptools.__file__)'
)
st_major="${st_version%%.*}"
if [ "$st_major" -ge 80 ]; then
  cat >&2 <<EOF
error: setuptools $st_version is too new, and colcon needs < 80 with
       \`setup.py develop --uninstall\` (\`colcon_core/task/python/build.py:222\`).
       Resolved from: $st_path
       Something reintroduced a setuptools >= 80 on sys.path despite
       PYTHONNOUSERSITE=1 (or apt upgraded python3-setuptools).
EOF
  exit 1
fi
echo "==> setuptools $st_version ($st_path)"

# 5. Clean ROS environment: a shell that sourced some *other* workspace must not
#    leak its PYTHONPATH/AMENT_PREFIX_PATH into the python builds.
for var in PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; do
  unset "$var" || true
done
set +u  # ROS setup files read unset variables (CLAUDE.md)
# shellcheck disable=SC1091
source "/opt/ros/$ros_distro/setup.bash"
set -u

# 6. Default verb, and keep the symlink-install policy.
args=("$@")
if [ ${#args[@]} -eq 0 ]; then
  args=(build)
fi
if [ "${args[0]}" = "build" ]; then
  case " ${args[*]} " in
    *" --symlink-install "*) ;;
    *) args+=("--symlink-install") ;;
  esac
fi

echo "==> colcon ${args[*]}"
exec "$venv/bin/colcon" "${args[@]}"
