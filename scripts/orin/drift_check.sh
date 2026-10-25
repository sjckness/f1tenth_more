#!/usr/bin/env bash
# Runs on the Orin HOST: does the car's native checkout carry a commit, an
# uncommitted src/ change or an untracked src/ file that the container image
# (its CAR_SYNC.json) does not? Read-only. See drift_check.py.
#
#   scripts/orin/drift_check.sh --car-sync <CAR_SYNC.json> [--ws ~/dev_ws/f1tenth_more]
#
# One JSON object on stdout. Exit 0 clean, 3 drift, 2 error.
# scripts/docker_orin.sh up refuses on exit 3 unless --allow-drift.
set -euo pipefail
here="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec python3 "$here/drift_check.py" "$@"
