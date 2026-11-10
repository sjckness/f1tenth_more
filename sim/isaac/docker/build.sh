#!/usr/bin/env bash
# Build the Isaac Sim image for the HIL sim (workstation only; nothing of this runs on the car).
#
#   sim/isaac/docker/build.sh [--tag NAME:TAG] [--network host]
#
# The base image nvcr.io/nvidia/isaac-sim:5.1.0 is ~20 GB and needs an NGC login:
#   docker login nvcr.io        (username '$oauthtoken', password = your NGC API key)
# Default tag f1tenth_isaac:5.1.0, which is config/hil.yaml -> isaac.image. If the machine already
# has the isaac_ws devcontainer image, retag it instead of building:
#   docker tag isaac_ws_devcontainer-isaac-sim:latest f1tenth_isaac:5.1.0
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TAG=f1tenth_isaac:5.1.0
NET=()
while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="${2:?--tag needs NAME:TAG}"; shift 2 ;;
    --network) NET=(--network "${2:?--network needs a value}"); shift 2 ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "build.sh: unknown argument '$1'" >&2; exit 2 ;;
  esac
done
echo "building $TAG from $HERE (context: $(dirname "$HERE"))"
exec docker build "${NET[@]}" -t "$TAG" -f "$HERE/Dockerfile" "$(dirname "$HERE")"
