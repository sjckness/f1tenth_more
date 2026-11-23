#!/usr/bin/env bash
# Gira SULL'ORIN (ssh bash -s < questo, da scripts/car/lib/orin_common.sh): le immagini dell'auto.
#   orin_images.sh REPO PREFIX
# Una riga per immagine REPO:PREFIX<sha7>[-norust]: "TAG REVISION TESTED PINNED", dove REVISION e' la
# label org.opencontainers.image.revision (sha completo), TESTED la label f1tenth.tested (o "-"),
# PINNED "pin" se e' l'immagine di ~/.config/f1tenth/car_image (o "-"). Prima le piu' recenti.
set -euo pipefail
repo=$1 prefix=$2
pin=$(awk '$1=="tag"{print $2}' "$HOME/.config/f1tenth/car_image" 2>/dev/null || true)
docker images --format '{{.Repository}}:{{.Tag}}' "$repo" | grep -E "^${repo}:${prefix}[0-9a-f]{7}(-norust)?\$" \
  | while read -r tag; do
      rev=$(docker image inspect -f '{{index .Config.Labels "org.opencontainers.image.revision"}}' "$tag" 2>/dev/null || true)
      tested=$(docker image inspect -f '{{index .Config.Labels "f1tenth.tested"}}' "$tag" 2>/dev/null || true)
      echo "$tag ${rev:--} ${tested:--} $([ "$tag" = "$pin" ] && echo pin || echo -)"
    done
