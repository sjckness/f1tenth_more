#!/bin/bash
# Gira SULL'ORIN: versioni dei moduli dentro un'immagine o nel container in esecuzione, righe
# "nome versione" lette dai package.xml in /opt/f1tenth_ws/src. Solo letture.
#   modules.sh --running CONTAINER EXCLUDE_RE
#   modules.sh --image IMAGE EXCLUDE_RE        (container usa e getta, rete none)
set -euo pipefail
MODE=$1 TARGET=$2 EXCL=$3
# shellcheck disable=SC2016  # expands inside the container
INNER='find /opt/f1tenth_ws/src -name package.xml 2>/dev/null | grep -v -E "$EXCL" | while read -r f; do
  n=$(sed -n "s:.*<name>[[:space:]]*\([^<[:space:]]*\).*:\1:p" "$f" | head -1)
  v=$(sed -n "s:.*<version>[[:space:]]*\([^<[:space:]]*\).*:\1:p" "$f" | head -1)
  [ -n "$n" ] && echo "$n ${v:-?}"
done | sort'
case "$MODE" in
  --running) docker exec -e EXCL="$EXCL" "$TARGET" sh -c "$INNER" < /dev/null ;;
  --image) docker run --rm --network none -e EXCL="$EXCL" --entrypoint sh "$TARGET" -c "$INNER" < /dev/null ;;
  *) echo "uso: modules.sh --running CONTAINER|--image IMAGE EXCLUDE_RE" >&2; exit 2 ;;
esac
