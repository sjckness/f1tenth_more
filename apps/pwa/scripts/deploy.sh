#!/usr/bin/env bash
# Copy the built app (dist/) to the Orin over Tailscale, into the directory the gateway
# serves (www_dir in src/f1tenth_gateway/config/gateway.yaml). Nothing restarts: the
# gateway reads the files per request and never caches index.html or sw.js, so the
# phone gets the new version the next time the app is opened.
#
#   npm run deploy                                    # build, then this
#   F1T_DEPLOY_HOST=jetson-orin F1T_DEPLOY_DIR=.local/share/f1tenth_gateway/www ./scripts/deploy.sh
#
# F1T_DEPLOY_HOST: an ssh destination (default jetson-orin, the alias in ~/.ssh/config);
# F1T_DEPLOY_DIR: relative to the remote home unless absolute.
set -euo pipefail
cd "$(dirname "$0")/.."

HOST="${F1T_DEPLOY_HOST:-jetson-orin}"
DIR="${F1T_DEPLOY_DIR:-.local/share/f1tenth_gateway/www}"

if [ ! -f dist/index.html ]; then
  echo "deploy: dist/index.html is missing: run npm run build (or npm run deploy)" >&2
  exit 1
fi
if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$HOST" true; then
  echo "deploy: cannot reach $HOST over ssh (Orin off? tailscale status)" >&2
  exit 1
fi

ssh -o BatchMode=yes "$HOST" "mkdir -p '$DIR'"
# --delete: the assets of older builds go (their file names change with every build).
rsync -rlt --delete --checksum --itemize-changes dist/ "$HOST:$DIR/"
echo "deploy: dist/ -> $HOST:$DIR (close and reopen the app on the iPhone)"
