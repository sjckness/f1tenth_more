#!/usr/bin/env bash
# Test di tools/core in bash puro (bats non serve). Ogni test gira su un repo finto in una cartella
# temporanea (copia di tools/, un package, un report, tag), con HOME finta e ssh/scp/ping finti che
# falliscono subito: nessuna rete, nessun Orin, niente scritto nel repo vero. Durata < 30 s.
#   tools/core/tests/run.sh [-v] [FILTRO]
set -uo pipefail
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
SRC_TOOLS=$(cd "$HERE/../.." && pwd)
VERBOSE=0 FILTER=''
for a in "$@"; do case "$a" in -v) VERBOSE=1 ;; -h|--help) sed -n '2,5p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; *) FILTER=$a ;; esac; done

TMP=$(mktemp -d "${TMPDIR:-/tmp}/f1t-selftest.XXXXXX")
cleanup() { case "$TMP" in "${TMPDIR:-/tmp}"/f1t-selftest.*) rm -rf -- "${TMP:?}" ;; esac; }
trap cleanup EXIT
export HOME=$TMP/home NO_COLOR=1 GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=$TMP/gitconfig
export GIT_AUTHOR_NAME=selftest GIT_AUTHOR_EMAIL=selftest@example.invalid
export GIT_COMMITTER_NAME=selftest GIT_COMMITTER_EMAIL=selftest@example.invalid
unset F1T_ROOT F1T_CORE F1T_SCRIPT F1T_LOG_DIR F1T_LOG_FILE ROS_DOMAIN_ID RMW_IMPLEMENTATION
mkdir -p "$HOME" "$TMP/stubs"
printf '[init]\n\tdefaultBranch = main\n[advice]\n\tdetachedHead = false\n' > "$GIT_CONFIG_GLOBAL"

# ---- comandi finti: l'Orin non risponde, nessun ping passa
for c in ssh scp; do
  printf '#!/bin/sh\necho "%s: connect to host orin-finto port 22: Connection refused" >&2\nexit 255\n' "$c" > "$TMP/stubs/$c"
done
cat > "$TMP/stubs/ping" <<'EOF'
#!/bin/sh
echo "--- finto ping statistics ---"
echo "3 packets transmitted, 0 received, 100% packet loss, time 2002ms"
exit 1
EOF
chmod +x "$TMP/stubs"/*
export PATH=$TMP/stubs:$PATH

# ---- repo finto
make_repo() {
  local r=$1
  mkdir -p "$r"
  git -C "$r" init -q
  mkdir -p "$r/tools" "$r/src/pkg_a/config" "$r/src/pkg_a/test" "$r/docs/analysis/_template" "$r/docs/analysis/mpc-control/2026-01-01_old" "$r/docs/modules" "$r/docker/orin"
  cp -a "$SRC_TOOLS/f1t" "$SRC_TOOLS/core" "$r/tools/"
  rm -rf "$r/tools/core/tests" "$r/tools/ext"; mkdir -p "$r/tools/ext"
  cat > "$r/src/pkg_a/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>pkg_a</name>
  <version>0.1.0</version>
</package>
EOF
  printf "from setuptools import setup\nsetup(name='pkg_a', version='0.1.0')\n" > "$r/src/pkg_a/setup.py"
  cat > "$r/src/pkg_a/CHANGELOG.md" <<'EOF'
# Changelog: `pkg_a`

## [Unreleased]

## [0.1.0] - 2026-01-02

### Changed
- Seconda versione.

## [0.0.1] - 2026-01-01

### Added
- Prima versione.
EOF
  printf 'gain: 1.0\n' > "$r/src/pkg_a/config/params.yaml"
  cp "$SRC_TOOLS/../docs/analysis/_template/report.md" "$r/docs/analysis/_template/report.md" 2>/dev/null \
    || printf -- '---\nid: x\nmodules: {}\nbranch: x\ncommit: x\nstatus: open\n---\n# T\n\n## Setup\n\nx\n\n## Metodo\n' > "$r/docs/analysis/_template/report.md"
  cat > "$r/docs/analysis/mpc-control/2026-01-01_old/report.md" <<'EOF'
---
id: 2026-01-01_old
title: "Prova vecchia"
area: mpc-control
date: 2026-01-01
tags: [x]
modules: {pkg_a: "0.1.0"}
branch: "main"
commit: "abc"
status: done
hypothesis: "h"
result: "risultato-di-prova 42"
decision: "decisione-di-prova"
raw_data: "n/d"
---
# Prova vecchia
EOF
  printf '#!/bin/bash\n# finto\n--dry-run\n' > "$r/docker/orin/build.sh"; chmod +x "$r/docker/orin/build.sh"
  printf 'logs/\n__pycache__/\n' > "$r/.gitignore"
  git -C "$r" add -A && git -C "$r" commit -q -m "v0.0.1 base"
  git -C "$r" tag -a pkg_a/v0.0.1 -m "pkg_a 0.0.1"
  printf 'gain: 2.0\n' > "$r/src/pkg_a/config/params.yaml"
  git -C "$r" commit -q -am "gain 2.0"
  git -C "$r" tag -a pkg_a/v0.1.0 -m "pkg_a 0.1.0"
  git -C "$r" update-ref refs/remotes/origin/main HEAD
}

# ---- asserzioni
OUT='' RC=0
run() { OUT=$("$@" 2>&1); RC=$?; }
fail() { echo "    FALLITO: $*"; [ "$VERBOSE" = 0 ] || printf '%s\n' "$OUT" | sed 's/^/    | /' | tail -40; return 1; }
assert_rc() { [ "$RC" = "$1" ] || fail "exit $RC, atteso $1 ($2)"; }
assert_in() { grep -qF -- "$1" <<< "$OUT" || fail "manca '$1' nell'output"; }
assert_not_in() { ! grep -qF -- "$1" <<< "$OUT" || fail "c'e' '$1' nell'output"; }
assert_clean() {  # niente errori di bash: un FAIL deve essere pulito
  local p
  for p in 'unbound variable' 'command not found' 'syntax error' 'Traceback' 'bad substitution' 'integer expression expected'; do
    assert_not_in "$p" || return 1
  done
}

passed=0 failed=0 names=()
for f in "$HERE"/test_*.sh; do
  # shellcheck disable=SC1090
  source "$f"
done
mapfile -t names < <(declare -F | awk '{print $3}' | grep '^test_' | sort)
t0=$(date +%s)
for t in "${names[@]}"; do
  [ -z "$FILTER" ] || [[ "$t" == *"$FILTER"* ]] || continue
  REPO=$TMP/repo_$t
  make_repo "$REPO" >/dev/null 2>&1 || { echo "FAIL $t (repo finto non creato)"; failed=$((failed + 1)); continue; }
  # shellcheck disable=SC2034  # usata dai test (test_*.sh)
  F1T=$REPO/tools/f1t
  if ( set +e; cd "$REPO" && "$t" ); then echo "ok   $t"; passed=$((passed + 1))
  else echo "FAIL $t"; failed=$((failed + 1)); fi
done
echo "selftest: $passed ok, $failed falliti in $(( $(date +%s) - t0 )) s"
[ "$failed" = 0 ]
