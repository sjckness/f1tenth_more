#!/usr/bin/env bash
# ok/ko ritornano sempre 0: `[ cond ] && ok .. || ko ..` qui e' un if-then-else vero.
# shellcheck disable=SC2015
# scripts/tests/orin_scripts_test.sh - test degli script scripts/orin_*.sh: bash puro, nessuna rete,
# nessun ssh, < 15 s. Da linus: scripts/tests/orin_scripts_test.sh
#
# Controlla: shellcheck e bash -n; image_commit / same_image / branch_of sulla storia del repo; i
# controlli del precheck su riepiloghi di esempio (pronto, sbagliato, container fermo); orin_run_tests
# che rifiuta di girare dentro il container e i codici d'uscita delle opzioni sbagliate.
set -uo pipefail
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)
SCRIPTS=("$ROOT"/scripts/orin_deploy.sh "$ROOT"/scripts/orin_run_tests.sh "$ROOT"/scripts/orin_precheck.sh \
         "$ROOT"/scripts/lib/orin_ui.sh "$ROOT"/scripts/lib/orin_common.sh "$ROOT"/scripts/lib/orin_summary.sh \
         "$ROOT"/scripts/lib/orin_build_progress.sh "$ROOT"/scripts/lib/orin_images.sh \
         "$ROOT"/scripts/tests/orin_scripts_test.sh)
NOK=0 NFAIL=0
ok() { NOK=$((NOK + 1)); echo "ok   $1"; }
ko() { NFAIL=$((NFAIL + 1)); echo "FAIL $1"; [ -z "${2:-}" ] || printf '%s\n' "$2" | sed 's/^/     | /' | tail -15; }
t() { local name=$1; shift; local out; if out=$("$@" 2>&1); then ok "$name"; else ko "$name" "$out"; fi; }

# ---- statici
# shellcheck disable=SC2016  # "$@" e $f si espandono nel bash -c, apposta
t "bash -n" bash -c 'for f in "$@"; do bash -n "$f" || exit 1; done' _ "${SCRIPTS[@]}"
if command -v shellcheck >/dev/null; then
  t "shellcheck" shellcheck -x "${SCRIPTS[@]}"
elif docker image inspect koalaman/shellcheck:stable >/dev/null 2>&1; then
  t "shellcheck (docker)" docker run --rm -v "$ROOT:/mnt" -w /mnt koalaman/shellcheck:stable -x \
    "${SCRIPTS[@]#"$ROOT/"}"
else
  echo "skip shellcheck (ne' shellcheck ne' koalaman/shellcheck:stable)"
fi

# ---- orin_common.sh sulla storia del repo
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=orin_scripts_test F1T_LOG_DIR=${TMPDIR:-/tmp}/orin_scripts_test
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
cfg_load
# shellcheck source=scripts/lib/orin_common.sh
source "$ROOT/scripts/lib/orin_common.sh"
# shellcheck source=scripts/lib/orin_summary.sh
source "$ROOT/scripts/lib/orin_summary.sh"

t "same_image HEAD HEAD" same_image HEAD HEAD
t "image_commit HEAD ha lo stesso contenuto di HEAD" same_image "$(image_commit HEAD)" HEAD
# Un commit che cambia solo scripts/orin_* (b566ab70) e uno che cambia src/ (01b8bc04).
if git -C "$ROOT" cat-file -e 'b566ab70^{commit}' 2>/dev/null; then
  t "commit solo-script: stessa immagine del padre" same_image b566ab70 b566ab70^
  if same_image 01b8bc04 01b8bc04^; then ko "src/ cambia l'immagine"; else ok "src/ cambia l'immagine"; fi
fi
want="$F1T_IMAGE_REPO:${F1T_IMAGE_PREFIX}168dc95"; [ "$F1T_IMAGE_RUST" = skip ] && want=$want-norust
if [ "$(image_tag_of 168dc958abc)" = "$want" ]; then ok "image_tag_of"; else ko "image_tag_of" "$(image_tag_of 168dc958abc)"; fi
# branch_of su un repo finto: nome con caratteri da shell, e HEAD staccato.
gr=$(mktemp -d)
git -C "$gr" init -q && git -C "$gr" -c user.email=t@t -c user.name=t commit -q --allow-empty -m x
# shellcheck disable=SC2016  # il $( ) deve restare letterale: e' il nome del branch
weird='feat/a$(touch_x)&b'
git -C "$gr" checkout -q -b "$weird"
got=$(F1T_ROOT=$gr branch_of HEAD)
[[ $got =~ ^[A-Za-z0-9_.-]+$ ]] && ok "branch_of ripulisce '$weird' -> $got" || ko "branch_of ripulisce" "$got"
git -C "$gr" checkout -q --detach
got=$(F1T_ROOT=$gr branch_of HEAD)
[[ $got =~ ^[A-Za-z0-9_.-]+$ ]] && ok "branch_of con HEAD staccato -> $got" || ko "branch_of HEAD staccato" "$got"
rm -rf "$gr"
b=$(branch_of HEAD)
if [ -n "$b" ] && [ "$b" != detached ]; then ok "branch_of HEAD = $b"; else ko "branch_of HEAD" "$b"; fi

# ---- controlli del riepilogo (summary_checks) su esempi
REC=()
rec() { REC+=("$1|$2|$3"); }
skip() { REC+=("SKIP|$1|$2"); }
count() { local n=0 r; for r in "${REC[@]}"; do [ "${r%%|*}" = "$1" ] && n=$((n + 1)); done; echo "$n"; }
GOOD="sha $(git -C "$ROOT" rev-parse HEAD)
branch campaign3-rrt
stack_args profile:=campaign3_rrt
stack_cmd ros2 launch f1tenth_bringup supervisor_bringup.launch.py enable_intelligence:=true profile:=campaign3_rrt
nodes_total 64
planner corridor_rrt (parametro del supervisor)
logger test_campaign_logger on
logger mission_logger_node on
node_ok /component_supervisor_node
node_ok /corridor_rrt
llm_model qwen25_3b_instruct /home/fabiocar/projects/llm/models/q.gguf present 2.3GB
llama_server up (port 8083)
foxglove up (ws://<orin>:8765)
archive /home/fabiocar/f1tenth_archive 468G liberi"
REC=(); summary_checks "$GOOD" HEAD 0
[ "$(count OK)" = 11 ] && ok "riepilogo pronto: 11 OK" || ko "riepilogo pronto: 11 OK" "$(printf '%s\n' "${REC[@]}")"
BAD=$(sed -e 's/^planner .*/planner corridor_mpc (parametro del supervisor)/' \
          -e 's/^logger mission_logger_node on/logger mission_logger_node off/' \
          -e 's/^stack_args .*/stack_args enable_test_campaign_logger:=true/' \
          -e 's/^foxglove up.*/foxglove down (8765)/' <<<"$GOOD")
BAD="$BAD
missing /test_campaign_logger"
REC=(); summary_checks "$BAD" HEAD 1
[ "$(count FAIL)" = 5 ] && ok "riepilogo sbagliato: 5 FAIL (nodi, planner, logger, stack_args, Foxglove)" \
  || ko "riepilogo sbagliato: 5 FAIL" "$(printf '%s\n' "${REC[@]}")"
REC=(); summary_checks "container fermo" HEAD skip
[ "$(count SKIP)" = 11 ] && ok "container fermo: 11 SKIP" || ko "container fermo: 11 SKIP" "$(printf '%s\n' "${REC[@]}")"
REC=(); summary_checks "container fermo" HEAD 1
[ "$(count FAIL)" = 1 ] && [ "$(count SKIP)" = 10 ] && ok "container fermo dopo un avvio: FAIL, non OK" \
  || ko "container fermo dopo un avvio: FAIL" "$(printf '%s\n' "${REC[@]}")"
[ "$(summary_val "$GOOD" planner)" = "corridor_rrt (parametro del supervisor)" ] && ok "summary_val" || ko "summary_val"

# ---- avanzamento della build (scripts/lib/orin_build_progress.sh) su un job finto
bt=$(mktemp -d); mkdir -p "$bt/abc1234/output/orin_container/logs"
echo running > "$bt/abc1234.status"; echo "11:00 [f1t build] build: ..." > "$bt/abc1234.log"
echo "#12 [ws 5/16] RUN colcon build" > "$bt/abc1234/output/orin_container/logs/build_abc1234_x.log"
p=$(bash "$ROOT/scripts/lib/orin_build_progress.sh" "$bt" abc1234)
[[ $p == running\|build\|5\|16\|*\|*"[ws 5/16]"* ]] && ok "build: passo 5/16 letto" || ko "build: passo 5/16" "$p"
echo "PKG src/foo rc=0 3 passed" >> "$bt/abc1234.log"
p=$(bash "$ROOT/scripts/lib/orin_build_progress.sh" "$bt" abc1234)
[[ $p == running\|test\|* ]] && ok "build: fase test riconosciuta" || ko "build: fase test" "$p"
echo "fail exit 1" > "$bt/abc1234.status"
p=$(bash "$ROOT/scripts/lib/orin_build_progress.sh" "$bt" abc1234)
[[ $p == "fail exit 1|fine|"* ]] && ok "build: stato fail finale" || ko "build: stato fail" "$p"
rm -rf "$bt"

# ---- orin_run_tests: lato sbagliato e argomenti
tmp=$(mktemp -d); touch "$tmp/.dockerenv"
out=$(F1T_CONTAINER_MARKERS="$tmp/.dockerenv" bash "$ROOT/scripts/orin_run_tests.sh" 2>&1); rc=$?
[ "$rc" = 3 ] && grep -q "DENTRO il container" <<<"$out" && ok "orin_run_tests nel container: exit 3" || ko "orin_run_tests nel container: exit 3" "rc=$rc $out"
out=$(bash "$ROOT/scripts/orin_run_tests.sh" --boh 2>&1); rc=$?
[ "$rc" = 2 ] && ok "orin_run_tests opzione sbagliata: exit 2" || ko "orin_run_tests opzione sbagliata: exit 2" "rc=$rc $out"
out=$(bash "$ROOT/scripts/orin_run_tests.sh" -h 2>&1); rc=$?
[ "$rc" = 0 ] && grep -q -- "--summary" <<<"$out" && ok "orin_run_tests -h" || ko "orin_run_tests -h" "$out"
rm -rf "$tmp"
for s in orin_deploy orin_precheck; do
  out=$(bash "$ROOT/scripts/$s.sh" --boh 2>&1); rc=$?
  [ "$rc" = 2 ] && ok "$s opzione sbagliata: exit 2" || ko "$s opzione sbagliata: exit 2" "rc=$rc $out"
  out=$(bash "$ROOT/scripts/$s.sh" nonesiste123 2>&1); rc=$?
  [ "$rc" = 2 ] && ok "$s commit sconosciuto: exit 2" || ko "$s commit sconosciuto: exit 2" "rc=$rc $out"
done

echo "orin_scripts_test: $NOK ok, $NFAIL falliti"
[ "$NFAIL" = 0 ]
