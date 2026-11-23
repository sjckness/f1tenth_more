#!/usr/bin/env bash
# ok/ko ritornano sempre 0: `[ cond ] && ok .. || ko ..` qui e' un if-then-else vero.
# shellcheck disable=SC2015
# scripts/car/tests/car_scripts_test.sh - test degli script di scripts/car/: bash puro, nessuna rete,
# nessun ssh, < 15 s. Da linus: scripts/car/tests/car_scripts_test.sh
#
# Controlla: shellcheck e bash -n; image_commit / same_image / branch_of (un commit che cambia solo un
# profilo non chiede una nuova immagine); l'avanzamento della build su un job finto; car-stack
# (riscrittura di stack_args, rifiuto dentro il container, argomenti); car-container (argomenti);
# i controlli di ready su fatti finti (READY, e un FAIL per ogni cosa che manca); i codici d'uscita
# di build/deploy/ready/profiles con argomenti sbagliati.
set -uo pipefail
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)
mapfile -t SCRIPTS < <(find "$ROOT/scripts/car" -name '*.sh' | sort)
NOK=0 NFAIL=0
ok() { NOK=$((NOK + 1)); echo "ok   $1"; }
ko() { NFAIL=$((NFAIL + 1)); echo "FAIL $1"; [ -z "${2:-}" ] || printf '%s\n' "$2" | sed 's/^/     | /' | tail -15; }
t() { local name=$1; shift; local out; if out=$("$@" 2>&1); then ok "$name"; else ko "$name" "$out"; fi; }
TMP=$(mktemp -d)
trap 'rm -rf "${TMP:?}"' EXIT

# ---- statici
# shellcheck disable=SC2016  # "$@" e $f si espandono nel bash -c, apposta
t "bash -n (${#SCRIPTS[@]} file)" bash -c 'for f in "$@"; do bash -n "$f" || exit 1; done' _ "${SCRIPTS[@]}"
if command -v shellcheck >/dev/null; then
  # shellcheck disable=SC2016  # "$@" e $1 si espandono nel bash -c, apposta
  t "shellcheck" bash -c 'cd "$1" && shift && shellcheck -x "$@"' _ "$ROOT" "${SCRIPTS[@]#"$ROOT/"}"
elif docker image inspect koalaman/shellcheck:stable >/dev/null 2>&1; then
  t "shellcheck (docker)" docker run --rm -v "$ROOT:/mnt" -w /mnt koalaman/shellcheck:stable -x \
    "${SCRIPTS[@]#"$ROOT/"}"
else
  echo "skip shellcheck (ne' shellcheck ne' koalaman/shellcheck:stable)"
fi

# ---- orin_common.sh
export F1T_CORE=$ROOT/tools/core F1T_SCRIPT=car_scripts_test F1T_LOG_DIR=$TMP/logs
# shellcheck source=tools/core/lib/config.sh
source "$F1T_CORE/lib/config.sh"
cfg_load
# shellcheck source=scripts/car/lib/orin_common.sh
source "$ROOT/scripts/car/lib/orin_common.sh"
# shellcheck source=scripts/car/lib/car_ready.sh
source "$ROOT/scripts/car/lib/car_ready.sh"

t "same_image HEAD HEAD" same_image HEAD HEAD
t "image_commit HEAD ha lo stesso contenuto di HEAD" same_image "$(image_commit HEAD)" HEAD
want="$F1T_IMAGE_REPO:${F1T_IMAGE_PREFIX}168dc95"; [ "$F1T_IMAGE_RUST" = skip ] && want=$want-norust
[ "$(image_tag_of 168dc958abc)" = "$want" ] && ok "image_tag_of" || ko "image_tag_of" "$(image_tag_of 168dc958abc)"

# Repo finto: un commit che cambia solo un profilo, uno che cambia src/, uno che cambia scripts/car.
gr=$TMP/repo
g() { git -C "$gr" -c user.email=t@t -c user.name=t "$@"; }
mkdir -p "$gr/src/f1tenth_bringup/config/profiles" "$gr/scripts/car"
git -C "$gr" init -q
echo a > "$gr/src/a.py"; echo 'launch_arguments: {}' > "$gr/src/f1tenth_bringup/config/profiles/p.yaml"
g add -A && g commit -q -m base
echo 'launch_arguments: {enable_comms: true}' > "$gr/src/f1tenth_bringup/config/profiles/p.yaml"
g commit -q -am profile
echo x > "$gr/scripts/car/x.sh"; g add -A; g commit -q -m script
echo b > "$gr/src/a.py"; g commit -q -am src
# Sul repo finto, non in una subshell (ok/ko devono contare): F1T_ROOT e CAR_TOOLS si ripristinano dopo.
SAVED_ROOT=$F1T_ROOT SAVED_TOOLS=("${CAR_TOOLS[@]}")
F1T_ROOT=$gr
same_image HEAD~3 HEAD~1 && ok "profilo e script: stessa immagine" || ko "profilo e script: stessa immagine"
same_image HEAD~1 HEAD && ko "src/ cambia l'immagine" || ok "src/ cambia l'immagine"
[ "$(image_commit HEAD~1)" = "$(git -C "$gr" rev-parse HEAD~3)" ] && ok "image_commit salta profilo e script" \
  || ko "image_commit salta profilo e script" "$(image_commit HEAD~1)"
# car_commit_gate: script dell'host assenti, poi committati, poi modifiche non committate.
CAR_TOOLS=(scripts/car/orin/stack.sh:car-stack)
H() { git -C "$gr" rev-parse "${1:-HEAD}"; }
{
  mkdir -p "$gr/scripts/car/orin"
  echo s > "$gr/scripts/car/orin/stack.sh"
  out=$(car_commit_gate "$(H)" all); rc=$?
  [ "$rc" = 1 ] && grep -q "non sono nel commit" <<<"$out" && grep -q "?? scripts/car/orin/stack.sh" <<<"$out" \
    && ok "commit_gate: script nuovo non committato -> rifiuta" || ko "commit_gate: script nuovo" "rc=$rc $out"
  car_commit_gate "$(H)" image >/dev/null && ok "commit_gate image: script non contano" || ko "commit_gate image: script non contano"
  g add -A >/dev/null; g commit -q -m tools
  car_commit_gate "$(H)" all >/dev/null && ok "commit_gate: tutto committato -> ok" || ko "commit_gate: tutto committato" "$(car_commit_gate "$(H)" all)"
  mkdir -p "$gr/src/nuovo"; echo z > "$gr/src/nuovo/z.py"
  car_commit_gate "$(H)" all >/dev/null && ok "commit_gate: file non tracciati in src/ ignorati" || ko "commit_gate: non tracciati in src/" "$(car_commit_gate "$(H)" all)"
  rm -rf "$gr/src/nuovo"
  echo c > "$gr/src/a.py"
  car_commit_gate "$(H)" image >/dev/null && ko "commit_gate: src/ modificato -> rifiuta" || ok "commit_gate: src/ modificato -> rifiuta"
  car_commit_gate "$(H HEAD~1)" all >/dev/null && ko "commit_gate: commit vecchio senza gli script -> rifiuta" || ok "commit_gate: commit vecchio senza gli script -> rifiuta"
  git -C "$gr" checkout -q -- src/a.py
  echo q > "$gr/src/f1tenth_bringup/config/profiles/p.yaml"
  car_commit_gate "$(H)" all >/dev/null && ko "commit_gate: profilo modificato -> rifiuta" || ok "commit_gate: profilo modificato -> rifiuta"
  car_commit_gate "$(H)" all --allow-dirty-profiles >/dev/null && ok "commit_gate: --allow-dirty-profiles" || ko "commit_gate: --allow-dirty-profiles"
  git -C "$gr" checkout -q -- .
}
F1T_ROOT=$SAVED_ROOT CAR_TOOLS=("${SAVED_TOOLS[@]}")
# shellcheck disable=SC2016  # il $( ) deve restare letterale: e' il nome del branch
weird='feat/a$(touch_x)&b'
git -C "$gr" checkout -q -b "$weird"
got=$(F1T_ROOT=$gr branch_of HEAD)
[[ $got =~ ^[A-Za-z0-9_.-]+$ ]] && ok "branch_of ripulisce '$weird' -> $got" || ko "branch_of ripulisce" "$got"
git -C "$gr" checkout -q --detach
got=$(F1T_ROOT=$gr branch_of HEAD)
[[ $got =~ ^[A-Za-z0-9_.-]+$ ]] && ok "branch_of con HEAD staccato -> $got" || ko "branch_of HEAD staccato" "$got"

# ---- avanzamento della build (orin_build_progress.sh) su un job finto
bt=$TMP/build; mkdir -p "$bt/abc1234/output/orin_container/logs"
echo running > "$bt/abc1234.status"; echo "11:00 [f1t build] build: ..." > "$bt/abc1234.log"
echo "#12 [ws 5/16] RUN colcon build" > "$bt/abc1234/output/orin_container/logs/build_abc1234_x.log"
p=$(bash "$ROOT/scripts/car/lib/orin_build_progress.sh" "$bt" abc1234)
[[ $p == running\|build\|5\|16\|*\|*"[ws 5/16]"* ]] && ok "build: passo 5/16 letto" || ko "build: passo 5/16" "$p"
echo "PKG src/foo rc=0 3 passed" >> "$bt/abc1234.log"
p=$(bash "$ROOT/scripts/car/lib/orin_build_progress.sh" "$bt" abc1234)
[[ $p == running\|test\|* ]] && ok "build: fase test riconosciuta" || ko "build: fase test" "$p"
echo "fail exit 1" > "$bt/abc1234.status"
p=$(bash "$ROOT/scripts/car/lib/orin_build_progress.sh" "$bt" abc1234)
[[ $p == "fail exit 1|fine|"* ]] && ok "build: stato fail finale" || ko "build: stato fail" "$p"

# ---- car-stack: stack_args
STACK=$ROOT/scripts/car/orin/stack.sh
af=$TMP/stack_args
sal() { bash -c 'source "$1"; stack_args_line "$2" "$3" "$4"' _ "$STACK" "$@"; }
rm -f "$af"
[ "$(sal "$af" p /d)" = "profile:=p profiles_dir:=/d" ] && ok "stack_args: assente -> profilo" || ko "stack_args: assente" "$(sal "$af" p /d)"
echo "guard:=shadow profile:=old profiles_dir:=/x corridor_impl:=rust" > "$af"
[ "$(sal "$af" new /d)" = "guard:=shadow corridor_impl:=rust profile:=new profiles_dir:=/d" ] \
  && ok "stack_args: tiene gli altri token, cambia il profilo" || ko "stack_args: altri token" "$(sal "$af" new /d)"
[ "$(sal "$af" '' /d)" = "guard:=shadow corridor_impl:=rust" ] && ok "stack_args: senza profilo toglie i token" \
  || ko "stack_args: senza profilo" "$(sal "$af" '' /d)"
printf 'a:=1\nb:=2\n' > "$af"
sal "$af" p /d >/dev/null 2>&1 && ko "stack_args: due righe rifiutate" || ok "stack_args: due righe rifiutate"
# shellcheck disable=SC2016  # il $( ) deve restare letterale: e' il token non valido
echo 'a:=1 $(rm)' > "$af"
sal "$af" p /d >/dev/null 2>&1 && ko "stack_args: token non valido rifiutato" || ok "stack_args: token non valido rifiutato"
# Il risultato rispetta la regola di ctl_container.env.
re=$(sed -n 's/^  _f1t_re=.\(.*\).$/\1/p' "$ROOT/src/f1tenth_ctl/config/ctl_container.env")
[ "$re" = "$(sed -n "s/^TOKEN_RE='\(.*\)'$/\1/p" "$STACK")" ] && ok "TOKEN_RE = regola di ctl_container.env" \
  || ko "TOKEN_RE = regola di ctl_container.env" "ctl: $re"

# ---- car-stack / car-container: lato sbagliato e argomenti
touch "$TMP/.dockerenv"
for s in stack container; do
  out=$(F1T_CONTAINER_MARKERS="$TMP/.dockerenv" bash "$ROOT/scripts/car/orin/$s.sh" status 2>&1); rc=$?
  [ "$rc" = 3 ] && grep -q "DENTRO il container" <<<"$out" && ok "car-$s nel container: exit 3" || ko "car-$s nel container: exit 3" "rc=$rc $out"
  out=$(bash "$ROOT/scripts/car/orin/$s.sh" boh 2>&1); rc=$?
  [ "$rc" = 2 ] && ok "car-$s comando sbagliato: exit 2" || ko "car-$s comando sbagliato: exit 2" "rc=$rc $out"
  out=$(bash "$ROOT/scripts/car/orin/$s.sh" -h 2>&1); rc=$?
  [ "$rc" = 0 ] && grep -q "status" <<<"$out" && ok "car-$s -h" || ko "car-$s -h" "$out"
done
out=$(F1T_CONTAINER_MARKERS=/nonesiste bash "$STACK" stop p 2>&1); rc=$?
[ "$rc" = 2 ] && ok "car-stack stop con un profilo: exit 2" || ko "car-stack stop con un profilo: exit 2" "rc=$rc $out"
out=$(F1T_CONTAINER_MARKERS=/nonesiste bash "$STACK" start a b 2>&1); rc=$?
[ "$rc" = 2 ] && ok "car-stack start con due profili: exit 2" || ko "car-stack start con due profili: exit 2" "rc=$rc $out"
out=$(F1T_CONTAINER_MARKERS=/nonesiste bash "$STACK" start --wait x 2>&1); rc=$?
[ "$rc" = 2 ] && ok "car-stack --wait non numerico: exit 2" || ko "car-stack --wait non numerico: exit 2" "rc=$rc $out"

# ---- ready_eval su fatti finti
F1T_ROOT=$ROOT
SHA=$(git -C "$ROOT" rev-parse HEAD)
# Strumenti che esistono in HEAD qualunque cosa sia committata (gli script nuovi possono non esserlo).
CAR_TOOLS=(scripts/orin/preflight.sh:car-preflight tools/f1t:car-stack)
PROF=$(car_profiles_sums | head -1)
declare -gA ORIN R
good_facts() {
  ORIN=([ctr_state]=running [ctr_health]=healthy [pin_id]=sha256:aaa [ctr_image_id]=sha256:aaa
        [pin_tag]=f1tenth:x [ctr_revision]=$SHA [ctr_tested]=true [ctr_stack_procs]=0
        [disk_root_free_gb]=50 [mem_avail_mb]=20000 [temp_max_c]=45.0 [time_sync]=yes [ctr_fs_changes]=0)
  R=([car_mode]=container [preflight_rc]=0 [preflight_msg]="PASS ZED;PASS lidar;PASS VESC;"
     [host_ros_procs]='' [stack_args]="profile:=${PROF%% *} profiles_dir:=/d" [stack_args_ok]=yes
     [scratch_free_gb]=400 [profile_present]=yes [profile_sha]=${PROF##* }
     [profile_check]="OK ${PROF%% *} ${PROF##* }" [profiles]="${PROF%% *}")
  local tl
  for tl in "${CAR_TOOLS[@]}"; do R[tool_${tl##*:}]=$(car_tool_sum "$SHA" "${tl%%:*}"); done
}
eval_counts() {   # eval_counts PROFILO -> "FAIL WARN"
  ( set -e; _CHK_NAME=() _CHK_VAL=() _CHK_STATE=(); ready_eval "$SHA" "$1" 2>/dev/null
    echo "$(checks_count FAIL) $(checks_count WARN)" )
}
good_facts
[ "$(eval_counts "${PROF%% *}")" = "0 0" ] && ok "ready: tutto a posto -> READY (0 FAIL, 0 WARN)" \
  || ko "ready: tutto a posto" "$(eval_counts "${PROF%% *}"); $( _CHK_STATE=(); ready_eval "$SHA" "${PROF%% *}" 2>&1 | grep -v '✔')"
[ "$(eval_counts -)" = "0 0" ] && ok "ready: senza profilo -> READY" || ko "ready: senza profilo" "$(eval_counts -)"
fails() {   # fails DESCRIZIONE ATTESO ARRAY CHIAVE VALORE [PROFILO]
  local desc=$1 want=$2; good_facts
  if [ "$3" = ORIN ]; then ORIN[$4]=$5; else R[$4]=$5; fi
  local got; got=$(eval_counts "${6:-${PROF%% *}}")
  [ "${got%% *}" = "$want" ] && ok "ready: $desc -> $want FAIL" || ko "ready: $desc -> $want FAIL" "got $got"
}
fails "modalita' nativa" 1 R car_mode native
fails "container non sano" 1 ORIN ctr_health unhealthy
fails "container fermo" 1 ORIN ctr_state exited
fails "pin diverso dal container" 1 ORIN pin_id sha256:bbb
fails "immagine di un altro contenuto" 1 ORIN ctr_revision "$(git -C "$ROOT" rev-parse "$(image_commit HEAD)~1")"
fails "car-stack vecchio" 1 R tool_car-stack deadbeef
fails "stack gia' in esecuzione" 1 ORIN ctr_stack_procs 3
fails "fatti vuoti (container assente)" 1 ORIN ctr_stack_procs ""
fails "ROS sull'host" 1 R host_ros_procs "123 "
fails "lidar assente" 1 R preflight_rc 1
fails "stack_args ignorato" 1 R stack_args_ok no
fails "profilo assente" 1 R profile_present no
fails "profilo diverso da linus" 1 R profile_sha abc
fails "profilo non valido" 1 R profile_check "ERROR profile 'p': component 'x' is not in components.yaml"
fails "scratch pieno" 1 R scratch_free_gb 3
fails "temperatura 90 C" 1 ORIN temp_max_c 90.0
fails "orologio non sincronizzato (solo WARN)" 0 ORIN time_sync no
good_facts; R[stack_args]="guard:=shadow profile:=x profiles_dir:=/d"
[ "$(eval_counts "${PROF%% *}")" = "0 1" ] && ok "ready: token che vincono sul profilo -> WARN" || ko "ready: token extra -> WARN" "$(eval_counts "${PROF%% *}")"

# ---- argomenti degli script di linus
for s in build deploy ready; do
  out=$(bash "$ROOT/scripts/car/$s.sh" --boh 2>&1); rc=$?
  [ "$rc" = 2 ] && ok "$s opzione sbagliata: exit 2" || ko "$s opzione sbagliata: exit 2" "rc=$rc $out"
  out=$(bash "$ROOT/scripts/car/$s.sh" nonesiste123 2>&1); rc=$?
  [ "$rc" = 2 ] && ok "$s commit sconosciuto: exit 2" || ko "$s commit sconosciuto: exit 2" "rc=$rc $out"
done
out=$(bash "$ROOT/scripts/car/ready.sh" --profile ../x 2>&1); rc=$?
[ "$rc" = 2 ] && ok "ready --profile con un percorso: exit 2" || ko "ready --profile con un percorso: exit 2" "rc=$rc $out"
out=$(bash "$ROOT/scripts/car/ready.sh" --profile nonesiste123 2>&1); rc=$?
[ "$rc" = 2 ] && ok "ready --profile sconosciuto: exit 2" || ko "ready --profile sconosciuto: exit 2" "rc=$rc $out"
out=$(bash "$ROOT/scripts/car/profiles.sh" boh 2>&1); rc=$?
[ "$rc" = 2 ] && ok "profiles comando sbagliato: exit 2" || ko "profiles comando sbagliato: exit 2" "rc=$rc $out"

echo "car_scripts_test: $NOK ok, $NFAIL falliti"
[ "$NFAIL" = 0 ]
