# shellcheck shell=bash
# Test dei comandi f1t (sorgente di tools/core/tests/run.sh: REPO, F1T, run, assert_* da li').

# VERSION del repo finto con la patch + 1 (i test non dipendono dal numero vero).
next_version() { awk -F. '{printf "%d.%d.%d\n", $1, $2, $3 + 1}' "$1/tools/core/VERSION"; }

test_help_lists_every_command() {
  run "$F1T" help
  assert_rc 0 help || return 1
  local f n
  for f in "$REPO"/tools/core/*/*; do
    [ -f "$f" ] && [ -x "$f" ] && sed -n '2,12p' "$f" | grep -q '^# desc: ' || continue
    n=$(sed -n '2,12s/^# desc: //p' "$f" | head -1)
    assert_in "$n" || return 1
  done
}

test_unknown_command_exit_2() {
  run "$F1T" nonesiste
  assert_rc 2 "comando sconosciuto" && assert_in "comando sconosciuto" && assert_clean
}

test_every_command_has_help() {
  local words
  while IFS=$'\t' read -r words _ _ kind; do
    [ "$kind" = 1 ] || continue
    # shellcheck disable=SC2086  # words e' "gruppo comando"
    run "$F1T" $words -h
    assert_rc 0 "$words -h" || return 1
    [ -n "$OUT" ] || { fail "$words -h non stampa niente"; return 1; }
    assert_clean || return 1
  done < <("$F1T" __index)
}

test_completion_words() {
  run "$F1T" __complete hil precheck -disp
  assert_in linus && assert_in work || return 1
  run "$F1T" __complete hil run
  assert_in smoke || return 1
  run "$F1T" __complete
  assert_in deploy && assert_in whatis
}

test_ext_dispatch() {
  mkdir -p "$REPO/tools/ext/demo"
  echo "Demo: prova di estensione" > "$REPO/tools/ext/demo/README.md"
  # shellcheck disable=SC2016  # $1 e' testo dello script generato
  printf '#!/bin/bash\necho "ciao $1"\n' > "$REPO/tools/ext/demo/hello.sh"; chmod +x "$REPO/tools/ext/demo/hello.sh"
  run "$F1T" ext demo hello mondo
  assert_rc 0 ext && assert_in "ciao mondo" || return 1
  run "$F1T" ext demo nope
  assert_rc 2 "ext comando sconosciuto" && assert_in hello || return 1
  run "$F1T" ext
  assert_in "demo"
}

test_config_schemas_valid() {
  local s
  for s in "$REPO"/tools/core/hil/scenarios/[!_]*.env; do
    run bash -c "F1T_CORE='$REPO/tools/core' F1T_ROOT='$REPO'; source \"\$F1T_CORE/lib/config.sh\"; cfg_load && cfg_load_host linus && cfg_load_scenario $(basename "$s" .env) && cfg_require config && cfg_require host && cfg_require scenario"
    assert_rc 0 "schema $(basename "$s")" || return 1
  done
}

test_config_missing_key_named() {
  sed -i 's/^STACK_TOPICS=.*/STACK_TOPICS=/' "$REPO/tools/core/hil/scenarios/smoke.env"
  run "$F1T" hil precheck -disp linus
  assert_rc 1 "chiave mancante" && assert_in "STACK_TOPICS" && assert_in "smoke.env" && assert_clean || return 1
  run "$F1T" hil precheck -disp work
  assert_rc 1 "host work incompleto" && assert_in "HOST_HOSTNAME" && assert_clean
}

test_env_overrides_config() {
  run bash -c "export F1T_ORIN_HOST=altro.example; F1T_CORE='$REPO/tools/core' F1T_ROOT='$REPO'; source \"\$F1T_CORE/lib/config.sh\"; cfg_load; echo \"host=\$F1T_ORIN_HOST\""
  assert_in "host=altro.example"
}

test_precheck_requires_disp() {
  run "$F1T" hil precheck
  assert_rc 2 "-disp mancante" && assert_in "-disp e' obbligatorio" && assert_in "uso:" || return 1
  run "$F1T" hil precheck -disp marte
  assert_rc 2 "-disp sconosciuto" && assert_in "linus" && assert_clean
}

test_precheck_orin_unreachable_fails_clean() {
  run "$F1T" hil precheck -disp linus --no-discovery
  assert_rc 1 precheck && assert_in "HIL PRECHECK: FAIL" && assert_in "ssh fabiocar@jetson-orin" && assert_clean || return 1
  [ -f "$REPO/logs/tools/hil_precheck_last.json" ] || { fail "manca hil_precheck_last.json"; return 1; }
  python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); assert d["status"]=="FAIL" and d["disp"]=="linus"' \
    "$REPO/logs/tools/hil_precheck_last.json" || { fail "json del precheck"; return 1; }
}

test_hil_run_needs_scenario_and_fresh_precheck() {
  run "$F1T" hil run -disp linus
  assert_rc 2 "scenario mancante" && assert_in "smoke" || return 1
  run "$F1T" hil run nessuno -disp linus
  assert_rc 2 "scenario sconosciuto" || return 1
  run "$F1T" hil run smoke -disp linus
  assert_rc 1 "precheck FAIL" && assert_in "precheck FAIL: il run non parte" && assert_clean || return 1
  [ ! -f "$REPO/logs/hil/current.env" ] || fail "run aperto dopo un precheck FAIL"
}

test_hil_down_orin_unreachable() {
  run "$F1T" hil down -disp linus
  assert_rc 1 "down senza Orin" && assert_in "Orin non raggiungibile" && assert_clean
}

test_orin_status_unreachable() {
  run "$F1T" orin status
  assert_rc 1 status && assert_in "ssh fabiocar@jetson-orin" && assert_in "SKIP" && assert_clean
}

test_build_dry_run_gates() {
  run "$F1T" build --dry-run
  assert_rc 1 "build senza Orin" && assert_in "working tree committato" && assert_in "non risponde" && assert_clean || return 1
  echo x >> "$REPO/src/pkg_a/config/params.yaml"
  run "$F1T" build --dry-run
  assert_rc 1 "build sporco" && assert_in "file tracciati modificati" || return 1
  run "$F1T" build --rust boh
  assert_rc 2 "--rust sbagliato" || return 1
  run "$F1T" build --dry-run deadbeef
  assert_rc 2 "commit sconosciuto" && assert_clean
}

test_deploy_unreachable_and_args() {
  run "$F1T" deploy --dry-run
  assert_rc 1 "deploy senza Orin" && assert_in "ssh" && assert_clean || return 1
  run "$F1T" deploy --boh
  assert_rc 2 "opzione sconosciuta" && assert_clean
}

test_diff_params_between_tags() {
  run "$F1T" diff-params --from pkg_a/v0.0.1 --to pkg_a/v0.1.0
  assert_rc 0 diff-params && assert_in "params.yaml" && assert_in "+gain: 2.0" || return 1
  run "$F1T" diff-params --from nonesiste --to HEAD
  assert_rc 1 "commit sconosciuto" && assert_clean
}

test_whatis_version() {
  run "$F1T" whatis pkg_a 0.1.0
  assert_rc 0 whatis && assert_in "## [0.1.0]" && assert_in "## [0.0.1]" && assert_in "2026-01-01_old" \
    && assert_in "risultato-di-prova 42" && assert_in "+gain: 2.0" || return 1
  run "$F1T" whatis pkg_a
  assert_rc 0 "whatis senza versione" && assert_in "pkg_a 0.1.0" || return 1
  run "$F1T" whatis nessuno
  assert_rc 2 "modulo sconosciuto" && assert_clean
}

test_exp_new_no_orin() {
  run "$F1T" exp new mpc-control prova-uno --title "Prova uno" --no-orin
  assert_rc 0 "exp new" || return 1
  local d; d=$(ls -d "$REPO"/docs/analysis/mpc-control/*_prova-uno)
  grep -q 'modules: {pkg_a: "0.1.0"}' "$d/report.md" && grep -q '^status: open' "$d/report.md" \
    && grep -q '^title: "Prova uno"' "$d/report.md" || { fail "front matter"; return 1; }
  run "$F1T" exp new mpc-control prova-uno --no-orin
  assert_rc 1 "cartella esistente" || return 1
  run "$F1T" exp new marte prova
  assert_rc 2 "area sconosciuta" && assert_in "mpc-control" && assert_clean
}

test_release_minor() {
  run "$F1T" release pkg_a minor --dry-run
  assert_rc 0 "release dry-run" && assert_in "0.1.0 -> 0.2.0" || return 1
  git -C "$REPO" diff --quiet || { fail "dry-run ha cambiato file"; return 1; }
  run "$F1T" release pkg_a minor --yes
  assert_rc 0 release && assert_clean || return 1
  grep -q '<version>0.2.0</version>' "$REPO/src/pkg_a/package.xml" && grep -q "version='0.2.0'" "$REPO/src/pkg_a/setup.py" \
    && grep -q '^## \[0.2.0\]' "$REPO/src/pkg_a/CHANGELOG.md" || { fail "file non aggiornati"; return 1; }
  git -C "$REPO" rev-parse -q --verify refs/tags/pkg_a/v0.2.0 >/dev/null || { fail "tag mancante"; return 1; }
  [ "$(git -C "$REPO" log -1 --format=%s)" = "release: pkg_a v0.2.0" ] || { fail "messaggio di commit"; return 1; }
  run "$F1T" release pkg_a 0.2.0 --yes
  assert_rc 2 "versione non maggiore" && assert_clean
}

test_doctor_and_core_sync() {
  local r=$REPO
  git -C "$r" checkout -q -b feature
  # main va avanti di una versione di tools/core
  git -C "$r" checkout -q main
  local next; next=$(next_version "$r")
  echo "$next" > "$r/tools/core/VERSION"; echo "## $next" >> "$r/tools/core/CHANGELOG.md"
  git -C "$r" add -A tools/core && F1T_CORE_EDIT=1 git -C "$r" commit -q -m "core $next"
  git -C "$r" update-ref refs/remotes/origin/main HEAD
  git -C "$r" checkout -q feature
  run "$F1T" doctor --offline
  assert_in "indietro" && assert_in "f1t core sync" && assert_clean || return 1
  run "$F1T" core sync
  assert_rc 0 "core sync" || return 1
  [ "$(cat "$r/tools/core/VERSION")" = "$next" ] && [ "$(git -C "$r" log -1 --format=%s)" = "chore: sync tools/core v$next" ] \
    || { fail "sync non committato"; return 1; }
  run "$F1T" doctor --offline
  assert_in "uguale a origin/main" || return 1
  run "$F1T" core sync
  assert_rc 0 "sync idempotente" && assert_in "gia' allineato"
}

test_precommit_hook_blocks_core_on_branch() {
  local r=$REPO hook
  run "$F1T" setup
  assert_rc 0 setup || return 1
  hook=$(git -C "$r" rev-parse --git-common-dir)/hooks/pre-commit
  [ -x "$r/$hook" ] || [ -x "$hook" ] || { fail "hook non installato"; return 1; }
  git -C "$r" checkout -q -b feature
  echo "# x" >> "$r/tools/core/config.env"; git -C "$r" add tools/core/config.env
  run git -C "$r" commit -q -m "tocca il core"
  assert_rc 1 "hook su branch" && assert_in "solo su main" || return 1
  F1T_CORE_EDIT=1 git -C "$r" commit -q -m "fix urgente" || { fail "bypass F1T_CORE_EDIT"; return 1; }
  git -C "$r" checkout -q main
  echo "# y" >> "$r/tools/core/config.env"; git -C "$r" add tools/core/config.env
  run git -C "$r" commit -q -m "core senza bump"
  assert_rc 1 "hook senza VERSION" && assert_in "VERSION" || return 1
  next_version "$r" > "$r/tools/core/VERSION"; echo "- y" >> "$r/tools/core/CHANGELOG.md"
  git -C "$r" add tools/core
  run git -C "$r" commit -q -m "core con bump"
  assert_rc 0 "hook con bump"
}
