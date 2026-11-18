# shellcheck shell=bash
# Test delle librerie (log.sh, config.sh, checks.sh) e della sintassi di ogni script.

test_check_summary_semantics() {
  run bash -c "
    F1T_CORE='$REPO/tools/core' F1T_ROOT='$REPO'; source \"\$F1T_CORE/lib/log.sh\"
    ok() { echo 'valore-ok'; }; w() { echo 'valore-warn'; return 10; }; ko() { echo 'riga 1'; echo 'valore-fail'; return 3; }
    check 'uno' ok; check 'due' w; check_warn 'tre' ko; check_skip 'quattro' 'motivo'
    summary t && echo SUMMARY0
    summary_json '$REPO/out.json' disp=x
    check 'cinque' ko; summary t || echo SUMMARY1"
  assert_in "valore-ok" && assert_in "valore-warn" && assert_in "SUMMARY0" && assert_in "SUMMARY1" && assert_in "valore-fail" || return 1
  python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); s=[c["state"] for c in d["checks"]]; assert s==["OK","WARN","WARN","SKIP"], s; assert d["status"]=="OK" and d["disp"]=="x"' \
    "$REPO/out.json" || { fail "summary_json"; return 1; }
}

test_log_file_written() {
  run "$F1T" exp new marte x
  ls "$REPO"/logs/tools/analysis_new_*.log >/dev/null 2>&1 || { fail "nessun log in logs/tools"; return 1; }
  grep -q "area sconosciuta" "$REPO"/logs/tools/analysis_new_*.log || fail "il log non ha il messaggio"
}

test_probe_ping_parsing() {
  run bash -c "F1T_CORE='$REPO/tools/core' F1T_ROOT='$REPO'; source \"\$F1T_CORE/lib/config.sh\"; source \"\$F1T_CORE/lib/checks.sh\"; probe_ping 10.0.0.1 3"
  assert_rc 1 "ping fallito" && assert_in "non risponde"
}

test_scripts_syntax() {
  local f
  while IFS= read -r f; do
    bash -n "$f" 2>/dev/null || { fail "bash -n $f"; return 1; }
  done < <(find "$REPO/tools" -name '*.sh' -o -name f1t -o -name pre-commit)
  while IFS= read -r f; do
    python3 -m py_compile "$f" 2>/dev/null || { fail "py_compile $f"; return 1; }
  done < <(find "$REPO/tools" -name '*.py')
}
