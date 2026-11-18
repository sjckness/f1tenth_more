//! The 42 tests of f1tenth_control/test/test_hard_stop.py, as JSON cases
//! (tests/data/test_cases.jsonl, written by
//! tools/rust_parity/hard_stop/make_test_cases.py): each test's inputs as
//! replay ops, and the outputs the Python code gives for them. Every op's
//! Rust output must equal the Python's bit for bit (floats by bit pattern,
//! the diagnostic JSON and log lines byte for byte).

use serde_json::Value;

use f1tenth_hard_stop_rs::replay::{crate_dependencies, same, Session};

const CASES: &str = include_str!("data/test_cases.jsonl");

fn first_difference(a: &Value, b: &Value, path: &str) -> Option<String> {
    if same(a, b) {
        return None;
    }
    match (a, b) {
        (Value::Object(x), Value::Object(y)) if x.len() == y.len() => {
            for (k, v) in x {
                if let Some(w) = y.get(k) {
                    if let Some(d) = first_difference(v, w, &format!("{path}.{k}")) {
                        return Some(d);
                    }
                }
            }
        }
        (Value::Array(x), Value::Array(y)) if x.len() == y.len() => {
            for (i, (v, w)) in x.iter().zip(y).enumerate() {
                if let Some(d) = first_difference(v, w, &format!("{path}[{i}]")) {
                    return Some(d);
                }
            }
        }
        _ => {}
    }
    Some(format!("{path}: rust {a} != python {b}"))
}

#[test]
fn the_42_python_tests_give_the_python_s_outputs() {
    let mut tests = 0;
    let mut ops = 0;
    for line in CASES.lines().filter(|l| !l.trim().is_empty()) {
        let case: Value = serde_json::from_str(line).unwrap();
        let name = case["test"].as_str().unwrap().to_owned();
        tests += 1;
        if case["rust_only"] == "deps" {
            let deps = crate_dependencies();
            assert!(!deps.is_empty());
            for forbidden in case["forbidden"].as_array().unwrap() {
                let forbidden = forbidden.as_str().unwrap();
                assert!(
                    deps.iter().all(|d| !d.contains(forbidden)),
                    "{name}: depends on {forbidden}: {deps:?}"
                );
            }
            continue;
        }
        let mut session = Session::default();
        let expected = case["expected"].as_array().unwrap();
        let case_ops = case["ops"].as_array().unwrap();
        assert_eq!(expected.len(), case_ops.len(), "{name}");
        for (op, want) in case_ops.iter().zip(expected) {
            ops += 1;
            let got = session.run(op).unwrap();
            if let Some(d) = first_difference(&got, want, "$") {
                panic!("{name}, op {}: {d}", op["id"]);
            }
        }
    }
    assert_eq!(tests, 42, "one case per test function");
    assert!(ops >= 200, "{ops} ops");
}
