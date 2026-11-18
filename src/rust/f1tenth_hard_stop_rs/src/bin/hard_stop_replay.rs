//! hard_stop_replay: the hard stop's decision and node logic on parity ops.
//!
//! stdin: JSON lines (f1tenth_hard_stop_rs::replay); stdout: one JSON result
//! per op, in order. Node ops act on the session the last `node` op started,
//! so one stdin carries any number of node sessions. A malformed op is an
//! error on stderr, an `{"id": .., "error": ..}` line on stdout (the harness
//! counts it) and exit status 1 at the end, never a silent skip. The first
//! stderr line is the build (build_info).

use std::io::{self, BufRead, Write};
use std::process::ExitCode;

use serde_json::{json, Value};

use f1tenth_hard_stop_rs::build_info;
use f1tenth_hard_stop_rs::replay::Session;

fn main() -> ExitCode {
    eprintln!("hard_stop_replay {}", build_info::summary());
    let stdin = io::stdin();
    let mut out = io::BufWriter::new(io::stdout().lock());
    let mut session = Session::default();
    let mut failed = false;
    for (n, line) in stdin.lock().lines().enumerate() {
        let line = match line {
            Ok(l) => l,
            Err(e) => {
                eprintln!("line {}: read error: {e}", n + 1);
                return ExitCode::FAILURE;
            }
        };
        if line.trim().is_empty() {
            continue;
        }
        let parsed: Result<Value, String> = serde_json::from_str(&line).map_err(|e| e.to_string());
        let id = parsed
            .as_ref()
            .ok()
            .and_then(|v| v.get("id").cloned())
            .unwrap_or(Value::Null);
        let result = parsed.and_then(|op| session.run(&op));
        let text = match result {
            Ok(v) => v.to_string(),
            Err(e) => {
                eprintln!("line {}: {e}", n + 1);
                failed = true;
                json!({"id": id, "error": e}).to_string()
            }
        };
        if writeln!(out, "{text}").is_err() {
            return ExitCode::FAILURE;
        }
    }
    if out.flush().is_err() || failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
