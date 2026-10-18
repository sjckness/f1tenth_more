//! proximity_guard_replay: the guard's decision on parity cases.
//!
//! stdin: JSON lines (f1tenth_proximity_guard::replay::Case);
//! stdout: one JSON outcome per line. A malformed line is an error on stderr
//! and exit status 1 after the remaining lines, never a silent skip.

use std::io::{self, BufRead, Write};
use std::process::ExitCode;

use f1tenth_proximity_guard::replay::run_line;

fn main() -> ExitCode {
    let stdin = io::stdin();
    let mut out = io::BufWriter::new(io::stdout().lock());
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
        let result =
            run_line(&line).and_then(|o| serde_json::to_string(&o).map_err(|e| e.to_string()));
        match result {
            Ok(json) => {
                if writeln!(out, "{json}").is_err() {
                    return ExitCode::FAILURE;
                }
            }
            Err(e) => {
                eprintln!("line {}: {e}", n + 1);
                failed = true;
            }
        }
    }
    if out.flush().is_err() || failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
