//! costmap_boundary_replay: the Rust extraction on parity cases.
//!
//! stdin: JSON lines (f1tenth_costmap_boundary_rs::replay::Case); stdout: one
//! JSON object per pose. A malformed line is an error on stderr and exit
//! status 1 after the remaining lines, never a silent skip.

use std::io::{self, BufRead, Write};
use std::process::ExitCode;

use f1tenth_costmap_boundary_rs::replay::run_line;

fn main() -> ExitCode {
    let mut out = io::BufWriter::new(io::stdout().lock());
    let mut failed = false;
    for (n, line) in io::stdin().lock().lines().enumerate() {
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
        match run_line(&line) {
            Ok(results) => {
                for r in results {
                    if writeln!(out, "{r}").is_err() {
                        return ExitCode::FAILURE;
                    }
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
