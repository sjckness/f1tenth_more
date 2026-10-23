//! The build this binary was compiled with, for its startup log.
//!
//! colcon-ros-cargo calls `cargo ament-build` without `--release`, so every
//! installed binary is built with cargo's dev profile. Before R0 (2026-10-23)
//! that meant opt-level 0; src/rust/Cargo.toml now sets `[profile.dev]
//! opt-level = 3`. build.rs passes cargo's PROFILE and OPT_LEVEL through
//! (`option_env!`: a copy of this crate built without build.rs says unknown).

/// cargo's profile kind: "debug" (dev, test) or "release".
pub const CARGO_PROFILE: &str = match option_env!("F1TENTH_CARGO_PROFILE") {
    Some(v) => v,
    None => "unknown",
};

/// rustc's opt-level for this build: "0" to "3", "s" or "z".
pub const OPT_LEVEL: &str = match option_env!("F1TENTH_CARGO_OPT_LEVEL") {
    Some(v) => v,
    None => "unknown",
};

/// `cfg!(debug_assertions)`: on in the dev profile whatever its opt-level.
pub const DEBUG_ASSERTIONS: bool = cfg!(debug_assertions);

/// One line for the startup log, e.g.
/// "build: cargo profile debug, opt-level 3, debug_assertions on".
pub fn summary() -> String {
    format!(
        "build: cargo profile {CARGO_PROFILE}, opt-level {OPT_LEVEL}, debug_assertions {}",
        if DEBUG_ASSERTIONS { "on" } else { "off" }
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The regression this guards: an installed node built at opt-level 0.
    /// cargo test builds with the test profile, which inherits dev's opt-level.
    #[test]
    fn built_optimised() {
        assert_eq!(OPT_LEVEL, "3", "{}", summary());
    }

    #[test]
    fn the_summary_names_profile_opt_level_and_assertions() {
        let s = summary();
        assert!(
            s.contains(&format!("cargo profile {CARGO_PROFILE},")),
            "{s}"
        );
        assert!(s.contains("opt-level 3,"), "{s}");
        let assertions = if DEBUG_ASSERTIONS { "on" } else { "off" };
        assert!(
            s.ends_with(&format!("debug_assertions {assertions}")),
            "{s}"
        );
    }
}
