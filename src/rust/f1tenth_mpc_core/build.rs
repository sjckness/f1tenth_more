//! Passes cargo's profile and opt-level to the crate (src/build_info.rs), so
//! every binary can log what it was built with. colcon-ros-cargo builds with
//! cargo's dev profile; src/rust/Cargo.toml sets its opt-level.

fn main() {
    for var in ["PROFILE", "OPT_LEVEL"] {
        let value = std::env::var(var).unwrap_or_else(|_| "unknown".to_owned());
        println!("cargo:rustc-env=F1TENTH_CARGO_{var}={value}");
    }
    println!("cargo:rerun-if-changed=build.rs");
}
