//! Passes cargo's profile and opt-level to the crate (src/build_info.rs), so
//! every binary can log what it was built with. colcon-ros-cargo builds with
//! cargo's dev profile; src/rust/Cargo.toml sets its opt-level.
//!
//! Also adds `-L <prefix>/lib` for every AMENT_PREFIX_PATH entry and re-runs
//! when AMENT_PREFIX_PATH changes. ros-env 0.3.0 regenerates its message
//! modules, with their `#[link(name = "<pkg>__rosidl_typesupport_c")]`
//! attributes, whenever AMENT_PREFIX_PATH changes; rclrs 0.8.0's build script,
//! which emits these search paths, re-runs only when ROS_DISTRO changes. A
//! second build in the same target dir with more prefixes on the path (the
//! workspace sourced, `colcon test`) then failed to link: "unable to find
//! library -lzed_msgs__rosidl_typesupport_c" (output/rust_main/r0b.md).

fn main() {
    for var in ["PROFILE", "OPT_LEVEL"] {
        let value = std::env::var(var).unwrap_or_else(|_| "unknown".to_owned());
        println!("cargo:rustc-env=F1TENTH_CARGO_{var}={value}");
    }
    if let Some(paths) = std::env::var_os("AMENT_PREFIX_PATH") {
        for prefix in std::env::split_paths(&paths) {
            let lib = prefix.join("lib");
            if lib.is_dir() {
                println!("cargo:rustc-link-search=native={}", lib.display());
            }
        }
    }
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-changed=build.rs");
}
