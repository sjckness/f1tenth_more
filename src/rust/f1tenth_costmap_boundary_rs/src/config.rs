//! Startup checks of the node's parameters: pure, so they are unit-tested
//! without ROS (main.rs reads the parameters, [`validate`] decides).
//!
//! The Rust node ports the nearest-occupied-cell path only. Each Python
//! switch that selects another path is declared with the Python's default
//! (an identical parameter interface) and refused when set: the node exits
//! with a configuration error instead of silently running the default path.
//!   use_convex_polytope            safe_corridor.py, not ported
//!   component_filter               costmap_boundary.py cleanup, not ported
//!   exclude_tracked_object_cells   costmap_boundary.py cleanup, not ported

/// The parameter values [`validate`] checks, as read at startup.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StartupParams {
    pub use_convex_polytope: bool,
    pub component_filter: bool,
    pub exclude_tracked_object_cells: bool,
    /// vehicle_frames values, passed by costmap.launch.py (no usable default).
    pub base_front_x_m: f64,
    pub front_side_limit_m: f64,
    pub min_front_distance_m: f64,
    pub extraction_rate_hz: f64,
}

/// The Python switches this port refuses, in declaration order.
pub const UNPORTED_SWITCHES: [&str; 3] = [
    "use_convex_polytope",
    "component_filter",
    "exclude_tracked_object_cells",
];

fn not_ported(name: &str) -> String {
    format!(
        "{name}:=true is not ported to Rust: run impl:=py \
         (costmap.launch.py costmap_boundary_impl:=py)"
    )
}

/// Ok when the node can run these parameters; otherwise every reason, joined
/// with "; " (the switches first, in [`UNPORTED_SWITCHES`] order).
pub fn validate(p: &StartupParams) -> Result<(), String> {
    let mut errors: Vec<String> = UNPORTED_SWITCHES
        .iter()
        .zip([
            p.use_convex_polytope,
            p.component_filter,
            p.exclude_tracked_object_cells,
        ])
        .filter(|(_, on)| *on)
        .map(|(name, _)| not_ported(name))
        .collect();
    for (name, v) in [
        ("base_front_x_m", p.base_front_x_m),
        ("front_side_limit_m", p.front_side_limit_m),
        ("min_front_distance_m", p.min_front_distance_m),
    ] {
        if !v.is_finite() {
            errors.push(format!(
                "{name} is required (costmap.launch.py passes vehicle_frames' value)"
            ));
        }
    }
    let rate = p.extraction_rate_hz;
    if !(rate.is_finite() && rate > 0.0) {
        errors.push(format!("extraction_rate_hz must be positive, got {rate}"));
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The launch path's values: Python defaults, car vehicle_frames.
    fn launch_defaults() -> StartupParams {
        StartupParams {
            use_convex_polytope: false,
            component_filter: false,
            exclude_tracked_object_cells: false,
            base_front_x_m: 0.405,
            front_side_limit_m: 0.186,
            min_front_distance_m: 0.01,
            extraction_rate_hz: 10.0,
        }
    }

    #[test]
    fn the_defaults_start() {
        assert_eq!(validate(&launch_defaults()), Ok(()));
    }

    #[test]
    fn each_unported_switch_is_refused_with_its_own_name() {
        for name in UNPORTED_SWITCHES {
            let mut p = launch_defaults();
            match name {
                "use_convex_polytope" => p.use_convex_polytope = true,
                "component_filter" => p.component_filter = true,
                "exclude_tracked_object_cells" => p.exclude_tracked_object_cells = true,
                other => panic!("untested switch {other}"),
            }
            let err = validate(&p).expect_err(name);
            assert_eq!(
                err,
                format!(
                    "{name}:=true is not ported to Rust: run impl:=py \
                     (costmap.launch.py costmap_boundary_impl:=py)"
                )
            );
        }
    }

    #[test]
    fn the_message_pattern_is_the_one_use_convex_polytope_always_had() {
        let p = StartupParams {
            use_convex_polytope: true,
            ..launch_defaults()
        };
        assert_eq!(
            validate(&p),
            Err(
                "use_convex_polytope:=true is not ported to Rust: run impl:=py \
                 (costmap.launch.py costmap_boundary_impl:=py)"
                    .to_owned()
            )
        );
    }

    #[test]
    fn both_cleanup_switches_are_reported_together() {
        let p = StartupParams {
            component_filter: true,
            exclude_tracked_object_cells: true,
            ..launch_defaults()
        };
        let err = validate(&p).unwrap_err();
        let parts: Vec<&str> = err.split("; ").collect();
        assert_eq!(parts.len(), 2, "{err}");
        assert!(parts[0].starts_with("component_filter:=true is not ported"));
        assert!(parts[1].starts_with("exclude_tracked_object_cells:=true is not ported"));
    }

    #[test]
    fn the_vehicle_frames_values_are_required() {
        for field in 0..3 {
            let mut p = launch_defaults();
            let name = match field {
                0 => {
                    p.base_front_x_m = f64::NAN;
                    "base_front_x_m"
                }
                1 => {
                    p.front_side_limit_m = f64::INFINITY;
                    "front_side_limit_m"
                }
                _ => {
                    p.min_front_distance_m = f64::NAN;
                    "min_front_distance_m"
                }
            };
            assert_eq!(
                validate(&p),
                Err(format!(
                    "{name} is required (costmap.launch.py passes vehicle_frames' value)"
                ))
            );
        }
    }

    #[test]
    fn the_rate_must_be_positive_and_finite() {
        for rate in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let p = StartupParams {
                extraction_rate_hz: rate,
                ..launch_defaults()
            };
            assert_eq!(
                validate(&p),
                Err(format!("extraction_rate_hz must be positive, got {rate}"))
            );
        }
        let p = StartupParams {
            extraction_rate_hz: 0.5,
            ..launch_defaults()
        };
        assert_eq!(validate(&p), Ok(()));
    }

    #[test]
    fn a_switch_and_a_missing_value_are_both_reported() {
        let p = StartupParams {
            component_filter: true,
            base_front_x_m: f64::NAN,
            ..launch_defaults()
        };
        let err = validate(&p).unwrap_err();
        assert!(
            err.starts_with("component_filter:=true is not ported"),
            "{err}"
        );
        assert!(
            err.ends_with(
                "base_front_x_m is required (costmap.launch.py passes vehicle_frames' value)"
            ),
            "{err}"
        );
    }
}
