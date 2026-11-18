//! One RTI tick's inputs, as `mpc_solver._solve_rti` reads them, and the
//! ways the tick can fail.
//!
//! The C ABI ([`crate::ffi::solver`]) fills an [`RtiInput`] from flat arrays;
//! `mpc_controller/mpc_native.py` fills those from `solve_mpc_step`'s
//! arguments. Weights arrive ALREADY scaled by `scale_stage_weights` (the
//! Python dispatcher does that before choosing an implementation, exactly as
//! for the Python path). Only the `stage_spec is None` path exists here; the
//! reverse candidates stay Python.

use crate::model::LagParams;
use crate::pyfloat::PyError;

/// The ten keys of `limits` the RTI QP reads. `v_min`/`v_max` are not read
/// with a speed model (the box moves onto the command).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    pub delta_min: f64,
    pub delta_max: f64,
    pub a_min: f64,
    pub a_max: f64,
    pub d_delta_min: f64,
    pub d_delta_max: f64,
    pub d_a_min: f64,
    pub d_a_max: f64,
    pub v_min: f64,
    pub v_max: f64,
}

/// The weights `_solve_rti` and `planner_cost_corridor` read, after
/// `scale_stage_weights`. The optional ones (`weights.get(key, 0.0)`) are
/// 0.0 when absent; a term is built when its weight is truthy (`!= 0.0`,
/// so NaN counts as on, as in Python).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Weights {
    pub w_v: f64,
    pub w_delta0: f64,
    pub w_u_a: f64,
    pub w_du_delta: f64,
    pub w_du_a: f64,
    pub w_obs: f64,
    pub w_corr: f64,
    pub w_line: f64,
    pub w_psi_stage: f64,
    pub w_term: f64,
    pub w_psi: f64,
}

/// `corridor["targetLine"]`: a point on the line and its heading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TargetLine {
    pub p: [f64; 2],
    pub psi: f64,
}

/// The corridor keys the solver reads (rti_fixture.CORRIDOR_KEYS). An
/// absent optional key is `None` and the solver's `.get()` default applies.
#[derive(Clone, Copy, Debug)]
pub struct CorridorView<'a> {
    pub xc: &'a [f64],
    pub yc: &'a [f64],
    pub nx: &'a [f64],
    pub ny: &'a [f64],
    pub half_width: &'a [f64],
    /// `corridor.get("psiRef")`: the QP falls back to the reference
    /// trajectory's terminal yaw; `planner_cost_corridor` needs it
    /// (KeyError) whenever `w_psi` is on.
    pub psi_ref: Option<f64>,
    /// `corridor.get("psiRefTurn")`: `None` is the shortest-branch target.
    pub psi_ref_turn: Option<f64>,
    /// `corridor.get("psiStart", psi_ref_N)`.
    pub psi_start: Option<f64>,
    pub target_line: Option<TargetLine>,
    /// `corridor.get("car_radius", 0.0)`.
    pub car_radius: f64,
    /// `corridor.get("avoidance_margin", 0.12)`.
    pub avoidance_margin: f64,
    /// `corridor["obstacles_world"]`: `(x, y, r)` rows.
    pub obstacles: &'a [[f64; 3]],
}

/// `speed_model` (the offset-free speed channel), arrays over the stages.
#[derive(Clone, Copy, Debug)]
pub struct SpeedModel<'a> {
    pub lag: LagParams,
    /// Hard box on the command `c_{k+1}`, at least `horizon` entries.
    pub c_lo: &'a [f64],
    pub c_hi: &'a [f64],
    /// Soft cap on the real speed `v_{k+1}` (inf allowed), or none.
    pub v_hi: Option<&'a [f64]>,
    /// `speed_model.get('slack_weight', 1e3)`.
    pub slack_weight: f64,
}

/// The boundary rows of `_solve_rti` (`normal . p <= offset`).
#[derive(Clone, Copy, Debug)]
pub struct Boundaries<'a> {
    /// `list(boundaries)[:boundary_max_sources]`; padded here to
    /// `max_sources` with `BOUNDARY_DISABLED = (0, 0, inf)`.
    pub rows: &'a [[f64; 3]],
    pub max_sources: usize,
    /// `bool(boundaries) and not boundary_hard`: one slack per slot.
    pub use_slack: bool,
    pub slack_weight: f64,
    pub slack_max: f64,
}

/// Everything one `_solve_rti` call reads (stage_spec None).
#[derive(Clone, Copy, Debug)]
pub struct RtiInput<'a> {
    /// 4 entries (legacy) or 5 (with a speed model: the command in force).
    pub x0: &'a [f64],
    pub last_u: [f64; 2],
    pub pref_nom: [f64; 2],
    pub horizon: usize,
    /// The prediction step.
    pub ts: f64,
    /// `None`: the same as `ts`.
    pub control_period: Option<f64>,
    pub wheelbase: f64,
    pub lr: f64,
    pub limits: Limits,
    pub weights: Weights,
    pub vdes: f64,
    /// `f1tenth_params.object_geometry.SOLVER_FRONT_POINT_OFFSETS_M`.
    pub front_offsets: &'a [f64],
    pub corridor: CorridorView<'a>,
    pub boundaries: Boundaries<'a>,
    /// `warm_start_z` as passed (raveled); a wrong length or a non-finite
    /// entry falls back to `last_u` tiled, as in the Python.
    pub warm_start: Option<&'a [f64]>,
    pub speed_model: Option<SpeedModel<'a>>,
}

impl RtiInput<'_> {
    /// The state dimension the speed model needs.
    pub fn nx(&self) -> usize {
        if self.speed_model.is_some() {
            5
        } else {
            4
        }
    }

    /// `t_first`: the first rate row's span (`control_period or ts`).
    pub fn t_first(&self) -> f64 {
        self.control_period.unwrap_or(self.ts)
    }
}

/// Why a tick has no result: each is an exception the Python raises (or an
/// input it cannot take). The binding maps them back to Python exceptions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolveError {
    /// A CPython float failure ([`PyError`]).
    Py(PyError),
    /// `ValueError(f'x0 has {got} entries, the speed model needs {want}')`.
    X0Size { got: usize, want: usize },
    /// `horizon < 1` (the Python builds a degenerate QP; refused here).
    Horizon,
    /// An empty corridor: `np.argmin` of an empty sequence.
    EmptyCorridor,
    /// An input array is shorter than the horizon / arrays differ in length.
    Length,
    /// `corridor["psiRef"]` absent while `w_psi` is on (KeyError).
    MissingPsiRef,
    /// More than `qp::MAX_VARIABLES` QP variables (refused, ValueError).
    TooLarge,
    /// `osqp_setup` failed with this OSQP error code (osqp-python raises
    /// `OSQPException(code)`).
    OsqpSetup(i64),
}

impl From<PyError> for SolveError {
    fn from(e: PyError) -> Self {
        SolveError::Py(e)
    }
}
