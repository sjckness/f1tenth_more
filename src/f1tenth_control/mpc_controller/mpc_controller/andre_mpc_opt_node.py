"""Optimized drop-in replacement for andre_mpc_node.py.

Same MPC formulation, parameters, topics and behavior as AndreMPCNode, but
restructured for maximum control-loop frequency on a Jetson Orin AGX that is
simultaneously running a llama.cpp HTTP server and several other ROS 2 nodes.

What is IDENTICAL to the original (by requirement):
  * cost function (radial + heading + steering-rate + lateral-accel penalties)
  * SLSQP solver, horizon N=10, dt=0.1, control horizon = N, decision vector
    u = [ddelta_0, accel_0(fixed 0), ddelta_1, 0, ...]  (length 2N)
  * every declared ROS parameter (names, defaults, units) and the /odom in,
    /drive out topics + AckermannDriveStamped message type
  * the per-loop INFO tracking log

What is OPTIMIZED (every change commented inline below):
  * hot cost function compiled with numba @njit when available, pure-Python
    fallback otherwise (numerically identical)
  * no per-evaluation array allocation (original did self.state.copy() on every
    single cost call); state captured once per loop and passed as scalars
  * constants/bounds precomputed once at startup, not rebuilt each iteration
  * parameters cached + refreshed via an on-set callback instead of polling 12
    get_parameter() calls every loop
  * best-effort, depth-1 QoS on /odom (sensor input) to minimize latency
  * reused (pre-allocated) output messages
  * optional Jetson CPU-affinity pinning + process nice level
  * perf_counter profiling (pre / solve / post) logged at DEBUG, and the
    achieved loop rate published on /andre_mpc/loop_hz (std_msgs/Float32)
"""

import math
import os
import time

import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32   # already a package dependency (package.xml)


# --- numba is optional -------------------------------------------------------
# The inner rollout is pure scalar arithmetic, so @njit gives a large speedup
# (the SLSQP finite-difference gradient calls the cost function many times per
# loop). If numba is not installed we transparently fall back to a no-op
# decorator and run the same function in plain Python.
try:
    from numba import njit
    NUMBA_AVAILABLE = True
except Exception:                       # pragma: no cover - depends on host
    NUMBA_AVAILABLE = False

    def njit(*args, **kwargs):
        """No-op @njit fallback. TODO: `pip install numba` on the Jetson to
        enable the compiled fast path (expect a large per-loop speedup)."""
        if len(args) == 1 and callable(args[0]) and not kwargs:
            return args[0]

        def _wrap(func):
            return func
        return _wrap


# ---------------------------------------------------------------------------
# Circle MPC node
# Steering = geometric feedforward (pure pursuit to circle) +
#            MPC correction term (bad params → LLM has something to tune)
# Speed    = open-loop sine profile around CIRCLE_SPEED
# ---------------------------------------------------------------------------
CIRCLE_STEER = 0.1     # [rad] desired steady-state steering for the circle
CIRCLE_SPEED = 1.0     # [m/s] centre of the sine speed profile
SINE_AMP     = 0.0     # [m/s] sine amplitude (0 → constant speed)
SINE_PERIOD  = 4.0     # [s]

_HALF_PI = math.pi / 2.0


# ---------------------------------------------------------------------------
# Hot inner cost — compiled once (njit) and reused for every solver evaluation.
#
# OPTIMIZATION: this is the single most-called piece of code per control loop
# (SLSQP evaluates it O(n_vars * maxiter) times via finite differences). It is
# a module-level function of plain scalars/arrays so numba can compile it; the
# feedforward steering is inlined here (it depends only on x, y) to avoid a
# Python call per rollout step. The math is line-for-line identical to
# AndreMPCNode.rollout_cost + feedforward_delta.
#
# NOTE: fastmath is intentionally NOT enabled — it would allow float
# reassociation and could perturb the optimum slightly, violating the
# "identical logic" requirement. It is available as a further speed lever if a
# tiny numeric difference is acceptable.
# ---------------------------------------------------------------------------
@njit(cache=True)
def _rollout_cost_core(u, sx, sy, syaw, sv, N, dt, L, cx, cy, R,
                       circle_steer, max_steer,
                       qn, qalpha, qddelta, alat_max, v_min):
    x = sx
    y = sy
    yaw = syaw
    v = sv if sv > v_min else v_min        # == max(v, v_min)

    cost = 0.0
    for k in range(N):
        ddelta = u[2 * k]                  # accel slots (2k+1) are fixed at 0

        # inlined feedforward (depends on x, y only)
        ff = circle_steer - 0.05 * (math.hypot(x - cx, y - cy) - R)
        if ff > max_steer:
            ff = max_steer
        elif ff < -max_steer:
            ff = -max_steer

        delta = ff + ddelta
        if delta > max_steer:
            delta = max_steer
        elif delta < -max_steer:
            delta = -max_steer

        # kinematic bicycle rollout
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v / L * math.tan(delta) * dt
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))     # wrap

        radial_err = math.hypot(x - cx, y - cy) - R

        tangent_yaw = math.atan2(y - cy, x - cx) + _HALF_PI
        heading_err = yaw - tangent_yaw
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        alat = v * v * math.tan(delta) / L
        if alat < 0.0:
            alat = -alat
        alat_violation = alat - alat_max
        if alat_violation < 0.0:
            alat_violation = 0.0

        cost += qn * radial_err * radial_err
        cost += qalpha * heading_err * heading_err
        cost += qddelta * ddelta * ddelta
        cost += 100.0 * alat_violation * alat_violation

    return cost


# Names of every parameter declared by the original node — kept identical so
# the ROS parameter interface (and the LLM tuner) sees exactly the same set.
_PARAM_NAMES = (
    'qn', 'qv', 'qalpha', 'qddelta', 'alat_max', 'a_min', 'a_max',
    'v_min', 'v_max', 'v_ref', 'sine_amp', 'sine_period', 'steer_sign',
)


class AndreMPCOptNode(Node):
    def __init__(self):
        super().__init__('andre_mpc_controller')   # same node name → drop-in

        self.N  = 10
        self.dt = 0.1
        self.L  = 0.25

        # ── MPC correction params (identical to original) ──────────────────
        self.declare_parameter('qn',       50.0)   # high: tight radial tracking
        self.declare_parameter('qv',       50.0)   # high: tight radial tracking
        self.declare_parameter('qalpha',   30.0)   # high: stay tangent to circle
        self.declare_parameter('qddelta',   2.0)   # smooth steering corrections
        self.declare_parameter('alat_max', 10.0)   # lateral accel limit
        self.declare_parameter('a_min',    -3.0)
        self.declare_parameter('a_max',     3.0)
        self.declare_parameter('v_min',     -1.5)
        self.declare_parameter('v_max',     1.5)
        self.declare_parameter('v_ref',     CIRCLE_SPEED)
        self.declare_parameter('sine_amp',    SINE_AMP)
        self.declare_parameter('sine_period', SINE_PERIOD)
        self.declare_parameter('steer_sign',  1.0)

        # Infra-only params (do NOT affect control). Empty/zero = disabled.
        # cpu_affinity: comma-separated core ids to pin this process to.
        # nice: process niceness (negative = higher priority, needs privilege).
        self.declare_parameter('cpu_affinity', '')
        self.declare_parameter('nice', -5)

        self.max_steer  = 0.18
        self.max_dsteer = 0.05   # larger step so correction can act fast

        # Circle geometry — fixed from first odom
        self.R  = self.L / math.tan(CIRCLE_STEER)
        self.cx = None
        self.cy = None
        self.t0 = None

        self.state: np.ndarray | None = None
        self.prev_delta      = CIRCLE_STEER   # start on the circle steering
        self.last_solution   = np.zeros(2 * self.N)   # warm-start buffer (reused)
        self.consec_failures = 0

        # OPTIMIZATION: cache parameters as attributes and refresh them only
        # when they actually change (on-set callback) instead of calling
        # get_parameter() 12x every control loop.
        self._read_all_params()
        self.add_on_set_parameters_callback(self._on_set_params)

        # OPTIMIZATION: the bounds list depends only on max_dsteer and N (both
        # fixed), so build it once here instead of rebuilding 20 tuples per loop.
        self._bounds = [(-self.max_dsteer, self.max_dsteer), (0.0, 0.0)] * self.N

        # OPTIMIZATION: best-effort, depth-1 QoS for odometry. Control wants the
        # freshest pose; reliable/queued delivery only adds latency and lets
        # stale samples pile up. (A best-effort sub is compatible with both
        # reliable and best-effort publishers.)
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, odom_qos)

        # /drive kept reliable depth 10 — identical to the original so mux QoS
        # compatibility and command delivery are unchanged.
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Achieved loop-rate telemetry for Foxglove.
        self.hz_pub = self.create_publisher(Float32, '/andre_mpc/loop_hz', 10)

        # OPTIMIZATION: pre-allocate the output messages once and only mutate
        # their fields each loop, avoiding per-iteration object construction.
        self._drive_msg = AckermannDriveStamped()
        self._drive_msg.header.frame_id = 'base_link'
        self._hz_msg = Float32()

        # Jetson tuning (CPU affinity + priority); safe no-op if unset/denied.
        self._apply_cpu_affinity_and_priority()

        # Compile the njit core now so the first real control loop is not hit
        # by JIT latency. Uses dummy geometry (cx/cy not known until first odom).
        self._warmup_solver()

        # ── Profiling state (perf_counter, rolling window of 50) ───────────
        self._prof_pre   = 0.0
        self._prof_solve = 0.0
        self._prof_post  = 0.0
        self._prof_n     = 0
        self._last_entry_t = None
        self._hz_ema       = 0.0

        # OPTIMIZATION: control runs in a fixed-rate timer (not driven by the
        # odom callback), so solver jitter never back-pressures the input.
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'Circle MPC (opt) — R={self.R:.2f} m, '
            f'steer_ff={CIRCLE_STEER} rad, speed=[{CIRCLE_SPEED-SINE_AMP:.1f}, '
            f'{CIRCLE_SPEED+SINE_AMP:.1f}] m/s | '
            f'numba={"on" if NUMBA_AVAILABLE else "OFF (pure-python fallback)"}'
        )

    # ── Parameter handling ─────────────────────────────────────────────────
    def _read_all_params(self):
        """Populate cached attributes from the node's current parameters."""
        for name in _PARAM_NAMES:
            setattr(self, name, float(self.get_parameter(name).value))

    def _on_set_params(self, params):
        """Update the cache when a control parameter is set at runtime.

        Behaviorally equivalent to the original's per-loop polling: a value set
        via `ros2 param set` takes effect on the next control loop. Returns
        success so the parameter server still stores the new value.
        """
        from rcl_interfaces.msg import SetParametersResult   # rclpy-bundled
        for p in params:
            if p.name in _PARAM_NAMES:
                setattr(self, p.name, float(p.value))
        return SetParametersResult(successful=True)

    # ── Jetson process tuning ──────────────────────────────────────────────
    def _apply_cpu_affinity_and_priority(self):
        """Pin the process and raise its scheduling priority (best effort).

        Jetson Orin AGX has 12 homogeneous Cortex-A78AE cores (NO big.LITTLE /
        efficiency cores — so "E-core" pinning does not apply). The goal here is
        ISOLATION from llama.cpp: dedicate a couple of cores to this control
        node and keep llama.cpp off them. Recommended on a 12-core Orin:
            ros2 run ... --ros-args -p cpu_affinity:=10,11
        and start llama.cpp restricted to the rest, e.g.
            taskset -c 0-9 ./server ...      (or numactl / --cpu-mask)
        Optionally boot with `isolcpus=10,11` and run MAXN power mode +
        jetson_clocks for stable latency. Default is empty (inherit affinity)
        because hard-coding core ids would break on machines with fewer cores.
        """
        spec = str(self.get_parameter('cpu_affinity').value).strip()
        if spec and hasattr(os, 'sched_setaffinity'):
            try:
                ncpu = os.cpu_count() or 1
                cores = {int(c) for c in spec.split(',') if c.strip() != ''}
                cores = {c for c in cores if 0 <= c < ncpu}
                if cores:
                    os.sched_setaffinity(0, cores)
                    self.get_logger().info(f'CPU affinity pinned to {sorted(cores)}')
                else:
                    self.get_logger().warn(
                        f'cpu_affinity="{spec}" has no valid core (cpu_count={ncpu})'
                    )
            except Exception as exc:
                self.get_logger().warn(f'Could not set CPU affinity: {exc}')

        nice_val = int(self.get_parameter('nice').value)
        if nice_val != 0:
            # Negative niceness lowers scheduling latency for the control loop
            # but needs CAP_SYS_NICE (root). Failure is non-fatal.
            try:
                os.nice(nice_val)
                self.get_logger().info(f'Process nice set to {nice_val:+d}')
            except Exception as exc:
                self.get_logger().warn(
                    f'Could not set nice {nice_val:+d} (need CAP_SYS_NICE/root): {exc}'
                )

    def _warmup_solver(self):
        """Trigger numba compilation of the cost core at startup."""
        try:
            u0 = np.zeros(2 * self.N)
            _rollout_cost_core(
                u0, 0.0, 0.0, 0.0, abs(CIRCLE_SPEED),
                self.N, self.dt, self.L, 0.0, 0.0, self.R,
                CIRCLE_STEER, self.max_steer,
                self.qn, self.qalpha, self.qddelta, self.alat_max, self.v_min,
            )
        except Exception as exc:
            self.get_logger().warn(f'Solver warmup skipped: {exc}')

    # ── Helpers (identical math to original) ────────────────────────────────
    def yaw_from_quat(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def sine_v_ref(self, t_offset_steps=0):
        t = (self.get_clock().now().nanoseconds * 1e-9 - self.t0) \
            + t_offset_steps * self.dt
        return self.v_ref + self.sine_amp * math.sin(
            2 * math.pi * t / self.sine_period
        )

    def feedforward_delta(self, x, y):
        """Geometric feedforward steering onto the circle (depends on x, y).
        Called once per loop for the final command; the per-rollout-step copy
        is inlined in _rollout_cost_core for speed."""
        r_err = math.hypot(x - self.cx, y - self.cy) - self.R
        ff = CIRCLE_STEER - 0.05 * r_err
        return float(min(self.max_steer, max(-self.max_steer, ff)))

    # ── Odom callback (cheap: just stores state) ────────────────────────────
    def odom_callback(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)
        v   = msg.twist.twist.linear.x   # keep sign for diagnostics

        if self.cx is None:
            self.cx = x - self.R * math.sin(yaw)
            self.cy = y + self.R * math.cos(yaw)
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f'Circle center: ({self.cx:.2f}, {self.cy:.2f}), R={self.R:.2f} m'
            )

        self.state = np.array([x, y, yaw, abs(v), self.prev_delta], dtype=float)

    # ── Control loop ────────────────────────────────────────────────────────
    def control_loop(self):
        if self.state is None or self.cx is None:
            return

        # Achieved loop-rate telemetry (wall-clock between active iterations).
        now_t = time.perf_counter()
        if self._last_entry_t is not None:
            dt_wall = now_t - self._last_entry_t
            if dt_wall > 0.0:
                inst_hz = 1.0 / dt_wall
                # light EMA so the Foxglove trace is readable
                self._hz_ema = (0.2 * inst_hz + 0.8 * self._hz_ema
                                if self._hz_ema > 0.0 else inst_hz)
                self._hz_msg.data = float(self._hz_ema)
                self.hz_pub.publish(self._hz_msg)
        self._last_entry_t = now_t

        # ── (1) preprocessing: snapshot state + cache params as locals ──────
        t0 = time.perf_counter()

        # OPTIMIZATION: read the state array ONCE per loop (the original copied
        # it on every cost evaluation) and hoist every value the solver needs
        # into plain locals, eliminating attribute lookups inside the hot path.
        state_snap = self.state
        x   = state_snap[0]
        y   = state_snap[1]
        yaw = state_snap[2]
        v   = state_snap[3]

        N, dt, L = self.N, self.dt, self.L
        cx, cy, R = self.cx, self.cy, self.R
        max_steer = self.max_steer
        qn, qalpha, qddelta = self.qn, self.qalpha, self.qddelta
        alat_max, v_min = self.alat_max, self.v_min

        ff_delta = self.feedforward_delta(x, y)

        # Closure handed to SLSQP: forwards into the compiled core with all
        # constants bound as locals (no `self.` access during the many evals).
        def cost(u):
            return _rollout_cost_core(
                u, x, y, yaw, v, N, dt, L, cx, cy, R,
                CIRCLE_STEER, max_steer, qn, qalpha, qddelta, alat_max, v_min,
            )

        t1 = time.perf_counter()

        # ── (2) solve ───────────────────────────────────────────────────────
        # OPTIMIZATION: warm-start — last_solution (shifted previous result) is
        # the initial guess, so SLSQP starts near the optimum every loop.
        # ftol/maxiter are kept identical to the original (ftol=1e-2 is already
        # a loose tolerance); loosening further would trade accuracy for speed.
        result = minimize(
            cost,
            self.last_solution,
            method='SLSQP',
            bounds=self._bounds,
            options={'maxiter': 50, 'ftol': 1e-2, 'disp': False},
        )

        if result.success:
            correction = float(result.x[0])
            # in-place warm-start shift (no new allocation)
            self.last_solution[:-2] = result.x[2:]
            self.last_solution[-2:] = result.x[-2:]
            self.consec_failures = 0
        else:
            self.consec_failures += 1
            correction = 0.0
            if self.consec_failures >= 3:
                self.last_solution.fill(0.0)   # in-place reset

        t2 = time.perf_counter()

        # ── (3) postprocessing + publish ────────────────────────────────────
        delta_cmd = float(min(max_steer, max(-max_steer, ff_delta + correction)))

        # Open-loop speed toward sine profile (identical to original)
        target_v   = self.sine_v_ref()
        speed_step = math.copysign(
            min(self.a_max * dt, abs(target_v - v)),
            target_v - v
        )
        speed_cmd = float(min(self.v_max, max(self.v_min, v + speed_step)))

        self.prev_delta = delta_cmd

        # Reuse the pre-allocated message object.
        self._drive_msg.header.stamp = self.get_clock().now().to_msg()
        self._drive_msg.drive.speed = speed_cmd
        self._drive_msg.drive.steering_angle = self.steer_sign * delta_cmd
        self.pub.publish(self._drive_msg)

        # Tracking errors for logging / LLM tuner (unchanged INFO line)
        dist        = math.hypot(x - cx, y - cy)
        radial_err  = dist - R
        tangent_yaw = math.atan2(y - cy, x - cx) + _HALF_PI
        heading_err = math.degrees(self.wrap_angle(yaw - tangent_yaw))

        self.get_logger().info(
            f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}° '
            f'v_ref={target_v:.2f} v={v:.2f} cmd_v={speed_cmd:.2f} '
            f'ff={ff_delta:.3f} corr={correction:.3f} delta={delta_cmd:.3f} '
            f'r_err={radial_err:.3f} h_err={heading_err:.1f}°'
        )

        t3 = time.perf_counter()

        # ── Rolling profiler: log mean phase times every 50 iters at DEBUG ──
        self._prof_pre   += (t1 - t0)
        self._prof_solve += (t2 - t1)
        self._prof_post  += (t3 - t2)
        self._prof_n     += 1
        if self._prof_n >= 50:
            n = self._prof_n
            self.get_logger().debug(
                f'[prof/50] pre={1e3*self._prof_pre/n:.3f}ms '
                f'solve={1e3*self._prof_solve/n:.3f}ms '
                f'post={1e3*self._prof_post/n:.3f}ms '
                f'total={1e3*(self._prof_pre+self._prof_solve+self._prof_post)/n:.3f}ms '
                f'(~{self._hz_ema:.1f} Hz achieved)'
            )
            self._prof_pre = self._prof_solve = self._prof_post = 0.0
            self._prof_n = 0


def main(args=None):
    rclpy.init(args=args)
    node = AndreMPCOptNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
