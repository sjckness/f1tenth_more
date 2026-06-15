"""CSV-waypoint path-tracking MPC — sibling of andre_mpc_opt_node.py.

Same MPC machinery as AndreMPCOptNode (SLSQP, horizon N=10, dt=0.1, decision
vector u=[ddelta_0, accel_0(fixed 0), ...] length 2N, warm-start, pre-allocated
arrays, numba-compiled hot cost core, on-set parameter cache, optional Jetson
CPU pinning, loop-rate telemetry). The ONLY behavioral change is the reference:
instead of an analytic circle, the car tracks a path defined by a CSV file of
(x, y) waypoints at a constant speed.

Reference generation (pure-pursuit lookahead on the waypoint array):
  * nearest-waypoint search gives the car's current index on the path
  * a feedforward steering is computed by pure pursuit toward the waypoint
    `lookahead_steps` ahead (the MPC then adds a correction term, exactly like
    the circle node adds a correction on top of its geometric feedforward)
  * the horizon reference (rx, ry, ryaw) is built by stepping one waypoint per
    MPC step from the lookahead index, wrapping with modulo indexing. The track
    is sampled at ~v_ref*dt spacing, so one-waypoint-per-step lines up.

What is intentionally NOT carried over from the circle node:
  * the sine speed profile — speed here is a constant v_ref (the sine_amp /
    sine_period parameters are still declared for interface parity but unused).

Topics:
  * sub  /odom                              (nav_msgs/Odometry, best-effort)
  * pub  <drive_topic> (default /drive)     (ackermann_msgs/AckermannDriveStamped)
  * pub  /track_mpc/loop_hz                 (std_msgs/Float32)
  * pub  /track_mpc/closest_waypoint_idx    (std_msgs/Int32)
"""

import math
import os
import time

import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Int32   # both already package deps (std_msgs)


# --- numba is optional (identical fallback to andre_mpc_opt_node) ------------
try:
    from numba import njit
    NUMBA_AVAILABLE = True
except Exception:                       # pragma: no cover - depends on host
    NUMBA_AVAILABLE = False

    def njit(*args, **kwargs):
        """No-op @njit fallback. `pip install numba` on the Jetson to enable the
        compiled fast path (large per-loop speedup)."""
        if len(args) == 1 and callable(args[0]) and not kwargs:
            return args[0]

        def _wrap(func):
            return func
        return _wrap


# ---------------------------------------------------------------------------
# Hot inner cost — compiled once (njit) and reused for every solver evaluation.
#
# Same structure as andre_mpc_opt_node._rollout_cost_core, but the per-step
# error is measured against a precomputed reference trajectory (rx, ry, ryaw)
# instead of an analytic circle. The geometric feedforward `ff` is computed once
# per loop (pure pursuit, depends only on the measured state) and held constant
# across the horizon, so it is passed in as a scalar rather than recomputed in
# the rollout. accel slots (u[2k+1]) are fixed at 0 by the bounds, exactly as in
# the circle node, so speed is held constant over the horizon.
# ---------------------------------------------------------------------------
@njit(cache=True)
def _track_cost_core(u, sx, sy, syaw, sv, N, dt, L, ff, max_steer,
                     qn, qalpha, qddelta, alat_max, v_min, rx, ry, ryaw):
    x = sx
    y = sy
    yaw = syaw
    v = sv if sv > v_min else v_min        # == max(v, v_min)

    cost = 0.0
    for k in range(N):
        ddelta = u[2 * k]                  # accel slots (2k+1) are fixed at 0

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

        # position error against the reference waypoint for this step
        ex = x - rx[k]
        ey = y - ry[k]
        pos_err2 = ex * ex + ey * ey

        heading_err = yaw - ryaw[k]
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        alat = v * v * math.tan(delta) / L
        if alat < 0.0:
            alat = -alat
        alat_violation = alat - alat_max
        if alat_violation < 0.0:
            alat_violation = 0.0

        cost += qn * pos_err2
        cost += qalpha * heading_err * heading_err
        cost += qddelta * ddelta * ddelta
        cost += 100.0 * alat_violation * alat_violation

    return cost


# Float parameters cached every loop — identical set to andre_mpc_opt_node so
# the ROS parameter interface (and any LLM tuner) sees the same control knobs.
# (qv, sine_amp, sine_period are declared for parity; this node does not use
# them — speed is a constant v_ref.)
_PARAM_NAMES = (
    'qn', 'qv', 'qalpha', 'qddelta', 'alat_max', 'a_min', 'a_max',
    'v_min', 'v_max', 'v_ref', 'sine_amp', 'sine_period', 'steer_sign',
)


class TrackMPCOptNode(Node):
    def __init__(self):
        super().__init__('track_mpc_controller')

        self.N  = 10
        self.dt = 0.1     # MPC rollout step; matches the ~v_ref*dt waypoint spacing
        self.L  = 0.25

        # ── MPC params (identical names/defaults to andre_mpc_opt_node) ────
        self.declare_parameter('qn',       50.0)   # position tracking weight
        self.declare_parameter('qv',       50.0)   # (parity only; unused here)
        self.declare_parameter('qalpha',   30.0)   # heading tracking weight
        self.declare_parameter('qddelta',   2.0)   # smooth steering corrections
        self.declare_parameter('alat_max', 10.0)   # lateral accel limit
        self.declare_parameter('a_min',    -3.0)
        self.declare_parameter('a_max',     3.0)
        self.declare_parameter('v_min',     -1.5)
        self.declare_parameter('v_max',     1.5)
        self.declare_parameter('v_ref',     1.0)   # constant target speed [m/s]
        self.declare_parameter('sine_amp',    0.0)  # parity only; unused
        self.declare_parameter('sine_period', 4.0)  # parity only; unused
        self.declare_parameter('steer_sign',  1.0)

        # ── Path-tracking params (new) ─────────────────────────────────────
        self.declare_parameter('track_file', 'tracks/default.csv')
        self.declare_parameter('lookahead_steps', 5)
        self.declare_parameter('loop_rate_hz', 20.0)

        # Publish topic — the existing node and the ackermann_mux navigation
        # input are both `drive` (see config/mux.yaml). Exposed as a parameter
        # so it can be retargeted (e.g. to `ackermann_drive`) without a rebuild.
        self.declare_parameter('drive_topic', '/drive')

        # Infra-only params (do NOT affect control). Empty/zero = disabled.
        self.declare_parameter('cpu_affinity', '')
        self.declare_parameter('nice', -5)

        self.max_steer  = 0.18
        self.max_dsteer = 0.05

        # ── Load the waypoint path (FATAL + clean shutdown if missing) ─────
        self._load_track()      # sets self._wx, self._wy, self._wp_yaw, self.M

        self.lookahead_steps = int(self.get_parameter('lookahead_steps').value)

        self.state: np.ndarray | None = None
        self.prev_delta      = 0.0
        self.last_solution   = np.zeros(2 * self.N)   # warm-start buffer (reused)
        self.consec_failures = 0

        # Pre-allocated reference horizon buffers (filled in place each loop).
        self._rx   = np.zeros(self.N)
        self._ry   = np.zeros(self.N)
        self._ryaw = np.zeros(self.N)

        # Cache float params + refresh via on-set callback (no per-loop polling).
        self._read_all_params()
        self.add_on_set_parameters_callback(self._on_set_params)

        # Bounds depend only on max_dsteer and N (both fixed) → build once.
        self._bounds = [(-self.max_dsteer, self.max_dsteer), (0.0, 0.0)] * self.N

        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, odom_qos)

        drive_topic = str(self.get_parameter('drive_topic').value)
        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.hz_pub = self.create_publisher(Float32, '/track_mpc/loop_hz', 10)
        self.idx_pub = self.create_publisher(Int32, '/track_mpc/closest_waypoint_idx', 10)

        # Pre-allocate output messages; only mutate fields each loop.
        self._drive_msg = AckermannDriveStamped()
        self._drive_msg.header.frame_id = 'base_link'
        self._hz_msg = Float32()
        self._idx_msg = Int32()

        self._apply_cpu_affinity_and_priority()
        self._warmup_solver()

        # Profiling state (perf_counter, rolling window of 50).
        self._prof_pre   = 0.0
        self._prof_solve = 0.0
        self._prof_post  = 0.0
        self._prof_n     = 0
        self._last_entry_t = None
        self._hz_ema       = 0.0

        loop_rate_hz = float(self.get_parameter('loop_rate_hz').value)
        timer_period = 1.0 / loop_rate_hz if loop_rate_hz > 0.0 else self.dt
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f'Track MPC (opt) — {self.M} waypoints, v_ref={self.v_ref:.2f} m/s, '
            f'lookahead={self.lookahead_steps} wp, loop={loop_rate_hz:.1f} Hz, '
            f'drive_topic={drive_topic} | '
            f'numba={"on" if NUMBA_AVAILABLE else "OFF (pure-python fallback)"}'
        )

    # ── Track loading ───────────────────────────────────────────────────────
    def _load_track(self):
        """Resolve track_file via the package share dir, load (x, y), close loop.

        Raises FileNotFoundError / ValueError (after logging FATAL) so main() can
        shut down cleanly without spinning a half-built node.
        """
        track_file = str(self.get_parameter('track_file').value)
        share = get_package_share_directory('mpc_controller')
        # Resolve relative to the package share dir (not the raw filesystem);
        # an absolute path is honored as-is for convenience.
        path = track_file if os.path.isabs(track_file) else os.path.join(share, track_file)

        if not os.path.isfile(path):
            self.get_logger().fatal(
                f'track_file not found: "{path}" (param track_file="{track_file}", '
                f'resolved against share dir "{share}"). Shutting down.'
            )
            raise FileNotFoundError(path)

        try:
            # comments='#' skips the header line; usecols=(0,1) takes only x, y
            # so any extra columns (yaw, kappa, s, ...) are ignored silently.
            data = np.loadtxt(path, delimiter=',', comments='#',
                              usecols=(0, 1), ndmin=2)
        except Exception as exc:
            self.get_logger().fatal(
                f'Failed to parse waypoints from "{path}": {exc}. '
                f'Expected comma-separated rows with x in column 0 and y in column 1.'
            )
            raise

        if data.shape[0] < 2:
            self.get_logger().fatal(
                f'Track "{path}" has only {data.shape[0]} waypoint(s); need >= 2.'
            )
            raise ValueError('track too short')

        wx = data[:, 0].astype(float)
        wy = data[:, 1].astype(float)

        # Close the path into a loop: append the first point if the last point
        # is farther than 0.5 m from it (otherwise it is already closed).
        gap = math.hypot(wx[-1] - wx[0], wy[-1] - wy[0])
        if gap > 0.5:
            wx = np.append(wx, wx[0])
            wy = np.append(wy, wy[0])
            self.get_logger().info(
                f'Closed track loop (end-to-start gap was {gap:.3f} m > 0.5 m).'
            )

        # Per-waypoint heading = direction to the next waypoint (wraps at end).
        nx = np.roll(wx, -1)
        ny = np.roll(wy, -1)
        wp_yaw = np.arctan2(ny - wy, nx - wx)

        self._wx = np.ascontiguousarray(wx)
        self._wy = np.ascontiguousarray(wy)
        self._wp_yaw = np.ascontiguousarray(wp_yaw)
        self.M = int(self._wx.shape[0])

        self.get_logger().info(f'Loaded {self.M} waypoints from "{path}".')

    # ── Parameter handling ─────────────────────────────────────────────────
    def _read_all_params(self):
        for name in _PARAM_NAMES:
            setattr(self, name, float(self.get_parameter(name).value))

    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in _PARAM_NAMES:
                setattr(self, p.name, float(p.value))
            elif p.name == 'lookahead_steps':
                self.lookahead_steps = int(p.value)
        return SetParametersResult(successful=True)

    # ── Jetson process tuning (identical to andre_mpc_opt_node) ─────────────
    def _apply_cpu_affinity_and_priority(self):
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
            _track_cost_core(
                u0, 0.0, 0.0, 0.0, abs(self.v_ref),
                self.N, self.dt, self.L, 0.0, self.max_steer,
                self.qn, self.qalpha, self.qddelta, self.alat_max, self.v_min,
                self._rx, self._ry, self._ryaw,
            )
        except Exception as exc:
            self.get_logger().warn(f'Solver warmup skipped: {exc}')

    # ── Helpers ──────────────────────────────────────────────────────────────
    def yaw_from_quat(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def _nearest_index(self, x, y):
        """Index of the closest waypoint to (x, y)."""
        dx = self._wx - x
        dy = self._wy - y
        return int(np.argmin(dx * dx + dy * dy))

    def _pure_pursuit_ff(self, x, y, yaw, tx, ty):
        """Geometric feedforward steering toward target (tx, ty)."""
        dx = tx - x
        dy = ty - y
        Ld = math.hypot(dx, dy)
        if Ld < 1e-3:
            return 0.0
        alpha = self.wrap_angle(math.atan2(dy, dx) - yaw)
        ff = math.atan2(2.0 * self.L * math.sin(alpha), Ld)
        return float(min(self.max_steer, max(-self.max_steer, ff)))

    # ── Odom callback (cheap: just stores state) ────────────────────────────
    def odom_callback(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)
        v   = msg.twist.twist.linear.x
        self.state = np.array([x, y, yaw, abs(v), self.prev_delta], dtype=float)

    # ── Control loop ─────────────────────────────────────────────────────────
    def control_loop(self):
        if self.state is None:
            return

        # Achieved loop-rate telemetry (wall-clock between active iterations).
        now_t = time.perf_counter()
        if self._last_entry_t is not None:
            dt_wall = now_t - self._last_entry_t
            if dt_wall > 0.0:
                inst_hz = 1.0 / dt_wall
                self._hz_ema = (0.2 * inst_hz + 0.8 * self._hz_ema
                                if self._hz_ema > 0.0 else inst_hz)
                self._hz_msg.data = float(self._hz_ema)
                self.hz_pub.publish(self._hz_msg)
        self._last_entry_t = now_t

        # ── (1) preprocessing: snapshot state + build the reference horizon ──
        t0 = time.perf_counter()

        state_snap = self.state
        x   = state_snap[0]
        y   = state_snap[1]
        yaw = state_snap[2]
        v   = state_snap[3]

        N, dt, L = self.N, self.dt, self.L
        max_steer = self.max_steer
        qn, qalpha, qddelta = self.qn, self.qalpha, self.qddelta
        alat_max, v_min = self.alat_max, self.v_min
        M = self.M

        # nearest waypoint -> lookahead index -> horizon reference (wrap modulo)
        closest = self._nearest_index(x, y)
        li = (closest + self.lookahead_steps) % M
        for k in range(N):
            idx = (li + k) % M
            self._rx[k]   = self._wx[idx]
            self._ry[k]   = self._wy[idx]
            self._ryaw[k] = self._wp_yaw[idx]

        # publish the closest-waypoint index for Foxglove debugging
        self._idx_msg.data = closest
        self.idx_pub.publish(self._idx_msg)

        # feedforward steering: pure pursuit toward the lookahead waypoint
        ff_delta = self._pure_pursuit_ff(x, y, yaw, self._wx[li], self._wy[li])

        rx, ry, ryaw = self._rx, self._ry, self._ryaw

        def cost(u):
            return _track_cost_core(
                u, x, y, yaw, v, N, dt, L, ff_delta, max_steer,
                qn, qalpha, qddelta, alat_max, v_min, rx, ry, ryaw,
            )

        t1 = time.perf_counter()

        # ── (2) solve (warm-started SLSQP, same options as the circle node) ──
        result = minimize(
            cost,
            self.last_solution,
            method='SLSQP',
            bounds=self._bounds,
            options={'maxiter': 50, 'ftol': 1e-2, 'disp': False},
        )

        if result.success:
            correction = float(result.x[0])
            self.last_solution[:-2] = result.x[2:]
            self.last_solution[-2:] = result.x[-2:]
            self.consec_failures = 0
        else:
            self.consec_failures += 1
            correction = 0.0
            if self.consec_failures >= 3:
                self.last_solution.fill(0.0)

        t2 = time.perf_counter()

        # ── (3) postprocessing + publish ────────────────────────────────────
        delta_cmd = float(min(max_steer, max(-max_steer, ff_delta + correction)))

        # Constant target speed (no sine profile), ramped by a_max.
        target_v   = self.v_ref
        speed_step = math.copysign(
            min(self.a_max * dt, abs(target_v - v)),
            target_v - v
        )
        speed_cmd = float(min(self.v_max, max(self.v_min, v + speed_step)))

        self.prev_delta = delta_cmd

        self._drive_msg.header.stamp = self.get_clock().now().to_msg()
        self._drive_msg.drive.speed = speed_cmd
        self._drive_msg.drive.steering_angle = self.steer_sign * delta_cmd
        self.pub.publish(self._drive_msg)

        # Tracking errors for logging / tuning.
        pos_err     = math.hypot(x - self._wx[closest], y - self._wy[closest])
        heading_err = math.degrees(self.wrap_angle(yaw - self._wp_yaw[closest]))

        self.get_logger().info(
            f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}° '
            f'wp={closest}/{M} v_ref={target_v:.2f} v={v:.2f} cmd_v={speed_cmd:.2f} '
            f'ff={ff_delta:.3f} corr={correction:.3f} delta={delta_cmd:.3f} '
            f'pos_err={pos_err:.3f} h_err={heading_err:.1f}°'
        )

        t3 = time.perf_counter()

        # ── Rolling profiler: mean phase times every 50 iters at DEBUG ──────
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
    try:
        node = TrackMPCOptNode()
    except Exception as exc:
        # Startup failure (e.g. missing track_file) already logged at FATAL.
        rclpy.logging.get_logger('track_mpc_controller').fatal(
            f'Startup aborted: {exc}')
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
