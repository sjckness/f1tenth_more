#!/usr/bin/env python3
"""LiDAR front wall: a robust line fit to the forward /scan sector, and the
usable-for-control wall distance built from it.

PUBLISHES one message on each topic for EVERY /scan -- valid or not, because
a topic that goes silent cannot be told apart from a dead node:

  /perception/lidar_front_wall          f1tenth_messages/WallLineFit
      The MEASURED wall: this scan's fit, whether it passed, and if not the
      first check it failed.
  /perception/lidar_front_wall_virtual  f1tenth_messages/WallEstimate
      The usable-for-control estimate: measured when available, dead-reckoned
      from the last accepted fit when not, seeded from /costmap/front_clearance
      when there is nothing to dead-reckon.

Nothing consumes either topic yet, deliberately, and neither is wired into
missions. The pure logic (fit, checks, state) is lidar_front_wall.py; this
file is the ROS glue and the documentation of record for both.

CONTRACT FOR /perception/lidar_front_wall_virtual
-------------------------------------------------
It always publishes. It does NOT always have a usable number: consumers must
check `valid`. Each scan the provenance is the first of these that applies:

  MEASURED       This scan's fit passed every check below. The estimate SNAPS
                 to it, never blends. When the previous scan was not MEASURED
                 the jump (measured - previous estimate) is logged: it is the
                 diagnostic for how far dead reckoning or the seed had drifted,
                 and blending would hide exactly that.
  DEAD_RECKONED  No accepted fit, the last accepted fit is at most
                 stale_age_sec old, and odometry is fresh.
  SEEDED         Nothing to dead-reckon (never measured, aged out, odometry
                 stale) and a fresh, in-range /costmap/front_clearance value.
                 This re-seeds after an age-out too: the node has no
                 move-start signal, so seeding only before the first ever
                 measurement would mean never seeding again once any wall had
                 been seen. A live seed beats nothing.
  NONE           None of the above. valid=false, distance NaN. This happens
                 at startup before anything arrives, and whenever a
                 dead-reckoned value ages out with no fresh seed to fall back
                 on.

WHAT "MEASURED" REQUIRES -- the checks, in the order a rejection reports them
-----------------------------------------------------------------------------
  no_transform         base_link <- scan frame not known yet (see LASER POSE).
  too_few_returns      Fewer valid returns in the sector than the inlier floor.
  low_inlier_count     The best line holds fewer inliers than the floor.
  low_inlier_fraction  The best line is not the dominant surface.
  oblique              Its normal is too far off the heading: the car is going
                       along this wall, not toward it.
  wrong_surface        It disagrees with the dead-reckoned wall (below).

THRESHOLDS -- measured on the run archive (~58k forward scans across the
archived missions), not guessed. stack_params.yaml carries the same values.

  sector_half_angle_deg   10.0  Normal-angle noise, 1 sigma / p99: 0.73/3.4 deg
                                at 0.4-0.8 m, 0.43/1.6 at 1.2-1.6 m, 0.16/0.7 at
                                2-3 m -- about half of +-5 deg's. +-5 deg holds on
                                those numbers; +-10 is margin for what they
                                cannot see. The base_link->laser TF is a
                                ruler-measured placeholder, and a mount-yaw error
                                is a constant bias invisible to every noise
                                figure. No archived wall_turn ever completed (max
                                rotation 50 deg), so the half-turn timing below is
                                geometric, not measured. Back to +-5 is this one
                                parameter.
  inlier_distance_m       0.03  Clean-wall acceptance is flat across 2/3/5 cm
                                (96.6/96.7/96.9%). 3 cm is ~2.7x the p99 inlier
                                residual (11.3 mm; median 2.8 mm).
  min_inlier_fraction     0.60  Of VALID returns. Majority rule with margin: the
                                accepted line must be the dominant surface, and a
                                50/50 wall/object split is rejected rather than
                                coin-flipped.
  min_inlier_count_ratio  0.50  Floor = ceil(0.5 x sector beams): 41 at +-10 deg,
                                21 at +-5 deg. Catches what the fraction cannot:
                                most beams returning nothing (glass, black
                                surfaces) while the few that do are 100% on-line.
  oblique_max_deg         45.0  Past 45 deg the car moves along the wall more
                                than toward it. Fit quality holds to 50 deg
                                (median fraction 1.00, 2.9 mm residual) and
                                collapses at 50-60 deg (0.56, 8.6 mm).
  max_hypothesis_pairs    800   81 beams make 3240 pairs: 5.0 ms per scan, 20% of
                                a core at 40 Hz. 800 is 1.3 ms, 5%.
  wrong_surface_gate_m    0.30  See WHY THE WRONG-SURFACE GATE EXISTS.
  stale_age_sec           3.0   The dead-reckoned half of a wall_turn: at the
                                0.278 rad steering limit R = 1.07 m, and 45->90
                                deg is 0.84 m of arc, 2.1 s at 0.4 m/s and 2.8 s
                                at 0.3 m/s. Also covers 97% of recorded fit
                                losses in object missions.
  odom_max_age_sec        0.2   /odometry/filtered runs at 50 Hz; its longest
                                recorded gap is 136 ms (p99.9 120 ms).
  seed_max_age_sec        0.5   /costmap/front_clearance runs at 20 Hz; its
                                longest recorded gap is 0.19 s.
  seed_max_valid_m        5.0   costmap_boundary_node's max_range_m. It publishes
                                max_range_m + 1.0 for "nothing within range",
                                which is not a distance.

WHY THE WRONG-SURFACE GATE EXISTS -- DO NOT REMOVE IT AS OVER-ENGINEERING
-------------------------------------------------------------------------
The single-scan checks cannot tell a wall from any other flat surface: when an
object stands in the sector, the fit lands on the object. On the archived
runs, 28% of fit losses lasting 0.3-3 s came back on a surface more than 2 m
from where dead reckoning put the wall. Snapping to those reports the object
as the wall. Without this gate, snap-on-reacquisition is actively unsafe.

A fit further than wrong_surface_gate_m from the dead-reckoned prediction is
rejected (wrong_surface) and dead reckoning carries on. 0.30 m sits in the
valley of the recorded innovation histogram; same-wall reacquisitions fell at
-16/+6 cm (p10/p90). The gate is tested only against a prediction from a real
measurement, never against a seed (a different quantity), and it lapses when
that prediction ages out. After that the next passing fit is accepted as a
fresh acquisition.

DEAD RECKONING
--------------
d = d0 - (p - p0) . n, where n is the wall normal captured in the odometry
frame at the last accepted fit (its normal_angle plus the odometry yaw). That
is the closing rate ds * cos(theta) integrated in closed form. It is NOT
d0 - distance_travelled. Path length is only right when driving square at the
wall; during a wall_turn it over-counts the closing and fires the turn early.
Over the 45->90 deg half-turn at R = 1.07 m that error is 0.53 m. It would
look like a tuning problem and be a geometry error.

Odometry is /odometry/filtered, the local EKF, chosen against the LiDAR on the
archived runs:

  /odom               Yaw integrated from the COMMANDED servo, no IMU. Its yaw
                      gain against the fitted wall was 0.62-0.92.
  /ekf_global         Right distance scale, but only because SLAM steps it back
                      ~1.9 times a second, and dead reckoning integrates those
                      steps as motion: 157 steps over 20 cm per hour, a ~12%
                      chance of one inside a 3 s window, worst seen 3.9 m and
                      44 deg. Disqualified.
  /slam/pose          Median 1.5 Hz with gaps to 9 s, slower than the gaps it
                      would have to bridge.
  /odometry/filtered  Smooth, gyro-fused yaw (gain 0.93-1.09). Chosen.

KNOWN BIAS -- present in /odometry/filtered, adopted anyway, NOT compensated
here. Its distance is short: the LiDAR measured 1.20x the closing the odometry
reported (IQR 1.17-1.25, 49 straight segments). Dead reckoning therefore
under-counts closing. The wall reads FARTHER than it is, and a turn keyed on
this estimate fires LATE -- about 25 cm over 3 s at 0.5 m/s. Expect logged
reacquisition jumps to lean negative. The cause is consistent with vesc.yaml's
speed_to_erpm_gain moving 4614 -> 5499.27 in commit 1099733 (ratio 1.19). That
gain also feeds distance_reached, the MPC and every other odometry distance
consumer, so it gets its own change with its own verification. Correcting for
it here would hide it, and double-correct the day it is fixed.

Odometry older than odom_max_age_sec turns off both dead reckoning and the
wrong-surface gate (there is nothing to predict with). The transition is
logged.

THE SEED
--------
/costmap/front_clearance (f1tenth_costmap's costmap_boundary_node) is the
nearest occupied SLAM-map cell in a +-35 deg forward cone. That is a DIFFERENT
quantity from a fitted wall line, and they disagree: on the archive seed - fit
was -18 cm median with an IQR of -270 to +9 cm, because the cone catches side
clutter. Expect a jump on the first real acquisition; provenance is there to
say so.

Its ABSENCE is the case to handle, not just a bad number.
costmap_boundary_node withholds the topic until it has received a map and a
pose once. After that it publishes every tick from cached inputs however old
they are (it has no recency gate), so a seed computed from a stale map looks
identical on the wire. It is a bare Float32 with no header, so its age here is
receipt time.

BLIND SPOTS -- known gaps, not oversights
-----------------------------------------
  * Glass. The LiDAR does not see it.
  * A flat object face square to the heading that appears within
    wrong_surface_gate_m of the dead-reckoned wall. It passes every check and
    is reported as the wall.
  * A flat object face that appears while there is no prediction to test it
    against: at startup, after a dead-reckoned value ages out, while seeded,
    or while odometry is stale. The first passing fit is accepted.

The intended later mitigation for the last two is a veto from YOLO detections
that fall inside the sector. It is not built.

NO COUPLING TO THE E-STOP
-------------------------
f1tenth_behavior's IsProximityTooClose reads raw /scan in its own process, and
must keep working however this node behaves. This node shares no code, no
parameter and no timing with it. It subscribes with best-effort sensor QoS,
which is compatible with urg_node's reliable publisher, sends it no
acknowledgements, and cannot back-pressure it. It runs as its own supervisor
component, never inside lidar.launch.py or the `perception` component; see
launch/lidar_front_wall.launch.py for why that placement is load-bearing.

LASER POSE
----------
base_link <- scan frame comes from /tf_static, fed into a tf2 Buffer this node
fills itself. It does not subscribe to /tf: a TransformListener would, and
would deserialize every dynamic transform in Python to read one static edge.
It is also not a hardcoded offset, because the current TF is a ruler-measured
placeholder and a recalibrated mount yaw must reach the fit. Until the
transform arrives every scan publishes no_transform.
"""

import math

import rclpy
from f1tenth_messages.msg import WallEstimate, WallLineFit
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from f1tenth_perception.cpu_affinity import apply_nice, declare_nice_param
from f1tenth_perception.lidar_front_wall import (
    PROVENANCE_NAMES,
    REASON_NO_TRANSFORM,
    REASON_OK,
    SectorFit,
    WallEstimator,
    classify_fit,
    fit_forward_line,
)

# Same profile tf2_ros.TransformListener uses for /tf_static: the static
# broadcasters latch, so a late subscriber still receives every edge.
_TF_STATIC_QOS = QoSProfile(
    depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LidarFrontWallNode(Node):

    def __init__(self, **kwargs):
        super().__init__('lidar_front_wall_node', **kwargs)

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.scan_topic = str(param('scan_topic', '/scan'))
        self.odom_topic = str(param('odom_topic', '/odometry/filtered'))
        self.seed_topic = str(param('seed_topic', '/costmap/front_clearance'))
        self.fit_topic = str(param('fit_topic', '/perception/lidar_front_wall'))
        self.estimate_topic = str(param('estimate_topic', '/perception/lidar_front_wall_virtual'))
        self.base_frame = str(param('base_frame', 'base_link'))

        self.sector_half_angle_rad = math.radians(float(param('sector_half_angle_deg', 10.0)))
        self.inlier_distance_m = float(param('inlier_distance_m', 0.03))
        self.min_inlier_fraction = float(param('min_inlier_fraction', 0.6))
        self.min_inlier_count_ratio = float(param('min_inlier_count_ratio', 0.5))
        self.oblique_max_rad = math.radians(float(param('oblique_max_deg', 45.0)))
        self.max_hypothesis_pairs = int(param('max_hypothesis_pairs', 800))
        self.estimator = WallEstimator(
            stale_age_sec=float(param('stale_age_sec', 3.0)),
            wrong_surface_gate_m=float(param('wrong_surface_gate_m', 0.3)),
            odom_max_age_sec=float(param('odom_max_age_sec', 0.2)),
            seed_max_age_sec=float(param('seed_max_age_sec', 0.5)),
            seed_max_valid_m=float(param('seed_max_valid_m', 5.0)),
        )

        declare_nice_param(self)
        apply_nice(self)

        self._tf_buffer = Buffer()
        self._laser_pose = None  # (scan frame_id, (x, y, yaw) in base_frame)
        self._odometry_was_fresh = None

        self.fit_pub = self.create_publisher(WallLineFit, self.fit_topic, 10)
        self.estimate_pub = self.create_publisher(WallEstimate, self.estimate_topic, 10)
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_cb, _TF_STATIC_QOS)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.seed_sub = self.create_subscription(
            Float32, self.seed_topic, self._seed_cb, 10)
        # Best-effort on purpose -- see the module docstring's NO COUPLING TO
        # THE E-STOP section.
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'lidar_front_wall_node up: {self.scan_topic} -> {self.fit_topic}, '
            f'{self.estimate_topic}; sector +-{math.degrees(self.sector_half_angle_rad):.1f} deg, '
            f'odometry {self.odom_topic}, seed {self.seed_topic}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tf_static_cb(self, msg):
        for transform in msg.transforms:
            self._tf_buffer.set_transform_static(transform, 'lidar_front_wall_node')

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        self.estimator.update_odometry(
            self._now(), pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation))

    def _seed_cb(self, msg):
        self.estimator.update_seed(self._now(), msg.data)

    def _laser_pose_for(self, frame_id):
        if self._laser_pose is not None and self._laser_pose[0] == frame_id:
            return self._laser_pose[1]
        try:
            stamped = self._tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except TransformException:
            return None
        t = stamped.transform
        pose = (t.translation.x, t.translation.y, _yaw_from_quaternion(t.rotation))
        self._laser_pose = (frame_id, pose)
        self.get_logger().info(
            f'{self.base_frame} <- {frame_id}: x={pose[0]:.3f} m y={pose[1]:.3f} m '
            f'yaw={math.degrees(pose[2]):.2f} deg')
        return pose

    def _scan_cb(self, msg):
        now = self._now()
        laser_pose = self._laser_pose_for(msg.header.frame_id)
        if laser_pose is None:
            fit = SectorFit(0, 0)
            reason = REASON_NO_TRANSFORM
            self.get_logger().warn(
                f'no {self.base_frame} <- {msg.header.frame_id} transform on /tf_static yet; '
                'publishing no_transform', throttle_duration_sec=5.0)
        else:
            fit = fit_forward_line(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                laser_pose, self.sector_half_angle_rad, self.inlier_distance_m,
                self.max_hypothesis_pairs)
            reason = classify_fit(
                fit, self.min_inlier_fraction, self.min_inlier_count_ratio, self.oblique_max_rad)
        step = self.estimator.step(now, fit, reason)
        self._publish(msg.header.stamp, fit, step)
        self._log_step(step)

    def _publish(self, stamp, fit, step):
        fit_msg = WallLineFit()
        fit_msg.header.stamp = stamp
        fit_msg.header.frame_id = self.base_frame
        fit_msg.valid = step.reason == REASON_OK
        fit_msg.reason = step.reason
        fit_msg.distance = float(fit.distance)
        fit_msg.normal_angle = float(fit.normal_angle)
        fit_msg.inlier_count = fit.inlier_count
        fit_msg.valid_returns = fit.valid_returns
        fit_msg.sector_beams = fit.sector_beams
        fit_msg.inlier_fraction = float(fit.inlier_fraction) if fit.has_line else math.nan
        fit_msg.rms_residual = float(fit.rms_residual)
        fit_msg.predicted_distance = float(step.predicted_distance)
        fit_msg.innovation = float(step.innovation)
        self.fit_pub.publish(fit_msg)

        estimate = step.estimate
        estimate_msg = WallEstimate()
        estimate_msg.header.stamp = stamp
        estimate_msg.header.frame_id = self.base_frame
        estimate_msg.valid = estimate.valid
        estimate_msg.provenance = estimate.provenance
        estimate_msg.distance = float(estimate.distance)
        estimate_msg.normal_angle = float(estimate.normal_angle)
        estimate_msg.source_age = float(estimate.source_age)
        self.estimate_pub.publish(estimate_msg)

    def _log_step(self, step):
        acquired = step.reacquisition
        if acquired is not None:
            self.get_logger().info(
                f'wall acquired at {step.estimate.distance:.3f} m after '
                f'{PROVENANCE_NAMES[acquired.previous_provenance]} '
                f'({acquired.previous_distance:.3f} m): jump {acquired.jump:+.3f} m, '
                f'{acquired.since_last_measurement:.2f} s since the last accepted fit')
        if step.odometry_fresh != self._odometry_was_fresh:
            if step.odometry_fresh:
                self.get_logger().info(f'{self.odom_topic} fresh: dead reckoning available')
            else:
                self.get_logger().warn(
                    f'{self.odom_topic} missing or stale: dead reckoning and the '
                    'wrong-surface gate are OFF until it returns')
            self._odometry_was_fresh = step.odometry_fresh


def main(args=None):
    rclpy.init(args=args)
    node = LidarFrontWallNode()
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
