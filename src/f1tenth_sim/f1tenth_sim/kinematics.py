"""Pure drive-command math for drive_bridge (no ROS imports, unit-testable).

The real car turns an AckermannDrive (speed, steering_angle) into ERPM and a
servo position (vesc_ackermann/src/ackermann_to_vesc.cpp), and vesc_driver
clamps the servo to [servo_min, servo_max]. The sim's ackermann_steering_
controller instead takes a body twist (v, omega). This module does that
conversion and reproduces the servo clamp as a steering-angle clamp.
"""
import math

# Real servo envelope expressed as steering angle, derived from
# f1tenth_hardware/config/steering_calibration.yaml:
#   servo = gain * delta + offset     (ackermann_to_vesc.cpp)
#   servo clamped to [servo_min, servo_max]   (vesc_driver.cpp servo_limit_)
# with servo_min 0.15, servo_max 0.8318, offset 0.4874, gain -1.2135 (both
# sides). The gain is negative, so servo_min bounds the LEFT (+) angle:
#   delta_max = (0.15   - 0.4874) / -1.2135 = +0.2780 rad
#   delta_min = (0.8318 - 0.4874) / -1.2135 = -0.2838 rad
STEERING_MIN_RAD = -0.2838
STEERING_MAX_RAD = 0.2780


def servo_envelope(servo_min, servo_max, offset, gain_left, gain_right):
    """Return (delta_min, delta_max) in rad for a servo calibration.

    Same arithmetic as the module constants, for when the calibration changes.
    Assumes negative gains (left = +delta = lower servo value), as on the car.
    """
    delta_max = (servo_min - offset) / gain_left
    delta_min = (servo_max - offset) / gain_right
    return delta_min, delta_max


def clamp_steering(delta, delta_min=STEERING_MIN_RAD, delta_max=STEERING_MAX_RAD):
    return min(max(delta, delta_min), delta_max)


def ackermann_to_twist(speed, steering_angle, wheelbase,
                       delta_min=STEERING_MIN_RAD, delta_max=STEERING_MAX_RAD):
    """Bicycle model: (v, delta) -> (v, omega), with delta clamped first.

    The controller inverts omega back to delta = atan(omega * L / v), so the
    round trip is exact for v != 0. At v == 0 omega is 0 and the steering
    angle is lost: the sim cannot pre-steer at standstill (the real servo can).
    """
    delta = clamp_steering(steering_angle, delta_min, delta_max)
    return speed, speed * math.tan(delta) / wheelbase
