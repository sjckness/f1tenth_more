#!/usr/bin/env python3
"""Keyboard teleop for the HIL sim: keys on the workstation -> the ackermann_mux teleop lane.

Inside the Isaac Sim process (the hook in docs/hydra_hil.md, `keyboard:=true`), keys are read
with carb.input from the Isaac window, so the workstation keyboard drives the car while the
Isaac window has focus. If carb.input has no keyboard (headless) or fails, it falls back to
evdev (/dev/input, needs the `input` group; it reads the keyboard WHATEVER window has focus).
Standalone, outside Isaac, it runs as a plain ROS node with evdev:

    python3 tools/isaac_keyboard/isaac_keyboard.py [--topic /teleop] [--max-speed 1.0]

Commands go to ackermann_mux's `joystick` lane (topic `teleop`, priority 100, timeout 0.2 s,
src/f1tenth_bringup/config/mux.yaml), which the Orin stack's mux reads over the link. The BT's
safety_stop lane (priority 200) still wins.

Keys: W/Up forward, S/Down reverse, A/Left steer left, D/Right steer right (held = full lock),
E/PageUp faster, Q/PageDown slower, Space stop (latched until every drive key is released).

Safety, enforced by KeyboardTeleopCore (unit-tested in test_isaac_keyboard.py):
- dead-man: the command is zero unless a drive key is held in an input sample no older than
  0.3 s; released keys stop the car on the next tick, and an input that stops updating stops it
  within 0.3 s. When idle it keeps publishing zero (hold_when_idle), so the teleop lane holds the
  car instead of handing it back to the autonomous lanes;
- speed is discrete: 0, or a level >= 0.4 m/s (never moving slower than 0.4 m/s);
- steering within the car's limits: +0.319 rad left, -0.309 rad right (stack_params
  mpc_steering_angle_max/min_rad, the servo envelope).

One self-contained file (stdlib + rclpy + ackermann_msgs) so the Isaac process can load it by
path; the Isaac-side imports (carb, omni) happen only in IsaacKeyboardTeleop.
"""

import argparse
import dataclasses
import sys
import time

MIN_MOVING_SPEED = 0.4      # m/s: never command a speed in (0, 0.4)

# logical keys
FWD, BACK, LEFT, RIGHT, STOP, FASTER, SLOWER = (
    'forward', 'back', 'left', 'right', 'stop', 'faster', 'slower')
DRIVE_KEYS = (FWD, BACK, LEFT, RIGHT)

CARB_KEYS = {   # carb.input.KeyboardInput member name -> logical key
    'W': FWD, 'UP': FWD, 'S': BACK, 'DOWN': BACK, 'A': LEFT, 'LEFT': LEFT,
    'D': RIGHT, 'RIGHT': RIGHT, 'SPACE': STOP, 'E': FASTER, 'PAGE_UP': FASTER,
    'Q': SLOWER, 'PAGE_DOWN': SLOWER,
}
EVDEV_KEYS = {  # linux input event codes (linux/input-event-codes.h) -> logical key
    17: FWD, 103: FWD, 31: BACK, 108: BACK, 30: LEFT, 105: LEFT, 32: RIGHT, 106: RIGHT,
    57: STOP, 18: FASTER, 104: FASTER, 16: SLOWER, 109: SLOWER,
}


@dataclasses.dataclass
class TeleopParams:
    topic: str = '/teleop'             # ackermann_mux joystick lane (mux.yaml `teleop`)
    rate_hz: float = 30.0
    deadman_s: float = 0.3
    min_speed: float = MIN_MOVING_SPEED
    max_speed: float = 1.0             # m/s, keyboard cap (car_spec max 2.0, MPC 0.5)
    speed_step: float = 0.1
    steer_left_max: float = 0.319      # rad, stack_params mpc_steering_angle_max_rad
    steer_right_max: float = 0.309     # rad, |mpc_steering_angle_min_rad|
    allow_reverse: bool = True
    hold_when_idle: bool = True        # publish zero when no key: the teleop lane holds the car
    allow_evdev: bool = True           # Isaac: fall back to evdev if carb.input has no keyboard

    @classmethod
    def from_dict(cls, d):
        names = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in (d or {}).items() if k in names})

    def levels(self):
        lo = max(self.min_speed, MIN_MOVING_SPEED)
        hi = max(lo, self.max_speed)
        n = int(round((hi - lo) / self.speed_step)) if self.speed_step > 0 else 0
        return [round(min(hi, lo + i * self.speed_step), 3) for i in range(n + 1)]


class KeyboardTeleopCore:
    """Pure state machine: input samples in, (speed, steering) out. No ROS, no clock."""

    def __init__(self, params=None):
        self.p = params or TeleopParams()
        self.speeds = self.p.levels()
        self.level = 0                     # index into speeds: starts at the slowest, 0.4 m/s
        self.held = frozenset()
        self.last_sample = None            # time of the newest input sample
        self.stop_latched = False

    def sample(self, now, held, pressed=()):
        """One input sample: the logical keys held now, and those newly pressed since the last."""
        held = frozenset(held)
        pressed = set(pressed) | (held - self.held)
        self.held = held
        self.last_sample = now
        if FASTER in pressed:
            self.level = min(self.level + 1, len(self.speeds) - 1)
        if SLOWER in pressed:
            self.level = max(self.level - 1, 0)
        if STOP in pressed or STOP in held:
            self.stop_latched = True
        elif self.stop_latched and not (held & set(DRIVE_KEYS)):
            self.stop_latched = False      # every drive key released: the next press drives

    def command(self, now):
        """(speed m/s, steering rad); (0, 0) unless a fresh sample holds a drive key."""
        if (self.last_sample is None or now - self.last_sample > self.p.deadman_s
                or self.stop_latched):
            return 0.0, 0.0
        h = self.held
        speed = 0.0
        if FWD in h and BACK not in h:
            speed = self.speeds[self.level]
        elif BACK in h and FWD not in h and self.p.allow_reverse:
            speed = -self.speeds[self.level]
        steer = 0.0
        if LEFT in h and RIGHT not in h:
            steer = self.p.steer_left_max
        elif RIGHT in h and LEFT not in h:
            steer = -self.p.steer_right_max
        return clamp_command(speed, steer, self.p)

    def active(self, now):
        return self.command(now) != (0.0, 0.0)

    def status(self, now):
        v, d = self.command(now)
        return (f'keyboard: level {self.speeds[self.level]:.1f} m/s, cmd {v:+.2f} m/s '
                f'{d:+.3f} rad{" STOP" if self.stop_latched else ""}')


def clamp_command(speed, steer, p):
    """Speed 0 or |v| in [max(0.4, min_speed), max_speed]; steering in the car's limits."""
    lo = max(MIN_MOVING_SPEED, p.min_speed)
    mag = abs(speed)
    if mag < 1e-9:
        speed = 0.0
    else:
        speed = (1.0 if speed > 0 else -1.0) * min(max(mag, lo), max(lo, p.max_speed))
    steer = min(max(steer, -abs(p.steer_right_max)), abs(p.steer_left_max))
    return speed, steer


# ------------------------------------------------------------------ input backends
class CarbKeyboard:
    """Keys of the Isaac/Kit app window (carb.input): events for edges, polling for state."""

    def __init__(self):
        import carb.input
        import omni.appwindow
        self.ci = carb.input
        self.input = carb.input.acquire_input_interface()
        window = omni.appwindow.get_default_app_window()
        self.keyboard = window.get_keyboard() if window else None
        if self.keyboard is None:
            raise RuntimeError('no app-window keyboard (headless Isaac?)')
        self.codes = {getattr(carb.input.KeyboardInput, name): key
                      for name, key in CARB_KEYS.items()
                      if hasattr(carb.input.KeyboardInput, name)}
        self.pressed = set()
        self.sub = self.input.subscribe_to_keyboard_events(self.keyboard, self._on_event)

    def _on_event(self, event, *args):
        if event.type == self.ci.KeyboardEventType.KEY_PRESS and event.input in self.codes:
            self.pressed.add(self.codes[event.input])
        return True    # never consume: the viewport keeps its own shortcuts

    def read(self):
        held = {key for code, key in self.codes.items()
                if self.input.get_keyboard_value(self.keyboard, code) > 0.5}
        pressed, self.pressed = self.pressed, set()
        return held, pressed

    def close(self):
        try:
            self.input.unsubscribe_to_keyboard_events(self.keyboard, self.sub)
        except Exception:  # noqa: BLE001
            pass


class EvdevKeyboard:
    """Every keyboard under /dev/input, polled with the kernel's key state (no thread)."""

    def __init__(self):
        import evdev
        self.devices = []
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
                keys = dev.capabilities().get(evdev.ecodes.EV_KEY, [])
                if evdev.ecodes.KEY_SPACE in keys and evdev.ecodes.KEY_W in keys:
                    self.devices.append(dev)
            except OSError:
                continue
        if not self.devices:
            raise RuntimeError('no readable keyboard in /dev/input (user not in group input?)')

    def read(self):
        held = set()
        for dev in list(self.devices):
            try:
                held |= {EVDEV_KEYS[c] for c in dev.active_keys() if c in EVDEV_KEYS}
            except OSError:            # unplugged: drop it; no sample at all -> dead-man
                self.devices.remove(dev)
        if not self.devices:
            raise RuntimeError('keyboard gone')
        return held, set()

    def close(self):
        for dev in self.devices:
            try:
                dev.close()
            except OSError:
                pass


# ------------------------------------------------------------------ ROS publisher
class KeyboardTeleop:
    """Backend + core + AckermannDriveStamped publisher on an existing rclpy node."""

    def __init__(self, node, params=None, backend=None, log=print):
        from ackermann_msgs.msg import AckermannDriveStamped
        self.msg_type = AckermannDriveStamped
        self.node = node
        self.p = params or TeleopParams()
        self.core = KeyboardTeleopCore(self.p)
        self.backend = backend
        self.log = log
        self.pub = node.create_publisher(AckermannDriveStamped, self.p.topic, 1)
        self.period = 1.0 / self.p.rate_hz
        self.next_pub = 0.0
        self.last_status = ''

    def step(self, now=None):
        """Read keys, publish at rate_hz. Call often (every sim step / timer tick)."""
        now = time.monotonic() if now is None else now
        if self.backend is not None:
            try:
                held, pressed = self.backend.read()
                self.core.sample(now, held, pressed)
            except Exception as exc:  # noqa: BLE001 - no sample -> dead-man stops the car
                self.log(f'keyboard input failed ({exc}): commands go to zero')
                self.backend = None
        if now < self.next_pub:
            return
        self.next_pub = now + self.period
        speed, steer = self.core.command(now)
        if speed == 0.0 and steer == 0.0 and not self.p.hold_when_idle:
            return
        msg = self.msg_type()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)
        status = self.core.status(now)
        if status != self.last_status:
            self.last_status = status
            self.log(status)

    def close(self):
        if self.backend is not None:
            self.backend.close()


class IsaacKeyboardTeleop(KeyboardTeleop):
    """The Isaac-side entry point: carb.input first, evdev only if carb.input fails."""

    def __init__(self, node, params=None, log=print):
        p = params or TeleopParams()
        backend = None
        try:
            backend = CarbKeyboard()
            log('keyboard: carb.input (keys work while the Isaac window has focus)')
        except Exception as exc:  # noqa: BLE001
            log(f'keyboard: carb.input unavailable ({exc})')
            if p.allow_evdev:
                try:
                    backend = EvdevKeyboard()
                    log('keyboard: evdev FALLBACK -- reads the keyboard whatever window has '
                        'focus')
                except Exception as exc2:  # noqa: BLE001
                    log(f'keyboard: evdev unavailable too ({exc2}): no keyboard input, the '
                        'teleop lane only holds zero')
        super().__init__(node, p, backend, log)


def main(argv=None):
    """Standalone: a ROS node with the evdev backend (outside Isaac)."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--topic', default=TeleopParams.topic)
    ap.add_argument('--max-speed', type=float, default=TeleopParams.max_speed)
    ap.add_argument('--release-when-idle', action='store_true',
                    help='publish nothing when no key is held (autonomy keeps the car)')
    a, ros_args = ap.parse_known_args(argv)
    import rclpy
    rclpy.init(args=ros_args)
    node = rclpy.create_node('keyboard_teleop')
    params = TeleopParams(topic=a.topic, max_speed=a.max_speed,
                          hold_when_idle=not a.release_when_idle)
    teleop = KeyboardTeleop(node, params, EvdevKeyboard(), log=node.get_logger().info)
    node.create_timer(1.0 / (2 * params.rate_hz), teleop.step)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
