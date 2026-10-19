#!/usr/bin/env python3
"""/clock for tools/replay_raw.launch.py: resumes a paused bag on the ZED's timeline.

    replay_clock_sync.py BAG_START_NS

Why not the player's --clock directly: the ZED wrapper with use_sim_time busy-
waits in its grab thread until /clock moves, and in 4 of 5 trial runs it
waited forever although the bag's /clock was on the graph with matching QoS.
Here /clock is already moving before the wrapper gets there (a pre-roll 30 s
before the bag's first stamp), so the wrapper never waits.

Why not resume the bag on the first ZED frame: in SVO real-time playback the
first frames come out early and the wrapper then settles about 180 ms AHEAD of
a clock started on frame one (measured: every image reached the recorder
~180 ms of sim time before its own stamp). So the launch starts the SVO
LEAD_S before the bag's first stamp, this node measures the settled offset
between wall time and image stamps (median over frames SKIP..SKIP+N), follows
that timeline with the pre-roll clock, and resumes the player (started with
--start-paused, its clock remapped to /clock_bag) at the wall instant the
ZED timeline reaches the bag's first stamp. From then on the bag's clock is
relayed. An image then reaches its subscribers at a sim time equal to its
stamp plus the wrapper's publish latency.

The relayed /clock is published reliable, which best-effort and reliable
subscribers both accept.
"""

import statistics
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import Resume
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo

PREROLL_S = 30.0
SKIP, N = 10, 15            # frames ignored while playback settles, frames averaged


def main():
    bag0 = int(sys.argv[1]) * 1e-9
    rclpy.init()
    node = rclpy.create_node('replay_clock_sync')
    log = node.get_logger()
    best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(Clock, '/clock', 10)
    resume = node.create_client(Resume, '/rosbag2_player/resume')
    wall0 = time.monotonic()
    state = {'offsets': [], 'offset': None, 'resumed': False}

    def publish(t):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(int(t * 1e9), 1000000000)
        pub.publish(msg)

    def tick():
        if state['resumed']:
            return
        now = time.monotonic()
        if state['offset'] is None:
            publish(bag0 - PREROLL_S + now - wall0)
            return
        publish(now - state['offset'])
        if now - state['offset'] >= bag0:
            state['resumed'] = True
            if not resume.service_is_ready():
                log.error('/rosbag2_player/resume is not up')
            resume.call_async(Resume.Request())
            log.info(f'bag resumed at ZED time {(now - state["offset"] - bag0) * 1e3:+.1f} ms '
                     f'from its first stamp (spread of the offset estimate '
                     f'{state["spread"] * 1e3:.1f} ms)')

    def bag_clock(msg):
        if state['resumed']:
            pub.publish(msg)

    def frame(msg):
        if state['offset'] is not None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        state['offsets'].append(time.monotonic() - stamp)
        if len(state['offsets']) == SKIP + N:
            window = state['offsets'][SKIP:]
            state['offset'] = statistics.median(window)
            state['spread'] = max(window) - min(window)
            if stamp > bag0:
                log.warn(f'ZED already {(stamp - bag0) * 1e3:.0f} ms past the bag start: '
                         'raise the launch lead')

    node.create_timer(0.001, tick)
    node.create_subscription(Clock, '/clock_bag', bag_clock, best_effort)
    node.create_subscription(CameraInfo, '/camera/camera_info', frame, best_effort)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
